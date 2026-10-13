"""评测框架：把"感觉它好像能行"变成"每一项都量得出来"。

设计原则（README 里要重点强调这一点）：
 1. 场景按种子可复现 —— 任何人 clone 下来跑 run_benchmark.py 都能得到同一张表。
 2. 每个 Ablation 只改一个开关 —— 才能把结论归因到具体设计上。
 3. 所有指标都换算成 每公里 —— 这样不同长度的场景可以横向聚合。
 4. 产出机器可读的 JSON + 人类可读的 RESULTS.md + 可视化 HTML 回放。
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, replace
from typing import Any, Dict, List, Optional

from .controller import ControlConfig
from .planner import PlannerConfig
from .sensors import SensorConfig
from .sim import EpisodeResult, HumanConfig, SimConfig, run_episode
from .world import SCENARIOS, World, make_scenario


@dataclass
class Variant:
    name: str
    note: str
    sensor: Optional[SensorConfig] = None
    planner: Optional[PlannerConfig] = None
    control: Optional[ControlConfig] = None
    human: Optional[HumanConfig] = None


def default_variants() -> List[Variant]:
    return [
        Variant("full", "完整栈：双层射线 + 下视探头 + 制动走廊 + 可控性门限"),
        Variant("no_brake_rule", "去掉制动距离不等式（只做纯反应式避障）",
                control=ControlConfig(enable_brake_rule=False)),
        Variant("no_gate", "去掉低速可控性门限",
                control=ControlConfig(enable_gate=False)),
        Variant("no_high_layer", "去掉悬空层射线（模拟传统盲杖的物理盲区）",
                sensor=SensorConfig(enable_high=False)),
        Variant("no_probe", "去掉近距下视探头（看不到坑洞/下行台阶）",
                sensor=SensorConfig(enable_probe=False)),
        Variant("no_prediction", "不使用运动预测（横穿行人来不及反应）",
                planner=PlannerConfig(use_prediction=False)),
    ]


def stress_variants() -> List[Variant]:
    """压力条件下的对照实验。

    为什么要单独跑一组压力实验？因为在感知良好、延迟很低时，单靠规划器
    本身就能躲开绝大多数障碍 —— 这时候"安全监督层"的价值根本看不出来
    （no_safety 也能拿 0 碰撞，那张表就是无效实验）。
    安全层的存在意义只有在**感知退化 + 端到端延迟变大**时才成立，
    所以把压力条件单独做成一组，Ablation 才有归因能力。
    """
    stressed_sensor = SensorConfig(dropout_prob=0.06)
    stressed_ctrl = ControlConfig(latency_s=0.30)
    no_safety_ctrl = ControlConfig(latency_s=0.30, enable_brake_rule=False,
                                   enable_gate=False)
    return [
        Variant("stress/full", "压力条件（延迟 300ms + 丢帧 6%）下的完整栈",
                sensor=stressed_sensor, control=stressed_ctrl),
        Variant("stress/no_safety", "压力条件 + 去掉安全监督层",
                sensor=stressed_sensor, control=no_safety_ctrl),
        Variant("stress/no_prediction", "压力条件 + 不做运动预测",
                sensor=stressed_sensor, control=stressed_ctrl,
                planner=PlannerConfig(use_prediction=False)),
    ]


def run_variant(variant: Variant, seeds: List[int],
                scenarios: List[str], sim_cfg: SimConfig,
                base: Dict[str, Any], record_seed: Optional[int] = None
                ) -> Dict[str, Any]:
    sensors = variant.sensor or replace(base["sensor"])
    planner = variant.planner or replace(base["planner"])
    control = variant.control or replace(base["control"])
    human = variant.human or replace(base["human"])

    rows: List[Dict[str, Any]] = []
    episodes: List[EpisodeResult] = []
    for sc in scenarios:
        for sd in seeds:
            world = make_scenario(sc, sd)
            res = run_episode(world, sc, sd, sim_cfg,
                              replace(human), replace(sensors),
                              replace(planner), replace(control),
                              record=(record_seed is not None and sd == record_seed
                                      and sc == scenarios[0]))
            rows.append(res.as_row())
            episodes.append(res)
    agg = aggregate(episodes)
    return {"variant": variant.name, "note": variant.note, "rows": rows,
            "agg": agg, "episodes": episodes}


def aggregate(eps: List[EpisodeResult]) -> Dict[str, float]:
    dist = sum(e.distance for e in eps)
    km = max(dist / 1000.0, 1e-6)
    n = max(len(eps), 1)
    return {
        "episodes": n,
        "total_km": round(dist / 1000.0, 3),
        "goal_rate": round(sum(1 for e in eps if e.reached_goal) / n, 3),
        "stuck_rate": round(sum(1 for e in eps if e.stuck) / n, 3),
        "collisions_per_km": round(sum(e.collisions for e in eps) / km, 2),
        "overhead_per_km": round(sum(e.overhead_hits for e in eps) / km, 2),
        "pedestrian_per_km": round(sum(e.pedestrian_hits for e in eps) / km, 2),
        "drop_per_km": round(sum(e.drop_falls for e in eps) / km, 2),
        "near_miss_per_km": round(sum(e.near_misses for e in eps) / km, 2),
        "stops_per_km": round(sum(e.stops for e in eps) / km, 2),
        "conservative_stops_per_km": round(
            sum(e.conservative_stops for e in eps) / km, 2),
        "lateral_rmse_cm": round(math.sqrt(
            sum(e.lateral_rmse ** 2 for e in eps) / n) * 100, 1),
        "steering_slew_rms_dps": round(math.degrees(math.sqrt(
            sum(e.steering_slew_rms ** 2 for e in eps) / n)), 1),
    }


def latency_sweep(base: Dict[str, Any], sim_cfg: SimConfig,
                  seeds: List[int], latencies: List[float]) -> List[Dict[str, Any]]:
    """端到端延迟敏感性实验 —— 这是能体现工程素养的一张表。"""
    out = []
    for lat in latencies:
        control = replace(base["control"], latency_s=lat)
        eps = []
        for sc in ["mixed"]:
            for sd in seeds:
                world = make_scenario(sc, sd)
                eps.append(run_episode(world, sc, sd, sim_cfg,
                                       replace(base["human"]),
                                       replace(base["sensor"]),
                                       replace(base["planner"]),
                                       replace(control), record=False))
        agg = aggregate(eps)
        agg["latency_ms"] = int(lat * 1000)
        out.append(agg)
    return out


def md_table(headers: List[str], rows: List[List[Any]]) -> str:
    line = "| " + " | ".join(str(h) for h in headers) + " |"
    sep = "| " + " | ".join("---" for _ in headers) + " |"
    body = "\n".join("| " + " | ".join(str(c) for c in r) + " |" for r in rows)
    return "\n".join([line, sep, body])


def write_results(path: str, variants: List[Dict[str, Any]],
                  sweep: List[Dict[str, Any]], elapsed: float,
                  stress: Optional[List[Dict[str, Any]]] = None) -> str:
    keys = ["episodes", "total_km", "goal_rate", "stuck_rate",
            "collisions_per_km", "overhead_per_km", "pedestrian_per_km", "drop_per_km",
            "near_miss_per_km", "stops_per_km", "conservative_stops_per_km",
            "lateral_rmse_cm", "steering_slew_rms_dps"]
    head = ["variant", *keys]
    rows = [[v["variant"], *[v["agg"][k] for k in keys]] for v in variants]

    sweep_keys = ["latency_ms", "collisions_per_km", "near_miss_per_km",
                  "stops_per_km", "conservative_stops_per_km",
                  "lateral_rmse_cm"]

    md = [
        "# RESULTS",
        "",
        "所有数字由 `run_benchmark.py` 自动生成，可完全复现（固定随机种子）。",
        "",
        "## 主实验：Ablation 对比",
        "",
        md_table(head, rows),
        "",
        "### 各变体说明",
        "",
    ]
    for v in variants:
        md.append(f"- **{v['variant']}**: {v['note']}")

    if stress:
        md += ["", "## 压力实验：感知退化 + 高延迟（延迟 300ms / 丢帧 6%）", "",
               "这一组才是安全监督层与运动预测真正起作用的场景：", ""]
        md += [md_table(head, [[v["variant"], *[v["agg"][k] for k in keys]]
                               for v in stress]), ""]
        for v in stress:
            md.append(f"- **{v['variant']}**: {v['note']}")

    md += ["", "## 延迟敏感性实验（mixed 场景）", "",
           md_table(sweep_keys, [[s[k] for k in sweep_keys] for s in sweep]),
           "", f"_总运行耗时 {elapsed:.1f}s，Python 标准库实现，无第三方依赖。_", ""]
    text = "\n".join(md)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    return text


# --------------------------------------------------------------------------
# 可视化：导出单文件 HTML 回放（canvas + 原生 JS，双击即可看）
# --------------------------------------------------------------------------

HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>GuideBot 回放 - __SCENARIO__</title>
<style>
body{font-family:-apple-system,"Segoe UI",sans-serif;background:#f7f7f5;margin:0;padding:16px}
.wrap{max-width:1080px;margin:0 auto}
canvas{background:#fff;border:1px solid #ddd;border-radius:8px;width:100%}
.bar{display:flex;gap:10px;align-items:center;margin:10px 0;flex-wrap:wrap}
button{padding:6px 14px;border:1px solid #bbb;background:#fff;border-radius:6px;cursor:pointer}
.tag{padding:3px 9px;border-radius:20px;font-size:12px;background:#eee}
.tag.stop{background:#fde8e8;color:#a32d2d}.tag.slow_down{background:#faeeda;color:#854f0b}
.tag.turn_left,.tag.turn_right{background:#e6f1fb;color:#185fa5}
.tag.overhead{background:#fbEAF0;color:#993556}.tag.drop_ahead{background:#e1f5ee;color:#0f6e56}
table{border-collapse:collapse;margin-top:14px;font-size:13px}
td,th{border:1px solid #ddd;padding:4px 10px}
</style></head><body><div class="wrap">
<h2>GuideBot 仿真回放 __SCENARIO__</h2>
<div class="bar">
<button onclick="toggle()">Pause</button>
<button onclick="reset()">Restart</button>
<label>速度 <input type="range" id="sp" min="1" max="8" value="3"></label>
<span>前向速度 v: <b id="vv">0.00</b> m/s</span>
<span>当前提示: <span id="cue" class="tag">none</span></span>
</div>
<canvas id="cv" width="1040" height="320"></canvas>
<div id="meta"></div>
</div><script>
const DATA = __DATA__;
const canvas=document.getElementById('cv'),ctx=canvas.getContext('2d');
let idx=0,playing=true;
const S=18.0, PAD=20;
function sx(x){return PAD+ x*S*1.05;}
function sy(y){return canvas.height/2 - y*S;}
function draw(){
  ctx.clearRect(0,0,canvas.width,canvas.height);
  const half=DATA.half_width;
  ctx.fillStyle='#faf9f6';ctx.fillRect(PAD, sy(half), canvas.width-2*PAD, sy(-half)-sy(half));
  ctx.strokeStyle='#ccc';ctx.beginPath();ctx.moveTo(PAD,sy(half));ctx.lineTo(canvas.width-PAD,sy(half));
  ctx.moveTo(PAD,sy(-half));ctx.lineTo(canvas.width-PAD,sy(-half));ctx.stroke();
  ctx.fillStyle='#ffb4b4';
  for(const d of DATA.drops){ctx.beginPath();ctx.arc(sx(d[0]),sy(d[1]),d[2]*S,0,6.283);ctx.fill();
    ctx.strokeStyle='#e24b4a';ctx.setLineDash([4,3]);ctx.stroke();ctx.setLineDash([]);}
  const f=DATA.frames[idx];
  const view_x=f.x-6;
  ctx.save();ctx.translate(-(sx(view_x)-PAD+15),0);
  for(const o of DATA.obstacles){
    const t=f.t;
    const ox=o[0]+o[3]*t, oy=o[1]+o[4]*t;
    if(ox<view_x-2||ox>view_x+34) continue;
    ctx.beginPath();ctx.arc(sx(ox),sy(oy),o[2]*S,0,6.283);
    if(o[5]==='overhead'){ctx.fillStyle='rgba(212,83,126,0.35)';ctx.strokeStyle='#993556';}
    else if(o[5]==='pedestrian'){ctx.fillStyle='rgba(55,138,221,0.35)';ctx.strokeStyle='#185fa5';}
    else{ctx.fillStyle='rgba(136,135,128,0.3)';ctx.strokeStyle='#5f5e5a';}
    ctx.fill();ctx.stroke();
  }
  if(f.path&&f.path.length>1){ctx.strokeStyle='#1d9e75';ctx.lineWidth=2;ctx.beginPath();
    ctx.moveTo(sx(f.path[0][0]),sy(f.path[0][1]));
    for(const p of f.path) ctx.lineTo(sx(p[0]),sy(p[1]));ctx.stroke();ctx.lineWidth=1;}
  ctx.beginPath();ctx.arc(sx(f.x),sy(f.y),0.22*S,0,6.283);
  ctx.fillStyle=f.brake?'#e24b4a':'#534ab7';ctx.fill();
  ctx.strokeStyle='#26215c';ctx.beginPath();ctx.moveTo(sx(f.x),sy(f.y));
  ctx.lineTo(sx(f.x+0.9*Math.cos(f.psi)),sy(f.y+0.9*Math.sin(f.psi)));ctx.stroke();
  ctx.restore();
  document.getElementById('vv').textContent=f.v.toFixed(2);
  const c=document.getElementById('cue');c.textContent=f.cue;c.className='tag '+f.cue;
  document.getElementById('meta').innerHTML='<table><tr><th>t</th><td>'+f.t.toFixed(1)+'s</td>'+
    '<th>clear</th><td>'+f.clear+'m</td><th>brake</th><td>'+f.brake+'</td></tr></table>';
}
function step(){ if(idx<DATA.frames.length-1) idx++; draw();
  if(playing) setTimeout(step, 90/ (document.getElementById('sp').value/3)); }
function toggle(){playing=!playing; if(playing) step();}
function reset(){idx=0;playing=true;step();}
draw();step();
</script></body></html>"""


def export_html(path: str, world: World, scenario: str, ep: EpisodeResult) -> None:
    payload = {
        "half_width": world.half_width,
        "drops": [[d.x, d.y, d.r] for d in world.drops],
        "obstacles": [[o.x, o.y, o.r, o.vx, o.vy, o.kind] for o in world.obstacles],
        "frames": ep.frames,
    }
    html = (HTML_TEMPLATE
            .replace("__DATA__", json.dumps(payload))
            .replace("__SCENARIO__", f"- {scenario}"))
    with open(path, "w", encoding="utf-8") as f:
        f.write(html)
