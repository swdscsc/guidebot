"""导出 Python 侧的仿真世界、参数与逐帧决策日志，供 MATLAB / Simulink 侧复用。

这是"模型在环（MIL）"的关键一步：
    Python（算法原型）  --scenarios.json / params.json / trace_ref.csv-->  MATLAB（MIL / 代码生成）

为什么需要它？
  1. 交叉验证：同一套控制律在两种语言下必须逐帧给出**完全一致**的决策，
     否则说明有隐藏的类型/截断差异 —— 这种差异在生成 C 代码部署到 MCU 时是致命的。
  2. 参数单一真源：MATLAB 侧不再手抄常数，全部从 params.json 读入，避免两边漂移。
  3. Simulink 模型的输入可以直接用 From Workspace 加载 trace_ref.csv。

用法：
    python export_matlab.py                     # 导出常规条件
    python export_matlab.py --stress            # 导出压力条件（延迟 300ms + 丢帧 6%）
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from guidebot.controller import ControlConfig
from guidebot.planner import PlannerConfig
from guidebot.sensors import SensorConfig
from guidebot.sim import HumanConfig, SimConfig, run_episode
from guidebot.world import SCENARIOS, WHEELBASE, ROBOT_RADIUS, make_scenario

# cue 与 reason 的枚举编码（必须与 matlab/gb_supervise.m 中的常量一致）
CUE_CODE = {"none": 0, "turn_left": 1, "turn_right": 2, "slow_down": 3,
            "stop": 4, "overhead": 5, "drop_ahead": 6}
REASON_CODE = {
    "nominal": 0,
    "brake_rule": 1,
    "overlap": 2,
    "gate": 3,
    "overhead_hazard": 4,
    "negative_obstacle": 5,
    "tight_clearance": 6,
    "steering": 7,
}


def dump_world(w) -> dict:
    return {
        "length": w.length,
        "half_width": w.half_width,
        "goal_x": w.goal_x,
        "obstacles": [
            {"x": o.x, "y": o.y, "r": o.r, "zmin": o.zmin, "zmax": o.zmax,
             "vx": o.vx, "vy": o.vy, "kind": o.kind}
            for o in w.obstacles
        ],
        "drops": [{"x": d.x, "y": d.y, "r": d.r, "depth": d.depth}
                  for d in w.drops],
    }


def collect_trace(world, scenario, seed, sim_cfg, human_cfg, sensor_cfg,
                  planner_cfg, ctrl_cfg) -> list:
    """跑一个回合，把 supervise() 的**每一次调用**的入参与出参都录下来。

    用 monkeypatch 而不是改核心代码，保证导出逻辑与生产代码解耦。
    """
    import guidebot.sim as S

    rows = []
    orig = S.supervise

    def spy(plan_delta, min_clear, v, prev_delta, cfg, horizon=1.5,
            delta_max=0.44, wheelbase=0.45, target_clear=0.65,
            overhead=False, drop=False, front_clear=1e9, ttc=1e9):
        cmd = orig(plan_delta, min_clear, v, prev_delta, cfg, horizon,
                   delta_max, wheelbase, target_clear, overhead, drop,
                   front_clear, ttc)
        rows.append({
            "plan_delta": plan_delta,
            "min_clear": min_clear,
            "v": v,
            "prev_delta": prev_delta,
            "horizon": horizon,
            "delta_max": delta_max,
            "wheelbase": wheelbase,
            "target_clear": target_clear,
            "overhead": int(bool(overhead)),
            "drop": int(bool(drop)),
            "front_clear": front_clear,
            "ttc": ttc,
            "delta_out": cmd.delta,
            "brake": int(cmd.brake),
            "cue": CUE_CODE.get(cmd.cue, 0),
            "reason": REASON_CODE.get(cmd.reason.split("(")[0], 0),
            "d_req": cmd.d_required,
        })
        return cmd

    S.supervise = spy
    try:
        res = run_episode(world, scenario, seed, sim_cfg, human_cfg,
                          sensor_cfg, planner_cfg, ctrl_cfg, record=True)
    finally:
        S.supervise = orig

    # 把位姿也对齐进去（每帧一条），便于 Simulink From Workspace 直接回放
    n = min(len(rows), len(res.frames))
    for i in range(n):
        f = res.frames[i]
        rows[i].update({"t": f["t"], "x": f["x"], "y": f["y"],
                        "psi": f["psi"]})
    for i in range(n, len(rows)):
        rows[i].update({"t": 0.0, "x": 0.0, "y": 0.0, "psi": 0.0})
    return rows[:n], res


def main() -> int:
    ap = argparse.ArgumentParser()
    # 默认写到仓库根的 matlab/data（本脚本放在 python/ 下，所以往上一层）
    ap.add_argument("--out", default="../matlab/data")
    ap.add_argument("--stress", action="store_true",
                    help="压力条件：延迟 300ms + 丢帧 6%")
    ap.add_argument("--max-time", type=float, default=75.0)
    args = ap.parse_args()

    out = os.path.abspath(args.out)
    os.makedirs(out, exist_ok=True)

    sim_cfg = SimConfig(dt=0.05, max_time=args.max_time, record_every=4)
    human_cfg = HumanConfig()
    sensor_cfg = SensorConfig(dropout_prob=0.06 if args.stress else 0.01)
    planner_cfg = PlannerConfig()
    ctrl_cfg = ControlConfig(dt=0.05,
                             latency_s=0.30 if args.stress else 0.10)

    # --- 1) 参数（单一真源）---
    params = {
        "control": {k: v for k, v in ctrl_cfg.__dict__.items()},
        "planner": {k: v for k, v in planner_cfg.__dict__.items()},
        "sensor": {k: (v if v is not None else 0)
                   for k, v in sensor_cfg.__dict__.items()},
        "human": {k: v for k, v in human_cfg.__dict__.items()},
        "const": {"WHEELBASE": WHEELBASE, "ROBOT_RADIUS": ROBOT_RADIUS},
    }
    with open(os.path.join(out, "params.json"), "w", encoding="utf-8") as f:
        json.dump(params, f, indent=2, ensure_ascii=False)

    # --- 2) 场景 ---
    worlds = {}
    for sc in SCENARIOS:
        for sd in (1, 2, 3):
            w = make_scenario(sc, sd)
            worlds[f"{sc}_{sd}"] = dump_world(w)
    with open(os.path.join(out, "scenarios.json"), "w", encoding="utf-8") as f:
        json.dump(worlds, f, indent=1, ensure_ascii=False)

    # --- 3) 逐帧决策日志（用于交叉验证 + Simulink 回放）---
    all_rows = []
    summary = []
    for sc in SCENARIOS:
        for sd in (1, 2, 3):
            w = make_scenario(sc, sd)
            rows, res = collect_trace(w, sc, sd, sim_cfg, human_cfg,
                                      SensorConfig(**sensor_cfg.__dict__),
                                      PlannerConfig(**planner_cfg.__dict__),
                                      ControlConfig(**ctrl_cfg.__dict__))
            for r in rows:
                r["scenario"] = sc
                r["seed"] = sd
            all_rows.extend(rows)
            summary.append({"scenario": sc, "seed": sd,
                            "dist_m": round(res.distance, 3),
                            "goal": bool(res.reached_goal),
                            "stuck": bool(res.stuck),
                            "collisions": res.collisions,
                            "drop_falls": res.drop_falls,
                            "stops": res.stops,
                            "frames": len(rows)})

    cols = ["scenario", "seed", "t", "x", "y", "psi", "v", "prev_delta",
            "plan_delta", "min_clear", "front_clear", "ttc", "horizon",
            "delta_max", "wheelbase", "target_clear", "overhead", "drop",
            "delta_out", "brake", "cue", "reason", "d_req"]
    with open(os.path.join(out, "trace_ref.csv"), "w", newline="",
              encoding="utf-8") as f:
        wr = csv.DictWriter(f, fieldnames=cols)
        wr.writeheader()
        for r in all_rows:
            wr.writerow({c: r.get(c, 0.0) for c in cols})

    with open(os.path.join(out, "summary_py.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=1, ensure_ascii=False)

    n_goal = sum(1 for s in summary if s["goal"])
    print(f"[export] -> {out}")
    print(f"  params.json      ({len(params['control'])} 控制参数 + ...")
    print(f"  scenarios.json   ({len(worlds)} 个场景)")
    print(f"  trace_ref.csv    ({len(all_rows)} 帧决策日志)")
    print(f"  summary_py.json  完成 {n_goal}/{len(summary)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
