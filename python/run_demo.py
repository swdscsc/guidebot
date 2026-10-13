#!/usr/bin/env python
"""终端版实时演示：直接在命令行里"看"Glide 式的被动引导如何工作。

    python run_demo.py                # 默认 mixed 场景
    python run_demo.py --scene crowd
    python run_demo.py --no-brake     # 关掉制动距离不等式，看它会怎么撞上去
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from guidebot.controller import ControlConfig
from guidebot.planner import PlannerConfig
from guidebot.sensors import SensorConfig
from guidebot.sim import HumanConfig, SimConfig, run_episode
from guidebot.world import make_scenario


def render(world, x, y, psi, v, cue, extra: str, width: int = 62) -> str:
    half = world.half_width
    rows = []
    for i in range(7):
        yy = half - (2 * half) * i / 6.0
        row = [" "] * width
        for o in world.obstacles:
            if abs(o.y - yy) > o.r + 0.25:
                continue
            cx = int((o.x - x + 4.0) / 12.0 * width)
            if 0 <= cx < width:
                row[cx] = "O" if o.kind == "static" else (
                    "P" if o.kind == "pedestrian" else "X")
        for d in world.drops:
            if abs(d.y - yy) > d.r + 0.25:
                continue
            cx = int((d.x - x + 4.0) / 12.0 * width)
            if 0 <= cx < width:
                row[cx] = "_"
        cy = int((half - yy) / (2 * half) * 6)
        if cy == i:
            pass
        rows.append("".join(row))
    ry = int((half - y) / (2 * half) * 6)
    rx = int(4.0 / 12.0 * width)
    if 0 <= ry < 7 and 0 <= rx < width:
        ch = "@" if psi > 0.15 else ("@" if psi < -0.15 else ">")
        rows[ry] = rows[ry][:rx] + ch + rows[ry][rx + 1:]
    art = "\n".join("|" + r + "|" for r in rows)
    return f"{art}\n v={v:.2f} m/s   cue={cue:<12} {extra}\n"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="mixed")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--no-brake", action="store_true")
    ap.add_argument("--no-gate", action="store_true")
    ap.add_argument("--no-high", action="store_true")
    args = ap.parse_args()

    world = make_scenario(args.scene, args.seed)
    sim = SimConfig(dt=0.05, record_every=1, seed=args.seed)
    ctrl = ControlConfig(enable_brake_rule=not args.no_brake,
                         enable_gate=not args.no_gate)
    sensor = SensorConfig(enable_high=not args.no_high)
    res = run_episode(world, args.scene, args.seed, sim, HumanConfig(),
                      sensor, PlannerConfig(), ctrl, record=True)

    print(f"scene={args.scene} seed={args.seed}  "
          f"(障碍物 @=人和设备, O=静态, P=行人, X=悬空横杆, _=坑洞)")
    for f in res.frames[::8]:
        extra = f"clear={f['clear']}m"
        if f["brake"]:
            extra += "  [BRAKE]"
        os.system("cls" if os.name == "nt" else "clear")
        print(render(world, f["x"], f["y"], f["psi"], f["v"], f["cue"], extra))
        print(f"t={f['t']:.1f}s   distance={f['x']:.1f}m")
        time.sleep(0.06)
    r = res.as_row()
    print("\n===== 回合结果 =====")
    for k, val in r.items():
        if k != "scenario":
            print(f"  {k}: {val}")
    print(f"  到达终点: {res.reached_goal}")
    if res.events:
        print("  事件:", res.events[:6])


if __name__ == "__main__":
    main()
