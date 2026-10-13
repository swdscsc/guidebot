#!/usr/bin/env python
"""运行完整评测：Ablation 对比 + 延迟敏感性实验 + 导出 HTML 回放。

用法：
    python run_benchmark.py                # 全量
    python run_benchmark.py --quick        # 快速冒烟：每个变体 2 个场景 x 1 个种子
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass, replace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from guidebot.benchmark import (aggregate, default_variants, export_html,
                                latency_sweep, run_variant, stress_variants,
                                write_results)
from guidebot.controller import ControlConfig
from guidebot.planner import PlannerConfig
from guidebot.sensors import SensorConfig
from guidebot.sim import HumanConfig, SimConfig, run_episode
from guidebot.world import SCENARIOS, make_scenario


def asciiprint(s: str) -> None:
    """Windows 控制台可能不是 UTF-8，安全打印。"""
    enc = sys.stdout.encoding or "utf-8"
    sys.stdout.write(s.encode(enc, errors="replace").decode(enc, errors="replace"))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="快速冒烟模式")
    # 默认写到仓库根的 results/（本脚本放在 python/ 下，所以往上一层）
    ap.add_argument("--out", default="../results", help="输出目录")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    scenarios = ["corridor", "mixed"] if args.quick else SCENARIOS
    seeds = [1] if args.quick else [1, 2, 3]

    sim_cfg = SimConfig(dt=0.05, max_time=75.0, seed=0)
    base = {
        "sensor": SensorConfig(),
        "planner": PlannerConfig(),
        "control": ControlConfig(),
        "human": HumanConfig(),
    }

    asciiprint("== GuideBot benchmark ==\n")
    asciiprint(f"scenarios={scenarios} seeds={seeds}\n")

    t0 = time.time()
    variants = default_variants()
    results = []
    for v in variants:
        r = run_variant(v, seeds, scenarios, sim_cfg, base, record_seed=1)
        results.append(r)
        a = r["agg"]
        asciiprint(f"[{r['variant']:<14}] col/km={a['collisions_per_km']:<5.2f} "
                   f"overhead/km={a['overhead_per_km']:<5.2f} "
                   f"drop/km={a['drop_per_km']:<5.2f} "
                   f"stop/km={a['stops_per_km']:<6.2f} "
                   f"goal%={a['goal_rate']}\n")

    stress_results = []
    if not args.quick:
        asciiprint("\n-- stress condition (300ms latency / 6% dropout) --\n")
        for v in stress_variants():
            r = run_variant(v, seeds, scenarios, sim_cfg, base)
            stress_results.append(r)
            a = r["agg"]
            asciiprint(f"[{r['variant']:<20}] col/km={a['collisions_per_km']:<5.2f} "
                       f"goal%={a['goal_rate']:<6} stop/km={a['stops_per_km']:<6.2f}\n")

    asciiprint("\n-- latency sweep --\n")
    sweep = latency_sweep(base, sim_cfg, seeds,
                          [0.02, 0.10, 0.20, 0.35] if not args.quick else [0.05, 0.20])
    for s in sweep:
        asciiprint(f"latency={s['latency_ms']:>4}ms  col/km={s['collisions_per_km']:<5.2f} "
                   f"near_miss/km={s['near_miss_per_km']:<5.2f} "
                   f"stop/km={s['stops_per_km']}\n")

    elapsed = time.time() - t0
    md = write_results(os.path.join(args.out, "RESULTS.md"), results, sweep,
                       elapsed, stress_results if stress_results else None)
    asciiprint(f"\nwrote {os.path.join(args.out, 'RESULTS.md')}\n")

    with open(os.path.join(args.out, "metrics.json"), "w", encoding="utf-8") as f:
        json.dump({"variants": [{"variant": r["variant"], "note": r["note"],
                                 "agg": r["agg"], "rows": r["rows"]}
                                for r in results],
                   "stress": [{"variant": r["variant"], "note": r["note"],
                               "agg": r["agg"], "rows": r["rows"]}
                              for r in stress_results],
                   "latency_sweep": sweep}, f, indent=2, ensure_ascii=False)

    # 导出一个完整栈回合的可视化回放（优先用信息量最大的 mixed 场景）
    pick = scenarios.index("mixed") if "mixed" in scenarios else len(scenarios) - 1
    full = results[0]["episodes"][pick * len(seeds)]
    world = make_scenario(full.scenario, seeds[0])
    export_html(os.path.join(args.out, "replay.html"), world,
                full.scenario, full)
    asciiprint(f"wrote {os.path.join(args.out, 'replay.html')}\n")
    asciiprint(f"total {elapsed:.1f}s\n")


if __name__ == "__main__":
    main()
