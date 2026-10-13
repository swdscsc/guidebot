"""CI 回归断言：读 run_benchmark.py 产出的 results/metrics.json，
对完整栈（full）做硬性门槛检查。失败则以非 0 退出，让 CI 变红。

这是给"别人 clone 下来 CI 自动跑"用的闸门 —— 任何让 full 栈出现碰撞、
或完成率大幅回退的改动都会立刻被发现。
"""

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
# run_benchmark.py 默认把报告写到仓库根的 results/（--out ../results）
METRICS = os.path.normpath(os.path.join(HERE, "..", "results", "metrics.json"))

# 门槛（留安全余量，避免随机种子外的抖动误报警）
# 注意：metrics.json 里的 goal_rate 是 0~1 的小数（不是百分比）
MIN_FULL_GOAL_RATE = 0.85      # README 报告 0.867
MAX_FULL_COLLISIONS = 0.01     # 完整栈必须零碰撞（每 km）
MAX_FULL_DROP_FALL = 0.01      # 悬空/踩空也必须零


def main():
    if not os.path.exists(METRICS):
        print(f"ERROR: 找不到 {METRICS}，请先运行 python run_benchmark.py", file=sys.stderr)
        return 2
    with open(METRICS, encoding="utf-8") as f:
        data = json.load(f)

    by_variant = {v["variant"]: v["agg"] for v in data["variants"]}
    ok = True

    def check(name, cond, detail):
        nonlocal ok
        mark = "PASS" if cond else "FAIL"
        if not cond:
            ok = False
        print(f"  [{mark}] {name}: {detail}")

    full = by_variant.get("full")
    if full is None:
        print("ERROR: metrics 中缺少 full 变体", file=sys.stderr)
        return 2

    check("full 零碰撞",
          full["collisions_per_km"] <= MAX_FULL_COLLISIONS,
          f"col/km = {full['collisions_per_km']:.3f} (<= {MAX_FULL_COLLISIONS})")
    check("full 完成率达标",
          full["goal_rate"] >= MIN_FULL_GOAL_RATE,
          f"goal% = {full['goal_rate']*100:.1f} (>= {MIN_FULL_GOAL_RATE*100:.0f})")

    # 关键归因：去掉悬空层射线应当出现物理盲区（碰撞/踩空高于完整栈）
    no_high = by_variant.get("no_high_layer")
    if no_high is not None and full is not None:
        check("no_high_layer 暴露盲区（碰撞 > full）",
              no_high["collisions_per_km"] > full["collisions_per_km"] + 1e-9,
              f"no_high col/km = {no_high['collisions_per_km']:.2f} vs full = {full['collisions_per_km']:.2f}")
    else:
        print("  [SKIP] no_high_layer 对照（变体缺失）")

    # 压力组存在且 full 在压力下明显优于 去安全层
    stress = {s["variant"]: s["agg"] for s in data.get("stress", [])}
    sf = stress.get("stress/full")
    sn = stress.get("stress/no_safety")
    if sf is not None and sn is not None:
        check("压力下 安全监督层仍有效（full 碰撞 < no_safety）",
              sf["collisions_per_km"] < sn["collisions_per_km"],
              f"stress/full col/km = {sf['collisions_per_km']:.2f} < stress/no_safety = {sn['collisions_per_km']:.2f}")
    else:
        print("  [SKIP] 压力组对照（变体缺失）")

    print()
    if ok:
        print(f"ALL CHECKS PASSED  (full goal%={full['goal_rate']*100:.1f}, col/km={full['collisions_per_km']:.3f})")
        return 0
    print("REGRESSION DETECTED —— CI 失败", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
