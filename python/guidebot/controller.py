"""被动转向控制器 + 失效安全监督层 + 人机反馈（触觉词表）。

这里是整个项目最有技术含量的部分，三条原创设计都在这一个文件里：

 1. 制动距离不等式（安全走廊约束）
    行进方向必须始终留有足够的安全走廊（自由空间）：
        d_free >= v^2 / (2*mu*g) + v * T_latency + margin
    一旦规划器给出的最优轨迹净空不足，就不许继续 -> 直接刹停。
    （真实自动驾驶里这就是 RSS / 安全走廊的思想，这里做了行人化改写。）

 2. 低速可控性门限（Controllability Gate）
    由 psi_dot = v * tan(delta) / L 可知：v -> 0 时转向权限 -> 0。
    也就是说用户站住不动的时候，再强的 AI 也转不动它。
    正确做法不是硬撑，而是诚实：刹停 + 提示用户后退/绕行重来。

 3. 转向速率限制（体感平滑）
    舵机有转速上限，指令必须做 slew-rate 限制，否则体感会"抽搐"。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

# 触觉/语音词表：低维度、可通过蒙眼实验盲测识别
CUE_NEUTRAL = "none"
CUE_LEFT = "turn_left"
CUE_RIGHT = "turn_right"
CUE_SLOW = "slow_down"
CUE_STOP = "stop"
CUE_OVERHEAD = "overhead"
CUE_DROP = "drop_ahead"


@dataclass
class ControlConfig:
    mu: float = 0.60              # 轮胎-地面摩擦系数（保守取值）
    g: float = 9.81
    latency_s: float = 0.10       # 端到端延迟（感知+规划+通信）
    margin: float = 0.12          # 安全余量
    slew_max_dps: float = 150.0   # 舵机最大角速度 (deg/s)
    v_min: float = 0.30           # 可控性门限速度
    body_clear: float = 0.15      # 预测轨迹最小净空低于此值即视为"会撞上"
    ttc_brake_s: float = 0.90     # 只有冲突在这么近的未来才硬刹停（远的交给转向）
    # 注意区分"舒适净空 target_clear(=0.65)"与"安全净空 safety_clear(=0.25)"：
    # 拿舒适阈值去触发硬刹停会让设备在密集场景里寸步难行（v0.4 踩过的坑）。
    safety_clear: float = 0.25
    slow_threshold: float = 0.45  # 净空低于此值才提示减速（再高会一路爬行）
    enable_brake_rule: bool = True
    enable_gate: bool = True
    enable_slew_limit: bool = True
    dt: float = 0.05


@dataclass
class Command:
    delta: float          # 实际下发给舵机的转向角（已限速）
    brake: bool
    cue: str
    reason: str
    d_required: float
    min_clear: float


def required_free_distance(v: float, cfg: ControlConfig) -> float:
    """制动距离不等式：安全走廊必须满足的最小自由空间。"""
    return v * v / (2.0 * cfg.mu * cfg.g) + v * cfg.latency_s + cfg.margin


def achievable_lateral(v: float, horizon: float, delta_max: float,
                       wheelbase: float) -> float:
    """这一条航道是什么意思：在给定速度下，"最多能把人横向带开多少米"。

    恒定转角 delta 下轨迹是半径 R = L / tan(delta) 的圆弧，
    走过弧长 s = v*T 后的横向偏移量约为 s^2 / (2R) = (vT)^2 * tan(delta) / (2L)。
    注意它 **正比于 v^2** —— 速度一低，横移能力平方级衰减，这就是"可控性"。
    """
    s = abs(v) * horizon
    return (s * s) * abs(math.tan(delta_max)) / (2.0 * wheelbase)


def supervise(plan_delta: float, min_clear: float, v: float,
              prev_delta: float, cfg: ControlConfig,
              horizon: float = 1.5, delta_max: float = 0.44,
              wheelbase: float = 0.45, target_clear: float = 0.65,
              overhead: bool = False, drop: bool = False,
              front_clear: float = 1e9, ttc: float = 1e9) -> Command:
    d_req = required_free_distance(abs(v), cfg)
    # 用 max(v, v_min) 评估横移能力：v=0 时 lateral_can 必然为 0，
    # 直接比较会得出"永远走不了"的荒谬结论并死锁（v0.2 第二个真实 bug）。
    # 正确的语义是：假设用户至少会以 v_min 的行走节奏继续前进。
    lateral_can = achievable_lateral(max(abs(v), cfg.v_min), horizon,
                                     delta_max, wheelbase)
    lateral_need = max(0.0, cfg.safety_clear - min_clear)
    reason = "nominal"
    brake = False
    cue = CUE_NEUTRAL

    # --- 1) 正前方安全走廊（制动距离）约束 ---
    #     注意这里刻意只用"正前方自由距离"，而不是整条预测轨迹的最小净空：
    #     后者包含"侧身擦过一个 20cm 外的柱子"这种良性情况，用它触发硬刹停
    #     会让设备在密集场景寸步难行（v0.5 踩的坑：STOP/km 高达 600）。
    #     真正的语义是：**我还能不能在撞上正前方的东西之前停下来**。
    if cfg.enable_brake_rule and front_clear < d_req:
        brake = True
        cue = CUE_STOP
        reason = f"brake_rule(front={front_clear:.2f}<req={d_req:.2f})"
    # --- 2) 预测轨迹很快就要压到障碍 -> 必须停 ---
    #     必须用 TTC 门限，不能只看"轨迹最小净空"：
    #     预测窗口有 3 米长，一个 1.5 秒之后才会擦到的东西如果现在就刹停，
    #     人一停 -> 转向权限归零 -> 再也规划不出绕行 -> 永久死锁（v0.5 第六个坑）。
    #     语义正确的判据是：**冲突发生在多远的未来**。远的交给转向解决。
    elif cfg.enable_brake_rule and min_clear < cfg.body_clear and ttc < cfg.ttc_brake_s:
        brake = True
        cue = CUE_STOP
        reason = f"overlap(clear={min_clear:.2f},ttc={ttc:.2f}s)"
    # --- 3) 行进中的可控性门限：这个速度下横向带不开 -> 诚实叫停 ---
    elif cfg.enable_gate and lateral_need > lateral_can:
        brake = True
        cue = CUE_STOP
        reason = f"gate(need={lateral_need:.2f}>can={lateral_can:.2f},v={v:.2f})"
    else:
        # --- 信息类提示 ---
        if overhead and min_clear < 1.5:
            cue = CUE_OVERHEAD
            reason = "overhead_hazard"
        elif drop:
            cue = CUE_DROP
            reason = "negative_obstacle"
        elif min_clear < cfg.slow_threshold:
            cue = CUE_SLOW
            reason = "tight_clearance"
        elif abs(plan_delta) > math.radians(8.0):
            cue = CUE_LEFT if plan_delta > 0 else CUE_RIGHT
            reason = "steering"

    # --- 3) 转向速率限制 ---
    delta = plan_delta
    if cfg.enable_slew_limit:
        max_step = math.radians(cfg.slew_max_dps) * cfg.dt
        step = max(-max_step, min(max_step, delta - prev_delta))
        delta = prev_delta + step
    if brake:
        # 刹车时不再试图转向，保持直线更安全
        delta = prev_delta * 0.5
    return Command(delta=delta, brake=brake, cue=cue, reason=reason,
                   d_required=d_req, min_clear=min_clear)
