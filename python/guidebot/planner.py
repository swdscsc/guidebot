"""局部规划：在"转向空间"里做 DWA —— 专为无动力轮设计的采样规划器。

为什么不能直接拿一个现成的 planner 来用？
因为 Glide 这类平台的特殊之处：**前向速度 v 由人提供，不可命令**。
经典 DWA 在 (v, omega) 二维速度空间采样，而这里 v 是外部输入，
可行控制空间退化成一维 {delta}，且预测时域内的可控程度强烈依赖 v。
于是规划问题变成：

    给定人的当前速度 v，选一个转向角 delta，使未来 T(v) 秒内预测轨迹的
    "最小通行净空"最大，同时贴合目标方向、足够平滑、不贴墙。

踩坑记录（真实发生过，别笑，面试讲这个很有说服力）：
  v0.1 用过 42 度转向 + 0.22m 轴距的组合，结果模型给出的偏航角速度高达
  4.7 rad/s —— sampler 发现"原地转满一圈"能让 wrap_angle(psi) 回到 0、
  端点也回到原点附近，于是它把这当成最优解，机器人开始原地画圈，
  90 秒走了 100 米却始终没走出走廊。
  修法：(a) 用真实的;i转向几何（25 度 + 0.45m 等效轴距）；
        (b) 目标代价改用"预测轨迹端点到前方瞄准点的距离"，
            而不是航向差 —— 端距天然惩罚绕圈和倒退。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from .world import ROBOT_RADIUS, WHEELBASE, Obstacle, wrap_angle


@dataclass
class PlannerConfig:
    delta_max_deg: float = 25.0
    n_candidates: int = 21
    preview_dist: float = 3.2        # 瞄准点距离（米），不是时间
    horizon_min: float = 0.6
    horizon_max: float = 4.0
    # 为什么 v_floor 要设成接近正常人步速（0.9 m/s），而不是 0.3：
    # 前瞻按"距离"算（horizon = preview_dist / v_pred），一旦用户在全面避障前
    # 被叫停，v≈0 会让预测轨迹缩到 v_floor * horizon_max 那么长；
    # 取 0.3 时轨迹只有 1.05m —— **站在坑洞前一米时，1 米的预测距离根本规划不出
    # 绕行路线**（这是我调试时发现的第四个坑：静止状态下规划视野坍缩）。
    # 语义上应当假设"用户会按正常步速继续走"，所以取接近 preferred_speed 的值。
    v_floor: float = 0.90
    dt_sim: float = 0.15
    target_clear: float = 0.65       # 期望通行净空，低于它就线性惩罚
    w_clear: float = 0.60
    # 安全屏障权重：见 plan_steering 里 v0.5 的踩坑注释，这两个必须是硬约束
    w_collision: float = 40.0     # 轨迹与障碍重叠 -> 一票否决
    w_barrier: float = 8.00       # 越贴越贵（1/(clear+b) 形态），逼 planner 挑宽敞的路
    w_goal: float = 0.55
    w_smooth: float = 0.15
    w_heading: float = 0.20
    w_wall: float = 0.30
    use_prediction: bool = True       # 是否使用运动障碍预测（关掉会撞上横穿者）
    conflict_window: float = 3.5       # >horizon_max 即等价于全时程严格评估
    # 为什么要有窗口：再往后其实还有时间减速/绕行，把它也算进最小净空会让机器人
    # "预见远期冲突 -> 刹车 -> 更慢 -> 冲突更晚但更多" —— 这就是著名的
    # freezing robot problem（Feurtey, 2021）。窗口外只保留软性偏好。
    # 为什么不是全时程预测：把行人的恒定速度外推到 3 秒后，等于假设他们会
    # 径直穿过我的整条路线 -> 规划器判定"处处是死路"，通过率直接归零。
    # 这是社会性导航里的经典坑：预测要做，但必须只做"马上要撞上"的那一段。
    self_ignore_dist: float = 0.40   # 评估避障时跳过自身站位附近这么远的一段
    unknown_is_obstacle: bool = True  # 未知即高危


@dataclass
class Plan:
    delta: float
    cost: float
    min_clear: float
    front_clear: float
    ttc_min: float
    path: List[Tuple[float, float]]
    horizon: float
    candidates: List[Tuple[float, float]] = field(default_factory=list)


def _clearance_at(px: float, py: float, obstacles: List[Obstacle],
                  t_rel: float,
                  virtual: List[Tuple[float, float, float]]) -> float:
    best = 1e9
    for ob in obstacles:
        cx, cy = ob.center_at(t_rel)
        d = math.hypot(px - cx, py - cy) - ob.r - ROBOT_RADIUS
        if d < best:
            best = d
    for (cx, cy, r) in virtual:
        d = math.hypot(px - cx, py - cy) - r - ROBOT_RADIUS
        if d < best:
            best = d
    return best


def plan_steering(x: float, y: float, psi: float, v: float,
                  prev_delta: float, obstacles: List[Obstacle],
                  detected_drops: List[Tuple[float, float, float]],
                  half_width: float, max_unknown_range: float,
                  cfg: PlannerConfig,
                  front_clear: float = 1e9, t_now: float = 0.0) -> Plan:
    v_pred = max(v, cfg.v_floor)

    # 1) 速度自适应前瞻：始终保持大致恒定的"向前看距离"
    horizon = max(cfg.horizon_min,
                  min(cfg.horizon_max, cfg.preview_dist / v_pred))

    # 2) 感知到的负障碍 -> 虚拟障碍（没探测到的坑依旧危险，会被 metric 统计出来）
    virtual = [(dx, dy, dr) for (dx, dy, dr) in detected_drops]

    # 3) 瞄准点：走廊中心线上前方 preview_dist 处
    aim_x, aim_y = x + cfg.preview_dist, 0.0

    dmax = math.radians(cfg.delta_max_deg)
    best_plan: Optional[Plan] = None
    cand_info: List[Tuple[float, float]] = []

    for i in range(cfg.n_candidates):
        if cfg.n_candidates == 1:
            delta = 0.0
        else:
            delta = -dmax + 2.0 * dmax * i / (cfg.n_candidates - 1)

        cx, cy, cpsi = x, y, psi
        min_clear = 1e9
        ttc_min = 1e9
        wall_pen = 0.0
        path: List[Tuple[float, float]] = [(cx, cy)]
        steps = max(3, int(horizon / cfg.dt_sim))
        hit_something = False

        for k in range(steps):
            cpsi += v_pred * math.tan(delta) / WHEELBASE * cfg.dt_sim
            cx += v_pred * math.cos(cpsi) * cfg.dt_sim
            cy += v_pred * math.sin(cpsi) * cfg.dt_sim
            path.append((cx, cy))
            tau = (k + 1) * cfg.dt_sim
            # 绝对时间外推（t_now + tau）：世界状态是按绝对时间演化的，
            # 相对量外推会在"感知时刻"和"预测时刻"之间错位。
            t_rel = t_now + (min(tau, cfg.conflict_window)
                             if cfg.use_prediction else 0.0)
            if math.hypot(cx - x, cy - y) < cfg.self_ignore_dist:
                # 跳过"当前站位"这一小段：机器人半径已经把障碍膨胀过，
                # 站在离柱子 0.5m 处时首点净空天然为负 -> 所有候选都是"必撞" ->
                # 安全层永久刹车 -> 死锁（v0.5 第五个真实 bug）。
                # 自身站位是不可控状态，评估避障必须从"接下来要去哪"开始。
                continue
            cl = _clearance_at(cx, cy, obstacles, t_rel, virtual)
            if tau <= cfg.conflict_window:
                if cl < min_clear:
                    min_clear = cl
                    ttc_min = min(ttc_min, tau)
                if cl < 0.6:
                    hit_something = True
            else:
                # 窗口外：只作为软性偏好，不参与"最小净空"这种硬性指标
                min_clear = min(min_clear, max(cl, cfg.target_clear))
            over = abs(cy) - (half_width - 0.45)
            if over > 0.0:
                wall_pen = max(wall_pen, over)
            if cfg.unknown_is_obstacle and \
                    math.hypot(cx - x, cy - y) > max_unknown_range:
                min_clear = min(min_clear, 0.4)

        # 低速时预测轨迹很短，会"看不见正前方的墙" —— 用前向锥形测距兜底
        eff_clear = min(min_clear, front_clear)

        # ---- 代价函数 ----
        # 关键教训（v0.5 的第三个真实 bug）：净空代价一开始写成了线性
        #   max(0, target_clear - clear)，于是"穿过坑洞躲到侧后方"这种代价只有 ~1.5，
        #   而"提前绕开"因为端点和航向偏移要付 ~3.7 的账 —— 规划器做出两害相权
        #   取其轻的正确计算、却选择了撞上去的那条路，然后 safety layer 立刻刹车，
        #   人停下 -> 低速转向权限归零 -> 永久死锁。
        #   结论：**凡是"是否会撞"这种二元约束，代价必须是屏障函数（barrier），
        #   不能用线性罰项去和舒适性指标做 trade-off。**
        clearance_cost = cfg.w_clear * max(0.0, cfg.target_clear - eff_clear)
        collision_pen = cfg.w_collision * max(0.0, -eff_clear)
        barrier = cfg.w_barrier / (max(eff_clear, 0.01) + 0.25)
        clearance_cost += collision_pen + barrier
        end_dist = math.hypot(cx - aim_x, cy - aim_y)
        goal_cost = cfg.w_goal * end_dist
        heading_cost = cfg.w_heading * abs(wrap_angle(cpsi))
        smooth_cost = cfg.w_smooth * abs(delta - prev_delta) / max(dmax, 1e-6)
        wall_cost = cfg.w_wall * wall_pen
        total = clearance_cost + goal_cost + heading_cost + smooth_cost + wall_cost

        cand_info.append((delta, total))
        if best_plan is None or total < best_plan.cost:
            best_plan = Plan(delta=delta, cost=total, min_clear=eff_clear,
                             front_clear=front_clear, ttc_min=ttc_min,
                             path=path, horizon=horizon)

    best_plan.candidates = cand_info
    return best_plan
