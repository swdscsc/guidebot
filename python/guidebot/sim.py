"""闭环仿真：把"人"也建模进来。

这是本项目与普通机器人仿真的关键区别：**前向速度由人提供**。
所以我们显式建模了一个"使用者"：他有偏好的步速，听到 slow/stop 提示后
会在 reaction_s 的反应延迟后才减速。这一层建模让我们能诚实地评估
"人机闭环"的真实性能（干预率、保守刹停率、体感平滑度）。
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from .controller import (CUE_NEUTRAL, CUE_SLOW, CUE_STOP, ControlConfig,
                         Command, supervise)
from .planner import PlannerConfig, plan_steering
from .sensors import SensorConfig, observe, perceived_obstacles
from .world import (HUMAN_HEIGHT, ROBOT_RADIUS, WHEELBASE, World, wrap_angle)


@dataclass
class HumanConfig:
    preferred_speed: float = 1.15
    slow_factor: float = 0.50
    reaction_s: float = 0.35
    accel_max: float = 1.00
    decel_max: float = 2.00
    obey_cues: bool = True        # Ablation：不听提示的用户
    speed_jitter: float = 0.08


@dataclass
class SimConfig:
    dt: float = 0.05
    max_time: float = 90.0
    record_every: int = 4
    seed: int = 0


@dataclass
class EpisodeResult:
    scenario: str
    seed: int
    distance: float
    duration: float
    reached_goal: bool
    collisions: int
    overhead_hits: int
    pedestrian_hits: int
    drop_falls: int
    near_misses: int
    stops: int
    conservative_stops: int
    lateral_rmse: float
    steering_slew_rms: float
    stuck: bool = False
    frames: List[Dict[str, Any]] = field(default_factory=list)
    events: List[Dict[str, Any]] = field(default_factory=list)

    def as_row(self) -> Dict[str, Any]:
        km = max(self.distance / 1000.0, 1e-6)
        return {
            "scenario": self.scenario,
            "seed": self.seed,
            "dist_m": round(self.distance, 1),
            "time_s": round(self.duration, 1),
            "goal": self.reached_goal,
            "avg_speed": round(self.distance / max(self.duration, 1e-6), 2),
            "col/km": round(self.collisions / km, 2),
            "overhead/km": round(self.overhead_hits / km, 2),
            "ped/km": round(self.pedestrian_hits / km, 2),
            "drop/km": round(self.drop_falls / km, 2),
            "stop/km": round(self.stops / km, 2),
            "conservative/km": round(self.conservative_stops / km, 2),
            "stuck": self.stuck,
            "lat_rmse_cm": round(self.lateral_rmse * 100, 1),
            "slew_rms_dps": round(math.degrees(self.steering_slew_rms), 1),
        }


class Human:
    """极简人类行为模型：延迟响应触觉/语音提示。"""

    def __init__(self, cfg: HumanConfig, rng: random.Random):
        self.cfg = cfg
        self.rng = rng
        self.target = cfg.preferred_speed
        self._pending: List[tuple] = []   # (t_release, target_speed)
        self.v = cfg.preferred_speed * 0.5
        self.backoff_until: Optional[float] = None

    def schedule(self, target: float, t: float, delay: Optional[float] = None) -> None:
        """延迟一段时间后改变目标步速。

        注意这个早退条件至关重要：如果每帧都重复 schedule 同一个目标，
        触发时刻会被一直往后推，形成"永久饥饿" —— 人接到指令却永远不执行，
        仿真里表现为明明没有刹车、人却停在障碍物前一动不动（v0.3 真实 bug）。
        """
        if abs(self.target - target) < 1e-6:
            return
        delay = self.cfg.reaction_s if delay is None else delay
        self._pending.append((t + delay, target))

    def hear(self, cue: str, t: float) -> None:
        """听到提示 -> 经过反应延迟后改变目标步速。

        关键点：没有 STOP/SLOW 提示时必须**主动恢复**舒适步速。
        v0.2 的 bug 就是忘了这条，导致人被叫停一次就永远停在原地不动了。
        """
        if not self.cfg.obey_cues or self.backoff_until is not None:
            return
        if cue == CUE_STOP:
            self.schedule(0.0, t)
        elif cue == CUE_SLOW:
            self.schedule(self.cfg.preferred_speed * self.cfg.slow_factor, t)
        else:
            if self.target < self.cfg.preferred_speed - 1e-6:
                self.schedule(self.cfg.preferred_speed, t)

    def start_backoff(self, t: float, duration: float = 1.2) -> None:
        """被连续叫停后，真实用户会本能地"退一步重新来"。

        没有这一条的话，仿真里会出现永久死锁：设备一直刹车 -> 人一直不动 ->
        低速可控性又不可能变好 -> 继续刹车。真人不会这样傻站着，
        所以把它建模进去（同时也顺带暴露了"该类产品必须有人工兜底"这一事实）。
        """
        self.backoff_until = t + duration
        self.target = -0.30

    def update(self, t: float, dt: float, external_brake: bool) -> None:
        if self.backoff_until is not None and t >= self.backoff_until:
            self.backoff_until = None
            self.target = self.cfg.preferred_speed
            self._pending = []

        if self.backoff_until is None:
            for rel, tgt in self._pending:
                if t >= rel:
                    self.target = tgt
            self._pending = [p for p in self._pending if t < p[0] + 3.0]

        jitter = self.rng.uniform(-self.cfg.speed_jitter, self.cfg.speed_jitter) \
            if self.cfg.speed_jitter > 0 else 0.0
        target = self.target + jitter
        accel = self.cfg.accel_max if target > self.v else -self.cfg.decel_max
        dv = accel * dt
        if abs(target - self.v) < abs(dv):
            self.v = target
        else:
            self.v += dv
        if external_brake and self.backoff_until is None:
            # 设备主动刹车：额外的摩擦制动
            self.v = max(0.0, self.v - 3.2 * dt)
        self.v = max(-0.45, min(2.0, self.v))


def run_episode(world: World, scenario: str, seed: int,
                scfg: SimConfig, hcfg: HumanConfig,
                sensor_cfg: SensorConfig, planner_cfg: PlannerConfig,
                ctrl_cfg: ControlConfig,
                record: bool = True) -> EpisodeResult:
    rng = random.Random(f"{scenario}:{seed}")
    if sensor_cfg.seed is None:
        sensor_cfg.seed = seed

    ctrl_cfg = ControlConfig(**{**ctrl_cfg.__dict__})
    ctrl_cfg.dt = scfg.dt

    human = Human(hcfg, rng)
    x, y, psi = 1.0, 0.0, 0.0
    delta = 0.0
    t = 0.0

    frames: List[Dict[str, Any]] = []
    events: List[Dict[str, Any]] = []
    lat_err_sq: List[float] = []
    slew_sq: List[float] = []
    lateral_samples = 0.0

    collisions = overhead_hits = ped_hits = drop_falls = 0
    near_misses = stops = conservative = 0
    start_dist = x
    traveled = 0.0

    latency_steps = max(0, int(round(ctrl_cfg.latency_s / scfg.dt)))
    cmd_queue: List[Command] = []
    last_delta = 0.0
    stop_cooldown = 0.0
    brake_held = 0.0
    best_x = x
    last_progress_t = 0.0
    near_miss_cooldown: Dict[int, float] = {}
    dropped_flag = set()
    stuck = False

    reached = False
    while t < scfg.max_time:
        t += scfg.dt
        step_dist = 0.0

        # --- 1) 感知（带噪声、丢帧、视野限制、层高度盲区）---
        obs = observe(x, y, psi, world, sensor_cfg, t, rng)
        seen = perceived_obstacles(x, y, psi, world, sensor_cfg,
                                   obs.angles, obs.ranges_low,
                                   obs.ranges_high, t)

        # 前向锥形净空：给规划器做低速时的"兜底视野"
        front_clear = 1e9
        for ang, lo, hi in zip(obs.angles, obs.ranges_low, obs.ranges_high):
            if abs(ang) <= 0.20:
                front_clear = min(front_clear, lo, hi)

        # --- 2) 规划（转向空间 DWA）---
        plan = plan_steering(
            x, y, psi, human.v, delta, seen,
            obs.detected_drops, world.half_width,
            max_unknown_range=sensor_cfg.max_range * 0.9
            if not planner_cfg.unknown_is_obstacle else 1e9,
            cfg=planner_cfg, front_clear=front_clear, t_now=t)

        # --- 3) 控制 + 失效安全监督（延迟由队列真实实现）---
        cmd = supervise(
            plan.delta, plan.min_clear, human.v, delta, ctrl_cfg,
            horizon=plan.horizon,
            delta_max=math.radians(planner_cfg.delta_max_deg),
            wheelbase=WHEELBASE, target_clear=planner_cfg.target_clear,
            overhead=bool(obs.overhead_hits),
            drop=obs.drop_probe_dist is not None,
            front_clear=plan.front_clear, ttc=plan.ttc_min)
        cmd_queue.append(cmd)
        if len(cmd_queue) > latency_steps + 1:
            active = cmd_queue.pop(0)
        else:
            active = cmd_queue[0]

        # --- 4) 人机闭环：人听到提示 -> 延迟反应 ---
        human.hear(active.cue, t)
        human.update(t, scfg.dt, active.brake)

        # --- 5) 运动学积分 ---
        prev_delta = delta
        delta = active.delta
        slew_sq.append(((delta - prev_delta) / scfg.dt) ** 2)

        if abs(human.v) > 1e-4:
            psi += human.v * math.tan(delta) / WHEELBASE * scfg.dt
            nx = x + human.v * math.cos(psi) * scfg.dt
            ny = y + human.v * math.sin(psi) * scfg.dt
            step_dist = math.hypot(nx - x, ny - y)
            x, y = nx, ny
            # 走廊墙壁
            limit = world.half_width - ROBOT_RADIUS
            if abs(y) > limit:
                y = math.copysign(limit, y)
                psi = 0.0
        traveled += step_dist
        lat_err_sq.append(y * y)
        lateral_samples += 1

        if x > best_x + 0.05:
            best_x = x
            last_progress_t = t

        # --- 6) 安全事件统计 ---
        if active.brake and abs(human.v) < 0.06:
            brake_held += scfg.dt
            if brake_held > 5.0:
                human.start_backoff(t)
                brake_held = 0.0
                events.append({"t": round(t, 2), "type": "recovery",
                               "kind": "user_backoff"})
        else:
            brake_held = 0.0

        if active.cue == CUE_STOP and stop_cooldown <= 0:
            stops += 1
            stop_cooldown = 1.0
            if plan.min_clear > 1.2:
                conservative += 1
        stop_cooldown -= scfg.dt

        for i, ob in enumerate(world.obstacles):
            # 真值碰撞判定必须用**当前绝对时刻**的位置，否则世界其实是静止的，
            # "运动预测"那条 ablation 会因为没有动态障碍而彻底失效（v0.6 的坑）。
            cx, cy = ob.center_at(t)
            d = math.hypot(x - cx, y - cy) - ob.r - ROBOT_RADIUS
            if d < 0.0 and getattr(ob, "_hit", False) is False and ob.is_lethal():
                setattr(ob, "_hit", True)
                collisions += 1
                if ob.kind == "overhead":
                    overhead_hits += 1
                elif ob.kind == "pedestrian":
                    ped_hits += 1
                events.append({"t": round(t, 2), "type": "collision",
                               "kind": ob.kind})
            elif 0.0 <= d < 0.20:
                key = i
                if t - near_miss_cooldown.get(key, -9.0) > 2.0:
                    near_misses += 1
                    near_miss_cooldown[key] = t

        for j, dz in enumerate(world.drops):
            if (x - dz.x) ** 2 + (y - dz.y) ** 2 <= dz.r ** 2 and j not in dropped_flag:
                dropped_flag.add(j)
                drop_falls += 1
                events.append({"t": round(t, 2), "type": "fall",
                               "kind": "dropzone"})

        if record and (len(frames) * scfg.record_every * scfg.dt) <= t:
            frames.append({
                "t": round(t, 2),
                "x": round(x, 3), "y": round(y, 3),
                "psi": round(psi, 4), "v": round(human.v, 3),
                "cue": active.cue, "brake": bool(active.brake),
                "clear": round(plan.min_clear, 2),
                "path": [[round(px, 2), round(py, 2)] for px, py in plan.path[::2]],
            })

        if x >= world.goal_x:
            reached = True
            break
        if t - last_progress_t > 12.0:
            stuck = True
            events.append({"t": round(t, 2), "type": "stuck", "kind": "no_progress"})
            break

    lat_rmse = math.sqrt(sum(lat_err_sq) / max(len(lat_err_sq), 1))
    slew_rms = math.sqrt(sum(slew_sq) / max(len(slew_sq), 1))
    return EpisodeResult(
        scenario=scenario, seed=seed, distance=max(traveled, x - start_dist),
        duration=t, reached_goal=reached, collisions=collisions,
        overhead_hits=overhead_hits, pedestrian_hits=ped_hits,
        drop_falls=drop_falls, near_misses=near_misses, stops=stops,
        conservative_stops=conservative, lateral_rmse=lat_rmse,
        steering_slew_rms=slew_rms, stuck=stuck, frames=frames, events=events)
