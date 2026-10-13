"""虚拟深度传感器：两层射线 + 近距下视探头 + 噪声/丢帧模型。

这一层是"感知前端"的抽象接口（对应 perception/ 目录）。
真实版本可以在这里替换成 Depth Anything V2 -> ONNX/TensorRT 的深度估计输出，
只要它们产出的也是"每个角度的最近命中距离"，下游 planner 一行都不用改。
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from .world import Obstacle, World


@dataclass
class SensorConfig:
    n_rays: int = 49
    fov_deg: float = 68.0
    max_range: float = 8.0
    h_low: float = 0.95        # 相机层射线高度
    h_high: float = 1.50       # 悬空层射线高度
    enable_high: bool = True   # Ablation 开关：关掉就看不见悬空障碍
    probe_reach: float = 1.30  # 下视探头前向探测距离
    probe_step: float = 0.22
    probe_halfwidth: float = 0.40
    enable_probe: bool = True  # Ablation 开关：关掉就看不见坑洞
    range_noise_rel: float = 0.015
    range_noise_abs: float = 0.01
    dropout_prob: float = 0.01
    seed: Optional[int] = None


@dataclass
class Observation:
    t: float
    angles: List[float]
    ranges_low: List[float]
    ranges_high: List[float]
    overhead_hits: List[float]        # 视锥内最近的悬空障碍距离
    detected_drops: List[Tuple[float, float, float]]   # (x, y, r) 世界坐标
    drop_probe_dist: Optional[float]  # 下视探头发现负障碍的前向距离


def _hit_obstacle(ox: float, oy: float, dx: float, dy: float,
                  ob: Obstacle, t_rel: float, height: float,
                  max_range: float) -> float:
    """单条射线与单个圆柱障碍求交，返回命中距离（未命中返回 max_range）。"""
    if not (ob.zmin <= height <= ob.zmax):
        return max_range
    cx, cy = ob.center_at(t_rel)
    fx, fy = ox - cx, oy - cy
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - ob.r * ob.r
    disc = b * b - 4.0 * c
    if disc < 0.0:
        return max_range
    sq = math.sqrt(disc)
    s1 = (-b - sq) / 2.0
    s2 = (-b + sq) / 2.0
    if s1 > 0.0:
        return s1
    if s2 > 0.0:
        return 0.0
    return max_range


def cast_layer(ox: float, oy: float, psi: float, obstacles: List[Obstacle],
               cfg: SensorConfig, height: float, t_rel: float,
               rng: Optional[random.Random] = None) -> List[float]:
    """投射一层扇形射线，返回每个角度的测距值。"""
    half = math.radians(cfg.fov_deg) / 2.0
    n = cfg.n_rays
    out: List[float] = []
    for i in range(n):
        if n == 1:
            th = 0.0
        else:
            th = -half + (2.0 * half) * i / (n - 1)
        ang = psi + th
        dx, dy = math.cos(ang), math.sin(ang)
        best = cfg.max_range
        for ob in obstacles:
            d = _hit_obstacle(ox, oy, dx, dy, ob, t_rel, height, cfg.max_range)
            if d < best:
                best = d
        if rng is not None:
            if rng.random() < cfg.dropout_prob:
                best = cfg.max_range          # 丢帧：当成"未知"，下游必须保守处理
            else:
                best += rng.gauss(0.0, cfg.range_noise_abs +
                                  cfg.range_noise_rel * best)
                best = max(0.0, min(cfg.max_range, best))
        out.append(best)
    return out


def probe_drops(ox: float, oy: float, psi: float, world: World,
                cfg: SensorConfig, t_rel: float = 0.0) -> Tuple[
                    List[Tuple[float, float, float]], Optional[float]]:
    """近距下视探头：只能看到脚前有限范围内的负障碍（坑/下行台阶）。

    这是整个系统里"距离最短却最要命"的通道 —— 因为探测距离只有 1.3m，
    所以速度越快，留给制动的时间越少，直接决定了控制层的制动距离约束。
    """
    if not cfg.enable_probe or not world.drops:
        return [], None
    fx, fy = math.cos(psi), math.sin(psi)
    lx, ly = -fy, fx
    found: List[Tuple[float, float, float]] = []
    nearest = None
    d = 0.2
    while d <= cfg.probe_reach:
        lat = -cfg.probe_halfwidth
        while lat <= cfg.probe_halfwidth + 1e-9:
            px = ox + fx * d + lx * lat
            py = oy + fy * d + ly * lat
            for dz in world.drops:
                if (px - dz.x) ** 2 + (py - dz.y) ** 2 <= dz.r ** 2:
                    key = (dz.x, dz.y, dz.r)
                    if key not in found:
                        found.append(key)
                    if nearest is None or d < nearest:
                        nearest = d
            lat += 0.18
        d += cfg.probe_step
    return found, nearest


def perceived_obstacles(x: float, y: float, psi: float, world: World,
                        cfg: SensorConfig, angles: List[float],
                        ranges_low: List[float],
                        ranges_high: List[float],
                        t: float = 0.0) -> List[Obstacle]:
    """把"这一帧实际能被感知到"的障碍挑出来交给规划器。

    这一步极其重要：早期版本直接把 world 里的真值障碍喂给规划器（上帝视角），
    结果关掉悬空层射线之后，机器人依然能完美避开看不见的横杆 —— 
    Ablation 完全失效，结论全是假的。任何带"我看得见 vs 看不见"主张的
    机器人评测，都必须在这一层做严格的信息隔离。
    """
    half_fov = math.radians(cfg.fov_deg) / 2.0
    out: List[Obstacle] = []
    for ob in world.obstacles:
        cx, cy = ob.center_at(t)          # 世界是按绝对时间演化的
        dx, dy = cx - x, cy - y
        dist = math.hypot(dx, dy)
        if dist > cfg.max_range + ob.r:
            continue
        rel = math.atan2(dy, dx) - psi
        while rel > math.pi:
            rel -= 2 * math.pi
        while rel < -math.pi:
            rel += 2 * math.pi
        angular_radius = math.atan2(ob.r, max(dist, 0.1))
        if abs(rel) > half_fov + angular_radius:
            continue                      # 视野外 -> 感知不到
        visible = (cfg.enable_high and ob.zmin <= cfg.h_high <= ob.zmax) or \
            (ob.zmin <= cfg.h_low <= ob.zmax)
        if not visible:
            continue                      # 层高度不匹配 -> 这一层看不见它
        out.append(ob)
    # 遮挡没建模（射线层已经近似处理），这里当作"视野内可见"
    _ = (angles, ranges_low, ranges_high)
    return out


def observe(x: float, y: float, psi: float, world: World, cfg: SensorConfig,
            t: float, rng: Optional[random.Random] = None) -> Observation:
    half = math.radians(cfg.fov_deg) / 2.0
    angles = [-half + (2.0 * half) * i / (cfg.n_rays - 1)
              for i in range(cfg.n_rays)]
    low = cast_layer(x, y, psi, world.obstacles, cfg, cfg.h_low, t, rng)
    high = cast_layer(x, y, psi, world.obstacles, cfg, cfg.h_high, t, rng) \
        if cfg.enable_high else [cfg.max_range] * cfg.n_rays
    found, nearest = probe_drops(x, y, psi, world, cfg)

    # 悬空层独有命中 = 相机层看不见的悬空危险（<= 这一条才值得报警，
    # 否则每个柱子和行人都会被误报成 overhead）
    overhead_hits = [hi for lo, hi in zip(low, high)
                     if hi < cfg.max_range and lo > hi + 0.25]
    return Observation(t=t, angles=angles, ranges_low=low, ranges_high=high,
                       overhead_hits=overhead_hits,
                       detected_drops=found, drop_probe_dist=nearest)
