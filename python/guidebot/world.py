"""GuideBot 仿真世界：场景模型、障碍物与行人。

坐标约定：x 向前（走廊方向），y 向左，航向角 psi 逆时针为正。
单位：米 / 秒 / 弧度。零第三方依赖，Python 3.9+ 可直接运行。

设计说明（对应 README 的 DESIGN.md）：
 - Obstacle 带 zmin/zmax 是本项目建模的关键。真实世界里：
   * 消防栓/石墩   -> z 低矮，低层（相机高度）射线易被糊掉
   * 广告牌横杆    -> z 悬空（1.35~1.8m），相机高度射线直接从下方穿过，
                      这正是"传统盲杖探不到、Glide 宣称能处理"的那类危险
   * 行人/柱子     -> 全高度
 因此必须有两层射线：h_low（相机层）与 h_high（头部/悬空层）。
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

ROBOT_RADIUS = 0.22   # 设备与人的等效安全半径
# 等效轴距：转向轮轴到使用者行进重心的距离。
# 注意别直接用两轮间距 0.22m —— 那会算出 >4 rad/s 的偏航角速度，
# 采样规划器会学会"原地转圈刷分"，详见 planner.py 的踩坑记录。
WHEELBASE = 0.45
HUMAN_HEIGHT = 1.70   # 碰撞判定用的使用者身高上限（决定悬空障碍是否真的伤人）


@dataclass
class Obstacle:
    x: float
    y: float
    r: float
    zmin: float = 0.0
    zmax: float = 2.2
    vx: float = 0.0
    vy: float = 0.0
    kind: str = "static"          # static | pedestrian | overhead

    def center_at(self, t_rel: float) -> Tuple[float, float]:
        """基于恒定速度模型预测 t_rel 秒后的位置。"""
        return (self.x + self.vx * t_rel, self.y + self.vy * t_rel)

    def is_lethal(self) -> bool:
        """是否真的会伤到人：悬空横杆也会撞到头部/躯干，所以同样致命。"""
        return not (self.zmin > HUMAN_HEIGHT or self.zmax < 0.0)


@dataclass
class DropZone:
    """负障碍：坑洞 / 下行台阶。只有近距下视探头能在有限探测距离内发现。"""
    x: float
    y: float
    r: float
    depth: float = 0.15


@dataclass
class World:
    length: float
    half_width: float
    obstacles: List[Obstacle] = field(default_factory=list)
    drops: List[DropZone] = field(default_factory=list)
    goal_x: float = 0.0

    def lethal_obstacles(self) -> List[Obstacle]:
        return [o for o in self.obstacles if o.is_lethal()]


# --------------------------------------------------------------------------
# 场景生成器：五种典型场景 + 随机种子，保证可复现（评测框架的基础）
# --------------------------------------------------------------------------

def make_scenario(name: str, seed: int, length: float = 40.0,
                  half_width: float = 2.0) -> World:
    """场景生成器（按种子可复现）。

    设计原则：**保证可解，但必须用对能力才能解**。
    早期版本是纯随机撒障碍，生成的某些布局物理上根本过不去，
    于是"卡死率"里混进了大量不可解样本的噪声，Ablation 结论完全没法归因。
    现在的做法是：静态障碍走"障碍跑法"贴在两侧留出中央走廊，
    悬空横杆和坑洞则故意摆在中央 —— 想过去就必须依赖对应的那一层感知。
    """
    rng = random.Random(f"{name}:{seed}")
    obstacles: List[Obstacle] = []
    drops: List[DropZone] = []

    # 障碍跑法：沿两侧交替布置，中央始终留出 >=1.4m 的通行带
    n_slalom = {"corridor": 5, "crowd": 4, "overhead": 4, "dropoff": 4,
                "mixed": 4}.get(name, 4)
    span = length - 8.0
    side = 1
    for i in range(n_slalom):
        x = 5.0 + span * i / max(n_slalom - 1, 1) + rng.uniform(-1.2, 1.2)
        y = side * rng.uniform(0.75, 1.55)
        y = max(-half_width + 0.3, min(half_width - 0.3, y))
        obstacles.append(Obstacle(x, y, rng.uniform(0.15, 0.30)))
        side *= -1

    def central_xs(n: int) -> List[float]:
        return [8.0 + (length - 12.0) * (i + 0.5) / n + rng.uniform(-1.0, 1.0)
                for i in range(n)]

    if name == "overhead":
        # 悬空横杆挡在正中间：不靠 high layer 就一定会撞
        for x in central_xs(4):
            obstacles.append(Obstacle(x, rng.uniform(-0.45, 0.45),
                                      rng.uniform(0.22, 0.35),
                                      zmin=1.35, zmax=1.85, kind="overhead"))
    elif name == "dropoff":
        # 中央的坑/下行台阶：不靠近距下视探头就一定会踩空
        for x in central_xs(3):
            drops.append(DropZone(x, rng.uniform(-0.35, 0.35),
                                  rng.uniform(0.35, 0.5)))
        for _ in range(2):
            drops.append(DropZone(rng.uniform(4, length - 3),
                                  rng.uniform(0.8, 1.5), rng.uniform(0.3, 0.5)))
    elif name == "crowd":
        for _ in range(8):
            obstacles.append(Obstacle(
                rng.uniform(4, length - 3), rng.uniform(-1.4, 1.4),
                rng.uniform(0.18, 0.26), zmin=0.0, zmax=1.75,
                vx=rng.uniform(-0.9, 0.2), vy=rng.uniform(-0.7, 0.7),
                kind="pedestrian"))
    elif name == "mixed":
        for x in central_xs(3):
            obstacles.append(Obstacle(x, rng.uniform(-0.4, 0.4),
                                      rng.uniform(0.22, 0.35),
                                      zmin=1.35, zmax=1.85, kind="overhead"))
        for x in central_xs(2):
            drops.append(DropZone(x, rng.uniform(-0.3, 0.3),
                                  rng.uniform(0.35, 0.5)))
        for _ in range(6):
            obstacles.append(Obstacle(
                rng.uniform(4, length - 3), rng.uniform(-1.4, 1.4),
                rng.uniform(0.18, 0.26), zmin=0.0, zmax=1.75,
                vx=rng.uniform(-0.9, 0.2), vy=rng.uniform(-0.7, 0.7),
                kind="pedestrian"))
    elif name not in ("corridor",):
        raise ValueError(f"unknown scenario: {name}")

    return World(length=length, half_width=half_width,
                 obstacles=obstacles, drops=drops, goal_x=length - 2.0)


SCENARIOS = ["corridor", "crowd", "overhead", "dropoff", "mixed"]


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a
