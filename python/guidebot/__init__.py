"""GuideBot：复刻 Glide 技术思路的纯软件导盲巡检栈（仿真 + 评测）。

零第三方依赖，Python 3.9+ 可直接运行。
"""
from .world import (Obstacle, DropZone, World, make_scenario, ROBOT_RADIUS,
                    WHEELBASE, HUMAN_HEIGHT)
from .sensors import SensorConfig, Observation, observe
from .planner import PlannerConfig, Plan, plan_steering
from .controller import ControlConfig, Command, supervise, required_free_distance
from .sim import (HumanConfig, SimConfig, EpisodeResult, Human, run_episode)

__version__ = "0.1.0"
__all__ = [
    "Obstacle", "DropZone", "World", "make_scenario", "ROBOT_RADIUS",
    "WHEELBASE", "HUMAN_HEIGHT",
    "SensorConfig", "Observation", "observe",
    "PlannerConfig", "Plan", "plan_steering",
    "ControlConfig", "Command", "supervise", "required_free_distance",
    "HumanConfig", "SimConfig", "EpisodeResult", "Human", "run_episode",
]
