# Simulink 模型结构验证报告

- 模型文件：`C:\Users\asus\WorkBuddy\2026-10-12-16-20-00\guidebot\matlab\guidebot_mil.slx`
- 顶层块数：6
- 块清单：Actuator_Plant / Perception_Planning / SafetyFSM / Split / State / logState / tNow

## 1. 输入端口连通性

| 块 | 端口 | 已连线 |
|---|---|---|
| Actuator_Plant | In1 | YES |
| Actuator_Plant | In2 | YES |
| Actuator_Plant | In3 | YES |
| Actuator_Plant | In4 | YES |
| Perception_Planning | In1 | YES |
| Perception_Planning | In2 | YES |
| Perception_Planning | In3 | YES |
| Perception_Planning | In4 | YES |
| Perception_Planning | In5 | YES |
| Perception_Planning | In6 | YES |
| SafetyFSM | In1 | YES |
| SafetyFSM | In2 | YES |
| SafetyFSM | In3 | YES |
| SafetyFSM | In4 | YES |
| SafetyFSM | In5 | YES |
| SafetyFSM | In6 | YES |
| Split | In1 | YES |
| State | In1 | YES |
| logState | In1 | YES |

悬空输入端口数：**0**

## 2. Stateflow 安全状态机

- 输入数据（6）：brakeRule, overlap, gate, tight, over, drop
- 输出数据（2）：brake, cue
- 状态（5）：NOMINAL, HAZARD_OVER, STOP, SLOW, HAZARD_DROP
- 转移数：12

| 源 | 目标 | 条件 |
|---|---|---|
| STOP | NOMINAL | `[~(brakeRule \|\| overlap \|\| gate)]` |
| HAZARD_OVER | NOMINAL | `[~over]` |
| HAZARD_DROP | NOMINAL | `[~drop]` |
| (default) | NOMINAL | `?` |
| SLOW | NOMINAL | `[~tight]` |
| NOMINAL | HAZARD_OVER | `[over]` |
| NOMINAL | HAZARD_DROP | `[drop]` |
| NOMINAL | SLOW | `[tight]` |
| NOMINAL | STOP | `[brakeRule \|\| overlap \|\| gate]` |
| HAZARD_OVER | STOP | `[brakeRule \|\| overlap \|\| gate]` |
| SLOW | STOP | `[brakeRule \|\| overlap \|\| gate]` |
| HAZARD_DROP | STOP | `[brakeRule \|\| overlap \|\| gate]` |

## 3. 闭环检查

- `Actuator_Plant` 输出 -> State
- 反馈闭环：**YES**（Plant 输出必须回到 Unit Delay 才能构成递推）

---

_由 `matlab/validate_guidebot_model.m` 自动生成。_
