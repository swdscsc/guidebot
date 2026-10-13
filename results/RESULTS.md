# RESULTS

所有数字由 `run_benchmark.py` 自动生成，可完全复现（固定随机种子）。

## 主实验：Ablation 对比

| variant | episodes | total_km | goal_rate | stuck_rate | collisions_per_km | overhead_per_km | pedestrian_per_km | drop_per_km | near_miss_per_km | stops_per_km | conservative_stops_per_km | lateral_rmse_cm | steering_slew_rms_dps |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| full | 15 | 0.55 | 0.867 | 0.133 | 0.0 | 0.0 | 0.0 | 0.0 | 5.46 | 72.77 | 0.0 | 67.1 | 38.6 |
| no_brake_rule | 15 | 0.586 | 1.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 5.12 | 0.0 | 0.0 | 71.3 | 38.8 |
| no_gate | 15 | 0.55 | 0.867 | 0.133 | 0.0 | 0.0 | 0.0 | 0.0 | 5.46 | 72.77 | 0.0 | 67.1 | 38.6 |
| no_high_layer | 15 | 0.545 | 0.867 | 0.133 | 18.35 | 18.35 | 0.0 | 0.0 | 27.52 | 60.55 | 0.0 | 59.2 | 33.8 |
| no_probe | 15 | 0.574 | 1.0 | 0.0 | 0.0 | 0.0 | 0.0 | 13.94 | 5.23 | 0.0 | 0.0 | 61.0 | 35.6 |
| no_prediction | 15 | 0.576 | 0.933 | 0.067 | 0.0 | 0.0 | 0.0 | 0.0 | 5.21 | 50.33 | 0.0 | 68.5 | 36.3 |

### 各变体说明

- **full**: 完整栈：双层射线 + 下视探头 + 制动走廊 + 可控性门限
- **no_brake_rule**: 去掉制动距离不等式（只做纯反应式避障）
- **no_gate**: 去掉低速可控性门限
- **no_high_layer**: 去掉悬空层射线（模拟传统盲杖的物理盲区）
- **no_probe**: 去掉近距下视探头（看不到坑洞/下行台阶）
- **no_prediction**: 不使用运动预测（横穿行人来不及反应）

## 压力实验：感知退化 + 高延迟（延迟 300ms / 丢帧 6%）

这一组才是安全监督层与运动预测真正起作用的场景：

| variant | episodes | total_km | goal_rate | stuck_rate | collisions_per_km | overhead_per_km | pedestrian_per_km | drop_per_km | near_miss_per_km | stops_per_km | conservative_stops_per_km | lateral_rmse_cm | steering_slew_rms_dps |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| stress/full | 15 | 0.516 | 0.667 | 0.333 | 1.94 | 0.0 | 1.94 | 0.0 | 3.87 | 129.8 | 1.94 | 69.6 | 38.4 |
| stress/no_safety | 15 | 0.592 | 1.0 | 0.0 | 3.38 | 0.0 | 1.69 | 3.38 | 5.07 | 0.0 | 0.0 | 77.5 | 53.5 |
| stress/no_prediction | 15 | 0.539 | 0.733 | 0.267 | 5.56 | 0.0 | 3.71 | 0.0 | 7.42 | 102.02 | 0.0 | 75.8 | 40.6 |

- **stress/full**: 压力条件（延迟 300ms + 丢帧 6%）下的完整栈
- **stress/no_safety**: 压力条件 + 去掉安全监督层
- **stress/no_prediction**: 压力条件 + 不做运动预测

## 延迟敏感性实验（mixed 场景）

| latency_ms | collisions_per_km | near_miss_per_km | stops_per_km | conservative_stops_per_km | lateral_rmse_cm |
| --- | --- | --- | --- | --- | --- |
| 20 | 0.0 | 8.48 | 0.0 | 0.0 | 79.8 |
| 100 | 0.0 | 10.6 | 211.94 | 0.0 | 76.7 |
| 200 | 0.0 | 14.28 | 371.37 | 0.0 | 73.6 |
| 350 | 10.03 | 40.12 | 140.42 | 0.0 | 82.2 |

_总运行耗时 154.4s，Python 标准库实现，无第三方依赖。_
