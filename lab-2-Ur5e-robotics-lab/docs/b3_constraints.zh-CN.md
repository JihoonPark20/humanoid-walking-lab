# B3: Joint Constraints and Safety

[English](b3_constraints.md) · [한국어](b3_constraints.ko.md) · [简体中文](b3_constraints.zh-CN.md)

`check_joint_limits()` 返回各 joint 范围裕量，`clamp_joint_positions()` 做 hard clipping。`joint_limit_repulsion()` 在边界附近增加反向 torque，默认 buffer 0.1 rad、gain 5。`scale_velocity()` 等比例缩小整个速度向量以保持方向；`scale_delta_q()` 考虑 dt。

`saturate_torques()` 将前三 joint 限制为 ±150 N·m，后三 wrist joint 为 ±28 N·m。`safe_command(q,qd,tau,dt)` 依次执行速度缩放、torque 饱和、增加反弹项、再次饱和、修正下一 q 范围，并记录激活的限制。

Self-collision 仅用非相邻 link frame 间 0.05 m 距离作为几何启发式，并非实际 mesh 碰撞检查；`safe_command` 不保证避开全部碰撞。Lab 4 才检查实际环境 collision pair 和路径内部。

在 Lab 目录运行 `python src/b3_constraints.py`，阅读 `b3_limit_guard.csv`、`b3_collision_cases.csv`、`b3_stress_test.csv`。分别测试已越界初态、过速、过 torque 和 near-limit 状态。

[学习笔记](README.zh-CN.md) · [运行指南](../README.zh-CN.md)
