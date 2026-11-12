# Interview Cheatsheet

[English](interview_cheatsheet.md) · [한국어](interview_cheatsheet.ko.md) · [简体中文](interview_cheatsheet.zh-CN.md)

- **FK 与 IK：** FK 从 q 求 pose，IK 从 pose 求可能的 q；IK 可能多解、无解或依赖初值。
- **Jacobian：** `v_task=J qd`，虚功关系 `tau=J.T wrench`。必须明确 frame、单位、linear/angular 顺序。
- **Singularity：** J 降秩导致失去运动方向。SVD 用于诊断，DLS/速度限制用于缓解，但不能让不可达目标变可达。
- **Computed torque：** 用 M 和 bias 补偿 nominal 非线性，形成 PD 误差动力学；模型误差和 saturation 会破坏理想关系。
- **OSC：** 考虑 task-space inertia 与 bias 的 task acceleration/force 控制，与简单 `J.T F` impedance 区分。
- **Humanoid 联系：** Floating base 不能直接驱动，contact wrench 决定平衡。Joint/task IK 不保证 friction、ZMP 或 torque 可行。Lab 5–10 将 support、WBC、trajectory optimization、MPC、OCP 连接起来。

解释公式后，用实际代码的 frame、验证标准和失败条件举例。

[学习笔记](README.zh-CN.md) · [运行指南](../README.zh-CN.md)
