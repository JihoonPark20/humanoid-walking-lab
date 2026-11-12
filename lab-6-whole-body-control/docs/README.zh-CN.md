# Lab 6：学习笔记

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

[运行命令与代码顺序](../README.zh-CN.md) · [共同概念](../../docs/CONCEPTS.zh-CN.md)

## 将平衡写入动力学约束

Floating base 没有直接驱动力矩，因此接触力必须满足下式的 base 部分：

```text
M(q) qddot + h(q, qdot) = Sᵀ τ + Jcᵀ f
Jc qddot + Jcdot qdot = 0
```

`wb_id_qp.py` 优化加速度与 contact wrench，同时处理 base dynamics、支撑足加速度、摩擦、CoP 和 torque 限制。在 `wb_tasks.py` 中追踪各 task 的 Jacobian 与 `Jdot qdot`。

Velocity-level IK/QP 不直接决定所需支撑力。本平衡任务还需独立的力控制层；kinematic QP 在这样的架构中仍有用途。

## 用 DCM 理解行走

```text
ξ = c + cdot/ω
ξdot = ω(ξ − p)
```

DCM 同时包含 CoM 位置与速度。`dcm_planner.py` 根据脚步和 ZMP 生成参考，`locomotion_controller.py` 根据测量状态修正参考。这不同于让 CoM 静止在单脚上方。

## 添加手部 task

手臂和 payload 的运动会影响 centroidal momentum 与 CoM。提高手部权重可能与平衡目标冲突。比较 momentum task、固定搬运姿态和运动中的手部参考。

QP 达到迭代上限后，即使返回数值也不能视为收敛。应共同检查 scaling、迭代数、residual 和运行时间。

## 模型与接触

CoP 区域必须匹配足底 geometry 与 wrench 参考系。踝部 frame 高于地面时，要计入水平力产生的力矩。添加家具后，应先查 contact pair，再调 gain。

启用 weld 时保留当前相对 pose；释放时检查物体目标和实际静止 pose，而不只是手的位置。已发布的成功不能保证新物体布局同样成功。
