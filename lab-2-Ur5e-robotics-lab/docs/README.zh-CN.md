# Lab 2：学习笔记

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

[运行命令与代码顺序](../README.zh-CN.md) · [共同概念](../../docs/CONCEPTS.zh-CN.md)

## 从二维到三维

UR5e 末端 pose 是包含位置 `p` 与旋转 `R` 的 `SE(3)` 变换。FK 连乘各关节变换；六维 Jacobian 将关节速度映射为线速度与角速度。只匹配位置不能决定工具朝向。

先阅读 `ur5e_common.py` 的关节名称与模型路径，再读 `a2_forward_kinematics.py`。MuJoCo body ID 与 Pinocchio frame ID 不可互换；还要匹配 flange 与工具末端位置。

## IK 的四项检查

1. 当前 pose 与目标是否在同一坐标系？
2. 误差与 Jacobian 的坐标约定是否一致？
3. 是否按旋转几何计算误差，而非直接相减 Euler angle？
4. 收敛结果是否满足关节范围与碰撞条件？

本仓库按 `[linear, angular]` 排列 Pinocchio spatial vector。不要混用 `LOCAL_WORLD_ALIGNED` 与 body-local Jacobian；FK 后还需更新 frame placement。

## Position servo 的含义

Menagerie servo 使用下列关系；实际 gain、bias、gear 要查模型：

```text
τ = Kp(ctrl − q) − Kd qdot
ctrl = q_des + τ_feedforward/Kp + Kd qd_des/Kp
```

只设 `ctrl=q_des` 时，需要位置误差来抵抗重力，并会产生速度跟踪滞后。在 `c3_draw_cube.py` 中找出 feedforward，区分轨迹生成、servo 转换与物理积分。

## 练习

在相同关节角下比较两个引擎的 FK。微调一个关节并与 Jacobian 预测比较。改变 DLS damping 和初始姿态，观察 IK 解的变化。

## 参考

[环境](a1_environment_setup.zh-CN.md) · [FK](a2_forward_kinematics.zh-CN.md) · [Jacobian](a3_jacobian.zh-CN.md) · [IK](a4_inverse_kinematics.zh-CN.md) · [Dynamics](a5_dynamics.zh-CN.md) · [轨迹](b1_trajectory_generation.zh-CN.md) · [控制层级](b2_control_hierarchy.zh-CN.md) · [约束](b3_constraints.zh-CN.md) · [流程](c1_full_pipeline.zh-CN.md) · [立方体](c3_draw_cube.zh-CN.md) · [复习问题](interview_cheatsheet.zh-CN.md)
