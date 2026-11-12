# Lab 1：学习笔记

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

[运行命令与代码顺序](../README.zh-CN.md) · [共同概念](../../docs/CONCEPTS.zh-CN.md)

## 核心概念的联系

Forward Kinematics 将关节角 `q` 转为末端位置 `x`。平面二连杆的关系为：

```text
x = L1 cos(q1) + L2 cos(q1 + q2)
y = L1 sin(q1) + L2 sin(q1 + q2)
```

比较 `a2_forward_kinematics.py` 中的 `fk_endeffector` 与 `fk_homogeneous`。两者得到相同位置；齐次变换更易扩展到长链。比较 MuJoCo 时必须匹配连杆长度及末端 site 的偏移。

Jacobian 描述 `xdot = J(q) qdot`，可对 FK 求导并用有限差分独立验证。手臂伸直或完全折叠时会出现 singularity，丢失一个运动方向。

Inverse Kinematics 从目标位置求关节角。解析 IK 可产生 elbow-up/down 两个分支；数值 IK 将当前 FK 误差转换为关节增量：

```text
DLS: Δq = α Jᵀ (J Jᵀ + λ² I)⁻¹ (x_target − x)
```

增加 `λ` 可抑制奇异位形附近的大增量，但可能减慢收敛。增加迭代次数不能使不可达目标变得可达。

## 区分轨迹生成与控制

生成器给出随时间变化的 `q_des、qd_des、qdd_des`。Cubic 指定端点位置与速度；quintic 还指定加速度。关节空间直线通常不是 Cartesian 直线。

控制器比较目标与实际位置、速度：

```text
PD + gravity: τ = Kp(q_des − q) + Kd(qd_des − qd) + g(q)
Computed torque: τ = M(q)[qdd_des + Kp e + Kd edot] + h(q, qd)
```

区分 `b2_pd_controller.py` 的解析 plant 与 `c1_draw_square.py` 的 MuJoCo 仿真。`qfrc_bias` 通常同时包含重力与速度相关偏置。

## 练习

1. 在 `q1=q2=0` 时手算 FK。
2. 为同一目标绘制两个解析 IK 分支。
3. 在奇异位形附近比较 pseudoinverse 与 DLS 的迭代次数及误差。
4. 对同一轨迹只改变 PD 增益或重力补偿。

## 参考

[A1 MuJoCo](a1_mujoco_basics.zh-CN.md) · [A2 FK](a2_forward_kinematics.zh-CN.md) · [A3 Jacobian](a3_jacobian.zh-CN.md) · [A4 IK](a4_inverse_kinematics.zh-CN.md) · [A5 Dynamics](a5_dynamics.zh-CN.md) · [B1 轨迹](b1_trajectory_generation.zh-CN.md) · [B2 PD](b2_pd_controller.zh-CN.md) · [B3 流程](b3_full_pipeline.zh-CN.md) · [C1 正方形](c1_draw_square.zh-CN.md)
