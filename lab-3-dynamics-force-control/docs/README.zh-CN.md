# Lab 3：学习笔记

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

[运行命令与代码顺序](../README.zh-CN.md) · [共同概念](../../docs/CONCEPTS.zh-CN.md)

## 阅读动力学方程

```text
M(q) qddot + h(q, qdot) = τ + Jᵀ f_external
h(q, qdot) = C(q, qdot) qdot + g(q)
```

`M` 为质量矩阵；`h` 合并 Coriolis、离心与重力项。RNEA 由位置、速度、加速度求所需力矩；CRBA 求 `M`。只比较重力时，将速度和加速度设为零。

`a1_dynamics_fundamentals.py` 比较 Pinocchio 与 MuJoCo。机器人名称相同并不保证 payload、惯性、damping、armature 一致，应先查模型再查求解器。

## 重力补偿与 impedance

重力补偿可减小位置误差，但不能代替刚度和阻尼。Cartesian impedance 将位置、速度误差转为虚拟弹簧阻尼 wrench，再用 `τ=JᵀF` 生成关节力矩。旋转任务还需要 `SO(3)` 误差与角速度。

提高刚度可能改善跟踪，也可能放大接触力和振荡。在 `b2_compliance_demo.py` 中比较外力引起的位移。

## Hybrid position/force control

沿桌面切向跟踪位置，沿法向调节接触力。Selection matrix 避免在同一方向同时施加强位置与强力目标。

先检查 `c1_force_control.py` 中力反馈的符号与坐标系，再读 `c2_line_trace.py`。分开查看接近、接触和释放阶段，以解释 overshoot。

代表运行使用闭合夹爪和固定于 tool body 的无质量刚性探针，在 μ=0.05 的平板上描绘 50 mm。Actuator 保持夹爪闭合。完整运行同时检查原始接触力、姿态误差、夹爪闭合和非探针接触。参阅[设置](c1_force_control.zh-CN.md)与[测量](../media/line_trace_summary.json)。

## 练习与参考

分别改变 payload、impedance 刚度与力目标。除平均误差外，还应记录峰值、振荡及 actuator saturation。

[Dynamics](a1_dynamics_fundamentals.zh-CN.md) · [重力补偿](a2_gravity_compensation.zh-CN.md) · [Impedance](b1_impedance_control.zh-CN.md) · [混合控制](c1_force_control.zh-CN.md)
