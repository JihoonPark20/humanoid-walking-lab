# A5: Dynamics

[English](a5_dynamics.md) · [한국어](a5_dynamics.ko.md) · [简体中文](a5_dynamics.zh-CN.md)

用 Pinocchio 计算 `M(q) qdd + C(q,v)v + g(q)=tau`。CRBA 求 M，RNEA 从 q/v/qdd 求 torque，ABA 从 torque 求加速度；`computeGeneralizedGravity` 求 g，`computeCoriolisMatrix` 求 C。

用 RNEA/ABA roundtrip 恢复 qdd，区分 v=0 的 gravity 与 v≠0 的 bias。模型、armature、tool 质量、关节顺序一致时，才能比较 MuJoCo `qfrc_bias` 与 Pinocchio `rnea(q,v,0)`。M 应对称 positive definite，其 condition number 随 q 变化。

在 Lab 目录运行 `python src/a5_dynamics.py`，阅读 `a5_dynamics_snapshot.csv` 和姿态 sweep，交叉验证 mass matrix、gravity、Coriolis、inverse/forward dynamics。不要把模型差异误认为 solver 错误。

[学习笔记](README.zh-CN.md) · [运行指南](../README.zh-CN.md)
