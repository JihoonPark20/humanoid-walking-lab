# A3: Jacobian and Manipulability

[English](a3_jacobian.md) · [한국어](a3_jacobian.ko.md) · [简体中文](a3_jacobian.zh-CN.md)

6D Jacobian 满足 `twist=[v,ω]=J(q) qd`。Revolute joint 的几何列为 `Jv_i=z_i×(p_ee-p_i)`、`Jω_i=z_i`；轴和位置必须使用同一 world frame。

比较 Pinocchio `LOCAL_WORLD_ALIGNED`、直接计算、finite difference 和 MuJoCo site Jacobian。LOCAL/BODY/WORLD 表达及 linear/angular 顺序不一致时不能直接比较数值。除位置差分外还要检查旋转 log。

Singularity 附近最小 singular value 变小，condition number 变大。Manipulability `sqrt(det(J J.T))` 是受单位 scaling 和 task 选择影响的局部指标。用 SVD 观察 shoulder/elbow/wrist singularity 丢失的方向。

在 Lab 目录运行 `python src/a3_jacobian.py`。比较 `a3_jacobian_validation.csv`、`a3_singularity_cases.csv`、`a3_manipulability_heatmap.csv`，结合 A4 理解为何在奇异点附近使用 DLS。

[学习笔记](README.zh-CN.md) · [运行指南](../README.zh-CN.md)
