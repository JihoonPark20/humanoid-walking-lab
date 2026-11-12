# A1: Dynamics Fundamentals

[English](a1_dynamics_fundamentals.md) · [한국어](a1_dynamics_fundamentals.ko.md) · [简体中文](a1_dynamics_fundamentals.zh-CN.md)

交叉验证 MuJoCo UR5e/Robotiq 与 Pinocchio URDF 的动力学。

```text
M(q) qdd + C(q,qd) qd + g(q) = tau
M=M.T, eig(M)>0
M_dot-2C = -(M_dot-2C).T
```

用 CRBA（`pin.crba`）求 M 并对称化三角部分。对 `computeCoriolisMatrix` 的 C 与数值 M_dot 检查 passivity 关系。将 `computeGeneralizedGravity` 的 g 与 qd=0 的 MuJoCo `qfrc_bias` 比较；qd≠0 时 bias 还包含 Cqd。

`lab3_common.py` 保留 Menagerie arm actuator，将期望 joint torque 转为 ctrl。Pinocchio 应匹配固定 Robotiq payload 和 `model.armature`，不要与 `rotorInertia` 混淆。应在调 controller 前先排除 model mismatch。

在 Lab 目录运行 `python src/a1_dynamics_fundamentals.py`。检查 M heatmap、gravity bar/sweep 及测试容差，在多个 q/qd 下验证对称性、positive definiteness 与跨引擎一致性。

[学习笔记](README.zh-CN.md) · [运行指南](../README.zh-CN.md)
