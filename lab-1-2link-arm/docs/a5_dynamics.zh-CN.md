# A5: Dynamics Basics — Study Notes

[English](a5_dynamics.md) · [한국어](a5_dynamics.ko.md) · [简体中文](a5_dynamics.zh-CN.md)

基本方程为 `tau=M(q) qdd + C(q,qd) qd + g(q)`。M 是随构型变化的惯性矩阵，Cqd 为 Coriolis/centrifugal 项，g 为重力 torque。本实验重点是从 MuJoCo 读取各项。

`qfrc_bias` 包含 Cqd+g。使用与 MuJoCo 版本匹配的 `mj_fullM` 获取质量矩阵，注意旧版 packed `qM` 与当前 API 调用方式不同。

XY 平面机械臂 joint 轴沿 z，因此 z 向重力几乎不产生 joint torque。`a5_dynamics_basics.py` 比较零重力、z 重力、y 重力，以及 `[0°,0°]` 与 `[90°,-90°]` 的 M。

```bash
python src/a5_dynamics_basics.py
```

在 Lab 目录运行；依赖齐全时生成 `a5_bias_cases.csv` 与 `a5_mass_matrix_cases.csv`。缺少依赖时只输出概念说明，不能视为物理验证。

[学习笔记](README.zh-CN.md) · [运行指南](../README.zh-CN.md)
