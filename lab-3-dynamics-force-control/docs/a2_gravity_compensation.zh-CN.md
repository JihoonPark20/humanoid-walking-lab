# A2: Gravity Compensation

[English](a2_gravity_compensation.md) · [한국어](a2_gravity_compensation.ko.md) · [简体中文](a2_gravity_compensation.zh-CN.md)

最简单的模型输入是 `tau=g(q)`。它只抵消重力，没有将机器人拉回初始姿态的位置 stiffness。

```text
M(q) qdd + C(q,qd) qd = tau_passive
```

精确补偿且初速度为零时可以保持当前姿态。Damping 可减小扰动后的速度，但不保证回到原 q；恢复姿态需要 PD 或 impedance。M 随构型变化，也不能将整个系统一概描述为单一指数衰减。

在 Lab 目录运行 `python src/a2_gravity_compensation.py`，分别检查 hold 最大 q 误差与 perturbation 后最终速度。`tests/test_gravity_comp.py` 要求 hold 误差 <0.01 rad、扰动后速度 <0.1 rad/s。通过图区分位置 drift 与速度 damping。

[学习笔记](README.zh-CN.md) · [运行指南](../README.zh-CN.md)
