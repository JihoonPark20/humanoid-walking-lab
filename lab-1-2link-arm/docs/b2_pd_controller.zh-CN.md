# B2: PD Controller — Study Notes

[English](b2_pd_controller.md) · [한국어](b2_pd_controller.ko.md) · [简体中文](b2_pd_controller.zh-CN.md)

PD + gravity compensation 为 `tau=Kp(q_des-q)+Kd(qd_des-qd)+g(q)`。Kp 提供位置恢复，Kd 对速度误差提供 damping。

`b2_pd_controller.py` 的默认 plant 是不依赖外部库的确定性 Python 模型。比较固定目标 `[90°,-45°]` 和 B1 cubic 跟踪中有无重力补偿的差异。不能将其视为实际 MuJoCo dynamics；MuJoCo hook 需要单独验证。

在 Lab 目录运行 `python src/b2_pd_controller.py`。状态保存到 `b2_fixed_target_{gc,no_gc}.csv`、`b2_tracking_{gc,no_gc}.csv`。比较 settling time、overshoot、稳态误差与 RMS，尝试修改 Kp/Kd。即使没有可选图，仍可分析 CSV。

[学习笔记](README.zh-CN.md) · [运行指南](../README.zh-CN.md)
