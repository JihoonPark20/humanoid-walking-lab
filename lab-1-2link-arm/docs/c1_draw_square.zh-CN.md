# C1: Cartesian Square Drawing — Study Notes

[English](c1_draw_square.md) · [한국어](c1_draw_square.ko.md) · [简体中文](c1_draw_square.zh-CN.md)

用实际 MuJoCo torque control 跟踪 XY 正方形：中心 `(0.30,0.30)` m、边长 0.10 m、每边 2 s。流程为 Cartesian quintic → analytic IK → Jacobian 速度 → 数值加速度 → computed torque。选择最接近上次 q 的 IK 分支，初始优先 q2 为负的分支。

```text
u = qdd_des + Kp (q_des-q) + Kd (qd_des-qd)
tau = M(q) u + qfrc_bias - qfrc_passive
Kp=400, Kd=40, |tau|≤5 N·m
```

在精确模型、无饱和条件下，误差方程为 `e_ddot+Kd e_dot+Kp e=0`。Kp=400/Kd=40 是理想临界阻尼配置，不代表在实际限制与模型误差下也绝无 overshoot。`mj_fullM` 调用适配安装的 MuJoCo 版本。

此基础跟踪示例禁用了 geom 碰撞，因此不是避障或 self-collision 验证。Lab 4 在启用接触的情况下验证避障。XML 关节范围必须按 radian 解释。

macOS interactive viewer 在 Lab 目录运行 `mjpython src/c1_draw_square.py`。绿色线为目标，红色 trail 为实际末端，黄色球为当前位置。查看[录制指标](../media/c1_draw_square.json)中的 RMS、最大误差与 torque。

[学习笔记](README.zh-CN.md) · [运行指南](../README.zh-CN.md)
