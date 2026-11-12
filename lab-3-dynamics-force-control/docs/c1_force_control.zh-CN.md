# C1: Hybrid Position-Force Control

[English](c1_force_control.md) · [한국어](c1_force_control.ko.md) · [简体中文](c1_force_control.zh-CN.md)

Hybrid position/force control 将 XY 位置与 Z 接触力分开，`S_p=diag(1,1,0)`、`S_f=diag(0,0,1)`。

```text
F_p = Kp S_p(x_des-x)+Kd S_p(v_des-v)
e_f = F_des-F_measured
F_z = -(F_des+Kfp e_f+Kfi integral(e_f)) - Kdz v_z
moment = K_R log3(R_des R.T)-K_w omega
tau = J_linear.T(F_p+F_f)+J_angular.T moment+g(q)
```

向下 feedforward 提供负载，PI 修正力误差，damping/orientation feedback 抑制振荡与转动。将 `mj_contactForce()` 包含摩擦的完整 force 转到 world frame 后投影至 Z。Controller 使用 EMA α=0.2，但判定同时检查 raw force。

代表场景为半径 10 mm、距 pinch frame 60 mm 的无质量刚性 probe 与 μ=0.05 平板。夹爪从闭合状态开始，以 actuator command 230/255 保持，不强制覆盖关节状态。Probe 固定在 tool 上，因此不是 articulated-finger 接触或 polishing benchmark。Wrist/finger 与平板接触即失败。

状态为 `APPROACH → SETTLE → TRACE → HOLD`。以 10 mm/s 接近，稳定 1.5 s，再以 5 N 在 6 s 内描绘 50 mm。视频包含全部阶段，指标来自同一次 `run_line_trace()`。

标准：raw/filtered 在 5±1 N 内的比例均 >90%，全程 peak <15 N，XY <5 mm，orientation <5°，完整执行，非 probe 接触为零，双侧 gripper driver >0.65 rad。Gain 为 XY Kp/Kd=2000/100、force PI=0.5/3、Z damping=30、rotation=60/1。

在 Lab 目录运行 `python src/c1_force_control.py` 或 `python src/record_lab3_demo.py`，结合[指标](../media/line_trace_summary.json)与[视频](../GALLERY.zh-CN.md)。更长线段、不利 Jacobian 或不同摩擦需要独立验证速度和控制设置。

[学习笔记](README.zh-CN.md) · [运行指南](../README.zh-CN.md)
