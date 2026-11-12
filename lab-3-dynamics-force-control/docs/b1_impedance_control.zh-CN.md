# B1: Cartesian Impedance Control

[English](b1_impedance_control.md) · [한국어](b1_impedance_control.ko.md) · [简体中文](b1_impedance_control.zh-CN.md)

Cartesian impedance 将末端虚拟 spring/damper 关系映射为 joint torque。

```text
F = Kp(x_des-x)+Kd(v_des-v)
tau = J.T F + g(q)
e_R = log3(R_des R.T)
wrench = [F_linear; K_R e_R + D_R(omega_des-omega)]
```

6D 必须对齐 linear/angular 顺序与 `LOCAL_WORLD_ALIGNED` frame。旋转 log 比 Euler-angle 差值合适，但不能消除 π 附近旋转表示的限制。增大 Kp 通常减小同一外力下位移，但实际响应也依赖构型、限制、接触和动态效应。

在 Lab 目录运行 `python src/b1_impedance_controller.py`、`python src/b2_compliance_demo.py`。比较 3D/6D pose 恢复及 soft/medium/stiff 设置，测量 displacement、orientation、torque 和扰动响应。`tests/test_impedance.py` 检查基本行为与 orientation 处理。

[学习笔记](README.zh-CN.md) · [运行指南](../README.zh-CN.md)
