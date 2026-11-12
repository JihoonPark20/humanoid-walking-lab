# B2: Control Hierarchy

[English](b2_control_hierarchy.md) · [한국어](b2_control_hierarchy.ko.md) · [简体中文](b2_control_hierarchy.zh-CN.md)

`b2_control_hierarchy.py` 在同一 reference 下比较 PD+gravity、computed torque、task-space impedance 和 Operational Space Control。

```text
tau_PD = Kp(q_des-q)+Kd(v_des-v)+g(q)
tau_CT = M(q)[a_des+Kp(q_des-q)+Kd(v_des-v)] + C(q,v)v + g(q)
F_imp = Kx(x_des-x)+Dx(v_des_task-v_task)
tau_imp = J.T F_imp + g(q)
Lambda = (J M^(-1) J.T)^(-1)
```

OSC 考虑 task-space inertia Lambda 与 bias。Singularity 附近需 regularize inverse。Task Jacobian、error、wrench 必须同 frame。精确模型中的 computed torque 线性误差关系在 torque saturation、contact 或模型误差下不再严格成立。

在 Lab 目录运行 `python src/b2_control_hierarchy.py`，比较 `b2_controller_summary.csv` 与各 tracking CSV 的误差和 torque。Controller 与 plant 使用同模型时的好结果不能直接视为模型鲁棒性。

[学习笔记](README.zh-CN.md) · [运行指南](../README.zh-CN.md)
