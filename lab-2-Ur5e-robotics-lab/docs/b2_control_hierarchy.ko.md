# B2: Control Hierarchy

[English](b2_control_hierarchy.md) · [한국어](b2_control_hierarchy.ko.md) · [简体中文](b2_control_hierarchy.zh-CN.md)

`b2_control_hierarchy.py`는 동일 reference에 PD+gravity, computed torque, task-space impedance, Operational Space Control을 비교합니다.

```text
tau_PD = Kp(q_des-q)+Kd(v_des-v)+g(q)
tau_CT = M(q)[a_des+Kp(q_des-q)+Kd(v_des-v)] + C(q,v)v + g(q)
F_imp = Kx(x_des-x)+Dx(v_des_task-v_task)
tau_imp = J.T F_imp + g(q)
Lambda = (J M^(-1) J.T)^(-1)
```

OSC는 task-space inertia Lambda와 bias를 고려합니다. Singularity에서는 inverse를 regularize해야 합니다. Task Jacobian·error·wrench의 frame을 일치시키세요. 정확한 모델에서의 computed torque 오차 선형화는 torque saturation·contact·모델 오차가 생기면 그대로 유지되지 않습니다.

Lab 폴더에서 `python src/b2_control_hierarchy.py`를 실행해 `b2_controller_summary.csv`와 각 tracking CSV의 오차·torque를 비교합니다. 추종기와 plant의 모델이 같은 경우 얻은 좋은 결과를 모델 강인성으로 확대 해석하지 않습니다.

[학습 노트](README.ko.md) · [실행 안내](../README.ko.md)
