# C1: Hybrid Position-Force Control

[English](c1_force_control.md) · [한국어](c1_force_control.ko.md) · [简体中文](c1_force_control.zh-CN.md)

Hybrid position/force control은 XY 위치와 Z 접촉력을 분리합니다. `S_p=diag(1,1,0)`, `S_f=diag(0,0,1)`입니다.

```text
F_p = Kp S_p(x_des-x)+Kd S_p(v_des-v)
e_f = F_des-F_measured
F_z = -(F_des+Kfp e_f+Kfi integral(e_f)) - Kdz v_z
moment = K_R log3(R_des R.T)-K_w omega
tau = J_linear.T(F_p+F_f)+J_angular.T moment+g(q)
```

아래 방향 feedforward가 하중을, PI가 오차 보정을, damping·orientation feedback이 진동·회전을 제어합니다. `mj_contactForce()`의 마찰 포함 전체 force를 world frame으로 변환해 Z로 투영합니다. EMA α=0.2를 사용하지만 판정은 raw force도 검사합니다.

대표 장면은 반경 10 mm·pinch frame에서 60 mm 떨어진 질량 없는 강체 probe와 μ=0.05 평판입니다. 그리퍼는 닫힌 상태로 시작하고 actuator command 230/255로 유지하며 관절 상태를 강제로 덮어쓰지 않습니다. Probe는 tool에 고정되어 있으므로 articulated finger 접촉·polishing benchmark가 아닙니다. Wrist·finger의 평판 접촉은 실패입니다.

상태는 `APPROACH → SETTLE → TRACE → HOLD`입니다. 10 mm/s로 접근하고 1.5 s 정착한 뒤 5 N으로 6 s 동안 50 mm 선을 그립니다. 영상은 모든 단계를 포함하며 `run_line_trace()`의 같은 실행에서 측정합니다.

기준: raw·filtered 모두 5±1 N 비율 >90%, 전체 peak <15 N, XY <5 mm, orientation <5°, 완료, probe 외 접촉 0, 양쪽 gripper driver >0.65 rad입니다. Gain은 XY Kp/Kd=2000/100, force PI=0.5/3, Z damping=30, rotation=60/1입니다.

Lab 폴더에서 `python src/c1_force_control.py` 또는 `python src/record_lab3_demo.py`를 실행합니다. [측정값](../media/line_trace_summary.json)과 [영상](../GALLERY.ko.md)을 함께 확인하세요. 긴 선·불리한 Jacobian·다른 마찰에서는 속도와 제어 설정을 별도 검증해야 합니다.

[학습 노트](README.ko.md) · [실행 안내](../README.ko.md)
