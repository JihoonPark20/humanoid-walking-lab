# B1: Cartesian Impedance Control

[English](b1_impedance_control.md) · [한국어](b1_impedance_control.ko.md) · [简体中文](b1_impedance_control.zh-CN.md)

Cartesian impedance는 끝점의 가상 spring·damper 관계를 joint torque로 옮깁니다.

```text
F = Kp(x_des-x)+Kd(v_des-v)
tau = J.T F + g(q)
e_R = log3(R_des R.T)
wrench = [F_linear; K_R e_R + D_R(omega_des-omega)]
```

6D에서는 linear/angular 순서와 `LOCAL_WORLD_ALIGNED` frame을 맞춥니다. 회전 log는 Euler-angle 차이보다 적합하지만 π 부근의 rotation 표현 한계까지 없애지는 않습니다. Kp가 크면 같은 외력의 변위가 작아지는 경향이 있으나 실제 변위는 configuration·제한·접촉·동특성에도 의존합니다.

Lab 폴더에서 `python src/b1_impedance_controller.py`, `python src/b2_compliance_demo.py`를 실행하세요. 3D·6D pose 복원과 soft/medium/stiff 설정을 비교하고 displacement·orientation·torque·외란 후 응답을 측정합니다. `tests/test_impedance.py`가 기본 동작과 orientation 처리를 검사합니다.

[학습 노트](README.ko.md) · [실행 안내](../README.ko.md)
