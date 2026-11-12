# C1: Cartesian Square Drawing — Study Notes

[English](c1_draw_square.md) · [한국어](c1_draw_square.ko.md) · [简体中文](c1_draw_square.zh-CN.md)

실제 MuJoCo torque 제어로 XY 평면 정사각형을 추종합니다. 중심 `(0.30,0.30)` m, 변 0.10 m, 변당 2 s입니다. Cartesian quintic → analytic IK → Jacobian 속도 → 수치 가속도 → computed torque 순서입니다. 이전 q에 가까운 IK 분기를 선택하고 첫 q2는 음수 분기를 선호합니다.

```text
u = qdd_des + Kp (q_des-q) + Kd (qd_des-qd)
tau = M(q) u + qfrc_bias - qfrc_passive
Kp=400, Kd=40, |tau|≤5 N·m
```

정확한 모델·비포화 조건에서 오차 식은 `e_ddot+Kd e_dot+Kp e=0`입니다. Kp=400·Kd=40은 이상적인 임계 감쇠 설정이며, 실제 제한·모델 오차에서도 무조건 overshoot가 없다는 뜻은 아닙니다. `mj_fullM` 호출은 설치된 MuJoCo 버전에 맞게 처리합니다.

이 기초 추종 예제는 geom 충돌을 비활성화하므로 장애물 회피·self-collision 검증이 아닙니다. 충돌 회피는 Lab 4에서 접촉을 활성화한 채 검증합니다. XML 관절 범위는 radian으로 해석되어야 합니다.

macOS의 interactive viewer는 Lab 폴더에서 `mjpython src/c1_draw_square.py`로 실행하세요. 초록 선은 목표, 빨간 trail은 실제 끝점, 노란 구는 현재 위치입니다. [녹화 측정값](../media/c1_draw_square.json)에서 RMS·최대 오차·torque를 확인하세요.

[학습 노트](README.ko.md) · [실행 안내](../README.ko.md)
