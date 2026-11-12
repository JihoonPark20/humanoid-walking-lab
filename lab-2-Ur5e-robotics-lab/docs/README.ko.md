# Lab 2: 학습 노트

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

[실행 명령과 코드 순서](../README.ko.md) · [공통 용어](../../docs/CONCEPTS.ko.md)

## 2차원에서 3차원으로

UR5e의 끝점 pose는 위치 `p`와 회전 `R`로 이루어진 `SE(3)` 변환입니다. FK는 관절 변환을 곱해 pose를 구하고, 6차원 Jacobian은 관절 속도를 선속도·각속도로 변환합니다. 2-link arm과 달리 위치 오차만 줄여서는 끝점의 방향이 정해지지 않습니다.

`ur5e_common.py`에서 관절 이름과 모델 경로를 확인한 뒤 `a2_forward_kinematics.py`를 읽으세요. MuJoCo body와 Pinocchio frame의 번호가 같다고 가정하면 안 됩니다. 비교할 지점이 flange인지 tool 끝인지도 맞춰야 합니다.

## IK에서 확인할 네 가지

1. 목표 pose와 현재 pose가 같은 좌표계에 있는가?
2. 오차 벡터와 Jacobian이 같은 frame convention을 사용하는가?
3. 회전 오차를 단순 Euler angle 차이가 아니라 회전의 기하에 맞게 계산하는가?
4. 수렴한 관절각이 joint limit과 충돌 조건을 만족하는가?

Pinocchio의 spatial vector는 이 코드에서 `[linear, angular]` 순서로 다룹니다. World 좌표로 작업할 때 사용하는 `LOCAL_WORLD_ALIGNED`와 body-local Jacobian을 섞지 마세요. FK 후 frame pose가 필요하면 frame placement도 갱신해야 합니다.

## Position servo의 의미

Menagerie의 servo는 내부적으로 다음 관계를 사용합니다. 실제 gain·bias·gear 설정은 모델에서 확인하세요.

```text
τ = Kp(ctrl − q) − Kd qdot
ctrl = q_des + τ_feedforward/Kp + Kd qd_des/Kp
```

`ctrl = q_des`만 넣으면 중력을 버티는 데 필요한 오차와 속도 추종 지연이 생깁니다. `c3_draw_cube.py`에서 feedforward가 들어가는 위치를 찾아 보고 궤적 생성·servo·물리 step의 역할을 분리해 읽으세요.

## 직접 해볼 실험

같은 관절각으로 Pinocchio와 MuJoCo의 FK를 비교한 뒤, 한 관절만 작은 값만큼 바꾸어 Jacobian 예측과 비교하세요. DLS damping과 초기 자세를 바꿔 IK의 해가 달라지는지도 확인합니다.

## 상세 참고 자료


- [A1: Environment Setup](a1_environment_setup.ko.md)
- [A2: Forward Kinematics](a2_forward_kinematics.ko.md)
- [A3: Jacobian and Manipulability](a3_jacobian.ko.md)
- [A4: Inverse Kinematics](a4_inverse_kinematics.ko.md)
- [A5: Dynamics](a5_dynamics.ko.md)
- [B1: Trajectory Generation](b1_trajectory_generation.ko.md)
- [B2: Control Hierarchy](b2_control_hierarchy.ko.md)
- [B3: Joint Constraints and Safety](b3_constraints.ko.md)
- [C1: Integrated Pick-and-Place Pipeline](c1_full_pipeline.ko.md)
- [C3: Draw a 3D Cube — Full Pipeline Demo](c3_draw_cube.ko.md)
- [Interview Cheatsheet](interview_cheatsheet.ko.md)
