# Lab 1: 학습 노트

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

[실행 명령과 코드 순서](../README.ko.md) · [공통 용어](../../docs/CONCEPTS.ko.md)

## 먼저 연결할 개념

**Forward Kinematics**는 관절각 `q`에서 끝점 위치 `x`를 구합니다. 평면 2-link arm에서 끝점은 다음과 같습니다.

```text
x = L1 cos(q1) + L2 cos(q1 + q2)
y = L1 sin(q1) + L2 sin(q1 + q2)
```

`a2_forward_kinematics.py`의 `fk_endeffector`와 `fk_homogeneous`를 비교하세요. 두 표현은 같은 위치를 계산하지만 동차 변환은 여러 link로 확장하기 쉽습니다. MuJoCo의 끝점 `site`에 추가 offset이 있으면 식의 link 길이와 비교 기준을 맞춰야 합니다.

**Jacobian**은 `xdot = J(q) qdot`의 관계입니다. FK를 각 관절각으로 미분해서 얻으며, finite difference로 독립 검증할 수 있습니다. 팔이 펴지거나 완전히 접히면 한 방향으로 움직일 능력을 잃는 singularity가 나타납니다.

**Inverse Kinematics**는 목표 위치에서 관절각을 구합니다. Analytical IK는 elbow-up/down의 두 해를 구할 수 있습니다. Numerical IK는 현재 FK와 목표의 차이를 Jacobian으로 관절 변화량으로 바꿉니다.

```text
DLS: Δq = α Jᵀ (J Jᵀ + λ² I)⁻¹ (x_target − x)
```

`λ`를 높이면 singularity 근처에서 변화량이 완만해지지만 수렴이 느려질 수 있습니다. 목표가 workspace 밖이면 반복 횟수만 늘려도 해결되지 않습니다.

## 궤적과 제어를 구분하기

궤적 생성기는 시간별 `q_des`, `qd_des`, `qdd_des`를 만듭니다. Cubic은 양 끝의 위치·속도, quintic은 가속도까지 지정할 수 있습니다. Joint-space 직선 보간이 Cartesian 직선을 만들지는 않습니다.

제어기는 실제 `q`, `qd`와 목표를 비교해 토크를 정합니다.

```text
PD + gravity: τ = Kp(q_des − q) + Kd(qd_des − qd) + g(q)
Computed torque: τ = M(q)[qdd_des + Kp e + Kd edot] + h(q, qd)
```

`b2_pd_controller.py`의 간단한 해석 plant와 `c1_draw_square.py`의 MuJoCo 시뮬레이션은 구분해서 읽으세요. `qfrc_bias`는 일반적으로 중력만이 아니라 속도에 따른 bias도 포함합니다.

## 직접 해볼 실험

1. `q1 = q2 = 0`일 때 FK를 손으로 계산하고 코드와 비교합니다.
2. 같은 목표의 두 Analytical IK 해를 그려 봅니다.
3. Singularity 근처에서 pseudo-inverse와 DLS의 반복 수·최종 오차를 비교합니다.
4. 동일한 궤적에 PD gain과 중력 보상 여부만 바꾸고 추종 오차를 그립니다.

## 상세 참고 자료


- [A1: MuJoCo Environment Setup — Study Notes](a1_mujoco_basics.ko.md)
- [A2: Forward Kinematics — Study Notes](a2_forward_kinematics.ko.md)
- [A3: Jacobian — Study Notes](a3_jacobian.ko.md)
- [A4: Inverse Kinematics — Study Notes](a4_inverse_kinematics.ko.md)
- [A5: Dynamics Basics — Study Notes](a5_dynamics.ko.md)
- [B1: Trajectory Generation — Study Notes](b1_trajectory_generation.ko.md)
- [B2: PD Controller — Study Notes](b2_pd_controller.ko.md)
- [B3: Full Pipeline — Study Notes](b3_full_pipeline.ko.md)
- [C1: Cartesian Square Drawing — Study Notes](c1_draw_square.ko.md)
