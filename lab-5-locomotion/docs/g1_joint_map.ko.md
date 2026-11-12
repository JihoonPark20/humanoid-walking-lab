# Unitree G1 — 관절과 DOF 배열

[English](g1_joint_map.md) · [한국어](g1_joint_map.ko.md) · [简体中文](g1_joint_map.zh-CN.md)

모델: MuJoCo Menagerie **Unitree G1 29-DOF** (`unitree_g1/scene.xml`)

---

## 모델 요약

| 항목 | 값 |
|----------|-------|
| nq (config dim) | 36 (7 freejoint + 29 hinge) |
| nv (velocity dim) | 35 (6 base + 29 hinge) |
| nu (actuators) | 29 (모두 position 제어, Kp=500) |
| nbody | 31 |
| 총 질량 | 33.34 kg |
| Pelvis 높이 (keyframe) | 0.79 m |
| CoM 높이 (정착 후) | ~0.66 m |

---

## 관절 표

| idx | 이름 | 분류 | qpos_adr | 범위 (rad) |
|-----|------|----------|----------|-------------|
| 0 | floating_base_joint (free) | BASE | 0 | — |
| 1 | left_hip_pitch_joint | LEFT_LEG | 7 | [-2.531, 2.880] |
| 2 | left_hip_roll_joint | LEFT_LEG | 8 | [-0.524, 2.967] |
| 3 | left_hip_yaw_joint | LEFT_LEG | 9 | [-2.758, 2.758] |
| 4 | left_knee_joint | LEFT_LEG | 10 | [-0.087, 2.880] |
| 5 | left_ankle_pitch_joint | LEFT_LEG | 11 | [-0.873, 0.524] |
| 6 | left_ankle_roll_joint | LEFT_LEG | 12 | [-0.262, 0.262] |
| 7 | right_hip_pitch_joint | RIGHT_LEG | 13 | [-2.531, 2.880] |
| 8 | right_hip_roll_joint | RIGHT_LEG | 14 | [-2.967, 0.524] |
| 9 | right_hip_yaw_joint | RIGHT_LEG | 15 | [-2.758, 2.758] |
| 10 | right_knee_joint | RIGHT_LEG | 16 | [-0.087, 2.880] |
| 11 | right_ankle_pitch_joint | RIGHT_LEG | 17 | [-0.873, 0.524] |
| 12 | right_ankle_roll_joint | RIGHT_LEG | 18 | [-0.262, 0.262] |
| 13 | waist_yaw_joint | WAIST | 19 | [-2.618, 2.618] |
| 14 | waist_roll_joint | WAIST | 20 | [-0.520, 0.520] |
| 15 | waist_pitch_joint | WAIST | 21 | [-0.520, 0.520] |
| 16 | left_shoulder_pitch_joint | LEFT_ARM | 22 | [-3.089, 2.670] |
| 17 | left_shoulder_roll_joint | LEFT_ARM | 23 | [-1.588, 2.252] |
| 18 | left_shoulder_yaw_joint | LEFT_ARM | 24 | [-2.618, 2.618] |
| 19 | left_elbow_joint | LEFT_ARM | 25 | [-1.047, 2.094] |
| 20 | left_wrist_roll_joint | LEFT_ARM | 26 | [-1.972, 1.972] |
| 21 | left_wrist_pitch_joint | LEFT_ARM | 27 | [-1.614, 1.614] |
| 22 | left_wrist_yaw_joint | LEFT_ARM | 28 | [-1.614, 1.614] |
| 23 | right_shoulder_pitch_joint | RIGHT_ARM | 29 | [-3.089, 2.670] |
| 24 | right_shoulder_roll_joint | RIGHT_ARM | 30 | [-2.252, 1.588] |
| 25 | right_shoulder_yaw_joint | RIGHT_ARM | 31 | [-2.618, 2.618] |
| 26 | right_elbow_joint | RIGHT_ARM | 32 | [-1.047, 2.094] |
| 27 | right_wrist_roll_joint | RIGHT_ARM | 33 | [-1.972, 1.972] |
| 28 | right_wrist_pitch_joint | RIGHT_ARM | 34 | [-1.614, 1.614] |
| 29 | right_wrist_yaw_joint | RIGHT_ARM | 35 | [-1.614, 1.614] |

---

## DOF 요약

| 분류 | DOFs | Lab 5 상태 |
|----------|------|-------------|
| BASE (freejoint) | 6 | 직접 구동 없음 |
| LEFT_LEG | 6 | **제어** |
| RIGHT_LEG | 6 | **제어** |
| WAIST | 3 | **제어** |
| LEFT_ARM | 7 | neutral 유지 |
| RIGHT_ARM | 7 | neutral 유지 |
| **Total nv** | **35** | |
| **Actuated (nu)** | **29** | |
| **Locomotion (legs+waist)** | **15** | 능동 제어 |

---

## Actuator 표

Lab 5 actuator는 `position` 형식이며 `Kp=500`, `dampratio=1`입니다. Lab 6 이후 torque 모델과 구분하세요.

| idx | Actuator | 분류 | ctrl range |
|-----|----------|----------|-----------|
| 0 | left_hip_pitch_joint | LEFT_LEG | [-2.531, 2.880] |
| 1 | left_hip_roll_joint | LEFT_LEG | [-0.524, 2.967] |
| 2 | left_hip_yaw_joint | LEFT_LEG | [-2.758, 2.758] |
| 3 | left_knee_joint | LEFT_LEG | [-0.087, 2.880] |
| 4 | left_ankle_pitch_joint | LEFT_LEG | [-0.873, 0.524] |
| 5 | left_ankle_roll_joint | LEFT_LEG | [-0.262, 0.262] |
| 6 | right_hip_pitch_joint | RIGHT_LEG | [-2.531, 2.880] |
| 7 | right_hip_roll_joint | RIGHT_LEG | [-2.967, 0.524] |
| 8 | right_hip_yaw_joint | RIGHT_LEG | [-2.758, 2.758] |
| 9 | right_knee_joint | RIGHT_LEG | [-0.087, 2.880] |
| 10 | right_ankle_pitch_joint | RIGHT_LEG | [-0.873, 0.524] |
| 11 | right_ankle_roll_joint | RIGHT_LEG | [-0.262, 0.262] |
| 12 | waist_yaw_joint | WAIST | [-2.618, 2.618] |
| 13 | waist_roll_joint | WAIST | [-0.520, 0.520] |
| 14 | waist_pitch_joint | WAIST | [-0.520, 0.520] |
| 15-21 | 왼팔 (7 관절) | LEFT_ARM | (neutral 유지) |
| 22-28 | 오른팔 (7 관절) | RIGHT_ARM | (neutral 유지) |

---

## Body 표

| idx | 이름 | 부모 | 질량 (kg) |
|-----|------|--------|-----------|
| 0 | world | — | 0.000 |
| 1 | pelvis | world | 3.813 |
| 2 | left_hip_pitch_link | pelvis | 1.350 |
| 3 | left_hip_roll_link | 2 | 1.520 |
| 4 | left_hip_yaw_link | 3 | 1.702 |
| 5 | left_knee_link | 4 | 1.932 |
| 6 | left_ankle_pitch_link | 5 | 0.074 |
| 7 | left_ankle_roll_link | 6 | 0.608 |
| 8-13 | 오른쪽 다리 (대칭) | pelvis | 7.186 |
| 14 | waist_yaw_link | pelvis | 0.214 |
| 15 | waist_roll_link | 14 | 0.086 |
| 16 | torso_link | 15 | 7.818 |
| 17-23 | 왼팔 (7 link) | torso | 3.519 |
| 24-30 | 오른팔 (7 link) | torso | 3.519 |
| | **TOTAL** | | **33.341** |

---

## qpos 배열 (nq=36)

```
인덱스   내용
──────  ────────────────────────────────────────
0-2     base 위치 (x, y, z)
3-6     base quaternion (w, x, y, z)  ← MuJoCo 순서
7-12    왼쪽 다리: hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
13-18   오른쪽 다리: hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
19-21   waist: yaw, roll, pitch
22-28   왼팔: shoulder_pitch/roll/yaw, elbow, wrist_roll/pitch/yaw
29-35   오른팔: shoulder_pitch/roll/yaw, elbow, wrist_roll/pitch/yaw
```

## qvel 배열 (nv=35)

```
인덱스   내용
──────  ────────────────────────────────────────
0-2     base 선속도 (vx, vy, vz)
3-5     base 각속도 (wx, wy, wz)
6-11    왼쪽 다리 관절 속도
12-17   오른쪽 다리 관절 속도
18-20   waist 관절 속도
21-27   왼팔 관절 속도
28-34   오른팔 관절 속도
```

## ctrl 배열 (nu=29)

```
인덱스   내용
──────  ────────────────────────────────────────
0-5     왼쪽 다리: hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
6-11    오른쪽 다리: (동일 순서)
12-14   waist: yaw, roll, pitch
15-21   왼팔: shoulder_pitch/roll/yaw, elbow, wrist_roll/pitch/yaw
22-28   오른팔: (동일 순서)
```

---

## 팔 neutral pose (keyframe "stand")

보행 중 팔은 keyframe 자세로 유지합니다:

| Joint | 왼쪽 | 오른쪽 |
|-------|------|-------|
| shoulder_pitch | 0.2 | 0.2 |
| shoulder_roll | 0.2 | -0.2 |
| shoulder_yaw | 0.0 | 0.0 |
| elbow | 1.28 | 1.28 |
| wrist_roll | 0.0 | 0.0 |
| wrist_pitch | 0.0 | 0.0 |
| wrist_yaw | 0.0 | 0.0 |

---

## Kinematic chain

```
world
└── pelvis (freejoint, z=0.793m, 3.813 kg)
    ├── left_hip_pitch_link → left_hip_roll_link → left_hip_yaw_link
    │   → left_knee_link → left_ankle_pitch_link → left_ankle_roll_link (4 foot spheres)
    ├── right_hip_pitch_link → ... → right_ankle_roll_link (4 foot spheres)
    └── waist_yaw_link → waist_roll_link → torso_link (7.818 kg)
        ├── left_shoulder_pitch_link → ... → left_wrist_yaw_link → left_rubber_hand
        └── right_shoulder_pitch_link → ... → right_wrist_yaw_link → right_rubber_hand
```

---

## 발 접촉 형상

각 발에는 sphere contact 4개가 있습니다 (반경 0.005 m, 마찰 0.6):
- 뒤쪽 2개: pos=(-0.05, ±0.025, -0.03)
- 앞쪽 2개: pos=(0.12, ±0.03, -0.03)

발 앞뒤 지지점 간 거리는 약 17 cm입니다.

---

## Lab 1–4와의 차이

| 항목 | Labs 2–4 (UR5e) | Lab 5 (G1) |
|----------|----------------|------------|
| Base | 고정 base | floating base |
| Total DOF | 6 | 35 (6 base + 29 관절) |
| Actuated | 6 | 29 (15 for locomotion) |
| Mass | ~28 kg | 33.34 kg |
| 주요 과제 | 목표 도달 | 균형 유지 |
| Gravity | torque 보상 | 지면 접촉으로 균형 유지 |
