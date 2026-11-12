# Unitree G1 — 关节与 DOF 布局

[English](g1_joint_map.md) · [한국어](g1_joint_map.ko.md) · [简体中文](g1_joint_map.zh-CN.md)

模型：MuJoCo Menagerie **Unitree G1 29-DOF**（`unitree_g1/scene.xml`）

---

## 模型摘要

| 属性 | 数值 |
|----------|-------|
| nq (config dim) | 36 (7 freejoint + 29 hinge) |
| nv (velocity dim) | 35 (6 base + 29 hinge) |
| nu (actuators) | 29 (全部 position 控制, Kp=500) |
| nbody | 31 |
| 总质量 | 33.34 kg |
| Pelvis 高度（keyframe） | 0.79 m |
| CoM 高度（稳定后） | ~0.66 m |

---

## 关节表

| idx | 名称 | 分类 | qpos_adr | 范围 (rad) |
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

## DOF 摘要

| 分类 | DOFs | Lab 5 状态 |
|----------|------|-------------|
| BASE (freejoint) | 6 | 无直接驱动 |
| LEFT_LEG | 6 | **控制** |
| RIGHT_LEG | 6 | **控制** |
| WAIST | 3 | **控制** |
| LEFT_ARM | 7 | 保持 neutral |
| RIGHT_ARM | 7 | 保持 neutral |
| **Total nv** | **35** | |
| **Actuated (nu)** | **29** | |
| **Locomotion (legs+waist)** | **15** | 主动控制 |

---

## Actuator 表

Lab 5 actuator 为 `position` 类型，`Kp=500`、`dampratio=1`。请与 Lab 6 之后的 torque 模型区分。

| idx | Actuator | 分类 | ctrl range |
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
| 15-21 | 左臂 (7 关节) | LEFT_ARM | (保持 neutral) |
| 22-28 | 右臂 (7 关节) | RIGHT_ARM | (保持 neutral) |

---

## Body 表

| idx | 名称 | 父级 | 质量 (kg) |
|-----|------|--------|-----------|
| 0 | world | — | 0.000 |
| 1 | pelvis | world | 3.813 |
| 2 | left_hip_pitch_link | pelvis | 1.350 |
| 3 | left_hip_roll_link | 2 | 1.520 |
| 4 | left_hip_yaw_link | 3 | 1.702 |
| 5 | left_knee_link | 4 | 1.932 |
| 6 | left_ankle_pitch_link | 5 | 0.074 |
| 7 | left_ankle_roll_link | 6 | 0.608 |
| 8-13 | 右腿 (对称) | pelvis | 7.186 |
| 14 | waist_yaw_link | pelvis | 0.214 |
| 15 | waist_roll_link | 14 | 0.086 |
| 16 | torso_link | 15 | 7.818 |
| 17-23 | 左臂 (7 link) | torso | 3.519 |
| 24-30 | 右臂 (7 link) | torso | 3.519 |
| | **TOTAL** | | **33.341** |

---

## qpos 布局 (nq=36)

```
索引   内容
──────  ────────────────────────────────────────
0-2     base 位置 (x, y, z)
3-6     base quaternion (w, x, y, z)  ← MuJoCo 约定
7-12    左腿: hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
13-18   右腿: hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
19-21   waist: yaw, roll, pitch
22-28   左臂: shoulder_pitch/roll/yaw, elbow, wrist_roll/pitch/yaw
29-35   右臂: shoulder_pitch/roll/yaw, elbow, wrist_roll/pitch/yaw
```

## qvel 布局 (nv=35)

```
索引   内容
──────  ────────────────────────────────────────
0-2     base 线速度 (vx, vy, vz)
3-5     base 角速度 (wx, wy, wz)
6-11    左腿 关节速度
12-17   右腿 关节速度
18-20   waist 关节速度
21-27   左臂 关节速度
28-34   右臂 关节速度
```

## ctrl 布局 (nu=29)

```
索引   内容
──────  ────────────────────────────────────────
0-5     左腿: hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
6-11    右腿: (相同顺序)
12-14   waist: yaw, roll, pitch
15-21   左臂: shoulder_pitch/roll/yaw, elbow, wrist_roll/pitch/yaw
22-28   右臂: (相同顺序)
```

---

## 手臂 neutral pose（keyframe "stand"）

行走时手臂保持 keyframe 姿态：

| Joint | 左 | 右 |
|-------|------|-------|
| shoulder_pitch | 0.2 | 0.2 |
| shoulder_roll | 0.2 | -0.2 |
| shoulder_yaw | 0.0 | 0.0 |
| elbow | 1.28 | 1.28 |
| wrist_roll | 0.0 | 0.0 |
| wrist_pitch | 0.0 | 0.0 |
| wrist_yaw | 0.0 | 0.0 |

---

## 运动链

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

## 足部接触几何

每只脚有四个 sphere contact（半径 0.005 m，摩擦 0.6）：
- 后方两个： pos=(-0.05, ±0.025, -0.03)
- 前方两个： pos=(0.12, ±0.03, -0.03)

每只脚前后支撑点间距约 17 cm。

---

## 与 Lab 1–4 的区别

| 属性 | Labs 2–4 (UR5e) | Lab 5 (G1) |
|----------|----------------|------------|
| Base | 固定 base | floating base |
| Total DOF | 6 | 35 (6 base + 29 关节) |
| Actuated | 6 | 29 (15 for locomotion) |
| Mass | ~28 kg | 33.34 kg |
| 主要任务 | 到达目标 | 保持平衡 |
| Gravity | torque 补偿 | 通过地面接触平衡 |
