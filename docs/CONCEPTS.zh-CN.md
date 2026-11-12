# 共通知识

[English](CONCEPTS.md) · [한국어](CONCEPTS.ko.md) · [简体中文](CONCEPTS.zh-CN.md)

| 术语 | 含义 | 首次实验 |
|---|---|---|
| DOF | 独立运动自由度 | 1 |
| FK / IK | 关节状态 → 末端 pose / 目标 pose → 关节状态 | 1 |
| Jacobian | 关节速度到 task-space 速度的微分映射 | 1 |
| Singularity | Jacobian 降秩、失去部分运动方向的构型 | 1 |
| DLS | Damped Least Squares，抑制奇异点附近过大的 IK 更新 | 1 |
| Pose / SE(3) | 三维位置与旋转组成的刚体变换 | 2 |
| RNEA / CRBA | Inverse dynamics / 质量矩阵算法 | 3 |
| Impedance | 位置/速度误差与力之间的期望关系 | 3 |
| Wrench | 三个力分量和三个力矩分量 | 3 |
| Configuration space | 表示机器人构型的坐标空间 | 4 |
| RRT / RRT* | 基于采样的无碰撞路径搜索 | 4 |
| TOPP-RA | 满足速度/加速度限制的路径时间参数化 | 4 |
| Floating base | 未固定且不能直接驱动的 base 自由度 | 5 |
| CoM / CoP | 质心 / 接触压力中心 | 5–6 |
| ZMP | 倾覆力矩为零的地面点 | 5 |
| LIPM / DCM | 线性倒立摆 / 由 CoM 位置和速度定义的发散分量 | 5–6 |
| QP | 二次目标函数和线性约束的优化问题 | 6 |

## MuJoCo 状态

`MjModel` 保存 link、joint、geom、actuator 及物理参数；`MjData` 保存当前状态和计算结果。

- `qpos`：广义位置；freejoint quaternion 使其长度可能不同于 `qvel`。
- `qvel`：广义速度。
- `ctrl`：actuator 输入，含义随模型改变，例如 torque 或位置目标。
- `body`：坐标系及质量/惯量基准。
- `geom`：碰撞与显示形状。
- `site`：末端、传感器等参考点。
- `mj_forward`：在当前状态更新运动学、接触等派生量。
- `mj_step`：计算动力学并推进仿真时间。

FK 交叉验证必须在两个模型中设置**相同状态**。只推进其中一个模型会混合时间误差与模型误差。

## 单位与坐标系

内部使用 m、s、kg、rad、N、N·m，注意图中 mm/degree 标签的转换。每个向量都应明确使用 world、base、tool 还是 contact frame。

[课程目录](../README.zh-CN.md)
