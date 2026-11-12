# A2: Forward Kinematics

[English](a2_forward_kinematics.md) · [한국어](a2_forward_kinematics.ko.md) · [简体中文](a2_forward_kinematics.zh-CN.md)

UR5e FK 从六关节 q 计算含位置 p 和旋转 R 的 `T=[R,p;0,1]∈SE(3)`。三维轴和固定 offset 需要按顺序组合 link 变换，不能仅靠平面三角关系。

DH 的 θ 是关节旋转，d 是轴向 offset，a 是 link 长度，α 是相邻轴 twist。但仓库 `ur5e_common.forward_chain()` 并非直接相乘一张标准 DH 表：与实际模型对齐的固定变换、joint rotation、tool offset 顺序才是关键。请区分 `a2_forward_kinematics.py` 中的教学 DH 计算。

变换最后一列表示位置，R 的各列表示变换后的坐标轴。RPY 便于阅读，但有 singularity 与 convention 问题；比较旋转应使用 R 或 SO(3) 误差。不要把 flange 与 tool tip 视为同一点。

```text
T_world_tool = T_world_base × Π_i(T_fixed_i T_joint_i(q_i)) × T_tool
```

在 Lab 目录运行 `python src/a2_forward_kinematics.py`。通过 `a2_fk_validation.csv` 检查直接 chain、Pinocchio、MuJoCo 是否使用同一 q、frame、tool offset。绘制 home/ready/over-table 姿态，理解 joint/task space 关系。

URDF 表达 joint axis、固定 origin、inertia 和 geometry；DH 是表达 kinematic chain 的参数惯例。任何一种都不会自动正确，必须在一致坐标系中描述同一机器人。FK 一致不代表 dynamics、碰撞或 IK 收敛已经验证。

[学习笔记](README.zh-CN.md) · [运行指南](../README.zh-CN.md)
