# B1: Trajectory Generation

[English](b1_trajectory_generation.md) · [한국어](b1_trajectory_generation.ko.md) · [简体中文](b1_trajectory_generation.zh-CN.md)

模块提供 cubic/quintic/trapezoidal 关节轨迹、Cartesian minimum-jerk 和多 waypoint 连接。`JointTrajectoryPoint` 包含时间、q、qd、qdd。

```text
s_cubic(u)=3u²-2u³
s_quintic(u)=10u³-15u⁴+6u⁵, u=t/T
q=q_start+(q_end-q_start)s
```

Cubic 两端速度为零，quintic 两端速度/加速度为零。Trapezoidal 包含加速、匀速、减速，短距离可能变为 triangular。多区间应检查重复时间和边界连续性。关节直线不等于 Cartesian 直线。

在 Lab 目录运行 `python src/b1_trajectory_generation.py`，比较 `b1_*_traj.csv` 的边界、数值微分及速度/加速度限制。此阶段生成时间参考；碰撞与 torque 可行性需要独立检查。

[学习笔记](README.zh-CN.md) · [运行指南](../README.zh-CN.md)
