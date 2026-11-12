# 考虑接触的行走

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

本实验承接 Lab 6 的 inverse-dynamics QP，连接平滑足端参考与测量接触，学习落地时如何切换控制任务和接触约束。

## C2 Swing 轨迹

对归一化 Swing 时间 `s ∈ [0, 1]`，水平插值使用 `10s³−15s⁴+6s⁵`，垂直抬脚高度使用 `64h·s³(1−s)³`。两端速度和加速度均为零。在 `smooth_swing.py` 中比较它们与 legacy 正弦高度轨迹的导数。平滑参考本身不保证实际落地冲击更小。

## 接触与控制

结合阅读 `contact_walk.py` 和 Lab 6 的 `locomotion_controller.py`、`wb_id_qp.py`。地面法向力用于判断承重接触；启用 Swing 任务前同步接触状态。Inverse-dynamics QP 使用 ProxQP、足端姿态和 CoM 高度任务，以及接触速度与姿态稳定项。LIPM 高度以地面为基准，而非踝关节 frame。

示例在平地走四个 8 cm 步，single support 为 0.85 s，double support 为 0.35 s，参考抬脚高度为 2.5 cm。足端姿态稳定项假设平地和固定 yaw。

## 练习

1. 按[运行命令](../README.zh-CN.md)运行 smooth 与 `legacy`，保持时间和控制器设置相同。
2. 比较参考轨迹与实际抬脚、落地，结合观察紫色 Swing 曲线、橙色 ZMP 和绿色承重支撑区。
3. 解释为何平滑参考仍可能产生落地冲击。除动画外，还要比较接触力和跟踪误差。

仿真记录足端姿态、接触、关节与力矩限位及 QP 残差。分析差异时可查看[原始数据](../GALLERY.zh-CN.md)。首次接触速度是接触 tick 上的有限差分，并非直接测量的碰撞前速度。
