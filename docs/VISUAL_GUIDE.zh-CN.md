# 如何阅读行走演示

[English](VISUAL_GUIDE.md) · [한국어](VISUAL_GUIDE.ko.md) · [简体中文](VISUAL_GUIDE.zh-CN.md)

Lab 5–10 同时展示 3D 视图与带长度标尺的地面俯视图。地面圆环表示 CoM 的垂直投影，不是接触点。俯视图跟随 CoM 移动，Lab 7 则以落脚计划为中心；上方为 +x，左方为 +y。请用 10 cm 标尺比较距离，不要跨实验比较像素大小。

| Lab | 观察重点 |
|---|---|
| 5 · 平衡 | 双脚保持着地，CoM 向目标移动。 |
| 6 · DCM/ZMP | 观察单脚与双脚支撑的切换，比较控制器 DCM、目标 DCM 与接触 ZMP。 |
| 7 · 离线优化 | 比较灰色初始落脚点与紫色优化落脚点及 CoM。执行前已确定计划，青色记录实际运动。 |
| 8 · Centroidal dynamics | 观察浮动质量、计划接触力、CoM 高度与角动量。这是简化模型动画，不是全身仿真。 |
| 9 · MPC/RTI | 每次控制器更新后，紫色 CoM 预测随之刷新。在红色扰动前、中、后与灰色原始轨迹比较。时间标记相对于当前帧。 |
| 10 · Whole-body OCP | 紫色 ghost 表示 +0.25/+0.50 s 的优化关节姿态，在 configuration manifold 上插值。实体机器人在 MuJoCo 中通过反馈跟踪；超出计划终点的 ghost 不再显示。 |

**颜色：** 青色 = 实际 CoM（Lab 8 为计划质量位置）；白色 = 参考；紫色 = 计划/预测（Lab 6 为 DCM）；橙色 = 接触 ZMP；绿色 = 承重支撑区域或计划力；红色 = 外部扰动。每个视频均有图例。

绿色区域是承重足端与地面接触点的 convex hull，因此脚卸载时可能缩小。橙色 ZMP 是平地接触位置按垂直载荷加权的结果；无承重接触时不显示。Lab 8 显示预定接触区域及计划力，箭头比例为 **1.5 mm/N**。Lab 9 的扰动箭头采用 **60 mm/N** 以便观察。球体大小和 ghost 透明度不表示不确定性。

Centroidal MPC 预测 CoM、动量与接触力，不预测关节姿态。因此 Lab 9 的半透明球体表示 CoM 预测；完整机器人 ghost 仅用于具有真实关节计划的 Lab 10。Lab 10 是离线 OCP 加反馈，不是全身 MPC。

标记仅添加到渲染场景，ghost 使用独立的 simulator data，不改变被控机器人的状态。各 Lab 的练习与测量结果链接提供原始分辨率视频及同次运行的通过结果。运行 README 的录制命令后，使用 `python tools/build_gallery.py` 更新 GIF。

[← Curriculum](../README.zh-CN.md)
