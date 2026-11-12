# C1: Integrated Pick-and-Place Pipeline

[English](c1_full_pipeline.md) · [한국어](c1_full_pipeline.ko.md) · [简体中文](c1_full_pipeline.zh-CN.md)

将 IK、轨迹、控制、约束和指标连成执行流程。`c1_full_pipeline.py` 包含 waypoint 往返、圆、多 waypoint 与接近 singularity 的示例，对比 reference 和 plant 跟踪。Pick-and-place 名称本身不代表物体 grasp 成功。

用 FK 检查每个 IK 解，检查插值边界与速度/torque 限制，分别记录 local IK 失败和 controller tracking 失败。Link frame 距离碰撞启发式与实际 mesh 检查不同。

在 Lab 目录运行 `python src/c1_full_pipeline.py`，分析 `c1_metrics_dashboard.csv` 和 `c1_*_log.csv` 中的 task 误差、joint 状态、约束激活。实际 MuJoCo 代表演示为 C3，不能将同一成功判定自动套用到所有模块。

[学习笔记](README.zh-CN.md) · [运行指南](../README.zh-CN.md)
