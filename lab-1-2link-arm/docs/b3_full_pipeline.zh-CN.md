# B3: Full Pipeline — Study Notes

[English](b3_full_pipeline.md) · [한국어](b3_full_pipeline.ko.md) · [简体中文](b3_full_pipeline.zh-CN.md)

连接 `target → IK → trajectory → PD → plant → metrics/log`。`b3_full_pipeline.py` 包含 waypoint 往返、圆跟踪及 workspace 边缘 singularity 对比。

Waypoint 示例 `(0.20,0.30) → (0.40,0.10) → (0.20,0.30)` 使用 analytic IK 与 quintic 区间。这里 pick-and-place 是目标点往返，并非真实物体抓取验证。圆轨迹逐样本求 IK；边界实验比较 pinv/DLS 的失败与误差。注意区分默认 B2 plant 与实际 MuJoCo C1。

在 Lab 目录运行 `python src/b3_full_pipeline.py`。主要输出为 `b3_pick_place_log.csv`、`b3_circle_log.csv`、`b3_singularity_log.csv`、`b3_metrics.csv`；plot/GIF 可选。除跟踪误差外，还应检查 IK 失败与关节连续性。

[学习笔记](README.zh-CN.md) · [运行指南](../README.zh-CN.md)
