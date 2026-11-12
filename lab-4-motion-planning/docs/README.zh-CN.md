# Lab 4: Motion planning

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Path 定义经过的关节构型，trajectory 还定义经过时间。无碰撞路径仍可能因速度过快而无法跟踪。流程为 IK → RRT* → shortcut → 时间参数化 → 独立检查 → torque control → MuJoCo。

绕障场景包含紫色 blocking post。前两个中间 task pose 之间的直线插值会与它碰撞；移除障碍物后，同一插值无碰撞。本任务要求经过中间 waypoint，并不声称全局起点到终点的直线受阻。

依次阅读[碰撞检查](01_collision_checking.zh-CN.md)、[RRT/RRT*](02_rrt_planning.zh-CN.md)、[轨迹处理](03_trajectory_processing.zh-CN.md)和[执行](04_trajectory_execution.zh-CN.md)。同时比较不同 seed 的路径长度、clearance 与跟踪误差；降低速度限制，观察时间与误差。比较 planner 时固定场景、tool 几何与判定阈值。

[运行命令](../README.zh-CN.md) · [规划验证](../media/planning_audit.json) · [执行结果](../media/slalom_summary.json)

[多环境评估](GENERALIZATION.zh-CN.md)
