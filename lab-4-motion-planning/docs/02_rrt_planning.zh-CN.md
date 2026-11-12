# RRT and RRT*

[English](02_rrt_planning.md) · [한국어](02_rrt_planning.ko.md) · [简体中文](02_rrt_planning.zh-CN.md)

`RRTStarPlanner` 在六关节 configuration space 保存 node `(q,parent,cost)`。在关节范围内采样，以 goal bias 概率选择目标。每轮寻找最近 node，最多前进 `step_size`，并验证整条新 edge。

RRT 连接最近 node，找到首个有效目标连接即返回。RRT* 在 `rewire_radius` 内选最便宜且无碰撞的 parent，必要时 rewire 邻居并传播所有后代成本；目标重连也更新后代成本，并搜索到迭代预算结束。成本是 joint-space Euclidean 距离和，不是物理时间或能量。本实现使用固定邻域半径，不保证有限次运行的最优性。

`plan(q_start,q_goal,max_iter=5000,rrt_star=True,seed=42)` 返回 waypoint 列表或 `None`。拒绝错误边界及非有限输入；碰撞/越界端点失败；相同安全端点返回单点路径。失败清除旧 tree。不可通过的墙必须返回 `None`，不能返回未经检查的连接。

Capstone 对必经 task waypoint 求 IK，再使用完整关节范围、共用搜索设置和固定 seed 42 逐段规划，不重试 seed。`planning_validation.py` 独立检查 raw 与 shortcut。[回归测试](../tests/test_planning_safety.py) 使用解析 2D 障碍检查、不可通过墙的反例，以及紫色柱阻挡的真实 UR5e 区间，并检查 rewiring 成本、深层后代更新和多个 seed。

单个成功 seed 不是成功率 benchmark。应在同场景、起终点、限制及预算下重复，报告失败和路径成本。

[多环境评估](GENERALIZATION.zh-CN.md)
