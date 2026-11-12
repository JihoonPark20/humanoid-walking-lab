# Collision checking

[English](01_collision_checking.md) · [한국어](01_collision_checking.ko.md) · [简体中文](01_collision_checking.zh-CN.md)

`collision_checker.py` 使用与执行相同的 MuJoCo 场景。`is_collision_free(q)` 检查六关节有限输入、实际关节范围、机器人/环境接触与选定 self-collision pair。夹爪内部和模型特定相邻 link pair 被排除，因此不能证明任意自碰撞都不存在。

`is_path_free(a,b,resolution)` 检查插值构型，而不只检查端点。有限分辨率可能漏过薄障碍。`compute_min_obstacle_distance(q)` 使用 collision geometry 加速规划；`scene_clearance.py` 独立检查 collision shape 与可见 mesh，避免遗漏超出 collision proxy 的可见部分。

Capstone 以 32 mm 裕量规划，独立验证要求至少 30 mm。Raw/shortcut edge 按 joint-space L2 距离不超过 0.005 rad 采样，检查每个时间参考及物理执行每个 1 ms tick。这是密集离散检查，不是连续时间无碰撞的形式化证明。

`planning_validation.py` 的对照要求：有障碍时实际碰撞，保留桌子与机器人但移除障碍后无碰撞。原本空旷的环境或仅安全裕量不足的路径不能通过该条件。

练习：阅读 [planning_audit.json](../media/planning_audit.json) 中受阻的必经区间，并与实际零接触执行比较。不能通过禁用障碍物 contact 来使演示成功。
