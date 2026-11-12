# Trajectory processing

[English](03_trajectory_processing.md) · [한국어](03_trajectory_processing.ko.md) · [简体中文](03_trajectory_processing.zh-CN.md)

`shortcut_path()` 选择不相邻 waypoint，仅当同一 checker 验证连接 edge 后才替换中间路径。它保留端点，不能假设安全 node 必然连接成安全 edge。

`parameterize_topp_ra(path,vel_limits,acc_limits,dt)` 返回 `(times,q,qd,qdd)`。拒绝空/非有限路径、非正限制和无效 dt；构造弧长前删除连续重复点。有 TOPP-RA 时参数化插值 spline，否则使用保守 quintic 区间。即使 duration 不是 dt 整数倍，也必须包含精确终点时间。

Quintic 沿 joint-space 直线 edge，区间端点速度/加速度为零。Spline 可能改变几何路径，切角进入障碍。因此 capstone 执行前重新检查每个时间样本的位置、速度、加速度，违规即停止。几何规划成功不能单独证明 spline 安全。

在 `planning_audit.json` 中区分 raw、shortcut、timed 检查。尝试穿过 blocking post 的 shortcut 并确认被拒绝。发布的 quintic 演示不要求 TOPP-RA；实际安装的可选 backend 需要单独运行验证，回归测试也用受控 backend 检查精确终点采样。
