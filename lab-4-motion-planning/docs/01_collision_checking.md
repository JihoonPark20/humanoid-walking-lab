# Collision checking

[English](01_collision_checking.md) · [한국어](01_collision_checking.ko.md) · [简体中文](01_collision_checking.zh-CN.md)

`collision_checker.py` uses the same MuJoCo scene as execution. `is_collision_free(q)` checks finite six-joint input, actual joint limits, robot/environment contacts and selected self-collision pairs. Internal gripper pairs and model-specific adjacent links are excluded; this policy does not prove arbitrary self-collision freedom.

`is_path_free(a,b,resolution)` checks interpolated configurations, not just endpoints. A finite resolution can miss thin obstacles. `compute_min_obstacle_distance(q)` uses collision geometry for efficient planning; `scene_clearance.py` independently checks robot collision shapes and visible meshes against obstacles, since a visible mesh can protrude beyond its collision proxy.

The capstone plans with a 32 mm margin and accepts only paths with at least 30 mm in the independent audit. Raw and shortcut edges are sampled at at most 0.005 rad in joint-space L2 distance. Every timed reference and every 1 ms physical tick is checked. These are dense discrete checks, not a formal continuous collision certificate.

The direct-path control in `planning_validation.py` requires physical collision with an obstacle and a collision-free replay after removing obstacles, retaining the table and robot. A clear environment or merely insufficient safety margin cannot pass this gate.

Exercise: inspect the colliding required segment in [planning_audit.json](../media/planning_audit.json), then compare its blocked samples with the zero-contact executed result. Never disable obstacle contact to make a planning demo pass.
