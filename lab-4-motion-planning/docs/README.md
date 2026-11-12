# Lab 4: Motion planning

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

A path specifies configurations; a trajectory adds timing. A collision-free path can still be too fast to track. The pipeline is IK → RRT* → shortcut → time parameterization → independent audit → torque control → MuJoCo.

The slalom includes a purple blocking post. Direct interpolation between the first two intermediate task poses collides with it; removing the obstacles makes the same interpolation collision-free. The task requires those intermediate waypoints. This is not a claim that the global start-to-final straight line is blocked.

Read the [collision checker](01_collision_checking.md), [RRT/RRT*](02_rrt_planning.md), [trajectory processing](03_trajectory_processing.md) and [execution](04_trajectory_execution.md). Compare seeds, path length, clearance and tracking error together. Lower velocity limits and observe execution time and error. Keep the scene, tool geometry and acceptance thresholds fixed when comparing planners.

[Commands](../README.md) · [Planning audit](../media/planning_audit.json) · [Executed result](../media/slalom_summary.json)

[Multi-environment evaluation](GENERALIZATION.md)
