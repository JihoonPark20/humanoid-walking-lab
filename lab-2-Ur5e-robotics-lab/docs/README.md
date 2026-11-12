# Lab 2: Study notes

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

[Commands and code order](../README.md) · [Shared concepts](../../docs/CONCEPTS.md)

## From 2D to 3D

The UR5e endpoint pose is an `SE(3)` transform containing position `p` and rotation `R`. FK multiplies joint transforms; the 6D Jacobian maps joint rates to linear/angular velocity. Matching position alone does not determine tool orientation.

Start with joint names and model paths in `ur5e_common.py`, then read `a2_forward_kinematics.py`. MuJoCo body IDs and Pinocchio frame IDs are not interchangeable. Match flange versus tool-tip locations.

## Four IK checks

1. Are target and current pose expressed in the same frame?
2. Do the error and Jacobian use compatible frame conventions?
3. Is rotation error computed geometrically instead of subtracting Euler angles?
4. Does a converged solution respect joint limits and collision constraints?

This repository uses Pinocchio spatial vectors in `[linear, angular]` order. Do not mix `LOCAL_WORLD_ALIGNED` and body-local Jacobians. Update frame placements after FK.

## Position-servo interpretation

The Menagerie servo implements the following relationship; inspect actual gain, bias and gear settings:

```text
τ = Kp(ctrl − q) − Kd qdot
ctrl = q_des + τ_feedforward/Kp + Kd qd_des/Kp
```

Using only `ctrl=q_des` requires error to support gravity and introduces velocity-tracking lag. Find the feedforward term in `c3_draw_cube.py`; separate trajectory generation, servo conversion and physics integration.

## Exercises

Compare both engines' FK at the same configuration. Perturb one joint and compare the Jacobian prediction. Change DLS damping and initial configuration to explore different IK solutions.

## References

[Environment](a1_environment_setup.md) · [FK](a2_forward_kinematics.md) · [Jacobian](a3_jacobian.md) · [IK](a4_inverse_kinematics.md) · [Dynamics](a5_dynamics.md) · [Trajectories](b1_trajectory_generation.md) · [Control hierarchy](b2_control_hierarchy.md) · [Constraints](b3_constraints.md) · [Pipeline](c1_full_pipeline.md) · [Cube](c3_draw_cube.md) · [Review questions](interview_cheatsheet.md)
