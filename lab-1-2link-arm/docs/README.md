# Lab 1: Study notes

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

[Commands and code order](../README.md) · [Shared concepts](../../docs/CONCEPTS.md)

## Connect the core concepts

Forward Kinematics maps joint angles `q` to endpoint position `x`. For the planar two-link arm:

```text
x = L1 cos(q1) + L2 cos(q1 + q2)
y = L1 sin(q1) + L2 sin(q1 + q2)
```

Compare `fk_endeffector` and `fk_homogeneous` in `a2_forward_kinematics.py`. They describe the same position; homogeneous transforms generalize to longer chains. Match the link lengths and any MuJoCo endpoint-site offset before comparing results.

The Jacobian expresses `xdot = J(q) qdot`. Differentiate FK and check it independently by finite differences. A straight or fully folded arm loses one motion direction at a singularity.

Inverse Kinematics maps the target to joint angles. Analytical IK gives elbow-up/down branches. Numerical IK converts the current FK error into a joint increment:

```text
DLS: Δq = α Jᵀ (J Jᵀ + λ² I)⁻¹ (x_target − x)
```

Larger `λ` moderates increments near singularities but can slow convergence. More iterations cannot make an unreachable target reachable.

## Separate trajectory generation from control

The generator produces `q_des`, `qd_des`, `qdd_des` over time. Cubic interpolation specifies endpoint position/velocity; quintic also specifies acceleration. A straight joint-space path is generally not Cartesian straight.

The controller compares desired and actual position/velocity:

```text
PD + gravity: τ = Kp(q_des − q) + Kd(qd_des − qd) + g(q)
Computed torque: τ = M(q)[qdd_des + Kp e + Kd edot] + h(q, qd)
```

Distinguish the analytical plant in `b2_pd_controller.py` from MuJoCo in `c1_draw_square.py`. `qfrc_bias` generally contains velocity-dependent bias as well as gravity.

## Exercises

1. Calculate FK by hand at `q1=q2=0`.
2. Plot both analytical IK branches for one target.
3. Compare pseudoinverse and DLS iterations/errors near a singularity.
4. Change only PD gains or gravity compensation for the same trajectory.

## References

[A1 MuJoCo](a1_mujoco_basics.md) · [A2 FK](a2_forward_kinematics.md) · [A3 Jacobian](a3_jacobian.md) · [A4 IK](a4_inverse_kinematics.md) · [A5 Dynamics](a5_dynamics.md) · [B1 Trajectories](b1_trajectory_generation.md) · [B2 PD](b2_pd_controller.md) · [B3 Pipeline](b3_full_pipeline.md) · [C1 Square](c1_draw_square.md)
