# Lab 3: Study notes

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

[Commands and code order](../README.md) · [Shared concepts](../../docs/CONCEPTS.md)

## Read the dynamics equation

```text
M(q) qddot + h(q, qdot) = τ + Jᵀ f_external
h(q, qdot) = C(q, qdot) qdot + g(q)
```

`M` is the mass matrix; `h` combines Coriolis, centrifugal and gravity terms. RNEA computes required torque from position, velocity and acceleration; CRBA computes `M`. Set velocity and acceleration to zero when comparing gravity alone.

`a1_dynamics_fundamentals.py` compares Pinocchio with MuJoCo. The same robot name does not ensure matching payload, inertia, damping or armature. Check models before blaming the solver.

## Gravity compensation and impedance

Gravity compensation reduces position error but does not replace stiffness or damping. Cartesian impedance maps position/velocity error to a virtual spring-damper wrench, then applies `τ=JᵀF`. Rotational tasks also require `SO(3)` error and angular velocity.

Higher stiffness may improve tracking while increasing contact force and oscillation. Compare deflection under external force in `b2_compliance_demo.py`.

## Hybrid position/force control

Track position tangential to the table and regulate force along its normal. Selection matrices avoid imposing strong position and force goals on the same direction.

Check the force-feedback sign/frame in `c1_force_control.py`, then read `c2_line_trace.py`. Compare approach, contact and release phases to explain overshoot.

The representative run traces 50 mm on a plate with μ=0.05 using a closed gripper and rigid massless probe. An actuator holds the fingers closed; the probe is fixed to the tool body. The same complete run checks raw force, orientation, gripper closure and non-probe contacts. See [settings](c1_force_control.md) and [measurements](../media/line_trace_summary.json).

## Exercises and references

Vary payload, impedance stiffness and force target separately. Measure peak force, oscillation and actuator saturation, not only average error.

[Dynamics](a1_dynamics_fundamentals.md) · [Gravity compensation](a2_gravity_compensation.md) · [Impedance](b1_impedance_control.md) · [Hybrid control](c1_force_control.md)
