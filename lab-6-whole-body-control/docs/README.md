# Lab 6: Study notes

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

[Commands and code order](../README.md) · [Shared concepts](../../docs/CONCEPTS.md)

## Put balance into the dynamics constraints

A floating base has no direct joint torque. Contact forces must satisfy the base rows:

```text
M(q) qddot + h(q, qdot) = Sᵀ τ + Jcᵀ f
Jc qddot + Jcdot qdot = 0
```

`wb_id_qp.py` optimizes acceleration and contact wrench while respecting base dynamics, stance acceleration, friction, CoP and torque limits. Follow each task's Jacobian and `Jdot qdot` term in `wb_tasks.py`.

Velocity-level IK/QP does not directly choose the required support forces. It needs a separate force-aware layer for this balancing task; kinematic QP remains useful in such architectures.

## Read walking through DCM

```text
ξ = c + cdot/ω
ξdot = ω(ξ − p)
```

DCM combines CoM position and velocity. `dcm_planner.py` builds references from footsteps and ZMP; `locomotion_controller.py` corrects them from measured state. This differs from commanding a stationary CoM over one foot.

## Add hand tasks carefully

Arm and payload motion affect centroidal momentum and CoM. Increasing hand weights can conflict with balance. Compare momentum tasks, fixed carry postures and moving hand references.

A QP stopped at its iteration limit is not a converged solution merely because it returns numbers. Inspect scaling, iterations, residuals and runtime together.

## Models and contacts

CoP regions must match the sole geometry and wrench reference frame. An ankle frame above the ground requires accounting for horizontal-force moments. After adding furniture, inspect contact pairs before tuning gains.

When enabling a weld, preserve the current relative pose. At release, inspect the object's target and actual resting pose rather than just the hand. A published success does not establish success for a new object arrangement.
