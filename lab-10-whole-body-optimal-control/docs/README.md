# W5: G1 Whole-body optimal control

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

## Torque regularization and tracking

The recommended FDDP profile plans with `control_weight=3.0`; blended/joint-PD profiles use `0.1`. Changing the cost requires solving the OCP again and changes both nominal torque and feedback gains. Metadata records the weight. Objective values from different weights do not rank physical performance. Use `--control-weight` for a comparison on the same cost; arbitrary cost/gain combinations are not guaranteed to work.

Optimization defaults to 10 ms; MuJoCo integration and tracking use 1 ms. The default policy is FDDP. The optional blend is:

```text
e = state.diff(x_nominal, x_actual)
du_fddp = -K e
du_pd = -400 e_joint_position - 20 e_joint_velocity
u = u_ff + alpha du_fddp + (1-alpha) du_pd
```

`blended` defaults to `alpha=0.1`; `fddp` uses all feedback gains, `joint-pd` only joint PD. Blending experimentally reduces large local-model corrections while maintaining joint tracking. Base feedback remains through FDDP joint torques. It adds neither contact estimation nor online replanning and is not a stability proof.

Acceptance requires a converged, independently validated plan, complete execution, no fall/flight/clipping and final foot gap below 3 cm. Default one/two-step regression tests additionally require foot gap below 2 cm. Actual state is never overwritten by nominal state. The validation matrix contains three recommended policies × one/two steps × 10/20 ms = 12 profiles.

## Model consistency

Pinocchio 4.0's MJCF loader already fills `model.armature`; adding it again to the Crocoddyl action double-counts it. `audit_mass_matrices()` aligns free-flyer velocity coordinates and compares MuJoCo, Pinocchio and contact/impact action mass matrices. Friction losses and compliant contacts still differ from rigid-contact planning.

## Implementation

1. Load the Lab 6 torque G1 and Pinocchio model; align floating-base state and actuator bounds.
2. Build contact dynamics and impact actions for stepping and double-support rest.
3. Use Box-FDDP for torque boxes and state/contact costs.
4. Compare manifold feedback rollout with independent MuJoCo execution.

- `whole_body_ocp.py`: models, shooting problem, Box-FDDP, states/controls/gains, independent checks.
- `whole_body_demo.py`: CLI, metrics, plots, optional torque replay and video.
- `whole_body_tracking.py`: FDDP/joint-PD/blended policies and settings validation.
- `validate_tracking_scenarios.py`: 12 physical tracking profiles and explicit planning weights.

Torque boxes are solver constraints. Joint and wrench barrier costs alone do not guarantee feasibility: inspect their residuals. Contact-implicit scheduling and obstacle avoidance are outside this lab.

## State, contacts and feedback

`x=(q,v)`, with a floating-base quaternion: `nq=36`, `nv=35`, stored state dimension 71, tangent error dimension 70. Costs and feedback use manifold differences.

```text
M(q) v_dot + h(q,v) = S.T u + J(q).T lambda
J(q) v_dot + J_dot(q,v) v = 0
u = u_nominal - K * state.diff(x_nominal, x_measured)
```

Contact actions impose stance acceleration constraints. Impact actions keep position and project velocity without advancing time. MuJoCo receives no such velocity reset; its contact dynamics determine the velocity change. Reduced/contact-model feasibility and compliant-contact execution must be reported separately.

See the [run guide](../README.md) for package versions, model loading and results.
