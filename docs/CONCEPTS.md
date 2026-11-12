# Common concepts

[English](CONCEPTS.md) · [한국어](CONCEPTS.ko.md) · [简体中文](CONCEPTS.zh-CN.md)

| Term | Meaning | First lab |
|---|---|---|
| DOF | Independent motion coordinate | 1 |
| FK / IK | Joint state → end-effector pose / target pose → joint state | 1 |
| Jacobian | Differential map from joint to task velocity | 1 |
| Singularity | Configuration where Jacobian rank drops and motion directions are lost | 1 |
| DLS | Damped Least Squares regularizes large IK updates near singularities | 1 |
| Pose / SE(3) | Rigid transform combining 3D position and rotation | 2 |
| RNEA / CRBA | Inverse dynamics / mass-matrix algorithms | 3 |
| Impedance | Desired relation between position/velocity error and force | 3 |
| Wrench | Three forces and three moments | 3 |
| Configuration space | Coordinates representing robot configurations | 4 |
| RRT / RRT* | Sampling-based collision-free path search | 4 |
| TOPP-RA | Path timing under velocity/acceleration constraints | 4 |
| Floating base | Unfixed, directly unactuated base coordinates | 5 |
| CoM / CoP | Center of mass / center of contact pressure | 5–6 |
| ZMP | Ground point with zero tipping moment | 5 |
| LIPM / DCM | Linear inverted pendulum / divergent component of CoM position and velocity | 5–6 |
| QP | Quadratic objective with linear constraints | 6 |

## MuJoCo state

`MjModel` stores links, joints, geoms, actuators and physical parameters. `MjData` stores current state and computed quantities.

- `qpos`: generalized position; freejoint quaternion means its size may differ from `qvel`.
- `qvel`: generalized velocity.
- `ctrl`: actuator input; depending on the model, torque, position target, etc.
- `body`: frame and mass/inertia reference.
- `geom`: collision and visual shape.
- `site`: reference point such as a tool tip or sensor.
- `mj_forward`: update derived kinematics/contact quantities at the current state.
- `mj_step`: compute dynamics and advance simulation time.

FK cross-validation must set both models to the **same state**. Stepping only one model mixes timing error with model mismatch.

## Units and frames

Use m, s, kg, rad, N and N·m internally; distinguish plot labels in mm/degrees. For every vector, identify its world, base, tool or contact frame.

[Curriculum](../README.md)
