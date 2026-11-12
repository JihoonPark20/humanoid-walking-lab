# Interpreting experiments

[English](VALIDATION.md) · [한국어](VALIDATION.ko.md) · [简体中文](VALIDATION.zh-CN.md)

Separate three questions when studying a demo:

1. **Is the plan feasible?** Check dynamics, contact constraints and collision clearance. Solver convergence alone is insufficient.
2. **Does the robot track it?** Compare reference and measured motion, loaded contacts, joint limits and torque limits. A feasible reduced-model plan may still be difficult for the full robot.
3. **Is the comparison fair?** Keep the scene, initial state, disturbance and controller settings fixed when comparing one design choice. Retain failures as well as successes.

Use each lab's learning notes for the model assumptions and comparison exercises. Galleries contain optional raw data when a plot or motion needs closer inspection. Reproduce a case with its README command before changing one parameter at a time.

## What the demonstrations assume

- Labs 1–2 teach arm tracking and kinematics; Lab 1's square has collision disabled, and Lab 2's frame-distance heuristic is not mesh collision checking. Lab 3 uses a rigid probe on a low-friction plate.
- Lab 4 uses sampled collision checks. A rejected search or exhausted iteration budget does not prove that no path exists; inspect the returned path and its physical tracking separately.
- Lab 8 animates a reduced centroidal model. Lab 9 predicts CoM, momentum and contact forces; its observer is not an IMU estimator. Lab 10 tracks an offline whole-body plan with feedback.
- Labs 11–13 use flat ground and simulator state. Lab 13 adjusts landing positions while keeping contact order and timing fixed. Compare physical constraints as well as completion; adaptive placement need not improve every metric.

## Checking your changes

Run the relevant lab's tests after modifying its implementation. The complete checks use two environments:

```bash
.venv/bin/python -m pytest lab-*/tests tools/tests -q
.venv-ocp/bin/python -m pytest lab-10-whole-body-optimal-control/tests -q
```

The main environment skips Crocoddyl; the second command covers it. Use the lab's simulation command to inspect behavior after changing a controller. Timing measurements depend on machine load and do not establish a hard real-time guarantee.

[Curriculum](../README.md) · [Setup](SETUP.md)
