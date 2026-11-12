# A2: Gravity Compensation

[English](a2_gravity_compensation.md) · [한국어](a2_gravity_compensation.ko.md) · [简体中文](a2_gravity_compensation.zh-CN.md)

## Goal

Implement the simplest dynamics-based controller: τ = g(q). The robot should hold any configuration against gravity with minimal drift.

## Files

- Script: `src/a2_gravity_compensation.py`
- Tests: `tests/test_gravity_comp.py`
- Platform: Menagerie UR5e + mounted Robotiq

## Theory

Gravity compensation cancels the gravitational torque at each timestep:

```
τ = g(q)
```

Where g(q) is computed by Pinocchio's `computeGeneralizedGravity()`. This is the foundation for all subsequent controllers — impedance and force control both add gravity compensation as a baseline.

### Why it works

When τ = g(q), the equations of motion reduce to:

```
M(q)q̈ + C(q,q̇)q̇ = 0
```

Joint damping dissipates motion after a perturbation, but gravity compensation has no position-restoring stiffness. It need not return to the initial configuration; that requires PD or impedance feedback. Configuration-dependent inertia also prevents a general single-exponential decay claim.

## Results

Canonical validation on the real UR5e + Robotiq stack:

| Check | Result |
|-------|--------|
| Hold max error | `8.91e-06 rad` |
| Hold criterion | pass (`< 0.01 rad`) |
| Perturbation final speed | `0.0134 rad/s` |
| Perturbation criterion | pass (`< 0.1 rad/s`) |

## How to Run

```bash
python3 src/a2_gravity_compensation.py
```

Outputs: joint position vs time plots, perturbation response plot.
