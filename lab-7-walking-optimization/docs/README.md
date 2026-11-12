# Lab 7: Walking optimization and module connections

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

This lab implements W0 baseline walking, W1 offline LIPM optimization, and W2 footstep/timing candidate comparison. W3 Centroidal optimization, W4 MPC and W5 Whole-body optimal control are Labs 8–10.

## Implementation and data flow

1. Define exact piecewise-constant-jerk dynamics, flat fixed-yaw feet and single/double support; test integration and boundary continuity.
2. Build a QP with initial/terminal rest conditions, support constraints and ZMP/jerk costs. Validate dynamics and within-interval ZMP extrema independently of solver status.
3. Optimize foot positions with step length, width and landing bounds. Record feasibility and cost for every timing candidate.
4. Export NPZ/CSV/JSON/plots through the CLI.
5. Track the plan with Lab 6 torque dynamics, Whole-body QP and contact handling; compare with baseline DCM walking. Planning and physical execution have separate acceptance gates.

| Module | Role |
|---|---|
| `walking_model.py` | Settings, contacts, footsteps, exact jerk dynamics |
| `lipm_optimizer.py` | State/jerk/footstep QP, infeasibility, continuous-interval checks |
| `walking_artifacts.py` | NPZ I/O, CSV/JSON, plots and animation |
| `offline_walk.py` | W1 fixed-foot and W2 position/timing CLI |
| `g1_walk.py` | Lab 6 adapter, baseline/optimized G1 execution and measurements |

Initial state + feet + contact times + distance → `WalkingSchedule` → sparse QP → `WalkingPlan` → independent checks → G1 reference adapter → Whole-body QP → torque → MuJoCo.

Contact order is fixed. During double support, a small rectangle follows a prescribed convex combination of the two foot centers. It lies conservatively inside their convex hull for equal-size, fixed-yaw feet. Its center depends affinely on foot coordinates, so each W2 timing candidate is still a convex QP. Timing is selected by enumerating candidates, not continuous optimization.

## Interfaces

- `build_schedule(config, initial_feet=None)` → `WalkingSchedule`
- `solve_walk(schedule, initial_state=None, optimize_feet=False, landing_bounds=None)` → `WalkingPlan`
- `WalkingPlan.sample(t)` → position, velocity, acceleration, ZMP and DCM reference
- `validate_plan(plan)` → dynamics residuals, continuous support violations, footsteps and boundaries

The optimizer needs no external robot model. G1 execution uses the Menagerie MJCF and Lab 5/6 loaders. Pinocchio computes FK, CoM, Jacobians and dynamics; MuJoCo integrates physical motion and contact.

## W1 equations and variables

For each horizontal axis, use `x=[c,v,a]`, constant jerk `u` over interval `h`. These are exact polynomial integrals:

```text
c[k+1] = c[k] + h v[k] + h²/2 a[k] + h³/6 u[k]
v[k+1] = v[k] + h a[k] + h²/2 u[k]
a[k+1] = a[k] + h u[k]
ω² = g / z_com
ZMP = c − a / ω²
DCM ξ = c + v / ω
ξ_dot = v + a / ω = ω (ξ − ZMP)
```

Read `jerk_matrices()`, `polynomial_state()` and `WalkingPlan.sample()`. Linear interpolation of states can break the ZMP/DCM relationships. QP variables are all node x/y states, interval x/y jerks and foot x/y coordinates. W1 fixes feet by equality constraints; W2 fixes only the first two feet.

```text
J = w_zmp ∫ ||ZMP(t) − support_center(t)||² dt
  + w_jerk ∫ ||jerk(t)||² dt
  + w_foot Σ ||foot_i − nominal_foot_i||²
```

The ZMP/support difference is cubic and its square degree six. Four-point Gauss quadrature integrates it exactly. QP construction and `plan_cost()` use the same formula. The score depends on weights; it is neither electrical energy nor mechanical work.

Boundary conditions specify initial `(c,v,a)`, terminal `v=a=0` and final CoM over the final foot center plus sole offset. The default ankle-to-sole x offset is 0.035 m, specific to the G1 geometry.

## Continuous support checks

Single support uses a small rectangle inside the stance sole. Double support uses the moving conservative rectangle described above. Conservative support can reject physically feasible walking; do not enlarge it beyond the actual contact footprint merely to obtain a solution.

The initial QP checks ZMP at interval start/middle/end. `validate_plan()` also finds real roots of the derivative of `ZMP − moving_support_center` and checks interior extrema, including velocity extrema. Violations above `2e-5` in SI numerical units trigger added constraints and up to eight solves. For positions this is 20 μm. A `solved` status alone is insufficient.

## W2 landing-box exercise

Run at the repository root. The first landing x is constrained to 0.12–0.125 m; the default fixed landing at 0.1 m cannot satisfy it.

```python
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path('lab-7-walking-optimization/src').resolve()))
from walking_model import WalkConfig, build_schedule
from lipm_optimizer import solve_walk, validate_plan
from walking_artifacts import export_plan
schedule = build_schedule(WalkConfig())
# Shape: (landings excluding the initial two feet, x/y, lower/upper).
bounds = np.tile([-np.inf, np.inf], (len(schedule.feet) - 2, 2, 1))
bounds[0, 0] = [0.12, 0.125]
plan = solve_walk(schedule, optimize_feet=True, landing_bounds=bounds)
print(plan.feet[2], validate_plan(plan))
export_plan(plan, Path('/tmp/landing-box'))
```

A single convex box cannot represent holes, a choice among disconnected stepping stones or nonconvex obstacle avoidance. Step-length/width bounds do not replace whole-body IK or self-collision checks.

## G1 adapter

1. `g1_walk.run()` calls `m3_walking.build()` and settles the torque model/controller.
2. Measured feet, CoM and velocity initialize a new plan. CoM height is relative to the MuJoCo ground plane, not the ankle frame. Initial reduced-model acceleration is assumed zero.
3. `OptimizedGait` extends `GaitSchedule`, changing timing and foot targets. Adapter tests check the `_foot_state`/`_swing_of_phase` contact and landing conventions.
4. `OptimizedReference` feeds plan `xi`, `xi_dot` and ZMP to the existing `DCMReference`.
5. Lab 6 solves contact wrench and torque using Pinocchio dynamics; MuJoCo steps and measures contact. Telemetry and Pinocchio updates refer to the same post-step state.

The swing reuses Lab 6 horizontal raised-cosine and vertical sine trajectories. Vertical sine endpoint velocities are nonzero; this is not impact-free optimized swing. The controller clamps ZMP using actual feet, so commanded and planned ZMP can differ. Compare `telemetry.csv` and measured contacts. Lab 9 replans states/wrenches with fixed contact order; it does not choose a new contact sequence.
