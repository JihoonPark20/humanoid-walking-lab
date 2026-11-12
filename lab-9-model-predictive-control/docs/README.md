# W4: MPC replanning from measured state

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

## Delay and model errors

The combined case retains `mass_ratio=1.05`, friction 0.4, one-interval measurement delay and a 0.03 m/s disturbance. Timestamped past measurements and previously issued commands predict current state. An unobserved disturbance cannot be predicted; it enters through later measurements. The controller receives neither the current plant state nor its true mass.

Mass identification uses three consecutive CoM height measurements and the vertical commands over two intervals:

```text
v_bar0 = (z1-z0)/h0
v_bar1 = (z2-z1)/h1
m_candidate = (F0*h0 + F1*h1) / (2*(v_bar1-v_bar0) + g*(h0+h1))
m_hat <- m_hat + 0.3*(m_candidate-m_hat)
```

Reject candidates outside 0.75–1.25 times nominal mass, with the same bounds in every profile. Reuse duplicate timestamps and reject reversed time. After a missing measurement, wait for three consecutive measurements before estimating again. Vertical external forces and command-force error are indistinguishable from mass error and violate these identification assumptions.

`CentroidalMPC.update_mass()` updates the solver mass parameter and reference momentum/wrench for the same nominal velocity. Revalidate fallback by rolling out under the updated model. A mass change invalidates an RTI preparation. The reduced loop runs measurement → estimation → preparation → feedback, solving one QP; fixed-mass G1 keeps advance preparation.

`validate_mpc_scenarios.py` tests eight environments × full NLP/RTI = 16 profiles. Require every interval to complete, final CoM error below 3 cm, and independent validation of actual plant state/wrench. `command_wrench_violation` and `saturation_intervals` expose plant command limiting.

## RTI preparation and feedback

`centroidal_rti.py` uses CasADi derivatives and OSQP for a single SQP step. `prepare(index)` shifts the previous solution and constructs exact-discrete-dynamics Jacobians, quadratic tracking cost and linear wrench/state bounds. `feedback(measured)` changes only the initial-state equality RHS and solves **one QP**. An independent nonlinear rollout must pass; otherwise only previous inputs revalidated from the new measured state may be used as fallback. Full NLP is never called as RTI fallback.

```text
min_delta  0.5 delta.T H delta + grad(f).T delta
subject to l <= g(z_bar) + J_g(z_bar) delta <= u
           delta_x0 = x_measured - x_bar0
```

The tracking objective is quadratic, so `H=Hessian(f)` equals its Gauss–Newton Hessian. Nonlinear constraint Hessians are omitted. Implementation uses the absolute QP `z=z_bar+delta`, scales momentum by mass and wrench by weight, and scales constraint rows. Wrench/state bounds remain; only nonlinear dynamics are linearized.

After the QP, exact nonlinear rollout determines acceptance. `linearization_gap` is the maximum componentwise discrepancy between linear prediction and exact rollout, mixing position and momentum units. A single SQP step need not be feasible or converged.

The fixed-model G1 loop prepares `k+1` in advance. `solve(measured,index)` calls both stages sequentially for reduced experiments with mass estimation. Preparation rebuilds the sparse QP and factorization; feedback changes RHS only. There is no inter-horizon factorization reuse, condensing or code generation. Timing records preparation, feedback, estimation and total, excluding initial graph construction.

This educational implementation follows the preparation/feedback split described in [acados documentation](https://docs.acados.org/features/), but uses no acados backend. It provides no hard real-time guarantee.

## Implementation and modules

First build exact ZMP-input LIPM dynamics and horizon QP, then terminal DCM viability by backward recursion from final double support. Compare offline feedback and MPC under the same disturbance. Reuse Lab 8's centroidal NLP in a shifted receding horizon, then connect CoM/momentum references to Lab 6 Whole-body QP.

- `lipm_mpc.py`: `[c_x,v_x,c_y,v_y]`, ZMP input, OSQP horizon, terminal DCM region, braking fallback.
- `centroidal_mpc.py`: Lab 8 solver, shifted primal warm start, delay/mass/friction experiments.
- `centroidal_rti.py`: automatic derivatives, one-QP RTI, nonlinear checks, split timing.
- `centroidal_estimation.py`: timestamp prediction and height/command mass identification.
- `validate_mpc_scenarios.py`: 16 environment/algorithm profiles and acceptance matrix.
- `mpc_experiment.py`: paired open-loop/offline feedback/MPC reduced rollouts; CSV/JSON/plots, timing median/p95/p99/deadline misses.
- `g1_mpc.py`: measured Pinocchio state → MPC → Lab 6 CoM/angular-momentum tasks → Whole-body QP → MuJoCo.

Never apply an infeasible solution. LIPM fallback brakes only when the capture point lies in the current conservative support region; otherwise stop. Centroidal fallback requires revalidating the previous plan from the new measured state, otherwise stop. Log failures and chosen paths. Compare full NLP and RTI with the same nominal plan, disturbance and horizon, including total computation time.

## Terminal DCM region

With `omega=sqrt(g/h)`, `xi=c+v/omega`, constant ZMP gives `xi_next=exp(omega*dt)*xi+(1-exp(omega*dt))*z`. Backward recursion is:

```text
alpha = exp(-omega * dt)
lower[k] = alpha * lower[k+1] + (1-alpha) * zmp_lower[k]
upper[k] = alpha * upper[k+1] + (1-alpha) * zmp_upper[k]
```

Constrain horizon-end DCM to this time-indexed region. It assumes fixed contact timing, separate horizontal axes and constant-input LIPM, and does not establish G1 joint/torque feasibility. Distinguish reduced and G1 results in the [gallery](../GALLERY.md). See also [observation robustness](OBSERVATION_ROBUSTNESS.md).
