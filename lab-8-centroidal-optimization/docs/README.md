# W3: Centroidal optimization

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Fix the contact schedule and footsteps from Lab 7, then optimize CoM height, linear/angular momentum and each foot's wrench. Read the model, nonlinear optimizer, independent validation and G1 quantity comparison in that order.

## Modules

- `centroidal_model.py`: state `[c,p,L]`, sole-center wrench `[f,tau]`, fixed contact schedule, exact discrete dynamics and independent constraint checks.
- `centroidal_optimizer.py`: CasADi derivatives and IPOPT Direct Multiple Shooting. Enforces initial state, node references, contact/friction/CoP/height constraints and optional terminal rest. `CentroidalSolver.solve(initial, reference, contacts, warm_start)` is reused by MPC.
- `offline_centroidal.py`: warm-starts from Lab 7 and solves a varying-height task, exporting NPZ/CSV/JSON/plots.
- `pinocchio_centroidal.py`: compares G1 centroidal momentum/map against an independent sum of body momenta.
- `centroidal_io.py`: converts the Lab 7 schedule/reference, saves and revalidates plans.

Flow: Lab 7 plan → fixed feet/contacts → 9D state and 12D wrench NLP → exact rollout validation → Lab 9 MPC and G1 Whole-body control.

## Model

`c_dot=p/m`, `p_dot=sum(f)+mg`, `L_dot=sum((r-c)×f+tau)`. With constant world-frame wrenches and foot positions over each interval, CoM is quadratic and momentum can be integrated exactly. Angular momentum uses interval-average CoM. Quaternion and torso orientation are absent from this reduced state.

Friction uses a square pyramid; CoP uses linear bounds on sole-center moments, with a separate yaw-moment bound. This approximates the full rectangular contact wrench cone. Swing-foot wrench is zero. Foot yaw and contact order stay fixed.

Check dynamics, initial/terminal conditions, friction, CoP, height and momentum independently of solver status. A feasible reduced-model height still needs whole-body execution to establish G1 joint feasibility. NumPy/CasADi implement the reduced dynamics; Pinocchio computes the actual G1 CoM, Jacobians and centroidal map.

See [CasADi documentation](https://web.casadi.org/docs/) and the [run guide](../README.md).
