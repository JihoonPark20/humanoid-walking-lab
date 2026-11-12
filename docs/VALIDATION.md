# Validation

These are flat-ground, fixed-contact experiments with explicit operating ranges.
A successful reduced plan is checked separately from whole-body MuJoCo tracking.
Tests also exercise rejection and failure handling; they are not all demonstration runs.

| Exercise | Criteria | Published evidence |
|---|---|---|
| Probe force control | Complete trace, raw and filtered force in 5 ± 1 N >90%, peak <15 N, XY <5 mm, attitude <5°, probe-only contact | [Recorded run](../lab-3-dynamics-force-control/media/line_trace_summary.json) |
| Slalom planning | Joint RMS <0.05 rad, sampled clearance ≥30 mm, no MuJoCo contacts | [Recorded run](../lab-4-motion-planning/media/slalom_summary.json) |
| DCM walking | ≥10 steps, ≥1 m, ZMP inside >90%, torque limits | [12-step result](../lab-6-whole-body-control/media/m3_walking_summary.json) |
| Centroidal animation | Saved plan satisfies dynamics, wrench, height, velocity, angular momentum and boundary constraints; no whole-body rollout | [Plan and video verification](../lab-8-centroidal-optimization/media/offline/animation_summary.json) |
| MPC / RTI | Completion, final CoM error <30 mm, physical residual ≤1e-4 | [16 conditions](../lab-9-model-predictive-control/media/validated_scenarios/matrix.json) |
| Observation robustness | Same physical gates; paired seeds with noise, delay and short packet-loss bursts | [24 runs](../lab-9-model-predictive-control/media/observation_benchmark/matrix.json) |
| Whole-body tracking | Feasible plan, completion, no flight/clipping, final foot gap <30 mm | [12 profiles](../lab-10-whole-body-optimal-control/media/validated_scenarios/matrix.json) |

```bash
.venv/bin/python -m pytest lab-*/tests -q
.venv-ocp/bin/python -m pytest lab-10-whole-body-optimal-control/tests -q
.venv/bin/python lab-9-model-predictive-control/src/validate_mpc_scenarios.py
.venv/bin/python lab-9-model-predictive-control/src/observation_benchmark.py
.venv-ocp/bin/python lab-10-whole-body-optimal-control/src/validate_tracking_scenarios.py
```

The main environment skips the Crocoddyl tests; run the second command to cover them.
Whole-body profiles use their recommended planning costs, so they do not rank feedback
policies on an identical plan. Timing depends on hardware and load; RTI is not a hard
real-time guarantee. Long sensor blackouts, online foothold changes, whole-body MPC and
uneven terrain are outside the release scope.

Gallery generation and release packaging require a passing result whose `video_sha256` matches the source recording. Numerical-only reruns cannot validate an older video. Lab 3 uses a rigid massless probe and a flat plate with μ=0.05; this is not a benchmark of articulated-finger contact.
