# C1: Hybrid Position-Force Control

## Goal

Implement hybrid control: position control in XY plane, PI force control in Z. Maintain constant contact force while controlling EE position on a table surface.

## Files

- Force controller: `src/c1_force_control.py`
- Line trace capstone: `src/c2_line_trace.py`
- Scene: Menagerie UR5e + mounted Robotiq and a rigid, massless contact probe loaded through `src/lab3_common.py`
- Tests: `tests/test_force_control.py`

## Theory

### Hybrid Position-Force Control

The key insight: in constrained tasks (e.g., polishing, assembly), some Cartesian directions should be position-controlled and others force-controlled. Selection matrices partition the task space:

```
S_p = diag(1, 1, 0)  — position control in XY
S_f = diag(0, 0, 1)  — force control in Z
```

### Control Law

**Position (XY):**
```
F_p = K_p · S_p · (x_d - x) + K_d · S_p · (ẋ_d - ẋ)
```

**Force (Z) with PI + velocity damping:**
```
e_f = F_desired - F_measured
F_f = -(F_desired + K_fp · e_f + K_fi · ∫e_f dt) - K_dz · ẋ_z
```

**Combined:**
```
τ = J_linear^T · (F_p + F_f) + J_angular^T · moment + g(q)
moment = K_R · log(R_des · R^T) - K_w · ω
```

Feedforward supplies the nominal downward load; PI corrects force error. Z damping and SO(3) orientation feedback limit oscillation and tool rotation. Stability must still be checked against measured contact forces.

### Contact Force Measurement

Forces are read via `mj_contactForce()`. The complete contact force, including friction, is transformed to world coordinates and projected onto Z. The controller uses an EMA (α=0.2); acceptance checks also inspect the unfiltered signal. Any wrist or finger contact with the tabletop rejects the filmed trace.

This is more reliable than MuJoCo's `<force>` sensor, which measures all constraint forces on the body (including articulation forces), not just contact.

### Intentional contact

In Lab 3, contacting the table is intentional. The goal is to establish gentle contact and then regulate approximately `5 N` normal force while holding or tracing in XY.

## Recorded scenario

The [video and measurements](../GALLERY.md) come from one `run_line_trace()` execution.
The recording callback observes the simulator; it does not implement another controller.

A spherical probe (radius 10 mm, offset 60 mm beyond the pinch frame) slides on a flat
plate with sliding friction **μ = 0.05**. Probe mass is neglected in this teaching model.
The fingers start closed and are held by their actuator at command 230/255; the
probe remains rigidly mounted to the tool body. Finger poses are not overwritten during simulation.
This isolates hybrid control from articulated-finger contact and is not a gripper-polishing benchmark.
Pinocchio and MuJoCo probe poses are cross-validated in `test_recorded_demo.py`.

The release scenario traces 50 mm in 6 s at 5 N after a 10 mm/s approach and 1.5 s settling.
Acceptance requires >90% of raw **and** filtered trace-force samples within 5 ± 1 N,
peak force <15 N over the entire run, XY error <5 mm, orientation error <5°, completion,
zero non-probe tabletop contacts, and both gripper driver angles >0.65 rad throughout.
Full approach/settle/trace/hold phases remain in the video.

## Architecture

```
              ┌───────────────────┐
  xy_d ──────▶│  Hybrid Controller│
  F_desired ─▶│                   │──▶ τ ──▶ MuJoCo
              │  Position (XY)    │         ▲
              │  + Force (Z)      │         │
              │  + g(q)           │    mj_contactForce
              └───────────────────┘    (contact probe ↔ plate)
                     ▲
                     │ FK, J (Pinocchio)
                     └─────────────────
```

### Phase State Machine

```
APPROACH → SETTLE → TRACE → HOLD
   │          │        │        │
   ▼          ▼        ▼        ▼
 Impedance  Hybrid   Hybrid   Hybrid
 descent    at start  + line   at end
            XY       traj
```

## Key Gains

| Parameter | Value | Role |
|-----------|-------|------|
| K_p (XY) | 2000 N/m | Position stiffness |
| K_d (XY) | 100 N·s/m | Position damping |
| K_fp | 0.5 | Force proportional |
| K_fi | 3.0 | Force integral |
| K_dz | 30.0 N·s/m | Z-velocity damping |
| K_R / K_w | 60 / 1 | Orientation stiffness / damping |
| α_filter | 0.2 | EMA force smoothing |

## How to Run

```bash
# Static force hold demo
python3 src/c1_force_control.py

# Capstone: constant-force line tracing
python3 src/record_lab3_demo.py
```

## Limitations

- The Jacobian X-row is small (~0.01) in the vertical tool configuration, limiting XY tracking bandwidth during contact. Longer lines (>50mm) require slower trajectories.
- Line traces work best near (0.4, 0.0) where the arm has good manipulability.
