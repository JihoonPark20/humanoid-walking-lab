"""Measured-DCM foot placement adjustment with fixed timing and C2 retargeting."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "lab-11-contact-transitions/src"))
from contact_walk import rebuild_plan
from smooth_swing import SmoothGait


def quintic_coefficients(position, velocity, acceleration, target, duration):
    """Boundary-value polynomial: preserve current p/v/a and finish at rest."""
    if duration <= 0:
        raise ValueError("Retargeting requires positive remaining time")
    c = np.zeros((6, 3))
    c[:3] = [position, velocity * duration, 0.5 * acceleration * duration**2]
    matrix = np.array([[1, 1, 1], [3, 4, 5], [6, 12, 20]], dtype=float)
    rhs = np.array([target - c[:3].sum(axis=0), -c[1] - 2 * c[2], -2 * c[2]])
    c[3:] = np.linalg.solve(matrix, rhs)
    return c


class RetargetableGait(SmoothGait):
    """One active swing can be retargeted without a p/v/a discontinuity."""

    def reference(self, t):
        result = super().reference(t)
        segment = getattr(self, "retarget", None)
        if segment is not None:
            begin, end, coeff, foot = segment
            if begin <= t < end and result.swing_foot == foot:
                s = (t - begin) / (end - begin)
                result.swing_position = sum(coeff[i] * s**i for i in range(6))
                result.swing_velocity = sum(
                    i * coeff[i] * s ** (i - 1) for i in range(1, 6)
                ) / (end - begin)
                result.swing_acceleration = (
                    sum(i * (i - 1) * coeff[i] * s ** (i - 2) for i in range(2, 6))
                    / (end - begin) ** 2
                )
        return result


def bounded_adjustment(error, omega, remaining, landing, stance, side):
    """Solve a separable box QP for the next landing; no disturbance oracle."""
    wanted = np.exp(omega * remaining) * np.asarray(error)
    lower, upper = np.full(2, -0.04), np.full(2, 0.04)
    lower[0] = max(lower[0], stance[0] - 0.02 - landing[0])
    upper[0] = min(upper[0], stance[0] + 0.20 - landing[0])
    width = (0.14, 0.24) if side == "left" else (-0.24, -0.14)
    lower[1] = max(lower[1], stance[1] + width[0] - landing[1])
    upper[1] = min(upper[1], stance[1] + width[1] - landing[1])
    if np.any(lower > upper):
        raise ValueError("Empty reachable landing box")
    # min ||delta-wanted||² + ||delta||²: exact unconstrained minimizer wanted/2.
    delta = np.clip(wanted / 2, lower, upper)
    return delta, lower, upper


class StepAdjustment:
    """Adjust at 45% of each swing using measured DCM; timing stays fixed."""

    gait_type = RetargetableGait

    def __init__(self):
        self.events = []
        self.visited = set()

    def update(self, t, schedule, plan, controller, data):
        index, phase = schedule._phase_at(t)
        if (
            index not in schedule._swing_of_phase
            or index in self.visited
            or phase.progress(t) < 0.45
        ):
            return plan
        self.visited.add(index)
        error = controller.com_task.current_dcm(data) - plan.reference(t).xi
        if np.linalg.norm(error) < 0.008:
            return plan
        foot, start, landing = schedule._swing_of_phase[index]
        stance_name = (
            schedule.right_frame if foot == schedule.left_frame else schedule.left_frame
        )
        stance = schedule._foot_state[index][stance_name]
        delta, lower, upper = bounded_adjustment(
            error,
            plan.omega,
            phase.t_end - t,
            landing,
            stance,
            "left" if foot == schedule.left_frame else "right",
        )
        before = schedule.reference(t)
        old_landing = landing.copy()
        delta3 = np.r_[delta, 0.0]
        current_feet = {
            name: pos.copy() for name, pos in schedule._foot_state[index].items()
        }
        for j in range(index, len(schedule._phases)):
            schedule._foot_state[j] = {
                name: pos.copy() for name, pos in current_feet.items()
            }
            if j in schedule._swing_of_phase:
                name, _, target = schedule._swing_of_phase[j]
                target = target + delta3
                schedule._swing_of_phase[j] = (name, current_feet[name].copy(), target)
                current_feet[name] = target.copy()
        target = schedule._swing_of_phase[index][2]
        coefficients = quintic_coefficients(
            before.swing_position,
            before.swing_velocity,
            before.swing_acceleration,
            target,
            phase.t_end - t,
        )
        schedule.retarget = (t, phase.t_end, coefficients, foot)
        # Inspect every vertical polynomial extremum, not only evenly spaced samples.
        roots = np.polynomial.polynomial.polyroots(
            np.arange(1, 6) * coefficients[1:, 2]
        )
        samples = [
            0.0,
            1.0,
            *[float(r.real) for r in roots if abs(r.imag) < 1e-9 and 0 < r.real < 1],
        ]
        heights = np.array(
            [sum(coefficients[i, 2] * s**i for i in range(6)) for s in samples]
        )
        if heights.min() < min(start[2], target[2]) - 1e-8:
            raise ValueError("Retargeted swing crosses the ground reference")
        self.events.append(
            {
                "time_s": t,
                "phase": index,
                "measured_dcm_error_m": error.tolist(),
                "old_landing": old_landing.tolist(),
                "landing": target.tolist(),
                "delta_m": delta.tolist(),
                "lower_m": lower.tolist(),
                "upper_m": upper.tolist(),
                "timing_changed": False,
            }
        )
        return rebuild_plan(schedule, plan)
