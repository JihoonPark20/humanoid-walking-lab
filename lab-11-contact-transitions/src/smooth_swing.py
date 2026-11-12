"""C2 swing references with explicit endpoint conditions, built on Lab 6."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "lab-6-whole-body-control/src"))
from gait_planner import GaitSchedule


def smooth_swing(t, duration, start, landing, height):
    """Quintic translation plus a sixth-degree clearance bump; SI units."""
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError("Swing duration must be positive and finite")
    s = float(np.clip(t / duration, 0, 1))
    blend = 10 * s**3 - 15 * s**4 + 6 * s**5
    velocity = (30 * s**2 - 60 * s**3 + 30 * s**4) / duration
    acceleration = (60 * s - 180 * s**2 + 120 * s**3) / duration**2
    delta = np.asarray(landing) - np.asarray(start)
    p = np.asarray(start) + blend * delta
    v, a = velocity * delta, acceleration * delta
    p[2] += height * 64 * (s**3 - 3 * s**4 + 3 * s**5 - s**6)
    v[2] += height * 64 * (3 * s**2 - 12 * s**3 + 15 * s**4 - 6 * s**5) / duration
    a[2] += height * 64 * (6 * s - 36 * s**2 + 60 * s**3 - 30 * s**4) / duration**2
    return p, v, a


class SmoothGait(GaitSchedule):
    """Keep the Lab 6 schedule while replacing only its swing interpolation."""

    def _swing_state(self, t_local, duration, start, landing):
        return smooth_swing(t_local, duration, start, landing, self.step_height)

    @classmethod
    def from_schedule(cls, source):
        """Rebuild the same timing and footholds without mutating the source."""
        return cls(
            source.left_frame,
            source.right_frame,
            source.foot_home[source.left_frame],
            source.foot_home[source.right_frame],
            source.com_home,
            n_steps=source.n_steps,
            t_initial=source.t_initial,
            t_double=source.t_double,
            t_single=source.t_single,
            step_length=source.step_length,
            step_width=source.step_width,
            step_height=source.step_height,
            first_swing=source.first_swing,
            close_stance=source.close_stance,
            com_shift_ratio=source.com_shift_ratio,
            com_forward_shift_ratio=source.com_forward_shift_ratio,
        )


class LegacyGait(SmoothGait):
    """Lab 6 interpolation with exactly the same timing and footholds as the comparison."""

    _swing_state = GaitSchedule._swing_state
