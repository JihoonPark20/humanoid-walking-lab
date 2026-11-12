"""Retargeting continuity and constrained projection, independent of simulation."""

import numpy as np
import pytest
from step_adjustment import bounded_adjustment, quintic_coefficients


def test_retarget_preserves_position_velocity_acceleration():
    p = np.array([0.1, 0.2, 0.06])
    v = np.array([0.1, 0, -0.04])
    a = np.array([1, 2, 3.0])
    target = np.array([0.2, 0.22, 0.03])
    duration = 0.3
    c = quintic_coefficients(p, v, a, target, duration)
    np.testing.assert_allclose(c[0], p)
    np.testing.assert_allclose(c[1] / duration, v)
    np.testing.assert_allclose(2 * c[2] / duration**2, a)
    np.testing.assert_allclose(c.sum(axis=0), target, atol=1e-12)
    np.testing.assert_allclose(sum(i * c[i] for i in range(1, 6)), 0, atol=1e-12)
    np.testing.assert_allclose(
        sum(i * (i - 1) * c[i] for i in range(2, 6)), 0, atol=1e-12
    )


@pytest.mark.parametrize("error", [[0, 0], [1, 1], [-1, -1], [0.01, -0.02]])
def test_landing_box_and_qp_optimality(error):
    landing = np.array([0.08, 0.18, 0])
    stance = np.zeros(3)
    delta, lo, hi = bounded_adjustment(error, 3.8, 0.3, landing, stance, "left")
    assert np.all(delta >= lo) and np.all(delta <= hi)
    assert 0.14 - 1e-12 <= landing[1] + delta[1] <= 0.24 + 1e-12
    wanted = np.exp(3.8 * 0.3) * np.asarray(error)
    cost = lambda x: np.sum((x - wanted) ** 2) + np.sum(x * x)
    rng = np.random.default_rng(3)
    assert all(cost(delta) <= cost(x) + 1e-12 for x in rng.uniform(lo, hi, (100, 2)))
