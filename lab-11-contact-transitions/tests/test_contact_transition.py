"""Boundary, derivative and rejection tests independent of recorded success."""

import numpy as np
import pytest
from smooth_swing import smooth_swing


@pytest.mark.parametrize("duration", [0.4, 0.65, 1.0])
@pytest.mark.parametrize("height_delta", [0.0, 0.02])
def test_endpoints_are_at_rest(duration, height_delta):
    a = np.array([0.0, 0.1, 0.03])
    b = np.array([0.1, 0.12, 0.03 + height_delta])
    for t, expected in ((0, a), (duration, b)):
        p, v, acc = smooth_swing(t, duration, a, b, 0.03)
        np.testing.assert_allclose(p, expected, atol=1e-12)
        np.testing.assert_allclose(v, 0, atol=1e-12)
        np.testing.assert_allclose(acc, 0, atol=1e-12)


def test_derivatives_and_clearance():
    a, b = np.zeros(3), np.array([0.1, 0.02, 0.0])
    for t in np.linspace(0.02, 0.63, 23):
        h = 1e-5
        p, v, acc = smooth_swing(t, 0.65, a, b, 0.03)
        before = smooth_swing(t - h, 0.65, a, b, 0.03)
        after = smooth_swing(t + h, 0.65, a, b, 0.03)
        np.testing.assert_allclose(v, (after[0] - before[0]) / (2 * h), atol=1e-7)
        np.testing.assert_allclose(acc, (after[1] - before[1]) / (2 * h), atol=1e-6)
        assert p[2] >= 0
    assert smooth_swing(0.325, 0.65, a, b, 0.03)[0][2] == pytest.approx(0.03)


@pytest.mark.parametrize("duration", [0, -1, float("nan")])
def test_invalid_duration_rejected(duration):
    with pytest.raises(ValueError):
        smooth_swing(0, duration, np.zeros(3), np.ones(3), 0.03)


def test_early_landing_disables_swing_before_qp_contact_change():
    from types import SimpleNamespace

    from locomotion_controller import SteppingController

    reference = SimpleNamespace(com_target=np.zeros(3), swing_foot="left")
    controller = SteppingController.__new__(SteppingController)
    controller.schedule = SimpleNamespace(reference=lambda t: reference)
    controller.contact_sync_before_tasks = True
    controller._current_stance = ("right",)
    controller._effective_stance = lambda ref: ("left", "right")
    controller._sync_contacts = lambda stance: setattr(
        controller, "_current_stance", stance
    )
    controller.dcm_plan = None
    controller.com_task = SimpleNamespace(set_target=lambda target: None)
    controller.swing_task = SimpleNamespace(enabled=True)
    controller.swing_orientation_task = None
    controller.update_targets(1.0)
    assert controller.swing_task.enabled is False


def test_dense_qp_satisfies_unactuated_dynamics_and_actuator_bounds():
    from lab6_common import load_g1_pinocchio, load_g1_torque_mujoco, mj_state_to_pin
    from m3_walking import build

    m, d = load_g1_torque_mujoco()
    pm, pd = load_g1_pinocchio()
    stack, _, _, qp, controller = build(m, d, pm, pd, step_length=0.08, n_steps=4)
    qp.backend = "proxqp"
    qp.tolerance = 1e-7
    qp.max_iterations = 100
    qp.torque_margin = 0.1
    q, v = mj_state_to_pin(d)
    stack.update_dynamics(q, v)
    controller.update_targets(0.0)
    result = qp.solve(stack, q, v)
    assert result.status == "solved"
    assert max(result.constraint_violations.values()) < 1e-5
    assert np.all(result.tau >= m.actuator_ctrlrange[:, 0])
    assert np.all(result.tau <= m.actuator_ctrlrange[:, 1])


@pytest.mark.parametrize(
    "change",
    [
        {"steps": 2.5},
        {"step_length": -0.1},
        {"push_force": float("nan")},
        {"mass_ratio": 0},
        {"push_axis": 2},
    ],
)
def test_invalid_physical_case_rejected(change):
    from contact_walk import WalkCase

    with pytest.raises(ValueError):
        WalkCase(**change)
