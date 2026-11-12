"""Packet identity, timing, loss and noise rejection independent of the optimizer."""

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
for lab in (
    "lab-7-walking-optimization",
    "lab-8-centroidal-optimization",
    "lab-9-model-predictive-control",
):
    sys.path.insert(0, str(ROOT / lab / "src"))
from centroidal_model import CentroidalConfig, Contacts
from centroidal_sensing import (
    PacketStateEstimator,
    SensorConfig,
    StatePacket,
    StateSensor,
)


def standing_problem(n: int = 100) -> tuple:
    """Supply a static equilibrium and real command history for observer tests."""
    cfg = CentroidalConfig()
    state = np.array([0, 0, 0.7, 0, 0, 0, 0, 0, 0], dtype=float)
    contacts = Contacts(np.arange(n + 1) * 0.1, np.zeros((n, 2, 3)), np.ones((n, 2)))
    u = np.zeros((2, 6))
    u[:, 2] = cfg.mass * cfg.gravity / 2
    return cfg, state, contacts, [u.copy() for _ in range(n)]


def test_packet_delay_loss_and_seed_identity() -> None:
    """Repeated reads cannot redraw noise; paired trials share the same loss pattern."""
    cfg = SensorConfig(delay_steps=2, blackout=(3, 6), seed=31)
    a, b = StateSensor(cfg), StateSensor(cfg)
    for k in range(15):
        a.capture(np.zeros(9), k)
        b.capture(np.zeros(9), k)
        pa, pb = a.receive(k), b.receive(k)
        assert (pa is None) == (pb is None)
        assert a.receive(k) is pa
        if pa is not None:
            assert pa.index == k - 2
            assert np.allclose(pa.state, pb.state, atol=0, rtol=0)
    assert a.dropped == b.dropped >= 3
    assert all(a.receive(k) is None for k in (0, 1, 5, 6, 7))
    with pytest.raises(ValueError):
        a.capture(np.zeros(9), 14)


def test_missing_duplicate_and_future_packets() -> None:
    """No packet means prediction only; duplicates cannot repeatedly shrink innovation."""
    cfg, initial, contacts, commands = standing_problem()
    observer = PacketStateEstimator(initial, gain=0.5)
    pushed = initial.copy()
    pushed[4] = 0.2
    packet = StatePacket(0, pushed)
    first = observer.estimate(packet, 0, [], contacts, cfg)
    repeated = observer.estimate(packet, 0, [], contacts, cfg)
    assert observer.corrections == 1
    assert np.allclose(first, repeated, atol=1e-12, rtol=0)
    prediction = observer.estimate(None, 1, commands[:1], contacts, cfg)
    assert prediction[1] == pytest.approx(0.1 * 0.1 / cfg.mass, abs=1e-12)
    assert observer.corrections == 1
    with pytest.raises(ValueError):
        observer.estimate(StatePacket(2, initial), 1, commands[:1], contacts, cfg)
    with pytest.raises(ValueError):
        observer.estimate(None, 0, [], contacts, cfg)


def test_filter_reduces_stationary_measurement_noise() -> None:
    """The same 100 noisy packets produce lower CoM RMS with a blended observer."""
    cfg, initial, contacts, commands = standing_problem()
    sensor = StateSensor(SensorConfig(delay_steps=1, dropout_probability=0, seed=7))
    observers = [PacketStateEstimator(initial, gain=g) for g in (1.0, 0.65)]
    errors = [[], []]
    for k in range(100):
        sensor.capture(initial, k)
        for observer, history in zip(observers, errors):
            estimate = observer.estimate(
                sensor.receive(k), k, commands[:k], contacts, cfg
            )
            history.append(np.linalg.norm(estimate[:3] - initial[:3]))
    rms = np.sqrt(np.mean(np.square(errors), axis=1))
    assert rms[1] < 0.85 * rms[0]


@pytest.mark.parametrize(
    "kwargs",
    [
        {"com_std": -1},
        {"momentum_std": float("nan")},
        {"delay_steps": -1},
        {"delay_steps": 0.5},
        {"dropout_probability": 1.1},
        {"blackout": (5, 2)},
    ],
)
def test_invalid_sensor_settings(kwargs: dict) -> None:
    """Reject malformed experiments before producing misleading result files."""
    with pytest.raises(ValueError):
        SensorConfig(**kwargs)


@pytest.fixture(scope="module")
def nominal_plan() -> object:
    """Use the same independently validated plan as the published benchmark."""
    from centroidal_io import make_walking_problem
    from centroidal_optimizer import CentroidalSolver

    cfg, contacts, ref, _ = make_walking_problem(2, 0.1, 0.015)
    return CentroidalSolver(len(contacts.times) - 1, cfg, True).solve(
        ref[0], ref, contacts
    )


@pytest.mark.parametrize(
    "profile", ["noise", "noise_delay", "packet_loss", "burst_loss", "blackout_stress"]
)
def test_closed_loop_sensor_profiles(nominal_plan: object, profile: str) -> None:
    """Recommended cases finish; unobserved prolonged disturbances still fail honestly."""
    from mpc_experiment import run_centroidal
    from observation_benchmark import PROFILES

    report, _ = run_centroidal(
        nominal_plan,
        "rti",
        adapt_mass=False,
        sensor_config=(SensorConfig(dropout_probability=0, blackout=(18, 60))
                       if profile == "blackout_stress" else PROFILES[profile]),
        observer_gain=0.65,
    )
    assert report["mass_observer_updates"] == 0
    assert report["push_requested_time_s"] == pytest.approx(2.1, abs=1e-12)
    assert report["push_applied_time_s"] == pytest.approx(2.15, abs=1e-12)
    assert report["sensing"]["corrections"] <= report["sensing"]["captured_packets"]
    if profile == "blackout_stress":
        assert not report["passed"] and report["completed_intervals"] < 60
    else:
        assert report["passed"] and report["completed_intervals"] == 60
        assert report["push_applied"] and report["final_com_error_m"] < 0.03
        assert max(report["plant_state_violations"].values()) <= 1e-4
        assert report["plant_wrench_violation"] <= 1e-4
        assert report["rti"]["qp_solve_calls"] == 60


def test_noisy_sensor_requires_explicit_mass_observer_opt_out(
    nominal_plan: object,
) -> None:
    """Do not silently run a noise-sensitive identification method on noisy packets."""
    from mpc_experiment import run_centroidal

    with pytest.raises(ValueError, match="adapt_mass=False"):
        run_centroidal(nominal_plan, "rti", sensor_config=SensorConfig())
