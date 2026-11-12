"""Timestamp predictor와 CoM 기반 질량 추정, 실제 실패 조건의 회귀 검사."""

import sys
from dataclasses import replace
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
from centroidal_estimation import VerticalMassObserver, predict_delayed_state
from centroidal_io import make_walking_problem
from centroidal_model import CentroidalConfig, CentroidalPlan, Contacts, centroidal_step
from centroidal_optimizer import CentroidalSolver
from centroidal_rti import CentroidalRTI
from mpc_experiment import run_centroidal
from validate_mpc_scenarios import SCENARIOS


def test_observer_identifies_mass_from_positions_only() -> None:
    """불균일 간격과 잘못된 momentum 측정에서도 높이·명령만으로 질량을 구한다."""
    nominal = CentroidalConfig()
    actual = replace(nominal, mass=nominal.mass * 1.05)
    contacts = Contacts(
        np.array([0.0, 0.08, 0.2, 0.27, 0.4, 0.55]),
        np.zeros((5, 2, 3)),
        np.ones((5, 2)),
    )
    observer = VerticalMassObserver(nominal, gain=1.0)
    state = np.array([0.0, 0.0, 0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    commands = []
    observer.observe(state, 0, commands, contacts)
    observer.observe(state, 0, commands, contacts)
    assert observer.updates == 0
    for k in range(5):
        wrench = np.zeros((2, 6))
        wrench[:, 2] = (nominal.mass * nominal.gravity + 10 * k) / 2
        commands.append(wrench)
        state = centroidal_step(
            state,
            wrench,
            contacts.positions[k],
            float(contacts.times[k + 1] - contacts.times[k]),
            actual,
        )
        measured = state.copy()
        measured[3:] = 1e6  # 질량 정보가 있는 실제 p를 observer에 주지 않는다.
        observer.observe(measured, k + 1, commands, contacts)
    assert np.isclose(observer.mass, actual.mass, atol=1e-8, rtol=0)
    assert observer.updates == 4 and observer.rejected == 0
    with pytest.raises(ValueError):
        observer.observe(state, 1, commands, contacts)


def test_delayed_prediction_is_causal() -> None:
    """과거 command만 적분하며 아직 측정하지 못한 외란을 미리 알 수 없다."""
    cfg = CentroidalConfig()
    contacts = Contacts(np.array([0.0, 0.1, 0.3]), np.zeros((2, 2, 3)), np.ones((2, 2)))
    initial = np.array([0.0, 0.0, 0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    command = np.zeros((2, 6))
    command[:, 2] = cfg.mass * cfg.gravity / 2
    command[:, 0] = 1.0
    predicted = predict_delayed_state(
        initial, 0, 1, [command, np.full((2, 6), 1e6)], contacts, cfg
    )
    expected = centroidal_step(initial, command, contacts.positions[0], 0.1, cfg)
    assert np.allclose(predicted, expected, atol=1e-12, rtol=0)
    pushed = expected.copy()
    pushed[4] += cfg.mass * 0.03
    assert not np.allclose(predicted, pushed, atol=1e-6, rtol=0)
    with pytest.raises(ValueError):
        predict_delayed_state(initial, 0, 2, [command], contacts, cfg)


@pytest.fixture(scope="module")
def walk() -> CentroidalPlan:
    """모든 조건에 같은 nominal과 0.03 m/s 외란을 사용한다."""
    cfg, contacts, ref, _ = make_walking_problem(2, 0.1, 0.015)
    return CentroidalSolver(len(contacts.times) - 1, cfg, True).solve(
        ref[0], ref, contacts
    )


@pytest.mark.parametrize("mode", ["mpc", "rti"])
@pytest.mark.parametrize("scenario", SCENARIOS)
def test_uncertain_delayed_scenarios(
    walk: CentroidalPlan, mode: str, scenario: str
) -> None:
    """외란·질량·마찰·1/2주기 지연을 낮추지 않고 전체 궤적을 완료해야 한다."""
    mass, friction, delay = SCENARIOS[scenario]
    report, _ = run_centroidal(
        walk, mode, mass_ratio=mass, friction=friction, delay_steps=delay
    )
    assert report["passed"] and report["push_applied"]
    assert report["completed_intervals"] == len(walk.wrenches)
    assert report["final_com_error_m"] < 0.03
    assert max(report["plant_state_violations"].values()) <= 1e-4
    assert report["solver_failures"] == report["validated_fallbacks"] == 0
    assert abs(report["estimated_mass_kg"] - mass * walk.config.mass) < 1e-4
    if mode == "rti":
        assert report["rti"]["qp_solve_calls"] == len(walk.wrenches)


def test_mass_change_invalidates_rti_preparation(walk: CentroidalPlan) -> None:
    """질량이 바뀐 후 이전 Jacobian/QP로 feedback을 하지 않도록 강제한다."""
    controller = CentroidalRTI(walk, 8)
    controller.prepare(0)
    controller.update_mass(walk.config.mass * 1.05)
    with pytest.raises(RuntimeError):
        controller.feedback(walk.initial)
    result = controller.solve(walk.initial, 0)
    assert result.config.mass == pytest.approx(walk.config.mass * 1.05)


def test_original_failure_remains_reproducible(walk: CentroidalPlan) -> None:
    """보상기를 끈 이전 복합 조건의 실패를 성공으로 다시 분류하지 않는다."""
    report, _ = run_centroidal(
        walk,
        "rti",
        mass_ratio=1.05,
        friction=0.4,
        delay_steps=1,
        compensate_delay=False,
        adapt_mass=False,
    )
    assert not report["passed"]
