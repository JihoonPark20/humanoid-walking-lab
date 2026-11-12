"""MPC의 exact Dynamics, terminal viability, 외란 복구, fallback 조건을 검증한다."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
ROOT = Path(__file__).resolve().parents[2]
for lab in ("lab-7-walking-optimization", "lab-8-centroidal-optimization"):
    sys.path.insert(0, str(ROOT / lab / "src"))
from centroidal_io import make_walking_problem
from centroidal_model import validate_centroidal
from centroidal_mpc import CentroidalMPC, horizon_contacts
from centroidal_optimizer import CentroidalFailure, CentroidalSolver
from lipm_mpc import LIPMMPC, MPCFailure, dcm_viability, lipm_matrices
from lipm_optimizer import solve_walk
from mpc_experiment import run_lipm
from walking_model import WalkConfig, build_schedule


def test_lipm_exact_composition() -> None:
    """같은 ZMP의 두 interval 합성이 단일 exact 적분과 일치한다."""
    a, b = lipm_matrices(0.05, 3.8)
    aa, bb = lipm_matrices(0.1, 3.8)
    assert np.allclose(a @ a, aa, atol=1e-12)
    assert np.allclose(a @ b + b, bb, atol=1e-12)


def test_viability_backward_boundary() -> None:
    """Backward 구간 경계가 허용 ZMP에서 다음 구간 경계로 이동한다."""
    times = np.array([0, 0.1, 0.2])
    lower = np.array([[-0.1, -0.1], [0, 0]])
    upper = lower + 0.05
    lo, hi = dcm_viability(
        times, lower, upper, 4.0, np.array([0.03, 0.03]), np.array([0.05, 0.05])
    )
    growth = np.exp(0.4)
    assert np.allclose(growth * lo[0] + (1 - growth) * lower[0], lo[1], atol=1e-12)
    assert np.allclose(growth * hi[0] + (1 - growth) * upper[0], hi[1], atol=1e-12)


def test_lipm_disturbance_and_failed_capture() -> None:
    """동일 외란에서 MPC 복구와 open-loop 발산을 확인하고 불가능한 capture를 거부한다."""
    plan = solve_walk(build_schedule(WalkConfig(steps=2)), optimize_feet=True)
    report, _ = run_lipm(plan, "mpc", push=0.03)
    assert report["passed"] and report["push_applied"]
    assert report["timing"]["count"] == len(plan.jerks)
    baseline, _ = run_lipm(plan, "open_loop", push=0.03)
    assert not baseline["passed"]
    controller = LIPMMPC(plan)
    x = np.array([10.0, 0.0, 10.0, 0.0])
    with pytest.raises(MPCFailure):
        controller.solve(x, 0)
    with pytest.raises(MPCFailure):
        controller.braking_input(x, 0)
    stationary = np.array([0.035, 0.0, 0.0, 0.0])
    assert np.allclose(controller.braking_input(stationary, 0), [0.035, 0], atol=1e-12)


def test_centroidal_replanning_warm_start_and_tail() -> None:
    """새 측정 상태로 초기 조건을 교체하며 tail에서도 유효한 접촉을 유지한다."""
    cfg, contacts, reference, _ = make_walking_problem(
        steps=1, dt=0.15, height_amplitude=0.01
    )
    plan = CentroidalSolver(len(contacts.times) - 1, cfg, True).solve(
        reference[0], reference, contacts
    )
    controller = CentroidalMPC(plan, 8)
    first = controller.solve(plan.initial, 0)
    measured = first.states[1].copy()
    measured[3] += 0.1
    second = controller.solve(measured, 1)
    assert np.allclose(second.states[0], measured, atol=1e-6)
    assert validate_centroidal(second)["valid"]
    tail, ref = horizon_contacts(plan, len(plan.wrenches) - 2, 8)
    assert tail.active[-1].sum() == 2
    assert np.allclose(ref[-1], plan.states[-1], atol=1e-12)
    bad = measured.copy()
    bad[2] = 2.0
    with pytest.raises(CentroidalFailure):
        controller.solve(bad, 2)


def test_centroidal_fallback_revalidates_measured_state(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Solver 실패를 주입해 새 상태로 검증한 이전 계획만 사용하는지 확인한다."""
    cfg, contacts, reference, _ = make_walking_problem(
        steps=1, dt=0.15, height_amplitude=0.0
    )
    nominal = CentroidalSolver(len(contacts.times) - 1, cfg, True).solve(
        reference[0], reference, contacts
    )
    controller = CentroidalMPC(nominal, 6)
    first = controller.solve(nominal.initial, 0)

    def fail(*args, **kwargs):
        """외부 solver 실패를 모사한다."""
        raise CentroidalFailure("injected failure")

    monkeypatch.setattr(controller.solver, "solve", fail)
    fallback = controller.solve(first.states[1].copy(), 1)
    assert controller.last_mode == "validated_previous"
    assert controller.failures == 1
    assert validate_centroidal(fallback)["valid"]
    outside = fallback.states[1].copy()
    outside[2] = 3.0
    with pytest.raises(CentroidalFailure):
        controller.solve(outside, 2)
