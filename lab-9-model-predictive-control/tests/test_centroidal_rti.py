"""RTI의 단일 QP, preparation/feedback 분리와 비선형 제약 검증."""

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
ROOT = Path(__file__).resolve().parents[2]
for lab in ("lab-7-walking-optimization", "lab-8-centroidal-optimization"):
    sys.path.insert(0, str(ROOT / lab / "src"))
from centroidal_io import make_walking_problem
from centroidal_model import CentroidalPlan, validate_centroidal
from centroidal_optimizer import CentroidalFailure, CentroidalSolver
from centroidal_rti import CentroidalRTI
from mpc_experiment import run_centroidal


@pytest.fixture(scope="module")
def nominal() -> CentroidalPlan:
    """접촉 전환이 있는 실제 offline 계획을 공유한다."""
    cfg, contacts, ref, _ = make_walking_problem(
        steps=1, dt=0.1, height_amplitude=0.015
    )
    return CentroidalSolver(len(contacts.times) - 1, cfg, True).solve(
        ref[0], ref, contacts
    )


def test_preparation_feedback_one_qp(
    nominal: CentroidalPlan, monkeypatch: pytest.MonkeyPatch
) -> None:
    """준비에는 QP 풀이가 없고 새 측정 후 정확히 한 번만 푼다. NLP fallback은 금지한다."""
    controller = CentroidalRTI(nominal, 8)

    def forbidden(*args: object, **kwargs: object) -> None:
        """RTI가 full NLP를 호출하면 실패한다."""
        pytest.fail("RTI must not call IPOPT")

    monkeypatch.setattr(controller.solver, "solve", forbidden)
    controller.prepare(0)
    assert controller.qp_solve_calls == 0
    measured = nominal.initial.copy()
    measured[4] += 0.1
    result = controller.feedback(measured)
    assert controller.qp_solve_calls == 1 and result.iterations == 1
    assert np.allclose(result.states[0], measured, atol=1e-12)
    assert validate_centroidal(result)["valid"]
    assert controller.last_seconds >= controller.feedback_seconds > 0
    with pytest.raises(RuntimeError):
        controller.feedback(measured)


def test_constraint_linearization_directional_derivative(
    nominal: CentroidalPlan,
) -> None:
    """같은 NLP graph의 Jacobian과 중앙 유한차분이 일치한다."""
    controller = CentroidalRTI(nominal, 8)
    controller.prepare(0)
    z = np.r_[nominal.states[:9].ravel(), nominal.wrenches[:8].ravel()]
    direction = np.random.default_rng(22).normal(size=len(z))
    h, _, jac, *_ = controller.linearize(z, controller.parameters)
    eps = 1e-6
    gp = controller.linearize(z + eps * direction, controller.parameters)[3]
    gm = controller.linearize(z - eps * direction, controller.parameters)[3]
    assert np.allclose(
        np.asarray((gp - gm) / (2 * eps)).ravel(),
        np.asarray(jac @ direction).ravel(),
        atol=2e-7,
        rtol=1e-6,
    )
    assert np.linalg.eigvalsh(np.asarray(h)).min() > 0


def test_rti_contact_transitions_and_impulse(nominal: CentroidalPlan) -> None:
    """외란과 SS/DS 전환을 포함한 전체 보행에서 single-SQP만으로 복구한다."""
    report, _ = run_centroidal(nominal, "rti", horizon=12)
    assert report["passed"] and report["push_applied"]
    assert report["validated_fallbacks"] == report["solver_failures"] == 0
    assert report["rti"]["qp_solve_calls"] == len(nominal.wrenches)
    assert max(r["linearization_gap"] for r in report["rti"]["updates"]) < 0.01


def test_rti_rejects_infeasible_initial_state(nominal: CentroidalPlan) -> None:
    """QP infeasibility를 숨기거나 full NLP로 재시도하지 않는다."""
    controller = CentroidalRTI(nominal, 8)
    bad = nominal.initial.copy()
    bad[2] = 2.0
    with pytest.raises(CentroidalFailure):
        controller.solve(bad, 0)
    assert controller.failures == controller.qp_solve_calls == 1


def test_rti_validated_fallback(
    nominal: CentroidalPlan, monkeypatch: pytest.MonkeyPatch
) -> None:
    """QP 실패 시 측정값에서 다시 검증한 이전 입력만 사용하고 재풀이하지 않는다."""
    controller = CentroidalRTI(nominal, 8)
    first = controller.solve(nominal.initial, 0)
    controller.prepare(1)

    def failed_qp(**kwargs: object) -> SimpleNamespace:
        """OSQP numerical failure를 주입한다."""
        return SimpleNamespace(
            info=SimpleNamespace(iter=10, status="primal infeasible"), x=None
        )

    monkeypatch.setattr(controller.qp, "solve", failed_qp)
    fallback = controller.feedback(first.states[1].copy())
    assert controller.last_mode == "validated_previous"
    assert controller.qp_solve_calls == 2 and controller.failures == 1
    assert validate_centroidal(fallback)["valid"]
