"""Exact centroidal 적분, wrench 제약, NLP와 저장 결과의 독립 검사."""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from centroidal_io import load_centroidal, make_walking_problem, save_centroidal
from centroidal_model import (
    CentroidalConfig,
    Contacts,
    centroidal_step,
    validate_centroidal,
    wrench_violation,
)
from centroidal_optimizer import CentroidalFailure, CentroidalSolver


def test_exact_integration_and_subdivision() -> None:
    """일정 wrench에서 subdivision과 합성한 exact 적분이 일치한다."""
    cfg = CentroidalConfig()
    x = np.array([0.02, 0, 0.66, 1.0, -0.5, 0.2, 0.2, 0.1, -0.1])
    u = np.array(
        [[4.0, -2.0, 150.0, 0.1, 0.2, 0.1], [-1.0, 3.0, 180.0, -0.1, 0.3, -0.1]]
    )
    r = np.array([[0, 0.1, 0], [0, -0.1, 0]])
    full = centroidal_step(x, u, r, 0.2, cfg)
    half = centroidal_step(centroidal_step(x, u, r, 0.1, cfg), u, r, 0.1, cfg)
    assert np.allclose(full, half, atol=1e-12)

    # 각운동량 RHS를 작은 RK4로 독립 적분한다.
    def rhs(value: np.ndarray) -> np.ndarray:
        """연속 centroidal 미분식."""
        return np.r_[
            value[3:6] / cfg.mass,
            u[:, :3].sum(axis=0) + [0, 0, -cfg.mass * cfg.gravity],
            (np.cross(r - value[:3], u[:, :3]) + u[:, 3:]).sum(axis=0),
        ]

    value = x.copy()
    h = 0.001
    for _ in range(200):
        a = rhs(value)
        b = rhs(value + h * a / 2)
        c = rhs(value + h * b / 2)
        d = rhs(value + h * c)
        value += h * (a + 2 * b + 2 * c + d) / 6
    assert np.allclose(value, full, atol=1e-10)


def test_friction_cop_and_inactive_force() -> None:
    """마찰, CoP, 비접촉 wrench 위반을 각각 잡는다."""
    cfg = CentroidalConfig()
    u = np.zeros((2, 6))
    u[:, 2] = 100
    assert wrench_violation(u, np.ones(2), cfg) == 0
    u[0, 0] = 61
    assert wrench_violation(u, np.ones(2), cfg) > 0
    u[0, 0] = 0
    u[0, 3] = 3
    assert wrench_violation(u, np.ones(2), cfg) > 0
    u[0, 3] = 0
    assert wrench_violation(u, np.array([0, 1]), cfg) > 0


def test_static_nlp_and_infeasible_initial() -> None:
    """정지 해의 중력 평형과 불가능한 초기 높이의 거부를 검증한다."""
    cfg = CentroidalConfig()
    n = 8
    contacts = Contacts(
        np.linspace(0, 1, n + 1),
        np.tile([[0, 0.09, 0], [0, -0.09, 0]], (n, 1, 1)),
        np.ones((n, 2)),
    )
    reference = np.tile(np.r_[0, 0, 0.66, np.zeros(6)], (n + 1, 1))
    solver = CentroidalSolver(n, cfg, True)
    plan = solver.solve(reference[0], reference, contacts)
    assert validate_centroidal(plan)["valid"]
    assert np.allclose(
        plan.wrenches[:, :, 2].sum(axis=1), cfg.mass * cfg.gravity, atol=1e-4
    )
    bad = reference[0].copy()
    bad[2] = 2
    with pytest.raises(CentroidalFailure):
        solver.solve(bad, reference, contacts)


def test_walking_height_and_roundtrip(tmp_path: Path) -> None:
    """접촉 전환과 높이 변화를 실제 NLP로 풀고 변조된 상태를 검출한다."""
    cfg, contacts, reference, _ = make_walking_problem(steps=1, dt=0.15)
    plan = CentroidalSolver(len(contacts.times) - 1, cfg, True).solve(
        reference[0], reference, contacts
    )
    assert validate_centroidal(plan)["valid"]
    assert np.ptp(plan.states[:, 2]) > 0.01
    save_centroidal(plan, tmp_path)
    loaded = load_centroidal(tmp_path / "plan.npz")
    assert np.array_equal(loaded.states, plan.states)
    altered = plan.states.copy()
    altered[3, 0] += 0.1
    assert not validate_centroidal(replace(plan, states=altered))["valid"]


def test_saved_metadata_and_nonfinite_plan(tmp_path: Path) -> None:
    """저장 파일에 solver/설정을 보존하며 non-finite 값은 성공 처리하지 않는다."""
    cfg=CentroidalConfig();n=2
    contacts=Contacts(np.array([0.,.1,.2]),np.tile([[0,.09,0],[0,-.09,0]],(n,1,1)),np.ones((n,2)))
    ref=np.tile(np.r_[0,0,.66,np.zeros(6)],(n+1,1))
    plan=CentroidalSolver(n,cfg,True).solve(ref[0],ref,contacts)
    states=plan.states.copy();states[1,0]=np.nan
    assert not validate_centroidal(replace(plan,states=states))['valid']
    save_centroidal(plan,tmp_path)
    restored=load_centroidal(tmp_path/'plan.npz')
    assert restored.status==plan.status
    assert restored.config==cfg
