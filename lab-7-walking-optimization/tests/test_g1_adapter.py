"""G1 adapter는 선택 의존성 설치 시 검증한다. 외부 mesh 없이 reference 계약을 확인한다."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")
pytest.importorskip("pinocchio")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from g1_walk import LEFT_FOOT, RIGHT_FOOT, OptimizedGait, OptimizedReference
from lipm_optimizer import solve_walk
from walking_model import WalkConfig, build_schedule


def test_optimized_schedule_preserves_contacts_and_landings() -> None:
    """최적화로 달라진 착지가 swing 목표와 다음 지지 phase에 그대로 전달된다."""
    schedule = build_schedule(WalkConfig(steps=2))
    plan = solve_walk(schedule, optimize_feet=True)
    homes = np.c_[schedule.feet[:2], np.full(2, 0.035)]
    gait = OptimizedGait(schedule, plan.feet, homes, np.array([0.035, 0, 0.695]))
    for phase in schedule.phases:
        if phase.swing < 0:
            continue
        name = (LEFT_FOOT, RIGHT_FOOT)[phase.swing]
        ref = gait.reference(phase.end - 1e-9)
        assert ref.swing_foot == name
        assert np.allclose(ref.swing_position[:2], plan.feet[phase.landing], atol=1e-8)
        index = gait.phase_index_at(phase.end + 1e-9)
        assert np.allclose(
            gait.foot_positions(index)[name][:2], plan.feet[phase.landing], atol=1e-12
        )
        assert name in gait.contact_frames(phase.end + 1e-9)
    assert np.isclose(gait.total_duration, schedule.duration)
    assert np.isclose(gait.total_advance, 0.2, atol=2e-5)
    adapted = OptimizedReference(plan)
    for t in np.linspace(0, schedule.duration, 21):
        ref = adapted.reference(float(t))
        assert np.allclose(ref.xi_dot, adapted.omega * (ref.xi - ref.vrp), atol=1e-12)
