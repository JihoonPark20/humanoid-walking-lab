"""축소 모델의 물리식, 제약, 저장/재생과 실패 처리를 검증한다."""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from lipm_optimizer import (
    PlanningError,
    WalkingPlan,
    interval_extrema,
    plan_cost,
    solve_walk,
    validate_plan,
)
from offline_walk import compare_timings
from walking_artifacts import load_plan, save_plan
from walking_model import WalkConfig, build_schedule, jerk_matrices, polynomial_state


@pytest.fixture(scope="module")
def fixed() -> WalkingPlan:
    """같은 QP를 반복 계산하지 않도록 기준 계획을 공유한다."""
    return solve_walk(build_schedule(WalkConfig()))


def test_exact_jerk_integration() -> None:
    """상수 jerk 적분이 닫힌 형태와 일치하고 합성 시간을 보존한다."""
    state = np.array([[0.2, 0.3, -0.4], [0.1, -0.2, 0.5]])
    jerk = np.array([1.0, -2.0])
    t = 0.23
    expected = np.array(
        [
            state[:, 0] + state[:, 1] * t + state[:, 2] * t * t / 2 + jerk * t**3 / 6,
            state[:, 1] + state[:, 2] * t + jerk * t * t / 2,
            state[:, 2] + jerk * t,
        ]
    ).T
    assert np.allclose(polynomial_state(state, jerk, t), expected, atol=1e-14)
    assert np.allclose(
        polynomial_state(polynomial_state(state, jerk, t), jerk, t),
        polynomial_state(state, jerk, 2 * t),
        atol=1e-14,
    )
    with pytest.raises(ValueError):
        jerk_matrices(0)


@pytest.mark.parametrize("first", [0, 1])
def test_schedule_contacts_and_endpoint(first: int) -> None:
    """발 전환은 착지 이후이고 모든 phase 경계가 적분 격자에 있다."""
    schedule = build_schedule(WalkConfig(first_swing=first, dt=0.07))
    assert sum(p.swing >= 0 for p in schedule.phases) == 5
    for i, phase in enumerate(schedule.phases):
        assert np.min(abs(schedule.times - phase.start)) < 1e-12
        if phase.swing >= 0:
            assert schedule.phases[i + 1].feet[phase.swing] == phase.landing
            assert phase.center_start[phase.feet[1 - phase.swing]] == 1
    assert np.allclose(schedule.final_weights @ schedule.feet, [0.4, 0], atol=1e-12)
    assert np.max(np.diff(schedule.times)) <= 0.07 + 1e-12


def test_continuous_lipm_and_dcm(fixed: WalkingPlan) -> None:
    """독립 dense sampling으로 ZMP, DCM Dynamics와 정지 경계를 검증한다."""
    assert validate_plan(fixed)["valid"]
    s = fixed.schedule
    for t in np.linspace(0, s.duration, 2001):
        value = fixed.sample(float(t))
        assert np.allclose(
            value["xi_dot"], fixed.omega * (value["xi"] - value["zmp"]), atol=1e-12
        )
        k = min(np.searchsorted(s.times, t, side="right") - 1, len(fixed.jerks) - 1)
        delta = value["zmp"] - s.center(k, t - s.times[k], fixed.feet)
        assert np.all(
            abs(delta) <= np.array([s.config.half_length, s.config.half_width]) + 2e-5
        )
    final = fixed.sample(s.duration)
    assert np.allclose(final["velocity"], 0, atol=2e-5)
    assert np.allclose(final["acceleration"], 0, atol=2e-5)
    assert np.allclose(fixed.sample(-1)["com"], fixed.initial_state[:, 0], atol=2e-5)


def test_optimized_feet_improve_same_problem(fixed: WalkingPlan) -> None:
    """고정 해를 포함하는 feasible set이므로 같은 비용에서 악화되면 안 된다."""
    optimized = solve_walk(fixed.schedule, optimize_feet=True)
    assert validate_plan(optimized)["valid"]
    assert plan_cost(optimized)["objective"] <= plan_cost(fixed)["objective"] + 1e-5
    assert np.max(abs(optimized.feet - fixed.feet)) > 1e-3
    assert np.allclose(
        optimized.schedule.final_weights @ optimized.feet, [0.4, 0], atol=2e-5
    )


def test_landing_box_and_infeasibility(fixed: WalkingPlan) -> None:
    """강제 착지 영역이 QP에 반영되며, 고정 발걸음과 충돌하면 실패한다."""
    bounds = np.tile([-np.inf, np.inf], (len(fixed.feet) - 2, 2, 1))
    bounds[0, 0] = [0.12, 0.125]
    plan = solve_walk(fixed.schedule, optimize_feet=True, landing_bounds=bounds)
    assert 0.12 - 2e-5 <= plan.feet[2, 0] <= 0.125 + 2e-5
    with pytest.raises(PlanningError):
        solve_walk(fixed.schedule, landing_bounds=bounds)


def test_impossible_initial_state(fixed: WalkingPlan) -> None:
    """초기 ZMP가 지지 영역 밖이면 실패를 숨기지 않는다."""
    initial = fixed.initial_state.copy()
    initial[1, 0] = 1.0
    with pytest.raises(PlanningError):
        solve_walk(fixed.schedule, initial)


def test_roundtrip_and_corruption(fixed: WalkingPlan, tmp_path: Path) -> None:
    """모든 상태·발 좌표가 왕복 저장되고 변조된 Dynamics를 거부한다."""
    path = tmp_path / "plan.npz"
    save_plan(fixed, path)
    restored = load_plan(path)
    assert np.array_equal(restored.feet, fixed.feet)
    for t in [0.0, 0.723, 3.17, 8.0]:
        assert np.allclose(
            restored.sample(t)["com"], fixed.sample(t)["com"], atol=1e-14
        )
    with np.load(path, allow_pickle=False) as original:
        data = {key: original[key] for key in original.files}
    data["states"][10, 0, 0] += 0.01
    np.savez_compressed(path, **data)
    with pytest.raises(PlanningError):
        load_plan(path)


def test_interior_extremum_is_checked(fixed: WalkingPlan) -> None:
    """양 끝점만 검사하면 놓치는 내부 속도 극값을 실제 validator가 찾는다."""
    states = fixed.states.copy()
    jerks = fixed.jerks.copy()
    states[0, 0, 1] = 0.799
    states[0, 0, 2] = 0.2
    jerks[0, 0] = -8.0
    altered = replace(fixed, states=states, jerks=jerks)
    _, times = interval_extrema(altered, 0, 0)
    assert any(abs(t - 0.025) < 1e-12 for t in times)
    assert validate_plan(altered)["velocity_violation"] > 0.001


def test_timing_comparison_records_failure() -> None:
    """후보 중 실패를 기록하며 유효한 후보의 명시적 score로 선택한다."""
    plan, trials = compare_timings(WalkConfig(steps=2), [-0.1, 0.65, 0.75])
    assert trials[0]["feasible"] is False
    valid = [r for r in trials if r["feasible"]]
    best = min(valid, key=lambda r: r["score"])
    assert plan.schedule.config.single_support == best["single_support_s"]
    assert np.isclose(best["score"], best["objective"] + 0.05 * best["duration_s"])


@pytest.mark.parametrize(
    "kwargs", [{"steps": 0}, {"dt": 0.001}, {"com_height": 0}, {"step_width": 0.4}]
)
def test_invalid_configuration(kwargs: dict) -> None:
    """설정 오류는 solver 이전에 명확히 실패한다."""
    with pytest.raises(ValueError):
        WalkConfig(**kwargs)


def test_zmp_peak_between_nodes_is_rejected(fixed: WalkingPlan) -> None:
    """두 node의 ZMP는 범위 안이어도 구간 내부에서 벗어나면 검증 실패다."""
    states = fixed.states.copy()
    jerks = fixed.jerks.copy()
    cfg = fixed.schedule.config
    p0 = cfg.foot_offset_x + cfg.half_length - 5e-5
    states[0, 0] = [p0 - 0.4 / fixed.omega**2, 0.01, -0.4]
    jerks[0, 0] = 0.0
    altered = replace(fixed, states=states, jerks=jerks)
    for t in [0.0, 0.05]:
        state = polynomial_state(states[0], jerks[0], t)
        assert (
            state[0, 0] - state[0, 2] / fixed.omega**2
            < cfg.foot_offset_x + cfg.half_length
        )
    assert validate_plan(altered)["support_violation_m"] > 7e-5
