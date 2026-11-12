"""실제 G1 contact OCP의 actuation, manifold, impact, 해를 검증한다."""

import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

pin = pytest.importorskip("pinocchio")
croc = pytest.importorskip("crocoddyl")
if pin.__version__.startswith("4.1") and croc.__version__.startswith("3.2"):
    pytest.skip(
        "Lab 10은 .venv-ocp의 Pinocchio 4.0 wheel 조합으로 실행합니다.",
        allow_module_level=True,
    )
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from whole_body_demo import simulate_mujoco
from whole_body_ocp import OCPConfig, audit_mass_matrices, solve_problem


@pytest.fixture(
    scope="module",
    params=[(1, 0.01), (2, 0.01), (1, 0.02), (2, 0.02)],
    ids=["one-10ms", "two-10ms", "one-20ms", "two-20ms"],
)
def solved(request: pytest.FixtureRequest) -> tuple:
    """한두 걸음·10/20 ms 문제를 각각 한 번 최적화한다."""
    steps, dt = request.param
    return solve_problem(
        OCPConfig(
            steps=steps,
            dt=dt,
            swing_knots=round(0.6 / dt),
            support_knots=round(0.6 / dt),
        )
    )


def test_floating_base_actuation_and_manifold(solved: tuple) -> None:
    """Root joint 이름과 무관하게 최종 문제의 base 6 DoF가 비구동임을 확인한다."""
    solver, gait, m, *_ = solved
    assert gait.actuation.nu == m.nu == 29
    data = gait.actuation.createData()
    gait.actuation.calc(data, solver.problem.x0, np.ones(29))
    assert np.allclose(data.tau[:6], 0, atol=1e-12)
    assert np.allclose(data.tau[6:], 1, atol=1e-12)
    dx = np.random.default_rng(3).normal(0, 0.01, gait.state.ndx)
    moved = gait.state.integrate(solver.problem.x0, dx)
    assert np.isclose(np.linalg.norm(moved[3:7]), 1, atol=1e-12)
    assert np.allclose(gait.state.diff(solver.problem.x0, moved), dx, atol=1e-10)


def test_converged_feasible_walk(solved: tuple) -> None:
    """Solver convergence와 독립 Dynamics/torque/joint/wrench 검사에 모두 통과한다."""
    report = solved[-1]
    assert report["converged"]
    assert report["model_rollout_valid"]
    assert report["final_foot_error_m"] < 0.001
    assert report["rollout_gap"] < 1e-8


def test_impact_has_no_position_jump_and_stops_contacts(solved: tuple) -> None:
    """Inelastic impact는 q를 바꾸지 않고 양발 접촉 속도를 0으로 만든다."""
    solver, gait, _, _, times, phases, _ = solved
    pm = gait.rmodel
    data = pm.createData()
    impacts = [i for i, phase in enumerate(phases) if phase == "impact"]
    assert len(impacts) in (1, 2)
    for i in impacts:
        assert times[i] == times[i + 1]
        before, after = solver.xs[i], solver.xs[i + 1]
        assert (
            np.linalg.norm(pin.difference(pm, before[: pm.nq], after[: pm.nq])) < 1e-10
        )
        for frame in (gait.lfId, gait.rfId):
            jac = pin.computeFrameJacobian(
                pm, data, after[: pm.nq], frame, pin.LOCAL_WORLD_ALIGNED
            )
            assert np.linalg.norm(jac @ after[pm.nq :]) < 1e-6


def test_contact_dynamics_directional_derivative(solved: tuple) -> None:
    """Analytical action Fx를 manifold perturbation의 유한차분과 비교한다."""
    solver, gait, *_ = solved
    action = solver.problem.runningModels[10]
    data = action.createData()
    x, u = solver.xs[10], solver.us[10]
    action.calc(data, x, u)
    base = data.xnext.copy()
    action.calcDiff(data, x, u)
    fx = data.Fx.copy()
    direction = np.random.default_rng(5).normal(0, 0.01, gait.state.ndx)
    eps = 1e-6
    action.calc(data, gait.state.integrate(x, eps * direction), u)
    numeric = gait.state.diff(base, data.xnext) / eps
    assert np.allclose(numeric, fx @ direction, atol=2e-5, rtol=1e-4)


def test_contact_and_impact_mass_matrix_parity(solved: tuple) -> None:
    """Base 회전과 관절 자세가 달라도 rotor inertia를 한 번만 포함해야 한다."""
    solver, gait, model, original, *_ = solved
    data = mujoco.MjData(model)
    data.qpos[:] = original.qpos
    rng = np.random.default_rng(31)
    for _ in range(3):
        rotation = pin.exp3(rng.normal(0, 0.1, 3))
        xyzw = pin.Quaternion(rotation).coeffs()
        data.qpos[3:7] = xyzw[[3, 0, 1, 2]]
        data.qpos[7:] += rng.normal(0, 0.01, 29)
        errors = audit_mass_matrices(solver.problem, gait, model, data)
        assert max(errors.values()) < 1e-9


def test_actual_mujoco_tracking(solved: tuple, tmp_path: Path) -> None:
    """모델 성공 외에 실제 접촉에서 전도·포화 없이 한두 걸음을 완료한다."""
    solver, gait, model, original, times, *_ = solved
    data = mujoco.MjData(model)
    data.qpos[:] = original.qpos
    data.qvel[:] = original.qvel
    mujoco.mj_forward(model, data)
    report = simulate_mujoco(solver, gait, model, data, times, tmp_path)
    assert report["passed"]
    assert report["clipping_ticks"] == report["flight_ticks"] == 0
    assert report["impact_state_overwrites"] == 0
    assert report["final_foot_gap_m"] < 0.02
