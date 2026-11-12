"""상태·jerk·발 디딤을 함께 푸는 sparse QP와 독립 계획 검증."""

from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter

import numpy as np
import osqp
from scipy import sparse
from walking_model import WalkingSchedule, jerk_matrices, polynomial_state

TOLERANCE = 2e-5


class PlanningError(RuntimeError):
    """유효한 계획을 얻지 못했을 때 발생한다. 실패한 해를 실행에 사용하지 않는다."""


@dataclass
class WalkingPlan:
    """시간 격자, (N+1,2,3) 상태, (N,2) jerk와 발 좌표를 보관한다."""

    schedule: WalkingSchedule
    states: np.ndarray
    jerks: np.ndarray
    feet: np.ndarray
    initial_state: np.ndarray
    landing_bounds: np.ndarray
    optimize_feet: bool
    solve_seconds: float
    iterations: int
    refinement_rounds: int

    @property
    def omega(self) -> float:
        """DCM 변환에 사용하는 LIPM 주파수."""
        return self.schedule.config.omega

    def sample(self, t: float) -> dict[str, np.ndarray]:
        """Polynomial을 평가한다. 보간으로 Dynamics를 바꾸지 않는다."""
        if not np.isfinite(t):
            raise ValueError("조회 시각은 유한해야 합니다.")
        times = self.schedule.times
        t = float(np.clip(t, 0.0, times[-1]))
        k = min(int(np.searchsorted(times, t, side="right") - 1), len(self.jerks) - 1)
        state = polynomial_state(self.states[k], self.jerks[k], t - times[k])
        c, v, a = state.T
        p = c - a / self.omega**2
        return {
            "com": c,
            "velocity": v,
            "acceleration": a,
            "zmp": p,
            "xi": c + v / self.omega,
            "xi_dot": v + a / self.omega,
        }


class _Rows:
    """선형식의 sparse row와 범위를 누적한다."""

    def __init__(self, size: int) -> None:
        """결정변수 차원을 저장한다."""
        self.size = size
        self.rows: list[int] = []
        self.cols: list[int] = []
        self.values: list[float] = []
        self.lower: list[float] = []
        self.upper: list[float] = []

    def add(self, entries: dict[int, float], lower: float, upper: float) -> None:
        """lower <= row @ z <= upper를 추가한다."""
        index = len(self.lower)
        for col, value in entries.items():
            if value:
                self.rows.append(index)
                self.cols.append(col)
                self.values.append(value)
        self.lower.append(lower)
        self.upper.append(upper)

    def matrix(self) -> sparse.csc_matrix:
        """OSQP가 사용하는 CSC 행렬을 만든다."""
        return sparse.csc_matrix(
            (self.values, (self.rows, self.cols)), shape=(len(self.lower), self.size)
        )


def _critical_times(coefficients: list[float], duration: float) -> list[float]:
    """다항식 미분의 실근 중 구간 내부에 있는 값과 양 끝점을 반환한다."""
    coefficients = list(coefficients)
    while coefficients and abs(coefficients[0]) < 1e-12:
        coefficients.pop(0)
    roots = np.roots(coefficients) if len(coefficients) > 1 else []
    return [0.0, duration] + [
        float(r.real) for r in roots if abs(r.imag) < 1e-9 and 0 < r.real < duration
    ]


def interval_extrema(
    plan: WalkingPlan, k: int, axis: int
) -> tuple[list[float], list[float]]:
    """ZMP−지지영역 중심과 속도의 extrema 시각을 구한다."""
    s = plan.schedule
    dt = s.times[k + 1] - s.times[k]
    _, velocity, acceleration = plan.states[k, axis]
    jerk = plan.jerks[k, axis]
    slope = float((s.center_end[k] - s.center_start[k]) @ plan.feet[:, axis] / dt)
    return (
        _critical_times(
            [jerk / 2, acceleration, velocity - jerk / plan.omega**2 - slope], dt
        ),
        _critical_times([jerk, acceleration], dt),
    )


def validate_plan(
    plan: WalkingPlan, tolerance: float = TOLERANCE
) -> dict[str, float | bool]:
    """Solver와 독립적으로 Dynamics, 연속시간 support·속도, 경계·발 제약을 확인한다."""
    s, cfg = plan.schedule, plan.schedule.config
    arrays = (plan.states, plan.jerks, plan.feet, plan.initial_state)
    if (
        not all(np.isfinite(a).all() for a in arrays)
        or np.isnan(plan.landing_bounds).any()
        or np.any(plan.landing_bounds[:, :, 0] > plan.landing_bounds[:, :, 1])
    ):
        return {"valid": False, "finite": False}
    dynamics = support = velocity = acceleration = 0.0
    half = [cfg.half_length, cfg.half_width]
    for k, dt in enumerate(np.diff(s.times)):
        predicted = polynomial_state(plan.states[k], plan.jerks[k], float(dt))
        dynamics = max(dynamics, float(np.max(np.abs(predicted - plan.states[k + 1]))))
        for axis in range(2):
            z_times, v_times = interval_extrema(plan, k, axis)
            for t in z_times:
                state = polynomial_state(plan.states[k], plan.jerks[k], t)
                p = state[axis, 0] - state[axis, 2] / plan.omega**2
                delta = abs(p - s.center(k, t, plan.feet)[axis]) - half[axis]
                support = max(support, float(delta))
            for t in v_times:
                state = polynomial_state(plan.states[k], plan.jerks[k], t)
                velocity = max(velocity, float(abs(state[axis, 1]) - cfg.max_velocity))
        acceleration = max(
            acceleration,
            float(np.max(np.abs(plan.states[k : k + 2, :, 2])) - cfg.max_acceleration),
        )
    offset = np.array([cfg.foot_offset_x, 0.0])
    target = s.final_weights @ s.feet
    final = s.final_weights @ plan.feet
    boundary = max(
        float(np.max(np.abs(plan.states[0] - plan.initial_state))),
        float(np.max(np.abs(plan.states[-1, :, 0] - final - offset))),
        float(np.max(np.abs(plan.states[-1, :, 1:]))),
        float(np.max(np.abs(final - target))),
    )
    foot = float(np.max(np.abs(plan.feet[:2] - s.feet[:2])))
    if not plan.optimize_feet:
        foot = max(foot, float(np.max(np.abs(plan.feet - s.feet))))
    for k, stance in enumerate(s.stance_nodes):
        dx, dy = plan.feet[k + 2] - plan.feet[stance]
        width = dy * (1 if s.landing_sides[k] == 0 else -1)
        foot = max(foot, cfg.min_width - width, width - cfg.max_width)
        if k == cfg.steps:
            foot = max(foot, abs(float(dx)))
        else:
            foot = max(foot, cfg.min_stride - dx, dx - cfg.max_stride)
    foot = max(
        foot,
        float(np.max(plan.landing_bounds[:, :, 0] - plan.feet[2:])),
        float(np.max(plan.feet[2:] - plan.landing_bounds[:, :, 1])),
    )
    result = {
        "dynamics_residual": dynamics,
        "support_violation_m": max(support, 0.0),
        "velocity_violation": max(velocity, 0.0),
        "acceleration_violation": max(acceleration, 0.0),
        "jerk_violation": max(float(np.max(np.abs(plan.jerks)) - cfg.max_jerk), 0.0),
        "boundary_residual": boundary,
        "foot_violation_m": max(float(foot), 0.0),
    }
    result["valid"] = bool(max(result.values()) <= tolerance)
    return result


def plan_cost(plan: WalkingPlan) -> dict[str, float]:
    """4점 Gauss quadrature로 시간 적분 비용을 계산한다. 에너지 지표는 아니다."""
    s, cfg = plan.schedule, plan.schedule.config
    nodes, weights = np.polynomial.legendre.leggauss(4)
    zmp_cost = 0.0
    for k, dt in enumerate(np.diff(s.times)):
        for node, weight in zip(nodes, weights):
            t = float((node + 1) * dt / 2)
            state = polynomial_state(plan.states[k], plan.jerks[k], t)
            zmp = state[:, 0] - state[:, 2] / plan.omega**2
            zmp_cost += float(
                weight * dt / 2 * np.sum((zmp - s.center(k, t, plan.feet)) ** 2)
            )
    jerk_cost = float(np.sum(np.diff(s.times)[:, None] * plan.jerks**2))
    foot_cost = float(np.sum((plan.feet[2:] - s.feet[2:]) ** 2))
    return {
        "zmp_squared_integral": zmp_cost,
        "jerk_squared_integral": jerk_cost,
        "foot_squared_deviation": foot_cost,
        "objective": cfg.zmp_weight * zmp_cost
        + cfg.jerk_weight * jerk_cost
        + cfg.foot_weight * foot_cost,
    }


def solve_walk(
    schedule: WalkingSchedule,
    initial_state: np.ndarray | None = None,
    optimize_feet: bool = False,
    landing_bounds: np.ndarray | None = None,
) -> WalkingPlan:
    """LIPM QP를 풀고 구간 내부 extrema에 제약을 추가한다. 실패하면 PlanningError."""
    s, cfg = schedule, schedule.config
    n, nf = len(s.times) - 1, len(s.feet)
    nx, nu = 6 * (n + 1), 2 * n
    size = nx + nu + 2 * nf
    initial = (
        np.zeros((2, 3))
        if initial_state is None
        else np.asarray(initial_state, dtype=float).copy()
    )
    if initial_state is None:
        initial[:, 0] = s.feet[:2].mean(axis=0) + [cfg.foot_offset_x, 0.0]
    if initial.shape != (2, 3) or not np.isfinite(initial).all():
        raise ValueError(
            "initial_state는 유한한 (2,3) [위치, 속도, 가속도] 배열이어야 합니다."
        )
    bounds = (
        np.tile([-np.inf, np.inf], (nf - 2, 2, 1))
        if landing_bounds is None
        else np.asarray(landing_bounds, dtype=float).copy()
    )
    if (
        bounds.shape != (nf - 2, 2, 2)
        or np.isnan(bounds).any()
        or np.any(bounds[:, :, 0] > bounds[:, :, 1])
    ):
        raise ValueError(
            "landing_bounds는 (착지 수,2,2)의 [lower,upper] 배열이어야 합니다."
        )

    def ix(k: int, axis: int, derivative: int) -> int:
        """상태의 결정변수 인덱스."""
        return 6 * k + 3 * axis + derivative

    def iu(k: int, axis: int) -> int:
        """Jerk의 결정변수 인덱스."""
        return nx + 2 * k + axis

    def ip(k: int, axis: int) -> int:
        """발 좌표의 결정변수 인덱스."""
        return nx + nu + 2 * k + axis

    def zmp_row(k: int, axis: int, t: float) -> dict[int, float]:
        """ZMP에서 움직이는 지지 중심을 뺀 affine 표현(offset은 RHS에 둔다)."""
        dt = s.times[k + 1] - s.times[k]
        weights = s.center_start[k] + (s.center_end[k] - s.center_start[k]) * t / dt
        hg = 1 / cfg.omega**2
        row = {
            ix(k, axis, 0): 1.0,
            ix(k, axis, 1): t,
            ix(k, axis, 2): t * t / 2 - hg,
            iu(k, axis): t**3 / 6 - hg * t,
        }
        row.update({ip(j, axis): -float(w) for j, w in enumerate(weights) if w})
        return row

    def velocity_row(k: int, axis: int, t: float) -> dict[int, float]:
        """구간 내 속도의 선형식."""
        return {ix(k, axis, 1): 1.0, ix(k, axis, 2): t, iu(k, axis): t * t / 2}

    constraints, residuals = _Rows(size), _Rows(size)

    def cost(row: dict[int, float], target: float, weight: float) -> None:
        """weight*(row@z-target)^2를 least-squares 행으로 추가한다."""
        scale = np.sqrt(weight)
        residuals.add(
            {k: v * scale for k, v in row.items()}, target * scale, target * scale
        )

    offset = [cfg.foot_offset_x, 0.0]
    half = [cfg.half_length, cfg.half_width]
    for axis in range(2):
        for d in range(3):
            constraints.add({ix(0, axis, d): 1.0}, initial[axis, d], initial[axis, d])
        for d in (1, 2):
            constraints.add({ix(n, axis, d): 1.0}, 0.0, 0.0)
        terminal = {ip(j, axis): float(w) for j, w in enumerate(s.final_weights) if w}
        target = float(s.final_weights @ s.feet[:, axis])
        constraints.add(terminal, target, target)
        row = {k: -v for k, v in terminal.items()}
        row[ix(n, axis, 0)] = 1.0
        constraints.add(row, offset[axis], offset[axis])
        for j in range(nf):
            if j < 2 or not optimize_feet:
                constraints.add({ip(j, axis): 1.0}, s.feet[j, axis], s.feet[j, axis])
            if j >= 2:
                constraints.add({ip(j, axis): 1.0}, *bounds[j - 2, axis])
                cost({ip(j, axis): 1.0}, s.feet[j, axis], cfg.foot_weight)
        for k in range(n + 1):
            constraints.add({ix(k, axis, 1): 1.0}, -cfg.max_velocity, cfg.max_velocity)
            constraints.add(
                {ix(k, axis, 2): 1.0}, -cfg.max_acceleration, cfg.max_acceleration
            )
    for k, stance in enumerate(s.stance_nodes):
        lower, upper = (
            (0.0, 0.0) if k == cfg.steps else (cfg.min_stride, cfg.max_stride)
        )
        constraints.add({ip(k + 2, 0): 1.0, ip(int(stance), 0): -1.0}, lower, upper)
        sign = 1 if s.landing_sides[k] == 0 else -1
        constraints.add(
            {ip(k + 2, 1): sign, ip(int(stance), 1): -sign},
            cfg.min_width,
            cfg.max_width,
        )
    nodes, weights = np.polynomial.legendre.leggauss(4)
    for k, dt in enumerate(np.diff(s.times)):
        a, b = jerk_matrices(float(dt))
        for axis in range(2):
            for d in range(3):
                row = {ix(k + 1, axis, d): 1.0, iu(k, axis): -b[d]}
                row.update({ix(k, axis, j): -a[d, j] for j in range(3)})
                constraints.add(row, 0.0, 0.0)
            constraints.add({iu(k, axis): 1.0}, -cfg.max_jerk, cfg.max_jerk)
            cost({iu(k, axis): 1.0}, 0.0, cfg.jerk_weight * dt)
            for t in (0.0, dt / 2, dt):
                constraints.add(
                    zmp_row(k, axis, float(t)),
                    offset[axis] - half[axis],
                    offset[axis] + half[axis],
                )
            for node, weight in zip(nodes, weights):
                cost(
                    zmp_row(k, axis, float((node + 1) * dt / 2)),
                    offset[axis],
                    cfg.zmp_weight * weight * dt / 2,
                )
    r = residuals.matrix()
    p = sparse.triu(2 * r.T @ r, format="csc")
    q = -2 * np.asarray(r.T @ np.asarray(residuals.lower)).ravel()
    started, iterations = perf_counter(), 0
    previous = None
    for refinement in range(8):
        solver = osqp.OSQP()
        solver.setup(
            P=p,
            q=q,
            A=constraints.matrix(),
            l=np.asarray(constraints.lower),
            u=np.asarray(constraints.upper),
            verbose=False,
            polishing=True,
            eps_abs=1e-7,
            eps_rel=1e-7,
            max_iter=30000,
            adaptive_rho_interval=50,
        )
        if previous is not None:
            solver.warm_start(x=previous)
        result = solver.solve(raise_error=False)
        iterations += result.info.iter
        if (
            result.info.status != "solved"
            or result.x is None
            or not np.isfinite(result.x).all()
        ):
            raise PlanningError(
                f"QP 실패: {result.info.status}; 제약/시간/초기 상태를 확인하세요."
            )
        previous = result.x
        plan = WalkingPlan(
            s,
            result.x[:nx].reshape(n + 1, 2, 3),
            result.x[nx : nx + nu].reshape(n, 2),
            result.x[nx + nu :].reshape(nf, 2),
            initial,
            bounds,
            optimize_feet,
            perf_counter() - started,
            iterations,
            refinement,
        )
        report = validate_plan(plan)
        if report["valid"]:
            return plan
        # Node 사이 cubic ZMP의 최댓값/최솟값도 제약으로 추가한다.
        for k in range(n):
            for axis in range(2):
                z_times, v_times = interval_extrema(plan, k, axis)
                for t in z_times:
                    constraints.add(
                        zmp_row(k, axis, t),
                        offset[axis] - half[axis],
                        offset[axis] + half[axis],
                    )
                for t in v_times:
                    constraints.add(
                        velocity_row(k, axis, t), -cfg.max_velocity, cfg.max_velocity
                    )
    raise PlanningError(f"연속 구간 검증 실패: {report}")
