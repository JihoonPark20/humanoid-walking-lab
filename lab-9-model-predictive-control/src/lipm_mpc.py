"""W4a: ZMP 입력의 exact LIPM MPC와 backward DCM viability interval."""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter

import numpy as np
import osqp
from scipy import sparse

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "lab-7-walking-optimization" / "src"))
from lipm_optimizer import WalkingPlan


class MPCFailure(RuntimeError):
    """사용할 수 있는 새 계획이나 검증된 정지 입력을 찾지 못했다."""


def lipm_matrices(dt: float, omega: float) -> tuple[np.ndarray, np.ndarray]:
    """상태 [cx,vx,cy,vy], 입력 [zmpx,zmpy]의 exact zero-order hold."""
    if not np.isfinite(dt) or not np.isfinite(omega) or min(dt, omega) <= 0:
        raise ValueError("dt/omega는 양수여야 합니다.")
    c, s = np.cosh(omega * dt), np.sinh(omega * dt)
    one = np.array([[c, s / omega], [omega * s, c]])
    a = np.kron(np.eye(2), one)
    b = np.kron(np.eye(2), np.array([[1 - c], [-omega * s]]))
    return a, b


def support_boxes(plan: WalkingPlan) -> tuple[np.ndarray, np.ndarray]:
    """ZMP를 한 interval 동안 유지할 수 있는 start/end tube의 교집합."""
    s, cfg = plan.schedule, plan.schedule.config
    half = np.array([cfg.half_length, cfg.half_width])
    lower = []
    upper = []
    for k, h in enumerate(np.diff(s.times)):
        start = s.center(k, 0.0, plan.feet)
        end = s.center(k, h, plan.feet)
        lower.append(np.maximum(start, end) - half)
        upper.append(np.minimum(start, end) + half)
    lower, upper = np.asarray(lower), np.asarray(upper)
    if np.any(lower > upper):
        raise ValueError("상수 ZMP의 허용 영역이 비었습니다. 계획 dt를 줄이세요.")
    return lower, upper


def dcm_viability(
    times: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    omega: float,
    terminal_lower: np.ndarray,
    terminal_upper: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """고정 접촉/상수 ZMP 모델에서 최종 capture 영역까지 도달 가능한 DCM 구간."""
    lo = np.zeros((len(times), 2))
    hi = lo.copy()
    lo[-1] = terminal_lower
    hi[-1] = terminal_upper
    for k in range(len(times) - 2, -1, -1):
        decay = np.exp(-omega * (times[k + 1] - times[k]))
        lo[k] = decay * lo[k + 1] + (1 - decay) * lower[k]
        hi[k] = decay * hi[k + 1] + (1 - decay) * upper[k]
    return lo, hi


@dataclass
class LIPMDecision:
    """최적 입력과 horizon state, wall-clock 시간, 허용 오차 진단."""

    states: np.ndarray
    inputs: np.ndarray
    solve_seconds: float
    status: str
    iterations: int
    residual: float
    mode: str = "mpc"


class LIPMMPC:
    """Nominal plan의 미래 contact를 보며 매 측정 시각마다 OSQP를 푼다."""

    def __init__(
        self, plan: WalkingPlan, horizon: int = 20, nominal: LIPMDecision | None = None
    ) -> None:
        """한 번 계산한 backward viability set을 모든 horizon에서 재사용한다."""
        if horizon < 2:
            raise ValueError("horizon은 2 이상이어야 합니다.")
        self.plan, self.horizon, self.nominal = plan, horizon, nominal
        self.lower, self.upper = support_boxes(plan)
        cfg = plan.schedule.config
        final = plan.schedule.final_weights @ plan.feet + [cfg.foot_offset_x, 0]
        self.final_lower = final - [cfg.half_length, cfg.half_width]
        self.final_upper = final + [cfg.half_length, cfg.half_width]
        self.viable_lower, self.viable_upper = dcm_viability(
            plan.schedule.times,
            self.lower,
            self.upper,
            plan.omega,
            self.final_lower,
            self.final_upper,
        )
        self.previous: LIPMDecision | None = None

    def solve(self, measured: np.ndarray, index: int) -> LIPMDecision:
        """측정 상태를 초기 equality로 고정한다. 모든 실제 제약 잔차를 검사한다."""
        started = perf_counter()
        plan = self.plan
        n = self.horizon
        measured = np.asarray(measured, dtype=float)
        if measured.shape != (4,) or not np.isfinite(measured).all() or index < 0:
            raise ValueError("측정 상태/index 오류")
        size = 4 * (n + 1) + 2 * n
        nx = 4 * (n + 1)
        diagonal = np.r_[
            np.tile([200.0, 20.0, 200.0, 20.0], n + 1), np.full(2 * n, 2.0)
        ]
        p = sparse.diags(diagonal, format="lil")
        q = np.zeros(size)
        row = []
        col = []
        value = []
        lo = []
        hi = []

        def add(entries: dict, lower: float, upper: float) -> None:
            """Sparse 선형 제약 한 행을 누적한다."""
            i = len(lo)
            for j, v in entries.items():
                if v:
                    row.append(i)
                    col.append(j)
                    value.append(v)
            lo.append(lower)
            hi.append(upper)

        count = len(self.lower)
        for d in range(4):
            add({d: 1.0}, measured[d], measured[d])
        for k in range(n + 1):
            j = min(index + k, count)
            t = plan.schedule.times[j]
            ref = plan.sample(float(t))
            state = (
                self.nominal.states[j]
                if self.nominal is not None
                else np.array(
                    [
                        ref["com"][0],
                        ref["velocity"][0],
                        ref["com"][1],
                        ref["velocity"][1],
                    ]
                )
            )
            q[4 * k : 4 * k + 4] = -diagonal[4 * k : 4 * k + 4] * state
            if k == n:
                break
            h = plan.schedule.times[j + 1] - t if j < count else plan.schedule.config.dt
            a, b = lipm_matrices(float(h), plan.omega)
            for d in range(4):
                entries = {4 * (k + 1) + d: 1.0}
                entries.update({4 * k + v: -a[d, v] for v in range(4)})
                entries.update({nx + 2 * k + v: -b[d, v] for v in range(2)})
                add(entries, 0.0, 0.0)
            lower = self.lower[j] if j < count else self.final_lower
            upper = self.upper[j] if j < count else self.final_upper
            for d in range(2):
                add({nx + 2 * k + d: 1.0}, lower[d], upper[d])
            nominal = np.clip(
                self.nominal.inputs[min(j, count - 1)]
                if self.nominal is not None and j < count
                else ref["zmp"],
                lower,
                upper,
            )
            q[nx + 2 * k : nx + 2 * k + 2] = -2 * nominal
        terminal = min(index + n, count)
        for axis in range(2):
            indices = [4 * n + 2 * axis, 4 * n + 2 * axis + 1]
            c = np.array([1.0, 1 / plan.omega])
            add(
                dict(zip(indices, c)),
                self.viable_lower[terminal, axis],
                self.viable_upper[terminal, axis],
            )
            xi = (
                self.nominal.states[terminal, 2 * axis]
                + self.nominal.states[terminal, 2 * axis + 1] / plan.omega
                if self.nominal is not None
                else plan.sample(float(plan.schedule.times[terminal]))["xi"][axis]
            )
            for i in range(2):
                q[indices[i]] -= 2000 * xi * c[i]
                for j in range(2):
                    p[indices[i], indices[j]] += 2000 * c[i] * c[j]
        a = sparse.csc_matrix((value, (row, col)), shape=(len(lo), size))
        solver = osqp.OSQP()
        solver.setup(
            P=sparse.triu(p, format="csc"),
            q=q,
            A=a,
            l=np.asarray(lo),
            u=np.asarray(hi),
            verbose=False,
            eps_abs=1e-7,
            eps_rel=1e-7,
            max_iter=15000,
            polishing=True,
            adaptive_rho_interval=50,
        )
        if self.previous is not None:
            old = self.previous
            solver.warm_start(
                x=np.r_[
                    np.vstack([old.states[1:], old.states[-1]]).ravel(),
                    np.vstack([old.inputs[1:], old.inputs[-1]]).ravel(),
                ]
            )
        solved = solver.solve(raise_error=False)
        if solved.info.status != "solved" or solved.x is None:
            raise MPCFailure(f"LIPM MPC {solved.info.status}")
        actual = a @ solved.x
        residual = float(
            np.maximum(
                np.maximum(np.asarray(lo) - actual, actual - np.asarray(hi)), 0
            ).max()
        )
        if not np.isfinite(solved.x).all() or residual > 2e-5:
            raise MPCFailure(f"LIPM MPC residual {residual}")
        result = LIPMDecision(
            solved.x[:nx].reshape(n + 1, 4),
            solved.x[nx:].reshape(n, 2),
            perf_counter() - started,
            solved.info.status,
            solved.info.iter,
            residual,
        )
        self.previous = result
        return result

    def braking_input(self, measured: np.ndarray, index: int) -> np.ndarray:
        """현재 접촉을 유지하며 DCM capture가 가능한 경우에만 정지 ZMP를 반환한다."""
        xi = measured[[0, 2]] + measured[[1, 3]] / self.plan.omega
        j = min(index, len(self.lower) - 1)
        if (
            not np.isfinite(xi).all()
            or np.any(xi < self.lower[j])
            or np.any(xi > self.upper[j])
        ):
            raise MPCFailure("현재 지지 영역에서 capture할 수 없습니다.")
        return xi
