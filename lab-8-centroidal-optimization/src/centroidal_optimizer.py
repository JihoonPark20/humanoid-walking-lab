"""CasADi/IPOPT Direct Multiple Shooting. 같은 solver graph를 MPC에서 재사용한다."""

from __future__ import annotations

from time import perf_counter

import casadi as ca
import numpy as np
from centroidal_model import (
    CentroidalConfig,
    CentroidalPlan,
    Contacts,
    validate_centroidal,
)


class CentroidalFailure(RuntimeError):
    """Solver 실패 또는 독립 제약 검증 실패. 해를 제어 입력으로 사용하지 않는다."""


class CentroidalSolver:
    """고정 horizon 크기에서 초기 상태·reference·접촉을 parameter로 갱신한다."""

    def __init__(
        self,
        intervals: int,
        config: CentroidalConfig,
        terminal_stop: bool = False,
        max_iterations: int = 300,
    ) -> None:
        """Sparse NLP와 reusable IPOPT graph를 한 번만 만든다."""
        if intervals < 1 or max_iterations < 1:
            raise ValueError("양의 horizon/iteration 수가 필요합니다.")
        self.n, self.config, self.terminal_stop = intervals, config, terminal_stop
        n, cfg = intervals, config
        opt = ca.Opti()
        self.opt = opt
        x = opt.variable(9, n + 1)
        u = opt.variable(12, n)
        self.x, self.u = x, u
        self.initial = opt.parameter(9)
        self.ref = opt.parameter(9, n + 1)
        self.positions = opt.parameter(6, n)
        self.active = opt.parameter(2, n)
        self.dt = opt.parameter(n)
        self.u_ref = opt.parameter(12, n)
        self.mass = opt.parameter()
        mass = self.mass
        opt.set_value(self.mass, cfg.mass)
        opt.subject_to(x[:, 0] == self.initial)
        opt.subject_to(opt.bounded(cfg.min_height, x[2, :], cfg.max_height))
        opt.subject_to(
            opt.bounded(-mass * cfg.max_velocity, x[3:6, :], mass * cfg.max_velocity)
        )
        opt.subject_to(
            opt.bounded(-cfg.max_angular_momentum, x[6:9, :], cfg.max_angular_momentum)
        )
        if terminal_stop:
            opt.subject_to(x[:, -1] == self.ref[:, -1])
        scale = ca.DM([30.0, 30.0, 70.0, 0.6, 0.6, 1.0, 3.0, 3.0, 3.0])
        cost = 0
        for k in range(n):
            h = self.dt[k]
            force = u[:3, k] + u[6:9, k]
            acc = force / mass + ca.DM([0, 0, -cfg.gravity])
            mean_c = x[:3, k] + h * x[3:6, k] / (2 * mass) + h * h * acc / 6
            angular_rate = ca.DM.zeros(3)
            for side in range(2):
                f = u[side * 6 : side * 6 + 3, k]
                tau = u[side * 6 + 3 : side * 6 + 6, k]
                r = self.positions[side * 3 : side * 3 + 3, k]
                angular_rate += ca.cross(r - mean_c, f) + tau
                opt.subject_to(
                    opt.bounded(0, f[2], self.active[side, k] * cfg.max_normal_force)
                )
                for axis in (0, 1):
                    opt.subject_to(
                        opt.bounded(-cfg.friction * f[2], f[axis], cfg.friction * f[2])
                    )
                for axis, radius in enumerate(
                    (cfg.half_width, cfg.half_length, cfg.yaw_radius)
                ):
                    opt.subject_to(
                        opt.bounded(-radius * f[2], tau[axis], radius * f[2])
                    )
            predicted = ca.vertcat(
                x[:3, k] + h * x[3:6, k] / mass + h * h * acc / 2,
                x[3:6, k] + h * mass * acc,
                x[6:9, k] + h * angular_rate,
            )
            opt.subject_to(x[:, k + 1] == predicted)
            mid_z = x[2, k] + 0.5 * h * x[5, k] / mass + h * h * acc[2] / 8
            opt.subject_to(opt.bounded(cfg.min_height, mid_z, cfg.max_height))
            cost += h * (
                ca.sumsqr(scale * (x[:, k] - self.ref[:, k]))
                + 0.002 * ca.sumsqr((u[:, k] - self.u_ref[:, k]) / (mass * cfg.gravity))
            )
        cost += 20 * ca.sumsqr(scale * (x[:, -1] - self.ref[:, -1]))
        opt.minimize(cost)
        opt.solver(
            "ipopt",
            {"expand": True, "print_time": False},
            {
                "print_level": 0,
                "sb": "yes",
                "max_iter": max_iterations,
                "tol": 1e-7,
                "acceptable_tol": 1e-6,
                "warm_start_init_point": "yes",
            },
        )

    def solve(
        self,
        initial: np.ndarray,
        reference: np.ndarray,
        contacts: Contacts,
        warm_start: CentroidalPlan | None = None,
    ) -> CentroidalPlan:
        """측정 초기 상태와 새 접촉으로 재풀이한 뒤 독립 검증을 통과한 해만 반환한다."""
        n, cfg = self.n, self.config
        initial = np.asarray(initial, dtype=float)
        reference = np.asarray(reference, dtype=float)
        if (
            initial.shape != (9,)
            or reference.shape != (n + 1, 9)
            or len(contacts.times) != n + 1
        ):
            raise ValueError("초기 상태/reference/horizon 크기가 맞지 않습니다.")
        if not np.isfinite(initial).all() or not np.isfinite(reference).all():
            raise ValueError("상태는 유한해야 합니다.")
        opt = self.opt
        opt.set_value(self.mass, cfg.mass)
        opt.set_value(self.initial, initial)
        opt.set_value(self.ref, reference.T)
        opt.set_value(self.positions, contacts.positions.reshape(n, 6).T)
        opt.set_value(self.active, contacts.active.T)
        opt.set_value(self.dt, np.diff(contacts.times))
        nominal = np.zeros((n, 2, 6))
        nominal[:, :, 2] = (
            contacts.active
            / contacts.active.sum(axis=1)[:, None]
            * cfg.mass
            * cfg.gravity
        )
        opt.set_value(self.u_ref, nominal.reshape(n, 12).T)
        if warm_start is not None and warm_start.states.shape == (n + 1, 9):
            opt.set_initial(self.x, warm_start.states.T)
            opt.set_initial(self.u, warm_start.wrenches.reshape(n, 12).T)
        else:
            opt.set_initial(self.x, reference.T)
            opt.set_initial(self.u, nominal.reshape(n, 12).T)
        started = perf_counter()
        try:
            solution = opt.solve()
        except RuntimeError as error:
            raise CentroidalFailure(
                f"IPOPT: {opt.stats().get('return_status', 'failed')}"
            ) from error
        stats = solution.stats()
        plan = CentroidalPlan(
            cfg,
            contacts,
            solution.value(self.x).reshape(9, n + 1).T,
            solution.value(self.u).reshape(12, n).T.reshape(n, 2, 6),
            reference.copy(),
            initial.copy(),
            self.terminal_stop,
            perf_counter() - started,
            int(stats["iter_count"]),
            str(stats["return_status"]),
            float(solution.value(opt.f)),
        )
        report = validate_centroidal(plan)
        if not report["valid"]:
            raise CentroidalFailure(f"독립 검증 실패: {report}")
        return plan
