"""Centroidal MPC의 single-SQP RTI: preparation → 측정 갱신 → QP 한 번.

기존 NLP의 quadratic tracking objective와 constraint graph를 미분한다.
QP 후보의 비선형 rollout을 검증하며 full NLP로 재시도하지 않는다.
"""

from __future__ import annotations

import sys
from pathlib import Path
from time import perf_counter

import casadi as ca
import numpy as np
import osqp
from scipy import sparse

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "lab-8-centroidal-optimization" / "src"))
from centroidal_model import CentroidalPlan, centroidal_step, validate_centroidal
from centroidal_mpc import CentroidalMPC, horizon_contacts
from centroidal_optimizer import CentroidalFailure


class CentroidalRTI(CentroidalMPC):
    """Gauss–Newton SQP 한 회와 nonlinear rollout 검증을 수행하는 교육용 RTI."""

    def __init__(self, nominal: CentroidalPlan, horizon: int = 12) -> None:
        """NLP를 풀지 않고 기존 graph에서 objective Hessian과 제약 Jacobian을 만든다."""
        super().__init__(nominal, horizon)
        opt = self.solver.opt
        hessian, gradient = ca.hessian(opt.f, opt.x)
        self.linearize = ca.Function(
            "rti_linearize",
            [opt.x, opt.p],
            [hessian, gradient, ca.jacobian(opt.g, opt.x), opt.g, opt.lbg, opt.ubg],
        )
        self.objective = ca.Function("rti_objective", [opt.x, opt.p], [opt.f])
        cfg = nominal.config
        # p와 wrench를 질량·무게로 정규화해 QP conditioning을 개선한다.
        self.scale = np.r_[
            np.tile(
                [1.0, 1.0, 1.0, cfg.mass, cfg.mass, cfg.mass, 1.0, 1.0, 1.0],
                horizon + 1,
            ),
            np.full(12 * horizon, cfg.mass * cfg.gravity),
        ]
        self.qp_solve_calls = 0
        self.preparation_seconds = self.feedback_seconds = 0.0
        self.qp_iterations = 0
        self.linearization_gap = self.qp_residual = 0.0
        self.prepared_index: int | None = None
        self.ready = False

    def update_mass(self, mass: float) -> None:
        """모델 parameter가 바뀌면 이전 preparation의 선형화를 사용할 수 없다."""
        super().update_mass(mass)
        self.ready = False

    def prepare(self, index: int) -> None:
        """측정 전 선형화·QP factorization을 준비한다. 아직 QP를 풀지 않는다."""
        if not isinstance(index, (int, np.integer)) or index < 0:
            raise ValueError("index는 0 이상의 정수여야 합니다.")
        started = perf_counter()
        self.ready = False
        contacts, reference = horizon_contacts(self.nominal, index, self.horizon)
        cfg, n, graph = self.nominal.config, self.horizon, self.solver
        nominal_u = np.zeros((n, 2, 6))
        nominal_u[:, :, 2] = (
            contacts.active
            / contacts.active.sum(axis=1)[:, None]
            * cfg.mass
            * cfg.gravity
        )
        shift = index - self.last_index if self.last_index is not None else n
        if self.previous is not None and 0 <= shift < n:
            predicted = self.previous.states[shift].copy()
            seed = self.shifted_plan(predicted, index, contacts, reference)
            warm_u = seed.wrenches
            # 새 tail은 중력 지지 입력으로 채워 이전 마지막 힘의 편향을 누적하지 않는다.
            if shift:
                warm_u[-shift:] = nominal_u[-shift:]
        else:
            predicted = reference[0].copy()
            warm_u = nominal_u.copy()
            count = min(n, max(0, len(self.nominal.wrenches) - index))
            warm_u[:count] = self.nominal.wrenches[index : index + count]
        warm_u *= contacts.active[:, :, None]
        states = [predicted]
        for k, dt in enumerate(np.diff(contacts.times)):
            states.append(
                centroidal_step(
                    states[-1], warm_u[k], contacts.positions[k], float(dt), cfg
                )
            )
        seed_z = np.r_[np.asarray(states).ravel(), warm_u.ravel()]
        opt = graph.opt
        opt.set_value(graph.mass, cfg.mass)
        opt.set_value(graph.initial, predicted)
        opt.set_value(graph.ref, reference.T)
        opt.set_value(graph.positions, contacts.positions.reshape(n, 6).T)
        opt.set_value(graph.active, contacts.active.T)
        opt.set_value(graph.dt, np.diff(contacts.times))
        opt.set_value(graph.u_ref, nominal_u.reshape(n, 12).T)
        parameters = np.asarray(opt.value(opt.p)).ravel()
        hessian, gradient, jac, value, lower, upper = self.linearize(seed_z, parameters)
        scaling = sparse.diags(self.scale)
        jac = jac.sparse() @ scaling
        # 행별 단위 차이를 줄인다. 초기 상태 행은 feedback에서 같은 scale을 사용한다.
        row_scale = 1 / np.maximum(
            np.asarray(abs(jac).max(axis=1).toarray()).ravel(), 1.0
        )
        matrix = (sparse.diags(row_scale) @ jac).tocsc()
        offset = matrix @ (seed_z / self.scale) - row_scale * np.asarray(value).ravel()
        lo = row_scale * np.asarray(lower).ravel() + offset
        hi = row_scale * np.asarray(upper).ravel() + offset
        hessian = hessian.sparse()
        p = sparse.triu(scaling @ hessian @ scaling, format="csc")
        q = self.scale * (np.asarray(gradient).ravel() - hessian @ seed_z)
        qp = osqp.OSQP()
        qp.setup(
            P=p,
            q=q,
            A=matrix,
            l=lo,
            u=hi,
            verbose=False,
            eps_abs=1e-7,
            eps_rel=1e-7,
            max_iter=15000,
            polishing=True,
        )
        qp.warm_start(x=seed_z / self.scale)
        self.qp, self.matrix = qp, matrix
        self.lower, self.upper, self.row_scale = lo, hi, row_scale
        self.contacts, self.reference = contacts, reference
        self.parameters = parameters
        self.prepared_index = int(index)
        self.ready = True
        self.preparation_seconds = perf_counter() - started

    def feedback(self, measured: np.ndarray) -> CentroidalPlan:
        """새 초기 상태로 equality RHS를 갱신하고 QP를 정확히 한 번 호출한다."""
        if not self.ready:
            raise RuntimeError("feedback 전에 해당 주기의 prepare(index)가 필요합니다.")
        measured = np.asarray(measured, dtype=float)
        if measured.shape != (9,) or not np.isfinite(measured).all():
            raise ValueError("측정 상태는 유한한 9차원 벡터여야 합니다.")
        self.ready = False
        started = perf_counter()
        self.last_mode = "failed"
        self.linearization_gap = self.qp_residual = float("inf")
        n, cfg = self.horizon, self.nominal.config
        try:
            # NLP graph의 첫 constraint가 x[:,0] == initial이다.
            self.lower[:9] = self.upper[:9] = self.row_scale[:9] * measured
            self.qp.update(l=self.lower, u=self.upper)
            self.qp_solve_calls += 1
            solution = self.qp.solve(raise_error=False)
            self.qp_iterations = int(solution.info.iter)
            if (
                solution.info.status != "solved"
                or solution.x is None
                or not np.isfinite(solution.x).all()
            ):
                raise CentroidalFailure(f"RTI QP: {solution.info.status}")
            value = self.matrix @ solution.x
            self.qp_residual = float(
                np.maximum(np.maximum(self.lower - value, value - self.upper), 0).max()
            )
            if self.qp_residual > 2e-5:
                raise CentroidalFailure(f"RTI QP residual: {self.qp_residual}")
            z = self.scale * solution.x
            linear_states = z[: 9 * (n + 1)].reshape(n + 1, 9)
            inputs = z[9 * (n + 1) :].reshape(n, 2, 6)
            states = [measured.copy()]
            for k, dt in enumerate(np.diff(self.contacts.times)):
                states.append(
                    centroidal_step(
                        states[-1],
                        inputs[k],
                        self.contacts.positions[k],
                        float(dt),
                        cfg,
                    )
                )
            states = np.asarray(states)
            self.linearization_gap = float(np.max(abs(states - linear_states)))
            plan = CentroidalPlan(
                cfg,
                self.contacts,
                states,
                inputs,
                self.reference,
                measured.copy(),
                False,
                iterations=1,
                status="RTI one SQP",
                objective=float(
                    self.objective(
                        np.r_[states.ravel(), inputs.ravel()], self.parameters
                    )
                ),
            )
            validation = validate_centroidal(plan)
            if not validation["valid"]:
                raise CentroidalFailure(f"RTI nonlinear rollout: {validation}")
            self.last_mode = "new"
        except CentroidalFailure:
            self.failures += 1
            fallback = self.shifted_plan(
                measured, self.prepared_index, self.contacts, self.reference
            )
            if fallback is None or not validate_centroidal(fallback)["valid"]:
                raise
            plan = fallback
            self.last_mode = "validated_previous"
        finally:
            self.feedback_seconds = perf_counter() - started
            self.last_seconds = self.preparation_seconds + self.feedback_seconds
        plan.solve_seconds = self.last_seconds
        self.previous, self.last_index = plan, self.prepared_index
        return plan

    def solve(self, measured: np.ndarray, index: int) -> CentroidalPlan:
        """동기식 편의 API. 전체 latency는 preparation과 feedback을 모두 포함한다."""
        self.prepare(index)
        return self.feedback(measured)
