"""W4b: fixed contact schedule의 full nonlinear centroidal MPC."""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path
from time import perf_counter

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "lab-8-centroidal-optimization" / "src"))
from centroidal_model import (
    CentroidalPlan,
    Contacts,
    centroidal_step,
    validate_centroidal,
)
from centroidal_optimizer import CentroidalFailure, CentroidalSolver


def horizon_contacts(
    plan: CentroidalPlan, index: int, horizon: int
) -> tuple[Contacts, np.ndarray]:
    """계획 끝에서는 최종 양발 접촉과 정지 reference를 연장한다."""
    source = plan.contacts
    n = len(source.times) - 1
    h = np.diff(source.times)
    times = [source.times[min(index, n)]]
    positions = []
    active = []
    refs = []
    for k in range(horizon + 1):
        j = min(index + k, n)
        refs.append(plan.states[j])
        if k < horizon:
            interval = min(j, n - 1)
            positions.append(source.positions[interval])
            active.append(source.active[interval])
            times.append(times[-1] + h[interval])
    return Contacts(
        np.asarray(times), np.asarray(positions), np.asarray(active)
    ), np.asarray(refs)


class CentroidalMPC:
    """재사용 NLP + shifted primal warm start. RTI/SQP 한 번 방식은 아니다."""

    def __init__(
        self, nominal: CentroidalPlan, horizon: int = 12, max_iterations: int = 120
    ) -> None:
        """Full nonlinear solver와 실패/소요 시간 기록을 초기화한다."""
        if horizon < 2:
            raise ValueError("horizon은 2 이상이어야 합니다.")
        self.nominal, self.horizon = nominal, horizon
        self.solver = CentroidalSolver(horizon, nominal.config, False, max_iterations)
        self.previous: CentroidalPlan | None = None
        self.last_index: int | None = None
        self.last_mode = "new"
        self.failures = 0
        self.last_seconds = 0.0

    def update_mass(self, mass: float) -> None:
        """추정 질량으로 Dynamics parameter와 같은 속도의 nominal momentum을 갱신한다."""
        if not np.isfinite(mass) or mass <= 0:
            raise ValueError("질량 추정은 유한한 양수여야 합니다.")
        old = self.nominal
        ratio = mass / old.config.mass
        states, initial, reference = (
            old.states.copy(),
            old.initial.copy(),
            old.reference.copy(),
        )
        states[:, 3:] *= ratio
        initial[3:] *= ratio
        reference[:, 3:] *= ratio
        cfg = replace(old.config, mass=float(mass))
        self.nominal = replace(
            old,
            config=cfg,
            states=states,
            initial=initial,
            reference=reference,
            wrenches=old.wrenches * ratio,
        )
        self.solver.config = cfg

    def shifted_plan(
        self,
        measured: np.ndarray,
        index: int,
        contacts: Contacts,
        reference: np.ndarray,
    ) -> CentroidalPlan | None:
        """이전 입력을 새 초기 상태에서 exact rollout한다. 미래 접촉과 일치해야 한다."""
        if self.previous is None:
            return None
        shift = max(0, index - int(self.last_index))
        if shift >= self.horizon:
            return None
        old = self.previous
        u = np.array(
            [
                old.wrenches[min(k + shift, len(old.wrenches) - 1)]
                for k in range(self.horizon)
            ]
        )
        # 새 접촉 발의 힘을 임의로 만들어내지 않는다. 비접촉 입력만 제거한다.
        u *= contacts.active[:, :, None]
        x = [np.asarray(measured).copy()]
        for k, h in enumerate(np.diff(contacts.times)):
            x.append(
                centroidal_step(
                    x[-1], u[k], contacts.positions[k], float(h), self.nominal.config
                )
            )
        return CentroidalPlan(
            self.nominal.config,
            contacts,
            np.asarray(x),
            u,
            reference,
            measured.copy(),
            False,
            status="shifted",
        )

    def solve(self, measured: np.ndarray, index: int) -> CentroidalPlan:
        """NLP 실패 시 새 상태에서 검증한 이전 rollout만 한 주기 fallback으로 사용한다."""
        if (
            index < 0
            or np.asarray(measured).shape != (9,)
            or not np.isfinite(measured).all()
        ):
            raise ValueError("측정 상태/index 오류")
        started = perf_counter()
        contacts, reference = horizon_contacts(self.nominal, index, self.horizon)
        warm = self.shifted_plan(measured, index, contacts, reference)
        try:
            result = self.solver.solve(measured, reference, contacts, warm)
            self.last_mode = "new"
        except CentroidalFailure:
            self.failures += 1
            if warm is None or not validate_centroidal(warm)["valid"]:
                raise
            result = warm
            self.last_mode = "validated_previous"
        finally:
            self.last_seconds = perf_counter() - started
        self.previous = result
        self.last_index = index
        return result
