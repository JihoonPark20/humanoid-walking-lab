"""과거 centroidal 측정과 이미 보낸 wrench로 현재 시각까지 상태를 예측한다."""

from __future__ import annotations

import numpy as np
from centroidal_model import CentroidalConfig, Contacts, centroidal_step


class VerticalMassObserver:
    """CoM 높이의 두 구간 변위와 force command로 질량을 추정한다."""

    def __init__(self, config: CentroidalConfig, gain: float = 0.3) -> None:
        """Nominal 질량에서 시작하며 momentum을 질량 식별에 사용하지 않는다."""
        if not np.isfinite(gain) or not 0 < gain <= 1:
            raise ValueError("observer gain은 (0,1]이어야 합니다.")
        self.config, self.gain = config, gain
        self.mass = config.mass
        self.samples: list[tuple[int, float]] = []
        self.updates = 0
        self.rejected = 0

    def observe(
        self,
        measured: np.ndarray,
        index: int,
        commands: list[np.ndarray],
        contacts: Contacts,
    ) -> float:
        """비균일 interval의 chord velocity 차이로 수직 가속도를 식별한다."""
        measured = np.asarray(measured, dtype=float)
        if (
            measured.shape != (9,)
            or not np.isfinite(measured).all()
            or not 0 <= index <= len(commands)
            or index >= len(contacts.times)
        ):
            raise ValueError("측정값 또는 timestamp가 잘못됐습니다.")
        if self.samples:
            previous = self.samples[-1][0]
            if index < previous:
                raise ValueError("측정 timestamp가 역행했습니다.")
            if index == previous:
                return self.mass
            if index != previous + 1:
                self.samples.clear()
        self.samples.append((index, float(measured[2])))
        self.samples = self.samples[-3:]
        if len(self.samples) == 3:
            (i0, z0), (i1, z1), (i2, z2) = self.samples
            h0, h1 = (
                contacts.times[i1] - contacts.times[i0],
                contacts.times[i2] - contacts.times[i1],
            )
            f0, f1 = float(commands[i0][:, 2].sum()), float(commands[i1][:, 2].sum())
            denominator = 2 * (
                (z2 - z1) / h1 - (z1 - z0) / h0
            ) + self.config.gravity * (h0 + h1)
            candidate = (
                (f0 * h0 + f1 * h1) / denominator
                if denominator > 1e-9
                else float("nan")
            )
            if (
                np.isfinite(candidate)
                and 0.75 * self.config.mass <= candidate <= 1.25 * self.config.mass
            ):
                self.mass += self.gain * (candidate - self.mass)
                self.updates += 1
            else:
                self.rejected += 1
        return self.mass


def predict_delayed_state(
    measured: np.ndarray,
    measured_index: int,
    current_index: int,
    commands: list[np.ndarray],
    contacts: Contacts,
    config: CentroidalConfig,
) -> np.ndarray:
    """측정 timestamp부터 현재까지 command를 재적분한다. 미래 상태·입력은 사용하지 않는다."""
    state = np.asarray(measured, dtype=float).copy()
    if state.shape != (9,) or not np.isfinite(state).all():
        raise ValueError("측정값은 유한한 9차원 벡터여야 합니다.")
    if not 0 <= measured_index <= current_index <= len(
        commands
    ) or current_index >= len(contacts.times):
        raise ValueError("timestamp 또는 command 이력이 맞지 않습니다.")
    for j in range(measured_index, current_index):
        u = np.asarray(commands[j])
        if u.shape != (2, 6) or not np.isfinite(u).all():
            raise ValueError("기록된 wrench가 잘못됐습니다.")
        state = centroidal_step(
            state,
            u,
            contacts.positions[j],
            float(contacts.times[j + 1] - contacts.times[j]),
            config,
        )
    return state
