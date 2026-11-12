"""평지 LIPM 실습의 설정, 발 디딤과 지지 영역. 로봇 모델 없이 실행할 수 있다."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class WalkConfig:
    """SI 단위의 계획 설정. steps는 전진 걸음 수이며 마지막 모으기 걸음은 별도다."""

    steps: int = 4
    step_length: float = 0.10
    step_width: float = 0.18
    single_support: float = 0.65
    double_support: float = 0.25
    settle_time: float = 1.5
    dt: float = 0.05
    com_height: float = 0.66
    gravity: float = 9.81
    foot_offset_x: float = 0.035
    half_length: float = 0.05
    half_width: float = 0.020
    step_height: float = 0.03
    min_stride: float = 0.04
    max_stride: float = 0.16
    min_width: float = 0.16
    max_width: float = 0.22
    max_velocity: float = 0.8
    max_acceleration: float = 3.0
    max_jerk: float = 30.0
    zmp_weight: float = 100.0
    jerk_weight: float = 0.01
    foot_weight: float = 5.0
    first_swing: int = 0

    def __post_init__(self) -> None:
        """잘못된 물리값과 너무 작은 시간 격자를 조기에 거부한다."""
        if (
            isinstance(self.steps, bool)
            or not isinstance(self.steps, int)
            or not 1 <= self.steps <= 30
        ):
            raise ValueError("steps는 1~30 사이의 정수여야 합니다.")
        if self.first_swing not in (0, 1):
            raise ValueError("first_swing은 0(왼발) 또는 1(오른발)입니다.")
        positive = (
            "step_length",
            "step_width",
            "single_support",
            "double_support",
            "settle_time",
            "dt",
            "com_height",
            "gravity",
            "half_length",
            "half_width",
            "step_height",
            "min_stride",
            "max_stride",
            "min_width",
            "max_width",
            "max_velocity",
            "max_acceleration",
            "max_jerk",
            "zmp_weight",
            "jerk_weight",
            "foot_weight",
        )
        for name in positive:
            value = getattr(self, name)
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name}은 유한한 양수여야 합니다.")
        if not np.isfinite(self.foot_offset_x):
            raise ValueError("foot_offset_x는 유한해야 합니다.")
        if self.dt < 0.005:
            raise ValueError("오프라인 격자는 dt >= 0.005 s를 사용하세요.")
        if not self.min_stride <= self.step_length <= self.max_stride:
            raise ValueError("step_length가 stride 범위 밖입니다.")
        if not self.min_width <= self.step_width <= self.max_width:
            raise ValueError("step_width가 width 범위 밖입니다.")

    @property
    def omega(self) -> float:
        """LIPM 고유 주파수 sqrt(g/h), 단위 1/s."""
        return math.sqrt(self.gravity / self.com_height)


@dataclass(frozen=True)
class SupportPhase:
    """한 접촉 구간. feet는 좌/우 node 번호이며 swing=-1이면 양발 지지다."""

    start: float
    end: float
    feet: tuple[int, int]
    swing: int
    landing: int
    center_start: np.ndarray
    center_end: np.ndarray

    @property
    def duration(self) -> float:
        """구간 지속 시간."""
        return self.end - self.start


@dataclass
class WalkingSchedule:
    """발 좌표에 affine인 허용 ZMP rectangle의 중심과 접촉 시간표."""

    config: WalkConfig
    feet: np.ndarray
    phases: list[SupportPhase]
    times: np.ndarray
    center_start: np.ndarray
    center_end: np.ndarray
    phase_ids: np.ndarray
    stance_nodes: np.ndarray
    landing_sides: np.ndarray

    @property
    def duration(self) -> float:
        """종료 시각."""
        return float(self.times[-1])

    @property
    def final_weights(self) -> np.ndarray:
        """마지막 양발 중심을 나타내는 convex weight."""
        return self.phases[-1].center_end

    def center(self, interval: int, local_time: float, feet: np.ndarray) -> np.ndarray:
        """구간 내 support rectangle 중심. 발목→발바닥 offset을 포함한다."""
        h = self.times[interval + 1] - self.times[interval]
        s = local_time / h
        weights = (1 - s) * self.center_start[interval] + s * self.center_end[interval]
        return weights @ feet + np.array([self.config.foot_offset_x, 0.0])


def jerk_matrices(dt: float) -> tuple[np.ndarray, np.ndarray]:
    """상수 jerk에서 [c, v, a]의 정확한 이산 적분 행렬 A, B."""
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("dt는 유한한 양수여야 합니다.")
    return (
        np.array([[1.0, dt, dt * dt / 2], [0.0, 1.0, dt], [0.0, 0.0, 1.0]]),
        np.array([dt**3 / 6, dt * dt / 2, dt]),
    )


def build_schedule(
    config: WalkConfig, initial_feet: np.ndarray | None = None
) -> WalkingSchedule:
    """양발 지지→교대 보행→발 모으기→정지의 시간표와 nominal 발 좌표를 만든다."""
    home = (
        np.array([[0.0, config.step_width / 2], [0.0, -config.step_width / 2]])
        if initial_feet is None
        else np.asarray(initial_feet, dtype=float).copy()
    )
    if home.shape != (2, 2) or not np.isfinite(home).all() or home[0, 1] <= home[1, 1]:
        raise ValueError(
            "initial_feet는 유한한 (2,2) 배열이며 왼발 y가 오른발보다 커야 합니다."
        )
    count = config.steps + 3  # 최초 두 발 + 전진 steps + 마지막 발 모으기
    feet = np.zeros((count, 2))
    feet[:2] = home
    ids = [0, 1]
    stance_nodes, sides = [], []
    for k in range(config.steps + 1):
        swing = (config.first_swing + k) % 2
        stance = ids[1 - swing]
        target = feet[stance].copy()
        target[0] += config.step_length if k < config.steps else 0.0
        target[1] += config.step_width * (1 if swing == 0 else -1)
        feet[k + 2] = target
        stance_nodes.append(stance)
        sides.append(swing)
        ids[swing] = k + 2
    eye = np.eye(count)
    phases: list[SupportPhase] = []
    ids = [0, 1]
    t = 0.0

    def append(
        duration: float, swing: int, landing: int, start: np.ndarray, end: np.ndarray
    ) -> None:
        """현재 발 node 상태를 보존해 구간을 추가한다."""
        nonlocal t
        phases.append(
            SupportPhase(t, t + duration, tuple(ids), swing, landing, start, end)
        )
        t += duration

    current = (eye[0] + eye[1]) / 2
    target = eye[1 - config.first_swing]
    append(config.settle_time, -1, -1, current, target)
    for k, swing in enumerate(sides):
        stance = ids[1 - swing]
        current = eye[stance]
        append(config.single_support, swing, k + 2, current, current)
        ids[swing] = k + 2
        target = eye[k + 2] if k < config.steps else (eye[ids[0]] + eye[ids[1]]) / 2
        append(config.double_support, -1, -1, current, target)
    append(config.settle_time, -1, -1, target, target)
    times = [0.0]
    starts, ends, phase_ids = [], [], []
    for index, phase in enumerate(phases):
        n = max(1, math.ceil(phase.duration / config.dt - 1e-9))
        for j in range(n):
            starts.append(
                phase.center_start + (phase.center_end - phase.center_start) * j / n
            )
            ends.append(
                phase.center_start
                + (phase.center_end - phase.center_start) * (j + 1) / n
            )
            phase_ids.append(index)
            times.append(phase.start + phase.duration * (j + 1) / n)
    return WalkingSchedule(
        config,
        feet,
        phases,
        np.asarray(times),
        np.asarray(starts),
        np.asarray(ends),
        np.asarray(phase_ids),
        np.asarray(stance_nodes),
        np.asarray(sides),
    )


def polynomial_state(state: np.ndarray, jerk: np.ndarray, t: float) -> np.ndarray:
    """한 구간의 (2,3) 상태를 exact polynomial로 평가한다."""
    if t == 0:
        return np.asarray(state).copy()
    a, b = jerk_matrices(t)
    return np.asarray(state) @ a.T + np.asarray(jerk)[:, None] * b
