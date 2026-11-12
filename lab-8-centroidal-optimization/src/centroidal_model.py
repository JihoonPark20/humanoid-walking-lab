"""평지 fixed-contact centroidal Dynamics. 위치 m, 운동량 kg·m/s와 kg·m²/s."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CentroidalConfig:
    """발바닥 중심에서 표현한 world-frame wrench의 물리 제약."""

    mass: float = 33.34
    gravity: float = 9.81
    friction: float = 0.6
    half_length: float = 0.05
    half_width: float = 0.02
    yaw_radius: float = 0.015
    max_normal_force: float = 800.0
    min_height: float = 0.50
    max_height: float = 0.85
    max_velocity: float = 1.0
    max_angular_momentum: float = 6.0

    def __post_init__(self) -> None:
        """수치적으로 의미 없는 물리 설정을 조기에 거부한다."""
        if not all(np.isfinite(v) and v > 0 for v in vars(self).values()):
            raise ValueError("모든 물리 설정은 유한한 양수여야 합니다.")
        if self.min_height >= self.max_height:
            raise ValueError("min_height < max_height가 필요합니다.")


@dataclass
class Contacts:
    """N개 interval의 발바닥 중심 (N,2,3), active mask (N,2), N+1 시각."""

    times: np.ndarray
    positions: np.ndarray
    active: np.ndarray

    def __post_init__(self) -> None:
        """접촉 배열과 시간의 일관성을 검사한다."""
        n = len(self.times) - 1
        if (
            n < 1
            or self.positions.shape != (n, 2, 3)
            or self.active.shape != (n, 2)
            or not np.isfinite(self.times).all()
            or not np.isfinite(self.positions).all()
            or np.any(np.diff(self.times) <= 0)
            or not np.isin(self.active, [0, 1]).all()
            or np.any(self.active.sum(axis=1) == 0)
        ):
            raise ValueError("시간·접촉 크기/값이 잘못됐거나 무접촉 phase가 있습니다.")


def centroidal_step(
    state: np.ndarray,
    wrench: np.ndarray,
    positions: np.ndarray,
    dt: float,
    config: CentroidalConfig,
) -> np.ndarray:
    """상수 wrench에서 exact integration. 각운동량에는 interval 평균 CoM을 사용한다."""
    if dt < 0 or not np.isfinite(dt):
        raise ValueError("dt는 유한한 0 이상의 값이어야 합니다.")
    c, p, angular = np.asarray(state).reshape(3, 3)
    force, moment = wrench[:, :3], wrench[:, 3:]
    acceleration = force.sum(axis=0) / config.mass + [0.0, 0.0, -config.gravity]
    mean_c = c + dt * p / (2 * config.mass) + dt**2 * acceleration / 6
    return np.r_[
        c + dt * p / config.mass + dt**2 * acceleration / 2,
        p + dt * config.mass * acceleration,
        angular + dt * (np.cross(positions - mean_c, force) + moment).sum(axis=0),
    ]


def wrench_violation(
    wrench: np.ndarray, active: np.ndarray, config: CentroidalConfig
) -> float:
    """Unilateral/friction pyramid/CoP/yaw/비접촉 제약의 최대 위반을 계산한다."""
    f, tau = wrench[:, :3], wrench[:, 3:]
    fz = f[:, 2]
    values = [
        0.0,
        float(np.max(-fz)),
        float(np.max(fz - active * config.max_normal_force)),
        float(np.max(abs(f[:, :2]) - config.friction * fz[:, None])),
        float(np.max(abs(tau[:, 0]) - config.half_width * fz)),
        float(np.max(abs(tau[:, 1]) - config.half_length * fz)),
        float(np.max(abs(tau[:, 2]) - config.yaw_radius * fz)),
    ]
    return max(values)


@dataclass
class CentroidalPlan:
    """검증 대상인 centroidal state/wrench trajectory와 solver 진단."""

    config: CentroidalConfig
    contacts: Contacts
    states: np.ndarray
    wrenches: np.ndarray
    reference: np.ndarray
    initial: np.ndarray
    terminal_stop: bool
    solve_seconds: float = 0.0
    iterations: int = 0
    status: str = ""
    objective: float = 0.0

    def sample(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        """구간 내부에서도 exact Dynamics로 상태와 wrench를 평가한다."""
        if not np.isfinite(t):
            raise ValueError("t는 유한해야 합니다.")
        times = self.contacts.times
        t = float(np.clip(t, times[0], times[-1]))
        k = min(
            int(np.searchsorted(times, t, side="right") - 1), len(self.wrenches) - 1
        )
        return centroidal_step(
            self.states[k],
            self.wrenches[k],
            self.contacts.positions[k],
            t - times[k],
            self.config,
        ), self.wrenches[k].copy()


def validate_centroidal(plan: CentroidalPlan, tolerance: float = 1e-4) -> dict:
    """NLP와 독립적으로 exact Dynamics, wrench, 경계와 연속 높이를 검사한다."""
    cfg = plan.config
    n = len(plan.contacts.times) - 1
    if plan.states.shape != (n + 1, 9) or plan.wrenches.shape != (n, 2, 6):
        return {"valid": False, "reason": "shape"}
    if not all(
        np.isfinite(a).all()
        for a in (plan.states, plan.wrenches, plan.reference, plan.initial)
    ):
        return {"valid": False, "reason": "nonfinite"}
    dynamics = wrench = height = velocity = angular = 0.0
    for k, dt in enumerate(np.diff(plan.contacts.times)):
        x, u, r = plan.states[k], plan.wrenches[k], plan.contacts.positions[k]
        next_x = centroidal_step(x, u, r, float(dt), cfg)
        dynamics = max(dynamics, float(np.max(abs(next_x - plan.states[k + 1]))))
        wrench = max(wrench, wrench_violation(u, plan.contacts.active[k], cfg))
        az = u[:, 2].sum() / cfg.mass - cfg.gravity
        samples = [0.0, float(dt)]
        if abs(az) > 1e-12:
            root = -x[5] / cfg.mass / az
            if 0 < root < dt:
                samples.append(float(root))
        # L(t)는 cubic이므로 각 축 미분의 실근도 검사한다.
        acceleration = u[:, :3].sum(axis=0) / cfg.mass + [0.0, 0.0, -cfg.gravity]
        coeff0 = (np.cross(r - x[:3], u[:, :3]) + u[:, 3:]).sum(axis=0)
        coeff1 = -np.cross(x[3:6] / cfg.mass, u[:, :3].sum(axis=0))
        coeff2 = -0.5 * np.cross(acceleration, u[:, :3].sum(axis=0))
        for axis in range(3):
            coeff = np.trim_zeros(
                np.array([coeff2[axis], coeff1[axis], coeff0[axis]]), "f"
            )
            for root in np.roots(coeff) if len(coeff) > 1 else []:
                if abs(root.imag) < 1e-10 and 0 < root.real < dt:
                    samples.append(float(root.real))
        for t in samples:
            value = centroidal_step(x, u, r, t, cfg)
            height = max(height, cfg.min_height - value[2], value[2] - cfg.max_height)
            velocity = max(
                velocity, float(np.max(abs(value[3:6])) / cfg.mass - cfg.max_velocity)
            )
            angular = max(
                angular, float(np.max(abs(value[6:])) - cfg.max_angular_momentum)
            )
    boundary = float(np.max(abs(plan.states[0] - plan.initial)))
    if plan.terminal_stop:
        boundary = max(
            boundary, float(np.max(abs(plan.states[-1] - plan.reference[-1])))
        )
    result = {
        "dynamics_residual": dynamics,
        "wrench_violation": wrench,
        "height_violation": max(0.0, height),
        "velocity_violation": max(0.0, velocity),
        "angular_violation": max(0.0, angular),
        "boundary_residual": boundary,
    }
    result["valid"] = bool(max(result.values()) <= tolerance)
    return result
