"""Contact OCP의 torque feedforward와 실제 물리 추종용 feedback 정책."""

from __future__ import annotations

from dataclasses import dataclass

import crocoddyl as croc
import numpy as np


@dataclass(frozen=True)
class TrackingConfig:
    """FDDP의 국소 feedback을 joint PD와 혼합한다. 안정성의 수학적 보장은 아니다."""

    mode: str = "fddp"
    joint_kp: float = 400.0
    joint_kd: float = 20.0
    feedback_blend: float = 0.1

    def __post_init__(self) -> None:
        """불명확한 mode나 비물리적 gain을 거부한다."""
        if self.mode not in ("fddp", "joint-pd", "blended"):
            raise ValueError("tracking은 fddp, joint-pd, blended 중 하나입니다.")
        if not np.isfinite([self.joint_kp, self.joint_kd, self.feedback_blend]).all():
            raise ValueError("tracking 설정은 유한해야 합니다.")
        if (
            self.joint_kp <= 0
            or self.joint_kd <= 0
            or not 0 <= self.feedback_blend <= 1
        ):
            raise ValueError("joint gain은 양수, blend는 [0,1]이어야 합니다.")


def tracking_torque(
    state: croc.StateMultibody,
    nominal: np.ndarray,
    actual: np.ndarray,
    feedforward: np.ndarray,
    gain: np.ndarray,
    config: TrackingConfig,
) -> np.ndarray:
    """Base를 직접 구동하지 않고 manifold 오차에서 joint torque 보정만 만든다."""
    error = state.diff(nominal, actual)
    fddp = -gain @ error
    if config.mode == "fddp":
        return feedforward + fddp
    # Free-flyer 6개를 제외한 관절 오차만 사용한다. Quaternion에 덧셈하지 않는다.
    joint_pd = (
        -config.joint_kp * error[6 : state.nv] - config.joint_kd * error[state.nv + 6 :]
    )
    blend = config.feedback_blend if config.mode == "blended" else 0.0
    return feedforward + blend * fddp + (1 - blend) * joint_pd


def recommended_control_weight(mode: str) -> float:
    """검증할 추종 정책에 맞는 계획 비용을 선택한다. 명시적인 CLI 값으로 재정의 가능하다."""
    if mode not in ("fddp", "blended", "joint-pd"):
        raise ValueError("알 수 없는 tracking mode입니다.")
    return 3.0 if mode == "fddp" else 0.1
