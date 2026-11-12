"""계획한 관절 궤적을 제어기로 추종하며 MuJoCo에서 실행한다."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

import mujoco
import numpy as np
import pinocchio as pin
from lab4_common import (
    DT,
    NUM_JOINTS,
    OBSTACLES,
    ObstacleSpec,
    apply_arm_torques,
    clip_torques,
    get_mj_ee_pos,
    load_mujoco_model,
    load_pinocchio_model,
)
from scene_clearance import ObstacleClearance


def execute_trajectory(
    times: np.ndarray,
    q_traj: np.ndarray,
    qd_traj: np.ndarray,
    scene_path: Path | None = None,
    obstacle_specs: tuple[ObstacleSpec, ...] | list[ObstacleSpec] | None = None,
    Kp: float = 400.0,
    Kd: float = 40.0,
    include_table: bool = True,
    frame_callback: Callable[[mujoco.MjModel, mujoco.MjData, dict], None] | None = None,
) -> dict[str, np.ndarray]:
    """Execute a timed trajectory using joint-space PD plus gravity compensation."""
    mj_model, mj_data = load_mujoco_model(scene_path, obstacle_specs=obstacle_specs, include_table=include_table)
    pin_model, pin_data, _ = load_pinocchio_model()

    mj_data.qpos[:NUM_JOINTS] = q_traj[0]
    mj_data.qvel[:NUM_JOINTS] = qd_traj[0]
    mujoco.mj_forward(mj_model, mj_data)

    traj_duration = float(times[-1])
    total_duration = traj_duration + 0.5
    n_steps = int(total_duration / DT) + 1

    log_time = np.zeros(n_steps)
    log_q_actual = np.zeros((n_steps, NUM_JOINTS))
    log_q_desired = np.zeros((n_steps, NUM_JOINTS))
    log_tau = np.zeros((n_steps, NUM_JOINTS))
    log_ee_pos = np.zeros((n_steps, 3))
    contact_counts = np.zeros(n_steps, dtype=int)
    specs = OBSTACLES if obstacle_specs is None else obstacle_specs
    clearance = ObstacleClearance(
        mj_model, [s.name for s in specs if s.name != "table"]
    )
    obstacle_clearance = np.zeros(n_steps)

    for step in range(n_steps):
        t = step * DT
        if t <= traj_duration:
            q_d = np.array(
                [np.interp(t, times, q_traj[:, j]) for j in range(NUM_JOINTS)]
            )
            qd_d = np.array(
                [np.interp(t, times, qd_traj[:, j]) for j in range(NUM_JOINTS)]
            )
        else:
            q_d = q_traj[-1]
            qd_d = np.zeros(NUM_JOINTS)

        q = mj_data.qpos[:NUM_JOINTS].copy()
        qd = mj_data.qvel[:NUM_JOINTS].copy()

        pin.computeGeneralizedGravity(pin_model, pin_data, q)
        g = pin_data.g.copy()

        tau = Kp * (q_d - q) + Kd * (qd_d - qd) + g
        tau = clip_torques(tau)
        apply_arm_torques(mj_model, mj_data, tau)

        log_time[step] = t
        log_q_actual[step] = q
        log_q_desired[step] = q_d
        log_tau[step] = tau
        log_ee_pos[step] = get_mj_ee_pos(mj_model, mj_data)

        # The tabletop scene has no intended robot/environment contact.
        contact_counts[step] = mj_data.ncon
        obstacle_clearance[step] = clearance.minimum(mj_data)
        if frame_callback is not None:
            frame_callback(
                mj_model,
                mj_data,
                {
                    "time": t,
                    "duration": total_duration,
                    "ee_pos": log_ee_pos[step].copy(),
                    "joint_error": float(np.max(np.abs(q - q_d))),
                },
            )
        mujoco.mj_step(mj_model, mj_data)

    return {
        "time": log_time,
        "q_actual": log_q_actual,
        "q_desired": log_q_desired,
        "tau": log_tau,
        "ee_pos": log_ee_pos,
        "contact_counts": contact_counts,
        "obstacle_clearance": obstacle_clearance,
    }
