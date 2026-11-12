"""W5 CLI: Box-FDDP 계획 저장과 MuJoCo feedback rollout 비교."""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import platform
import sys
from dataclasses import asdict
from pathlib import Path

import crocoddyl as croc
import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import pinocchio as pin

from tools.walking_visuals import FPS as VIS_FPS
from tools.walking_visuals import Overlay, WalkingVisualizer

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "lab-6-whole-body-control" / "src"))

from lab6_common import DT, foot_contact_state, mj_state_to_pin, pin_q_to_mj
from whole_body_ocp import G1GaitProblem, OCPConfig, solve_problem
from whole_body_tracking import (
    TrackingConfig,
    recommended_control_weight,
    tracking_torque,
)


def save_solution(
    solver: croc.SolverBoxFDDP,
    times: np.ndarray,
    phases: list,
    report: dict,
    output: Path,
    config: OCPConfig,
) -> None:
    """Impulse node의 빈 control을 mask와 함께 저장한다. Quaternion은 원본을 보존한다."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output.mkdir(parents=True, exist_ok=True)
    n = len(solver.us)
    nu = 29
    ndx = solver.problem.runningModels[0].state.ndx
    controls = np.zeros((n, nu))
    gains = np.zeros((n, nu, ndx))
    active = np.zeros(n, dtype=bool)
    for k, u in enumerate(solver.us):
        if len(u):
            controls[k] = u
            gains[k] = solver.K[k]
            active[k] = True
    metadata = {
        "schema": 1,
        "config": asdict(config),
        "report": report,
        "environment": {
            "python": platform.python_version(),
            "platform": platform.platform(),
            "versions": {
                key: importlib.metadata.version(key)
                for key in ("pin", "crocoddyl", "mujoco", "numpy")
            },
        },
    }
    np.savez_compressed(
        output / "whole_body.npz",
        metadata=json.dumps(metadata),
        times=times,
        states=np.asarray(solver.xs),
        controls=controls,
        feedback_gains=gains,
        actuated_nodes=active,
        phases=np.asarray(phases),
    )
    (output / "summary.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n"
    )
    np.savetxt(
        output / "trajectory.csv",
        np.c_[times, np.asarray(solver.xs)],
        delimiter=",",
        comments="",
        header=",".join(["time_s"] + [f"x{i}" for i in range(len(solver.xs[0]))]),
    )
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    xs = np.asarray(solver.xs)
    nq = solver.problem.runningModels[0].state.nq
    axes[0].plot(times, xs[:, 7:nq])
    axes[0].set_ylabel("joint q (rad)")
    axes[1].plot(times[:-1][active], controls[active])
    axes[1].set_ylabel("joint torque (Nm)")
    axes[2].plot(times, np.linalg.norm(xs[:, nq:], axis=1))
    axes[2].set(ylabel="|v|", xlabel="time (s)")
    for k, phase in enumerate(phases):
        if phase == "impact":
            for ax in axes:
                ax.axvline(times[k], color="k", ls="--", alpha=0.4)
    for ax in axes:
        ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(output / "whole_body.png", dpi=130)
    plt.close(fig)


def simulate_mujoco(
    solver: croc.SolverBoxFDDP,
    gait: G1GaitProblem,
    mj_model: mujoco.MjModel,
    mj_data: mujoco.MjData,
    times: np.ndarray,
    output: Path,
    video: bool = False,
    tracking: TrackingConfig | None = None,
) -> dict:
    """선택한 feedback으로 물리 rollout한다. Impulse 상태를 MuJoCo에 덮어쓰지 않는다."""
    output.mkdir(parents=True, exist_ok=True)
    tracking = TrackingConfig() if tracking is None else tracking
    state = gait.state
    pm = gait.rmodel
    initial_data = pm.createData()
    pin.forwardKinematics(pm, initial_data, solver.xs[0][: pm.nq])
    pin.updateFramePlacements(pm, initial_data)
    rows = []
    failure = None
    clipped_ticks = 0
    writer = renderer = None
    frame_clock = 0.0
    if video:
        import imageio.v2 as imageio

        writer = imageio.get_writer(
            str(output / "rollout.mp4"), fps=VIS_FPS, codec="libx264"
        )
        renderer = WalkingVisualizer(mj_model, 10)

    t = 0.0
    try:
        for k, (action, u) in enumerate(zip(solver.problem.runningModels, solver.us)):
            duration = times[k + 1] - times[k]
            if not action.nu:
                continue  # 실제 접촉 충격은 MuJoCo가 계산한다.
            nodes = max(1, round(duration / DT))
            delta = state.diff(solver.xs[k], solver.xs[k + 1])
            for inner in range(nodes):
                q, v = mj_state_to_pin(mj_data)
                actual = np.r_[q, v]
                nominal = state.integrate(solver.xs[k], (inner / nodes) * delta)
                command = tracking_torque(
                    state, nominal, actual, np.asarray(u), solver.K[k], tracking
                )
                clipped = np.clip(
                    command,
                    mj_model.actuator_ctrlrange[:, 0],
                    mj_model.actuator_ctrlrange[:, 1],
                )
                clipped_ticks += bool(np.any(abs(command - clipped) > 1e-6))
                mj_data.ctrl[:] = clipped
                mujoco.mj_step(mj_model, mj_data)
                t += DT
                q, v = mj_state_to_pin(mj_data)
                actual = np.r_[q, v]
                next_nominal = state.integrate(
                    solver.xs[k], ((inner + 1) / nodes) * delta
                )
                gap = float(np.linalg.norm(state.diff(next_nominal, actual)))
                left, right = foot_contact_state(mj_model, mj_data)
                rows.append(
                    [
                        t,
                        gap,
                        np.linalg.norm(state.diff(next_nominal, actual)[: pm.nv]),
                        mj_data.qpos[2],
                        int(left),
                        int(right),
                        np.max(abs(clipped)),
                    ]
                )
                if writer is not None and t >= frame_clock:
                    ghosts = []
                    for offset in (0.25, 0.5):
                        future = t + offset
                        if future > times[-1]:
                            continue
                        j = min(
                            int(np.searchsorted(times, future, side="right") - 1),
                            len(times) - 2,
                        )
                        fraction = (future - times[j]) / max(
                            times[j + 1] - times[j], 1e-9
                        )
                        nominal_q = pin.interpolate(
                            pm,
                            solver.xs[j][: pm.nq],
                            solver.xs[j + 1][: pm.nq],
                            float(fraction),
                        )
                        ghosts.append((offset, pin_q_to_mj(nominal_q)))
                    writer.append_data(
                        renderer.render(
                            mj_data,
                            Overlay(
                                time=t,
                                com=mj_data.subtree_com[0].copy(),
                                ghosts=ghosts,
                            ),
                        )
                    )
                    frame_clock += 1 / VIS_FPS

                if mj_data.qpos[2] < 0.50 or not np.isfinite(actual).all():
                    failure = "fall/nonfinite"
                    break
            if failure:
                break
    finally:
        if writer is not None:
            writer.close()
        if renderer is not None:
            renderer.close()
    rows = np.asarray(rows)
    np.savetxt(
        output / "mujoco.csv",
        rows,
        delimiter=",",
        comments="",
        header="t,state_gap,tangent_q_gap,pelvis_z,left_contact,right_contact,max_torque",
    )
    q, v = mj_state_to_pin(mj_data)
    data = pm.createData()
    pin.forwardKinematics(pm, data, q)
    pin.updateFramePlacements(pm, data)
    nominal_data = pm.createData()
    pin.forwardKinematics(pm, nominal_data, solver.xs[-1][: pm.nq])
    pin.updateFramePlacements(pm, nominal_data)
    foot_error = max(
        float(
            np.linalg.norm(
                data.oMf[frame].translation - nominal_data.oMf[frame].translation
            )
        )
        for frame in (gait.lfId, gait.rfId)
    )
    complete = t >= times[-1] - DT
    flight_ticks = int(np.sum(rows[:, 4:6].sum(axis=1) == 0)) if len(rows) else 0
    report = {
        "passed": bool(
            failure is None and complete and foot_error < 0.03 and flight_ticks == 0
        ),
        "tracking": asdict(tracking),
        "failure": failure,
        "completed": bool(complete),
        "elapsed_s": t,
        "duration_s": float(times[-1]),
        "final_foot_gap_m": foot_error,
        "final_state_gap": float(rows[-1, 1]) if len(rows) else None,
        "clipping_ticks": clipped_ticks,
        "flight_ticks": flight_ticks,
        "impact_state_overwrites": 0,
        "max_state_gap": float(rows[:, 1].max()) if len(rows) else None,
        "max_tangent_q_gap": float(rows[:, 2].max()) if len(rows) else None,
        "min_pelvis_height_m": float(rows[:, 3].min()) if len(rows) else None,
    }
    if video:
        report["video_sha256"] = hashlib.sha256(
            (output / "rollout.mp4").read_bytes()
        ).hexdigest()
    (output / "mujoco_summary.json").write_text(json.dumps(report, indent=2) + "\n")
    plot_tracking(rows, output / "tracking.png")
    return report


def plot_tracking(rows: np.ndarray, output: Path) -> None:
    """실제 추종 오차와 접촉·토크를 기록해 timestep 비교에 사용한다."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if not len(rows):
        return
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(rows[:, 0], rows[:, 1], label="state tangent gap")
    axes[0].plot(rows[:, 0], rows[:, 2], label="configuration tangent gap")
    axes[0].legend()
    axes[0].set_ylabel("mixed tangent units")
    axes[1].plot(rows[:, 0], rows[:, 4] + rows[:, 5])
    axes[1].set_ylabel("foot contacts")
    axes[2].plot(rows[:, 0], rows[:, 6])
    axes[2].set(ylabel="max |torque| (Nm)", xlabel="time (s)")
    for ax in axes:
        ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(output, dpi=130)
    plt.close(fig)


def main() -> None:
    """정해진 contact problem을 풀고 모델 판정과 실제 rollout 판정을 구분한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=1)
    parser.add_argument("--step-length", type=float, default=0.06)
    parser.add_argument("--max-iterations", type=int, default=600)
    parser.add_argument(
        "--control-weight",
        type=float,
        help="OCP torque regularization; 기본 fddp=3, blended/joint-pd=0.1",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=0.01,
        help="최적화 간격 s; 각 지지/swing 시간 0.6초 유지",
    )
    parser.add_argument("--mujoco", action="store_true")
    parser.add_argument("--video", action="store_true")
    parser.add_argument(
        "--tracking", choices=["blended", "fddp", "joint-pd"], default="fddp"
    )
    parser.add_argument("--feedback-blend", type=float, default=0.1)
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
    )
    args = parser.parse_args()
    if args.output is None:
        args.output = (
            Path(__file__).resolve().parents[1]
            / "media"
            / ("one_step_validated" if args.steps == 1 else "two_steps_validated")
        )
    try:
        if (
            not np.isfinite(args.dt)
            or args.dt <= 0
            or not np.isclose(0.6 / args.dt, round(0.6 / args.dt), atol=1e-8, rtol=0)
            or not np.isclose(args.dt / DT, round(args.dt / DT), atol=1e-8, rtol=0)
            or args.dt < DT
        ):
            raise ValueError(
                "dt는 1 ms의 정수 배이며 0.6초를 정수 개 구간으로 나누어야 합니다."
            )
        cfg = OCPConfig(
            steps=args.steps,
            step_length=args.step_length,
            max_iterations=args.max_iterations,
            control_weight=recommended_control_weight(args.tracking)
            if args.control_weight is None
            else args.control_weight,
            dt=args.dt,
            swing_knots=round(0.6 / args.dt),
            support_knots=round(0.6 / args.dt),
        )
        tracking = TrackingConfig(
            mode=args.tracking, feedback_blend=args.feedback_blend
        )
        solver, gait, m, d, times, phases, report = solve_problem(cfg)
    except (ValueError, RuntimeError) as error:
        parser.exit(1, f"Whole-body 최적화 실패: {error}\n")
    save_solution(solver, times, phases, report, args.output, cfg)
    print(json.dumps(report, indent=2, ensure_ascii=False), flush=True)
    if args.mujoco or args.video:
        if not report["model_rollout_valid"]:
            parser.exit(1, "모델 제약 검증에 실패해 물리 재생을 건너뜁니다.\n")
        actual = simulate_mujoco(
            solver, gait, m, d, times, args.output, args.video, tracking
        )
        print(json.dumps(actual, indent=2), flush=True)
        if not actual["passed"]:
            raise SystemExit(1)
    if not report["converged"] or not report["model_rollout_valid"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
