"""W0/W2: 동일한 G1 Whole-body controller로 DCM baseline과 오프라인 계획을 비교한다."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.metadata
import json
import platform
import subprocess
import sys
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lipm_optimizer import PlanningError, WalkingPlan, solve_walk
from walking_artifacts import export_plan
from walking_model import WalkConfig, WalkingSchedule, build_schedule

from tools.walking_visuals import FPS as VIS_FPS
from tools.walking_visuals import Overlay, WalkingVisualizer

ROOT = Path(__file__).resolve().parents[2]
LAB6_SRC = ROOT / "lab-6-whole-body-control" / "src"
# Whole-body 제어 모듈은 Lab 6에서 가져온다.
sys.path.insert(0, str(LAB6_SRC))
import mujoco
from dcm_planner import DCMPlan, DCMReference
from g1_torque_model import torque_limits
from gait_planner import GaitPhase, GaitSchedule, Phase
from lab6_common import (
    DT,
    foot_contact_state,
    load_g1_pinocchio,
    load_g1_torque_mujoco,
    mj_state_to_pin,
    pin_point_to_world,
)
from m3_walking import LEFT_FOOT, RIGHT_FOOT, build
from wb_id_qp import QPInfeasible


class OptimizedGait(GaitSchedule):
    """Lab 6의 swing/reference를 재사용하고, 시간표·착지 좌표만 새 계획으로 교체한다."""

    def __init__(
        self,
        schedule: WalkingSchedule,
        feet: np.ndarray,
        homes: np.ndarray,
        com_home: np.ndarray,
    ) -> None:
        """발의 z는 실제 초기 pose를 유지한다. 평지·고정 yaw 전용이다."""
        self.optimized_schedule = schedule
        self.optimized_feet = np.asarray(feet).copy()
        cfg = schedule.config
        super().__init__(
            LEFT_FOOT,
            RIGHT_FOOT,
            homes[0],
            homes[1],
            com_home,
            n_steps=cfg.steps,
            t_initial=cfg.settle_time,
            t_double=cfg.double_support,
            t_single=cfg.single_support,
            step_length=cfg.step_length,
            step_width=cfg.step_width,
            step_height=cfg.step_height,
            first_swing=(LEFT_FOOT, RIGHT_FOOT)[cfg.first_swing],
            close_stance=True,
        )

    def _build_timeline(self) -> None:
        """상위 클래스의 phase 자료구조를 동일한 최적화 시간표로 구성한다."""
        names = (self.left_frame, self.right_frame)
        for index, phase in enumerate(self.optimized_schedule.phases):
            positions = {
                name: np.r_[
                    self.optimized_feet[phase.feet[side]], self.foot_home[name][2]
                ]
                for side, name in enumerate(names)
            }
            kind = (
                (Phase.SINGLE_LEFT, Phase.SINGLE_RIGHT)[phase.swing]
                if phase.swing >= 0
                else Phase.DOUBLE
            )
            if index == len(self.optimized_schedule.phases) - 1:
                kind = Phase.DONE
            self._phases.append(GaitPhase(kind, phase.start, phase.end))
            self._foot_state[index] = positions
            if phase.swing >= 0:
                name = names[phase.swing]
                target = np.r_[
                    self.optimized_feet[phase.landing], self.foot_home[name][2]
                ]
                self._swing_of_phase[index] = (name, positions[name].copy(), target)
        self.total_duration = self.optimized_schedule.duration
        self.final_foot_positions = {k: v.copy() for k, v in positions.items()}
        midpoint = np.mean(list(positions.values()), axis=0)
        self.total_advance = float(midpoint[0] - self._home_midpoint[0])


class OptimizedReference:
    """WalkingPlan을 Lab 6 DCMTask 입력으로 변환한다."""

    def __init__(self, plan: WalkingPlan) -> None:
        """검증된 계획의 LIPM 주파수를 사용한다."""
        self.plan = plan
        self.omega = plan.omega

    def reference(self, t: float) -> DCMReference:
        """Constant-height LIPM에서 수평 VRP=ZMP로 변환한다."""
        value = self.plan.sample(t)
        return DCMReference(
            value["xi"], value["xi_dot"], value["zmp"], value["com"], value["velocity"]
        )


def environment_info() -> dict:
    """수치 실험의 Python·플랫폼·패키지·외부 모델 revision을 기록한다."""
    model_dir = ROOT / "third_party" / "mujoco_menagerie"
    revision = subprocess.run(
        ["git", "-C", str(model_dir), "rev-parse", "HEAD"],
        capture_output=True,
        text=True,
        check=False,
    )
    return {
        "python": platform.python_version(),
        "platform": platform.platform(),
        "packages": {
            name: importlib.metadata.version(name)
            for name in ("mujoco", "pin", "numpy", "scipy", "osqp")
        },
        "menagerie_commit": revision.stdout.strip()
        if revision.returncode == 0
        else "unknown",
    }


def run(
    config: WalkConfig,
    mode: str,
    output: Path,
    optimize_feet: bool = False,
    video: bool = False,
) -> dict:
    """새 모델에서 시작해 보행을 실행하고, 실패하더라도 측정 기록을 보존한다."""
    output.mkdir(parents=True, exist_ok=True)
    mj_model, mj_data = load_g1_torque_mujoco(timestep=DT)
    pin_model, pin_data = load_g1_pinocchio()
    stack, _, _, qp, controller = build(
        mj_model,
        mj_data,
        pin_model,
        pin_data,
        step_length=config.step_length,
        n_steps=config.steps,
    )
    homes = np.array(
        [
            pin_point_to_world(pin_data.oMf[pin_model.getFrameId(name)].translation)
            for name in (LEFT_FOOT, RIGHT_FOOT)
        ]
    )
    com = pin_point_to_world(pin_data.com[0])
    cfg = replace(
        config,
        # 발목 frame은 바닥에서 약 35 mm 위에 있다. LIPM 높이는 실제 지면 기준이다.
        com_height=float(com[2] - mj_model.geom_pos[mj_model.geom("floor").id, 2]),
        first_swing=0 if homes[0, 0] <= homes[1, 0] + 1e-4 else 1,
    )
    schedule = build_schedule(cfg, homes[:, :2])
    plan = None
    if mode == "optimized":
        initial = np.zeros((2, 3))
        initial[:, 0] = com[:2]
        initial[:, 1] = pin_data.vcom[0][:2]
        plan = solve_walk(schedule, initial, optimize_feet=optimize_feet)
        export_plan(plan, output / "reference")
        gait = OptimizedGait(schedule, plan.feet, homes, com)
        reference = OptimizedReference(plan)
    else:
        gait = OptimizedGait(schedule, schedule.feet, homes, com)
        reference = DCMPlan(gait, cfg.com_height, com, settle_sweep=1.0)
    controller.schedule = gait
    controller.dcm_plan = reference
    controller.com_task.omega = reference.omega
    controller.vrp_shrink = 0.8  # 25 mm 반폭의 80%; 계획의 20 mm 반폭과 일치
    qp.max_iterations = 20000  # 오프라인 실험: 실시간 속도보다 수렴을 우선한다.
    limits = torque_limits(mj_model)
    rows = []
    failure = None
    start_x = float(com[0])
    writer = renderer = None
    if video:
        import imageio.v2 as imageio

        renderer = WalkingVisualizer(mj_model, 7)
        writer = imageio.get_writer(
            str(output / "walk.mp4"), fps=VIS_FPS, codec="libx264"
        )
        planned_path = np.array(
            [
                np.r_[reference.reference(t).com, cfg.com_height]
                for t in np.linspace(0, gait.total_duration, 100)
            ]
        )
        seed_feet = schedule.feet.copy()
        optimized_feet = plan.feet.copy() if plan is not None else schedule.feet.copy()
        seed_feet[:, 0] += cfg.foot_offset_x
        optimized_feet[:, 0] += cfg.foot_offset_x

    elapsed = 0.0
    try:
        for tick in range(int(np.ceil(gait.total_duration / DT))):
            t = tick * DT
            q, v = mj_state_to_pin(mj_data)
            stack.update_dynamics(q, v)
            target = controller.update_targets(t)
            try:
                result = qp.solve(stack, q, v)
            except QPInfeasible as error:
                failure = f"whole-body QP: {error}"
                break
            if (
                result.status
                not in ("solved", "solved inaccurate", "maximum iterations reached")
                or not np.isfinite(result.tau).all()
            ):
                failure = f"unusable torque solution: {result.status}"
                break
            residuals = result.constraint_violations
            if (
                not residuals
                or not all(np.isfinite(value) for value in residuals.values())
                or max(residuals.values()) > 0.05
            ):
                failure = f"whole-body constraint residual: {residuals}"
                break
            tau_violation = float(
                np.maximum(
                    np.maximum(limits[:, 0] - result.tau, result.tau - limits[:, 1]), 0
                ).max()
            )
            mj_data.ctrl[:] = result.tau
            mujoco.mj_step(mj_model, mj_data)
            elapsed = min((tick + 1) * DT, gait.total_duration)
            controller.record(t, target, result.tau)
            left, right = foot_contact_state(mj_model, mj_data)
            # 기록 시점은 물리 step 이후다. Pinocchio 상태도 같은 시점으로 갱신한다.
            q, v = mj_state_to_pin(mj_data)
            stack.update_dynamics(q, v)
            actual = pin_point_to_world(pin_data.com[0])
            desired = reference.reference(elapsed)
            dcm = actual[:2] + pin_data.vcom[0][:2] / reference.omega
            rows.append(
                dict(
                    time_s=elapsed,
                    com_x_m=float(actual[0]),
                    com_y_m=float(actual[1]),
                    ref_com_x_m=float(desired.com[0]),
                    ref_com_y_m=float(desired.com[1]),
                    dcm_error_m=float(np.linalg.norm(dcm - desired.xi)),
                    zmp_margin_mm=float(controller.log.zmp_margin_mm[-1]),
                    pelvis_z_m=float(mj_data.qpos[2]),
                    left_contact=int(left),
                    right_contact=int(right),
                    torque_violation_nm=tau_violation,
                    qp_iterations=int(result.iterations),
                    qp_solve_ms=float(result.solve_time_ms),
                    qp_status=result.status,
                    **{
                        f"qp_{key}_violation": value for key, value in residuals.items()
                    },
                )
            )
            if writer is not None and elapsed + 1e-9 >= renderer.frames / VIS_FPS:
                writer.append_data(
                    renderer.render(
                        mj_data,
                        Overlay(
                            time=elapsed,
                            com=actual,
                            target=desired.com,
                            path=planned_path,
                            seed_feet=seed_feet,
                            planned_feet=optimized_feet,
                        ),
                    )
                )

            if mj_data.qpos[2] < 0.50 or not np.isfinite(mj_data.qpos).all():
                failure = "fall or non-finite state"
                break
            if tau_violation > 1e-3:
                failure = "actuator torque limit violation"
                break
    finally:
        if writer is not None:
            writer.close()
        if renderer is not None:
            renderer.close()
    if rows:
        with (output / "telemetry.csv").open("w", newline="") as file:
            csv_writer = csv.DictWriter(
                file, fieldnames=list(rows[0]), lineterminator="\n"
            )
            csv_writer.writeheader()
            csv_writer.writerows(rows)
    completed = sum(
        phase.swing >= 0 and phase.end <= elapsed + 1e-8 for phase in schedule.phases
    )
    distance = rows[-1]["com_x_m"] - start_x if rows else 0.0
    inside = controller.stance_fraction_zmp_inside()
    final_error = (
        float(np.linalg.norm(actual[:2] - reference.reference(gait.total_duration).com))
        if rows
        else None
    )
    checks = {
        "final_com_error": final_error is not None and final_error < 0.03,
        "full_duration": failure is None and elapsed >= gait.total_duration - 1e-8,
        "all_swings_completed": completed == cfg.steps + 1,
        "distance": distance >= 0.9 * gait.total_advance,
        "zmp_inside": inside > 0.9,
        "measured_contact": bool(rows)
        and all(row["left_contact"] or row["right_contact"] for row in rows),
        "torque_limits": bool(rows)
        and max(row["torque_violation_nm"] for row in rows) <= 1e-3,
    }
    report = {
        "mode": mode,
        "optimize_feet": optimize_feet,
        "passed": all(checks.values()),
        "checks": checks,
        "failure": failure,
        "final_com_error_m": final_error,
        "completed_swings": completed,
        "planned_swings": cfg.steps + 1,
        "elapsed_s": elapsed,
        "planned_duration_s": gait.total_duration,
        "distance_m": distance,
        "planned_distance_m": gait.total_advance,
        "zmp_inside_fraction": inside,
        "controller_flight_ticks": controller.flight_ticks,
        "qp_nonconverged_ticks": sum(
            r["qp_status"] == "maximum iterations reached" for r in rows
        ),
        "dcm_rms_m": float(np.sqrt(np.mean([row["dcm_error_m"] ** 2 for row in rows])))
        if rows
        else None,
        "config": asdict(cfg),
        "qp_max_iterations": qp.max_iterations,
        "qp_max_violations": {
            key: max((r[f"qp_{key}_violation"] for r in rows), default=None)
            for key in (
                "base_dynamics",
                "contact_acceleration",
                "contact_wrench",
                "raw_torque",
            )
        },
        "environment": environment_info(),
    }
    if video:
        report["video_sha256"] = hashlib.sha256(
            (output / "walk.mp4").read_bytes()
        ).hexdigest()
    (output / "summary.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    )
    if rows:
        plot_run(rows, output / "tracking.png")
    return report


def plot_run(rows: list[dict], path: Path) -> None:
    """기준·실측 CoM, DCM 오차와 실제 접촉을 표시한다."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    times = [r["time_s"] for r in rows]
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(times, [r["com_x_m"] for r in rows], label="measured CoM x")
    axes[0].plot(times, [r["ref_com_x_m"] for r in rows], label="reference CoM x")
    axes[0].set_ylabel("m")
    axes[0].legend()
    axes[1].plot(times, [1000 * r["dcm_error_m"] for r in rows])
    axes[1].set_ylabel("DCM error (mm)")
    axes[2].plot(times, [r["left_contact"] + r["right_contact"] for r in rows])
    axes[2].set(ylabel="measured contacts", xlabel="time (s)")
    for ax in axes:
        ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main() -> None:
    """Baseline/optimized를 독립 초기 상태에서 실행한다. 실패는 exit 1이다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode", choices=["baseline", "optimized"], default="optimized"
    )
    parser.add_argument("--steps", type=int, default=4)
    parser.add_argument(
        "--optimize-feet",
        action="store_true",
        default=None,
        help="optimized 모드의 기본 설정: 착지 위치도 함께 최적화",
    )
    parser.add_argument("--single-support", type=float, default=0.65)
    parser.add_argument("--double-support", type=float, default=0.25)
    parser.add_argument(
        "--settle-time",
        type=float,
        default=2.0,
        help="초기/종료 양발 지지 시간; G1의 넓은 초기 stance에 맞춘 값",
    )
    parser.add_argument("--video", action="store_true")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "media" / "g1",
    )
    args = parser.parse_args()
    if args.optimize_feet is None:
        args.optimize_feet = args.mode == "optimized"
    if args.mode == "baseline" and args.optimize_feet:
        parser.error("--optimize-feet는 optimized 모드에서 사용하세요.")
    try:
        report = run(
            WalkConfig(
                steps=args.steps,
                single_support=args.single_support,
                double_support=args.double_support,
                settle_time=args.settle_time,
            ),
            args.mode,
            args.output / ("optimized_feet" if args.optimize_feet else args.mode),
            args.optimize_feet,
            args.video,
        )
    except (PlanningError, ValueError, FileNotFoundError) as error:
        parser.exit(1, f"실행 실패: {error}\n")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
