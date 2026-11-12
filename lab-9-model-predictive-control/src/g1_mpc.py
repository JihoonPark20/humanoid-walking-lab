"""W4b: Pinocchio 측정 → centroidal MPC → Whole-body QP → MuJoCo G1."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import pinocchio as pin

from tools.walking_visuals import FPS as VIS_FPS
from tools.walking_visuals import Overlay, WalkingVisualizer

ROOT = Path(__file__).resolve().parents[2]
for lab in (
    "lab-7-walking-optimization",
    "lab-8-centroidal-optimization",
    "lab-6-whole-body-control",
):
    sys.path.insert(0, str(ROOT / lab / "src"))
import mujoco
from centroidal_io import contacts_from_walk, reference_from_walk, save_centroidal
from centroidal_model import CentroidalConfig
from centroidal_mpc import CentroidalMPC
from centroidal_optimizer import CentroidalFailure, CentroidalSolver
from centroidal_rti import CentroidalRTI
from g1_walk import OptimizedGait, environment_info
from lab6_common import (
    DT,
    foot_contact_state,
    load_g1_pinocchio,
    load_g1_torque_mujoco,
    mj_state_to_pin,
    pin_point_to_world,
)
from lipm_optimizer import PlanningError, solve_walk
from m3_walking import LEFT_FOOT, RIGHT_FOOT, build
from mpc_experiment import rti_diagnostics, timing_summary
from walking_model import WalkConfig, build_schedule
from wb_id_qp import QPInfeasible
from wb_tasks import CentroidalAngularMomentumTask, CoMTask


class CentroidalCoMTask(CoMTask):
    """CoM 3축 위치/속도와 force-derived acceleration feedforward를 추종한다."""

    def __init__(self, model: pin.Model) -> None:
        """Lab 6 Jacobian과 drift 계산을 재사용한다."""
        super().__init__(model, axes=(0, 1, 2), weight=1e4, gain=40.0)
        self.velocity = np.zeros(3)
        self.acceleration = np.zeros(3)

    def set_plan(self, state: np.ndarray, wrench: np.ndarray, mass: float) -> None:
        """Centroidal 상태와 wrench를 같은 물리 단위의 reference로 변환한다."""
        self.target = state[:3].copy()
        self.velocity = state[3:6] / mass
        self.acceleration = wrench[:, :3].sum(axis=0) / mass + [0, 0, -9.81]

    def desired_acceleration(
        self,
        model: pin.Model,
        data: pin.Data,
        q: np.ndarray,
        v: np.ndarray,
        damping_gain: float | None = None,
    ) -> np.ndarray:
        """Feedforward + PD. Lab 6 QP가 Jdot*v를 별도로 뺀다."""
        return (
            self.acceleration
            + self.gain * (self.target - self.current_com(data))
            + 12 * (self.velocity - data.vcom[0])
        )


class MomentumFeedforwardTask(CentroidalAngularMomentumTask):
    """계획 angular momentum rate와 실측 운동량 오차를 함께 사용한다."""

    def __init__(self) -> None:
        """지나친 팔 반작용을 피하도록 낮은 우선순위의 momentum task를 만든다."""
        super().__init__(weight=10.0, gain=8.0)
        self.rate = np.zeros(3)

    def set_plan(
        self, state: np.ndarray, wrench: np.ndarray, positions: np.ndarray
    ) -> None:
        """발바닥 중심 moment를 CoM 주위의 angular momentum rate로 변환한다."""
        self.set_reference(state[6:])
        self.rate = (
            np.cross(positions - state[:3], wrench[:, :3]) + wrench[:, 3:]
        ).sum(axis=0)

    def desired_acceleration(
        self,
        model: pin.Model,
        data: pin.Data,
        q: np.ndarray,
        v: np.ndarray,
        damping_gain: float | None = None,
    ) -> np.ndarray:
        """기존 momentum damping에 계획 Ldot를 더한다."""
        return (
            super().desired_acceleration(model, data, q, v, damping_gain)
            + self.axis_scale * self.rate
        )


def measured_state(data: pin.Data, v: np.ndarray) -> np.ndarray:
    """이미 갱신된 Pinocchio 데이터로 world CoM 및 centroidal momentum을 읽는다."""
    return np.r_[pin_point_to_world(data.com[0]), data.Ag @ v]


def run(
    steps: int,
    output: Path,
    push_force: float = 5.0,
    video: bool = False,
    mode: str = "mpc",
) -> dict:
    """같은 nominal 계획을 offline/online으로 추종하고 실제 접촉과 solver 시간을 기록한다."""
    output.mkdir(parents=True, exist_ok=True)
    m, d = load_g1_torque_mujoco()
    pm, pd = load_g1_pinocchio()
    stack, _, _, qp, controller = build(m, d, pm, pd, step_length=0.08, n_steps=steps)
    q, v = mj_state_to_pin(d)
    stack.update_dynamics(q, v)
    homes = np.array(
        [
            pin_point_to_world(pd.oMf[pm.getFrameId(name)].translation)
            for name in (LEFT_FOOT, RIGHT_FOOT)
        ]
    )
    initial = measured_state(pd, v)
    mass = float(sum(item.mass for item in pm.inertias))
    cfg = WalkConfig(
        steps=steps,
        step_length=0.08,
        settle_time=2.0,
        com_height=float(initial[2]),
        first_swing=0 if homes[0, 0] <= homes[1, 0] + 1e-4 else 1,
    )
    seed_state = np.zeros((2, 3))
    seed_state[:, 0] = initial[:2]
    seed_state[:, 1] = initial[3:5] / mass
    walk = solve_walk(build_schedule(cfg, homes[:, :2]), seed_state, optimize_feet=True)
    coarse = build_schedule(replace(cfg, dt=0.1), homes[:, :2])
    contacts = contacts_from_walk(walk, coarse.times)
    reference = reference_from_walk(walk, coarse.times, mass, 0.0)
    reference[0] = initial
    ccfg = CentroidalConfig(mass=mass)
    nominal = CentroidalSolver(len(contacts.times) - 1, ccfg, True).solve(
        initial, reference, contacts
    )
    save_centroidal(nominal, output / "nominal")
    mpc = CentroidalRTI(nominal, 12) if mode == "rti" else CentroidalMPC(nominal, 12)
    rti_rows = []
    if mode == "rti":
        mpc.prepare(0)
    controller.com_task.enabled = False
    com_task = stack.add(CentroidalCoMTask(pm))
    angular_task = stack.add(MomentumFeedforwardTask())
    controller.com_task = com_task
    controller.dcm_plan = None
    controller.schedule = OptimizedGait(walk.schedule, walk.feet, homes, initial[:3])
    qp.max_iterations = 20000
    rows = []
    latencies = []
    periods = []
    failure = None
    index = -1
    decision = None
    fallbacks = 0
    pelvis = m.body("pelvis").id
    writer = renderer = None
    if video:
        import imageio.v2 as imageio

        writer = imageio.get_writer(
            str(output / "mpc.mp4"), fps=VIS_FPS, codec="libx264"
        )
        renderer = WalkingVisualizer(m, 9)

    try:
        for tick in range(round(contacts.times[-1] / DT)):
            t = tick * DT
            if tick % 1000 == 0:
                print(f"G1 {mode}: {t:.1f}/{contacts.times[-1]:.1f} s", flush=True)
            q, v = mj_state_to_pin(d)
            stack.update_dynamics(q, v)
            measured = measured_state(pd, v)
            new_index = min(
                int(np.searchsorted(contacts.times, t + 1e-9, side="right") - 1),
                len(nominal.wrenches) - 1,
            )
            if mode in ("mpc", "rti") and new_index != index:
                index = new_index
                try:
                    decision = (
                        mpc.feedback(measured)
                        if mode == "rti"
                        else mpc.solve(measured, index)
                    )
                except CentroidalFailure as error:
                    failure = str(error)
                    latencies.append(mpc.last_seconds)
                    periods.append(
                        float(contacts.times[index + 1] - contacts.times[index])
                    )
                    if mode == "rti":
                        rti_rows.append(rti_diagnostics(mpc, index))
                    break
                latencies.append(mpc.last_seconds)
                periods.append(float(contacts.times[index + 1] - contacts.times[index]))
                fallbacks += mpc.last_mode != "new"
                if mode == "rti":
                    rti_rows.append(rti_diagnostics(mpc, index))
                    if index + 1 < len(nominal.wrenches):
                        mpc.prepare(index + 1)
            plan = decision if mode in ("mpc", "rti") else nominal
            desired, wrench = plan.sample(t)
            local = min(
                np.searchsorted(plan.contacts.times, t, side="right") - 1,
                len(plan.wrenches) - 1,
            )
            controller.update_targets(t)
            com_task.set_plan(desired, wrench, mass)
            angular_task.set_plan(desired, wrench, plan.contacts.positions[local])
            try:
                result = qp.solve(stack, q, v)
            except QPInfeasible as error:
                failure = str(error)
                break
            if (
                result.status
                not in ("solved", "solved inaccurate", "maximum iterations reached")
                or not np.isfinite(result.tau).all()
            ):
                failure = f"WBC status: {result.status}"
                break
            if (
                not result.constraint_violations
                or max(result.constraint_violations.values()) > 0.05
            ):
                failure = f"WBC constraints: {result.constraint_violations}"
                break
            d.xfrc_applied[:] = 0
            if 2.3 <= t < 2.45:
                d.xfrc_applied[pelvis, 1] = push_force
            d.ctrl[:] = result.tau
            mujoco.mj_step(m, d)
            q, v = mj_state_to_pin(d)
            stack.update_dynamics(q, v)
            actual = measured_state(pd, v)
            left, right = foot_contact_state(m, d)
            goal, _ = nominal.sample(min(t + DT, contacts.times[-1]))
            rows.append(
                {
                    "t": t + DT,
                    "cx": float(actual[0]),
                    "cy": float(actual[1]),
                    "cz": float(actual[2]),
                    "com_error_m": float(np.linalg.norm(actual[:3] - goal[:3])),
                    "angular_norm": float(np.linalg.norm(actual[6:])),
                    "left_contact": int(left),
                    "right_contact": int(right),
                    "qp_status": result.status,
                    "qp_residual": max(result.constraint_violations.values()),
                }
            )
            if writer is not None and t + DT + 1e-9 >= renderer.frames / VIS_FPS:
                now = t + DT
                future_times = np.linspace(
                    now, max(now, float(plan.contacts.times[-1])), 20
                )
                predicted = np.array(
                    [plan.sample(time)[0][:3] for time in future_times]
                )
                original = np.array(
                    [nominal.sample(time)[0][:3] for time in future_times]
                )
                push = np.array([0.0, push_force, 0.0]) if 2.3 <= t < 2.45 else None
                writer.append_data(
                    renderer.render(
                        d,
                        Overlay(
                            time=now,
                            com=actual[:3],
                            target=goal[:3],
                            prediction=predicted,
                            nominal_path=original,
                            push=push,
                        ),
                    )
                )

            if d.qpos[2] < 0.50 or not np.isfinite(actual).all():
                failure = "fall/nonfinite"
                break
    finally:
        d.xfrc_applied[:] = 0
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
    final_error = rows[-1]["com_error_m"] if rows else None
    no_flight = bool(rows) and all(
        r["left_contact"] or r["right_contact"] for r in rows
    )
    complete = bool(rows and rows[-1]["t"] >= contacts.times[-1] - DT)
    report = {
        "mode": mode,
        "passed": failure is None and complete and no_flight and final_error < 0.05,
        "failure": failure,
        "completed": complete,
        "no_flight": no_flight,
        "final_com_error_m": final_error,
        "elapsed_s": rows[-1]["t"] if rows else 0.0,
        "duration_s": float(contacts.times[-1]),
        "push_force_n": push_force,
        "validated_fallbacks": fallbacks,
        "timing": timing_summary(latencies, periods),
        "environment": environment_info(),
    }
    if mode == "rti":
        report["rti"] = {
            "qp_solve_calls": mpc.qp_solve_calls,
            "sqp_iterations_per_update": 1,
            "preparation": timing_summary(
                [r["preparation_s"] for r in rti_rows], periods
            ),
            "feedback": timing_summary([r["feedback_s"] for r in rti_rows], periods),
            "updates": rti_rows,
        }
    if video:
        report["video_sha256"] = hashlib.sha256(
            (output / "mpc.mp4").read_bytes()
        ).hexdigest()
    (output / "summary.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    )
    if rows:
        plot_telemetry(output / "telemetry.csv", output / "tracking.png")
    return report


def plot_telemetry(path: Path, output: Path) -> None:
    """실측 CoM 오차·운동량과 접촉을 CSV에서 그린다."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with path.open() as file:
        rows = list(csv.DictReader(file))
    if not rows:
        return
    times = [float(r["t"]) for r in rows]
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(times, [1000 * float(r["com_error_m"]) for r in rows])
    axes[0].set_ylabel("CoM error (mm)")
    axes[1].plot(times, [float(r["angular_norm"]) for r in rows])
    axes[1].set_ylabel("|L| (kg m²/s)")
    axes[2].plot(
        times, [int(r["left_contact"]) + int(r["right_contact"]) for r in rows]
    )
    axes[2].set(ylabel="measured contacts", xlabel="time (s)")
    for ax in axes:
        ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(output, dpi=130)
    plt.close(fig)


def main() -> None:
    """검증된 G1 MPC 또는 RTI 보행을 실행한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=2)
    parser.add_argument("--mode", choices=["mpc", "rti"], default="mpc")
    parser.add_argument("--push-force", type=float, default=5.0)
    parser.add_argument("--video", action="store_true")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "media" / "g1",
    )
    args = parser.parse_args()
    try:
        report = run(
            args.steps, args.output / args.mode, args.push_force, args.video, args.mode
        )
    except (CentroidalFailure, PlanningError, ValueError) as error:
        parser.exit(1, f"실행 실패: {error}\n")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
