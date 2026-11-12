"""Audited G1 walking: smooth swing, measured contacts and physical disturbances."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import platform
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import mujoco
import numpy as np
from smooth_swing import ROOT, LegacyGait, SmoothGait

sys.path.insert(0, str(ROOT))
from dcm_planner import DCMPlan
from lab6_common import (
    DT,
    G1_MJCF_PATH,
    load_g1_pinocchio,
    load_g1_torque_mujoco,
    measured_zmp,
    mj_state_to_pin,
    pin_point_to_world,
    point_in_support_polygon,
)
from m3_walking import LEFT_FOOT, RIGHT_FOOT, build
from wb_id_qp import QPInfeasible
from wb_tasks import CoMTask, FrameOrientationTask

from tools.walking_evidence import source_fingerprints
from tools.walking_visuals import FPS, Overlay, WalkingVisualizer, ground_contacts


@dataclass(frozen=True)
class WalkCase:
    """Environment changes are applied to MuJoCo, never supplied to the controller."""

    name: str = "nominal"
    steps: int = 4
    step_length: float = 0.08
    friction: float = 0.6
    mass_ratio: float = 1.0
    push_force: float = 0.0
    push_axis: int = 1
    push_start: float = 1.60
    push_duration: float = 0.15

    def __post_init__(self):
        values = (
            self.step_length,
            self.friction,
            self.mass_ratio,
            self.push_force,
            self.push_start,
            self.push_duration,
        )
        if (
            not all(np.isfinite(values))
            or not isinstance(self.steps, int)
            or isinstance(self.steps, bool)
            or self.steps < 2
        ):
            raise ValueError("Finite case settings and at least two steps are required")
        if self.step_length < 0 or self.push_start < 0:
            raise ValueError("Step length and push time must be nonnegative")
        if min(
            self.friction, self.mass_ratio, self.push_duration
        ) <= 0 or self.push_axis not in (0, 1):
            raise ValueError("Invalid physical environment")


def rebuild_plan(schedule, previous):
    """Recompute DCM from the same initial state and revised future footholds."""
    return DCMPlan(
        schedule,
        previous.com_height,
        previous.com_home,
        omega=previous.omega,
        foot_offset=previous.foot_offset,
        settle_sweep=previous.settle_sweep,
    )


def acceptance(metrics):
    """Fixed physical gates shared by every case; no case-specific thresholds."""
    return {
        "complete": metrics["completed"] and metrics["failure"] is None,
        "no_flight": metrics["flight_ticks"] == 0,
        "all_landings": metrics["landings_completed"] == metrics["planned_landings"],
        "actual_liftoff": metrics["airborne_swings"] == metrics["planned_landings"],
        "swing_clearance": metrics["minimum_swing_peak_m"] >= 0.008,
        "landing_accuracy": metrics["max_landing_error_m"] <= 0.03,
        "progress": metrics["distance_m"] >= 0.8 * metrics["planned_distance_m"],
        "zmp": metrics["zmp_inside_fraction"] >= 0.90,
        "joint_limits": metrics["joint_limit_violation_rad"] <= 0.002,
        "no_clipping": metrics["clipped_ticks"] == 0,
        "qp_convergence": metrics["qp_nonconverged_ticks"] == 0,
        "base_dynamics": metrics["qp_violations"]["base_dynamics"] <= 0.02,
        "contact_acceleration": metrics["qp_violations"]["contact_acceleration"]
        <= 0.01,
        "contact_wrench": metrics["qp_violations"]["contact_wrench"] <= 0.05,
        "raw_torque": metrics["qp_violations"]["raw_torque"] <= 1e-6,
        "torque_buffer": metrics["qp_violations"]["torque_buffer"] <= 0.01,
        "no_unexpected_contact": metrics["unexpected_contact_ticks"] == 0,
    }


def run(case: WalkCase, output: Path, *, profile="smooth", video=False, adapter=None):
    """Integrate actual torque dynamics; only rendering uses additional scene geoms."""
    output.mkdir(parents=True, exist_ok=True)
    source_sha256 = source_fingerprints()
    m, d = load_g1_torque_mujoco()
    m.geom_friction[:, 0] = case.friction
    m.body_mass[1:] *= case.mass_ratio
    m.body_inertia[1:] *= case.mass_ratio
    mujoco.mj_setConst(m, d)
    # mj_setConst uses qpos0 internally; restore the initial keyframe before settling.
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    pm, pd = load_g1_pinocchio()
    stack, schedule, plan, qp, controller = build(
        m, d, pm, pd, step_length=case.step_length, n_steps=case.steps
    )
    schedule.step_width = 0.18
    schedule.t_single = 0.85
    schedule.t_double = 0.35
    schedule.step_height = 0.025
    if profile not in ("smooth", "legacy"):
        raise ValueError("Unknown swing profile")
    gait_class = (
        getattr(adapter, "gait_type", SmoothGait) if profile == "smooth" else LegacyGait
    )
    schedule = gait_class.from_schedule(schedule)
    controller.schedule = schedule
    # This scene's ground is z=0: the ankle frame is above it, not the LIPM ground plane.
    plan = DCMPlan(
        schedule,
        float(schedule.com_home[2]),
        schedule.com_home,
        foot_offset=plan.foot_offset,
        settle_sweep=plan.settle_sweep,
    )
    controller.dcm_plan = plan
    controller.com_task.omega = plan.omega
    orientation = stack.add(
        FrameOrientationTask(
            LEFT_FOOT, pm, weight=1e3, gain=60.0, name="swing_orientation"
        )
    )
    orientation.capture_current(pd)
    height_task = stack.add(
        CoMTask(pm, axes=(2,), weight=100.0, gain=40.0, name="height")
    )
    height_task.set_target(pin_point_to_world(pd.com[0]))
    controller.swing_orientation_task = orientation
    controller.swing_orientation_weight = 1e3
    controller.contact_sync_before_tasks = True
    controller.contact_state_reader = lambda model, data: ground_contacts(model, data)[
        2
    ]
    qp.backend = "proxqp"
    qp.max_iterations = 100
    qp.tolerance = 1e-7
    qp.torque_margin = 0.1
    qp.contact_velocity_gain = 20.0
    qp.contact_orientation_gain = 80.0
    original_feet = np.array([s[2].copy() for s in schedule._swing_of_phase.values()])
    frames = [pm.getFrameId(n) for n in (LEFT_FOOT, RIGHT_FOOT)]
    bodies = [m.body(n).id for n in (LEFT_FOOT, RIGHT_FOOT)]
    pelvis = m.body("pelvis").id
    hinged = np.flatnonzero(m.jnt_limited)
    start_x = float(d.subtree_com[0, 0])
    start_clock = time.perf_counter()
    rows, landing_errors = [], []
    airborne_swings, touchdowns = set(), {}
    peak_heights = dict.fromkeys(schedule._swing_of_phase, 0.0)
    checked_landings = set()
    maxima = dict.fromkeys(
        (
            "base_dynamics",
            "contact_acceleration",
            "contact_wrench",
            "raw_torque",
            "torque_buffer",
        ),
        0.0,
    )
    failure = None
    writer = renderer = None
    pending = output / "walk.pending.mp4"
    if video:
        import imageio.v2 as imageio

        writer = imageio.get_writer(str(pending), fps=FPS, codec="libx264")
        renderer = WalkingVisualizer(m, 6)
    try:
        for tick in range(round(schedule.total_duration / DT)):
            t = tick * DT
            if tick % 2000 == 0:
                print(
                    f"{case.name}/{profile}: {t:.1f}/{schedule.total_duration:.1f}s",
                    flush=True,
                )
            q, v = mj_state_to_pin(d)
            stack.update_dynamics(q, v)
            if adapter is not None:
                try:
                    plan = adapter.update(t, schedule, plan, controller, pd)
                except ValueError as error:
                    failure = f"Retargeting rejected: {error}"
                    break
                controller.dcm_plan = plan
            # Reject loss of support before any fictitious stance wrench can be used.
            if not any(ground_contacts(m, d)[2]):
                failure = "measured support lost before control"
                break
            controller.update_targets(t)
            before_feet = np.array(
                [pin_point_to_world(pd.oMf[f].translation) for f in frames]
            )
            solved_at = time.perf_counter()
            try:
                result = qp.solve(stack, q, v)
            except QPInfeasible as error:
                failure = str(error)
                break
            solve_ms = 1000 * (time.perf_counter() - solved_at)
            for key, value in result.constraint_violations.items():
                maxima[key] = max(maxima[key], value)
            thresholds = {
                "base_dynamics": 0.02,
                "contact_acceleration": 0.01,
                "contact_wrench": 0.05,
                "raw_torque": 1e-6,
                "torque_buffer": 0.01,
            }
            if result.status not in ("solved", "solved inaccurate") or any(
                value > thresholds[key]
                for key, value in result.constraint_violations.items()
            ):
                failure = f"QP rejected at {t:.3f}s: {result.status}; {result.constraint_violations}"
                break
            if not np.isfinite(result.tau).all():
                failure = "nonfinite torque"
                break
            d.xfrc_applied[:] = 0
            active_push = case.push_start <= t < case.push_start + case.push_duration
            if active_push:
                d.xfrc_applied[pelvis, case.push_axis] = case.push_force
            d.ctrl[:] = result.tau
            mujoco.mj_step(m, d)
            mujoco.mj_forward(m, d)
            q, v = mj_state_to_pin(d)
            stack.update_dynamics(q, v)
            feet = np.array([pin_point_to_world(pd.oMf[f].translation) for f in frames])
            loaded = ground_contacts(m, d)[2]
            xi = controller.com_task.current_dcm(pd)
            expected = plan.reference(t + DT)
            zmp = measured_zmp(m, d)
            margin = point_in_support_polygon(m, d, zmp) if zmp is not None else -1.0
            unexpected = False
            peak_normal = 0.0
            force = np.zeros(6)
            for ci, contact in enumerate(d.contact):
                mujoco.mj_contactForce(m, d, ci, force)
                if force[0] <= 1e-5:
                    continue
                peak_normal = max(peak_normal, float(force[0]))
                pair = {
                    int(m.geom_bodyid[contact.geom1]),
                    int(m.geom_bodyid[contact.geom2]),
                }
                if not (0 in pair and any(b in pair for b in bodies)):
                    unexpected = True
            for idx, (foot, _, target) in schedule._swing_of_phase.items():
                phase = schedule._phases[idx]
                side = 0 if foot == LEFT_FOOT else 1
                if phase.t_start < t < phase.t_end:
                    peak_heights[idx] = max(
                        peak_heights[idx], float(feet[side, 2] - target[2])
                    )
                if phase.t_start < t < phase.t_end and not loaded[side]:
                    airborne_swings.add(idx)
                if idx in airborne_swings and idx not in touchdowns and loaded[side]:
                    touchdowns[idx] = {
                        "time_s": t + DT,
                        "vertical_velocity_m_s": float(
                            (feet[side, 2] - before_feet[side, 2]) / DT
                        ),
                    }
                if idx not in checked_landings and t + DT >= phase.t_end + 0.10:
                    landing_errors.append(float(np.linalg.norm(feet[side] - target)))
                    if loaded[side]:
                        checked_landings.add(idx)
            limits = m.jnt_range[hinged]
            values = d.qpos[m.jnt_qposadr[hinged]]
            joint_violation = float(
                np.maximum(
                    np.maximum(limits[:, 0] - values, values - limits[:, 1]), 0
                ).max()
            )
            rows.append(
                dict(
                    t=t + DT,
                    cx=float(d.subtree_com[0, 0]),
                    cy=float(d.subtree_com[0, 1]),
                    pelvis_z=float(d.qpos[2]),
                    dcm_error_m=float(np.linalg.norm(xi - expected.xi)),
                    left_contact=int(loaded[0]),
                    right_contact=int(loaded[1]),
                    zmp_margin_m=float(margin),
                    joint_violation_rad=joint_violation,
                    unexpected_contact=int(unexpected),
                    clipped=int(result.constraint_violations["raw_torque"] > 1e-6),
                    qp_converged=int(result.status in ("solved", "solved inaccurate")),
                    qp_status=result.status,
                    qp_ms=solve_ms,
                    peak_contact_force_n=peak_normal,
                    tau_max=float(np.max(np.abs(result.tau))),
                    push_x_n=float(d.xfrc_applied[pelvis, 0]),
                    push_y_n=float(d.xfrc_applied[pelvis, 1]),
                    **{
                        f"{side}_{axis}": float(feet[i, j])
                        for i, side in enumerate(("left", "right"))
                        for j, axis in enumerate("xyz")
                    },
                    **{
                        f"residual_{key}": value
                        for key, value in result.constraint_violations.items()
                    },
                )
            )
            if writer is not None and t + DT + 1e-9 >= renderer.frames / FPS:
                planned = np.array([s[2] for s in schedule._swing_of_phase.values()])
                path = None
                phase_index, phase = schedule._phase_at(t)
                if phase_index in schedule._swing_of_phase:
                    path = np.array(
                        [
                            schedule.reference(sample).swing_position
                            for sample in np.linspace(t, phase.t_end - 1e-8, 30)
                        ]
                    )
                push = np.zeros(3)
                push[case.push_axis] = case.push_force if active_push else 0
                writer.append_data(
                    renderer.render(
                        d,
                        Overlay(
                            time=t + DT,
                            com=d.subtree_com[0].copy(),
                            target=expected.xi,
                            dcm=xi,
                            path=path,
                            seed_feet=original_feet,
                            planned_feet=planned,
                            push=push if active_push else None,
                        ),
                    )
                )
            if not any(loaded) or d.qpos[2] < 0.5 or not np.isfinite(d.qpos).all():
                failure = "flight, fall or nonfinite state"
                break
    finally:
        d.xfrc_applied[:] = 0
        if writer is not None:
            writer.close()
            renderer.close()
    telemetry = output / "telemetry.csv"
    if rows:
        with telemetry.open("w", newline="") as file:
            csv_writer = csv.DictWriter(
                file, fieldnames=list(rows[0]), lineterminator="\n"
            )
            csv_writer.writeheader()
            csv_writer.writerows(rows)
    metrics = {
        "case": asdict(case),
        "profile": profile,
        "failure": failure,
        "source_sha256": source_sha256,
        "environment": {
            "python": platform.python_version(),
            "platform": platform.platform(),
            "mujoco": mujoco.__version__,
            "numpy": np.__version__,
            "model_xml_sha256": hashlib.sha256(
                Path(G1_MJCF_PATH).read_bytes()
            ).hexdigest(),
        },
        "applied_impulse_ns": [
            DT * sum(r[f"push_{axis}_n"] for r in rows) for axis in "xy"
        ],
        "controller_settings": {
            "backend": qp.backend,
            "qp_tolerance": qp.tolerance,
            "qp_max_iterations": qp.max_iterations,
            "torque_margin_nm": qp.torque_margin,
            "contact_velocity_gain": qp.contact_velocity_gain,
            "contact_orientation_gain": qp.contact_orientation_gain,
            "step_width": schedule.step_width,
            "single_support_s": schedule.t_single,
            "double_support_s": schedule.t_double,
            "step_height_m": schedule.step_height,
            "swing_orientation_weight": controller.swing_orientation_weight,
            "swing_orientation_gain": orientation.gain,
            "contact_sync_before_tasks": True,
            "contact_detection": "positive ground normal force",
            "com_height_weight": height_task.weight,
            "com_height_gain": height_task.gain,
        },
        "completed": bool(rows and rows[-1]["t"] >= schedule.total_duration - DT / 2),
        "duration_s": schedule.total_duration,
        "elapsed_s": rows[-1]["t"] if rows else 0,
        "wall_seconds": time.perf_counter() - start_clock,
        "sample_interval_s": DT,
        "planned_landings": len(schedule._swing_of_phase),
        "landings_completed": len(checked_landings),
        "airborne_swings": len(airborne_swings),
        "touchdowns": list(touchdowns.values()),
        "minimum_swing_peak_m": min(peak_heights.values()),
        "landings": [
            {
                "side": "left" if entry[0] == LEFT_FOOT else "right",
                "start_s": schedule._phases[index].t_start,
                "end_s": schedule._phases[index].t_end,
                "target": entry[2].tolist(),
            }
            for index, entry in schedule._swing_of_phase.items()
        ],
        "max_landing_error_m": max(landing_errors, default=1.0),
        "distance_m": float(d.subtree_com[0, 0]) - start_x,
        "planned_distance_m": schedule.total_advance,
        "flight_ticks": sum(
            not (r["left_contact"] or r["right_contact"]) for r in rows
        ),
        "zmp_inside_fraction": float(np.mean([r["zmp_margin_m"] >= 0 for r in rows]))
        if rows
        else 0,
        "joint_limit_violation_rad": max(
            (r["joint_violation_rad"] for r in rows), default=1
        ),
        "clipped_ticks": sum(r["clipped"] for r in rows),
        "qp_nonconverged_ticks": sum(not r["qp_converged"] for r in rows),
        "qp_violations": maxima,
        "unexpected_contact_ticks": sum(r["unexpected_contact"] for r in rows),
        "dcm_rms_m": float(np.sqrt(np.mean([r["dcm_error_m"] ** 2 for r in rows])))
        if rows
        else 1,
        "recovery_dcm_rms_m": float(
            np.sqrt(np.mean([r["dcm_error_m"] ** 2 for r in rows if r["t"] >= 2.3]))
        )
        if rows and rows[-1]["t"] >= 2.3
        else 1,
        "peak_contact_force_n": max(
            (r["peak_contact_force_n"] for r in rows), default=0
        ),
        "qp_p95_ms": float(np.percentile([r["qp_ms"] for r in rows], 95))
        if rows
        else 0,
        "telemetry_sha256": hashlib.sha256(telemetry.read_bytes()).hexdigest()
        if rows
        else None,
        "adaptation": adapter.events if adapter is not None else [],
    }
    metrics["checks"] = acceptance(metrics)
    metrics["passed"] = all(metrics["checks"].values())
    if video:
        metrics["frames"], metrics["fps"] = renderer.frames, FPS
        if metrics["passed"]:
            pending.replace(output / "walk.mp4")
            metrics["video_sha256"] = hashlib.sha256(
                (output / "walk.mp4").read_bytes()
            ).hexdigest()
        else:
            pending.unlink(missing_ok=True)
    (output / "summary.json").write_text(
        json.dumps(metrics, indent=2, allow_nan=False) + "\n"
    )
    print(json.dumps(metrics, indent=2), flush=True)
    return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=["smooth", "legacy"], default="smooth")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "media/demo",
    )
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--push", type=float, default=0)
    args = parser.parse_args()
    if not run(
        WalkCase(push_force=args.push),
        args.output,
        profile=args.profile,
        video=args.video,
    )["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
