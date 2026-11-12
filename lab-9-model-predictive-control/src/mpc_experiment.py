"""W4 reduced-model 외란 실험. Offline 입력/feedback과 receding-horizon MPC를 비교한다."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, replace
from pathlib import Path
from time import perf_counter

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for lab in ("lab-7-walking-optimization", "lab-8-centroidal-optimization"):
    sys.path.insert(0, str(ROOT / lab / "src"))
from centroidal_estimation import VerticalMassObserver, predict_delayed_state
from centroidal_io import make_walking_problem
from centroidal_model import (
    CentroidalConfig,
    CentroidalPlan,
    Contacts,
    centroidal_step,
    validate_centroidal,
    wrench_violation,
)
from centroidal_mpc import CentroidalMPC
from centroidal_optimizer import CentroidalFailure, CentroidalSolver
from centroidal_rti import CentroidalRTI
from centroidal_sensing import PacketStateEstimator, SensorConfig, StateSensor
from lipm_mpc import LIPMMPC, MPCFailure, lipm_matrices
from lipm_optimizer import WalkingPlan, solve_walk
from walking_model import WalkConfig, build_schedule


def timing_summary(seconds: list[float], periods: list[float]) -> dict:
    """전체 solve 경로의 소요 시간과 실제 제어 deadline 초과 수를 보고한다."""
    if not seconds:
        return {
            "count": 0,
            "median_ms": None,
            "p95_ms": None,
            "p99_ms": None,
            "deadline_misses": 0,
        }
    values = np.asarray(seconds)
    return {
        "count": len(values),
        "median_ms": float(np.median(values) * 1000),
        "p95_ms": float(np.percentile(values, 95) * 1000),
        "p99_ms": float(np.percentile(values, 99) * 1000),
        "deadline_misses": int(np.sum(values > np.asarray(periods))),
    }


def run_lipm(
    plan: WalkingPlan,
    mode: str,
    push: float = 0.03,
    push_time: float = 2.1,
    horizon: int = 20,
    delay_steps: int = 0,
) -> tuple[dict, np.ndarray]:
    """속도 impulse를 같은 시각에 넣는다. 선형 모델에서 offline/feedback/MPC를 비교한다."""
    times = plan.schedule.times
    ref = plan.sample(0)
    x = np.array([ref["com"][0], ref["velocity"][0], ref["com"][1], ref["velocity"][1]])
    # ZOH 입력으로 다시 푼 offline 계획을 세 방식의 공통 reference로 쓴다.
    offline = LIPMMPC(plan, len(plan.jerks)).solve(x, 0)
    controller = LIPMMPC(plan, horizon, offline)
    history = [x.copy()]
    rows = []
    durations = []
    periods = []
    applied = False
    failure = None
    fallback = None
    for k, h in enumerate(np.diff(times)):
        t = float(times[k])
        if not applied and t >= push_time - 1e-9:
            x[3] += push
            applied = True
        measured = (
            history[max(0, len(history) - 1 - delay_steps)].copy()
            if delay_steps
            else x.copy()
        )
        ref_state = offline.states[k]
        ref_xi = ref_state[[0, 2]] + ref_state[[1, 3]] / plan.omega
        lower, upper = controller.lower[k], controller.upper[k]
        if mode == "mpc":
            started = perf_counter()
            try:
                decision = controller.solve(measured, k)
                u = decision.inputs[0]
            except MPCFailure as error:
                failure = str(error)
                try:
                    brake = controller.braking_input(x, k)
                    # 접촉 순서를 중단하고 현재 지지발을 유지하는 정지 rollout을 검증한다.
                    a, b = lipm_matrices(1.5, plan.omega)
                    stopped = a @ x + b @ brake
                    fallback = {
                        "mode": "capture_and_hold",
                        "zmp": brake.tolist(),
                        "state_after_1_5s": stopped.tolist(),
                    }
                except MPCFailure:
                    fallback = {"mode": "unavailable"}
                durations.append(perf_counter() - started)
                periods.append(float(h))
                break
            durations.append(perf_counter() - started)
            periods.append(float(h))
        elif mode == "feedback":
            xi = measured[[0, 2]] + measured[[1, 3]] / plan.omega
            u = np.clip(
                offline.inputs[k] + (1 + 3 / plan.omega) * (xi - ref_xi), lower, upper
            )
        else:
            u = offline.inputs[k]
        a, b = lipm_matrices(float(h), plan.omega)
        x = a @ x + b @ u
        history.append(x.copy())
        target_state = offline.states[k + 1]
        target = {
            "com": target_state[[0, 2]],
            "xi": target_state[[0, 2]] + target_state[[1, 3]] / plan.omega,
        }
        error = float(np.linalg.norm(x[[0, 2]] - target["com"]))
        xi_error = float(
            np.linalg.norm(x[[0, 2]] + x[[1, 3]] / plan.omega - target["xi"])
        )
        rows.append([times[k + 1], *x, *u, error, xi_error])
        if not np.isfinite(x).all() or np.linalg.norm(x[[0, 2]]) > 2:
            failure = "LIPM state diverged"
            break
    final_error = rows[-1][-2] if rows else None
    report = {
        "mode": mode,
        "model": "lipm",
        "push_m_s": push,
        "push_applied": applied,
        "delay_steps": delay_steps,
        "passed": failure is None and final_error is not None and final_error < 0.03,
        "failure": failure,
        "fallback": fallback,
        "completed_intervals": len(rows),
        "planned_intervals": len(times) - 1,
        "final_com_error_m": final_error,
        "max_dcm_error_m": max((r[-1] for r in rows), default=None),
        "timing": timing_summary(durations, periods),
    }
    return report, np.asarray(rows)


def project_wrench(
    u: np.ndarray, active: np.ndarray, config: CentroidalConfig
) -> np.ndarray:
    """Reduced plant의 마찰 오차 실험용 saturation. 실제 slip Dynamics 모델은 아니다."""
    u = u.copy()
    u[:, 2] = np.clip(u[:, 2], 0, active * config.max_normal_force)
    u[:, :2] = np.clip(
        u[:, :2], -config.friction * u[:, 2, None], config.friction * u[:, 2, None]
    )
    radii = np.array([config.half_width, config.half_length, config.yaw_radius])
    u[:, 3:] = np.clip(u[:, 3:], -u[:, 2, None] * radii, u[:, 2, None] * radii)
    return u


def run_centroidal(
    plan: CentroidalPlan,
    mode: str,
    push: float = 0.03,
    push_time: float = 2.1,
    horizon: int = 12,
    delay_steps: int = 0,
    mass_ratio: float = 1.0,
    friction: float = 0.6,
    compensate_delay: bool = True,
    adapt_mass: bool = True,
    sensor_config: SensorConfig | None = None,
    observer_gain: float = 1.0,
) -> tuple[dict, np.ndarray]:
    """Nominal solver와 mass/friction이 다른 reduced plant를 구분해 실행한다."""
    if mass_ratio <= 0:
        raise ValueError("mass_ratio는 양수여야 합니다.")
    if sensor_config is not None and (
        adapt_mass or delay_steps or not compensate_delay
    ):
        raise ValueError(
            "Packet sensing requires adapt_mass=False, delay_steps=0 and "
            "compensate_delay=True; set transport delay in SensorConfig"
        )
    plant = replace(plan.config, mass=plan.config.mass * mass_ratio, friction=friction)
    controller = (
        CentroidalRTI(plan, horizon) if mode == "rti" else CentroidalMPC(plan, horizon)
    )
    x = plan.initial.copy()
    history = [x.copy()]
    rows = []
    durations = []
    periods = []
    applied = False
    push_applied_time = None
    failure = None
    max_wrench = 0.0
    max_command_violation = 0.0
    saturation_intervals = 0
    fallbacks = 0
    rti_rows = []
    commands = []
    estimation_errors = []
    estimation_seconds = []
    plant_violations = {
        key: 0.0
        for key in ("height_violation", "velocity_violation", "angular_violation")
    }
    mass_observer = VerticalMassObserver(plan.config)
    mass_history = []
    sensor = StateSensor(sensor_config) if sensor_config is not None else None
    observer = PacketStateEstimator(plan.initial, observer_gain) if sensor else None
    observation_rows = []
    for k, h in enumerate(np.diff(plan.contacts.times)):
        t = float(plan.contacts.times[k])
        if not applied and t >= push_time - 1e-9:
            x[4] += plant.mass * push
            applied = True
            push_applied_time = t
        estimation_started = perf_counter()
        measured = (
            history[max(0, len(history) - 1 - delay_steps)].copy()
            if delay_steps
            else x.copy()
        )
        measured_index = max(0, k - delay_steps)
        if sensor is not None:
            sensor.capture(x, k)
            packet = sensor.receive(k)
            measured = observer.estimate(
                packet, k, commands, plan.contacts, controller.nominal.config
            )
            age_index = max(0, observer.last_packet_index)
            observation_rows.append(
                {
                    "time_s": t,
                    "packet_index": packet.index if packet is not None else None,
                    "last_packet_index": observer.last_packet_index,
                    "age_s": t - float(plan.contacts.times[age_index]),
                    "com_error_m": float(np.linalg.norm(measured[:3] - x[:3])),
                    "momentum_error": float(np.linalg.norm(measured[3:6] - x[3:6])),
                }
            )
        if mode in ("mpc", "rti") and adapt_mass:
            mass = mass_observer.observe(
                measured, measured_index, commands, plan.contacts
            )
            controller.update_mass(mass)
        mass_history.append(controller.nominal.config.mass)
        if delay_steps and compensate_delay and mode in ("mpc", "rti"):
            measured = predict_delayed_state(
                measured,
                measured_index,
                k,
                commands,
                plan.contacts,
                controller.nominal.config,
            )
        estimation_errors.append(float(np.linalg.norm(measured[:3] - x[:3])))
        estimation_seconds.append(perf_counter() - estimation_started)
        if mode in ("mpc", "rti"):
            try:
                decision = controller.solve(measured, k)
            except CentroidalFailure as error:
                failure = str(error)
                durations.append(controller.last_seconds + estimation_seconds[-1])
                periods.append(float(h))
                if mode == "rti":
                    rti_rows.append(rti_diagnostics(controller, k))
                break
            u = decision.wrenches[0]
            durations.append(controller.last_seconds + estimation_seconds[-1])
            periods.append(float(h))
            fallbacks += controller.last_mode != "new"
            if mode == "rti":
                rti_rows.append(rti_diagnostics(controller, k))
        else:
            u = plan.wrenches[k].copy()
            if mode == "feedback":
                target = plan.states[k]
                force = plan.config.mass * 20 * (target[:3] - measured[:3]) + 8 * (
                    target[3:6] - measured[3:6]
                )
                weights = plan.contacts.active[k] / sum(plan.contacts.active[k])
                u[:, :3] += weights[:, None] * force
                u[:, 3:] += weights[:, None] * 3 * (target[6:] - measured[6:])
        commands.append(u.copy())
        max_command_violation = max(
            max_command_violation, wrench_violation(u, plan.contacts.active[k], plant)
        )
        u = project_wrench(u, plan.contacts.active[k], plant)
        saturation_intervals += bool(np.any(abs(u - commands[-1]) > 1e-4))
        max_wrench = max(
            max_wrench, wrench_violation(u, plan.contacts.active[k], plant)
        )
        before = x.copy()
        x = centroidal_step(x, u, plan.contacts.positions[k], float(h), plant)
        # 실제 상태도 각 interval 안의 extrema까지 검사한다. 외란 impulse는 경계에서 적용된다.
        segment = CentroidalPlan(
            plant,
            Contacts(
                plan.contacts.times[k : k + 2],
                plan.contacts.positions[k : k + 1],
                plan.contacts.active[k : k + 1],
            ),
            np.array([before, x]),
            u[None],
            np.array([before, x]),
            before,
            False,
        )
        physical = validate_centroidal(segment)
        for key, previous in plant_violations.items():
            plant_violations[key] = max(previous, physical.get(key, float("inf")))
        history.append(x.copy())
        err = float(np.linalg.norm(x[:3] - plan.states[k + 1, :3]))
        rows.append(
            [plan.contacts.times[k + 1], *x, err, np.linalg.norm(x[6:]), u[:, 2].sum()]
        )
        if (
            not physical["valid"]
            or not np.isfinite(x).all()
            or not plant.min_height - 0.02 < x[2] < plant.max_height + 0.02
            or np.linalg.norm(x[:2]) > 2
        ):
            failure = "reduced plant state left operating region"
            break
    final_error = rows[-1][-3] if rows else None
    report = {
        "model": "centroidal",
        "mode": mode,
        "push_m_s": push,
        "push_applied": applied,
        "push_requested_time_s": push_time,
        "push_applied_time_s": push_applied_time,
        "delay_steps": delay_steps,
        "mass_ratio": mass_ratio,
        "plant_friction": friction,
        "delay_compensation": compensate_delay and mode in ("mpc", "rti"),
        "mass_adaptation": adapt_mass and mode in ("mpc", "rti"),
        "estimated_mass_kg": mass_history[-1] if mass_history else None,
        "mass_estimates_kg": mass_history,
        "mass_observer_updates": mass_observer.updates,
        "mass_observer_rejected": mass_observer.rejected,
        "plant_state_violations": plant_violations,
        "estimation_timing": timing_summary(
            estimation_seconds,
            list(np.diff(plan.contacts.times)[: len(estimation_seconds)]),
        ),
        "max_estimated_com_error_m": max(estimation_errors, default=0.0),
        "rms_estimated_com_error_m": float(
            np.sqrt(np.mean(np.square(estimation_errors)))
        ),
        "passed": failure is None and final_error is not None and final_error < 0.03,
        "failure": failure,
        "completed_intervals": len(rows),
        "planned_intervals": len(plan.wrenches),
        "final_com_error_m": final_error,
        "solver_failures": controller.failures,
        "validated_fallbacks": fallbacks,
        "plant_wrench_violation": max_wrench,
        "command_wrench_violation": max_command_violation,
        "saturation_intervals": saturation_intervals,
        "timing": timing_summary(durations, periods),
    }
    if sensor is not None:
        report["sensing"] = {
            "config": asdict(sensor_config),
            "observer_gain": observer_gain,
            "captured_packets": len(sensor.packets),
            "dropped_at_capture": sensor.dropped,
            "corrections": observer.corrections,
            "initialization": "known simulated reset state",
            "max_age_s": max((r["age_s"] for r in observation_rows), default=0.0),
            "observations": observation_rows,
        }
    if mode == "rti":
        report["rti"] = {
            "qp_solve_calls": controller.qp_solve_calls,
            "sqp_iterations_per_update": 1,
            "preparation": timing_summary(
                [r["preparation_s"] for r in rti_rows], periods[: len(rti_rows)]
            ),
            "feedback": timing_summary(
                [r["feedback_s"] for r in rti_rows], periods[: len(rti_rows)]
            ),
            "updates": rti_rows,
        }
    return report, np.asarray(rows)


def rti_diagnostics(controller: CentroidalRTI, index: int) -> dict:
    """한 번의 SQP/QP와 exact rollout 간 차이를 tick별로 보존한다."""
    return {
        "index": index,
        "mode": controller.last_mode,
        "preparation_s": controller.preparation_seconds,
        "feedback_s": controller.feedback_seconds,
        "qp_iterations": controller.qp_iterations,
        "qp_residual": controller.qp_residual
        if np.isfinite(controller.qp_residual)
        else None,
        "linearization_gap": controller.linearization_gap
        if np.isfinite(controller.linearization_gap)
        else None,
    }


def save_comparison(results: dict, output: Path, model: str) -> None:
    """모든 비교 실행을 보존하고 동일 축에서 오차를 표시한다."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output.mkdir(parents=True, exist_ok=True)
    reports = {name: value[0] for name, value in results.items()}
    (output / "comparison.json").write_text(
        json.dumps(reports, indent=2, ensure_ascii=False) + "\n"
    )
    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    for name, (report, rows) in results.items():
        if not len(rows):
            continue
        header = (
            "t,cx,vx,cy,vy,zmpx,zmpy,com_error,dcm_error"
            if model == "lipm"
            else "t,cx,cy,cz,px,py,pz,Lx,Ly,Lz,com_error,angular_norm,total_fz"
        )
        np.savetxt(
            output / f"{name}.csv", rows, delimiter=",", comments="", header=header
        )
        error = rows[:, -2] if model == "lipm" else rows[:, -3]
        axes[0].plot(rows[:, 0], 1000 * error, label=name)
        axes[1].plot(
            rows[:, 0], rows[:, -1] if model == "lipm" else rows[:, -2], label=name
        )
    axes[0].set_ylabel("CoM error (mm)")
    axes[0].legend()
    axes[1].set(
        ylabel="DCM error (m)" if model == "lipm" else "|L| (kg m²/s)",
        xlabel="time (s)",
    )
    for ax in axes:
        ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(output / "comparison.png", dpi=130)
    plt.close(fig)


def main() -> None:
    """선택한 reduced model의 외란 실험을 실행한다. MPC 실패면 exit 1이다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=["lipm", "centroidal"], default="lipm")
    parser.add_argument("--steps", type=int, default=2)
    parser.add_argument(
        "--push", type=float, default=0.03, help="y 방향 속도 impulse m/s"
    )
    parser.add_argument("--push-time", type=float, default=2.1)
    parser.add_argument("--horizon", type=int)
    parser.add_argument("--delay-steps", type=int, default=0)
    parser.add_argument("--mass-ratio", type=float, default=1.0)
    parser.add_argument("--friction", type=float, default=0.6)
    parser.add_argument(
        "--no-delay-compensation",
        action="store_true",
        help="이전 지연 측정 직접 사용 baseline 재현",
    )
    parser.add_argument(
        "--no-mass-adaptation",
        action="store_true",
        help="질량 추정 없이 nominal 질량 유지",
    )
    parser.add_argument(
        "--centroidal-solver", choices=["nlp", "rti", "both"], default="both"
    )
    parser.add_argument(
        "--output", type=Path, default=Path(__file__).resolve().parents[1] / "media"
    )
    args = parser.parse_args()
    if (
        args.delay_steps < 0
        or (args.horizon is not None and args.horizon < 2)
        or not np.isfinite(
            [args.push, args.push_time, args.mass_ratio, args.friction]
        ).all()
    ):
        parser.error("외란/모델 인자 오류")
    if args.model == "lipm":
        plan = solve_walk(
            build_schedule(WalkConfig(steps=args.steps)), optimize_feet=True
        )
        results = {
            mode: run_lipm(
                plan,
                mode,
                args.push,
                args.push_time,
                args.horizon or 20,
                args.delay_steps,
            )
            for mode in ("mpc",)
        }
    else:
        cfg, contacts, ref, _ = make_walking_problem(args.steps, 0.1, 0.015)
        plan = CentroidalSolver(len(contacts.times) - 1, cfg, True).solve(
            ref[0], ref, contacts
        )
        results = {
            mode: run_centroidal(
                plan,
                mode,
                args.push,
                args.push_time,
                args.horizon or 12,
                args.delay_steps,
                args.mass_ratio,
                args.friction,
                not args.no_delay_compensation,
                not args.no_mass_adaptation,
            )
            for mode in (
                {"nlp": ("mpc",), "rti": ("rti",), "both": ("mpc", "rti")}[
                    args.centroidal_solver
                ]
            )
        }
    save_comparison(results, args.output / args.model, args.model)
    print(
        json.dumps(
            {name: value[0] for name, value in results.items()},
            indent=2,
            ensure_ascii=False,
        )
    )
    if any(
        not results[name][0]["passed"] for name in ("mpc", "rti") if name in results
    ):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
