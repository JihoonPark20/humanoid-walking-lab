"""계획 저장·재생과 수치 비교 그림. NPZ에는 pickle 객체를 저장하지 않는다."""

from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path

import numpy as np
from lipm_optimizer import PlanningError, WalkingPlan, plan_cost, validate_plan
from walking_model import WalkConfig, build_schedule


def save_plan(plan: WalkingPlan, path: Path) -> None:
    """검증된 계획과 설정을 schema 1 NPZ로 저장한다."""
    if not validate_plan(plan)["valid"]:
        raise PlanningError("제약을 위반한 계획은 저장할 수 없습니다.")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    metadata = {
        "schema": 1,
        "config": asdict(plan.schedule.config),
        "optimize_feet": plan.optimize_feet,
        "solve_seconds": plan.solve_seconds,
        "iterations": plan.iterations,
        "refinement_rounds": plan.refinement_rounds,
    }
    np.savez_compressed(
        path,
        metadata=json.dumps(metadata),
        states=plan.states,
        jerks=plan.jerks,
        feet=plan.feet,
        initial_state=plan.initial_state,
        initial_feet=plan.schedule.feet[:2],
        landing_bounds=plan.landing_bounds,
    )


def load_plan(path: Path) -> WalkingPlan:
    """NPZ를 읽어 접촉 일정을 재구성하고, 실행 전에 다시 검증한다."""
    with np.load(path, allow_pickle=False) as data:
        meta = json.loads(str(data["metadata"]))
        if meta["schema"] != 1:
            raise ValueError("지원하지 않는 계획 schema입니다.")
        schedule = build_schedule(WalkConfig(**meta["config"]), data["initial_feet"])
        n, nf = len(schedule.times) - 1, len(schedule.feet)
        expected = {
            "states": (n + 1, 2, 3),
            "jerks": (n, 2),
            "feet": (nf, 2),
            "initial_state": (2, 3),
            "landing_bounds": (nf - 2, 2, 2),
        }
        arrays = {key: data[key].copy() for key in expected}
        if any(arrays[key].shape != shape for key, shape in expected.items()):
            raise ValueError("저장된 계획의 배열 크기가 설정과 일치하지 않습니다.")
        plan = WalkingPlan(
            schedule=schedule,
            **arrays,
            **{
                key: meta[key]
                for key in (
                    "optimize_feet",
                    "solve_seconds",
                    "iterations",
                    "refinement_rounds",
                )
            },
        )
    if not validate_plan(plan)["valid"]:
        raise PlanningError("저장된 계획이 Dynamics 또는 제약 검증에 실패했습니다.")
    return plan


def sampled_data(plan: WalkingPlan, dt: float = 0.01) -> np.ndarray:
    """CSV 열: t,cx,cy,vx,vy,ax,ay,zmpx,zmpy,dcmx,dcmy."""
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("샘플 간격은 유한한 양수여야 합니다.")
    times = np.linspace(
        0, plan.schedule.duration, int(np.ceil(plan.schedule.duration / dt)) + 1
    )
    return np.asarray(
        [
            [
                t,
                *value["com"],
                *value["velocity"],
                *value["acceleration"],
                *value["zmp"],
                *value["xi"],
            ]
            for t in times
            for value in [plan.sample(float(t))]
        ]
    )


def export_plan(plan: WalkingPlan, output: Path) -> dict:
    """재생용 NPZ, 분석 CSV, 검증 JSON, 비교 plot을 같은 폴더에 저장한다."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    save_plan(plan, output / "plan.npz")
    data = sampled_data(plan)
    np.savetxt(
        output / "trajectory.csv",
        data,
        delimiter=",",
        comments="",
        header="time_s,com_x_m,com_y_m,vx_m_s,vy_m_s,ax_m_s2,ay_m_s2,zmp_x_m,zmp_y_m,dcm_x_m,dcm_y_m",
    )
    np.savetxt(
        output / "footsteps.csv",
        np.c_[np.arange(len(plan.feet)), plan.schedule.feet, plan.feet],
        delimiter=",",
        comments="",
        header="node,nominal_x_m,nominal_y_m,planned_x_m,planned_y_m",
    )
    report = {
        "validation": validate_plan(plan),
        "cost": plan_cost(plan),
        "duration_s": plan.schedule.duration,
        "solve_seconds": plan.solve_seconds,
        "iterations": plan.iterations,
        "refinement_rounds": plan.refinement_rounds,
        "optimize_feet": plan.optimize_feet,
        "config": asdict(plan.schedule.config),
    }
    (output / "metrics.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    )
    cfg = plan.schedule.config
    fig, axes = plt.subplots(3, 1, figsize=(10, 10))
    axes[0].plot(data[:, 1], data[:, 2], label="CoM")
    axes[0].plot(data[:, 7], data[:, 8], label="ZMP", lw=1)
    axes[0].plot(data[:, 9], data[:, 10], label="DCM", lw=1, alpha=0.7)
    axes[0].scatter(
        plan.schedule.feet[:, 0] + cfg.foot_offset_x,
        plan.schedule.feet[:, 1],
        marker="x",
        color="grey",
        label="nominal feet",
    )
    for i, (x, y) in enumerate(plan.feet):
        axes[0].add_patch(
            Rectangle(
                (x + cfg.foot_offset_x - cfg.half_length, y - cfg.half_width),
                2 * cfg.half_length,
                2 * cfg.half_width,
                fill=False,
                color="C3",
                alpha=0.6,
            )
        )
        axes[0].annotate(str(i), (x + cfg.foot_offset_x, y), fontsize=8)
    axes[0].set(
        xlabel="x (m)", ylabel="y (m)", title="Offline LIPM plan — reduced model"
    )
    axes[0].axis("equal")
    axes[0].legend(ncol=4, fontsize=8)
    for axis in range(2):
        ax = axes[axis + 1]
        centers = []
        for t in data[:, 0]:
            k = min(
                np.searchsorted(plan.schedule.times, t, side="right") - 1,
                len(plan.jerks) - 1,
            )
            centers.append(
                plan.schedule.center(k, t - plan.schedule.times[k], plan.feet)[axis]
            )
        centers = np.asarray(centers)
        half = (cfg.half_length, cfg.half_width)[axis]
        ax.fill_between(
            data[:, 0],
            centers - half,
            centers + half,
            color="C2",
            alpha=0.2,
            label="allowed ZMP tube",
        )
        ax.plot(data[:, 0], data[:, 7 + axis], label="ZMP", color="C1")
        ax.plot(data[:, 0], data[:, 1 + axis], label="CoM", color="C0")
        ax.set(xlabel="time (s)", ylabel=("x", "y")[axis] + " (m)")
        ax.legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(output / "trajectory.png", dpi=140)
    plt.close(fig)
    return report


def animate_plan(plan: WalkingPlan, path: Path) -> None:
    """LIPM의 top view GIF를 만든다. G1의 물리 시뮬레이션 영상은 아니다."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    data = sampled_data(plan, dt=0.05)
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(data[:, 1], data[:, 2], color="C0", alpha=0.25)
    ax.scatter(
        plan.feet[:, 0] + plan.schedule.config.foot_offset_x,
        plan.feet[:, 1],
        marker="s",
        color="grey",
    )
    (com,) = ax.plot([], [], "o", label="CoM")
    (zmp,) = ax.plot([], [], "x", label="ZMP")
    clock = ax.text(0.02, 0.95, "", transform=ax.transAxes)
    ax.set(xlabel="x (m)", ylabel="y (m)", title="LIPM plan (not full-body simulation)")
    ax.axis("equal")
    ax.legend()
    ax.grid(alpha=0.2)

    def update(i: int) -> tuple:
        """한 프레임의 CoM/ZMP 표시를 갱신한다."""
        com.set_data([data[i, 1]], [data[i, 2]])
        zmp.set_data([data[i, 7]], [data[i, 8]])
        clock.set_text(f"t = {data[i, 0]:.2f} s")
        return com, zmp, clock

    animation = FuncAnimation(fig, update, frames=len(data), interval=50, blit=True)
    animation.save(path, writer=PillowWriter(fps=20))
    plt.close(fig)
