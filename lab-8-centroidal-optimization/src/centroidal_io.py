"""Lab 7 접촉/계획 변환과 centroidal 실험 산출물."""

from __future__ import annotations

import importlib.metadata
import json
import platform
import sys
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "lab-7-walking-optimization" / "src"))
import itertools

from centroidal_model import (
    CentroidalConfig,
    CentroidalPlan,
    Contacts,
    validate_centroidal,
)
from lipm_optimizer import WalkingPlan, solve_walk
from walking_model import WalkConfig, build_schedule


def contacts_from_walk(plan: WalkingPlan, times: np.ndarray) -> Contacts:
    """모든 phase 경계를 포함하는 격자에 고정 발바닥 중심·접촉 mask를 만든다."""
    positions = []
    active = []
    phases = plan.schedule.phases
    for start, end in itertools.pairwise(times):
        if any(start + 1e-9 < p.end < end - 1e-9 for p in phases):
            raise ValueError(
                "적분 interval 안에서 contact가 바뀝니다. phase 경계를 격자에 넣으세요."
            )
        phase = next((p for p in phases if (start + end) / 2 < p.end), phases[-1])
        feet = np.c_[plan.feet[list(phase.feet)], np.zeros(2)]
        feet[:, 0] += plan.schedule.config.foot_offset_x
        positions.append(feet)
        mask = np.ones(2)
        if phase.swing >= 0:
            mask[phase.swing] = 0
        active.append(mask)
    return Contacts(np.asarray(times), np.asarray(positions), np.asarray(active))


def reference_from_walk(
    plan: WalkingPlan, times: np.ndarray, mass: float, height_amplitude: float = 0.0
) -> np.ndarray:
    """LIPM 수평 궤적에 높이 변화 과제를 더한다. 양 끝의 수직 속도는 0이다."""
    rows = []
    duration = plan.schedule.duration
    for t in times:
        value = plan.sample(float(t))
        phase = np.clip(t, 0, duration) / duration
        z = (
            plan.schedule.config.com_height
            + height_amplitude * np.sin(np.pi * phase) ** 2
        )
        vz = (
            height_amplitude * np.pi / duration * np.sin(2 * np.pi * phase)
            if 0 < t < duration
            else 0.0
        )
        rows.append(
            np.r_[value["com"], z, mass * np.r_[value["velocity"], vz], np.zeros(3)]
        )
    return np.asarray(rows)


def make_walking_problem(
    steps: int = 2,
    dt: float = 0.1,
    height_amplitude: float = 0.025,
    config: CentroidalConfig | None = None,
) -> tuple:
    """Lab 7의 feasible footstep 계획으로 W3 학습 문제를 만든다."""
    cfg = config or CentroidalConfig()
    walk = solve_walk(build_schedule(WalkConfig(steps=steps)), optimize_feet=True)
    schedule = build_schedule(replace(walk.schedule.config, dt=dt))
    contacts = contacts_from_walk(walk, schedule.times)
    reference = reference_from_walk(walk, schedule.times, cfg.mass, height_amplitude)
    return cfg, contacts, reference, walk


def save_centroidal(plan: CentroidalPlan, output: Path) -> dict:
    """상태·wrench·접촉·reference와 검증 결과를 저장한다."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output.mkdir(parents=True, exist_ok=True)
    report = {
        "schema": 1,
        "config": asdict(plan.config),
        "validation": validate_centroidal(plan),
        "solve_seconds": plan.solve_seconds,
        "iterations": plan.iterations,
        "status": plan.status,
        "objective": plan.objective,
        "terminal_stop": plan.terminal_stop,
        "environment": {
            "python": platform.python_version(),
            "platform": platform.platform(),
            "versions": {
                name: importlib.metadata.version(name)
                for name in ("numpy", "casadi", "osqp")
            },
        },
    }
    np.savez_compressed(
        output / "plan.npz",
        metadata=json.dumps(report),
        times=plan.contacts.times,
        positions=plan.contacts.positions,
        active=plan.contacts.active,
        states=plan.states,
        wrenches=plan.wrenches,
        reference=plan.reference,
        initial=plan.initial,
    )
    np.savetxt(
        output / "states.csv",
        np.c_[plan.contacts.times, plan.states, plan.reference],
        delimiter=",",
        comments="",
        header="t,cx,cy,cz,px,py,pz,Lx,Ly,Lz,ref_cx,ref_cy,ref_cz,ref_px,ref_py,ref_pz,ref_Lx,ref_Ly,ref_Lz",
    )
    np.savetxt(
        output / "wrenches.csv",
        np.c_[
            plan.contacts.times[:-1],
            plan.wrenches.reshape(-1, 12),
            plan.contacts.active,
        ],
        delimiter=",",
        comments="",
        header="t,lfx,lfy,lfz,ltx,lty,ltz,rfx,rfy,rfz,rtx,rty,rtz,left_active,right_active",
    )
    (output / "metrics.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    )
    fig, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    t = plan.contacts.times
    axes[0].plot(t, plan.states[:, :2])
    axes[0].set_ylabel("CoM xy (m)")
    axes[1].plot(t, plan.states[:, 2], label="optimized")
    axes[1].plot(t, plan.reference[:, 2], "--", label="reference")
    axes[1].set_ylabel("height (m)")
    axes[1].legend()
    axes[2].plot(t, plan.states[:, 6:])
    axes[2].set_ylabel("L (kg m²/s)")
    axes[3].step(t[:-1], plan.wrenches[:, :, 2], where="post")
    axes[3].set(ylabel="normal force (N)", xlabel="time (s)")
    for ax in axes:
        ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(output / "centroidal.png", dpi=130)
    plt.close(fig)
    return report


def load_centroidal(path: Path) -> CentroidalPlan:
    """pickle 없이 저장 계획을 읽고 독립 검증한다."""
    with np.load(path, allow_pickle=False) as data:
        meta = json.loads(str(data["metadata"]))
        if meta.get("schema", 1) != 1:
            raise ValueError("지원하지 않는 centroidal schema입니다.")
        contacts = Contacts(data["times"], data["positions"], data["active"])
        plan = CentroidalPlan(
            CentroidalConfig(**meta["config"]),
            contacts,
            data["states"],
            data["wrenches"],
            data["reference"],
            data["initial"],
            meta["terminal_stop"],
            meta["solve_seconds"],
            meta["iterations"],
            meta["status"],
            meta["objective"],
        )
    if not validate_centroidal(plan)["valid"]:
        raise ValueError("저장된 centroidal 계획 검증 실패")
    return plan
