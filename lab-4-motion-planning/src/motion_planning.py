"""Plan an arbitrary static box scene using full joint limits and independent audits.

No task-specific via points, local sampling box or successful-seed retries are used.
"""

from __future__ import annotations

import argparse
import itertools
import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
from collision_checker import CollisionChecker
from lab4_common import ACC_LIMITS, JOINT_LOWER, JOINT_UPPER, VEL_LIMITS, ObstacleSpec
from planning_validation import audit_samples, edge_samples
from rrt_planner import RRTStarPlanner
from scene_clearance import ObstacleClearance
from trajectory_smoother import _parameterize_quintic, shortcut_path


@dataclass(frozen=True)
class PlannerConfig:
    """One configuration shared by all benchmark scenes and seeds."""

    max_iter: int = 1500
    step_size: float = 0.3
    goal_bias: float = 0.2
    local_bias: float = 0.6
    local_std: float = 0.8
    rewire_radius: float = 0.7
    goal_tolerance: float = 0.15
    clearance_m: float = 0.03
    planning_buffer_m: float = 0.002
    edge_resolution_rad: float = 0.02
    reference_dt_s: float = 0.01
    velocity_scale: float = 0.15
    acceleration_scale: float = 0.1

    def __post_init__(self):
        positive = (
            self.step_size,
            self.rewire_radius,
            self.goal_tolerance,
            self.edge_resolution_rad,
            self.reference_dt_s,
            self.velocity_scale,
            self.acceleration_scale,
            self.local_std,
        )
        if not all(np.isfinite(x) and x > 0 for x in positive):
            raise ValueError("Positive finite planner/timing settings are required")
        if not isinstance(self.max_iter, int) or self.max_iter < 1:
            raise ValueError("max_iter must be a positive integer")
        if (
            not np.isfinite(self.local_bias)
            or self.local_bias < 0
            or self.local_bias + self.goal_bias >= 1
        ):
            raise ValueError("Retain positive global sampling probability")
        if not np.isfinite(self.goal_bias) or not 0 <= self.goal_bias <= 1:
            raise ValueError("goal_bias must be between zero and one")
        if not all(
            np.isfinite(x) and x >= 0
            for x in (self.clearance_m, self.planning_buffer_m)
        ):
            raise ValueError("Clearance and buffer must be finite and nonnegative")


class SceneChecker:
    """Use one scene for endpoint/edge checks; never guess clearance from endpoints."""

    def __init__(self, base: CollisionChecker, config: PlannerConfig):
        self.base, self.config = base, config
        self.cache = {}
        self.meshes = ObstacleClearance(
            base.mj_model, [o.name for o in base.obstacle_specs]
        )

    def is_collision_free(self, q):
        key = np.asarray(q, dtype=float).tobytes()
        if key not in self.cache:
            free = self.base.is_collision_free(q)
            if free:
                free = self.meshes.is_clear(
                    self.base.mj_data,
                    self.config.clearance_m + self.config.planning_buffer_m,
                )
            if len(self.cache) > 50000:
                self.cache.clear()
            self.cache[key] = free
        return self.cache[key]

    def is_path_free(self, a, b):
        count = max(
            2, int(np.ceil(np.linalg.norm(b - a) / self.config.edge_resolution_rad)) + 1
        )
        return all(self.is_collision_free(q) for q in np.linspace(a, b, count))


def read_scene(path: Path) -> dict:
    """Read a six-joint start/goal and axis-aligned box environment."""
    scene = json.loads(path.read_text())
    for key in ("start", "goal"):
        q = np.asarray(scene[key], dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError(f"{key} must contain six finite joint angles")
    if not isinstance(scene.get("include_table", True), bool):
        raise TypeError("include_table must be boolean")
    names = set()
    for item in scene["obstacles"]:
        if not item["name"].startswith("obstacle_") or item["name"] in names:
            raise ValueError("Use unique obstacle_ prefixed names")
        names.add(item["name"])
        for key in ("position", "half_extents"):
            a = np.asarray(item[key], dtype=float)
            if a.shape != (3,) or not np.isfinite(a).all():
                raise ValueError(f"{key} must contain three finite values")
            if key == "half_extents" and np.any(a <= 0):
                raise ValueError("Box half extents must be positive")
    return scene


def scene_checker(scene: dict) -> CollisionChecker:
    """Build the robot, ground, optional table and supplied obstacle boxes."""
    obstacles = [
        ObstacleSpec(o["name"], tuple(o["position"]), tuple(o["half_extents"]))
        for o in scene["obstacles"]
    ]
    return CollisionChecker(
        obstacle_specs=obstacles, include_table=scene.get("include_table", True)
    )


def plan_motion(
    scene: dict,
    *,
    seed: int = 0,
    rrt_star: bool = True,
    config: PlannerConfig | None = None,
    checker=None,
):
    """Return an audited trajectory or an explicit rejection, never an unsafe path.

    A depleted sampling budget means unknown feasibility, not proof of no path.
    The returned report records unsuccessful attempts as well as successful ones.
    """
    config = config or PlannerConfig()
    base = checker if checker is not None else scene_checker(scene)
    checked = SceneChecker(base, config)
    a, b = np.asarray(scene["start"]), np.asarray(scene["goal"])
    report = {
        "seed": seed,
        "algorithm": "rrt_star" if rrt_star else "rrt",
        "config": asdict(config),
        "status": "pending",
        "success": False,
    }
    lower = np.maximum(JOINT_LOWER, base.mj_model.jnt_range[:6, 0])
    upper = np.minimum(JOINT_UPPER, base.mj_model.jnt_range[:6, 1])
    report["sampling_lower"], report["sampling_upper"] = lower.tolist(), upper.tolist()
    started = time.perf_counter()
    for name, q in [("start", a), ("goal", b)]:
        if not checked.is_collision_free(q):
            report.update(
                status=f"invalid_{name}", total_seconds=time.perf_counter() - started
            )
            return report, None
    planner = RRTStarPlanner(
        checked,
        lower,
        upper,
        step_size=config.step_size,
        goal_bias=config.goal_bias,
        rewire_radius=config.rewire_radius,
        goal_tolerance=config.goal_tolerance,
        local_bias=config.local_bias,
        local_std=config.local_std,
    )
    raw = planner.plan(a, b, max_iter=config.max_iter, rrt_star=rrt_star, seed=seed)
    report.update(
        search_seconds=time.perf_counter() - started,
        iterations=planner.iterations,
        nodes=len(planner.tree),
    )
    if raw is None:
        report.update(
            status="budget_exhausted", total_seconds=time.perf_counter() - started
        )
        return report, None
    short = shortcut_path(raw, checked, max_iter=150, seed=seed)
    # Geometry-preserving quintic timing avoids optional spline corner cutting.
    times, q, qd, qdd = _parameterize_quintic(
        short,
        config.velocity_scale * VEL_LIMITS,
        config.acceleration_scale * ACC_LIMITS,
        config.reference_dt_s,
    )
    audits = {
        "raw": audit_samples(base, edge_samples(raw), config.clearance_m),
        "shortcut": audit_samples(base, edge_samples(short), config.clearance_m),
        "timed": audit_samples(base, q, config.clearance_m),
    }
    limits_ok = bool(
        np.all(np.abs(qd) <= config.velocity_scale * VEL_LIMITS + 1e-6)
        and np.all(np.abs(qdd) <= config.acceleration_scale * ACC_LIMITS + 1e-6)
    )
    success = all(r["passed"] for r in audits.values()) and limits_ok
    report.update(
        status="success" if success else "audit_rejected",
        success=success,
        audits=audits,
        velocity_acceleration_limits=limits_ok,
        path_length_rad=float(
            sum(np.linalg.norm(y - x) for x, y in itertools.pairwise(short))
        ),
        raw_path=[x.tolist() for x in raw],
        path=[x.tolist() for x in short],
        trajectory_duration_s=float(times[-1]),
        total_seconds=time.perf_counter() - started,
    )
    return report, (times, q, qd) if success else None


def main():
    """Plan a user-supplied scene, optionally execute exactly that scene in MuJoCo."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", type=Path)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-iter", type=int, default=1500)
    parser.add_argument("--algorithm", choices=["rrt", "rrt_star"], default="rrt_star")
    parser.add_argument("--output", type=Path, default=Path("/tmp/rrt-plan"))
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    scene = read_scene(args.scene)
    report, trajectory = plan_motion(
        scene,
        seed=args.seed,
        rrt_star=args.algorithm == "rrt_star",
        config=PlannerConfig(max_iter=args.max_iter),
    )
    if args.execute and trajectory is not None:
        from trajectory_executor import execute_trajectory

        base = scene_checker(scene)
        result = execute_trajectory(
            *trajectory,
            obstacle_specs=base.obstacle_specs,
            include_table=scene.get("include_table", True),
        )
        physical = audit_samples(base, result["q_actual"], 0.03)
        rms = float(np.sqrt(np.mean((result["q_actual"] - result["q_desired"]) ** 2)))
        physical.update(
            joint_rms_rad=rms, max_contacts=int(result["contact_counts"].max())
        )
        physical["passed"] = (
            physical["passed"] and rms < 0.05 and physical["max_contacts"] == 0
        )
        report["execution"] = physical
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "result.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n"
    )
    if trajectory is not None:
        np.savez(
            args.output / "trajectory.npz",
            times=trajectory[0],
            q=trajectory[1],
            qd=trajectory[2],
        )
    else:
        (args.output / "trajectory.npz").unlink(missing_ok=True)
    print(
        json.dumps(
            {
                k: v
                for k, v in report.items()
                if k not in ("raw_path", "path", "audits")
            },
            indent=2,
        )
    )
    if not report["success"] or not report.get("execution", {"passed": True})["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
