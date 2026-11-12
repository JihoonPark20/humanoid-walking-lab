"""Evaluate RRT and RRT* across fixed scenes/seeds, retaining every unsuccessful query."""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np
from motion_planning import PlannerConfig, plan_motion, read_scene, scene_checker
from planning_scenarios import RectangleWorld, planar_scenes
from planning_validation import audit_samples
from rrt_planner import RRTStarPlanner
from trajectory_executor import execute_trajectory
from trajectory_smoother import shortcut_path

LAB = Path(__file__).resolve().parents[1]
SEEDS = (0, 7, 23, 42, 91)
ALGORITHMS = ("rrt", "rrt_star")


def planar_trial(scene, config, seed, algorithm):
    world = RectangleWorld(scene["rectangles"])
    a, b = np.asarray(scene["start"]), np.asarray(scene["goal"])
    witness = scene["witness"]
    if witness is not None:
        assert all(
            world.is_path_free(np.asarray(x), np.asarray(y))
            for x, y in itertools.pairwise(witness)
        )
    planner = RRTStarPlanner(
        world,
        -np.ones(2),
        np.ones(2),
        step_size=config.step_size,
        goal_bias=config.goal_bias,
        rewire_radius=config.rewire_radius,
        goal_tolerance=config.goal_tolerance,
        local_bias=config.local_bias,
        local_std=config.local_std,
    )
    start = time.perf_counter()
    path = planner.plan(
        a, b, max_iter=config.max_iter, rrt_star=algorithm == "rrt_star", seed=seed
    )
    elapsed = time.perf_counter() - start
    status = (
        "success"
        if path is not None
        else "invalid_goal"
        if not world.is_collision_free(b)
        else "budget_exhausted"
    )
    report = {
        "scene": scene["name"],
        "domain": "planar",
        "algorithm": algorithm,
        "seed": seed,
        "known_feasible": witness is not None,
        "status": status,
        "success": path is not None,
        "search_seconds": elapsed,
        "iterations": planner.iterations,
        "nodes": len(planner.tree),
        "direct_blocked": not world.is_path_free(a, b),
        "safety_passed": True,
    }
    if path is not None:
        short = shortcut_path(path, world, seed=seed)
        safe = all(
            world.is_path_free(x, y)
            for p in (path, short)
            for x, y in itertools.pairwise(p)
        )
        safe = safe and np.allclose(short[0], a) and np.allclose(short[-1], b)
        safe = safe and all(np.all(np.abs(q) <= 1) for q in short)
        report.update(
            safety_passed=bool(safe),
            path=[q.tolist() for q in short],
            path_length=float(
                sum(np.linalg.norm(y - x) for x, y in itertools.pairwise(short))
            ),
            minimum_clearance=world.minimum_clearance(short),
        )
    return report


def summarize(reports):
    """Keep availability separate from safe handling of negative controls."""
    groups = []
    for domain in ("planar", "robot"):
        for algorithm in ALGORITHMS:
            rows = [
                r
                for r in reports
                if r["domain"] == domain and r["algorithm"] == algorithm
            ]
            candidates = [r for r in rows if r.get("known_feasible", True)]
            successes = [r for r in candidates if r["success"]]
            groups.append(
                {
                    "domain": domain,
                    "algorithm": algorithm,
                    "queries": len(rows),
                    "candidate_queries": len(candidates),
                    "solutions": len(successes),
                    "solution_rate": len(successes) / len(candidates),
                    "median_search_seconds": float(
                        np.median([r.get("search_seconds", 0) for r in rows])
                    ),
                    "p95_search_seconds": float(
                        np.percentile([r.get("search_seconds", 0) for r in rows], 95)
                    ),
                    "budget_exhausted": sum(
                        r["status"] == "budget_exhausted" for r in candidates
                    ),
                    "audit_rejected": sum(
                        r["status"] == "audit_rejected" for r in candidates
                    ),
                }
            )
    return groups


def run(output: Path, execute: bool = True):
    config = PlannerConfig()
    scenes = planar_scenes()
    robot_paths = sorted((LAB / "scenarios").glob("robot_*.json"))
    if len(robot_paths) != 6:
        raise ValueError("Expected the six fixed geometry-generated robot scenes")
    reports = []
    controls = []
    started = time.perf_counter()
    for scene in scenes:
        for algorithm, seed in itertools.product(ALGORITHMS, SEEDS):
            result = planar_trial(scene, config, seed, algorithm)
            reports.append(result)
            print(f"{scene['name']}/{algorithm}/{seed}: {result['status']}", flush=True)
    for path in robot_paths:
        scene = read_scene(path)
        base = scene_checker(scene)
        # Evaluate the controls independently of planner resolution and RNG.
        direct = np.linspace(scene["start"], scene["goal"], 1001)
        empty = scene_checker({**scene, "obstacles": []})
        control = {
            "scene": scene["name"],
            "direct_collision_samples": sum(
                not base.is_collision_free(q) for q in direct
            ),
            "removed_obstacle_collision_samples": sum(
                not empty.is_collision_free(q) for q in direct
            ),
            "endpoints_valid": all(
                base.is_collision_free(q) for q in (direct[0], direct[-1])
            ),
        }
        controls.append(control)
        for algorithm, seed in itertools.product(ALGORITHMS, SEEDS):
            result, trajectory = plan_motion(
                scene,
                seed=seed,
                rrt_star=algorithm == "rrt_star",
                config=config,
                checker=base,
            )
            result.update(scene=scene["name"], domain="robot", safety_passed=True)
            # The public API must not expose a trajectory after any failed audit.
            if not result["success"] and trajectory is not None:
                result["safety_passed"] = False
            if result["success"]:
                result["safety_passed"] = all(
                    a["passed"] for a in result["audits"].values()
                )
            # Preselected representative: RRT*, seed 0, every robot scene. No best-seed selection.
            if execute and algorithm == "rrt_star" and seed == 0:
                if trajectory is None:
                    result["execution"] = {"status": "not_run_no_plan"}
                else:
                    physical = execute_trajectory(
                        *trajectory,
                        obstacle_specs=base.obstacle_specs,
                        include_table=scene["include_table"],
                    )
                    audit = audit_samples(
                        base, physical["q_actual"], config.clearance_m
                    )
                    rms = float(
                        np.sqrt(
                            np.mean((physical["q_actual"] - physical["q_desired"]) ** 2)
                        )
                    )
                    audit.update(
                        joint_rms_rad=rms,
                        max_contact_count=int(physical["contact_counts"].max()),
                        sample_interval_s=0.001,
                        duration_s=float(physical["time"][-1]),
                    )
                    audit["passed"] = (
                        audit["passed"]
                        and rms < 0.05
                        and audit["max_contact_count"] == 0
                    )
                    result["execution"] = audit
            reports.append(result)
            print(
                f"{scene['name']}/{algorithm}/{seed}: {result['status']}, {result.get('search_seconds', 0):.2f}s",
                flush=True,
            )
    groups = summarize(reports)
    checks = {
        "coverage": len(reports)
        == (len(scenes) + len(robot_paths)) * len(SEEDS) * len(ALGORITHMS),
        "safe_outputs": all(r["safety_passed"] for r in reports),
        "negative_controls": all(
            not r["success"]
            for r in reports
            if r["scene"] in ("separating_wall", "invalid_goal")
        ),
        "blocked_robot_controls": all(
            c["endpoints_valid"]
            and c["direct_collision_samples"] > 0
            and c["removed_obstacle_collision_samples"] == 0
            for c in controls
        ),
        "planar_known_feasible_rate": all(
            g["solution_rate"] >= 0.9 for g in groups if g["domain"] == "planar"
        ),
        "executed_paths": all(
            r.get("execution", {}).get("passed", True) for r in reports
        ),
    }
    result = {
        "schema_version": 1,
        "passed": all(checks.values()),
        "checks": checks,
        "meaning": "Benchmark coverage/safety and known-feasible planar availability; not all queries solved.",
        "config": asdict(config),
        "seeds": list(SEEDS),
        "algorithms": list(ALGORITHMS),
        "scene_sha256": {
            p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in robot_paths
        },
        "planar_scenes": scenes,
        "groups": groups,
        "robot_controls": controls,
        "reports": reports,
        "execution_selection": "RRT*, seed 0, all six robot scenes"
        if execute
        else "not requested",
        "elapsed_seconds": time.perf_counter() - started,
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "matrix.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n"
    )
    print(
        json.dumps(
            {"passed": result["passed"], "checks": checks, "groups": groups}, indent=2
        )
    )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=LAB / "media/planning_benchmark")
    parser.add_argument(
        "--no-execute",
        action="store_true",
        help="Plan-only run; omit physical tracking checks",
    )
    args = parser.parse_args()
    if not run(args.output, execute=not args.no_execute)["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
