"""Reproducible test scenes generated without consulting planner outcomes."""

from __future__ import annotations

import itertools
from dataclasses import asdict

import numpy as np
from collision_checker import CollisionChecker
from lab4_common import JOINT_LOWER, JOINT_UPPER, Q_HOME, ObstacleSpec
from motion_planning import PlannerConfig, SceneChecker


class RectangleWorld:
    """2D point robot with exact segment/AABB intersection, including interiors."""

    def __init__(self, rectangles):
        self.rectangles = np.asarray(rectangles, dtype=float).reshape(-1, 2, 2)

    def is_collision_free(self, q):
        return not any(np.all((lo <= q) & (q <= hi)) for lo, hi in self.rectangles)

    def is_path_free(self, a, b):
        for lower, upper in self.rectangles:
            enter, leave = 0.0, 1.0
            for k, delta in enumerate(b - a):
                if abs(delta) < 1e-14:
                    if a[k] < lower[k] or a[k] > upper[k]:
                        enter, leave = 1.0, 0.0
                        break
                else:
                    t = sorted(((lower[k] - a[k]) / delta, (upper[k] - a[k]) / delta))
                    enter, leave = max(enter, t[0]), min(leave, t[1])
            if enter <= leave:
                return False
        return True

    def minimum_clearance(self, path):
        """Dense measurement only; exact segment intersection determines safety."""
        distances = []
        for a, b in itertools.pairwise(path):
            q = np.linspace(
                a, b, max(2, int(np.ceil(np.linalg.norm(b - a) / 0.005)) + 1)
            )
            for lo, hi in self.rectangles:
                distances.extend(
                    np.linalg.norm(np.maximum(np.maximum(lo - q, q - hi), 0), axis=1)
                )
        return float(min(distances)) if distances else None


def planar_scenes():
    """Known feasible witnesses and a separating-wall impossibility certificate."""
    return [
        {
            "name": "free",
            "start": [-0.8, 0],
            "goal": [0.8, 0],
            "rectangles": [],
            "witness": [[-0.8, 0], [0.8, 0]],
        },
        {
            "name": "wide_detour",
            "start": [-0.8, 0],
            "goal": [0.8, 0],
            "rectangles": [[[-0.25, -0.8], [0.25, 0.8]]],
            "witness": [[-0.8, 0], [-0.5, 0.9], [0.5, 0.9], [0.8, 0]],
        },
        {
            "name": "narrow_gap",
            "start": [-0.8, 0],
            "goal": [0.8, 0],
            "rectangles": [[[-0.15, -1], [0.15, 0.55]], [[-0.15, 0.69], [0.15, 1]]],
            "witness": [[-0.8, 0], [-0.4, 0.62], [0.4, 0.62], [0.8, 0]],
        },
        {
            "name": "offset_gates",
            "start": [-0.9, -0.6],
            "goal": [0.9, 0.6],
            "rectangles": [[[-0.5, -1], [-0.35, 0.25]], [[0.35, -0.25], [0.5, 1]]],
            "witness": [
                [-0.9, -0.6],
                [-0.7, 0.5],
                [0, 0.5],
                [0, -0.5],
                [0.7, -0.5],
                [0.9, 0.6],
            ],
        },
        {
            "name": "separating_wall",
            "start": [-0.8, 0],
            "goal": [0.8, 0],
            "rectangles": [[[-0.15, -1], [0.15, 1]]],
            "witness": None,
            "impossibility": "A closed wall joins both y boundaries of the [-1,1]^2 domain.",
        },
        {
            "name": "invalid_goal",
            "start": [-0.8, 0],
            "goal": [0, 0],
            "rectangles": [[[-0.2, -0.2], [0.2, 0.2]]],
            "witness": None,
        },
    ]


def generate_robot_scene(seed: int) -> dict:
    """Choose valid endpoints and a truly blocked interpolation, not an easy RRT seed.

    The generator only uses geometry. It never runs RRT or retries a planner failure.
    Feasibility of a detour is deliberately unknown until the benchmark runs.
    """
    rng = np.random.default_rng(seed)
    include_table = seed % 2 == 0
    empty = CollisionChecker(obstacle_specs=[], include_table=include_table)
    config = PlannerConfig()
    for attempt in range(200):
        q = Q_HOME + rng.normal(size=(2, 6)) * np.array([1.0, 0.6, 0.6, 0.4, 0.5, 0.6])
        q = np.clip(q, JOINT_LOWER + 0.05, JOINT_UPPER - 0.05)
        if not all(empty.is_collision_free(x) for x in q):
            continue
        if not empty.is_path_free(*q, resolution=0.01):
            continue  # isolate actual obstacle avoidance from an already invalid baseline
        middle = q[0] + rng.uniform(0.35, 0.65) * (q[1] - q[0])
        empty._forward(middle)
        center = empty.mj_data.xpos[empty.mj_model.body("wrist_3_link").id].copy()
        if center[2] < 0.2:
            continue
        specs = [
            ObstacleSpec(
                "obstacle_0",
                tuple(center),
                tuple(rng.uniform([0.025, 0.025, 0.04], [0.07, 0.07, 0.13])),
            )
        ]
        for i in (1, 2):
            pos = center + rng.uniform([-0.3, -0.3, -0.15], [0.3, 0.3, 0.15])
            pos[2] = max(pos[2], 0.15)
            specs.append(
                ObstacleSpec(
                    f"obstacle_{i}", tuple(pos), tuple(rng.uniform(0.025, 0.065, 3))
                )
            )
        base = CollisionChecker(obstacle_specs=specs, include_table=include_table)
        checked = SceneChecker(base, config)
        if not all(checked.is_collision_free(x) for x in q):
            continue
        if base.is_collision_free(middle):
            continue
        return {
            "name": f"robot_{seed}",
            "generator_seed": seed,
            "geometry_attempt": attempt,
            "include_table": include_table,
            "start": q[0].tolist(),
            "goal": q[1].tolist(),
            "obstacles": [asdict(o) for o in specs],
            "feasibility": "unknown",
            "control": "Endpoints valid; direct interpolation collides; removing boxes clears it.",
        }
    raise RuntimeError(f"Could not generate geometry for seed {seed}")


def main():
    """Regenerate all geometry fixtures without selecting successful planner trials."""
    import argparse
    import json
    from pathlib import Path

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("/tmp/rrt-scenes"))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for seed in range(101, 107):
        scene = generate_robot_scene(seed)
        path = args.output / f"robot_{seed}.json"
        path.write_text(json.dumps(scene, indent=2, allow_nan=False) + "\n")
        print(path)


if __name__ == "__main__":
    main()
