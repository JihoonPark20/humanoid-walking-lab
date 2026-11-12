"""Independent dense audits of direct, planned, smoothed and timed robot paths."""

from __future__ import annotations

import itertools

import numpy as np
from collision_checker import CollisionChecker
from scene_clearance import ObstacleClearance

EDGE_RESOLUTION_RAD = 0.005


def edge_samples(path: list[np.ndarray] | np.ndarray) -> np.ndarray:
    """Sample every joint-space edge independently of the planner's resolution."""
    path = np.asarray(path, dtype=float)
    if path.ndim != 2 or not len(path) or not np.isfinite(path).all():
        raise ValueError("A nonempty finite waypoint matrix is required")
    values = []
    for a, b in itertools.pairwise(path):
        count = max(2, int(np.ceil(np.linalg.norm(b - a) / EDGE_RESOLUTION_RAD)) + 1)
        values.extend(np.linspace(a, b, count)[:-1])
    values.append(np.asarray(path[-1]))
    return np.asarray(values)


def audit_samples(
    checker: CollisionChecker, samples: np.ndarray, margin: float
) -> dict:
    """Check physical contacts, joint limits and visible meshes at every sample."""
    if (
        not len(samples)
        or not np.isfinite(samples).all()
        or not np.isfinite(margin)
        or margin < 0
    ):
        raise ValueError(
            "A nonempty finite trajectory and nonnegative margin are required"
        )
    geometry = ObstacleClearance(
        checker.mj_model, [o.name for o in checker.obstacle_specs if o.name != "table"]
    )
    minimum = float("inf")
    collisions, violations = 0, 0
    for q in samples:
        free = checker.is_collision_free(q)
        checker._forward(q)
        distance = geometry.minimum(checker.mj_data)
        minimum = min(minimum, distance)
        collisions += not free
        violations += distance < margin
    return {
        "passed": collisions == 0 and violations == 0,
        "samples": len(samples),
        "collision_or_joint_limit_samples": collisions,
        "clearance_violation_samples": violations,
        "minimum_geometry_clearance_m": minimum if np.isfinite(minimum) else None,
        "required_clearance_m": margin,
    }


def audit_direct_segments(checker: CollisionChecker, configs: list[np.ndarray]) -> dict:
    """Require a genuine obstacle-blocked interpolation and a removed-box control."""
    empty = CollisionChecker(obstacle_specs=[], include_table=True)
    reports = []
    for i, (a, b) in enumerate(itertools.pairwise(configs)):
        samples = edge_samples([a, b])
        direct = audit_samples(checker, samples, 0)
        without = sum(not empty.is_collision_free(q) for q in samples)
        blocked = (
            direct["collision_or_joint_limit_samples"] > 0
            and without == 0
            and direct["minimum_geometry_clearance_m"] < -1e-4
        )
        reports.append(
            {
                "segment": i,
                "direct": direct,
                "without_obstacles_collision_samples": without,
                "obstacle_blocks_direct_path": blocked,
            }
        )
    return {
        "passed": any(r["obstacle_blocks_direct_path"] for r in reports),
        "edge_resolution_rad": EDGE_RESOLUTION_RAD,
        "segments": reports,
    }
