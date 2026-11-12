"""Generalization is tested across geometry, rather than only successful demo seeds."""

from __future__ import annotations

import itertools
import json
from pathlib import Path

import motion_planning as motion
import numpy as np
import pytest
from collision_checker import CollisionChecker
from lab4_common import JOINT_LOWER, JOINT_UPPER, Q_HOME
from planning_scenarios import RectangleWorld, planar_scenes
from planning_validation import audit_samples, edge_samples
from rrt_planner import RRTStarPlanner

SCENES = Path(__file__).resolve().parents[1] / "scenarios"


def test_large_detour_needs_full_search_domain():
    scene = next(s for s in planar_scenes() if s["name"] == "wide_detour")
    world = RectangleWorld(scene["rectangles"])
    a, b = np.array(scene["start"]), np.array(scene["goal"])
    restricted = RRTStarPlanner(world, np.array([-1, -0.5]), np.array([1, 0.5]))
    assert restricted.plan(a, b, max_iter=500, rrt_star=False, seed=7) is None
    full = RRTStarPlanner(world, -np.ones(2), np.ones(2))
    path = full.plan(a, b, max_iter=1500, rrt_star=False, seed=7)
    assert path is not None
    assert max(abs(q[1]) for q in path) > 0.8
    assert all(world.is_path_free(a, b) for a, b in itertools.pairwise(path))


@pytest.mark.parametrize("scene", planar_scenes(), ids=lambda s: s["name"])
def test_analytic_scene_witnesses_and_negative_controls(scene):
    world = RectangleWorld(scene["rectangles"])
    if scene["witness"] is not None:
        w = np.asarray(scene["witness"])
        assert all(world.is_path_free(a, b) for a, b in itertools.pairwise(w))
    elif scene["name"] == "invalid_goal":
        assert not world.is_collision_free(scene["goal"])
    else:
        # Any continuous left-to-right path must cross this boundary-to-boundary wall.
        assert scene["rectangles"][0][0][1] == -1 and scene["rectangles"][0][1][1] == 1


@pytest.mark.parametrize("seed", range(101, 107))
def test_robot_geometry_has_real_blockage_with_clear_endpoints(seed):
    scene = motion.read_scene(SCENES / f"robot_{seed}.json")
    cc = motion.scene_checker(scene)
    empty = motion.scene_checker({**scene, "obstacles": []})
    points = np.linspace(scene["start"], scene["goal"], 501)
    assert all(cc.is_collision_free(q) for q in (points[0], points[-1]))
    assert any(not cc.is_collision_free(q) for q in points)
    assert all(empty.is_collision_free(q) for q in points)


def test_ground_is_checked_even_without_boxes_or_table():
    cc = CollisionChecker(obstacle_specs=[], include_table=False, self_collision=False)
    q = Q_HOME.copy()
    q[1] = 0.3
    q[2] = 0
    assert not cc.is_collision_free(q)
    floor = cc.mj_model.geom("floor").id
    assert any(floor in pair for pair in cc._environment_pairs)
    assert cc.is_collision_free(Q_HOME)


def test_arbitrary_scene_uses_full_model_limits_and_does_not_retry(monkeypatch):
    calls = []

    class NoSolution(RRTStarPlanner):
        def plan(self, a, b, **kwargs):
            calls.append((self.lower.copy(), self.upper.copy(), kwargs))
            self.iterations = kwargs["max_iter"]

    monkeypatch.setattr(motion, "RRTStarPlanner", NoSolution)
    scene = motion.read_scene(SCENES / "robot_101.json")
    base = motion.scene_checker(scene)
    report, path = motion.plan_motion(scene, checker=base)
    assert path is None and report["status"] == "budget_exhausted"
    assert len(calls) == 1
    np.testing.assert_allclose(
        calls[0][0], np.maximum(JOINT_LOWER, base.mj_model.jnt_range[:6, 0])
    )
    np.testing.assert_allclose(
        calls[0][1], np.minimum(JOINT_UPPER, base.mj_model.jnt_range[:6, 1])
    )


def test_broken_planner_cannot_return_colliding_executable_trajectory(monkeypatch):
    class Broken(RRTStarPlanner):
        def plan(self, a, b, **kwargs):
            self.iterations = 1
            return [a, b]

    monkeypatch.setattr(motion, "RRTStarPlanner", Broken)
    scene = motion.read_scene(SCENES / "robot_101.json")
    report, trajectory = motion.plan_motion(scene)
    assert not report["success"]
    assert report["status"] == "audit_rejected"
    assert trajectory is None
    assert not report["audits"]["raw"]["passed"]


def test_empty_obstacle_audit_serializes_strict_json():
    cc = CollisionChecker(obstacle_specs=[], include_table=False)
    result = audit_samples(cc, edge_samples([Q_HOME, Q_HOME]), 0.03)
    assert result["passed"]
    assert result["minimum_geometry_clearance_m"] is None
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize(
    "change",
    [
        {"max_iter": 0},
        {"edge_resolution_rad": 0},
        {"goal_bias": 1.1},
        {"clearance_m": -1},
    ],
)
def test_invalid_configuration_is_rejected(change):
    with pytest.raises(ValueError):
        motion.PlannerConfig(**change)


def test_scene_file_rejects_invalid_obstacles(tmp_path):
    scene = motion.read_scene(SCENES / "robot_101.json")
    scene["obstacles"][0]["half_extents"][0] = -1
    path = tmp_path / "scene.json"
    path.write_text(json.dumps(scene))
    with pytest.raises(ValueError):
        motion.read_scene(path)


def test_mixed_sampling_keeps_global_support():
    from rrt_planner import RRTNode

    planner = RRTStarPlanner(
        RectangleWorld([]),
        -np.ones(2) * 6,
        np.ones(2) * 6,
        goal_bias=0.2,
        local_bias=0.6,
        local_std=0.1,
    )
    planner._tree = [RRTNode(np.zeros(2))]
    planner._rng = np.random.default_rng(7)
    samples = np.array([planner._sample(np.ones(2) * 0.1) for _ in range(1000)])
    assert np.any(samples[:, 0] > 4) and np.any(samples[:, 0] < -4)
    with pytest.raises(ValueError):
        RRTStarPlanner(RectangleWorld([]), goal_bias=0.2, local_bias=0.8)


def test_threshold_clearance_matches_minimum_query():
    from scene_clearance import ObstacleClearance

    scene = motion.read_scene(SCENES / "robot_101.json")
    cc = motion.scene_checker(scene)
    geometry = ObstacleClearance(cc.mj_model, [o.name for o in cc.obstacle_specs])
    for q in np.linspace(scene["start"], scene["goal"], 51):
        cc._forward(q)
        assert geometry.is_clear(cc.mj_data, 0.032) == (
            geometry.minimum(cc.mj_data) >= 0.032
        )
