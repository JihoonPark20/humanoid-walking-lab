"""Adversarial checks: blocked interiors, removed-obstacle controls and invalid inputs."""

from __future__ import annotations

import itertools
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import trajectory_smoother as smoother
from capstone_demo import ClearanceAwareChecker
from collision_checker import CollisionChecker
from lab4_common import JOINT_LOWER, JOINT_UPPER, Q_HOME
from planning_validation import audit_direct_segments, audit_samples, edge_samples
from rrt_planner import RRTNode, RRTStarPlanner


class Rectangle:
    """Analytic segment/box intersection; independent of planner sampling."""

    def __init__(self, wall=False):
        self.lo = np.array([-0.12, -2 if wall else -0.4])
        self.hi = np.array([0.12, 2 if wall else 0.4])

    def is_collision_free(self, q):
        return not np.all((self.lo <= q) & (q <= self.hi))

    def is_path_free(self, a, b):
        lo, hi = 0.0, 1.0
        for i in range(2):
            delta = b[i] - a[i]
            if abs(delta) < 1e-12:
                if a[i] < self.lo[i] or a[i] > self.hi[i]:
                    return True
            else:
                t = sorted([(self.lo[i] - a[i]) / delta, (self.hi[i] - a[i]) / delta])
                lo, hi = max(lo, t[0]), min(hi, t[1])
                if lo > hi:
                    return True
        return False


@pytest.mark.parametrize("star", [False, True])
@pytest.mark.parametrize("seed", [0, 7, 42])
def test_blocked_edge_detour_and_tree_costs(star, seed):
    cc = Rectangle()
    a, b = np.array([-0.8, 0.0]), np.array([0.8, 0.0])
    assert cc.is_collision_free(a) and cc.is_collision_free(b)
    assert not cc.is_path_free(a, b)
    planner = RRTStarPlanner(cc, -np.ones(2), np.ones(2), rewire_radius=0.5)
    path = planner.plan(a, b, max_iter=450, rrt_star=star, seed=seed)
    assert path is not None
    short = smoother.shortcut_path(path, cc, seed=seed)
    assert len(short) > 2
    for x, y in itertools.pairwise(short):
        assert cc.is_path_free(x, y)
    for node in planner.tree[1:]:
        parent = planner.tree[node.parent]
        assert node.cost == pytest.approx(
            parent.cost + np.linalg.norm(node.q - parent.q)
        )
    if star:
        assert np.all(np.diff(planner.cost_history) <= 1e-10)


def test_impossible_wall_never_returns_a_path():
    planner = RRTStarPlanner(Rectangle(wall=True), -np.ones(2), np.ones(2))
    assert (
        planner.plan(np.array([-0.8, 0]), np.array([0.8, 0]), max_iter=500, seed=42)
        is None
    )


@pytest.fixture(scope="module")
def real_blocked_segment():
    # Task poses are recorded in the audited demonstration, not a synthetic free-space case.
    evidence = Path(__file__).parents[1] / "media/planning_audit.json"
    configs = np.asarray(json.loads(evidence.read_text())["configuration_endpoints"])
    return CollisionChecker(), configs[1:3]


def test_real_robot_direct_path_requires_obstacle_avoidance(real_blocked_segment):
    cc, configs = real_blocked_segment
    assert all(cc.is_collision_free(q) for q in configs)
    report = audit_direct_segments(cc, list(configs))
    assert report["passed"]
    segment = report["segments"][0]
    assert segment["direct"]["collision_or_joint_limit_samples"] > 0
    assert segment["direct"]["minimum_geometry_clearance_m"] < -0.01
    assert segment["without_obstacles_collision_samples"] == 0
    # Checking endpoints alone would incorrectly accept this trajectory.
    assert not audit_samples(cc, edge_samples(configs), 0.03)["passed"]
    assert not audit_direct_segments(
        CollisionChecker(obstacle_specs=[]), list(configs)
    )["passed"]


@pytest.mark.parametrize("seed", [7, 42])
def test_real_robot_detour_is_independently_clear(real_blocked_segment, seed):
    cc, (a, b) = real_blocked_segment
    lower = np.maximum(np.minimum(a, b) - 0.6, JOINT_LOWER)
    upper = np.minimum(np.maximum(a, b) + 0.6, JOINT_UPPER)
    guarded = ClearanceAwareChecker(cc)
    planner = RRTStarPlanner(guarded, lower, upper, step_size=0.2, goal_bias=0.2)
    path = planner.plan(a, b, max_iter=3000, rrt_star=False, seed=seed)
    assert path is not None
    short = smoother.shortcut_path(path, guarded, seed=seed)
    assert audit_samples(cc, edge_samples(path), 0.03)["passed"]
    assert audit_samples(cc, edge_samples(short), 0.03)["passed"]


def test_invalid_joint_limits_and_stale_tree():
    cc = CollisionChecker()
    invalid = Q_HOME.copy()
    invalid[2] = 4.0
    assert not cc.is_collision_free(invalid)
    assert not cc.is_collision_free(np.full(6, np.nan))
    planner = RRTStarPlanner(cc)
    assert len(planner.plan(Q_HOME, Q_HOME)) == 1
    assert planner.plan(Q_HOME, invalid) is None
    assert planner.tree == []
    with pytest.raises(ValueError):
        planner.plan(np.zeros(5), Q_HOME)
    with pytest.raises(ValueError):
        RRTStarPlanner(cc, step_size=0)
    with pytest.raises(ValueError):
        cc.is_path_free(Q_HOME, Q_HOME, resolution=0)


def test_deep_rewiring_does_not_overflow_recursion():
    planner = RRTStarPlanner(Rectangle(), -np.ones(2), np.ones(2))
    planner._tree = [RRTNode(np.zeros(2), None)] + [
        RRTNode(np.array([i, 0.0]), i - 1) for i in range(1, 1200)
    ]
    planner._propagate_cost(0, 0.0)
    assert planner.tree[-1].cost == 1199


@pytest.mark.parametrize("dt", [0, -1, np.nan])
def test_invalid_timing_is_rejected(dt):
    with pytest.raises(ValueError):
        smoother.parameterize_topp_ra(
            [np.zeros(2), np.ones(2)], np.ones(2), np.ones(2), dt
        )


def test_empty_and_invalid_limits_rejected():
    with pytest.raises(ValueError):
        smoother.parameterize_topp_ra([], np.ones(2), np.ones(2))
    with pytest.raises(ValueError):
        smoother.parameterize_topp_ra([np.zeros(2)], np.array([0.0, 1]), np.ones(2))
    with pytest.raises(ValueError):
        edge_samples([])


def test_toppra_exact_endpoint_even_with_fractional_final_tick(monkeypatch):
    class Trajectory:
        duration = 1.0002

        def __call__(self, t, derivative=0):
            return np.array([t, t]) if derivative == 0 else np.zeros(2)

    fake = SimpleNamespace(
        SplineInterpolator=lambda *a: None,
        algorithm=SimpleNamespace(
            TOPPRA=lambda *a, **kw: SimpleNamespace(
                compute_trajectory=lambda: Trajectory()
            )
        ),
    )
    monkeypatch.setattr(smoother, "_TOPPRA_AVAILABLE", True)
    monkeypatch.setattr(smoother, "ta", fake)
    monkeypatch.setattr(
        smoother,
        "constraint",
        SimpleNamespace(
            JointVelocityConstraint=lambda a: None,
            JointAccelerationConstraint=lambda a: None,
        ),
    )
    t, q, _, _ = smoother.parameterize_topp_ra(
        [np.zeros(2), np.full(2, 1.0002)], np.ones(2), np.ones(2), 0.001
    )
    assert t[-1] == 1.0002
    assert np.all(np.diff(t) > 0)
    np.testing.assert_allclose(q[-1], [1.0002, 1.0002])
