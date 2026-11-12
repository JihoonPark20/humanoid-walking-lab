"""Configuration space에서 RRT와 RRT*로 충돌 없는 경로를 탐색한다.

Implements sampling-based motion planning with:
- Basic RRT: grow tree toward random samples
- RRT*: rewiring for asymptotically optimal paths
- Goal bias: configurable fraction of samples aimed at goal
- Edge collision checking via CollisionChecker.is_path_free"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import matplotlib

matplotlib.use("Agg")           # must be set before pyplot is imported
import matplotlib.pyplot as plt
import numpy as np

# 3-D projection — optional; broken when system mpl_toolkits conflicts with pip matplotlib
if TYPE_CHECKING:
    from mpl_toolkits.mplot3d import Axes3D

_3D_AVAILABLE = False
try:
    import mpl_toolkits.mplot3d  # noqa: F401 — registers '3d' projection
    _3D_AVAILABLE = True
except Exception:
    _3D_AVAILABLE = False

from collision_checker import CollisionChecker
from lab4_common import JOINT_LOWER, JOINT_UPPER


@dataclass
class RRTNode:
    """Node in the RRT search tree.

    Attributes:
        q: Joint configuration (6,).
        parent: Index of parent node in the tree (None for root).
        cost: Cumulative path cost from root (L2 in C-space).
    """

    q: np.ndarray
    parent: int | None = None
    cost: float = 0.0


class RRTStarPlanner:
    """RRT / RRT* planner for collision-free C-space paths.

    Args:
        collision_checker: Collision checker instance.
        joint_limits_lower: Lower joint limits (6,).
        joint_limits_upper: Upper joint limits (6,).
        step_size: Maximum extension distance per step (rad in C-space L2).
        goal_bias: Probability of sampling the goal configuration.
        rewire_radius: Radius for RRT* rewiring neighborhood.
        goal_tolerance: L2 distance to goal for success.
        local_bias: Optional Gaussian sampling probability near the endpoint segment.
        local_std: Gaussian standard deviation; global sampling remains nonzero.
    """

    def __init__(
        self,
        collision_checker: CollisionChecker,
        joint_limits_lower: np.ndarray | None = None,
        joint_limits_upper: np.ndarray | None = None,
        step_size: float = 0.3,
        goal_bias: float = 0.1,
        rewire_radius: float = 1.0,
        goal_tolerance: float = 0.15,
        local_bias: float = 0.0,
        local_std: float = 0.8,
    ) -> None:
        self.cc = collision_checker
        self.lower = np.asarray(joint_limits_lower if joint_limits_lower is not None else JOINT_LOWER, dtype=float).copy()
        self.upper = np.asarray(joint_limits_upper if joint_limits_upper is not None else JOINT_UPPER, dtype=float).copy()
        if self.lower.ndim != 1 or self.upper.shape != self.lower.shape or not np.isfinite([self.lower, self.upper]).all() or np.any(self.lower >= self.upper):
            raise ValueError("Finite, ordered joint limits with matching shapes are required")
        if not np.isfinite([step_size, goal_bias, rewire_radius, goal_tolerance]).all() or min(step_size, rewire_radius, goal_tolerance) <= 0 or not 0 <= goal_bias <= 1:
            raise ValueError("Invalid RRT step, radius, tolerance or goal bias")
        if not np.isfinite([local_bias, local_std]).all() or local_bias < 0 or local_std <= 0:
            raise ValueError("Invalid local sampling settings")
        if local_bias + goal_bias >= 1 and local_bias > 0:
            raise ValueError("Local bias must retain a positive global sampling probability")
        self.local_bias, self.local_std = local_bias, local_std
        self.step_size = step_size
        self.goal_bias = goal_bias
        self.rewire_radius = rewire_radius
        self.goal_tolerance = goal_tolerance
        self._tree: list[RRTNode] = []
        self._rng = np.random.default_rng()

    @property
    def tree(self) -> list[RRTNode]:
        """Access the search tree for visualization."""
        return self._tree

    def plan(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        max_iter: int = 5000,
        rrt_star: bool = True,
        seed: int | None = None,
    ) -> list[np.ndarray] | None:
        """Plan a collision-free path from q_start to q_goal.

        Args:
            q_start: Start configuration (6,).
            q_goal: Goal configuration (6,).
            max_iter: Maximum number of iterations.
            rrt_star: If True, use RRT* with rewiring. If False, basic RRT.
            seed: Random seed for reproducibility.

        Returns:
            List of waypoint configurations from start to goal, or None if
            no path found within max_iter.
        """
        if seed is not None:
            self._rng = np.random.default_rng(seed)

        self.cost_history = []
        self.iterations = 0

        self._tree = []
        q_start = np.asarray(q_start, dtype=float)
        q_goal = np.asarray(q_goal, dtype=float)
        if not isinstance(max_iter, (int, np.integer)) or max_iter <= 0:
            raise ValueError("max_iter must be a positive integer")
        for q in (q_start, q_goal):
            if q.shape != self.lower.shape or not np.isfinite(q).all():
                raise ValueError("Endpoint shape/values do not match the joint space")
            if np.any(q < self.lower) or np.any(q > self.upper):
                return None
        # Validate start and goal
        if not self.cc.is_collision_free(q_start):
            return None
        if not self.cc.is_collision_free(q_goal):
            return None

        # Initialize tree with start node
        self._tree = [RRTNode(q=q_start.copy(), parent=None, cost=0.0)]
        if np.array_equal(q_start, q_goal):
            self.cost_history.append(0.0)
            return [q_start.copy()]
        goal_node_idx: int | None = None

        for _iteration in range(max_iter):
            self.iterations = _iteration + 1
            # Sample random configuration (with goal bias)
            q_rand = self._sample(q_goal)

            # Find nearest node in tree
            nearest_idx = self._nearest(q_rand)
            q_nearest = self._tree[nearest_idx].q

            # Steer toward sample
            q_new = self._steer(q_nearest, q_rand)

            # Check edge collision
            if not self.cc.is_path_free(q_nearest, q_new):
                if goal_node_idx is not None:
                    self.cost_history.append(self._tree[goal_node_idx].cost)
                continue

            if rrt_star:
                # RRT*: find best parent in neighborhood
                new_idx = self._extend_rrt_star(q_new, nearest_idx)
            else:
                # Basic RRT: just add with nearest as parent
                cost = self._tree[nearest_idx].cost + self._dist(q_nearest, q_new)
                new_node = RRTNode(q=q_new, parent=nearest_idx, cost=cost)
                self._tree.append(new_node)
                new_idx = len(self._tree) - 1

            # Check if we reached the goal
            if self._dist(q_new, q_goal) < self.goal_tolerance:
                if self.cc.is_path_free(q_new, q_goal):
                    cost_to_goal = (
                        self._tree[new_idx].cost + self._dist(q_new, q_goal)
                    )
                    # Only update if better path found
                    if goal_node_idx is None or cost_to_goal < self._tree[goal_node_idx].cost:
                        if goal_node_idx is None:
                            goal_node = RRTNode(
                                q=q_goal.copy(),
                                parent=new_idx,
                                cost=cost_to_goal,
                            )
                            self._tree.append(goal_node)
                            goal_node_idx = len(self._tree) - 1
                        else:
                            self._tree[goal_node_idx].parent = new_idx
                            self._propagate_cost(goal_node_idx, cost_to_goal)

                    # For basic RRT, return immediately
                    if not rrt_star:
                        self.cost_history.append(self._tree[goal_node_idx].cost)
                        return self._extract_path(goal_node_idx)

            if goal_node_idx is not None:
                self.cost_history.append(self._tree[goal_node_idx].cost)

        if goal_node_idx is not None:
            return self._extract_path(goal_node_idx)
        return None

    def _sample(self, q_goal: np.ndarray) -> np.ndarray:
        """Sample a random configuration with goal bias."""
        draw = self._rng.random()
        if draw < self.goal_bias:
            return q_goal.copy()
        if draw < self.goal_bias + self.local_bias:
            alpha = self._rng.random()
            center = (1 - alpha) * self._tree[0].q + alpha * q_goal
            return np.clip(center + self._rng.normal(0, self.local_std, len(q_goal)), self.lower, self.upper)
        return self._rng.uniform(self.lower, self.upper)

    def _nearest(self, q: np.ndarray) -> int:
        """Find index of nearest node in tree to q."""
        delta = np.asarray([node.q for node in self._tree]) - q
        return int(np.argmin(np.einsum("ij,ij->i", delta, delta)))

    def _steer(self, q_from: np.ndarray, q_to: np.ndarray) -> np.ndarray:
        """Steer from q_from toward q_to, limited by step_size."""
        diff = q_to - q_from
        dist = np.linalg.norm(diff)
        if dist <= self.step_size:
            return q_to.copy()
        return q_from + (diff / dist) * self.step_size

    def _dist(self, q1: np.ndarray, q2: np.ndarray) -> float:
        """L2 distance in configuration space."""
        return float(np.linalg.norm(q1 - q2))

    def _near(self, q: np.ndarray) -> list[int]:
        """Find all nodes within rewire_radius of q."""
        delta = np.asarray([node.q for node in self._tree]) - q
        squared = np.einsum("ij,ij->i", delta, delta)
        return np.flatnonzero(squared <= self.rewire_radius**2).tolist()

    def _extend_rrt_star(self, q_new: np.ndarray, nearest_idx: int) -> int:
        """Add q_new to tree with RRT* best-parent selection and rewiring.

        Args:
            q_new: New configuration to add.
            nearest_idx: Index of nearest node (default parent).

        Returns:
            Index of the new node in the tree.
        """
        # Find neighborhood
        near_indices = self._near(q_new)

        # Choose best parent from neighborhood
        best_parent = nearest_idx
        best_cost = self._tree[nearest_idx].cost + self._dist(
            self._tree[nearest_idx].q, q_new
        )

        for idx in near_indices:
            candidate_cost = self._tree[idx].cost + self._dist(
                self._tree[idx].q, q_new
            )
            if candidate_cost < best_cost:
                if self.cc.is_path_free(self._tree[idx].q, q_new):
                    best_parent = idx
                    best_cost = candidate_cost

        # Add new node
        new_node = RRTNode(q=q_new, parent=best_parent, cost=best_cost)
        self._tree.append(new_node)
        new_idx = len(self._tree) - 1

        # Rewire: check if new node provides shorter path to neighbors
        for idx in near_indices:
            if idx == best_parent:
                continue
            new_cost = best_cost + self._dist(q_new, self._tree[idx].q)
            if new_cost < self._tree[idx].cost:
                if self.cc.is_path_free(q_new, self._tree[idx].q):
                    self._tree[idx].parent = new_idx
                    self._propagate_cost(idx, new_cost)

        return new_idx

    def _propagate_cost(self, idx: int, new_cost: float) -> None:
        """Propagate updated cost to all descendants of node idx."""
        pending = [(idx, new_cost)]
        while pending:
            parent, cost = pending.pop()
            self._tree[parent].cost = cost
            for i, node in enumerate(self._tree):
                if node.parent == parent:
                    pending.append((i, cost + self._dist(self._tree[parent].q, node.q)))

    def _extract_path(self, goal_idx: int) -> list[np.ndarray]:
        """Extract path from root to goal by backtracking through parents."""
        path = []
        idx: int | None = goal_idx
        while idx is not None:
            path.append(self._tree[idx].q.copy())
            idx = self._tree[idx].parent
        path.reverse()
        return path


def visualize_plan(
    planner: RRTStarPlanner,
    path: list[np.ndarray],
    ee_fid: int,
    title: str = "RRT* Plan",
    save_path: Path | None = None,
) -> None:
    """Visualize the RRT tree and planned path in 3D task space.

    Projects tree nodes and path waypoints to EE positions using FK,
    then plots them in 3D alongside obstacle boxes.

    Args:
        planner: Planner with populated tree.
        path: Planned path (list of configurations).
        ee_fid: Pinocchio EE frame ID.
        title: Plot title.
        save_path: If provided, save figure to this path.

    Raises:
        RuntimeError: If 3D visualization is unavailable (broken system mpl_toolkits).
    """
    if not _3D_AVAILABLE:
        raise RuntimeError(
            "3D visualization unavailable: mpl_toolkits.mplot3d could not be imported "
            "(likely a conflict between the system and pip matplotlib versions). "
            "Skipping visualization."
        )

    from lab4_common import TABLE_SPEC, get_ee_pos

    model = planner.cc.model
    data = planner.cc.data

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Plot tree edges (subsample for large trees)
    tree = planner.tree
    max_edges = 2000
    step = max(1, len(tree) // max_edges)
    for i in range(0, len(tree), step):
        node = tree[i]
        if node.parent is not None:
            p1 = get_ee_pos(model, data, ee_fid, tree[node.parent].q)
            p2 = get_ee_pos(model, data, ee_fid, node.q)
            ax.plot(
                [p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]],
                color="lightblue", alpha=0.3, linewidth=0.5,
            )

    # Plot path
    if path:
        path_pos = np.array([get_ee_pos(model, data, ee_fid, q) for q in path])
        ax.plot(
            path_pos[:, 0], path_pos[:, 1], path_pos[:, 2],
            "g-o", linewidth=2.5, markersize=5, label="Path",
        )
        ax.scatter(*path_pos[0], c="blue", s=100, marker="^", label="Start")
        ax.scatter(*path_pos[-1], c="red", s=100, marker="*", label="Goal")

    # Plot obstacle boxes
    for obs in tuple(planner.cc.obstacle_specs) + (TABLE_SPEC,):
        _draw_box(ax, obs.position, obs.half_extents, color="red" if obs.name != "table" else "brown")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(title)
    ax.legend()
    fig.tight_layout()

    if save_path is not None:
        save_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, dpi=150)
    plt.close(fig)


def _draw_box(
    ax: Axes3D,
    center: np.ndarray,
    half: np.ndarray,
    color: str = "red",
) -> None:
    """Draw a wireframe box on a 3D axes."""
    c, h = center, half
    # 8 corners
    corners = np.array([
        [c[0] - h[0], c[1] - h[1], c[2] - h[2]],
        [c[0] + h[0], c[1] - h[1], c[2] - h[2]],
        [c[0] + h[0], c[1] + h[1], c[2] - h[2]],
        [c[0] - h[0], c[1] + h[1], c[2] - h[2]],
        [c[0] - h[0], c[1] - h[1], c[2] + h[2]],
        [c[0] + h[0], c[1] - h[1], c[2] + h[2]],
        [c[0] + h[0], c[1] + h[1], c[2] + h[2]],
        [c[0] - h[0], c[1] + h[1], c[2] + h[2]],
    ])
    # 12 edges
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),  # bottom
        (4, 5), (5, 6), (6, 7), (7, 4),  # top
        (0, 4), (1, 5), (2, 6), (3, 7),  # verticals
    ]
    for i, j in edges:
        ax.plot(
            [corners[i, 0], corners[j, 0]],
            [corners[i, 1], corners[j, 1]],
            [corners[i, 2], corners[j, 2]],
            color=color, alpha=0.6, linewidth=1.0,
        )
