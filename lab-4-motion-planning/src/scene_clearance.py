"""실행 중의 collision 형상과 화면에 보이는 mesh를 함께 검사한다."""

from __future__ import annotations

import mujoco
import numpy as np


class ObstacleClearance:
    """충돌 mask와 무관하게 실제 자세의 robot/obstacle 최소 간격을 읽는다.

    Mesh 거리는 MuJoCo의 convex hull 기준이므로 오목한 형상에는 보수적이다.
    Bounding sphere로 먼 쌍만 제외하며 simulator state는 수정하지 않는다.
    """

    def __init__(self, model: mujoco.MjModel, obstacle_names: list[str]) -> None:
        self.model = model
        obstacle_bodies = {model.body(name).id for name in obstacle_names}
        environment = obstacle_bodies | {0}
        table = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "table")
        if table >= 0:
            environment.add(table)
        robot = [
            i for i in range(model.ngeom) if model.geom_bodyid[i] not in environment
        ]
        obstacles = [
            i for i in range(model.ngeom) if model.geom_bodyid[i] in obstacle_bodies
        ]
        self.pairs = np.asarray(
            [(i, j) for i in robot for j in obstacles], dtype=int
        ).reshape(-1, 2)
        self.radii = model.geom_rbound[self.pairs].sum(axis=1)

    def minimum(self, data: mujoco.MjData, limit: float = 1.0) -> float:
        """모든 표본의 접촉 상태를 유지한 채 limit 이내의 최소 거리를 구한다."""
        if not len(self.pairs):
            return float("inf")
        centers = data.geom_xpos[self.pairs]
        lower_bounds = (
            np.linalg.norm(centers[:, 0] - centers[:, 1], axis=1) - self.radii
        )
        distance = limit
        # 작은 하한부터 검사하면 정확한 거리 호출 수를 줄일 수 있다.
        for index in np.argsort(lower_bounds):
            if lower_bounds[index] >= distance:
                break
            i, j = self.pairs[index]
            distance = min(
                distance,
                float(mujoco.mj_geomDistance(self.model, data, i, j, limit, None)),
            )
        return distance

    def is_clear(self, data: mujoco.MjData, margin: float) -> bool:
        """Check a threshold without solving unnecessary far-away distance queries."""
        if not len(self.pairs):
            return True
        centers = data.geom_xpos[self.pairs]
        lower = np.linalg.norm(centers[:, 0] - centers[:, 1], axis=1) - self.radii
        for index in np.flatnonzero(lower < margin):
            i, j = self.pairs[index]
            if mujoco.mj_geomDistance(self.model, data, i, j, margin, None) < margin:
                return False
        return True
