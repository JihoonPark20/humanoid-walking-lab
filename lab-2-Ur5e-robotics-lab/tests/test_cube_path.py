"""The cube demo must visit every edge without inventing diagonal transitions."""

import itertools

import numpy as np
from c3_draw_cube import cube_drawing_path, cube_edges, cube_vertices


def test_complete_cube_walk_has_only_geometric_edges():
    vertices = cube_vertices(np.zeros(3), 0.1)
    expected = {
        tuple(sorted((i, j)))
        for i in range(8)
        for j in range(i + 1, 8)
        if np.isclose(np.linalg.norm(vertices[i] - vertices[j]), 0.1)
    }
    assert len(expected) == 12
    assert {tuple(sorted(e)) for e in cube_edges()} == expected
    path = cube_drawing_path()
    actual = [tuple(sorted(e)) for e in itertools.pairwise(path)]
    assert set(actual) == expected
    assert len(actual) == 15  # three repeated edges connect the eight odd vertices


def test_time_parameterization_has_no_duplicate_segment_boundaries():
    from c3_draw_cube import build_cube_trajectory

    points = build_cube_trajectory(
        [np.zeros(6), np.ones(6) * 0.1, np.zeros(6)], dt=0.02
    )
    assert np.all(np.diff([p.t for p in points]) > 0)
    assert points[-1].t == 4.0


def test_line_markers_reset_reused_mesh_slots():
    import mujoco
    from c3_draw_cube import _add_line

    model = mujoco.MjModel.from_xml_string("<mujoco/>")
    scene = mujoco.MjvScene(model, maxgeom=2)
    # Connectors set pose/size only: stale mesh IDs would reach the GL renderer.
    scene.geoms[0].dataid = 12345
    scene.geoms[0].matid = 12345
    _add_line(scene, [0, 0, 0], [1, 0, 0], 0.01, [1, 0, 0, 1])
    assert scene.geoms[0].dataid == -1
    assert scene.geoms[0].matid == -1
    assert scene.ngeom == 1
