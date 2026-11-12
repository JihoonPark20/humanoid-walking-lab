"""Visual connectors must not reuse stale mesh IDs from a previous scene."""

import mujoco
from c1_draw_square import _add_line


def test_line_marker_resets_mesh_metadata():
    model = mujoco.MjModel.from_xml_string("<mujoco/>")
    scene = mujoco.MjvScene(model, maxgeom=1)
    scene.geoms[0].dataid = 12345
    scene.geoms[0].matid = 12345
    _add_line(scene, [0, 0, 0], [1, 0, 0], 0.01, [0, 1, 0, 1])
    assert scene.geoms[0].dataid == -1
    assert scene.geoms[0].matid == -1
