"""충돌 필터로 보이지 않는 관통도 실제 형상 검사에서 검출한다."""

import mujoco
import numpy as np
from scene_clearance import ObstacleClearance


def test_disabled_contact_geometry_still_counts_and_state_is_unchanged():
    """ncon=0만으로 관통을 허용하지 않으며 거리 질의는 상태를 보존한다."""
    model = mujoco.MjModel.from_xml_string("""<mujoco><worldbody>
      <body name="robot"><freejoint/><geom type="box" size=".05 .05 .05" contype="0" conaffinity="0"/></body>
      <body name="box_1" pos=".08 0 0"><geom type="box" size=".05 .05 .05"/></body>
    </worldbody></mujoco>""")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    query = ObstacleClearance(model, ["box_1"])
    state = mujoco.mjtState.mjSTATE_INTEGRATION
    before = np.empty(mujoco.mj_stateSize(model, state))
    after = np.empty_like(before)
    mujoco.mj_getState(model, data, before, state)
    assert data.ncon == 0
    assert query.minimum(data) < -0.019
    mujoco.mj_getState(model, data, after, state)
    np.testing.assert_array_equal(before, after)
    data.qpos[0] = -0.1
    mujoco.mj_forward(model, data)
    np.testing.assert_allclose(query.minimum(data), 0.08, atol=1e-7)
