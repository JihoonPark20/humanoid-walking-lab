"""시각화의 접촉 ZMP는 실제 하중을 반영하며 시뮬레이터 상태를 바꾸지 않는다."""

import mujoco
import numpy as np

from tools.walking_visuals import ground_contacts, ground_point


def test_contact_zmp_uses_loads_and_ignores_unloaded_feet():
    """서로 다른 질량의 두 발에서 하중 가중 ZMP와 무접촉 상태를 확인한다."""
    model = mujoco.MjModel.from_xml_string("""
    <mujoco><option timestep="0.002"/>
    <worldbody><geom type="plane" size="1 1 .01"/>
      <body name="left_ankle_roll_link" pos="0 .1 .03"><freejoint/>
        <geom type="box" size=".05 .02 .03" mass="2"/></body>
      <body name="right_ankle_roll_link" pos="0 -.1 .03"><freejoint/>
        <geom type="box" size=".05 .02 .03" mass="1"/></body>
    </worldbody></mujoco>""")
    data = mujoco.MjData(model)
    for _ in range(500):
        mujoco.mj_step(model, data)
    state_spec = mujoco.mjtState.mjSTATE_INTEGRATION
    before = np.empty(mujoco.mj_stateSize(model, state_spec))
    after = np.empty_like(before)
    mujoco.mj_getState(model, data, before, state_spec)
    hull, zmp, active = ground_contacts(model, data)
    mujoco.mj_getState(model, data, after, state_spec)
    np.testing.assert_array_equal(before, after)
    assert active == (True, True)
    np.testing.assert_allclose(zmp, [0, 1 / 30], atol=1e-5)
    np.testing.assert_allclose(hull.min(axis=0), [-0.05, -0.12], atol=1e-5)
    np.testing.assert_allclose(hull.max(axis=0), [0.05, 0.12], atol=1e-5)

    data.qpos[[2, 9]] += 1
    mujoco.mj_forward(model, data)
    hull, zmp, active = ground_contacts(model, data)
    assert hull.shape == (0, 2)
    assert zmp is None
    assert active == (False, False)


def test_ground_projection_preserves_the_planner_sample():
    """지면 표시를 만들 때 원본의 계획 높이를 덮어쓰지 않는다."""
    planned = np.array([0.2, -0.1, 0.65])
    ground = ground_point(planned)
    np.testing.assert_array_equal(planned, [0.2, -0.1, 0.65])
    np.testing.assert_array_equal(ground, [0.2, -0.1, 0.012])
