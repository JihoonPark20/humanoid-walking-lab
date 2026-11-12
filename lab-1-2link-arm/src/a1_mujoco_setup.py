"""MuJoCo 모델을 읽고 step, qpos, actuator 입력을 확인하는 입문 예제.
Verification:
  1. mj_step() runs without errors
  2. data.qpos returns a 2-element array
  3. Joint angles change when motor commands are applied"""
import numpy as np
import mujoco

MODEL_PATH = "models/two_link.xml"

def load_model(xml_path: str = MODEL_PATH):
    """MJCF에서 MuJoCo 모델과 상태 데이터를 생성한다."""
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    return model, data


def test_basic_simulation():
    """mj_step을 100회 실행할 수 있는지 확인한다."""
    model, data = load_model()

    # Run 100 steps
    for _ in range(100):
        mujoco.mj_step(model, data)

    print("[OK] mj_step() ran 100 steps without errors")
    print(f"     Simulation time: {data.time:.4f} s")
    return model, data


def test_qpos():
    """qpos가 두 관절의 위치 배열인지 확인한다."""
    model, data = load_model()
    mujoco.mj_step(model, data)

    qpos = data.qpos
    assert qpos.shape == (2,), f"Beklenen (2,), gelen {qpos.shape}"
    print(f"[OK] data.qpos shape: {qpos.shape}")
    print(f"     Joint angles (rad): [{qpos[0]:.4f}, {qpos[1]:.4f}]")
    print(f"     Joint angles (deg): [{np.degrees(qpos[0]):.2f}°, {np.degrees(qpos[1]):.2f}°]")


def test_motor_control():
    """Actuator 명령을 넣으면 관절각이 변하는지 확인한다."""
    model, data = load_model()

    # Save initial state
    mujoco.mj_step(model, data)
    qpos_before = data.qpos.copy()

    # Apply motor commands: +5 to joint1, -3 to joint2
    data.ctrl[0] = 5.0
    data.ctrl[1] = -3.0

    # Run 500 steps (to allow motion to develop)
    for _ in range(500):
        mujoco.mj_step(model, data)

    qpos_after = data.qpos.copy()
    delta = qpos_after - qpos_before

    print(f"[OK] Joint angles changed after motor command")
    print(f"     Before: [{qpos_before[0]:.4f}, {qpos_before[1]:.4f}] rad")
    print(f"     After:  [{qpos_after[0]:.4f}, {qpos_after[1]:.4f}] rad")
    print(f"     Change: [{delta[0]:.4f}, {delta[1]:.4f}] rad")
    assert np.any(np.abs(delta) > 0.01), "Joint angles did not change!"


def test_sensors():
    """Read sensor data."""
    model, data = load_model()

    # Generate some motion
    data.ctrl[0] = 3.0
    data.ctrl[1] = -2.0
    for _ in range(200):
        mujoco.mj_step(model, data)

    # Sensor data
    print(f"\n--- Sensor Data ---")
    print(f"Joint positions (qpos):  [{data.qpos[0]:.4f}, {data.qpos[1]:.4f}] rad")
    print(f"Joint velocities (qvel): [{data.qvel[0]:.4f}, {data.qvel[1]:.4f}] rad/s")
    print(f"Sensor data (sensordata):   {data.sensordata}")

    # End-effector position (via site)
    ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
    mujoco.mj_forward(model, data)
    ee_pos = data.site_xpos[ee_id]
    print(f"End-effector position:   x={ee_pos[0]:.4f}, y={ee_pos[1]:.4f}, z={ee_pos[2]:.4f}")


def print_model_info():
    """Basic information about the model."""
    model, data = load_model()

    print("=" * 50)
    print("MODEL INFO")
    print("=" * 50)
    print(f"Model name:   {model.names[1:model.names.index(0, 1)].decode()}" if model.names[0] == 0 else "")
    print(f"Body count:   {model.nbody}")
    print(f"Joint count:  {model.njnt}")
    print(f"DOF:          {model.nv}")
    print(f"Actuator:     {model.nu}")
    print(f"Sensor:       {model.nsensor}")
    print(f"Timestep:     {model.opt.timestep} s")
    print(f"Gravity:      {model.opt.gravity}")
    print("=" * 50)


if __name__ == "__main__":
    print_model_info()
    print()
    test_basic_simulation()
    print()
    test_qpos()
    print()
    test_motor_control()
    print()
    test_sensors()
    print()
    print("✓ All A1 verification criteria passed!")
