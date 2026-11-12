"""Record a video of the UR5e cube-drawing demo using MuJoCo offscreen rendering.

Produces an MP4 file without needing a display (headless). Reuses all
trajectory generation and control logic from c3_draw_cube.py.

Usage:
    python3 lab-2-Ur5e-robotics-lab/src/c3_record_video.py
    # → lab-2-Ur5e-robotics-lab/media/c3_draw_cube.mp4
"""

from __future__ import annotations

import hashlib
import itertools
import json
import math
import sys
from pathlib import Path

_SRC_DIR = Path(__file__).resolve().parent
if str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))

import imageio.v2 as iio
import mujoco
import numpy as np
import pinocchio as pin
from c3_draw_cube import (
    CUBE_CENTER,
    CUBE_SIDE,
    EE_MARKER_COLOR,
    KD,
    KP,
    SEGMENT_DURATION,
    TRAIL_DECIMATE,
    _add_sphere,
    _draw_cube_wireframe,
    _draw_trail,
    build_cube_trajectory,
    cube_drawing_path,
    cube_edges,
    cube_vertices,
    solve_cube_ik,
)
from ur5e_common import (
    MEDIA_DIR,
    MJCF_SCENE_PATH,
    NUM_JOINTS,
    load_pinocchio_model,
)

# ── Video settings ────────────────────────────────────────────────────────
WIDTH, HEIGHT = 960, 640
FPS = 30
OUT_PATH = MEDIA_DIR / "c3_draw_cube.mp4"


def main() -> None:
    print("=" * 72)
    print("C3: Record Cube-Drawing Video (Offscreen)")
    print("=" * 72)

    # Load models
    pin_model, pin_data, ee_fid = load_pinocchio_model()

    # Cube geometry
    verts = cube_vertices(CUBE_CENTER, CUBE_SIDE)
    path = cube_drawing_path()

    # Solve IK
    print("\n  Solving IK for cube path...")
    joint_wps = solve_cube_ik(verts, path, pin_model, pin_data, ee_fid)
    print(f"  Solved {len(joint_wps)} waypoints")

    # Build trajectory
    dt = 0.002
    traj = build_cube_trajectory(joint_wps, dt)
    total_time = traj[-1].t

    print(f"\n  Recording {total_time:.1f}s → {OUT_PATH}")
    print(f"  Resolution: {WIDTH}x{HEIGHT} @ {FPS} fps")
    print(f"  Simulation dt={dt}; frames sampled by simulation time")

    # MuJoCo model
    model = mujoco.MjModel.from_xml_path(str(MJCF_SCENE_PATH))
    data = mujoco.MjData(model)

    # Initialise at first trajectory waypoint
    data.qpos[:NUM_JOINTS] = traj[0].q
    data.qvel[:NUM_JOINTS] = 0.0
    data.ctrl[:NUM_JOINTS] = traj[0].q
    mujoco.mj_forward(model, data)

    # Settle at start position (3 s) so PD servo reaches steady state
    print("  Settling at start position (3 s)...")
    for _ in range(1500):
        g = data.qfrc_bias[:NUM_JOINTS].copy()
        data.ctrl[:NUM_JOINTS] = traj[0].q + g / KP
        mujoco.mj_step(model, data)

    ee_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")

    # Match the requested recording size to the offscreen framebuffer.
    model.vis.global_.offwidth = WIDTH
    model.vis.global_.offheight = HEIGHT

    # Renderer
    renderer = mujoco.Renderer(model, HEIGHT, WIDTH)

    # Camera — angled view to see the 3D cube
    cam = mujoco.MjvCamera()
    cam.lookat[:] = CUBE_CENTER
    cam.distance = 1.2
    cam.elevation = -30.0
    cam.azimuth = 135.0

    # Simulation + rendering loop
    trail: list[np.ndarray] = []
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    pending = OUT_PATH.with_name(OUT_PATH.stem + ".pending.mp4")
    writer = iio.get_writer(str(pending), fps=FPS, codec="libx264")
    frame_count = 0
    cart_errors: list[float] = []
    max_torque = 0.0
    max_edge_distance = 0.0
    max_contacts = 0

    for step in range(len(traj)):
        tp = traj[step]

        # Position control with gravity compensation + velocity feedforward
        g = data.qfrc_bias[:NUM_JOINTS].copy()
        data.ctrl[:NUM_JOINTS] = tp.q + g / KP + KD * tp.qd / KP
        mujoco.mj_step(model, data)

        mujoco.mj_forward(model, data)

        # Metrics
        ee = data.site_xpos[ee_site_id].copy()
        pin.forwardKinematics(pin_model, pin_data, tp.q)
        pin.updateFramePlacements(pin_model, pin_data)
        ee_des = pin_data.oMf[ee_fid].translation.copy()
        err = np.linalg.norm(ee_des - ee)
        cart_errors.append(err)
        segment = min(int(tp.t / SEGMENT_DURATION), len(path) - 2)
        a, b = verts[path[segment]], verts[path[segment + 1]]
        fraction = np.clip(np.dot(ee - a, b - a) / np.dot(b - a, b - a), 0., 1.)
        max_edge_distance = max(max_edge_distance, float(np.linalg.norm(ee - (a + fraction * (b - a)))))
        max_contacts = max(max_contacts, data.ncon)
        act_force = data.qfrc_actuator[:NUM_JOINTS]
        max_torque = max(max_torque, float(np.max(np.abs(act_force))))

        if step % TRAIL_DECIMATE == 0:
            trail.append(ee.copy())

        # Render frame
        if tp.t + 1e-9 >= frame_count / FPS:
            renderer.update_scene(data, cam)

            # Add overlay geoms
            _draw_cube_wireframe(renderer.scene, verts)
            _draw_trail(renderer.scene, trail)
            _add_sphere(renderer.scene, ee, 0.006, EE_MARKER_COLOR)

            frame = renderer.render()
            writer.append_data(frame)
            frame_count += 1

            pct = 100.0 * step / len(traj)
            print(f"\r  {pct:5.1f}%  t={tp.t:.2f}s  frames={frame_count}", end="", flush=True)

    renderer.close()

    # Write video
    MEDIA_DIR.mkdir(parents=True, exist_ok=True)
    writer.close()

    rms = math.sqrt(sum(e * e for e in cart_errors) / len(cart_errors))
    print(f"\n\n  Video saved: {OUT_PATH}  ({frame_count} frames)")
    print(f"  RMS error : {rms * 1000:.3f} mm")
    print(f"  Max error : {max(cart_errors) * 1000:.3f} mm")
    print(f"  Max torque: {max_torque:.2f} N·m")
    traversed = {tuple(sorted(edge)) for edge in itertools.pairwise(path)}
    expected = {tuple(sorted(edge)) for edge in cube_edges()}
    checks = {"all_cube_edges_only": traversed == expected,
              "edge_distance_below_5mm": max_edge_distance < .005,
              "no_contacts": max_contacts == 0,
              "rms_below_1mm": rms < 0.001,
              "peak_below_5mm": bool(max(cart_errors) < 0.005),
              "complete": len(cart_errors) == len(traj)}
    summary = {"passed": all(checks.values()), "checks": checks,
               "rms_error_m": rms, "max_error_m": float(max(cart_errors)),
               "max_torque_nm": max_torque, "vertex_path": path,
               "max_geometric_edge_distance_m": max_edge_distance, "max_contact_count": max_contacts,
               "unique_edges": len(traversed), "segments": len(path) - 1, "simulation_duration_s": float(total_time),
               "frames": frame_count, "fps": FPS,
               "video_sha256": hashlib.sha256(pending.read_bytes()).hexdigest()}
    if not summary["passed"]:
        pending.unlink(missing_ok=True)
        raise SystemExit("Recorded trajectory did not pass the tracking checks.")
    pending.replace(OUT_PATH)
    OUT_PATH.with_suffix(".json").write_text(json.dumps(summary, indent=2) + "\n")


    print("\n" + "=" * 72)
    print("C3: Video Recording — COMPLETE")
    print("=" * 72)


if __name__ == "__main__":
    main()
