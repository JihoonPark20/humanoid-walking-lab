# C3: Trace the twelve cube edges

[English](c3_draw_cube.md) · [한국어](c3_draw_cube.ko.md) · [简体中文](c3_draw_cube.zh-CN.md)

The UR5e visits all twelve cube edges in one continuous walk. A cube has eight odd-degree vertices, so traversing each edge exactly once is impossible. The demo repeats three edges: 16 vertices, 15 segments, 30 s of reference motion. Each consecutive vertex pair is a real cube edge; regression tests reject diagonal connections.

## Pipeline

`cube_vertices()` defines a 0.10 m cube centered at `[0.3,0.2,0.55]`. DLS IK uses fixed home orientation and the previous solution as the next seed. `build_cube_trajectory()` connects the solved joint poses with 2 s quintic segments, avoiding duplicate timestamps. This is joint-space interpolation: the end-effector can bow slightly away from a straight geometric edge.

The Menagerie position servo obeys approximately `tau=Kp(ctrl-q)-Kd*qd`. Gravity/bias and velocity feedforward are added through the position command:

```text
ctrl = q_des + bias(q,qd)/Kp + Kd*qd_des/Kp
```

The recorder uses MuJoCo `qfrc_bias`, which includes Coriolis terms as well as gravity, and settles for 3 s before tracing. It does not disable scene contact.

## Verification

The same run generates video and [measurements](../media/c3_draw_cube.json). Acceptance requires all twelve geometric edges and no diagonal transitions, joint-reference FK tracking RMS <1 mm, peak <5 mm, actual distance from the intended geometric edge <5 mm, zero MuJoCo contacts and complete execution. Reference-FK tracking error and geometric-edge distance are different metrics.

```bash
# From this lab directory; macOS interactive viewer:
mjpython src/c3_draw_cube.py
# Offscreen recording:
python src/c3_record_video.py
```

Compare plain position control with bias/velocity feedforward, keeping the same scene and speed. Inspect actuator limits, actual edge deviation and contacts together. The [gallery](../GALLERY.md) shows the complete run at 3×.
