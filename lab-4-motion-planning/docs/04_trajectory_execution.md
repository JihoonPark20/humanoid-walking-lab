# Trajectory execution

[English](04_trajectory_execution.md) · [한국어](04_trajectory_execution.ko.md) · [简体中文](04_trajectory_execution.zh-CN.md)

`trajectory_executor.py` runs the torque-controlled UR5e with PD plus gravity compensation. It applies actuator limits, integrates MuJoCo and logs desired/actual joints, torque and end-effector position. Planning feasibility and tracking feasibility are separate.

`record_lab4_demo.py` records the complete native MuJoCo scene without text panels. It uses the same capstone run for video and measurements, writes `slalom_summary.json` and binds it to the video SHA-256. The summary also binds `planning_audit.json` by SHA-256. The gallery shows the full run at 2×; metrics are from the physical run, not GIF frames.

Acceptance requires RMS joint error <0.05 rad, final configuration error <0.05 rad, zero reported prohibited contacts and minimum collision/visible-mesh obstacle clearance ≥30 mm at every 1 ms tick. The independent planning audit must pass before execution. See the [recorded metrics](../media/slalom_summary.json), [planning checks](../media/planning_audit.json) and [gallery](../GALLERY.md).

Compare the desired trajectory, actual tracking, clearance curve and contact count. A visually plausible movie or small endpoint error does not establish collision avoidance. The guarantee is limited to the recorded scene, collision-pair policy and sampled resolution.
