# Trajectory processing

[English](03_trajectory_processing.md) · [한국어](03_trajectory_processing.ko.md) · [简体中文](03_trajectory_processing.zh-CN.md)

`shortcut_path()` chooses nonadjacent waypoints and replaces their subpath only after checking the connecting edge with the same collision checker. It preserves endpoints and cannot assume that safe nodes imply a safe edge.

`parameterize_topp_ra(path,vel_limits,acc_limits,dt)` returns `(times,q,qd,qdd)`. Reject empty/nonfinite paths, nonpositive limits and invalid time steps. Remove consecutive duplicates before constructing arc length. If TOPP-RA is available, parameterize its interpolating spline; otherwise use conservative quintic segments. Always include the exact final time even when duration is not a multiple of `dt`.

Quintic segments stay on the straight joint-space edge, with zero endpoint velocity/acceleration. Spline interpolation can change path geometry and cut a corner into an obstacle. Therefore the capstone rechecks every timed position, velocity and acceleration before execution and aborts on a violation. A successful geometric planner alone does not certify a spline.

Read `planning_audit.json`: raw, shortcut and timed checks are separate. Try a shortcut across the blocking post and confirm rejection. The optional TOPP-RA backend is not required for the published quintic demo; its real installed backend requires its own run, while regression tests also check final-time sampling with a controlled backend.
