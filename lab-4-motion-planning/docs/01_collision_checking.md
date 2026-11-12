# Lab 4: Collision Checking

## Final Architecture

Lab 4 collision checking now uses the canonical executed MuJoCo geometry:

- Menagerie `universal_robots_ur5e`
- mounted `robotiq_2f85`
- table and obstacle geoms added to that same scene

The planner no longer treats a separate custom collision URDF as the primary
truth source. Pinocchio remains available for FK utilities, but collision truth
comes from the exact geometry that MuJoCo executes.

## CollisionChecker API

`CollisionChecker` preserves the original Lab 4 interface:

- `is_collision_free(q) -> bool`
- `is_path_free(q1, q2, resolution=0.05) -> bool`
- `compute_min_distance(q) -> float`

Internally it:

1. loads the canonical MuJoCo scene
2. registers robot-vs-environment and relevant self-collision geom pairs
3. evaluates collisions on the actual MuJoCo scene state for each queried `q`

## Self-Collision Policy

- Arm-vs-arm and arm-vs-gripper pairs are checked
- Robot-vs-environment pairs are checked
- Internal Robotiq finger-link proximity inside the same mechanism is excluded
  from the monitored self-collision set because it is not meaningful planning
  signal for this lab

## Why This Change Matters

Planning and execution now reason about the same robot geometry. That removes
the earlier mismatch where a separate planning model could disagree with the
executed stack.

## Final Validation

The recorded run also measures obstacle clearance at every 1 ms physics tick,
using the live finger configuration and both collision primitives and visible
meshes (`scene_clearance.py`). Visual-only geometry is checked even when its
contact mask is disabled. Mesh distances use MuJoCo's convex hulls, which are
conservative for concave surfaces. Publication requires at least 30 mm clearance
throughout and zero physical contacts; a sparse planned-path check alone is insufficient.

- `tests/test_collision.py` passes on the canonical stack
- free and colliding configurations are validated on the real scene
- FK agreement between Pinocchio and MuJoCo is preserved for the canonical
  end-effector frame
