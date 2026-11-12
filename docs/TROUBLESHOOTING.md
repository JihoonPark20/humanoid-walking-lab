# Troubleshooting

[English](TROUBLESHOOTING.md) · [한국어](TROUBLESHOOTING.ko.md) · [简体中文](TROUBLESHOOTING.zh-CN.md)

## Imports and model loading

- For a failed `import pinocchio`, inspect `python -m pip show pin` in the active environment. Package and import names differ.
- Missing mesh/XML: run `bash tools/setup_env.sh` to install external Menagerie assets. Copying the repository alone does not install them.
- Preserve Lab 2's `models/assets` symlink and relative lab directories.
- For same-named modules imported from another lab, inspect `module.__file__`. Keep root `conftest.py`, which handles these collisions during pytest collection.

## Rendering

Linux headless recording needs an EGL runtime and `MUJOCO_GL=egl`. Do not set EGL globally on macOS; use `mjpython` for passive viewers. First run numerical tests to separate model issues from rendering issues. Run offscreen recordings sequentially.

## Pinocchio/MuJoCo mismatch

1. Match joint names/order and the timestamp of `q`/`qdot`.
2. Compare the same frame/body/site; flange and tool tip differ.
3. Convert quaternion order: MuJoCo `(w,x,y,z)` ↔ Pinocchio `(x,y,z,w)`.
4. Match units, gravity, payload, mass/inertia and armature.
5. Update Pinocchio FK/frame placements and MuJoCo `mj_forward`.
6. Express Jacobian and error in the same frame. Pinocchio spatial vectors here use `[linear,angular]`.

## IK succeeds but the scene collides

IK may only satisfy pose. Set joints, call `mj_forward`, and inspect contact pairs. Planning must inspect edge interiors, not only endpoints. Adjacent-link exclusions are collision-model-specific. Lab 4 additionally validates the time-parameterized trajectory and physical execution.

## Humanoid falls

Gravity compensation alone does not stabilize posture. Compare CoM/DCM error, foot contacts, pelvis height and torque saturation over time. Check unintended environment/link collisions. Read QP status and physical residuals together; reaching the iteration cap is not normal convergence.

[Overview](../README.md)
