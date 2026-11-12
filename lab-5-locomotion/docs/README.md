# Lab 5: Study notes

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

[Commands and code order](../README.md) · [Shared concepts](../../docs/CONCEPTS.md)

## What a floating base adds

G1's base is not fixed to the world. Its pose has three position and four quaternion components; base velocity has three linear and three angular components. Thus `nq=36`, `nv=35`, with 29 actuators. The six base velocity coordinates have no direct motors.

MuJoCo stores quaternions as `(w,x,y,z)`, Pinocchio as `(x,y,z,w)`. Use the state conversion in `lab5_common.py`; never update a quaternion by ordinary `q+dq` addition.

## CoM, support polygon and ZMP

A static pose can be assessed using the ground projection of CoM and support polygon. Motion also depends on velocity, acceleration and contact force. ZMP is the ground point with zero tipping moment; it need not equal the CoM projection.

With constant CoM height, LIPM simplifies horizontal motion:

```text
cddot = ω²(c − p),  ω = sqrt(g / z_com)
c: horizontal CoM; p: ZMP
```

`lipm_planner.py` and `lipm_preview_control.py` are planning models. Stable reduced-model curves do not prove full-body MuJoCo feasibility.

## Whole-body IK and control

`whole_body_ik.py` matches both foot poses, CoM and pelvis conditions. Small IK error does not establish actuator/contact feasibility. During weight shifting, measure foot drift and pelvis height as well as CoM error.

This lab keeps double support for balance and weight shifting. Lab 6 adds contact-wrench and torque control for dynamic walking.

## Exercises

Vary lateral CoM displacement and transition duration separately. Compare foot slip, pelvis height and tracking error. When a fall occurs, identify the first torque saturation or contact change, not just the final frame.

[G1 joint map](g1_joint_map.md)
