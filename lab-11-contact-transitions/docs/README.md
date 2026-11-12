# Contact-aware walking

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Start with Lab 6's inverse-dynamics QP. This lab connects smooth foot references to measured contact: when a foot lands, the controller must change which tasks and contact constraints are active.

## C2 swing reference

For normalized swing time `s ∈ [0, 1]`, horizontal interpolation uses `10s³−15s⁴+6s⁵` and vertical clearance uses `64h·s³(1−s)³`. Endpoint velocity and acceleration vanish. In `smooth_swing.py`, compare these derivatives with the legacy sine-height trajectory. A smooth reference does not alone guarantee a smaller physical impact.

## Contact and control

Read `contact_walk.py` alongside Lab 6's `locomotion_controller.py` and `wb_id_qp.py`. Ground normal force determines loaded contact. Contact state is synchronized before enabling swing tasks. The inverse-dynamics QP uses ProxQP, foot orientation and CoM-height tasks, and contact velocity/orientation stabilization. LIPM height is measured from the ground, rather than the ankle frame.

The example takes four 8 cm steps on flat ground, with 0.85 s single support, 0.35 s double support and 2.5 cm reference clearance. Flat-foot orientation stabilization assumes flat ground and fixed yaw.

## Try it

1. Run the smooth and `legacy` profiles using the [run commands](../README.md), keeping timing and controller settings identical.
2. Compare the reference foot trajectory with actual lift and landing. Watch the violet swing arc, amber ZMP and green loaded support.
3. Explain why a foot can follow a smooth reference yet still land with an impact. Use contact force and tracking error, not the animation alone.

The simulation records foot poses, contacts, joint/torque limits and QP residuals. Use the [optional data](../GALLERY.md) when investigating a difference. First-contact velocity is a finite difference over the contact tick, not a direct pre-impact measurement.
