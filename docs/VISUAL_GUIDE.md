# Reading the walking demos

[English](VISUAL_GUIDE.md) · [한국어](VISUAL_GUIDE.ko.md) · [简体中文](VISUAL_GUIDE.zh-CN.md)

Labs 5–13 render markers directly in MuJoCo, without text or inset panels. The ground ring is the vertical CoM projection, not a contact point. Paths and support boundaries use world coordinates; the camera follows the robot.

| Lab | What to watch |
|---|---|
| 5 · Balance | CoM moves toward the target while both feet stay planted. |
| 6 · DCM/ZMP | Loaded support switches between one and two feet. Compare the controller's DCM, its target, and contact ZMP. |
| 7 · Offline optimization | Gray seed footsteps versus violet optimized footsteps and CoM. The plan is fixed before execution; cyan traces actual motion. |
| 8 · Centroidal dynamics | A floating mass, planned contact forces, CoM height and angular momentum. This is a reduced-model animation, not a whole-body simulation. |
| 9 · MPC/RTI | The violet CoM prediction is refreshed after each controller update. Compare it with the gray nominal trajectory before, during and after the red push. Translucent spheres sample the current prediction horizon. |
| 10 · Whole-body OCP | Violet ghosts show the offline optimized joint configurations at +0.25/+0.50 s, interpolated on the configuration manifold. The solid robot runs feedback in MuJoCo. Ghosts disappear beyond the plan's end. |
| 11 · Contact-aware Walking | Left: sine swing; right: C2 swing. Compare the foot and remaining reference near the first landing. |
| 12 · Physical Walking Robustness | Left: no push; right: lateral push. Compare the response at the same gait phase with the same controller. |
| 13 · Adaptive Foot Placement | Left: fixed feet; right: adaptive feet under the same push. Watch the revised footprint and swing path. |

Labs 11–13 show synchronized comparisons: the reference case is on the left, the changed case on the right. Lab 11 changes the swing profile, Lab 12 adds a push, and Lab 13 enables foot placement adjustment under the same push. Read the caption for playback speed. Close-ups use the original recording pixels; full-length source videos remain linked. Compare the indicated mechanism rather than interpreting every change as improved performance.

**Colors:** cyan = measured CoM (planned mass in Lab 8); white = reference; violet = plan/prediction (DCM in Lab 6); amber = contact ZMP; green = loaded support or planned force; red = applied disturbance.

The green hull encloses force-bearing foot/ground contact points, so it may shrink as a foot unloads. Amber ZMP is the normal-force-weighted contact position on flat ground; it disappears without loaded contacts. Lab 8 instead shows scheduled contact patches and planned forces, with arrows scaled to **1.5 mm/N**. Lab 9's push arrow uses **60 mm/N** for visibility; sphere size and ghost transparency do not represent uncertainty.

Centroidal MPC predicts CoM/momentum/contact forces, not joint configurations. Its translucent spheres therefore show predicted CoM; full robot ghosts are reserved for Lab 10's actual joint-space plan. Lab 10 is offline OCP with feedback, not whole-body MPC.

Each lab’s **Demo** page provides the full-resolution recording, comparison plots and optional raw data. After recording with its README command, regenerate previews with `python tools/build_gallery.py`.

[← Curriculum](../README.md)
