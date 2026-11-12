# Walking under model mismatch

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

A controller uses an approximate model. This lab changes MuJoCo mass/inertia, ground/foot friction and pelvis forces while keeping the Pinocchio model and controller gains fixed. Unlike Lab 9's reduced-model observation experiments, these trials include full-body contact dynamics.

## Experiment design

`walking_benchmark.py` compares nominal walking with lateral and forward/backward pushes, friction changes, mass changes, a later push and a combined condition. Pushes last 0.15 s. Changing one factor at a time helps identify its effect; the combined condition tests interactions.

Keep controller settings fixed across conditions. Compare the same phase of the gait: the same impulse can have different effects in single and double support. A completed walk can still violate a joint or actuator limit. The mass +5% case illustrates this limitation, so completion alone is not a success criterion.

## Try it

1. Run the [lateral-push demo](../README.md). Watch the red force arrow and how CoM, DCM and measured ZMP respond.
2. Run the benchmark and compare nominal walking with each changed condition. Inspect landing error, loaded support, joint limits and contact force together.
3. Form a prediction before changing a push direction or its timing. Keep other parameters fixed and explain any difference from your prediction.

The [saved data](../GALLERY.md) include unsuccessful trials. These flat-ground examples describe a limited operating range, not reliability in arbitrary terrain or under arbitrary forces. State is measured from the simulator; no IMU estimator is used.
