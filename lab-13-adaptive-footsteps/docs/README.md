# Adjusting footsteps from measured state

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Lab 9 replans CoM and wrench with fixed feet. This lab uses measured DCM to change future landings while keeping the contact order and timing fixed.

## Bounded adjustment

In `step_adjustment.py`, at 45% of each swing, a DCM error above 8 mm triggers a correction. The objective is `||delta − exp(omega·remaining)·error||² + ||delta||²`. Projecting its unconstrained minimizer into a rectangle solves this small convex QP. Bounds limit correction to ±4 cm per axis, forward reach to −2…20 cm and step width to 14…24 cm.

A quintic boundary-value polynomial preserves the current reference position, velocity and acceleration, then ends at rest at the new landing. Vertical extrema are checked against the endpoint ground reference. Future landings move consistently and DCM references are rebuilt. This is a placement policy, not joint footstep/timing optimization; state comes from the simulator.

## Try it

1. Run the [demo](../README.md) and compare gray original footprints with violet revised landings. Watch how the remaining swing path changes after the push.
2. In `adaptive_walk.py`, compare fixed and adaptive modes under identical 0, ±10 and ±20 N lateral pushes lasting 0.15 s.
3. Inspect adjustment time, old/new landing and bounds in the [data](../GALLERY.md). Explain why some disturbances trigger no correction and others reach a bound.

Compare landing accuracy, joint limits and recovery motion together. DCM error is measured against each policy's own reference, so a smaller value alone does not establish better recovery.
