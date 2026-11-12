# RRT and RRT*

[English](02_rrt_planning.md) · [한국어](02_rrt_planning.ko.md) · [简体中文](02_rrt_planning.zh-CN.md)

`RRTStarPlanner` stores nodes `(q,parent,cost)` in six-joint configuration space. Sampling respects joint bounds; goal bias occasionally selects the goal. Each iteration finds the nearest node, steers by at most `step_size` and validates the full new edge.

RRT attaches to the nearest node and returns the first valid goal connection. RRT* chooses the cheapest collision-free parent within `rewire_radius`, rewires neighbors when cheaper, propagates costs through all descendants and continues until the iteration budget. Goal reconnection also updates descendant costs. Cost is cumulative joint-space Euclidean distance, not physical time or energy. This implementation uses a fixed neighborhood radius; finite-run optimality is not guaranteed.

`plan(q_start,q_goal,max_iter=5000,rrt_star=True,seed=42)` returns a waypoint list or `None`. Invalid bounds/nonfinite inputs are rejected, colliding/out-of-range endpoints fail, identical safe endpoints give a one-point path. Failure clears the previous tree. An impossible wall must return `None`, never an unchecked shortcut.

The capstone solves IK for required task waypoints and plans each segment with full joint limits, shared search settings and one fixed seed (42), without seed retries. `planning_validation.py` audits each raw path and its shortcut independently. [Regression tests](../tests/test_planning_safety.py) use an analytic 2D obstacle checker, impossible-wall negative control and a real UR5e segment blocked by the purple post. They also verify rewiring costs, deep descendant updates and multiple seeds.

A single successful seed is not a success-rate benchmark. Compare repeated trials under the same scene, start/goal, limits and budget; report failures as well as path costs.

[Multi-environment evaluation](GENERALIZATION.md)
