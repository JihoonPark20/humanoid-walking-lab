# W5：G1 Whole-body optimal control

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

## Torque regularization 与跟踪

推荐 FDDP 使用 `control_weight=3.0`，blended/joint-PD 使用 `0.1`。修改权重需要重新求解 OCP，并同时改变 nominal torque 和 feedback gain；metadata 保存该权重。不同权重的 objective 数值不能直接用于比较物理性能。若要使用同一成本比较策略，请显式指定 `--control-weight`。任意成本/gain 组合并不保证成功。

优化默认间隔 10 ms，MuJoCo 积分与跟踪间隔 1 ms。默认策略为 FDDP，可选混合策略为：

```text
e = state.diff(x_nominal, x_actual)
du_fddp = -K e
du_pd = -400 e_joint_position - 20 e_joint_velocity
u = u_ff + alpha du_fddp + (1-alpha) du_pd
```

`blended` 默认 `alpha=0.1`；`fddp` 使用完整 feedback，`joint-pd` 仅使用关节 PD。混合策略用于减弱局部模型产生的大幅校正，同时保留关节跟踪。Base feedback 仍通过 FDDP 关节 torque 作用。它不增加接触估计或在线重规划，也不提供稳定性证明。

通过条件为：计划收敛且通过独立约束检查、完整执行、无跌倒/flight/clipping、最终足端误差小于 3 cm。默认一/两步回归测试还要求足端误差小于 2 cm。不得用 nominal state 覆盖实际状态。验证矩阵包含三种推荐策略 × 一/两步 × 10/20 ms，共 12 个条件。

## 模型一致性

Pinocchio 4.0 的 MJCF loader 已填充 `model.armature`，再次添加到 Crocoddyl action 会重复计算。`audit_mass_matrices()` 对齐 free-flyer 速度坐标后，对照 MuJoCo、Pinocchio、contact/impact action 的质量矩阵。摩擦损失与柔顺接触仍不同于刚性接触规划。

## 实现

1. 加载 Lab 6 的 torque G1 和 Pinocchio 模型，对齐 floating-base 状态与 actuator 限制。
2. 使用 contact dynamics 与 impact action 构造迈步和双脚静止。
3. 用 Box-FDDP 处理 torque box 及 state/contact cost。
4. 比较 manifold feedback rollout 与独立 MuJoCo 执行。

- `whole_body_ocp.py`：模型、shooting problem、Box-FDDP、状态/输入/gain、独立验证。
- `whole_body_demo.py`：CLI、指标、图、可选 torque 回放与视频。
- `whole_body_tracking.py`：FDDP/joint-PD/blended 策略及参数验证。
- `validate_tracking_scenarios.py`：明确规划权重的 12 个实际跟踪条件。

Torque box 是直接约束。Joint/wrench barrier cost 不能单独保证可行性，必须检查残差。本实验不包含 contact-implicit 接触顺序搜索或避障。

## 状态、接触与 feedback

`x=(q,v)`，其中 q 包含 floating-base quaternion；`nq=36`、`nv=35`，保存状态为 71 维，切空间误差为 70 维。成本与 feedback 使用 manifold 差值。

```text
M(q) v_dot + h(q,v) = S.T u + J(q).T lambda
J(q) v_dot + J_dot(q,v) v = 0
u = u_nominal - K * state.diff(x_nominal, x_measured)
```

Contact action 约束支撑脚加速度。Impact action 不推进时间，保持位置并投影速度。MuJoCo 不应用这种速度 reset，实际速度变化由接触动力学决定。因此必须区分规划接触模型中的可行性与柔顺接触中的实际执行。

模型加载、依赖版本及结果见[运行指南](../README.zh-CN.md)。
