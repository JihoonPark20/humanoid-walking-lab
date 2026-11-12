# W4：从测量状态重新规划的 MPC

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

## 延迟与模型误差

组合场景保持 `mass_ratio=1.05`、摩擦 0.4、一个区间测量延迟和 0.03 m/s 扰动。利用带 timestamp 的历史测量与已发送命令预测当前状态；未观测扰动不能提前预测，只能从后续测量进入估计。Controller 不接收当前 plant 状态或真实质量。

质量识别只使用连续三次 CoM 高度与两个区间的竖直命令：

```text
v_bar0 = (z1-z0)/h0
v_bar1 = (z2-z1)/h1
m_candidate = (F0*h0 + F1*h1) / (2*(v_bar1-v_bar0) + g*(h0+h1))
m_hat <- m_hat + 0.3*(m_candidate-m_hat)
```

拒绝 nominal 质量 0.75–1.25 倍之外的候选；所有条件使用相同边界。重复 timestamp 复用，逆序时间拒绝。丢失测量后，等待三次连续测量再估计。竖直外力及 command-force 误差无法与质量误差区分，超出该识别方法的假设。

`CentroidalMPC.update_mass()` 更新 solver mass parameter，以及同一 nominal 速度对应的 reference momentum/wrench。Fallback 必须用新模型重新 rollout 和验证。质量改变使 RTI preparation 失效。简化实验按测量 → 估计 → preparation → feedback 执行，只解一次 QP；固定质量 G1 保留提前 preparation。

`validate_mpc_scenarios.py` 测试八种环境 × full NLP/RTI，共 16 个条件。要求完成全部区间、最终 CoM 误差小于 3 cm，并独立验证实际 plant state/wrench。`command_wrench_violation` 与 `saturation_intervals` 展示 plant 对命令的限幅。

## RTI preparation 与 feedback

`centroidal_rti.py` 用 CasADi 微分和 OSQP 实现单次 SQP。`prepare(index)` shift 上次解，构造精确离散动力学 Jacobian、二次 tracking cost 和线性 wrench/state 约束。`feedback(measured)` 仅修改初态等式 RHS，**只解一次 QP**。候选输入须通过独立非线性 rollout；否则仅允许从新测量状态重新验证过的旧输入作为 fallback，不重新调用 full NLP。

```text
min_delta  0.5 delta.T H delta + grad(f).T delta
subject to l <= g(z_bar) + J_g(z_bar) delta <= u
           delta_x0 = x_measured - x_bar0
```

Tracking objective 是二次函数，所以 `H=Hessian(f)` 等于其 Gauss–Newton Hessian；不使用非线性约束 Hessian。实现采用 absolute QP `z=z_bar+delta`，momentum 按质量、wrench 按重量归一化，并缩放约束行。保留 wrench/state 限制，仅线性化非线性动力学。

QP 后对候选 wrench 做精确非线性 rollout 决定是否采用。`linearization_gap` 是线性预测与精确 rollout 的最大分量差，混合位置与动量单位。单次 SQP 不保证可行或收敛。

固定模型 G1 提前调用 `prepare(k+1)`。便捷接口 `solve(measured,index)` 顺序执行两个阶段，供带质量估计的简化实验使用。每次 preparation 重建 sparse QP 与 factorization，feedback 只更新 RHS。没有跨 horizon factorization 复用、condensing 或代码生成。时间统计含 preparation、feedback、估计与总耗时，不含首次 graph 创建。

该教学实现参考 [acados 文档](https://docs.acados.org/features/)的阶段划分，但不使用 acados backend，也不保证 hard real-time。

## 实现与模块

先构造精确 ZMP-input LIPM 和 horizon QP，再从最终双脚支撑区域反向计算 terminal DCM viability；在同一扰动下比较 offline feedback 与 MPC。随后以 shifted horizon 复用 Lab 8 centroidal NLP，并将 CoM/momentum reference 接入 Lab 6 Whole-body QP。

- `lipm_mpc.py`：`[c_x,v_x,c_y,v_y]`、ZMP 输入、OSQP horizon、terminal DCM 区域、制动 fallback。
- `centroidal_mpc.py`：Lab 8 solver、shifted primal warm start、延迟/质量/摩擦实验。
- `centroidal_rti.py`：自动微分、one-QP RTI、非线性验证、分阶段计时。
- `centroidal_estimation.py`：timestamp 预测和高度/命令质量识别。
- `validate_mpc_scenarios.py`：16 个环境/算法条件与判定矩阵。
- `mpc_experiment.py`：配对 open-loop/offline feedback/MPC 简化模型 rollout，CSV/JSON/图及 median/p95/p99/deadline miss。
- `g1_mpc.py`：Pinocchio 实测状态 → MPC → Lab 6 CoM/angular-momentum task → Whole-body QP → MuJoCo。

不执行不可行解。LIPM 仅在 capture point 位于当前保守支撑区时使用已验证制动，否则停止。Centroidal fallback 必须从新测量状态重新验证旧计划，否则停止。记录失败和实际采用路径。比较 full NLP 与 RTI 时保持 nominal、扰动、horizon 一致，并比较总计算时间。

## Terminal DCM 区域

令 `omega=sqrt(g/h)`、`xi=c+v/omega`；constant ZMP 下 `xi_next=exp(omega*dt)*xi+(1-exp(omega*dt))*z`。反向递推为：

```text
alpha = exp(-omega * dt)
lower[k] = alpha * lower[k+1] + (1-alpha) * zmp_lower[k]
upper[k] = alpha * upper[k+1] + (1-alpha) * zmp_upper[k]
```

将 horizon 末端 DCM 限制在该时刻的区域。它假设固定接触时间、解耦水平轴与 constant-input LIPM，并不证明 G1 关节/torque 可行。阅读[实验展示](../GALLERY.zh-CN.md)时区分简化模型与 G1；另见[观测鲁棒性](OBSERVATION_ROBUSTNESS.zh-CN.md)。
