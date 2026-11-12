# W3：Centroidal 优化

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

固定 Lab 7 的接触时间表和落脚点，再共同优化 CoM 高度、线动量、角动量与双脚 wrench。建议按模型、非线性优化、独立验证、G1 物理量对照的顺序学习。

## 模块

- `centroidal_model.py`：状态 `[c,p,L]`、足底中心 wrench `[f,tau]`、固定接触时间表、精确离散动力学及独立约束检查。
- `centroidal_optimizer.py`：CasADi 自动微分与 IPOPT Direct Multiple Shooting；包含初始状态、节点参考、接触/摩擦/CoP/高度约束及可选终端静止条件。MPC 复用 `CentroidalSolver.solve(initial, reference, contacts, warm_start)`。
- `offline_centroidal.py`：以 Lab 7 结果 warm start，求解变高度任务，输出 NPZ/CSV/JSON/图。
- `pinocchio_centroidal.py`：将 G1 centroidal momentum/map 与独立的各刚体动量求和对照。
- `centroidal_io.py`：转换 Lab 7 时间表与参考，保存并重新验证计划。

数据流：Lab 7 计划 → 固定足位/接触 → 9D 状态、12D wrench NLP → 精确 rollout 验证 → Lab 9 MPC 与 G1 Whole-body controller。

## 模型

使用 `c_dot=p/m`、`p_dot=sum(f)+mg`、`L_dot=sum((r-c)×f+tau)`。区间内 world-frame wrench 与足位保持常量时，CoM 为二次函数，动量可以精确积分；角动量积分使用区间平均 CoM。简化状态不包含 quaternion 或躯干姿态。

摩擦采用方形 pyramid，CoP 采用足底中心力矩的线性限制，yaw moment 单独限制。这是完整矩形 contact wrench cone 的近似。摆动脚 wrench 为零，足部 yaw 和接触顺序固定。

独立于 solver 状态检查动力学、初末边界、摩擦、CoP、高度及动量约束。简化模型中可行的高度仍需通过 Whole-body 执行检验 G1 关节可行性。NumPy/CasADi 实现简化动力学，Pinocchio 计算实际 G1 的 CoM、Jacobian 与 centroidal map。

参见 [CasADi 文档](https://web.casadi.org/docs/)及[运行指南](../README.zh-CN.md)。
