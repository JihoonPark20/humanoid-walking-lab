# Lab 7：行走优化与模块连接

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

本实验实现 W0 基准行走、W1 离线 LIPM 优化，以及 W2 落脚点和时间候选比较。W3 Centroidal 优化、W4 MPC、W5 Whole-body optimal control 分别位于 Lab 8–10。

## 实现与数据流

1. 定义分段常量 jerk 的精确动力学、平地固定 yaw 足位及单/双脚支撑；测试积分与边界连续性。
2. 构建初始/终端静止条件、支撑约束、ZMP/jerk 成本的 QP；独立验证动力学和区间内 ZMP 极值。
3. 将足位加入变量，约束步长、步宽和着陆区域；记录每个时间候选的可行性与成本。
4. CLI 导出 NPZ/CSV/JSON/图。
5. 用 Lab 6 的 torque dynamics、Whole-body QP 与接触处理跟踪计划，并与基准 DCM 行走比较。规划和物理执行分别判定。

| 模块 | 作用 |
|---|---|
| `walking_model.py` | 配置、接触、落脚点、精确 jerk dynamics |
| `lipm_optimizer.py` | 状态/jerk/足位 QP、不可行处理、连续区间验证 |
| `walking_artifacts.py` | NPZ 读写、CSV/JSON、图与动画 |
| `offline_walk.py` | W1 固定足位与 W2 足位/时间 CLI |
| `g1_walk.py` | Lab 6 adapter、基准/优化 G1 执行和测量 |

初态 + 足位 + 接触时间 + 距离 → `WalkingSchedule` → sparse QP → `WalkingPlan` → 独立验证 → G1 reference adapter → Whole-body QP → torque → MuJoCo。

接触顺序固定。双脚支撑时，小矩形沿两个足中心的给定凸组合移动。对同尺寸、固定 yaw 的脚，该区域是实际支撑凸包的保守子集。区域中心与足位呈 affine 关系，因此每个 W2 时间候选仍是 convex QP。时间通过枚举候选选择，并非连续优化。

## 接口

- `build_schedule(config, initial_feet=None)` → `WalkingSchedule`
- `solve_walk(schedule, initial_state=None, optimize_feet=False, landing_bounds=None)` → `WalkingPlan`
- `WalkingPlan.sample(t)` → 位置、速度、加速度、ZMP、DCM 参考
- `validate_plan(plan)` → 动力学残差、连续区间支撑违规、足位与边界验证

核心优化不依赖外部机器人模型。G1 使用 Menagerie MJCF 和 Lab 5/6 loader。Pinocchio 计算 FK、CoM、Jacobian 与动力学，MuJoCo 积分实际运动和接触。

## W1 方程与变量

每个水平轴采用 `x=[c,v,a]`，在区间 `h` 内 jerk `u` 恒定。以下是精确多项式积分：

```text
c[k+1] = c[k] + h v[k] + h²/2 a[k] + h³/6 u[k]
v[k+1] = v[k] + h a[k] + h²/2 u[k]
a[k+1] = a[k] + h u[k]
ω² = g / z_com
ZMP = c − a / ω²
DCM ξ = c + v / ω
ξ_dot = v + a / ω = ω (ξ − ZMP)
```

先读 `jerk_matrices()`、`polynomial_state()`、`WalkingPlan.sample()`。直接线性插值状态可能破坏 ZMP/DCM 关系。QP 变量是所有节点的 x/y 状态、区间的 x/y jerk 和各足 x/y 坐标。W1 用等式固定全部足位，W2 仅固定最初两脚。

```text
J = w_zmp ∫ ||ZMP(t) − support_center(t)||² dt
  + w_jerk ∫ ||jerk(t)||² dt
  + w_foot Σ ||foot_i − nominal_foot_i||²
```

ZMP 与支撑中心的差为三次函数，平方为六次；四点 Gauss quadrature 可精确积分。QP 与 `plan_cost()` 使用同一公式。成本依赖权重，不等同于电能或机械功。

边界条件固定初始 `(c,v,a)`、终端 `v=a=0`，并让最终 CoM 位于最终足中心加足底 offset。默认 ankle-to-sole x offset 为 0.035 m，是 G1 几何参数，不是通用常数。

## 连续支撑验证

单脚支撑使用支撑足内的小矩形，双脚支撑使用上述移动保守区域。保守区域可能排除实际可行的行走；不要为了求解成功而将它扩大到真实 contact footprint 之外。

初始 QP 检查每个区间起点、中点、终点的 ZMP。`validate_plan()` 额外求解 `ZMP − moving_support_center` 导数的实根，检查内部极值及速度极值。违规超过 SI 数值容差 `2e-5` 时添加约束，最多求解八轮；对位置而言为 20 μm。不能只凭 `solved` 状态认定成功。

## W2 着陆区域练习

在仓库根目录运行。首次着陆 x 限制为 0.12–0.125 m，默认固定落脚点 0.1 m 无法满足。

```python
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path('lab-7-walking-optimization/src').resolve()))
from walking_model import WalkConfig, build_schedule
from lipm_optimizer import solve_walk, validate_plan
from walking_artifacts import export_plan
schedule = build_schedule(WalkConfig())
# 维度：(不含最初两脚的着陆数, x/y, lower/upper)。
bounds = np.tile([-np.inf, np.inf], (len(schedule.feet) - 2, 2, 1))
bounds[0, 0] = [0.12, 0.125]
plan = solve_walk(schedule, optimize_feet=True, landing_bounds=bounds)
print(plan.feet[2], validate_plan(plan))
export_plan(plan, Path('/tmp/landing-box'))
```

单个 convex box 不能表示空洞、不相连踏脚石的选择或非凸避障区域。步长/步宽限制不能替代 Whole-body IK 与 self-collision 检查。

## G1 adapter

1. `g1_walk.run()` 调用 `m3_walking.build()`，稳定 torque 模型与 controller。
2. 使用实测足位、CoM、速度规划。CoM 高度以 MuJoCo 地面为基准，而非 ankle frame；简化模型初始加速度假定为零。
3. `OptimizedGait` 继承 `GaitSchedule`，修改时间表与足目标。Adapter 测试检查 `_foot_state`/`_swing_of_phase` 的着陆和接触约定。
4. `OptimizedReference` 将计划 `xi`、`xi_dot`、ZMP 传入原有 `DCMReference`。
5. Lab 6 用 Pinocchio dynamics 求解 contact wrench 与 torque，MuJoCo 执行并测量接触。Telemetry 和 Pinocchio 更新对应同一 post-step 状态。

Swing 复用 Lab 6 的水平 raised-cosine 与竖直 sine 轨迹；竖直 sine 端点速度不为零，不能视为消除落地冲击的最优 swing。Controller 根据实际足位 clamp ZMP，因此计划和命令 ZMP 可能不同。请对照 `telemetry.csv` 与实测接触。Lab 9 在固定接触顺序下重规划状态和 wrench，不自动选择接触序列。
