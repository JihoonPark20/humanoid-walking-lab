# A4: Inverse Kinematics

[English](a4_inverse_kinematics.md) · [한국어](a4_inverse_kinematics.ko.md) · [简体中文](a4_inverse_kinematics.zh-CN.md)

UR5e 数值 IK 同时匹配位置与 orientation。`IKResult` 保存 success、q、iterations、最终/位置/旋转误差和 elapsed_s。

```text
e = [p_target-p_current; log3(R_target R_current.T)]
dq_pinv = pinv(J) e
dq_DLS = J.T (J J.T + λ² I)^(-1) e
```

J 使用 `LOCAL_WORLD_ALIGNED`，每轮 clamp 到 joint 范围。默认 α=0.5、tol=1e-4；pinv 最多 200 次，DLS 最多 300 次。误差下降时减小 λ，上升时增大，并限制在 `[1e-4,1]`。这只是 damping 调节，不是全局收敛保证。

`validate_ik_solution()` 用 FK roundtrip 检查默认位置 1 mm、旋转 0.01 rad 容差。在 MuJoCo 设置同一 q，调用 `mj_forward` 后比较 site。即使目标由随机 q 的 FK 生成而确实可达，另一初值的局部 solver 仍可能失败。

在 Lab 目录运行 `python src/a4_inverse_kinematics.py`，通过 `a4_ik_benchmark.csv` 比较 50 目标的成功率、迭代数、时间和误差。IK 成功不等于无碰撞或动力学可执行。

[学习笔记](README.zh-CN.md) · [运行指南](../README.zh-CN.md)
