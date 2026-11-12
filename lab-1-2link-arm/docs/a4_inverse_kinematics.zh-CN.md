# A4: Inverse Kinematics — Study Notes

[English](a4_inverse_kinematics.md) · [한국어](a4_inverse_kinematics.ko.md) · [简体中文](a4_inverse_kinematics.zh-CN.md)

IK 求满足目标 `(x,y)` 的 `(θ1,θ2)`。包含 site offset 后，`L1=0.3`、`L2_eff=0.315` m，可达半径为 0.015–0.615 m。

```text
cos(θ2) = (x²+y²-L1²-L2_eff²)/(2 L1 L2_eff)
θ2 = ±acos(cos(θ2))
θ1 = atan2(y,x)-atan2(L2_eff sin(θ2), L1+L2_eff cos(θ2))
dq_DLS = J.T (J J.T + λ² I)^(-1) e
```

± 分支对应不同 elbow 姿态，应检查实际 q 和 FK，而非只看名称。选择最接近上一步的分支以保持连续。数值 `pinv` 对 singularity 敏感；DLS 用 damping 抑制过大更新，但也不保证对所有目标收敛。

`a4_inverse_kinematics.py` 包含解析解 FK roundtrip、20 目标 benchmark、singularity stress 和可选 MuJoCo site 验证，输出 `a4_ik_benchmark.csv`。不可达目标应返回失败，同时比较成功状态、迭代次数与最终误差。

[学习笔记](README.zh-CN.md) · [运行指南](../README.zh-CN.md)
