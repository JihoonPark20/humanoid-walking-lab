# A3: Jacobian — Study Notes

[English](a3_jacobian.md) · [한국어](a3_jacobian.ko.md) · [简体中文](a3_jacobian.zh-CN.md)

Jacobian 是 FK 对关节角的导数，给出局部速度关系 `x_dot=J(q) q_dot`。二连杆 J 为 2×2，`L2_eff=L2+0.015` m。

```text
J = [[-L1 sin(q1)-L2_eff sin(q1+q2), -L2_eff sin(q1+q2)],
     [ L1 cos(q1)+L2_eff cos(q1+q2),  L2_eff cos(q1+q2)]]
det(J) = L1 L2_eff sin(q2)
```

当 `q2≈0,π` 时 determinant 趋于零，失去某些末端运动方向；很小的 Cartesian 命令也可能要求很大的关节速度。

`a3_jacobian.py` 比较解析式、finite difference 和 MuJoCo `mj_jacSite`，将 determinant sweep 保存为 `a3_det_sweep.csv`。必须对齐 frame 与 site。不依赖外部库的 fallback 不能替代 MuJoCo 交叉验证。

练习：改变 finite-difference 步长，比较截断和舍入误差；接近 singularity 时查看 J 的 singular value。

[学习笔记](README.zh-CN.md) · [运行指南](../README.zh-CN.md)
