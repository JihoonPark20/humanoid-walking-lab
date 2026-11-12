# Lab 5：学习笔记

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

[运行命令与代码顺序](../README.zh-CN.md) · [共同概念](../../docs/CONCEPTS.zh-CN.md)

## Floating base 增加了什么

G1 的 base 不固定于 world。Base pose 包含三个位置量和四个 quaternion 分量；base velocity 包含三个线速度和三个角速度。因此 `nq=36、nv=35`，共有 29 个 actuator。Base 的六个速度自由度没有直接电机。

MuJoCo quaternion 顺序为 `(w,x,y,z)`，Pinocchio 为 `(x,y,z,w)`。使用 `lab5_common.py` 的状态转换，不要用普通 `q+dq` 更新 quaternion。

## CoM、support polygon 与 ZMP

静态姿态可通过 CoM 的地面投影和支撑多边形判断。动态运动还依赖速度、加速度和接触力。ZMP 是倾覆力矩为零的地面点，不一定等于 CoM 投影。

在 CoM 高度固定时，LIPM 将水平运动简化为：

```text
cddot = ω²(c − p),  ω = sqrt(g / z_com)
c: 水平 CoM；p: ZMP
```

`lipm_planner.py` 与 `lipm_preview_control.py` 是规划模型。简化模型曲线稳定不能证明全身 MuJoCo 动作可行。

## Whole-body IK 与控制

`whole_body_ik.py` 同时匹配双脚 pose、CoM 与 pelvis 条件。IK 误差小不代表 actuator 与 contact 能实现动作。转移重心时，还需测量足端漂移和 pelvis 高度。

本 Lab 在双脚支撑下学习平衡与重心转移。Lab 6 通过 contact wrench 和 torque 控制进入动态行走。

## 练习

分别改变侧向 CoM 位移和过渡时间，比较足端滑移、pelvis 高度和跟踪误差。发生跌倒时，找出最早的 torque saturation 或接触变化，不要只看最后一帧。

[G1 关节表](g1_joint_map.zh-CN.md)
