# 实验解读

[English](VALIDATION.md) · [한국어](VALIDATION.ko.md) · [简体中文](VALIDATION.zh-CN.md)

学习演示时，区分三个问题：

1. **计划是否可行？** 检查动力学、接触约束和碰撞间隙，仅有 solver 收敛还不够。
2. **机器人能否跟踪？** 比较参考与实际运动、承重接触、关节和力矩限位。Reduced 模型中的可行计划，全身机器人也可能难以跟踪。
3. **比较是否公平？** 比较一个设计因素时，固定环境、初始状态、外力和控制器设置，同时保留失败尝试。

各 Lab 的学习笔记说明模型假设与比较练习。需要深入分析动作或图表时，可展开演示页中的原始数据。先用 README 命令复现，再一次改变一个参数。

## 演示的假设

- Lab 1–2 学习机械臂跟踪与 kinematics。Lab 1 的正方形练习禁用碰撞；Lab 2 的 frame 距离启发式不是 mesh 碰撞检测。Lab 3 使用低摩擦平板和刚性探针。
- Lab 4 使用采样碰撞检查。搜索失败或迭代预算耗尽不证明路径不存在，应分别检查返回路径和实际跟踪。
- Lab 8 是 reduced centroidal 模型动画。Lab 9 预测 CoM、动量与接触力，其观测器不是 IMU 估计器。Lab 10 通过反馈跟踪离线全身计划。
- Lab 11–13 使用平地和仿真器状态。Lab 13 固定接触顺序与时间，只调整落脚点。除完成情况外，还要比较物理约束；自适应落脚不一定改善每项指标。

## 修改代码后的检查

修改实现后，运行相应 Lab 的测试。完整检查使用两个环境：

```bash
.venv/bin/python -m pytest lab-*/tests tools/tests -q
.venv-ocp/bin/python -m pytest lab-10-whole-body-optimal-control/tests -q
```

主环境跳过 Crocoddyl，第二条命令覆盖该部分。修改控制器后，还应运行相应 Lab 的仿真命令观察运动。执行时间受机器负载影响，不代表 hard real-time 保证。

[课程](../README.zh-CN.md) · [环境设置](SETUP.zh-CN.md)
