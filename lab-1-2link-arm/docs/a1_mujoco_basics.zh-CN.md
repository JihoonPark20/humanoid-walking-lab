# A1: MuJoCo Environment Setup — Study Notes

[English](a1_mujoco_basics.md) · [한국어](a1_mujoco_basics.ko.md) · [简体中文](a1_mujoco_basics.zh-CN.md)

MuJoCo 是计算机器人动力学与接触的物理引擎。MJCF 的 `option` 定义重力/时间步，`worldbody` 定义 body 层级，`actuator` 定义驱动器，`sensor` 定义测量。Body 表示坐标系和惯量，joint 定义父子间自由度，geom 定义显示/碰撞形状。Site 仅标记参考点，本身没有物理作用。

模型是 XY 平面二连杆：两段 0.3 m link、两个 z 轴 hinge、两个 torque motor。末端计算还包含 0.015 m tool offset。父 joint 旋转会带动下级 link。

`MjModel` 保存参数，`MjData` 保存状态。读取 `qpos`/`qvel`，通过 `ctrl` 输入；`site_xpos` 是 world-frame 末端坐标。`mj_step` 推进时间，`mj_forward` 只更新当前状态的派生量。

```python
import mujoco
model = mujoco.MjModel.from_xml_path('models/two_link.xml')
data = mujoco.MjData(model)
data.ctrl[:] = [5, -3]
mujoco.mj_step(model, data)
mujoco.mj_forward(model, data)
print(data.qpos, data.site_xpos)
```

在 Lab 目录运行。检查状态维度、时间增长、输入引起的角度变化和 site 位置，然后与 A2 手算结果比较。

[学习笔记](README.zh-CN.md) · [运行指南](../README.zh-CN.md)
