# A1: Environment Setup

[English](a1_environment_setup.md) · [한국어](a1_environment_setup.ko.md) · [简体中文](a1_environment_setup.zh-CN.md)

`a1_model_setup.py` 检查 MuJoCo MJCF 与 Pinocchio URDF 是否表示同一 UR5e。`ur5e_common.py` 集中路径、joint 名称/限制、Q_HOME、DH 参数；`mujoco_sim.py` 的 `UR5eSimulator` 统一状态、torque 和末端访问。

依赖 NumPy、MuJoCo、Pinocchio、Matplotlib。用仓库 setup script 安装模型，并保留 `models/lab_scene.xml`、`models/ur5e.urdf` 和 Menagerie mesh 链接。

```bash
python src/a1_model_setup.py
```

在 Lab 目录检查模型维度、joint 列表及 Q_HOME 下的 step。对比同一 q 的 MuJoCo `attachment_site` 与 Pinocchio `ee_link`，要求位置差在 1 mm 内。输出 `a1_environment_status.csv`。若 frame 对应错误，后续 IK/Jacobian/控制比较也不可信。

[学习笔记](README.zh-CN.md) · [运行指南](../README.zh-CN.md)
