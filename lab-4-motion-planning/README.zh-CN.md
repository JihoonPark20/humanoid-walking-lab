# Lab 4: 运动规划

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

用 RRT* 规划无碰撞路径，再进行时间参数化和轨迹跟踪。

![Demo](media/gallery/hero.gif)

RRT* 绕过阻挡必经直线段的柱体。比较规划路径与机器人的实际运动（2 倍速）。

[演示](GALLERY.zh-CN.md) · [学习笔记](docs/README.zh-CN.md)

## 运行

[安装](../docs/SETUP.zh-CN.md)

[多环境评估](docs/GENERALIZATION.zh-CN.md)

```bash
python lab-4-motion-planning/src/motion_planning.py lab-4-motion-planning/scenarios/robot_101.json --seed 0 --execute
python lab-4-motion-planning/src/benchmark_planning.py
python lab-4-motion-planning/src/capstone_demo.py
```

```bash
python -m pytest lab-4-motion-planning/tests -q
```

[← Curriculum](../README.zh-CN.md)
