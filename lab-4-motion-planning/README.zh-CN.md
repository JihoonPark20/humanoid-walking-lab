# Lab 4: 运动规划

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

用 RRT* 规划无碰撞路径，再进行时间参数化和轨迹跟踪。

![Demo](media/gallery/hero.gif)

三维绕障、2 倍速：零接触，最小障碍物间隙 30.35 mm。每 1 ms 检查碰撞形状与可见 mesh。

[实验与测量](GALLERY.md) · [学习笔记](docs/README.md)

## 运行

[安装](../docs/SETUP.md)

```bash
python lab-4-motion-planning/src/capstone_demo.py
```

```bash
python -m pytest lab-4-motion-planning/tests -q
```

[← Curriculum](../README.zh-CN.md)
