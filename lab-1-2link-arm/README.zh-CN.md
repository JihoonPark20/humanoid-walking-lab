# Lab 1: 二连杆平面机械臂

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

通过平面机械臂学习 FK、Jacobian、Inverse Kinematics 和 PD 控制。

![Demo](media/gallery/hero.gif)

完整正方形跟踪：RMS 误差 0.008 mm；指标来自录像对应的运行。

[实验与测量](GALLERY.md) · [学习笔记](docs/README.md)

## 运行

[安装](../docs/SETUP.md)

```bash
mjpython lab-1-2link-arm/src/c1_draw_square.py
```

```bash
python -m pytest lab-1-2link-arm/tests -q
```

[← Curriculum](../README.zh-CN.md)
