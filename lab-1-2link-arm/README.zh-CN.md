# Lab 1: 二连杆平面机械臂

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

通过平面机械臂学习 FK、Jacobian、Inverse Kinematics 和 PD 控制。

![Demo](media/gallery/hero.gif)

观察关节运动如何形成末端的正方形路径，尤其注意转角处的运动。

[演示](GALLERY.zh-CN.md) · [学习笔记](docs/README.zh-CN.md)

## 运行

[安装](../docs/SETUP.zh-CN.md)

```bash
mjpython lab-1-2link-arm/src/c1_draw_square.py
```

```bash
python -m pytest lab-1-2link-arm/tests -q
```

[← Curriculum](../README.zh-CN.md)
