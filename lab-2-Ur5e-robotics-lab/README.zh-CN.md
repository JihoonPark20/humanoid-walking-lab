# Lab 2: UR5e 运动学

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

将运动学和轨迹跟踪扩展到 6-DOF UR5e。

![Demo](media/gallery/hero.gif)

完整三维立方体跟踪、3 倍速：RMS 误差 0.088 mm；指标来自录像对应的运行。

[实验与测量](GALLERY.md) · [学习笔记](docs/README.md)

## 运行

[安装](../docs/SETUP.md)

```bash
python lab-2-Ur5e-robotics-lab/src/c3_record_video.py
```

```bash
python -m pytest lab-2-Ur5e-robotics-lab/tests -q
```

[← Curriculum](../README.zh-CN.md)
