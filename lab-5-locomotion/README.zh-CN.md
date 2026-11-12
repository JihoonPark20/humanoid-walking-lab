# Lab 5: 平衡与重心转移

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

学习 floating-base 状态、CoM、Whole-body IK 和双脚接触平衡。

![Demo](media/gallery/hero.gif)

放大、0.65 倍速：观察青色 CoM 圆环移向白色目标时，绿色支撑区内橙色接触 ZMP 的响应。

[演示](GALLERY.zh-CN.md) · [学习笔记](docs/README.zh-CN.md) · [可视化指南](../docs/VISUAL_GUIDE.zh-CN.md)

## 运行

[安装](../docs/SETUP.zh-CN.md)

```bash
python lab-5-locomotion/src/m3d_weight_shift.py
```

本 Lab 研究准静态平衡；基于 torque 的动态行走见 Lab 6。

```bash
python -m pytest lab-5-locomotion/tests -q
```

[← Curriculum](../README.zh-CN.md)
