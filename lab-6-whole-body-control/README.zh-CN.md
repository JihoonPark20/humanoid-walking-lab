# Lab 6: 全身行走控制

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

使用 inverse-dynamics QP、contact wrench 和 DCM 跟踪控制 G1 行走。

![Demo](media/gallery/hero.gif)

观察单脚与双脚支撑区域的切换，以及 DCM（紫色）和接触 ZMP（橙色）的变化（1 倍速）。

[演示](GALLERY.zh-CN.md) · [学习笔记](docs/README.zh-CN.md) · [可视化指南](../docs/VISUAL_GUIDE.zh-CN.md)

## 运行

[安装](../docs/SETUP.zh-CN.md)

```bash
python lab-6-whole-body-control/src/m3_walking.py
```

```bash
python -m pytest lab-6-whole-body-control/tests -q
```

[← Curriculum](../README.zh-CN.md)
