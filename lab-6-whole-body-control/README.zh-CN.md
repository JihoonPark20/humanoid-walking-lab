# Lab 6: 全身行走控制

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

使用 inverse-dynamics QP、contact wrench 和 DCM 跟踪控制 G1 行走。

![Demo](media/gallery/hero.gif)

DCM 行走：12 步、1.182 m；1.5 倍速。

[实验与测量](GALLERY.md) · [学习笔记](docs/README.md)

## 运行

[安装](../docs/SETUP.md)

```bash
python lab-6-whole-body-control/src/m3_walking.py
```

```bash
python -m pytest lab-6-whole-body-control/tests -q
```

[← Curriculum](../README.zh-CN.md)
