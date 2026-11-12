# Lab 5: 平衡与重心转移

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

学习 floating-base 状态、CoM、Whole-body IK 和双脚接触平衡。

![Demo](media/gallery/hero.gif)

保持双脚接触的准静态重心转移。

[实验与测量](GALLERY.md) · [学习笔记](docs/README.md)

## 运行

[安装](../docs/SETUP.md)

```bash
python lab-5-locomotion/src/m3d_weight_shift.py
```

本 Lab 研究准静态平衡；基于 torque 的动态行走见 Lab 6。

```bash
python -m pytest lab-5-locomotion/tests -q
```

[← Curriculum](../README.zh-CN.md)
