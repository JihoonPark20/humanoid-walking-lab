# Lab 6: 全身行走控制

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

使用 inverse-dynamics QP、contact wrench 和 DCM 跟踪控制 G1 行走。

![Demo](media/gallery/hero.gif)

足端特写、0.65 倍速：观察承重支撑区从双脚到单脚再回到双脚的变化，比较紫色 DCM 与橙色接触 ZMP。

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
