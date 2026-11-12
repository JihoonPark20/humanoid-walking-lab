# Lab 3: 动力学与力控制

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

学习 RNEA、CRBA、重力补偿、impedance 和接触力控制。

![Demo](media/gallery/hero.gif)

闭合夹爪与刚性探针在平板（μ=0.05）上以 5 N 描绘 50 mm。完整过程、1 倍速；原始接触力合格率 93.0%。

[实验与测量](GALLERY.md) · [学习笔记](docs/README.md)

## 运行

[安装](../docs/SETUP.md)

```bash
python lab-3-dynamics-force-control/src/record_lab3_demo.py
```

```bash
python -m pytest lab-3-dynamics-force-control/tests -q
```

[← Curriculum](../README.zh-CN.md)
