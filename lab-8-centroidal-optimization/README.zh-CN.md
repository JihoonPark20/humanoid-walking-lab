# Lab 8: Centroidal 优化

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

使用 nonlinear multiple shooting 优化 CoM 高度、动量和 contact wrench。

![Demo](media/offline/centroidal.png)

简化模型的 CoM 高度、动量及接触力优化；不是机器人 rollout。

[实验与测量](GALLERY.md) · [学习笔记](docs/README.md)

## 运行

[安装](../docs/SETUP.md)

```bash
python lab-8-centroidal-optimization/src/offline_centroidal.py
```

简化模型使用平地固定接触；全身可行性需单独验证。

```bash
python -m pytest lab-8-centroidal-optimization/tests -q
```

[← Curriculum](../README.zh-CN.md)
