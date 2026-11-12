# Lab 7: 行走轨迹优化

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

优化离线 LIPM 轨迹、落脚点与时序候选，并用 G1 跟踪。

![Demo](media/gallery/hero.gif)

优化落脚点与 CoM 的 MuJoCo 跟踪：5/5 次摆动，最终 CoM 误差 15.15 mm。

[实验与测量](GALLERY.md) · [学习笔记](docs/README.md)

## 运行

[安装](../docs/SETUP.md)

```bash
python lab-7-walking-optimization/src/offline_walk.py --compare
python lab-7-walking-optimization/src/g1_walk.py --mode optimized --optimize-feet --video
```

离线落脚点/时序优化使用固定接触顺序。

```bash
python -m pytest lab-7-walking-optimization/tests -q
```

[← Curriculum](../README.zh-CN.md)
