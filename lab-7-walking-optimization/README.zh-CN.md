# Lab 7: 行走轨迹优化

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

优化离线 LIPM 轨迹、落脚点与时序候选，并用 G1 跟踪。

![Demo](media/gallery/hero.gif)

比较初始落脚点（灰色）、优化落脚点与 CoM（紫色）、实际 CoM（青色）。执行期间计划固定，完成 5/5 次摆动。

[实验与测量](GALLERY.md) · [学习笔记](docs/README.md) · [可视化指南](../docs/VISUAL_GUIDE.zh-CN.md)

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
