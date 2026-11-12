# Lab 9: MPC、RTI 与观测鲁棒性

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

使用 full-NLP MPC 或 single-QP RTI 重规划，评估延迟、模型误差和观测噪声。

![Demo](media/gallery/hero.gif)

紫色球体与轨迹为当前 CoM 预测，灰色为原始计划，红色箭头为 5 N 扰动。最终 CoM 误差 0.922 mm。

[实验与测量](GALLERY.md) · [学习笔记](docs/README.md) · [可视化指南](../docs/VISUAL_GUIDE.zh-CN.md)

## 运行

[安装](../docs/SETUP.md)

```bash
python lab-9-model-predictive-control/src/g1_mpc.py --mode rti --video
python lab-9-model-predictive-control/src/observation_benchmark.py
```

已验证平地固定落脚点条件。数据包估计使用简化 centroidal 状态，并非 IMU 估计器；长时间观测中断不在支持范围内。

[观测模型](docs/OBSERVATION_ROBUSTNESS.zh-CN.md)

```bash
python -m pytest lab-9-model-predictive-control/tests -q
```

[← Curriculum](../README.zh-CN.md)
