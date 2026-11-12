# Humanoid Walking Lab

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

使用 MuJoCo 学习机器人基础、平衡控制与基于模型的人形行走。

![G1 walking](lab-7-walking-optimization/media/gallery/hero.gif)

## 课程

Pinocchio 计算运动学与动力学，MuJoCo 模拟接触与运动。按 Lab 1–10 的顺序学习。

| Lab | 课程 |
|---|---|
| 1 | [二连杆平面机械臂](lab-1-2link-arm/README.zh-CN.md) |
| 2 | [UR5e 运动学](lab-2-Ur5e-robotics-lab/README.zh-CN.md) |
| 3 | [动力学与力控制](lab-3-dynamics-force-control/README.zh-CN.md) |
| 4 | [运动规划](lab-4-motion-planning/README.zh-CN.md) |
| 5 | [平衡与重心转移](lab-5-locomotion/README.zh-CN.md) |
| 6 | [全身行走控制](lab-6-whole-body-control/README.zh-CN.md) |
| 7 | [行走轨迹优化](lab-7-walking-optimization/README.zh-CN.md) |
| 8 | [Centroidal 优化](lab-8-centroidal-optimization/README.zh-CN.md) |
| 9 | [MPC、RTI 与观测鲁棒性](lab-9-model-predictive-control/README.zh-CN.md) |
| 10 | [全身最优控制](lab-10-whole-body-optimal-control/README.zh-CN.md) |

## 快速开始

Python 3.11+ · CPU · 已在 Apple Silicon 验证。macOS 交互 viewer 使用 `mjpython`；Lab 10 需要独立环境。

```bash
python3 -m venv .venv
source .venv/bin/activate
bash tools/setup_env.sh
python lab-9-model-predictive-control/src/g1_mpc.py --mode rti --video
```

已验证平地配置：**MPC/RTI 16 个 · 观测实验 24 个 · 全身跟踪 12 个**。

[可视化指南](docs/VISUAL_GUIDE.zh-CN.md) · [安装](docs/SETUP.md) · [验证](docs/VALIDATION.md) · [通用概念](docs/CONCEPTS.md)

基于 [M. Ozkan Ceylan 的 robotics labs](https://github.com/ozkannceylan/mujoco-robotics-lab)。 [Apache-2.0](LICENSE) · [NOTICE](NOTICE) · [Model licenses](THIRD_PARTY_NOTICES.md)
