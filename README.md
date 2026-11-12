# Humanoid Walking Lab

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Robotics fundamentals, balance and model-based humanoid walking in MuJoCo.

![G1 walking](lab-7-walking-optimization/media/gallery/hero.gif)

## Curriculum

Pinocchio computes kinematics and dynamics; MuJoCo simulates contacts and motion. Follow Labs 1–13 in order.

| Lab | Curriculum |
|---|---|
| 1 | [2-Link Planar Arm](lab-1-2link-arm/README.md) |
| 2 | [UR5e Kinematics](lab-2-Ur5e-robotics-lab/README.md) |
| 3 | [Dynamics & Force Control](lab-3-dynamics-force-control/README.md) |
| 4 | [Motion Planning](lab-4-motion-planning/README.md) |
| 5 | [Balance & Weight Shifting](lab-5-locomotion/README.md) |
| 6 | [Whole-Body Walking Control](lab-6-whole-body-control/README.md) |
| 7 | [Walking Trajectory Optimization](lab-7-walking-optimization/README.md) |
| 8 | [Centroidal Optimization](lab-8-centroidal-optimization/README.md) |
| 9 | [MPC, RTI & Observation Robustness](lab-9-model-predictive-control/README.md) |
| 10 | [Whole-Body Optimal Control](lab-10-whole-body-optimal-control/README.md) |
| 11 | [Contact-aware Walking](lab-11-contact-transitions/README.md) |
| 12 | [Physical Walking Robustness](lab-12-walking-robustness/README.md) |
| 13 | [Adaptive Foot Placement](lab-13-adaptive-footsteps/README.md) |

## Quick start

Python 3.11+ · CPU · Apple Silicon tested. Use `mjpython` for interactive macOS viewers. Lab 10 requires a separate environment.

```bash
python3 -m venv .venv
source .venv/bin/activate
bash tools/setup_env.sh
python lab-9-model-predictive-control/src/g1_mpc.py --mode rti --video
```

[Visual guide](docs/VISUAL_GUIDE.md) · [Setup](docs/SETUP.md) · [Interpreting experiments](docs/VALIDATION.md) · [Study concepts](docs/CONCEPTS.md)

Based on [M. Ozkan Ceylan’s robotics labs](https://github.com/ozkannceylan/mujoco-robotics-lab). [Apache-2.0](LICENSE) · [NOTICE](NOTICE) · [Model licenses](THIRD_PARTY_NOTICES.md)
