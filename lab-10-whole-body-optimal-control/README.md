# Lab 10: Whole-Body Optimal Control

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Optimize contact/impact dynamics with Box-FDDP and track the plan in MuJoCo.

![Demo](media/gallery/hero.gif)

Close view, 0.65×: translucent robots show offline joint poses 0.25/0.50 s ahead. The solid robot tracks this plan with FDDP feedback.

[Demo](GALLERY.md) · [Study notes](docs/README.md) · [Visual guide](../docs/VISUAL_GUIDE.md)

## Run

[Setup](../docs/SETUP.md)

```bash
.venv-ocp/bin/python lab-10-whole-body-optimal-control/src/whole_body_demo.py --steps 2 --mujoco --video
```

Offline OCP with feedback, not whole-body MPC. Use the separate OCP environment and the recommended planning costs.

```bash
.venv-ocp/bin/python -m pytest lab-10-whole-body-optimal-control/tests -q
```

[← Curriculum](../README.md)
