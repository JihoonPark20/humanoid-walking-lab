# Lab 6: Whole-Body Walking Control

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Control G1 walking with inverse-dynamics QP, contact wrenches and DCM tracking.

![Demo](media/gallery/hero.gif)

Watch the support hull switch between feet while DCM (violet) and contact ZMP (amber) evolve. 12 steps, 1.182 m; full run at 1×.

[Exercises and measurements](GALLERY.md) · [Study notes](docs/README.md) · [Visual guide](../docs/VISUAL_GUIDE.md)

## Run

[Setup](../docs/SETUP.md)

```bash
python lab-6-whole-body-control/src/m3_walking.py
```

```bash
python -m pytest lab-6-whole-body-control/tests -q
```

[← Curriculum](../README.md)
