# Lab 6: Whole-Body Walking Control

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Control G1 walking with inverse-dynamics QP, contact wrenches and DCM tracking.

![Demo](media/gallery/hero.gif)

Foot-level view, 0.65×: watch the loaded support hull change from two feet to one and back. Compare violet DCM with amber contact ZMP.

[Demo](GALLERY.md) · [Study notes](docs/README.md) · [Visual guide](../docs/VISUAL_GUIDE.md)

## Run

[Setup](../docs/SETUP.md)

```bash
python lab-6-whole-body-control/src/m3_walking.py
```

```bash
python -m pytest lab-6-whole-body-control/tests -q
```

[← Curriculum](../README.md)
