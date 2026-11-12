# Lab 3: Dynamics & Force Control

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Learn RNEA, CRBA, gravity compensation, impedance and contact force control.

![Demo](media/gallery/hero.gif)

Closed gripper with a rigid probe: 5 N force control over a 50 mm trace on a flat plate (μ=0.05). Full run at 1×; raw-force in-band 93.0%.

[Exercises and measurements](GALLERY.md) · [Study notes](docs/README.md)

## Run

[Setup](../docs/SETUP.md)

```bash
python lab-3-dynamics-force-control/src/record_lab3_demo.py
```

```bash
python -m pytest lab-3-dynamics-force-control/tests -q
```

[← Curriculum](../README.md)
