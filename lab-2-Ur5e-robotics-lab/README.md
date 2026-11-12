# Lab 2: UR5e Kinematics

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Extend kinematics and trajectory tracking to a 6-DOF UR5e.

![Demo](media/gallery/hero.gif)

Complete 3D cube tracking at 3×: RMS error 0.088 mm; measurements from the recorded run.

[Exercises and measurements](GALLERY.md) · [Study notes](docs/README.md)

## Run

[Setup](../docs/SETUP.md)

```bash
python lab-2-Ur5e-robotics-lab/src/c3_record_video.py
```

```bash
python -m pytest lab-2-Ur5e-robotics-lab/tests -q
```

[← Curriculum](../README.md)
