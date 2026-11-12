# Lab 4: Motion Planning

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Plan collision-free motion with RRT*, then parameterize and track the trajectory.

![Demo](media/gallery/hero.gif)

3D slalom at 2×: zero contacts, minimum obstacle clearance 30.35 mm. Collision shapes and visible meshes checked every 1 ms.

[Exercises and measurements](GALLERY.md) · [Study notes](docs/README.md)

## Run

[Setup](../docs/SETUP.md)

```bash
python lab-4-motion-planning/src/capstone_demo.py
```

```bash
python -m pytest lab-4-motion-planning/tests -q
```

[← Curriculum](../README.md)
