# Lab 4: Motion Planning

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Plan collision-free motion with RRT*, then parameterize and track the trajectory.

![Demo](media/gallery/hero.gif)

RRT* finds a detour around a post blocking a required direct segment. Compare the planned path with the robot motion (2× playback).

[Demo](GALLERY.md) · [Study notes](docs/README.md)

## Run

[Setup](../docs/SETUP.md)

[Multi-environment evaluation](docs/GENERALIZATION.md)

```bash
python lab-4-motion-planning/src/motion_planning.py lab-4-motion-planning/scenarios/robot_101.json --seed 0 --execute
python lab-4-motion-planning/src/benchmark_planning.py
python lab-4-motion-planning/src/capstone_demo.py
```

```bash
python -m pytest lab-4-motion-planning/tests -q
```

[← Curriculum](../README.md)
