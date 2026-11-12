# Lab 7: Walking Trajectory Optimization

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Optimize offline LIPM trajectories, footstep locations and timing candidates; track them on G1.

![Demo](media/gallery/hero.gif)

Optimized feet + CoM tracked in MuJoCo: 5/5 swings; final CoM error 15.15 mm.

[Exercises and measurements](GALLERY.md) · [Study notes](docs/README.md)

## Run

[Setup](../docs/SETUP.md)

```bash
python lab-7-walking-optimization/src/offline_walk.py --compare
python lab-7-walking-optimization/src/g1_walk.py --mode optimized --optimize-feet --video
```

Offline footstep/timing optimization uses a fixed contact sequence.

```bash
python -m pytest lab-7-walking-optimization/tests -q
```

[← Curriculum](../README.md)
