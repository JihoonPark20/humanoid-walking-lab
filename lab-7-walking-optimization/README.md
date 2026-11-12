# Lab 7: Walking Trajectory Optimization

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Optimize offline LIPM trajectories, footstep locations and timing candidates; track them on G1.

![Demo](media/gallery/hero.gif)

Close view, 0.7×: compare gray seed footprints with violet optimized footsteps and CoM. Cyan is measured motion; the plan stays fixed during execution.

[Demo](GALLERY.md) · [Study notes](docs/README.md) · [Visual guide](../docs/VISUAL_GUIDE.md)

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
