# Lab 5: Balance & Weight Shifting

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Learn floating-base states, CoM, Whole-body IK and balance with both feet in contact.

![Demo](media/gallery/hero.gif)

Watch the cyan CoM ring move toward the white target inside green support; amber marks contact ZMP. CoM shift 53.6 mm, foot drift ≤1.39 mm.

[Exercises and measurements](GALLERY.md) · [Study notes](docs/README.md) · [Visual guide](../docs/VISUAL_GUIDE.md)

## Run

[Setup](../docs/SETUP.md)

```bash
python lab-5-locomotion/src/m3d_weight_shift.py
```

This lab covers quasi-static balance; torque-level walking follows in Lab 6.

```bash
python -m pytest lab-5-locomotion/tests -q
```

[← Curriculum](../README.md)
