# Lab 5: Balance & Weight Shifting

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Learn floating-base states, CoM, Whole-body IK and balance with both feet in contact.

![Demo](media/gallery/hero.gif)

Close view, 0.65×: the cyan CoM ring moves toward the white target while amber contact ZMP responds inside green support.

[Demo](GALLERY.md) · [Study notes](docs/README.md) · [Visual guide](../docs/VISUAL_GUIDE.md)

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
