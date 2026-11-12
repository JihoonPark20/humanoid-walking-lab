# Lab 9: MPC, RTI & Observation Robustness

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Replan with full-NLP MPC or single-QP RTI; evaluate delay, model mismatch and noisy observations.

![Demo](media/gallery/hero.gif)

Violet spheres and the path show the current CoM prediction; gray is the nominal plan. Watch how the prediction changes around the red push arrow.

[Demo](GALLERY.md) · [Study notes](docs/README.md) · [Visual guide](../docs/VISUAL_GUIDE.md)

## Run

[Setup](../docs/SETUP.md)

```bash
python lab-9-model-predictive-control/src/g1_mpc.py --mode rti --video
python lab-9-model-predictive-control/src/observation_benchmark.py
```

Validated on flat ground with fixed footsteps. The packet observer uses reduced centroidal states, not an IMU estimator. Prolonged sensor blackouts are outside the supported envelope.

[Observation model](docs/OBSERVATION_ROBUSTNESS.md)

```bash
python -m pytest lab-9-model-predictive-control/tests -q
```

[← Curriculum](../README.md)
