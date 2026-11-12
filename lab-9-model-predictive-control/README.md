# Lab 9: MPC, RTI & Observation Robustness

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Replan with full-NLP MPC or single-QP RTI; evaluate delay, model mismatch and noisy observations.

![Demo](media/gallery/hero.gif)

G1 RTI with a 5 N lateral push: 6.7 s completed, final CoM error 0.922 mm.

[Exercises and measurements](GALLERY.md) · [Study notes](docs/README.md)

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
