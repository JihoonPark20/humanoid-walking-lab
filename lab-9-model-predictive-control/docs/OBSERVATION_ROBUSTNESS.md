# Observation model

[English](OBSERVATION_ROBUSTNESS.md) · [한국어](OBSERVATION_ROBUSTNESS.ko.md) · [简体中文](OBSERVATION_ROBUSTNESS.zh-CN.md)

The reduced centroidal sensor supplies timestamped `[c, p, L]` packets. It is not an IMU/contact estimator. Only past commanded wrenches enter prediction. The reset state and nominal mass are known; the ideal-data mass observer is disabled.

Each axis has Gaussian noise: 0.5 mm CoM, 0.01 kg m/s linear momentum and 0.002 kg m²/s angular momentum. Delay is one sample on an 83.3–100 ms grid. Evaluate no delay, one-sample delay, 15% random loss and a four-sample burst. Long blackouts are outside the supported range.

At each new packet time, predict the prior using past controls, apply `posterior = prior + gain * (measurement - prior)`, then predict to now. Gain 1 is raw-packet prediction; 0.65 filters the innovation. Repeated packets are ignored. This is a fixed-gain observer, not an EKF.

All 24 paired runs (seeds 0, 7, 23) complete 60 intervals over 5.7 s. The lateral impulse is requested at 2.1 s and applied at 2.15 s. Noise-only CoM estimate RMS averages 0.807 mm raw / 0.560 mm filtered; lower gain trades disturbance response for noise rejection.

![CoM estimation](../media/observation_benchmark/noise_delay/seed_7/observations.png)

[Measurements](../media/observation_benchmark/matrix.json) · [Implementation](../src/centroidal_sensing.py)

```bash
python lab-9-model-predictive-control/src/observation_benchmark.py
```
