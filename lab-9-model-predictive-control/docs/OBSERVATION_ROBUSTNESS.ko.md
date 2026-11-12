# 관측 모델

[English](OBSERVATION_ROBUSTNESS.md) · [한국어](OBSERVATION_ROBUSTNESS.ko.md) · [简体中文](OBSERVATION_ROBUSTNESS.zh-CN.md)

축소 centroidal 센서는 timestamp를 포함한 `[c, p, L]` 패킷을 제공합니다. IMU·접촉 기반 추정기는 아닙니다. 예측에는 과거 wrench 명령만 사용합니다. 초기 상태와 nominal 질량을 알고 있다고 가정하며 이상적 측정용 질량 추정기는 끕니다.

축별 Gaussian 잡음은 CoM 0.5 mm, 선운동량 0.01 kg m/s, 각운동량 0.002 kg m²/s입니다. 83.3–100 ms 격자에서 지연 없음, 한 sample 지연, 15% 무작위 누락과 네 sample 연속 누락을 평가합니다. 긴 관측 중단은 지원 범위 밖입니다.

새 패킷의 시각까지 과거 명령으로 예측한 뒤 `posterior = prior + gain * (measurement - prior)`로 보정하고 현재까지 다시 예측합니다. Gain 1은 raw 패킷 예측, 0.65는 필터입니다. 중복 패킷은 무시하며 EKF가 아닌 고정 gain 추정기입니다.

Seed 0·7·23의 24개 비교 실행이 5.7초의 60구간을 완료합니다. 측방 impulse는 2.1초에 요청되어 2.15초에 적용됩니다. 잡음만 있을 때 CoM 추정 RMS 평균은 raw 0.807 mm / 필터 0.560 mm입니다. 낮은 gain에는 잡음 억제와 외란 반응의 trade-off가 있습니다.

![CoM estimation](../media/observation_benchmark/noise_delay/seed_7/observations.png)

[Measurements](../media/observation_benchmark/matrix.json) · [Implementation](../src/centroidal_sensing.py)

```bash
python lab-9-model-predictive-control/src/observation_benchmark.py
```
