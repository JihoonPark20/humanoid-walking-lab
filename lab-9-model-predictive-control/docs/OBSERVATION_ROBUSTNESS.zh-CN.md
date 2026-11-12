# 观测模型

[English](OBSERVATION_ROBUSTNESS.md) · [한국어](OBSERVATION_ROBUSTNESS.ko.md) · [简体中文](OBSERVATION_ROBUSTNESS.zh-CN.md)

简化 centroidal 传感器提供带 timestamp 的 `[c, p, L]` 数据包，并非 IMU/接触状态估计器。预测仅使用历史 wrench 指令。假设初始状态与 nominal 质量已知，关闭理想观测质量估计器。

每轴 Gaussian 噪声为 CoM 0.5 mm、线动量 0.01 kg m/s、角动量 0.002 kg m²/s。在 83.3–100 ms 网格上评估无延迟、一个 sample 延迟、15% 随机丢包及连续四个 sample 丢失。长时间观测中断不在支持范围内。

用历史指令预测到新数据包时刻，按 `posterior = prior + gain * (measurement - prior)` 校正，再预测到当前。Gain 1 为 raw 数据包预测，0.65 为滤波；忽略重复数据包。这是固定增益估计器，不是 EKF。

Seed 0、7、23 的 24 个配对实验均完成 5.7 s 的 60 个区间。横向 impulse 在 2.1 s 请求，于 2.15 s 施加。仅噪声条件下 CoM 估计 RMS 平均为 raw 0.807 mm / 滤波 0.560 mm。较低 gain 在抑制噪声和响应扰动之间存在取舍。

![CoM estimation](../media/observation_benchmark/noise_delay/seed_7/observations.png)

[Measurements](../media/observation_benchmark/matrix.json) · [Implementation](../src/centroidal_sensing.py)

```bash
python lab-9-model-predictive-control/src/observation_benchmark.py
```
