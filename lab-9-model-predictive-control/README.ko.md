# Lab 9: MPC·RTI·관측 강인성

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Full-NLP MPC와 single-QP RTI로 재계획하고 지연·모델 오차·관측 잡음을 평가합니다.

![Demo](media/gallery/hero.gif)

외란 전후 확대·0.25배속: 빨간 외란에 따라 보라 CoM 예측이 바뀌며 회색 원래 계획은 유지됩니다. 미래 관절 자세가 아닌 CoM 예측입니다.

[데모](GALLERY.ko.md) · [학습 노트](docs/README.ko.md) · [시각화 읽는 법](../docs/VISUAL_GUIDE.ko.md)

## 실행

[설치](../docs/SETUP.ko.md)

```bash
python lab-9-model-predictive-control/src/g1_mpc.py --mode rti --video
python lab-9-model-predictive-control/src/observation_benchmark.py
```

평지·고정 발 디딤 조건을 검증했습니다. 패킷 추정기는 축소 centroidal 상태를 사용하며 IMU 추정기가 아닙니다. 긴 관측 중단은 지원 범위 밖입니다.

[관측 모델](docs/OBSERVATION_ROBUSTNESS.ko.md)

```bash
python -m pytest lab-9-model-predictive-control/tests -q
```

[← Curriculum](../README.ko.md)
