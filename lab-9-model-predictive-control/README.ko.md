# Lab 9: MPC·RTI·관측 강인성

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Full-NLP MPC와 single-QP RTI로 재계획하고 지연·모델 오차·관측 잡음을 평가합니다.

![Demo](media/gallery/hero.gif)

보라 구와 궤적은 현재 CoM 예측, 회색은 원래 계획입니다. 빨간 화살표는 5 N 외란입니다. 최종 CoM 오차 0.922 mm.

[실습과 측정 결과](GALLERY.md) · [학습 노트](docs/README.md) · [시각화 읽는 법](../docs/VISUAL_GUIDE.ko.md)

## 실행

[설치](../docs/SETUP.md)

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
