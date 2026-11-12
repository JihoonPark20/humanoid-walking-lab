# Lab 8: Centroidal 최적화

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Nonlinear multiple shooting으로 CoM 높이·운동량·contact wrench를 최적화합니다.

![Demo](media/offline/centroidal.png)

축소 모델의 CoM 높이·운동량·접촉력 최적화 결과입니다. 로봇 rollout은 아닙니다.

[실습과 측정 결과](GALLERY.md) · [학습 노트](docs/README.md)

## 실행

[설치](../docs/SETUP.md)

```bash
python lab-8-centroidal-optimization/src/offline_centroidal.py
```

평지의 고정 접촉을 사용하는 축소 모델입니다. 전신 실행 가능성은 별도로 검증합니다.

```bash
python -m pytest lab-8-centroidal-optimization/tests -q
```

[← Curriculum](../README.ko.md)
