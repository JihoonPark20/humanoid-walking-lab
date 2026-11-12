# Lab 6: Whole-body 보행 제어

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Inverse-dynamics QP, contact wrench와 DCM 추종으로 G1 보행을 제어합니다.

![Demo](media/gallery/hero.gif)

발 주변 확대·0.65배속: 양발→한발→양발로 실제 지지 영역이 바뀌는 모습을 보세요. 보라 DCM과 주황 접촉 ZMP를 비교하세요.

[데모](GALLERY.ko.md) · [학습 노트](docs/README.ko.md) · [시각화 읽는 법](../docs/VISUAL_GUIDE.ko.md)

## 실행

[설치](../docs/SETUP.ko.md)

```bash
python lab-6-whole-body-control/src/m3_walking.py
```

```bash
python -m pytest lab-6-whole-body-control/tests -q
```

[← Curriculum](../README.ko.md)
