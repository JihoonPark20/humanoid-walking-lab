# Lab 6: Whole-body 보행 제어

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Inverse-dynamics QP, contact wrench와 DCM 추종으로 G1 보행을 제어합니다.

![Demo](media/gallery/hero.gif)

한발·양발 지지 영역이 바뀔 때 DCM(보라)과 접촉 ZMP(주황)의 움직임을 보세요. 12걸음·1.182 m, 전체 실행 1배속.

[실습과 측정 결과](GALLERY.md) · [학습 노트](docs/README.md) · [시각화 읽는 법](../docs/VISUAL_GUIDE.ko.md)

## 실행

[설치](../docs/SETUP.md)

```bash
python lab-6-whole-body-control/src/m3_walking.py
```

```bash
python -m pytest lab-6-whole-body-control/tests -q
```

[← Curriculum](../README.ko.md)
