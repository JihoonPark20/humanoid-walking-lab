# Lab 5: 균형과 무게 이동

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Floating-base 상태, CoM, Whole-body IK와 양발 접촉 균형을 익힙니다.

![Demo](media/gallery/hero.gif)

확대·0.65배속: 청록 CoM 원이 흰 목표로 이동할 때 초록 지지 영역 안의 주황 접촉 ZMP가 어떻게 반응하는지 보세요.

[데모](GALLERY.ko.md) · [학습 노트](docs/README.ko.md) · [시각화 읽는 법](../docs/VISUAL_GUIDE.ko.md)

## 실행

[설치](../docs/SETUP.ko.md)

```bash
python lab-5-locomotion/src/m3d_weight_shift.py
```

준정적 균형 과정이며 torque 기반 동적 보행은 Lab 6에서 이어집니다.

```bash
python -m pytest lab-5-locomotion/tests -q
```

[← Curriculum](../README.ko.md)
