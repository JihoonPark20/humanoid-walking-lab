# Lab 1: 2-link 평면 로봇 팔

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

평면 로봇 팔로 FK, Jacobian, Inverse Kinematics와 PD 제어를 익힙니다.

![Demo](media/gallery/hero.gif)

정사각형 궤적 추종. 기존 실행 영상입니다.

[실습과 측정 결과](GALLERY.md) · [학습 노트](docs/README.md)

## 실행

[설치](../docs/SETUP.md)

```bash
mjpython lab-1-2link-arm/src/c1_draw_square.py
```

```bash
python -m pytest lab-1-2link-arm/tests -q
```

[← Curriculum](../README.ko.md)
