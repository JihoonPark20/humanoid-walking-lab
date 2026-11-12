# Lab 1: 2-link 평면 로봇 팔

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

평면 로봇 팔로 FK, Jacobian, Inverse Kinematics와 PD 제어를 익힙니다.

![Demo](media/gallery/hero.gif)

관절 운동이 말단의 정사각형 경로를 만드는 과정을 관찰하세요. 특히 모서리에서의 움직임을 비교하세요.

[데모](GALLERY.ko.md) · [학습 노트](docs/README.ko.md)

## 실행

[설치](../docs/SETUP.ko.md)

```bash
mjpython lab-1-2link-arm/src/c1_draw_square.py
```

```bash
python -m pytest lab-1-2link-arm/tests -q
```

[← Curriculum](../README.ko.md)
