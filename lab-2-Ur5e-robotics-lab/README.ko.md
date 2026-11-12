# Lab 2: UR5e Kinematics

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

6-DOF UR5e로 Kinematics와 궤적 추종을 확장합니다.

![Demo](media/gallery/hero.gif)

3차원 큐브 전체 추종 3배속: RMS 오차 0.088 mm. 녹화한 실행의 측정값입니다.

[실습과 측정 결과](GALLERY.md) · [학습 노트](docs/README.md)

## 실행

[설치](../docs/SETUP.md)

```bash
python lab-2-Ur5e-robotics-lab/src/c3_record_video.py
```

```bash
python -m pytest lab-2-Ur5e-robotics-lab/tests -q
```

[← Curriculum](../README.ko.md)
