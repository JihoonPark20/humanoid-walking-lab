# Lab 10: Whole-body 최적제어

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Box-FDDP로 contact/impact Dynamics를 최적화하고 MuJoCo에서 추종합니다.

![Demo](media/gallery/hero.gif)

두 걸음 FDDP 추종: 3.0초, 최종 발 오차 4.29 mm, flight·clipping 0 tick.

[실습과 측정 결과](GALLERY.md) · [학습 노트](docs/README.md)

## 실행

[설치](../docs/SETUP.md)

```bash
.venv-ocp/bin/python lab-10-whole-body-optimal-control/src/whole_body_demo.py --steps 2 --mujoco --video
```

Offline OCP와 feedback 추종입니다. 별도 OCP 환경과 권장 계획 비용을 사용합니다. Whole-body MPC는 포함하지 않습니다.

```bash
.venv-ocp/bin/python -m pytest lab-10-whole-body-optimal-control/tests -q
```

[← Curriculum](../README.ko.md)
