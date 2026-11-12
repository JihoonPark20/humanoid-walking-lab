# Lab 10: Whole-body 최적제어

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Box-FDDP로 contact/impact Dynamics를 최적화하고 MuJoCo에서 추종합니다.

![Demo](media/gallery/hero.gif)

반투명 로봇은 0.25/0.50초 뒤의 최적화된 관절 자세입니다. 오프라인 기준 자세와 FDDP feedback으로 움직이는 로봇을 비교하세요.

[데모](GALLERY.ko.md) · [학습 노트](docs/README.ko.md) · [시각화 읽는 법](../docs/VISUAL_GUIDE.ko.md)

## 실행

[설치](../docs/SETUP.ko.md)

```bash
.venv-ocp/bin/python lab-10-whole-body-optimal-control/src/whole_body_demo.py --steps 2 --mujoco --video
```

Offline OCP와 feedback 추종입니다. 별도 OCP 환경과 권장 계획 비용을 사용합니다. Whole-body MPC는 포함하지 않습니다.

```bash
.venv-ocp/bin/python -m pytest lab-10-whole-body-optimal-control/tests -q
```

[← Curriculum](../README.ko.md)
