# Lab 4: Motion Planning

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

RRT* 충돌 회피 경로를 만들고 시간 매개화와 궤적 추종을 실습합니다.

![Demo](media/gallery/hero.gif)

필수 직선 구간을 막는 기둥을 RRT*로 우회합니다. 계획 경로와 실제 로봇 움직임을 비교하세요(2배속).

[데모](GALLERY.ko.md) · [학습 노트](docs/README.ko.md)

## 실행

[설치](../docs/SETUP.ko.md)

[다중 환경 평가](docs/GENERALIZATION.ko.md)

```bash
python lab-4-motion-planning/src/motion_planning.py lab-4-motion-planning/scenarios/robot_101.json --seed 0 --execute
python lab-4-motion-planning/src/benchmark_planning.py
python lab-4-motion-planning/src/capstone_demo.py
```

```bash
python -m pytest lab-4-motion-planning/tests -q
```

[← Curriculum](../README.ko.md)
