# Lab 4: Motion Planning

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

RRT* 충돌 회피 경로를 만들고 시간 매개화와 궤적 추종을 실습합니다.

![Demo](media/gallery/hero.gif)

3차원 slalom 2배속: 접촉 0회, 최소 장애물 간격 30.35 mm. 매 1 ms마다 collision 형상과 화면 mesh를 함께 검사합니다.

[실습과 측정 결과](GALLERY.md) · [학습 노트](docs/README.md)

## 실행

[설치](../docs/SETUP.md)

```bash
python lab-4-motion-planning/src/capstone_demo.py
```

```bash
python -m pytest lab-4-motion-planning/tests -q
```

[← Curriculum](../README.ko.md)
