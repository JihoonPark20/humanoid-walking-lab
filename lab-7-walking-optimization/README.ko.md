# Lab 7: 보행 궤적 최적화

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Offline LIPM 궤적·발 디딤·시간 후보를 최적화하고 G1에서 추종합니다.

![Demo](media/gallery/hero.gif)

발 위치·CoM 최적화의 MuJoCo 추종. Swing 5/5, 최종 CoM 오차 15.15 mm.

[실습과 측정 결과](GALLERY.md) · [학습 노트](docs/README.md)

## 실행

[설치](../docs/SETUP.md)

```bash
python lab-7-walking-optimization/src/offline_walk.py --compare
python lab-7-walking-optimization/src/g1_walk.py --mode optimized --optimize-feet --video
```

고정된 접촉 순서의 오프라인 발 디딤·시간 최적화입니다.

```bash
python -m pytest lab-7-walking-optimization/tests -q
```

[← Curriculum](../README.ko.md)
