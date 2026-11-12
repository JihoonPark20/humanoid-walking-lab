# Lab 11: 접촉을 고려한 보행

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

![Demo](media/gallery/hero.gif)

왼쪽은 기존 sine Swing, 오른쪽은 C2 Swing입니다. 같은 첫 걸음을 0.25배속으로 보며 발과 보라 기준 궤적이 착지에 접근하는 모습을 비교하세요. C2는 기준 궤적 끝점을 매끄럽게 하며, 충격 감소를 뜻하지는 않습니다.

[데모](GALLERY.ko.md) · [학습 노트](docs/README.ko.md) · [환경 설정](../docs/SETUP.ko.md)

```bash
python lab-11-contact-transitions/src/contact_walk.py --video
python lab-11-contact-transitions/src/contact_walk.py --profile legacy --video --output lab-11-contact-transitions/media/legacy
```

[← 커리큘럼](../README.ko.md)
