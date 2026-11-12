# Humanoid Walking Lab

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

MuJoCo로 배우는 로봇 공학 기초, 균형 제어와 모델 기반 휴머노이드 보행.

![G1 walking](lab-7-walking-optimization/media/gallery/hero.gif)

## 커리큘럼

Pinocchio는 Kinematics·Dynamics를 계산하고 MuJoCo는 접촉과 운동을 시뮬레이션합니다. Lab 1–13 순서로 진행하세요.

| Lab | 커리큘럼 |
|---|---|
| 1 | [2-link 평면 로봇 팔](lab-1-2link-arm/README.ko.md) |
| 2 | [UR5e Kinematics](lab-2-Ur5e-robotics-lab/README.ko.md) |
| 3 | [Dynamics와 힘 제어](lab-3-dynamics-force-control/README.ko.md) |
| 4 | [Motion Planning](lab-4-motion-planning/README.ko.md) |
| 5 | [균형과 무게 이동](lab-5-locomotion/README.ko.md) |
| 6 | [Whole-body 보행 제어](lab-6-whole-body-control/README.ko.md) |
| 7 | [보행 궤적 최적화](lab-7-walking-optimization/README.ko.md) |
| 8 | [Centroidal 최적화](lab-8-centroidal-optimization/README.ko.md) |
| 9 | [MPC·RTI·관측 강인성](lab-9-model-predictive-control/README.ko.md) |
| 10 | [Whole-body 최적제어](lab-10-whole-body-optimal-control/README.ko.md) |
| 11 | [접촉을 고려한 보행](lab-11-contact-transitions/README.ko.md) |
| 12 | [물리 보행 강건성 평가](lab-12-walking-robustness/README.ko.md) |
| 13 | [온라인 발 위치 조정](lab-13-adaptive-footsteps/README.ko.md) |

## 빠른 시작

Python 3.11 이상 · CPU · Apple Silicon 검증. macOS 대화형 viewer는 `mjpython`을 사용하며 Lab 10은 별도 환경이 필요합니다.

```bash
python3 -m venv .venv
source .venv/bin/activate
bash tools/setup_env.sh
python lab-9-model-predictive-control/src/g1_mpc.py --mode rti --video
```

[시각화 읽는 법](docs/VISUAL_GUIDE.ko.md) · [설치](docs/SETUP.ko.md) · [실험 해석](docs/VALIDATION.ko.md) · [공통 개념](docs/CONCEPTS.ko.md)

[M. Ozkan Ceylan의 robotics labs](https://github.com/ozkannceylan/mujoco-robotics-lab)를 바탕으로 합니다. [Apache-2.0](LICENSE) · [NOTICE](NOTICE) · [Model licenses](THIRD_PARTY_NOTICES.md)
