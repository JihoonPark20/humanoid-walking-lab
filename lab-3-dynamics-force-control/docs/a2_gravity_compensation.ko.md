# A2: Gravity Compensation

[English](a2_gravity_compensation.md) · [한국어](a2_gravity_compensation.ko.md) · [简体中文](a2_gravity_compensation.zh-CN.md)

가장 단순한 모델 기반 입력은 `tau=g(q)`입니다. 중력만 상쇄하며 초기 자세로 되돌리는 위치 stiffness는 없습니다.

```text
M(q) qdd + C(q,qd) qd = tau_passive
```

정확한 보상·초기 속도 0에서는 현재 자세를 유지할 수 있습니다. Damping은 외란 후 속도를 줄이지만 원래 q로 복귀시킨다는 보장은 없습니다. 외란에서 복귀하려면 PD 또는 impedance가 필요합니다. 자세에 따른 M 때문에 전체 시스템을 항상 단일 exponential decay라고 표현할 수도 없습니다.

Lab 폴더에서 `python src/a2_gravity_compensation.py`를 실행합니다. Hold의 최대 q 오차와 perturbation 이후 최종 속도를 분리해 검사하세요. `tests/test_gravity_comp.py`는 hold 오차 <0.01 rad, perturbation 최종 속도 <0.1 rad/s를 확인합니다. 결과 plot에서 위치 drift와 속도 damping을 구분하세요.

[학습 노트](README.ko.md) · [실행 안내](../README.ko.md)
