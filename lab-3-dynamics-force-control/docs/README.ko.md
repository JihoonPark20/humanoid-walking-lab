# Lab 3: 학습 노트

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

[실행 명령과 코드 순서](../README.ko.md) · [공통 용어](../../docs/CONCEPTS.ko.md)

## Dynamics 방정식 읽기

```text
M(q) qddot + h(q, qdot) = τ + Jᵀ f_external
h(q, qdot) = C(q, qdot) qdot + g(q)
```

`M`은 질량 행렬, `h`는 Coriolis·원심력·중력 항을 묶은 bias입니다. **RNEA**는 주어진 위치·속도·가속도에서 필요한 토크를, **CRBA**는 질량 행렬을 계산합니다. 중력만 비교하려면 속도와 가속도를 0으로 맞추세요.

`a1_dynamics_fundamentals.py`에서 Pinocchio 결과를 MuJoCo와 비교합니다. 로봇 본체가 같아도 tool payload, 관성, damping, armature가 다르면 결과가 달라집니다. 차이가 나면 solver보다 모델부터 확인하세요.

## 중력 보상과 impedance

중력 보상은 자세 오차를 줄여 주지만 자세 강성과 damping을 대신하지 않습니다. Cartesian impedance는 끝점 위치·속도 오차로 가상의 spring-damper wrench를 만들고 `τ = Jᵀ F`로 관절 토크에 반영합니다. 회전 task를 포함할 때는 `SO(3)` 회전 오차와 각속도도 함께 사용합니다.

강성을 높이면 목표 추종이 빨라질 수 있지만 접촉력과 진동도 커질 수 있습니다. `b2_compliance_demo.py`에서 외력에 대한 변위를 비교하세요.

## Hybrid position/force control

탁자 표면의 접선 방향은 위치를 추종하고 법선 방향은 접촉력을 조절합니다. Selection matrix로 두 방향을 분리하는 이유는 같은 자유도에 강한 위치 목표와 힘 목표를 동시에 강제하지 않기 위해서입니다.

`c1_force_control.py`에서 force feedback의 부호와 기준 frame을 확인한 뒤 `c2_line_trace.py`로 넘어가세요. 접촉 직전·접촉 중·접촉 해제 구간을 나누어 그래프를 읽으면 힘의 overshoot 원인을 찾기 쉽습니다.

대표 영상은 닫힌 그리퍼와 질량을 무시한 강체 probe로 마찰계수 μ=0.05인 평판에서 50 mm를 추종합니다. 손가락은 actuator로 닫힘을 유지하고 probe는 tool body에 고정됩니다. 접근·안정화·추종·유지 전 구간을 포함하며, 같은 실행에서 원시 접촉력, 자세 오차, 그리퍼 닫힘과 probe 이외의 접촉 여부를 검사합니다. [설정과 판정 기준](c1_force_control.ko.md), [측정값](../media/line_trace_summary.json)을 확인하세요.

## 직접 해볼 실험

Payload 유무, impedance stiffness, 목표 접촉력을 각각 따로 바꿔 봅니다. 목표 힘과 측정 힘의 평균 오차뿐 아니라 최대값, 진동, actuator saturation도 기록하세요.

## 상세 참고 자료


- [A1: Dynamics Fundamentals](a1_dynamics_fundamentals.ko.md)
- [A2: Gravity Compensation](a2_gravity_compensation.ko.md)
- [B1: Cartesian Impedance Control](b1_impedance_control.ko.md)
- [C1: Hybrid Position-Force Control](c1_force_control.ko.md)
