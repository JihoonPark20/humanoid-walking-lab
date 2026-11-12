# Lab 6: 한국어 학습 노트

[실행 명령과 코드 순서](../README.md) · [공통 용어](../../docs/CONCEPTS.md)

## 균형을 Dynamics 제약에 넣기

Floating base에는 직접 토크를 줄 수 없습니다. 따라서 아래 방정식의 base 부분을 발의 접촉력이 만족해야 합니다.

```text
M(q) qddot + h(q, qdot) = Sᵀ τ + Jcᵀ f
Jc qddot + Jcdot qdot = 0
```

`wb_id_qp.py`는 `qddot`과 contact wrench `f`를 변수로 두고, task 오차를 줄이면서 base dynamics·지지발 유지·마찰·CoP·토크 한계를 만족시키려 합니다. `wb_tasks.py`에서 각 task의 Jacobian과 `Jdot qdot` 항을 추적하세요.

Velocity-level IK/QP는 발이 요구하는 힘을 직접 결정하지 않습니다. 이 실습에서 그것만으로 균형을 유지하지 못한 이유입니다. 그렇다고 kinematic QP 자체가 불필요한 것은 아니며, 접촉력을 다른 계층에서 다루는 구조와 구분해야 합니다.

## DCM으로 보행 읽기

```text
ξ = c + cdot/ω
ξdot = ω(ξ − p)
```

DCM은 CoM 위치와 속도를 함께 나타냅니다. `dcm_planner.py`에서 발 디딤과 ZMP 계획으로 reference를 만들고 `locomotion_controller.py`에서 실제 상태에 따라 보정합니다. 한 발 위에서 CoM이 정지하도록 명령하는 준정적 전략과 다릅니다.

## 손 task를 추가하면 생기는 변화

팔과 payload의 움직임은 centroidal momentum과 CoM에 영향을 줍니다. 손 오차 weight만 높이면 균형 task와 충돌할 수 있습니다. `wb_tasks.py`의 momentum task를 확인하고, 고정된 운반 자세와 움직이는 손 궤적의 목표가 어떻게 다른지 비교하세요.

QP의 status가 최대 반복 횟수 도달이면 숫자가 반환되었어도 제약을 만족한다고 가정하지 않습니다. Tolerance를 무조건 줄이기보다 scaling, iteration 수, residual과 solve time을 함께 살펴보세요.

## 모델과 접촉의 중요성

CoP 영역은 실제 발바닥 geometry와 wrench 기준 frame에 맞아야 합니다. 발목 frame이 바닥보다 높으면 수평 접촉력의 모멘트도 고려합니다. 장면에 탁자나 선반을 추가한 뒤 넘어지면 제어 gain을 바꾸기 전에 contact pair부터 확인하세요.

Weld를 켤 때 현재 상대 pose를 반영하고, 놓을 때는 손이 아니라 물체의 목표 위치와 실제 resting pose를 확인합니다. 실습의 기존 결과와 새로운 물체 배치의 성공률을 구분하세요.

## 상세 참고 자료


