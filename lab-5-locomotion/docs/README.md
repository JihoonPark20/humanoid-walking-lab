# Lab 5: 한국어 학습 노트

[실행 명령과 코드 순서](../README.md) · [공통 용어](../../docs/CONCEPTS.md)

## Floating base가 추가하는 것

G1의 base는 world에 고정되지 않습니다. Base pose는 위치 3개와 quaternion 4개, base velocity는 선속도 3개와 각속도 3개입니다. 그래서 이 모델은 `nq = 36`, `nv = 35`, actuator 수는 29입니다. Base의 6개 속도 자유도에는 직접 motor가 없습니다.

MuJoCo의 quaternion 순서는 `(w, x, y, z)`, Pinocchio는 `(x, y, z, w)`입니다. 두 모델 사이를 이동할 때 `lab5_common.py`의 상태 변환을 사용하세요. `q + dq` 같은 단순 덧셈으로 quaternion을 갱신하지 않습니다.

## CoM, support polygon, ZMP

CoM은 질량 중심입니다. 정적인 자세에서는 CoM의 지면 투영과 support polygon이 유용한 균형 지표지만, 움직일 때는 속도·가속도와 접촉력도 중요합니다. ZMP는 지면의 기울이는 모멘트가 0이 되는 점으로, CoM 투영과 항상 같은 점이 아닙니다.

일정한 CoM 높이를 가정한 LIPM은 수평 운동을 다음과 같이 단순화합니다.

```text
cddot = ω²(c − p),  ω = sqrt(g / z_com)
c: CoM 수평 위치, p: ZMP
```

`lipm_planner.py`와 `lipm_preview_control.py`는 계획 모델입니다. 계획 곡선이 안정적이라는 사실만으로 full-body MuJoCo 모델에서 같은 운동이 실현되지는 않습니다.

## Whole-body IK와 제어

`whole_body_ik.py`는 두 발 pose, CoM, pelvis 조건을 동시에 맞춥니다. IK 오차가 작아도 actuator와 contact가 그 동작을 실행할 수 있는지는 별도 문제입니다. 무게 이동에서 발의 drift와 pelvis 높이를 함께 관찰하세요.

이 Lab은 양발 접촉을 유지한 균형과 무게 이동을 다룹니다. 동적 보행은 Lab 6의 contact wrench·torque 제어로 이어집니다.

## 직접 해볼 실험

CoM의 좌우 목표 이동량과 전환 시간을 각각 바꾸며 발 미끄러짐·pelvis 높이·추종 오차를 비교하세요. 넘어질 때는 마지막 frame만 보지 말고 torque saturation과 발 접촉이 처음 달라지는 시점을 찾습니다.

## 상세 참고 자료


- [Unitree G1 — Joint Map & DOF Layout](g1_joint_map.md)
