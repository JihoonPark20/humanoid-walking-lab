# 실습에 필요한 공통 개념

[English](CONCEPTS.md) · [한국어](CONCEPTS.ko.md) · [简体中文](CONCEPTS.zh-CN.md)

| 용어 | 의미 | 처음 다루는 곳 |
|---|---|---|
| DOF | 독립적으로 움직일 수 있는 자유도 | Lab 1 |
| FK (Forward Kinematics) | 관절 상태에서 끝점 pose 계산 | Lab 1 |
| IK (Inverse Kinematics) | 목표 pose를 만족하는 관절 상태 계산 | Lab 1 |
| Jacobian | 관절 속도를 task-space 속도로 연결하는 미분 행렬 | Lab 1 |
| Singularity | Jacobian의 rank가 낮아져 특정 운동 방향을 잃는 자세 | Lab 1 |
| DLS (Damped Least Squares) | 정규화 항으로 singularity 부근의 큰 관절 변화를 완화하는 IK 방법 | Lab 1 |
| Pose / SE(3) | 3차원 위치와 회전을 묶은 강체 변환 | Lab 2 |
| RNEA / CRBA | Inverse dynamics / 질량 행렬 계산 알고리즘 | Lab 3 |
| Impedance | 위치·속도 오차와 힘 사이의 원하는 관계 | Lab 3 |
| Wrench | 힘 3개와 모멘트 3개를 묶은 벡터 | Lab 3 |
| Configuration space | 가능한 관절 자세를 좌표로 표현한 공간 | Lab 4 |
| RRT / RRT* | 무작위 sampling으로 충돌 없는 경로를 찾는 방법 | Lab 4 |
| TOPP-RA | 경로에 속도·가속도 제약을 만족하는 시간을 부여하는 방법 | Lab 4 |
| Floating base | World에 고정되지 않아 직접 구동할 수 없는 base 자유도가 있는 모델 | Lab 5 |
| CoM / CoP | 질량 중심 / 지면 접촉 압력의 중심 | Lab 5–6 |
| ZMP | 지면의 기울이는 모멘트가 0이 되는 점 | Lab 5 |
| LIPM / DCM | 선형 도립진자 모델 / CoM 위치와 속도로 정의하는 발산 성분 | Lab 5–6 |
| QP | 선형 제약과 이차 비용함수로 구성한 최적화 문제 | Lab 6 |

## MuJoCo 상태 읽기

`MjModel`은 link·joint·geom·actuator와 물리 파라미터를 담고, `MjData`는 현재 시뮬레이션 상태와 계산 결과를 담습니다.

- `qpos`: 일반화 위치입니다. Freejoint의 quaternion 때문에 `qvel`과 길이가 다를 수 있습니다.
- `qvel`: 일반화 속도입니다.
- `ctrl`: actuator 입력입니다. 모델에 따라 torque, 위치 목표 등 의미가 달라집니다.
- `body`: 좌표계와 질량·관성의 기준입니다.
- `geom`: 충돌과 표시를 위한 형상입니다.
- `site`: 끝점, 센서 위치 등 관심 있는 지점을 표시하는 기준입니다.
- `mj_forward`: 현재 상태에서 위치·접촉 등의 파생량을 갱신합니다.
- `mj_step`: Dynamics를 계산하고 시뮬레이션 시간을 진행합니다.

FK 교차 검증에서는 두 모델에 **같은 상태**를 설정하고 계산해야 합니다. 한쪽만 `mj_step`으로 전진시킨 뒤 비교하면 모델 차이와 시간 차이가 섞입니다.

## 단위와 좌표계

기본 단위는 m, s, kg, rad, N, N·m입니다. Plot의 mm·degree 표시와 내부 계산 단위를 구분하세요. World, base, tool, contact frame 중 어디에서 표현한 값인지 모든 벡터에 대해 확인하는 습관이 중요합니다.

[학습 순서로 돌아가기](../README.ko.md)
