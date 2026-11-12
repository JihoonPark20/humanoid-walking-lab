# Lab 4: 한국어 학습 노트

[실행 명령과 코드 순서](../README.md) · [공통 용어](../../docs/CONCEPTS.md)

## 경로와 궤적의 차이

**Path**는 어떤 자세들을 지나갈지 정한 순서이며, **trajectory**는 각 자세를 언제 지날지까지 정한 함수입니다. 충돌 없는 path에도 너무 빠른 시간이 붙으면 로봇은 이를 추종하지 못할 수 있습니다.

```text
q_start, q_goal
  → collision checker
  → RRT / RRT*
  → shortcut
  → time parameterization
  → q(t), qdot(t), qddot(t)
  → 제어기 → MuJoCo
```

RRT는 random sample 방향으로 tree를 확장합니다. RRT*는 주변 node의 parent와 연결을 다시 선택해 비용을 줄입니다. 샘플 수를 늘려도 충돌 판정의 누락은 고쳐지지 않으므로 `collision_checker.py`부터 읽으세요.

## 충돌 검사에서 놓치기 쉬운 부분

Node만 안전해도 두 node를 잇는 edge 중간에서 충돌할 수 있습니다. Edge의 보간 검사 해상도를 확인하세요. 인접 link의 겹치는 geometry를 제외하는 것은 이 모델의 실용적인 규칙이며, 모든 인접 link가 항상 안전하다는 일반 법칙은 아닙니다.

`shortcut_path`도 원래 path와 같은 checker를 써야 합니다. 계획 단계와 실행 장면의 물체 크기·위치·tool 형상이 다르면 계획상의 clearance가 의미 없어집니다.

## 시간 매개화

`parameterize_topp_ra`는 `(times, q, qd, qdd)`를 반환합니다. TOPP-RA가 설치되지 않으면 보수적인 quintic 방식으로 대체됩니다. 반복 waypoint는 arc-length를 증가시키지 않으므로 spline 구성 전에 제거합니다.

## 직접 해볼 실험

같은 시작·목표에서 random seed를 바꾸어 성공률과 비용을 비교합니다. Shortcut 전후의 길이, obstacle clearance, 실행 후 tracking error를 함께 읽으세요. 속도 제한을 낮추면 소요 시간과 tracking error가 어떻게 바뀌는지 확인합니다.

## 상세 참고 자료


- [Lab 4: Collision Checking](01_collision_checking.md)
- [Lab 4: RRT and RRT* Motion Planning](02_rrt_planning.md)
- [Lab 4: Trajectory Processing](03_trajectory_processing.md)
- [Lab 4: Trajectory Execution](04_trajectory_execution.md)
