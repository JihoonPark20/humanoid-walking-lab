# RRT and RRT*

[English](02_rrt_planning.md) · [한국어](02_rrt_planning.ko.md) · [简体中文](02_rrt_planning.zh-CN.md)

`RRTStarPlanner`는 6관절 configuration space의 node `(q,parent,cost)`를 저장합니다. 관절 범위 안에서 샘플링하고 goal bias 확률로 목표를 선택합니다. 매 반복에서 최근접 node를 찾고 `step_size` 이내로 이동한 뒤 새 edge 전체를 검사합니다.

RRT는 최근접 node에 연결하고 처음 유효한 목표 연결을 반환합니다. RRT*는 `rewire_radius` 안에서 비용이 가장 작은 충돌 없는 parent를 고르고, 더 짧은 연결로 이웃을 rewire하여 모든 자손의 비용을 갱신합니다. 목표 재연결도 자손 비용을 갱신하며 반복 예산까지 탐색합니다. 비용은 joint-space Euclidean 거리의 합으로, 실제 시간·에너지가 아닙니다. 고정 이웃 반경을 사용하는 구현이며 유한 반복의 최적성을 보장하지 않습니다.

`plan(q_start,q_goal,max_iter=5000,rrt_star=True,seed=42)`는 waypoint 목록 또는 `None`을 반환합니다. 잘못된 범위·비유한 입력을 거부하고 충돌·범위 밖 endpoint는 실패합니다. 동일한 안전 endpoint는 한 점의 경로입니다. 실패 시 이전 tree를 지웁니다. 통과 불가능한 벽은 검사 없는 연결 대신 `None`이어야 합니다.

Capstone은 필수 task waypoint의 IK를 구하고 전체 관절 범위·공통 탐색 설정·고정 seed 42로 재시도 없이 구간별 계획을 만듭니다. `planning_validation.py`는 raw·shortcut을 각각 독립 검사합니다. [회귀 테스트](../tests/test_planning_safety.py)는 해석적인 2D 장애물 검사, 통과 불가능한 벽, 보라색 기둥에 막힌 실제 UR5e 구간을 사용하며 rewiring 비용·깊은 자손 갱신·여러 seed도 검사합니다.

하나의 성공 seed는 성공률 benchmark가 아닙니다. 같은 장면·시작/목표·한계·예산으로 반복하고 실패도 함께 기록하세요.

[다중 환경 평가](GENERALIZATION.ko.md)
