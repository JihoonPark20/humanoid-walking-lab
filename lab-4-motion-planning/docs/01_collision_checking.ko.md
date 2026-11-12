# Collision checking

[English](01_collision_checking.md) · [한국어](01_collision_checking.ko.md) · [简体中文](01_collision_checking.zh-CN.md)

`collision_checker.py`는 실행과 같은 MuJoCo 장면을 사용합니다. `is_collision_free(q)`는 유한한 6관절 입력, 실제 관절 범위, 로봇·환경 접촉, 선택한 self-collision pair를 검사합니다. 그리퍼 내부 및 모델별 인접 link pair는 제외하므로 임의의 self-collision까지 보장하지는 않습니다.

`is_path_free(a,b,resolution)`는 양 끝점뿐 아니라 보간 자세를 검사합니다. 유한한 해상도에서는 얇은 장애물을 놓칠 수 있습니다. `compute_min_obstacle_distance(q)`는 효율적인 계획을 위해 collision 형상을 사용합니다. `scene_clearance.py`는 collision proxy 밖으로 나온 표시 mesh도 잡도록 collision 형상과 표시 mesh를 독립적으로 검사합니다.

Capstone은 32 mm 여유로 계획하고 독립 검사에서 30 mm 이상인 경로만 허용합니다. Raw·shortcut edge는 joint-space L2 거리 최대 0.005 rad로 샘플링합니다. 모든 시간 궤적 reference와 실제 실행의 매 1 ms tick도 검사합니다. 이는 조밀한 이산 검사이며 연속시간 충돌 없음의 수학적 증명은 아닙니다.

`planning_validation.py`의 대조 검사는 장애물이 있을 때 실제 충돌하고, 탁자·로봇은 유지한 채 장애물만 제거하면 통과해야 합니다. 원래 비어 있는 환경이나 안전 여유만 부족한 경로는 이 기준을 통과하지 못합니다.

실습: [planning_audit.json](../media/planning_audit.json)의 막힌 필수 구간과 실제 실행의 무접촉 결과를 비교하세요. 성공을 위해 장애물 contact를 끄지 않습니다.
