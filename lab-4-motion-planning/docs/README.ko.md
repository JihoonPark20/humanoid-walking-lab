# Lab 4: Motion planning

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Path는 지나갈 관절 자세를, trajectory는 통과 시간까지 정의합니다. 충돌 없는 path도 너무 빠르면 추종할 수 없습니다. 파이프라인은 IK → RRT* → shortcut → 시간 매개화 → 독립 검사 → torque 제어 → MuJoCo입니다.

Slalom에는 보라색 blocking post가 있습니다. 첫 두 중간 task pose 사이의 직선 보간은 이 물체와 충돌하며, 장애물을 제거하면 동일한 보간이 통과합니다. 중간 waypoint를 방문하는 과제이며, 전체 시작점에서 최종점까지의 직선이 막힌다는 뜻은 아닙니다.

[충돌 검사](01_collision_checking.ko.md), [RRT/RRT*](02_rrt_planning.ko.md), [궤적 처리](03_trajectory_processing.ko.md), [실행](04_trajectory_execution.ko.md) 순서로 읽으세요. Seed별 경로 길이·clearance·추종 오차를 함께 비교하고, 속도 제한을 낮춰 시간과 오차 변화를 확인하세요. Planner 비교 시 장면·tool 형상·판정 기준을 고정합니다.

[실행 명령](../README.ko.md) · [계획 검증](../media/planning_audit.json) · [실행 결과](../media/slalom_summary.json)

[다중 환경 평가](GENERALIZATION.ko.md)
