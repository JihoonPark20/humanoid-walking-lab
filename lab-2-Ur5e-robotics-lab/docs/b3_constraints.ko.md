# B3: Joint Constraints and Safety

[English](b3_constraints.md) · [한국어](b3_constraints.ko.md) · [简体中文](b3_constraints.zh-CN.md)

`check_joint_limits()`는 joint별 범위 여유를, `clamp_joint_positions()`는 hard clipping을 제공합니다. 한계 부근 `joint_limit_repulsion()`은 기본 0.1 rad buffer·gain 5로 반발 torque를 더합니다. `scale_velocity()`는 전체 속도 벡터를 비례 축소해 방향을 유지하며 `scale_delta_q()`는 dt를 고려합니다.

`saturate_torques()`는 첫 3관절 ±150 N·m, wrist 3관절 ±28 N·m로 제한합니다. `safe_command(q,qd,tau,dt)`는 속도 축소 → torque 포화 → 반발 추가 → 재포화 → 다음 q 범위 보정 순서로 처리하고 활성화된 제한을 기록합니다.

Self-collision 검사는 비인접 link frame 간 거리 0.05 m의 기하학적 휴리스틱입니다. 실제 mesh 충돌 검사가 아니며 `safe_command`가 모든 충돌을 회피한다는 보장은 없습니다. 실제 환경 collision pair와 경로 내부 검사는 Lab 4에서 다룹니다.

Lab 폴더에서 `python src/b3_constraints.py`를 실행하고 `b3_limit_guard.csv`, `b3_collision_cases.csv`, `b3_stress_test.csv`를 읽으세요. 이미 범위 밖인 초기 상태, 과속·과토크, near-limit 상태를 별도로 검사합니다.

[학습 노트](README.ko.md) · [실행 안내](../README.ko.md)
