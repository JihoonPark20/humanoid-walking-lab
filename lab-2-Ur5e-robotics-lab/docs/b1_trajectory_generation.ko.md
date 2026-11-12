# B1: Trajectory Generation

[English](b1_trajectory_generation.md) · [한국어](b1_trajectory_generation.ko.md) · [简体中文](b1_trajectory_generation.zh-CN.md)

이 모듈은 cubic·quintic·trapezoidal 관절 궤적, Cartesian minimum-jerk, 여러 waypoint 연결을 제공합니다. `JointTrajectoryPoint`는 시간·q·qd·qdd를 담습니다.

```text
s_cubic(u)=3u²-2u³
s_quintic(u)=10u³-15u⁴+6u⁵, u=t/T
q=q_start+(q_end-q_start)s
```

Cubic은 끝 속도 0, quintic은 끝 속도·가속도 0입니다. Trapezoidal은 가속·등속·감속 구간이며 짧은 이동에서는 triangular가 될 수 있습니다. 여러 구간에서 시간 중복과 경계 연속성을 검사하세요. 관절 직선은 Cartesian 직선과 다릅니다.

Lab 폴더에서 `python src/b1_trajectory_generation.py`를 실행하고 `b1_*_traj.csv`를 비교합니다. 경계조건·수치미분·속도/가속도 한계를 확인하세요. 이 단계는 시간 reference를 만들며 collision/torque feasibility는 별도 검사입니다.

[학습 노트](README.ko.md) · [실행 안내](../README.ko.md)
