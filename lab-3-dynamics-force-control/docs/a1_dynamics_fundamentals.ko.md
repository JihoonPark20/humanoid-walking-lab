# A1: Dynamics Fundamentals

[English](a1_dynamics_fundamentals.md) · [한국어](a1_dynamics_fundamentals.ko.md) · [简体中文](a1_dynamics_fundamentals.zh-CN.md)

MuJoCo UR5e·Robotiq와 Pinocchio URDF의 Dynamics를 교차 검증합니다.

```text
M(q) qdd + C(q,qd) qd + g(q) = tau
M=M.T, eig(M)>0
M_dot-2C = -(M_dot-2C).T
```

CRBA(`pin.crba`)로 M을 구하고 삼각 부분을 대칭화합니다. `computeCoriolisMatrix`의 C는 수치미분한 M_dot과 passivity 관계를 검사합니다. `computeGeneralizedGravity`의 g는 qd=0인 MuJoCo `qfrc_bias`와 비교합니다. qd≠0에서는 bias가 Cqd도 포함합니다.

`lab3_common.py`는 Menagerie arm actuator를 유지하고 원하는 joint torque를 ctrl로 변환합니다. Pinocchio에는 고정 Robotiq payload와 `model.armature`를 맞춥니다. `rotorInertia`에 넣는 것과 혼동하지 마세요. Model mismatch는 controller 문제가 생기기 전에 확인해야 합니다.

Lab 폴더에서 `python src/a1_dynamics_fundamentals.py`를 실행하세요. M heatmap·gravity bar/sweep과 tests의 오차 한계를 확인하고, 여러 q·qd에서 대칭성·positive definiteness·cross-engine parity를 검사합니다.

[학습 노트](README.ko.md) · [실행 안내](../README.ko.md)
