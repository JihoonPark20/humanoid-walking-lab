# A2: Forward Kinematics

[English](a2_forward_kinematics.md) · [한국어](a2_forward_kinematics.ko.md) · [简体中文](a2_forward_kinematics.zh-CN.md)

UR5e FK는 6관절 q에서 위치 p와 회전 R을 포함하는 `T=[R,p;0,1]∈SE(3)`를 구합니다. 평면 삼각함수만으로는 3차원 회전축·고정 offset을 모두 다루기 어려우므로 link 변환을 순서대로 곱합니다.

DH의 θ는 joint 회전, d는 축 방향 offset, a는 link 길이, α는 인접 축의 twist입니다. 그러나 이 저장소의 `ur5e_common.forward_chain()`은 단순 표준 DH 표를 그대로 곱하는 구현이 아닙니다. 실제 모델과 맞춘 고정 변환·joint rotation·tool offset의 순서가 핵심입니다. `a2_forward_kinematics.py`의 교육용 DH 계산과 구분하세요.

변환의 마지막 열은 위치, R의 열들은 변환된 축 방향입니다. RPY는 사람이 보기 위한 표현이며 singularity와 convention 문제가 있으므로 회전 비교에는 R 또는 SO(3) 오차를 사용합니다. Flange와 tool 끝을 같은 점으로 가정하지 마세요.

```text
T_world_tool = T_world_base × Π_i(T_fixed_i T_joint_i(q_i)) × T_tool
```

Lab 폴더에서 `python src/a2_forward_kinematics.py`를 실행합니다. `a2_fk_validation.csv`에서 직접 chain·Pinocchio·MuJoCo가 같은 q, frame, tool offset을 사용했는지 확인합니다. Home·ready·over-table 자세를 그려 joint/task space 관계를 살펴보세요.

URDF는 joint axis·고정 origin·inertia·geometry를 표현하는 모델 형식이고, DH는 kinematic chain을 표로 표현하는 관례입니다. 둘 중 하나가 자동으로 참인 것이 아니라 같은 로봇을 같은 좌표계로 표현해야 합니다. FK 일치만으로 Dynamics·충돌·IK 수렴까지 검증되지는 않습니다.

[학습 노트](README.ko.md) · [실행 안내](../README.ko.md)
