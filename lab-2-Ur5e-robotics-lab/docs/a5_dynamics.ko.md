# A5: Dynamics

[English](a5_dynamics.md) · [한국어](a5_dynamics.ko.md) · [简体中文](a5_dynamics.zh-CN.md)

Pinocchio로 `M(q) qdd + C(q,v)v + g(q)=tau`를 계산합니다. CRBA는 M, RNEA는 주어진 q·v·qdd의 torque, ABA는 주어진 torque의 가속도를 구합니다. `computeGeneralizedGravity`는 g, `computeCoriolisMatrix`는 C를 제공합니다.

RNEA와 ABA를 왕복시켜 qdd를 복원하고, v=0인 gravity와 v≠0인 bias를 구분하세요. MuJoCo `qfrc_bias`와 Pinocchio `rnea(q,v,0)`는 모델·armature·tool 질량·관절 순서가 같을 때 비교할 수 있습니다. M은 대칭 positive definite여야 하며 q에 따라 condition number가 변합니다.

Lab 폴더에서 `python src/a5_dynamics.py`를 실행합니다. `a5_dynamics_snapshot.csv`와 자세 sweep을 읽고 mass matrix·gravity·Coriolis·inverse/forward dynamics를 교차 검증하세요. 서로 다른 모델의 차이를 solver 오류로 오해하지 마세요.

[학습 노트](README.ko.md) · [실행 안내](../README.ko.md)
