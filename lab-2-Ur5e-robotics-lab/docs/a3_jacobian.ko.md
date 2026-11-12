# A3: Jacobian and Manipulability

[English](a3_jacobian.md) · [한국어](a3_jacobian.ko.md) · [简体中文](a3_jacobian.zh-CN.md)

6D Jacobian은 `twist=[v,ω]=J(q) qd`를 연결합니다. Revolute joint의 기하학적 열은 `Jv_i=z_i×(p_ee-p_i)`, `Jω_i=z_i`입니다. Joint 축과 위치를 모두 같은 world frame으로 표현해야 합니다.

Pinocchio의 `LOCAL_WORLD_ALIGNED` Jacobian과 직접 계산, finite difference, MuJoCo site Jacobian을 비교합니다. LOCAL/BODY/WORLD 표현과 linear/angular 순서가 다르면 숫자만 비교할 수 없습니다. 위치 차분뿐 아니라 회전 log도 확인하세요.

Singularity에서는 최소 singular value가 작아지고 condition number가 커집니다. Manipulability `sqrt(det(J J.T))`는 국소 지표이며 단위 scaling과 task 선택에 영향을 받습니다. 어깨·팔꿈치·wrist singularity의 잃는 방향을 SVD로 확인하세요.

Lab 폴더에서 `python src/a3_jacobian.py`를 실행합니다. `a3_jacobian_validation.csv`, `a3_singularity_cases.csv`, `a3_manipulability_heatmap.csv`를 비교하고, singularity 부근의 속도 명령을 DLS로 regularize하는 이유를 A4와 연결하세요.

[학습 노트](README.ko.md) · [실행 안내](../README.ko.md)
