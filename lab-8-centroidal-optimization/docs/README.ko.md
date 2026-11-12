# W3: Centroidal optimization 학습 설계

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Lab 7의 접촉 시간표와 발 위치를 고정한 뒤 CoM의 높이와 선형·각운동량, 각 발의 wrench를 함께 최적화합니다. 구현 순서는 모델 → 비선형 최적화 → 독립 검증 → G1 quantity 비교입니다.

## 모듈 연결

- `centroidal_model.py`: 상태 `[c, p, L]`, 발바닥 중심의 wrench `[f, tau]`, fixed-contact schedule, exact discrete Dynamics와 독립 제약 검사.
- `centroidal_optimizer.py`: CasADi 자동 미분과 IPOPT 기반 Direct Multiple Shooting. 초기 상태, node별 reference, 접촉·마찰·CoP·높이 제약, 선택적 종료 정지 조건. `CentroidalSolver.solve(initial, reference, contacts, warm_start)`는 MPC에서도 재사용합니다.
- `offline_centroidal.py`: Lab 7 reference를 warm start로 사용하고 높이 변화 과제를 실행합니다. NPZ/CSV/JSON/그림으로 저장합니다.
- `pinocchio_centroidal.py`: G1의 centroidal momentum과 centroidal map을 Pinocchio로 계산하고 독립 운동량 합산과 비교합니다.

데이터 흐름: Lab 7 계획 → 고정 발/접촉 → 9D 상태·12D wrench NLP → exact rollout 검증 → Lab 9 MPC 및 G1 Whole-body controller.

## 모델 정의

`c_dot=p/m`, `p_dot=sum(f)+mg`, `L_dot=sum((r-c)×f+tau)`를 사용합니다. 각 interval에서 world-frame wrench와 발 위치가 일정하다고 가정하면 CoM은 quadratic이며 운동량까지 exact integration할 수 있습니다. 각운동량 적분에는 interval 평균 CoM을 사용합니다. Quaternion이나 몸통 orientation은 이 축소 상태에 포함하지 않습니다.

마찰은 사각 pyramid, CoP는 발바닥 중심 모멘트의 선형 제한, yaw moment는 별도 제한으로 둡니다. 이는 완전한 직사각형 contact wrench cone의 근사입니다. 접촉하지 않는 발의 wrench는 0입니다. 발 yaw와 접촉 순서는 고정합니다.

최적화 solver의 성공과 별개로 Dynamics, 초기·종료조건, 마찰·CoP·높이·운동량 제약을 검증합니다. 최적화한 높이가 G1 관절로 실현 가능한지는 Whole-body 실행에서 따로 확인합니다.

`centroidal_io.py`는 Lab 7의 접촉 일정과 reference를 변환하고 계획을 저장·재검증합니다. 축소 모델 자체의 적분식은 NumPy/CasADi로 구현하며, 실제 G1의 CoM·Jacobian·centroidal map 계산은 Pinocchio에 맡깁니다.

[CasADi 공식 문서](https://web.casadi.org/docs/)의 Opti stack과 Multiple Shooting 설명, [실행 안내](../README.ko.md)을 함께 참고하세요.
