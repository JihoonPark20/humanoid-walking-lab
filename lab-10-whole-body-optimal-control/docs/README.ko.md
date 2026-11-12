# W5: G1 Whole-body optimal control

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

## Torque regularization

FDDP는 `control_weight=3.0`으로 계획합니다. Torque 비용은 nominal 입력과 feedback gain을 함께 바꾸며, 관절·접촉 제약은 별도로 검증합니다.

비용 변경은 단순히 실행 시 gain을 줄이는 것이 아닙니다. OCP를 다시 풀어 nominal torque와 gain을 함께 바꿉니다. `OCPConfig.control_weight`가 실제 비용에 적용되고 저장 metadata에도 포함됩니다. 비용이 다른 두 실행의 objective 숫자를 그대로 성능 순위로 비교하지 않습니다.

권장 프로파일은 FDDP 비용 3.0, blended/joint PD 비용 0.1입니다. 동일 비용에서 정책만 비교하려면 `--control-weight`를 명시하세요. 모든 임의의 비용·gain 조합이 성공한다는 뜻은 아닙니다. `validate_tracking_scenarios.py`는 이 세 권장 정책 × 한두 걸음 × 10/20 ms의 12개 프로파일을 기록합니다.

## 실제 추종 정책

기본 최적화 간격은 10 ms이고 실제 MuJoCo 적분·추종은 1 ms입니다. 기본 정책은 FDDP이며, 선택 가능한 혼합 정책의 식은 다음과 같습니다.

```text
e = state.diff(x_nominal, x_actual)
du_fddp = -K e
du_pd = -400 e_joint_position - 20 e_joint_velocity
u = u_ff + alpha du_fddp + (1-alpha) du_pd
```

`blended`의 기본값은 `alpha=0.1`입니다. `fddp`는 gain 전체를, `joint-pd`는 joint PD만 사용합니다. 혼합은 국소 모델의 큰 feedback 보정을 줄이고 관절 위치·속도 추종을 유지하기 위한 실험적 선택입니다. Base feedback은 FDDP의 관절 torque 경로에 남습니다. 접촉 추정이나 online 재최적화를 추가한 controller는 아니며, 안정성 증명을 제공하지 않습니다.

통과 기준은 계획의 수렴·독립 제약 검사, 실제 계획 시간 완료, 전도 없음, 무접촉 tick 0, 최종 발 오차 3 cm 미만입니다. 실제 상태를 nominal로 덮어쓰거나 torque 제한·판정 기준을 완화하지 않습니다. 한두 걸음 기본 회귀 테스트는 추가로 torque clipping 0과 발 오차 2 cm 미만을 확인합니다.

Pinocchio 4.0의 MJCF 로더는 이미 `model.armature`를 채웁니다. Crocoddyl action의 별도 `armature`에 같은 값을 더하면 중복 가산됩니다. `audit_mass_matrices()`는 free-flyer 속도 좌표계를 맞춘 뒤 MuJoCo, Pinocchio, contact/impact action의 mass matrix가 일치하는지 검사합니다. 마찰 손실과 compliant contact가 rigid-contact 계획과 완전히 같다는 뜻은 아닙니다.

## 구현 순서와 구조

1. 기존 torque G1과 Pinocchio model을 불러와 floating-base state와 actuator 제한을 맞춥니다.
2. Crocoddyl의 contact Dynamics·impact action model로 한 걸음과 양발 정지를 구성합니다.
3. Box-FDDP로 joint torque 제한을 적용하며 state/contact cost를 최적화합니다.
4. `state.diff`/`pin.integrate`를 사용한 feedback rollout과 MuJoCo rollout gap을 비교합니다.

- `whole_body_ocp.py`: 모델 준비, Crocoddyl shooting problem, Box-FDDP, state·control·feedback gain 저장, 독립 rollout 검사.
- `whole_body_demo.py`: CLI, 지표·그래프, 선택적 MuJoCo torque 재생 및 영상.
- `whole_body_tracking.py`: FDDP·joint PD·혼합 feedback과 설정 검증.
- `validate_tracking_scenarios.py`: 각 정책의 권장 계획 비용을 명시한 12개 프로파일과 실제 물리 추종 판정.

Lab 6의 동일 G1 MJCF/Pinocchio loader를 재사용합니다. Crocoddyl은 Pinocchio 분석 Dynamics를 사용하고 MuJoCo는 별도의 실제 접촉 적분을 수행합니다. Optimization contact model에서의 feasible rollout과 MuJoCo의 compliant contact는 같지 않으므로 결과를 구분합니다.

Torque box는 solver가 직접 제한합니다. 관절 범위와 wrench는 barrier cost만으로 엄밀하게 보장되지 않으므로 결과를 검사하고 위반을 보고합니다. Contact-implicit 순서 탐색과 장애물 충돌 회피는 범위 밖입니다.

## 상태·접촉·feedback

상태는 `x=(q,v)`이며 `q`는 floating-base quaternion을 포함합니다. `nq=36`, `nv=35`이므로 저장 상태는 71차원이고 `dx`는 70차원입니다. 비용과 feedback은 manifold 차이로 계산합니다.

```text
M(q) v_dot + h(q,v) = S.T u + J(q).T lambda
J(q) v_dot + J_dot(q,v) v = 0
u = u_nominal - K * state.diff(x_nominal, x_measured)
```

Contact action은 고정된 stance의 가속도 조건을 풉니다. Impact action은 시간이 흐르지 않은 채 위치를 유지하고 접촉 속도를 투영합니다. MuJoCo에서는 이 속도 reset을 적용하지 않으며, contact Dynamics가 실제 속도 변화를 결정합니다.

모델 로딩·floating-base joint 이름·패키지 조합의 주의점과 실제 결과는 [실행 안내](../README.ko.md)에 있습니다.
