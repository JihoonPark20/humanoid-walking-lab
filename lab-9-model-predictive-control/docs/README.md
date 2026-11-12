# W4: 측정 상태로 다시 계획하는 MPC

## 지연·모델 오차 보완

기존 `mass_ratio=1.05`, 마찰 0.4, 측정 1주기 지연, 외란 0.03 m/s를 그대로 유지합니다. Timestamp가 있는 과거 측정값과 이미 보낸 command 이력으로 현재 상태를 예측합니다. 아직 측정하지 못한 외란은 예측할 수 없으며 다음 측정에 반영됩니다. Controller에는 현재 plant 상태나 실제 질량을 전달하지 않습니다.

질량 식별은 연속된 세 CoM 높이 측정과 두 interval의 수직 command만 사용합니다. 높이를 `z0,z1,z2`, interval 길이를 `h0,h1`, 두 발의 합력 command를 `F0,F1`이라 하면, constant-wrench Dynamics에서 다음 관계를 얻습니다.

```text
v_bar0 = (z1-z0)/h0
v_bar1 = (z2-z1)/h1
m_candidate = (F0*h0 + F1*h1) / (2*(v_bar1-v_bar0) + g*(h0+h1))
m_hat <- m_hat + 0.3*(m_candidate-m_hat)
```

후보가 nominal 질량의 0.75–1.25배 범위 밖이면 거부합니다. 이 범위는 실제 질량을 읽어 선택하지 않으며 모든 프로파일에서 같습니다. 중복 timestamp는 재사용하고 순서가 역행하면 거부합니다. 측정이 누락되면 연속된 세 측정이 모일 때까지 새 추정을 보류합니다. 수직 외력이나 force command 오차는 질량과 구별되지 않으므로 이 방식의 적용 조건을 벗어납니다.

`CentroidalMPC.update_mass()`는 solver graph의 mass parameter와 같은 nominal 속도에 해당하는 reference momentum·wrench를 갱신합니다. 이전 계획의 fallback도 새 모델에서 다시 rollout·검증합니다. RTI는 질량이 바뀌면 준비된 QP를 무효화하며, reduced 실험에서는 측정 → 추정 → preparation → feedback 순서로 한 번의 QP를 풉니다. 고정 질량인 G1 실험의 advance preparation 경로는 유지합니다.

`validate_mpc_scenarios.py`는 8가지 환경 × full NLP/RTI의 16개 프로파일을 검증합니다. 모든 interval 완료와 최종 CoM 오차 3 cm 미만, 실제 plant 상태·wrench의 독립 검증을 유지합니다. `command_wrench_violation`과 `saturation_intervals`는 plant가 명령을 얼마나 제한했는지도 보여줍니다.

## RTI의 preparation과 feedback

`centroidal_rti.py`는 CasADi 미분 + OSQP 기반 single-SQP 경로입니다. `prepare(index)`는 이전 해를 shift하고 exact discrete Dynamics의 Jacobian, quadratic tracking cost, 선형 wrench/상태 제약을 구성합니다. `feedback(measured)`는 초기 상태 equality의 RHS만 갱신하고 **QP를 한 번** 풉니다. 비선형 입력 rollout을 별도로 검증하며, 유효하지 않으면 새 측정 상태에서 검증된 이전 입력만 fallback으로 허용합니다. Full NLP를 재호출하지 않습니다.

결정변수 전체를 `z`, 이전 해를 `z_bar`, nonlinear constraint를 `g`라 하면 한 번의 SQP 문제는 다음과 같습니다.

```text
min_delta  0.5 delta.T H delta + grad(f).T delta
subject to l <= g(z_bar) + J_g(z_bar) delta <= u
           delta_x0 = x_measured - x_bar0
```

Tracking objective가 quadratic이므로 `H=Hessian(f)`는 Gauss–Newton Hessian과 같습니다. 비선형 제약의 Hessian은 사용하지 않습니다. 구현은 `z=z_bar+delta` 형태의 absolute QP와 단위 scaling을 사용합니다. Momentum은 질량, wrench는 무게로 정규화하고 제약 행도 scaling합니다. Wrench·상태 한계는 유지하며 nonlinear Dynamics만 선형화됩니다.

QP를 푼 뒤에는 후보 wrench를 exact nonlinear rollout합니다. 이 상태로 독립 검증을 통과해야 입력을 사용합니다. `linearization_gap`은 linearized prediction과 exact rollout의 성분별 최대 차이이며 위치·운동량 단위가 섞여 있습니다. 단일 SQP가 항상 feasible하거나 수렴한다는 보장은 없습니다.

고정 모델의 G1 루프는 다음 주기 `prepare(k+1)`를 미리 호출합니다. 편의 API `solve(measured,index)`는 두 단계를 연속 실행하며 질량 추정을 사용하는 reduced 루프에서 사용합니다. 매 preparation에 sparse QP와 factorization을 새로 만들고 feedback에서 RHS만 갱신합니다. Horizon 간 factorization 재사용·condensing·코드 생성은 구현하지 않았습니다. 계측은 preparation/feedback/상태 추정 및 합계를 기록하며 초기 graph 생성은 제외합니다.

[acados 공식 RTI 설명](https://docs.acados.org/features/)의 preparation/feedback 구분을 따르는 교육용 구현이며 acados backend를 사용하는 것은 아닙니다. 실시간 OS나 hard deadline 보장은 범위 밖입니다.

## 구현 순서

1. LIPM의 exact ZMP-input Dynamics와 유한 horizon QP를 구성합니다.
2. 최종 양발 capture 영역에서 backward recursion으로 terminal DCM viability 영역을 구합니다.
3. 같은 외란에서 offline feedback과 MPC를 비교하고 solver 시간·실패를 저장합니다.
4. Lab 8의 centroidal NLP를 receding horizon으로 재사용합니다. Warm start를 shift하고 매번 초기 상태를 측정값으로 교체합니다.
5. CoM·운동량 reference를 Lab 6 Whole-body QP에 연결해 MuJoCo 추종을 측정합니다.

## 모듈·데이터 흐름

- `lipm_mpc.py`: 상태 `[c_x,v_x,c_y,v_y]`, ZMP 입력, OSQP horizon, terminal DCM viability 영역, braking fallback.
- `centroidal_mpc.py`: Lab 8 모델/solver, shifted primal warm start, 상태 추정 지연·질량·마찰 오차 실험.
- `centroidal_rti.py`: 같은 NLP graph의 자동 미분, one-QP RTI, nonlinear rollout 검사와 split timing.
- `centroidal_estimation.py`: timestamp 기반 상태 예측과 CoM 높이·force command 기반 질량 추정.
- `validate_mpc_scenarios.py`: 고정된 16개 환경·알고리즘 조합의 실행 및 판정표.
- `mpc_experiment.py`: 동일 외란의 open-loop/offline feedback/MPC reduced-model rollout, CSV·JSON·plot와 solve-time median/p95/p99/deadline miss.
- `g1_mpc.py`: Pinocchio 실측 상태 → centroidal MPC → Lab 6 CoM/angular-momentum task → Whole-body QP → MuJoCo.

실패 처리: infeasible 해를 적용하지 않습니다. LIPM은 capture point가 현재 보수적 지지 영역 안에 있을 때 검증된 정지 입력을 사용합니다. 그렇지 않으면 중단합니다. Centroidal MPC는 새 측정 상태에서 재검증한 이전 계획만 fallback으로 허용하고, 사용할 수 없으면 중단합니다. 실패 횟수와 사용한 경로를 숨기지 않습니다.

Centroidal MPC는 full NLP와 RTI를 모두 제공합니다. 같은 nominal·외란·horizon에서 오차와 전체 계산 시간을 비교합니다.

## Terminal DCM 영역

`omega=sqrt(g/h)`, `xi=c+v/omega`이면 constant ZMP `z`에 대해 `xi_next=exp(omega*dt)*xi+(1-exp(omega*dt))*z`입니다. 따라서 미래 capture 구간과 ZMP box에서 이전 구간을 다음과 같이 구합니다.

```text
alpha = exp(-omega * dt)
lower[k] = alpha * lower[k+1] + (1-alpha) * zmp_lower[k]
upper[k] = alpha * upper[k+1] + (1-alpha) * zmp_upper[k]
```

MPC horizon 끝의 DCM을 해당 시각의 구간 안에 둡니다. 이 계산은 고정 접촉 일정·분리된 수평축·constant-input LIPM에 대한 것입니다. 전체 G1의 관절·토크 feasibility를 뜻하지 않습니다.

[실험 지표](../GALLERY.md)에서 reduced model과 G1의 결과를 구분해 읽으세요.
