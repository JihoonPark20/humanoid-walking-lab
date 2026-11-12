# Lab 7 학습 설계와 모듈 연결

이 실습은 W0 기준 보행, W1 오프라인 LIPM 최적화, W2 발 디딤 위치와 시간 후보 비교를 구현합니다. W3 Centroidal 최적화, W4 MPC, W5 Whole-body optimal control은 각각 Lab 8–10에 구현되어 있습니다.

## 구현 순서와 검증

1. **LIPM과 접촉 일정**: piecewise-constant jerk의 정확한 이산 Dynamics, 고정 yaw의 평지 발판, single/double support를 정의합니다. 적분식과 경계 연속성을 검증합니다.
2. **오프라인 QP**: 시작 상태와 종료 정지 조건, 지지 영역 제약, ZMP 추종·jerk 비용을 구성합니다. Solver 종료 외에 Dynamics·제약 잔차와 구간 내부의 ZMP extrema를 검증합니다.
3. **발 디딤 최적화**: 발 좌표를 QP 변수에 포함하고 전진 거리·발 사이 간격·착지 허용 영역을 제한합니다. 여러 접촉 시간 후보의 성공 여부와 비용을 모두 기록합니다.
4. **실습 CLI**: 정해진 발걸음과 최적화된 발걸음의 NPZ·CSV·JSON·그림을 생성합니다. 결과 읽기와 재생 방법을 제공합니다.
5. **G1 연동**: 기존 Lab 6의 torque 모델·Whole-body QP·접촉 처리를 재사용합니다. 계획된 궤적을 feedback으로 추종하고 기준 DCM 보행과 비교합니다. 계획 검증과 실제 보행 판정은 분리합니다.

## 모듈과 데이터 흐름

| 모듈 | 역할 |
|---|---|
| `walking_model.py` | 설정, phase별 접촉, footstep reference, exact jerk dynamics |
| `lipm_optimizer.py` | 상태·jerk·footstep QP, infeasible 처리, 연속 구간 제약 검증 |
| `walking_artifacts.py` | NPZ 저장·읽기, CSV·JSON·plot·animation |
| `offline_walk.py` | W1 고정 발 디딤과 W2 위치·시간 최적화 CLI |
| `g1_walk.py` | Lab 6 adapter, baseline/optimized G1 실행과 측정 |

```text
초기 상태 + 발 위치 + 접촉 시간 + 목표 거리
    → WalkingSchedule → sparse QP → WalkingPlan
    → 독립 Dynamics/구간 내부 검증 → NPZ/CSV/plot
    → G1 reference adapter → Lab 6 Whole-body QP → torque → MuJoCo
```

고정 접촉 순서를 사용합니다. Double support의 허용 영역은 두 발 중심의 주어진 비율로 이동하는 작은 rectangle로 정의합니다. 이는 실제 두 발 convex hull 안의 보수적인 부분집합입니다. 발 좌표와 영역 중심의 관계가 affine이므로 W2도 시간 후보 하나당 convex QP입니다. 접촉 시간은 연속 최적화하지 않고 후보를 전부 비교합니다.

## 주요 인터페이스

- `build_schedule(config, initial_feet=None)` → `WalkingSchedule`
- `solve_walk(schedule, initial_state=None, optimize_feet=False, landing_bounds=None)` → `WalkingPlan`
- `WalkingPlan.sample(t)` → 위치·속도·가속도·ZMP·DCM reference
- `validate_plan(plan)` → Dynamics 잔차, 연속 구간 support 위반, 발 디딤·경계조건 검증

코어 최적화에는 외부 로봇 모델이 필요 없습니다. G1 실행은 `third_party/mujoco_menagerie/unitree_g1/`의 MJCF와 Lab 5·6의 모델 로더를 사용합니다. Pinocchio가 G1의 FK·CoM·Jacobian·Dynamics를 계산하고, MuJoCo는 실제 운동과 접촉을 실행·측정합니다.

## W1의 수식과 결정변수

각 수평 축에 대해 `x = [c, v, a]`로 두고 한 구간의 jerk `u`를 상수로 둡니다. 시간 간격 `h`에서 다음 적분식은 근사가 아니라 정확한 polynomial 적분입니다.

```text
c[k+1] = c[k] + h v[k] + h²/2 a[k] + h³/6 u[k]
v[k+1] = v[k] + h a[k] + h²/2 u[k]
a[k+1] = a[k] + h u[k]

ω² = g / z_com
ZMP = c − a / ω²
DCM ξ = c + v / ω
ξ_dot = v + a / ω = ω (ξ − ZMP)
```

`walking_model.jerk_matrices()`와 `polynomial_state()`를 먼저 읽으세요. `WalkingPlan.sample()`은 이 식으로 제어 주기의 reference를 생성합니다. 상태를 단순 선형 보간하면 ZMP/DCM 관계를 깨뜨릴 수 있습니다.

QP 변수는 모든 node의 `(c,v,a)` x/y 상태, interval의 x/y jerk, 모든 발목의 x/y 위치입니다. W1은 발 좌표를 equality constraint로 고정합니다. W2는 처음 두 발만 고정하고 이후 착지를 최적화합니다.

목적함수는 다음 세 항입니다.

```text
J = w_zmp ∫ ||ZMP(t) − support_center(t)||² dt
  + w_jerk ∫ ||jerk(t)||² dt
  + w_foot Σ ||foot_i − nominal_foot_i||²
```

ZMP와 지지 중심의 차이는 구간별 cubic이므로 제곱은 6차입니다. 4점 Gauss quadrature로 그 적분을 정확히 계산합니다. QP 구성과 `plan_cost()`가 동일한 식을 사용하므로 fixed/optimized의 비용을 직접 비교할 수 있습니다. 다만 전체 점수는 가중치에 의존하며 전기 에너지나 기계적 일과는 다릅니다.

경계조건은 초기 `(c,v,a)`, 마지막 `v=a=0`, 마지막 CoM이 최종 발 중심에 발바닥 offset을 더한 위치에 오는 것입니다. 발목 기준 x와 발바닥 중심 기준 x의 차이는 기본 0.035 m입니다. 이는 G1 모델의 발 형상을 참고한 값이며 모든 로봇에 공통인 상수가 아닙니다.

## 지지 영역과 연속시간 검증

Single support에서는 지지발의 작은 직사각형 안에 ZMP를 둡니다. Double support에서는 두 발 중심의 convex combination을 시간에 따라 이동시킵니다. 고정 yaw·같은 크기의 두 발 직사각형에 대해 이 이동 영역은 두 발 convex hull 내부에 있습니다. 전체 convex hull을 사용하지 않으므로 어떤 실제 가능한 보행도 이 보수적 제한에서는 infeasible일 수 있습니다.

QP는 처음에 interval의 시작·중간·끝 ZMP 제약을 풉니다. `validate_plan()`은 추가로 `ZMP − moving_support_center`의 미분 실근을 찾아 구간 내부 extrema를 검사합니다. 속도의 extrema도 검사합니다. 허용 오차 `2e-5`보다 위반이 크면 해당 시각의 제약을 추가하고 최대 8회 재풀이합니다. 이 오차는 각 항의 SI 수치 기준이며, 위치에서는 20 μm입니다. Solver의 `solved` 상태만으로 성공을 판정하지 않습니다.

초기 구현에서 발 반폭을 15 mm로 제한했을 때 기본 보행이 infeasible이었습니다. 20 mm로 두면 동일한 시간표에서 계획을 얻습니다. 이는 실제 발 폭을 늘리는 코드가 아니라 **계획이 사용할 수 있는 기존 발 내부 영역**을 바꾸는 것입니다. 모델의 실제 contact footprint보다 크게 설정해서 성공시키지 마세요.

## W2 착지 허용 영역 실습

저장소 최상위에서 다음 Python 코드를 실행합니다. 첫 착지 x를 0.12–0.125 m로 제한합니다. 기본 고정 착지는 0.1 m이므로 고정 발걸음은 이 제약을 만족하지 못합니다.

```python
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path('lab-7-walking-optimization/src').resolve()))
from walking_model import WalkConfig, build_schedule
from lipm_optimizer import solve_walk, validate_plan
from walking_artifacts import export_plan

schedule = build_schedule(WalkConfig())
# shape: (착지 수, x/y, lower/upper); 초기 두 발은 포함하지 않음
bounds = np.tile([-np.inf, np.inf], (len(schedule.feet) - 2, 2, 1))
bounds[0, 0] = [0.12, 0.125]
plan = solve_walk(schedule, optimize_feet=True, landing_bounds=bounds)
print(plan.feet[2], validate_plan(plan))
export_plan(plan, Path('/tmp/landing-box'))
```

한 개의 convex box만 표현합니다. 구멍 있는 지형, 여러 개 중 하나를 고르는 발판, 장애물 회피의 비볼록 영역은 이 제약으로 표현되지 않습니다. W2의 보폭·폭 제한도 전신 IK 또는 self-collision을 대신하지 않습니다.

## G1 연결을 읽는 순서

1. `g1_walk.run()`은 `m3_walking.build()`를 호출해 torque 모델과 controller를 안정화합니다.
2. Pinocchio의 실측 초기 발·CoM·속도로 새 계획을 만듭니다. CoM 높이는 발목 frame이 아닌 MuJoCo 바닥 plane을 기준으로 계산합니다. 축소 모델의 가속도 초기값은 0으로 가정합니다.
3. `OptimizedGait`는 Lab 6 `GaitSchedule`을 상속해 시간표와 발 목표만 바꿉니다. 내부 `_foot_state`, `_swing_of_phase` 형식에 의존하며 adapter 테스트로 착지/접촉 일치를 검사합니다.
4. `OptimizedReference`는 plan의 `xi`, `xi_dot`, ZMP를 기존 `DCMReference`로 전달합니다.
5. Lab 6이 Pinocchio Dynamics로 contact wrench와 joint torque를 풀고, MuJoCo가 step과 접촉을 수행합니다. 기록 시점은 step 이후이며 Pinocchio 상태를 같은 시점으로 갱신합니다.

Swing의 수평 raised cosine와 수직 sine 궤적은 Lab 6 구현을 재사용합니다. 수직 sine의 양 끝점 속도는 0이 아닙니다. 따라서 이 실습을 착지 충격이 제거된 최적 swing으로 해석하면 안 됩니다. 향후 zero-endpoint-velocity swing과 orientation tracking을 비교할 수 있습니다.

추종기의 ZMP clamp는 실제 발 위치에 따라 갱신되므로 계획의 ZMP와 같다고 가정하지 않습니다. 계획/실측 차이는 `telemetry.csv`와 contact 측정으로 확인합니다. Lab 9의 MPC는 고정 접촉 순서에서 상태와 wrench를 재계획합니다. 접촉 순서 자체를 자동으로 선택하는 기능은 포함하지 않습니다.
