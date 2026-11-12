# A4: Inverse Kinematics

[English](a4_inverse_kinematics.md) · [한국어](a4_inverse_kinematics.ko.md) · [简体中文](a4_inverse_kinematics.zh-CN.md)

UR5e 수치 IK는 위치와 orientation을 함께 맞춥니다. `IKResult`는 success·q·iterations·최종/위치/회전 오차·elapsed_s를 저장합니다.

```text
e = [p_target-p_current; log3(R_target R_current.T)]
dq_pinv = pinv(J) e
dq_DLS = J.T (J J.T + λ² I)^(-1) e
```

J는 `LOCAL_WORLD_ALIGNED`이며 매 반복 joint 범위로 clamp합니다. 기본 step α=0.5, tol=1e-4입니다. Pinv는 최대 200회, DLS는 300회이며 오차 감소 시 λ를 줄이고 증가 시 키워 `[1e-4,1]`로 제한합니다. 이는 damping 조절이며 전역 수렴 보장은 아닙니다.

`validate_ik_solution()`은 FK roundtrip에서 기본 위치 1 mm·회전 0.01 rad를 검사합니다. MuJoCo에서 같은 q를 설정하고 `mj_forward` 후 site 차이를 확인합니다. 랜덤 q의 FK로 만든 도달 가능한 목표라도 다른 초기값의 국소 solver가 실패할 수 있습니다.

Lab 폴더에서 `python src/a4_inverse_kinematics.py`를 실행하고 50개 목표 benchmark의 성공률·반복 수·시간·오차를 `a4_ik_benchmark.csv`에서 비교하세요. 성공한 IK가 충돌 없음이나 동역학적 실행 가능성을 의미하지는 않습니다.

[학습 노트](README.ko.md) · [실행 안내](../README.ko.md)
