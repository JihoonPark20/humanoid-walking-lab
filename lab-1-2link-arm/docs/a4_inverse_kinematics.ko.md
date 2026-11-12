# A4: Inverse Kinematics — Study Notes

[English](a4_inverse_kinematics.md) · [한국어](a4_inverse_kinematics.ko.md) · [简体中文](a4_inverse_kinematics.zh-CN.md)

IK는 목표 `(x,y)`를 만족하는 `(θ1,θ2)`를 구합니다. Site offset 때문에 유효 길이는 `L1=0.3`, `L2_eff=0.315` m이고 도달 반경은 0.015–0.615 m입니다.

```text
cos(θ2) = (x²+y²-L1²-L2_eff²)/(2 L1 L2_eff)
θ2 = ±acos(cos(θ2))
θ1 = atan2(y,x)-atan2(L2_eff sin(θ2), L1+L2_eff cos(θ2))
dq_DLS = J.T (J J.T + λ² I)^(-1) e
```

± 분기는 서로 다른 elbow 자세입니다. 이름보다 실제 q와 FK를 확인하고 이전 해에 가까운 분기를 선택해 연속성을 유지하세요. 수치 `pinv`는 singularity에 민감하고 DLS는 damping으로 큰 갱신을 줄입니다. DLS도 모든 목표에서 수렴하지는 않습니다.

`a4_inverse_kinematics.py`는 해석해 FK roundtrip, 20개 목표 benchmark, singularity stress, 선택적 MuJoCo site 검증을 수행합니다. 결과는 `a4_ik_benchmark.csv`입니다. 도달 불가능한 목표는 실패로 처리하고, 성공 여부·반복 수·최종 오차를 함께 비교하세요.

[학습 노트](README.ko.md) · [실행 안내](../README.ko.md)
