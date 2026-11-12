# A3: Jacobian — Study Notes

[English](a3_jacobian.md) · [한국어](a3_jacobian.ko.md) · [简体中文](a3_jacobian.zh-CN.md)

Jacobian은 FK를 관절각으로 미분한 국소 속도 관계 `x_dot=J(q) q_dot`입니다. 두 link에서 J는 2×2이며 `L2_eff=L2+0.015` m입니다.

```text
J = [[-L1 sin(q1)-L2_eff sin(q1+q2), -L2_eff sin(q1+q2)],
     [ L1 cos(q1)+L2_eff cos(q1+q2),  L2_eff cos(q1+q2)]]
det(J) = L1 L2_eff sin(q2)
```

`q2≈0,π`에서는 determinant가 0으로 가며 특정 끝점 운동 방향을 잃습니다. 작은 Cartesian 명령이 큰 관절 속도를 요구할 수 있습니다.

`a3_jacobian.py`는 해석식·finite difference·MuJoCo `mj_jacSite`를 비교하고 `a3_det_sweep.csv`에 determinant sweep을 기록합니다. 각 방법의 frame과 site를 맞추세요. 의존성 없는 fallback은 MuJoCo 교차 검증을 대신하지 않습니다.

실습: finite-difference 간격을 바꾸며 절단·반올림 오차를 비교하고 singularity에 접근할 때 J의 singular value를 확인하세요.

[학습 노트](README.ko.md) · [실행 안내](../README.ko.md)
