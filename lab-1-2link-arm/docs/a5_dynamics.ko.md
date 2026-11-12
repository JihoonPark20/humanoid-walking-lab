# A5: Dynamics Basics — Study Notes

[English](a5_dynamics.md) · [한국어](a5_dynamics.ko.md) · [简体中文](a5_dynamics.zh-CN.md)

기본 식은 `tau=M(q) qdd + C(q,qd) qd + g(q)`입니다. M은 자세에 따른 관성 행렬, Cqd는 Coriolis·centrifugal 항, g는 중력 torque입니다. 이 실습은 유도보다 MuJoCo에서 각 항을 읽는 데 초점을 둡니다.

`qfrc_bias`는 Cqd+g를 포함합니다. Mass matrix는 MuJoCo 버전에 맞는 `mj_fullM`으로 얻습니다. 오래된 API의 packed `qM`과 현재 API 호출 형식을 혼동하지 마세요.

XY 평면 팔의 joint 축은 z이므로 z방향 중력은 joint torque에 거의 나타나지 않습니다. `a5_dynamics_basics.py`에서 무중력·z중력·y중력을 비교하고 `[0°,0°]`와 `[90°,-90°]`의 M을 비교합니다.

```bash
python src/a5_dynamics_basics.py
```

Lab 폴더에서 실행하면 의존성이 있을 때 `a5_bias_cases.csv`·`a5_mass_matrix_cases.csv`를 생성합니다. 의존성이 없으면 개념 설명만 출력하므로 이를 물리 검증으로 해석하지 않습니다.

[학습 노트](README.ko.md) · [실행 안내](../README.ko.md)
