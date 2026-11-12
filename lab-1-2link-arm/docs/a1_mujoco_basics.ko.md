# A1: MuJoCo Environment Setup — Study Notes

[English](a1_mujoco_basics.md) · [한국어](a1_mujoco_basics.ko.md) · [简体中文](a1_mujoco_basics.zh-CN.md)

MuJoCo는 로봇의 Dynamics와 접촉을 계산하는 물리 엔진입니다. MJCF의 `option`은 중력·시간 간격, `worldbody`는 body 계층, `actuator`는 구동기, `sensor`는 측정을 정의합니다. Body는 좌표계와 관성, joint는 부모·자식 간 자유도, geom은 표시·충돌 형상입니다. Site는 관심 지점이며 자체 물리 효과가 없습니다.

이 모델은 길이 0.3 m의 두 link, z축 hinge 두 개, torque motor 두 개를 가진 XY 평면 팔입니다. 끝점 계산에는 0.015 m tool offset을 포함합니다. 부모 joint가 회전하면 그 아래 link도 함께 움직입니다.

`MjModel`은 모델 파라미터, `MjData`는 현재 상태를 담습니다. `qpos`·`qvel`로 관절 상태를 읽고 `ctrl`로 입력합니다. `site_xpos`는 world-frame 끝점입니다. `mj_step`은 시간을 전진시키며 `mj_forward`는 현재 상태의 파생값만 갱신합니다.

```python
import mujoco
model = mujoco.MjModel.from_xml_path('models/two_link.xml')
data = mujoco.MjData(model)
data.ctrl[:] = [5, -3]
mujoco.mj_step(model, data)
mujoco.mj_forward(model, data)
print(data.qpos, data.site_xpos)
```

Lab 폴더에서 실행하세요. 상태 크기, 시간 증가, 입력에 따른 각도 변화, site 위치를 확인한 뒤 A2의 직접 계산과 비교합니다.

[학습 노트](README.ko.md) · [실행 안내](../README.ko.md)
