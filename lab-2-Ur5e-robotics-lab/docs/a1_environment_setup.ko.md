# A1: Environment Setup

[English](a1_environment_setup.md) · [한국어](a1_environment_setup.ko.md) · [简体中文](a1_environment_setup.zh-CN.md)

`a1_model_setup.py`는 MuJoCo MJCF와 Pinocchio URDF가 같은 UR5e를 표현하는지 확인합니다. `ur5e_common.py`에 경로·관절 이름·한계·Q_HOME·DH 상수를 모으고 `mujoco_sim.py`의 `UR5eSimulator`로 상태·torque·끝점 접근을 통일합니다.

기본 의존성은 NumPy, MuJoCo, Pinocchio, Matplotlib입니다. 모델 설치는 저장소의 setup script를 사용하세요. Lab 폴더의 `models/lab_scene.xml`, `models/ur5e.urdf`와 Menagerie mesh 링크를 유지합니다.

```bash
python src/a1_model_setup.py
```

Lab 폴더에서 실행해 모델 크기·joint 목록·Q_HOME에서의 step을 확인합니다. MuJoCo `attachment_site`와 Pinocchio `ee_link`의 같은 q 위치를 비교하며 기준은 1 mm 이내입니다. 결과는 `a1_environment_status.csv`입니다. 이 frame 대응이 틀리면 이후 IK·Jacobian·제어 비교도 의미가 없어집니다.

[학습 노트](README.ko.md) · [실행 안내](../README.ko.md)
