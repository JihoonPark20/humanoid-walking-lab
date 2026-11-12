# 실습 중 문제가 생겼을 때

[English](TROUBLESHOOTING.md) · [한국어](TROUBLESHOOTING.ko.md) · [简体中文](TROUBLESHOOTING.zh-CN.md)

## Import 또는 모델 로딩 실패

- `import pinocchio`가 실패하면 가상 환경에서 `python -m pip show pin`으로 설치 위치를 확인합니다. 설치 이름과 import 이름이 다릅니다.
- Mesh나 XML을 찾지 못하면 `bash tools/setup_env.sh`로 외부 Menagerie 모델을 내려받았는지 확인합니다. 저장소 파일만 복사해서는 외부 모델이 설치되지 않습니다.
- Lab 2의 `models/assets` symbolic link와 각 Lab 폴더의 상대 위치를 유지합니다.
- 다른 Lab의 같은 이름 모듈이 import되면 `module.__file__`을 확인합니다. 최상위 `conftest.py`는 pytest 수집 시 이 충돌을 처리하므로 유지해야 합니다.

## 렌더링 실패

Linux headless 실행은 `MUJOCO_GL=egl`과 EGL runtime이 필요합니다. macOS에서는 EGL을 일괄 설정하지 말고, passive viewer 실행에 `mjpython`을 사용합니다. 먼저 계산만 하는 테스트가 통과하는지 확인한 뒤 렌더링 문제를 분리해서 살펴보세요.

## Pinocchio와 MuJoCo의 결과 불일치

1. 관절 이름과 배열 순서, 같은 시점의 `q`·`qdot`을 사용하는지 확인합니다.
2. 같은 지점의 frame/body/site인지 확인합니다. Flange와 tool 끝은 다릅니다.
3. Quaternion 순서를 변환합니다: MuJoCo `(w,x,y,z)` ↔ Pinocchio `(x,y,z,w)`.
4. 단위, gravity 방향, tool payload, 질량·관성, armature를 확인합니다.
5. Pinocchio의 FK와 frame placement, MuJoCo의 `mj_forward`로 파생 상태를 갱신합니다.
6. Jacobian과 오차 벡터의 표현 frame을 맞춥니다. 이 저장소의 Pinocchio spatial vector는 `[linear, angular]` 순서입니다.

## IK는 성공했는데 장면에서 충돌

IK는 목표 pose만 맞출 수 있습니다. 관절각을 설정한 뒤 `mj_forward`로 접촉을 갱신하고 contact pair를 확인하세요. 경로 계획에서도 endpoint뿐 아니라 edge 중간을 검사해야 합니다. 인접 link 제외 규칙은 해당 collision model에 맞게 적용합니다.

## Humanoid가 넘어짐

중력 보상만으로 자세가 안정해지지는 않습니다. CoM·DCM 오차, 발 접촉, pelvis 높이, torque saturation을 시간순으로 비교합니다. 새 장면에서는 탁자·선반과 link의 예상치 못한 충돌도 확인하세요. QP는 solver status와 residual을 함께 읽고 iteration cap에 걸린 해를 정상 수렴으로 해석하지 않습니다.

[전체 안내](../README.ko.md)
