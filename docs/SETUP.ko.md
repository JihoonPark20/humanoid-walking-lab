# 설치

[English](SETUP.md) · [한국어](SETUP.ko.md) · [简体中文](SETUP.zh-CN.md)

저장소 최상위에서 실행합니다. 기본 CPU 환경은 기초·보행 실습을 지원하며 macOS arm64에서 검증했습니다. Python 3.11 이상과 Git을 사용합니다.

```bash
python3 -m venv .venv
source .venv/bin/activate
bash tools/setup_env.sh
```

설치기는 `requirements.txt`를 설치하고 MuJoCo Menagerie revision `370c43965323c0944b433b202aac5058b5f907a9`에서 UR5e·Robotiq·G1만 내려받아 모델 링크를 만듭니다. 모델만 설치하려면:

```bash
bash tools/setup_env.sh --models-only
```

설치 이름은 `pin`, import 이름은 `pinocchio`입니다. Lab 4는 선택적으로 `toppra`를 사용하며 없으면 quintic 시간 매개화를 사용합니다.

Lab 11–13은 메인 환경의 ProxQP(`proxsuite==0.8.2`)를 사용하며 macOS arm64 wheel을 검증했습니다. 기존 checkout을 갱신했다면 `python -m pip install -r requirements.txt`를 다시 실행하세요.

## Whole-body OCP

Lab 10은 Pinocchio 4.0.0·Crocoddyl 3.2.1을 별도 환경에서 사용합니다. 기본 환경의 Pinocchio 4.1.0과 분리하세요.

```bash
python3 -m venv .venv-ocp
.venv-ocp/bin/python -m pip install -r lab-10-whole-body-optimal-control/requirements.txt
.venv-ocp/bin/python lab-10-whole-body-optimal-control/src/whole_body_demo.py --steps 2 --mujoco --video
```

## 렌더링

- macOS: passive viewer는 `mjpython`, offscreen 녹화는 `python`을 사용합니다.
- Linux headless: EGL runtime을 설치하고 녹화 명령에 `MUJOCO_GL=egl`을 지정합니다.
- 수치 테스트에는 렌더링 backend가 필요 없습니다. 녹화는 하나씩 실행합니다.

## 미디어와 배포

각 Lab의 대표 데모와 지표를 참고하세요. 녹화 후 preview를 다시 만듭니다.

```bash
python tools/build_gallery.py
python tools/build_release.py --output ../humanoid-walking-lab.zip
```

압축 파일은 가상 환경·외부 모델·캐시·개발 상태를 제외합니다. 압축 해제 후 설치 스크립트로 모델을 설치하세요. 배포 도구는 문서 링크·언어별 노트·검증표·영상 증거를 검사합니다.
