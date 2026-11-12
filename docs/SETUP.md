# Setup

[English](SETUP.md) · [한국어](SETUP.ko.md) · [简体中文](SETUP.zh-CN.md)

Run commands from the repository root. The main CPU environment supports the
fundamentals and walking labs; macOS arm64 is tested. Use Python 3.11+ and Git.

```bash
python3 -m venv .venv
source .venv/bin/activate
bash tools/setup_env.sh
```

The installer uses `requirements.txt` and downloads only UR5e, Robotiq and G1 assets
from MuJoCo Menagerie revision `370c43965323c0944b433b202aac5058b5f907a9`.
It creates the model links used by the arm labs. To install just models:

```bash
bash tools/setup_env.sh --models-only
```

`pin` is the package name; `pinocchio` is the Python import. Lab 4 can optionally
use `toppra`; without it the trajectory code uses quintic parameterization.

Labs 11–13 use ProxQP (`proxsuite==0.8.2`) in the main environment; macOS arm64 wheels were tested. Rerun `python -m pip install -r requirements.txt` when updating an existing checkout.

## Whole-body OCP

Lab 10 uses Pinocchio 4.0.0 and Crocoddyl 3.2.1 in a separate environment.
Keep it separate from the main environment's Pinocchio 4.1.0.

```bash
python3 -m venv .venv-ocp
.venv-ocp/bin/python -m pip install -r lab-10-whole-body-optimal-control/requirements.txt
.venv-ocp/bin/python lab-10-whole-body-optimal-control/src/whole_body_demo.py --steps 2 --mujoco --video
```

## Rendering

- macOS: use `mjpython` for passive interactive viewers, `python` for offscreen recording.
- Linux headless: install an EGL runtime and set `MUJOCO_GL=egl` for the rendering command.
- Numerical tests do not require a rendering backend.

## Media and distribution

Each lab links its selected demo and measurements. Recreate previews after recording:

```bash
python tools/build_gallery.py
python tools/build_release.py --output ../humanoid-walking-lab.zip
```

The archive excludes environments, model downloads, caches and development state.
Robot assets are installed with the setup script after extraction. The publisher
checks local document links and saved validation matrices before packaging.
