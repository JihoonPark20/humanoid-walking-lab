# 安装

[English](SETUP.md) · [한국어](SETUP.ko.md) · [简体中文](SETUP.zh-CN.md)

在仓库根目录运行。主 CPU 环境支持基础与行走实验，已在 macOS arm64 验证。使用 Python 3.11+ 和 Git。

```bash
python3 -m venv .venv
source .venv/bin/activate
bash tools/setup_env.sh
```

安装器使用 `requirements.txt`，仅下载 MuJoCo Menagerie revision `370c43965323c0944b433b202aac5058b5f907a9` 中的 UR5e、Robotiq、G1，并创建模型链接。仅安装模型：

```bash
bash tools/setup_env.sh --models-only
```

安装包名是 `pin`，import 名是 `pinocchio`。Lab 4 可选 `toppra`，未安装时采用 quintic 时间参数化。

Lab 11–13 使用主环境的 ProxQP（`proxsuite==0.8.2`），已验证 macOS arm64 wheel。更新现有 checkout 后，请再次执行 `python -m pip install -r requirements.txt`。

## Whole-body OCP

Lab 10 在独立环境使用 Pinocchio 4.0.0 和 Crocoddyl 3.2.1，请与主环境 Pinocchio 4.1.0 分离。

```bash
python3 -m venv .venv-ocp
.venv-ocp/bin/python -m pip install -r lab-10-whole-body-optimal-control/requirements.txt
.venv-ocp/bin/python lab-10-whole-body-optimal-control/src/whole_body_demo.py --steps 2 --mujoco --video
```

## 渲染

- macOS：passive viewer 使用 `mjpython`，offscreen 录制使用 `python`。
- Linux headless：安装 EGL runtime，并在录制命令设置 `MUJOCO_GL=egl`。
- 数值测试不需要渲染 backend。录制请逐个执行。

## 媒体与发布

各 Lab 提供代表演示与指标。录制后重新生成 preview：

```bash
python tools/build_gallery.py
python tools/build_release.py --output ../humanoid-walking-lab.zip
```

压缩包排除虚拟环境、外部模型、缓存与开发状态。解压后用安装脚本获取模型。发布工具检查文档链接、各语言笔记、验证矩阵与视频证据。
