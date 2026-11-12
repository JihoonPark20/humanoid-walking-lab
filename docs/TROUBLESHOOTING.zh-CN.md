# 故障排查

[English](TROUBLESHOOTING.md) · [한국어](TROUBLESHOOTING.ko.md) · [简体中文](TROUBLESHOOTING.zh-CN.md)

## Import 与模型加载

- `import pinocchio` 失败时，在当前虚拟环境运行 `python -m pip show pin`；安装包名与 import 名不同。
- 缺少 mesh/XML 时运行 `bash tools/setup_env.sh` 安装外部 Menagerie 模型。只复制仓库不会安装模型。
- 保留 Lab 2 的 `models/assets` symbolic link 与各 Lab 相对目录。
- 同名模块误从其他 Lab 导入时检查 `module.__file__`。根目录 `conftest.py` 负责 pytest 收集时的冲突处理，应保留。

## 渲染

Linux headless 录制需要 EGL runtime 和 `MUJOCO_GL=egl`。macOS 不要全局设置 EGL，passive viewer 使用 `mjpython`。先运行数值测试，区分模型问题与渲染问题。离屏录制应顺序执行。

## Pinocchio 与 MuJoCo 不一致

1. 对齐关节名称/顺序以及 `q`/`qdot` 时间戳。
2. 比较同一 frame/body/site；flange 与 tool tip 不同。
3. 转换 quaternion 顺序：MuJoCo `(w,x,y,z)` ↔ Pinocchio `(x,y,z,w)`。
4. 对齐单位、重力、payload、质量/惯量和 armature。
5. 更新 Pinocchio FK/frame placement 与 MuJoCo `mj_forward`。
6. Jacobian 与误差使用同一 frame；本仓库 Pinocchio spatial vector 顺序为 `[linear,angular]`。

## IK 成功但发生碰撞

IK 可能只满足目标 pose。设置关节后调用 `mj_forward` 并查看 contact pair。规划应检查 edge 内部，而不只检查端点。相邻 link 排除规则依赖具体 collision model。Lab 4 还验证时间参数化轨迹与实际执行。

## Humanoid 跌倒

单独 gravity compensation 不能稳定姿态。按时间比较 CoM/DCM 误差、足接触、pelvis 高度和 torque saturation。检查意外的环境/link 碰撞。结合 QP 状态与物理残差判断；达到 iteration cap 不等于正常收敛。

[总览](../README.zh-CN.md)
