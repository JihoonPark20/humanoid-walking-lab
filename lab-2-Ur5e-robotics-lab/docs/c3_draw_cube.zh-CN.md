# C3：跟踪立方体的十二条棱

[English](c3_draw_cube.md) · [한국어](c3_draw_cube.ko.md) · [简体中文](c3_draw_cube.zh-CN.md)

UR5e 连续访问全部十二条棱。立方体有八个奇数度顶点，因此不可能连续且恰好一次经过每条棱。本演示重复三条棱，共 16 个顶点、15 段、30 s reference。相邻顶点必须由真实棱连接，回归测试拒绝对角线连接。

## 流程

`cube_vertices()` 生成中心 `[0.3,0.2,0.55]`、边长 0.10 m 的立方体。DLS IK 固定 home orientation，使用上次解作为下一初值。`build_cube_trajectory()` 以 2 s quintic 连接 joint pose，并消除重复 timestamp。由于采用 joint-space 插值，末端 reference 可能略微偏离几何直棱。

Menagerie position servo 近似为 `tau=Kp(ctrl-q)-Kd*qd`。在位置命令加入 bias/速度 feedforward：

```text
ctrl = q_des + bias(q,qd)/Kp + Kd*qd_des/Kp
```

Recorder 使用 MuJoCo `qfrc_bias`，其中除重力外还含 Coriolis；跟踪前稳定 3 s，不禁用场景 contact。

## 验证

视频与[指标](../media/c3_draw_cube.json)来自同一次运行。标准为访问全部十二条棱且没有对角线、相对 joint-reference FK 的 RMS <1 mm/peak <5 mm、实际末端到预期几何棱的距离 <5 mm、MuJoCo 接触为零、完整执行。Reference FK 跟踪误差与几何直棱偏差是不同指标。

```bash
# 在 Lab 目录运行 macOS interactive viewer：
mjpython src/c3_draw_cube.py
# Offscreen 录制：
python src/c3_record_video.py
```

固定场景与速度，比较简单 position control 与 bias/velocity feedforward。同时检查 actuator 限制、实际棱偏差与接触。[Gallery](../GALLERY.zh-CN.md) 为完整运行、3 倍速。
