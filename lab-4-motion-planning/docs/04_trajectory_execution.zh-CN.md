# Trajectory execution

[English](04_trajectory_execution.md) · [한국어](04_trajectory_execution.ko.md) · [简体中文](04_trajectory_execution.zh-CN.md)

`trajectory_executor.py` 用 PD + gravity compensation 控制 torque UR5e，在 actuator 限制下积分 MuJoCo，记录目标/实际关节、torque 和末端位置。规划可行与实际可跟踪是两个问题。

`record_lab4_demo.py` 录制无文字面板的完整 MuJoCo 场景。视频和指标来自同一 capstone 执行，`slalom_summary.json` 绑定视频 SHA-256，并保存 `planning_audit.json` 的 SHA-256。Gallery 以 2 倍速展示完整执行，指标来自物理运行，而非 GIF 帧。

判定要求：关节 RMS <0.05 rad、最终构型误差 <0.05 rad、禁止接触为零、每个 1 ms tick 的 collision/可见 mesh 障碍间隙 ≥30 mm。执行前还必须通过独立规划检查。请结合[指标](../media/slalom_summary.json)、[规划验证](../media/planning_audit.json)和[gallery](../GALLERY.zh-CN.md)。

比较目标轨迹、实际跟踪、间隙曲线和接触数。看起来合理的视频或很小的终点误差不能证明避障。结果仅适用于录制场景、碰撞 pair 规则和采样分辨率。
