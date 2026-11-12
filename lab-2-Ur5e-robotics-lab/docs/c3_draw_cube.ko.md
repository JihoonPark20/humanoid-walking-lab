# C3: 큐브의 12개 모서리 추종

[English](c3_draw_cube.md) · [한국어](c3_draw_cube.ko.md) · [简体中文](c3_draw_cube.zh-CN.md)

UR5e가 큐브의 12개 모서리를 연속적으로 방문합니다. 큐브에는 홀수 차수 꼭짓점이 8개 있으므로 각 모서리를 정확히 한 번만 지나는 연속 경로는 불가능합니다. 세 모서리를 다시 지나며 총 16개 꼭짓점·15구간·30 s reference입니다. 연속한 꼭짓점은 실제 모서리로만 연결하고 회귀 테스트로 대각선 연결을 거부합니다.

## 파이프라인

`cube_vertices()`는 중심 `[0.3,0.2,0.55]`, 변 0.10 m 큐브를 만듭니다. DLS IK는 home orientation을 고정하고 이전 해를 다음 초기값으로 씁니다. `build_cube_trajectory()`는 각 joint pose를 2 s quintic으로 연결하며 중복 시각을 제거합니다. Joint-space 보간이므로 실제 끝점 reference는 직선 모서리에서 조금 휠 수 있습니다.

Menagerie position servo는 대략 `tau=Kp(ctrl-q)-Kd*qd`입니다. 위치 명령에 bias·속도 feedforward를 더합니다.

```text
ctrl = q_des + bias(q,qd)/Kp + Kd*qd_des/Kp
```

Recorder는 중력뿐 아니라 Coriolis도 포함한 MuJoCo `qfrc_bias`를 쓰며 추종 전 3 s 동안 정착합니다. 장면의 contact를 비활성화하지 않습니다.

## 검증

같은 실행으로 영상과 [측정값](../media/c3_draw_cube.json)을 생성합니다. 기준은 12개 모서리 방문·대각선 없음, joint reference의 FK 대비 RMS <1 mm·최대 <5 mm, 실제 기하학적 모서리까지 거리 <5 mm, MuJoCo 접촉 0, 전체 완료입니다. Reference FK 추종 오차와 실제 직선 모서리 편차는 다른 지표입니다.

```bash
# Lab 폴더에서 macOS interactive viewer:
mjpython src/c3_draw_cube.py
# Offscreen 녹화:
python src/c3_record_video.py
```

장면·속도를 고정하고 단순 position control과 bias/velocity feedforward를 비교하세요. Actuator 한계·실제 모서리 편차·접촉을 함께 봅니다. [Gallery](../GALLERY.ko.md)는 전체 실행 3배속입니다.
