# 접촉을 고려한 보행

[English](README.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md)

Lab 6의 inverse-dynamics QP에서 이어집니다. 매끄러운 발 궤적과 측정 접촉을 연결하며, 발이 착지할 때 활성 task와 접촉 제약을 어떻게 바꿔야 하는지 학습합니다.

## C2 Swing 궤적

정규화한 Swing 시간 `s ∈ [0, 1]`에서 수평 보간은 `10s³−15s⁴+6s⁵`, 수직 clearance는 `64h·s³(1−s)³`입니다. 양 끝의 속도와 가속도가 0이 됩니다. `smooth_swing.py`에서 legacy sine 높이 궤적과 도함수를 비교하세요. 매끄러운 reference만으로 실제 착지 충격 감소가 보장되지는 않습니다.

## 접촉과 제어

`contact_walk.py`와 Lab 6의 `locomotion_controller.py`, `wb_id_qp.py`를 함께 읽으세요. 지면 normal force로 하중을 받는 접촉을 판정하고, Swing task를 켜기 전에 접촉 상태를 동기화합니다. Inverse-dynamics QP는 ProxQP, 발 자세·CoM 높이 task, 접촉 속도·자세 안정화를 사용합니다. LIPM 높이는 발목 frame이 아닌 지면을 기준으로 합니다.

예제는 평지에서 8 cm 네 걸음을 수행하며 single support 0.85초, double support 0.35초, reference clearance 2.5 cm를 사용합니다. 발 자세 안정화는 평평한 지면과 고정 yaw를 가정합니다.

## 실습

1. [실행 명령](../README.ko.md)으로 smooth와 `legacy`를 실행하되 시간과 제어기 설정은 동일하게 유지하세요.
2. 발의 기준 궤적과 실제 발 들기·착지를 비교하세요. 보라 Swing 곡선, 주황 ZMP, 초록 실제 지지 영역을 함께 보세요.
3. 매끄러운 reference를 추종해도 착지 충격이 생길 수 있는 이유를 설명하세요. 영상뿐 아니라 접촉력과 추종 오차도 비교하세요.

발 자세·접촉·관절 및 토크 한계·QP 잔차가 기록됩니다. 차이를 분석할 때 [원시 데이터](../GALLERY.ko.md)를 참고하세요. 첫 접촉 속도는 접촉 tick의 유한 차분이며 충돌 직전 속도의 직접 측정값은 아닙니다.
