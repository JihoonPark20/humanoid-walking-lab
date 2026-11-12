# B2: PD Controller — Study Notes

[English](b2_pd_controller.md) · [한국어](b2_pd_controller.ko.md) · [简体中文](b2_pd_controller.zh-CN.md)

PD + gravity compensation은 `tau=Kp(q_des-q)+Kd(qd_des-qd)+g(q)`입니다. Kp는 위치 복원, Kd는 속도 오차 damping을 담당합니다.

`b2_pd_controller.py`의 기본 plant는 의존성 없는 결정적 Python 모델입니다. 고정 목표 `[90°,-45°]`와 B1 cubic 추종에서 중력 보상 유무를 비교합니다. 이 결과를 실제 MuJoCo Dynamics라고 해석하지 마세요. MuJoCo hook은 별도로 검증해야 합니다.

Lab 폴더에서 `python src/b2_pd_controller.py`를 실행하세요. `b2_fixed_target_{gc,no_gc}.csv`, `b2_tracking_{gc,no_gc}.csv`에 상태를 기록합니다. 정착 시간·overshoot·정상상태 오차·RMS를 비교하고 Kp/Kd를 바꿔 보세요. 선택적 plot이 없어도 CSV를 읽을 수 있습니다.

[학습 노트](README.ko.md) · [실행 안내](../README.ko.md)
