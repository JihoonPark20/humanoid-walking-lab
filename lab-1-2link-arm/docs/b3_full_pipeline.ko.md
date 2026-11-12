# B3: Full Pipeline — Study Notes

[English](b3_full_pipeline.md) · [한국어](b3_full_pipeline.ko.md) · [简体中文](b3_full_pipeline.zh-CN.md)

`target → IK → trajectory → PD → plant → metrics/log`를 연결합니다. `b3_full_pipeline.py`는 waypoint 왕복, 원 추종, workspace 경계의 singularity 비교를 실행합니다.

Waypoint 예제 `(0.20,0.30) → (0.40,0.10) → (0.20,0.30)`는 analytic IK와 quintic 구간을 사용합니다. 여기의 pick-and-place는 목표점 왕복이며 실제 물체 집기 검증이 아닙니다. 원은 매 샘플 IK를 풀며, 경계에서는 pinv와 DLS의 실패·오차를 비교합니다. 기본 B2 plant 결과와 실제 MuJoCo C1 결과를 구분하세요.

Lab 폴더에서 `python src/b3_full_pipeline.py`를 실행합니다. `b3_pick_place_log.csv`, `b3_circle_log.csv`, `b3_singularity_log.csv`, `b3_metrics.csv`가 주요 출력이며 plot/GIF는 선택 사항입니다. 추종 오차뿐 아니라 IK 실패와 관절 연속성을 확인하세요.

[학습 노트](README.ko.md) · [실행 안내](../README.ko.md)
