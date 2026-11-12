# Trajectory execution

[English](04_trajectory_execution.md) · [한국어](04_trajectory_execution.ko.md) · [简体中文](04_trajectory_execution.zh-CN.md)

`trajectory_executor.py`는 torque UR5e에 PD + gravity compensation을 적용합니다. Actuator 한계 아래에서 MuJoCo를 적분하고 목표·실제 관절, torque, 끝점 위치를 기록합니다. 계획 가능성과 실제 추종 가능성은 별도입니다.

`record_lab4_demo.py`는 텍스트 패널 없이 MuJoCo 전체 장면을 기록합니다. 같은 capstone 실행의 영상·측정값을 사용하고 `slalom_summary.json`을 영상 SHA-256에 연결합니다. Summary는 `planning_audit.json`의 SHA-256도 저장합니다. Gallery는 전체 실행 2배속이며 지표는 GIF frame이 아닌 실제 물리 실행에서 계산합니다.

판정은 관절 RMS <0.05 rad, 최종 자세 오차 <0.05 rad, 금지된 접촉 0회, 매 1 ms tick에서 collision·표시 mesh의 장애물 간격 ≥30 mm입니다. 실행 전에 독립 계획 검사도 통과해야 합니다. [측정값](../media/slalom_summary.json), [계획 검사](../media/planning_audit.json), [gallery](../GALLERY.ko.md)를 함께 읽으세요.

목표 궤적·실제 추종·clearance 곡선·접촉 수를 비교하세요. 보기 좋은 영상이나 작은 endpoint 오차만으로 충돌 회피를 입증할 수 없습니다. 결과는 녹화한 장면·collision pair 규칙·샘플 해상도 범위에서 유효합니다.
