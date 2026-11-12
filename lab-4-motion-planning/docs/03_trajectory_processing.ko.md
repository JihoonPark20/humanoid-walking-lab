# Trajectory processing

[English](03_trajectory_processing.md) · [한국어](03_trajectory_processing.ko.md) · [简体中文](03_trajectory_processing.zh-CN.md)

`shortcut_path()`는 서로 떨어진 waypoint를 선택하고 같은 checker로 연결 edge를 검사한 뒤에만 중간 경로를 대체합니다. Endpoint를 유지하며, node가 안전하다고 edge도 안전하다고 가정하지 않습니다.

`parameterize_topp_ra(path,vel_limits,acc_limits,dt)`는 `(times,q,qd,qdd)`를 반환합니다. 빈 경로·비유한 값·양수가 아닌 한계·잘못된 dt를 거부하고 연속 중복점을 제거합니다. TOPP-RA가 있으면 보간 spline을 시간 매개화하며, 없으면 보수적인 quintic 구간을 사용합니다. Duration이 dt의 배수가 아니어도 정확한 마지막 시각을 포함합니다.

Quintic은 관절 공간의 직선 edge 위에 머물며 구간 끝 속도·가속도는 0입니다. Spline은 경로 형상을 바꾸고 모서리를 가로질러 장애물에 들어갈 수 있습니다. 따라서 capstone은 실행 전에 모든 시간 샘플의 자세·속도·가속도를 다시 검사하고 위반 시 중단합니다. 기하학적 경로의 성공만으로 spline을 인증할 수 없습니다.

`planning_audit.json`의 raw·shortcut·timed 검사를 구분하세요. Blocking post를 가로지르는 shortcut이 거부되는지 실험하세요. 게시된 quintic 데모에 TOPP-RA는 필요하지 않습니다. 실제 설치된 선택 backend는 별도 실행 검증이 필요하며, 회귀 테스트에서는 제어된 backend로 마지막 시각 처리도 검사합니다.
