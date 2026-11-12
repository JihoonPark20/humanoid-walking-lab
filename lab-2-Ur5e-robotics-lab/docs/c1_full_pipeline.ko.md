# C1: Integrated Pick-and-Place Pipeline

[English](c1_full_pipeline.md) · [한국어](c1_full_pipeline.ko.md) · [简体中文](c1_full_pipeline.zh-CN.md)

IK·궤적·제어·제약·지표를 하나의 실행 흐름으로 연결합니다. `c1_full_pipeline.py`의 waypoint 왕복·원·다중 waypoint·singularity 접근 예제에서 계획한 reference와 실제 plant 추종을 비교합니다. Pick-and-place라는 이름만으로 물체 grasp 성공을 뜻하지 않습니다.

각 IK 결과는 FK로 확인하고, 보간 구간의 경계조건과 속도·torque 제한을 검사합니다. Local IK 실패와 controller tracking 실패를 구분해 기록하세요. Joint frame 거리 기반 collision 휴리스틱은 실제 mesh 검사와 다릅니다.

Lab 폴더에서 `python src/c1_full_pipeline.py`를 실행합니다. `c1_metrics_dashboard.csv`와 각 `c1_*_log.csv`에서 task 오차·joint 상태·제약 활성화를 비교하세요. 실제 MuJoCo 대표 시연은 C3이며, 동일한 성공 판정을 모든 모듈에 자동으로 적용하지 않습니다.

[학습 노트](README.ko.md) · [실행 안내](../README.ko.md)
