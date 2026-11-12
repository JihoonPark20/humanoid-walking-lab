# Interview Cheatsheet

[English](interview_cheatsheet.md) · [한국어](interview_cheatsheet.ko.md) · [简体中文](interview_cheatsheet.zh-CN.md)

- **FK와 IK:** FK는 q에서 pose, IK는 pose에서 가능한 q를 구합니다. IK는 여러 해·해 없음·초기값 의존성이 있습니다.
- **Jacobian:** `v_task=J qd`, 가상일 관계 `tau=J.T wrench`입니다. Frame·단위·linear/angular 순서를 명시하세요.
- **Singularity:** J의 rank 감소로 운동 방향을 잃습니다. SVD로 진단하고 DLS·속도 제한을 사용하지만 도달 불가능성을 없애지는 않습니다.
- **Computed torque:** M과 bias로 nominal 비선형성을 보상하고 PD 오차 Dynamics를 만듭니다. 모델 오차·saturation에서 이상적 결과가 깨집니다.
- **OSC:** Task-space inertia와 bias를 고려한 task acceleration/force 제어입니다. 단순 `J.T F` impedance와 구분하세요.
- **휴머노이드 연결:** Floating base는 직접 구동되지 않으며 contact wrench가 균형을 결정합니다. Joint/task IK만으로는 friction·ZMP·torque feasibility를 보장하지 않습니다. Lab 5–10에서 support·WBC·trajectory optimization·MPC·OCP를 연결합니다.

수식을 말한 뒤 실제 코드의 frame, 검증 기준, 실패 조건을 예로 설명해 보세요.

[학습 노트](README.ko.md) · [실행 안내](../README.ko.md)
