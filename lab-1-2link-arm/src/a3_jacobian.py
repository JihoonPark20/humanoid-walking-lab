"""Jacobian: 관절 속도에서 끝점 속도로의 변환.

    x_dot = J(theta) * theta_dot

해석 Jacobian, finite difference, determinant/singularity 분석을 포함한다.
MuJoCo가 설치되어 있으면 mj_jacSite 결과와 교차 검증한다."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Iterable

MODEL_PATH = "models/two_link.xml"
L1 = 0.3
L2 = 0.3
EE_OFFSET = 0.015
OUTPUT_CSV = Path("docs/a3_det_sweep.csv")


Vector2 = tuple[float, float]
Matrix2 = tuple[tuple[float, float], tuple[float, float]]


def fk_endeffector(theta1: float, theta2: float, include_site_offset: bool = True) -> Vector2:
    """끝점의 (x, y) 위치를 계산한다."""
    total = theta1 + theta2
    l2_eff = L2 + (EE_OFFSET if include_site_offset else 0.0)
    x = L1 * math.cos(theta1) + l2_eff * math.cos(total)
    y = L1 * math.sin(theta1) + l2_eff * math.sin(total)
    return (x, y)


def analytic_jacobian(theta1: float, theta2: float, include_site_offset: bool = True) -> Matrix2:
    """2-link planar arm의 해석 Jacobian을 계산한다."""
    total = theta1 + theta2
    l2_eff = L2 + (EE_OFFSET if include_site_offset else 0.0)

    return (
        (
            -L1 * math.sin(theta1) - l2_eff * math.sin(total),
            -l2_eff * math.sin(total),
        ),
        (
            L1 * math.cos(theta1) + l2_eff * math.cos(total),
            l2_eff * math.cos(total),
        ),
    )


def numeric_jacobian(theta1: float, theta2: float, eps: float = 1e-6) -> Matrix2:
    """중앙 finite difference로 FK를 미분해 Jacobian을 계산한다."""
    # 양방향의 작은 관절 변화를 사용한 중앙 차분으로 해석식을 검증한다.
    fx_p = fk_endeffector(theta1 + eps, theta2)
    fx_m = fk_endeffector(theta1 - eps, theta2)
    fy_p = fk_endeffector(theta1, theta2 + eps)
    fy_m = fk_endeffector(theta1, theta2 - eps)

    dtheta1 = (
        (fx_p[0] - fx_m[0]) / (2.0 * eps),
        (fx_p[1] - fx_m[1]) / (2.0 * eps),
    )
    dtheta2 = (
        (fy_p[0] - fy_m[0]) / (2.0 * eps),
        (fy_p[1] - fy_m[1]) / (2.0 * eps),
    )
    return (
        (dtheta1[0], dtheta2[0]),
        (dtheta1[1], dtheta2[1]),
    )


def determinant(jacobian: Matrix2) -> float:
    """2×2 행렬의 determinant를 계산한다."""
    return jacobian[0][0] * jacobian[1][1] - jacobian[0][1] * jacobian[1][0]


def max_abs_diff(lhs: Matrix2, rhs: Matrix2) -> float:
    """두 2×2 행렬 원소의 절댓값 차이 중 최댓값을 반환한다."""
    values = [
        abs(lhs[0][0] - rhs[0][0]),
        abs(lhs[0][1] - rhs[0][1]),
        abs(lhs[1][0] - rhs[1][0]),
        abs(lhs[1][1] - rhs[1][1]),
    ]
    return max(values)


def endeffector_velocity(theta1: float, theta2: float, theta1_dot: float, theta2_dot: float) -> Vector2:
    """Jacobian과 관절 속도의 곱으로 task-space 속도를 계산한다."""
    jac = analytic_jacobian(theta1, theta2)
    vx = jac[0][0] * theta1_dot + jac[0][1] * theta2_dot
    vy = jac[1][0] * theta1_dot + jac[1][1] * theta2_dot
    return (vx, vy)


def print_validation_table(samples: Iterable[tuple[float, float]]) -> float:
    """해석 Jacobian과 수치 Jacobian의 오차를 비교한다."""
    print("=" * 84)
    print("A3: Jacobian validation")
    print("=" * 84)
    print(
        f"{'theta1':>8} {'theta2':>8} | "
        f"{'det(J)':>12} {'max|J_a-J_n|':>14} | "
        f"{'singular?':>10}"
    )
    print("-" * 84)

    worst_error = 0.0
    for theta1_deg, theta2_deg in samples:
        theta1 = math.radians(theta1_deg)
        theta2 = math.radians(theta2_deg)
        jac_a = analytic_jacobian(theta1, theta2)
        jac_n = numeric_jacobian(theta1, theta2)
        det_j = determinant(jac_a)
        error = max_abs_diff(jac_a, jac_n)
        worst_error = max(worst_error, error)
        is_singular = abs(det_j) < 1e-6
        print(
            f"{theta1_deg:8.1f} {theta2_deg:8.1f} | "
            f"{det_j:12.8f} {error:14.10f} | "
            f"{str(is_singular):>10}"
        )

    print("-" * 84)
    print(f"Worst analytic/numeric diff: {worst_error:.10f}")
    return worst_error


def save_det_sweep(theta1_deg: float = 20.0, steps: int = 81) -> None:
    """theta2를 바꾸며 determinant를 계산하여 CSV로 저장한다."""
    theta1 = math.radians(theta1_deg)
    OUTPUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    with OUTPUT_CSV.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["theta1_deg", "theta2_deg", "det_j"])
        for idx in range(steps):
            theta2_deg = -180.0 + idx * (360.0 / (steps - 1))
            theta2 = math.radians(theta2_deg)
            det_j = determinant(analytic_jacobian(theta1, theta2))
            writer.writerow([f"{theta1_deg:.1f}", f"{theta2_deg:.1f}", f"{det_j:.10f}"])

    print(f"Determinant sweep saved: {OUTPUT_CSV}")


def mujoco_jacobian(theta1: float, theta2: float) -> Matrix2 | None:
    """MuJoCo가 있으면 끝점의 선속도 Jacobian을 반환한다."""
    try:
        import mujoco
        import numpy as np
    except ModuleNotFoundError:
        return None

    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    data.qpos[0] = theta1
    data.qpos[1] = theta2
    mujoco.mj_forward(model, data)

    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp, jacr, site_id)

    return (
        (float(jacp[0, 0]), float(jacp[0, 1])),
        (float(jacp[1, 0]), float(jacp[1, 1])),
    )


def compare_with_mujoco(theta1_deg: float = 30.0, theta2_deg: float = 45.0) -> None:
    """현재 해석 Jacobian을 MuJoCo의 결과와 비교한다."""
    theta1 = math.radians(theta1_deg)
    theta2 = math.radians(theta2_deg)
    jac_mj = mujoco_jacobian(theta1, theta2)
    if jac_mj is None:
        print("mj_jacSite comparison skipped: MuJoCo not installed.")
        return

    jac_a = analytic_jacobian(theta1, theta2)
    error = max_abs_diff(jac_a, jac_mj)
    print("\nMuJoCo Jacobian comparison")
    print(f"Angles: theta1={theta1_deg:.1f} deg, theta2={theta2_deg:.1f} deg")
    print(f"Analytic J: {jac_a}")
    print(f"MuJoCo   J: {jac_mj}")
    print(f"Max diff  : {error:.10f}")


def explain_jacobian() -> None:
    """Jacobian의 직관적인 의미를 설명한다."""
    print("\nWhat is the Jacobian?")
    print("1. The Jacobian is the local linear map from joint-space velocities to task-space velocities.")
    print("2. It is obtained by differentiating FK with respect to the joint angles.")
    print("3. When the determinant approaches zero the robot is near a singularity and motion in some directions becomes difficult.")


def main() -> None:
    samples = [
        (0.0, 0.0),
        (30.0, 45.0),
        (45.0, -45.0),
        (60.0, 60.0),
        (90.0, -90.0),
        (-30.0, 10.0),
        (15.0, 179.0),
    ]

    worst_error = print_validation_table(samples)
    near_zero = determinant(analytic_jacobian(math.radians(20.0), math.radians(0.1)))
    near_pi = determinant(analytic_jacobian(math.radians(20.0), math.radians(179.9)))
    print(f"\ndet(J) near theta2=0  ~= {near_zero:.10f}")
    print(f"det(J) near theta2=pi ~= {near_pi:.10f}")

    velocity = endeffector_velocity(math.radians(30.0), math.radians(45.0), 0.5, -0.2)
    print(f"\nExample velocity mapping: theta_dot=[0.5, -0.2] rad/s -> x_dot=[{velocity[0]:.6f}, {velocity[1]:.6f}] m/s")

    save_det_sweep()
    compare_with_mujoco()
    explain_jacobian()

    print("\nValidation summary")
    print(f"- Analytic vs numeric worst diff: {worst_error:.10f}")
    print("- Target threshold: < 0.0001")


if __name__ == "__main__":
    main()
