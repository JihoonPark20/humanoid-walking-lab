"""G1 centroidal momentum을 map 및 link별 공간운동량 합과 교차 검증한다."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "lab-6-whole-body-control" / "src"))
from lab6_common import load_g1_pinocchio, load_g1_torque_mujoco, mj_state_to_pin


def validate_pinocchio(samples: int = 5) -> dict:
    """각 link의 운동량을 world origin으로 옮겨 합한 뒤 CoM으로 다시 이동한다."""
    if samples < 1:
        raise ValueError("samples는 양수여야 합니다.")
    model, data = load_g1_pinocchio()
    _, mj_data = load_g1_torque_mujoco()
    q0, _ = mj_state_to_pin(mj_data)
    rng = np.random.default_rng(41)
    map_error = sum_error = linear_error = 0.0
    mass = sum(i.mass for i in model.inertias)
    for _ in range(samples):
        q = pin.integrate(model, q0, rng.normal(0, 0.015, model.nv))
        v = rng.normal(0, 0.1, model.nv)
        h = pin.computeCentroidalMomentum(model, data, q, v).vector.copy()
        ag = pin.computeCentroidalMap(model, data, q).copy()
        pin.forwardKinematics(model, data, q, v)
        total = np.zeros(6)
        for joint in range(1, model.njoints):
            local = model.inertias[joint] * data.v[joint]
            total += data.oMi[joint].act(local).vector
        pin.centerOfMass(model, data, q, v)
        total[3:] -= np.cross(data.com[0], total[:3])
        map_error = max(map_error, float(np.max(abs(ag @ v - h))))
        sum_error = max(sum_error, float(np.max(abs(total - h))))
        linear_error = max(
            linear_error, float(np.max(abs(h[:3] - mass * data.vcom[0])))
        )
    return {
        "samples": samples,
        "mass_kg": mass,
        "map_error": map_error,
        "link_sum_error": sum_error,
        "linear_momentum_error": linear_error,
        "passed": max(map_error, sum_error, linear_error) < 1e-9,
    }


def main() -> None:
    """검증 지표를 저장하고 불일치가 있으면 실패한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "media" / "pinocchio.json",
    )
    args = parser.parse_args()
    report = validate_pinocchio()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
