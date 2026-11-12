"""한두 걸음 × 10/20 ms × 세 tracking 정책의 권장 설정을 검증한다."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
from whole_body_demo import save_solution, simulate_mujoco
from whole_body_ocp import OCPConfig, solve_problem
from whole_body_tracking import TrackingConfig, recommended_control_weight


def run_scenarios(output: Path, video: bool = False) -> dict:
    """각 정책의 명시적인 torque 비용으로 계획하고 동일 물리 기준으로 판정한다."""
    output.mkdir(parents=True, exist_ok=True)
    reports = {}
    for steps in (1, 2):
        for dt in (0.01, 0.02):
            for weight, modes in ((3.0, ("fddp",)), (0.1, ("blended", "joint-pd"))):
                cfg = OCPConfig(
                    steps=steps,
                    dt=dt,
                    swing_knots=round(0.6 / dt),
                    support_knots=round(0.6 / dt),
                    control_weight=weight,
                )
                label = f"steps{steps}_dt{round(dt * 1000)}_weight{weight:g}"
                print(f"Planning {label}", flush=True)
                solver, gait, model, initial, times, phases, plan = solve_problem(cfg)
                save_solution(solver, times, phases, plan, output / label, cfg)
                for mode in modes:
                    assert recommended_control_weight(mode) == weight
                    if plan["converged"] and plan["model_rollout_valid"]:
                        data = mujoco.MjData(model)
                        data.qpos[:], data.qvel[:] = initial.qpos, initial.qvel
                        mujoco.mj_forward(model, data)
                        actual = simulate_mujoco(
                            solver,
                            gait,
                            model,
                            data,
                            times,
                            output / label / mode,
                            video=video and mode == "fddp" and dt == 0.01,
                            tracking=TrackingConfig(mode=mode),
                        )
                    else:
                        actual = {
                            "passed": False,
                            "failure": "OCP validation failed; physical replay skipped",
                        }
                    reports[f"{label}/{mode}"] = {
                        "config": {
                            "steps": steps,
                            "dt": dt,
                            "control_weight": weight,
                            "tracking": mode,
                        },
                        "plan": plan,
                        "actual": actual,
                    }
                    print(f"{label}/{mode}: passed={actual['passed']}", flush=True)
    result = {
        "all_passed": all(
            r["actual"]["passed"] and r["actual"].get("clipping_ticks", 1) == 0
            for r in reports.values()
        ),
        "cases": len(reports),
        "criteria": "converged, model feasible, complete, no fall/flight/clipping, final foot gap < 0.03 m",
        "reports": reports,
    }
    (output / "matrix.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n"
    )
    return result


def main() -> None:
    """12개 권장 프로파일 중 실패가 있으면 exit 1로 보고한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", action="store_true")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "media" / "validated_scenarios",
    )
    args = parser.parse_args()
    result = run_scenarios(args.output, args.video)
    print(f"{result['cases']} conditions: all_passed={result['all_passed']}")
    if not result["all_passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
