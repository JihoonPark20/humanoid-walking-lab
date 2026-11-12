"""W1/W2 실습 CLI: 고정 발걸음, footstep QP, 접촉 시간 후보 비교."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import replace
from pathlib import Path

from lipm_optimizer import PlanningError, WalkingPlan, plan_cost, solve_walk
from walking_artifacts import animate_plan, export_plan, load_plan
from walking_model import WalkConfig, build_schedule

DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / "media" / "offline"


def compare_timings(
    config: WalkConfig, candidates: list[float], time_weight: float = 0.05
) -> tuple[WalkingPlan, list[dict]]:
    """같은 목표 거리에서 SS 후보를 비교한다. score=목적함수+time_weight*T."""
    if time_weight < 0 or not math.isfinite(time_weight):
        raise ValueError("time_weight는 유한한 0 이상의 값이어야 합니다.")
    trials, best, best_score = [], None, float("inf")
    for single in candidates:
        try:
            cfg = replace(config, single_support=single)
            plan = solve_walk(build_schedule(cfg), optimize_feet=True)
            objective = plan_cost(plan)["objective"]
            score = objective + time_weight * plan.schedule.duration
            trials.append(
                {
                    "single_support_s": single,
                    "feasible": True,
                    "objective": objective,
                    "duration_s": plan.schedule.duration,
                    "score": score,
                }
            )
            if score < best_score:
                best, best_score = plan, score
        except (PlanningError, ValueError) as error:
            trials.append(
                {"single_support_s": single, "feasible": False, "error": str(error)}
            )
    if best is None:
        raise PlanningError(
            "모든 시간 후보가 실패했습니다: " + json.dumps(trials, ensure_ascii=False)
        )
    return best, trials


def main() -> None:
    """계획을 실행하거나 저장된 계획을 검증·재생한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=4)
    parser.add_argument("--step-length", type=float, default=0.10)
    parser.add_argument("--single-support", type=float, default=0.65)
    parser.add_argument("--double-support", type=float, default=0.25)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--optimize-feet", action="store_true")
    parser.add_argument(
        "--compare", action="store_true", help="고정 발/최적화 발/시간 후보를 모두 비교"
    )
    parser.add_argument("--timings", type=float, nargs="+", default=[0.55, 0.65, 0.75])
    parser.add_argument("--time-weight", type=float, default=0.05)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--replay", type=Path, help="저장된 NPZ 검증 후 그림·CSV 재생성"
    )
    parser.add_argument("--gif", action="store_true")
    args = parser.parse_args()
    try:
        if args.replay:
            plans = {"replay": load_plan(args.replay)}
        else:
            cfg = WalkConfig(
                steps=args.steps,
                step_length=args.step_length,
                single_support=args.single_support,
                double_support=args.double_support,
                dt=args.dt,
            )
            if args.compare:
                plans = {
                    "fixed": solve_walk(build_schedule(cfg)),
                    "footsteps": solve_walk(build_schedule(cfg), optimize_feet=True),
                }
                best, trials = compare_timings(cfg, args.timings, args.time_weight)
                plans["timing"] = best
                args.output.mkdir(parents=True, exist_ok=True)
                (args.output / "timing_candidates.json").write_text(
                    json.dumps(
                        {"time_weight": args.time_weight, "trials": trials},
                        indent=2,
                        ensure_ascii=False,
                    )
                    + "\n"
                )
            else:
                plans = {
                    "footsteps" if args.optimize_feet else "fixed": solve_walk(
                        build_schedule(cfg), optimize_feet=args.optimize_feet
                    )
                }
        comparison = {}
        for name, plan in plans.items():
            folder = args.output / name
            report = export_plan(plan, folder)
            comparison[name] = {
                "cost": report["cost"],
                "duration_s": report["duration_s"],
                "validation": report["validation"],
            }
            if args.gif:
                animate_plan(plan, folder / "preview.gif")
            print(
                f"{name}: 검증 통과 · 비용 {report['cost']['objective']:.6f} · {folder}"
            )
        (args.output / "comparison.json").write_text(
            json.dumps(comparison, indent=2, ensure_ascii=False) + "\n"
        )
    except (PlanningError, ValueError) as error:
        parser.exit(1, f"계획 실패: {error}\n")


if __name__ == "__main__":
    main()
