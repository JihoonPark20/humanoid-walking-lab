"""고정된 외란에서 nominal·지연·질량·마찰 조건을 full NLP와 RTI로 검증한다."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for lab in ("lab-7-walking-optimization", "lab-8-centroidal-optimization"):
    sys.path.insert(0, str(ROOT / lab / "src"))
from centroidal_io import make_walking_problem, save_centroidal
from centroidal_optimizer import CentroidalSolver
from mpc_experiment import run_centroidal, save_comparison

SCENARIOS = {
    "nominal": (1.0, 0.6, 0),
    "mass_plus_5pct": (1.05, 0.6, 0),
    "delay_one": (1.0, 0.6, 1),
    "friction_04": (1.0, 0.4, 0),
    "combined": (1.05, 0.4, 1),
    "combined_delay_two": (1.05, 0.4, 2),
    "mass_minus_5pct": (0.95, 0.4, 1),
    "mass_plus_10pct": (1.10, 0.4, 1),
}


def run_scenarios(output: Path) -> dict:
    """실제 plant 설정과 controller 설정을 분리한 16개 조건의 결과를 보존한다."""
    cfg, contacts, reference, _ = make_walking_problem(2, 0.1, 0.015)
    nominal = CentroidalSolver(len(contacts.times) - 1, cfg, True).solve(
        reference[0], reference, contacts
    )
    save_centroidal(nominal, output / "nominal_plan")
    reports = {}
    for name, (mass_ratio, friction, delay) in SCENARIOS.items():
        results = {}
        for mode in ("mpc", "rti"):
            results[mode] = run_centroidal(
                nominal,
                mode,
                mass_ratio=mass_ratio,
                friction=friction,
                delay_steps=delay,
            )
            result = results[mode][0]
            print(
                f"{name}/{mode}: passed={result['passed']}, error={result['final_com_error_m']}",
                flush=True,
            )
        save_comparison(results, output / name, "centroidal")
        reports[name] = {mode: value[0] for mode, value in results.items()}
    summary = {
        "all_passed": all(
            r["passed"] for case in reports.values() for r in case.values()
        ),
        "cases": 2 * len(SCENARIOS),
        "push_m_s": 0.03,
        "push_time_s": 2.1,
        "criteria": "60/60 intervals, final CoM error < 0.03 m, physical state/wrench residual <= 1e-4",
        "reports": reports,
    }
    (output / "matrix.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
    )
    return summary


def main() -> None:
    """실패한 조건이 하나라도 있으면 exit 1을 반환한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "media" / "validated_scenarios",
    )
    args = parser.parse_args()
    report = run_scenarios(args.output)
    print(f"{report['cases']} conditions: all_passed={report['all_passed']}")
    if not report["all_passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
