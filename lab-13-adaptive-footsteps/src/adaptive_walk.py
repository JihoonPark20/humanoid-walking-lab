"""Compare fixed footholds with measured-DCM landing adjustments in actual G1 dynamics."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "lab-11-contact-transitions/src"))
from contact_walk import WalkCase, run
from step_adjustment import StepAdjustment

FORCES = (0.0, 10.0, -10.0, 20.0, -20.0)


def evaluate(output):
    """Paired conditions; every fixed/adaptive failure remains in the matrix."""
    output.mkdir(parents=True, exist_ok=True)
    reports = []
    with tempfile.TemporaryDirectory(prefix="adaptive-walking-") as work:
        for force in FORCES:
            case = WalkCase(name=f"lateral_{force:g}", push_force=force)
            for policy in ("fixed", "adaptive"):
                directory = Path(work) / f"{case.name}_{policy}"
                report = run(
                    case,
                    directory,
                    adapter=StepAdjustment() if policy == "adaptive" else None,
                )
                report["policy"] = policy
                destination = output / directory.name
                destination.mkdir(exist_ok=True)
                if (directory / "telemetry.csv").exists():
                    shutil.copy2(
                        directory / "telemetry.csv", destination / "telemetry.csv"
                    )
                reports.append(report)
    adaptive = [r for r in reports if r["policy"] == "adaptive"]
    selected = next(r for r in adaptive if r["case"]["push_force"] == 10)
    result = {
        "schema_version": 1,
        "passed": all(r["passed"] for r in adaptive) and bool(selected["adaptation"]),
        "forces_n": list(FORCES),
        "policies": ["fixed", "adaptive"],
        "total": len(reports),
        "successful_cases": sum(r["passed"] for r in reports),
        "reports": reports,
        "meaning": "All five adaptive executions must pass physical gates. Fixed-policy failures are retained; this does not assert superiority on every metric.",
    }
    (output / "matrix.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n"
    )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
    )
    parser.add_argument("--video", action="store_true")
    args = parser.parse_args()
    if args.output is None:
        args.output = (
            Path(__file__).resolve().parents[1]
            / "media"
            / ("demo" if args.video else "benchmark")
        )
    result = (
        run(
            WalkCase(name="lateral_10", push_force=10),
            args.output,
            video=True,
            adapter=StepAdjustment(),
        )
        if args.video
        else evaluate(args.output)
    )
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
