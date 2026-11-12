"""Fixed physical G1 environments; retain every unsuccessful case in one matrix."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
import tempfile
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "lab-11-contact-transitions/src"))
from contact_walk import WalkCase, run

CASES = (
    WalkCase(),
    WalkCase(name="push_left", push_force=10),
    WalkCase(name="push_right", push_force=-10),
    WalkCase(name="push_forward", push_force=10, push_axis=0),
    WalkCase(name="push_backward", push_force=-10, push_axis=0),
    WalkCase(name="friction_04", friction=0.4),
    WalkCase(name="mass_plus_5pct", mass_ratio=1.05),
    WalkCase(name="mass_minus_5pct", mass_ratio=0.95),
    WalkCase(name="later_push", push_force=10, push_start=2.50),
    WalkCase(name="combined", mass_ratio=1.05, friction=0.4, push_force=10),
)


def evaluate(output):
    """Same controller and gates for every row; no case-dependent gain changes."""
    output.mkdir(parents=True, exist_ok=True)
    reports = []
    with tempfile.TemporaryDirectory(prefix="walking-evaluation-") as work:
        for case in CASES:
            directory = Path(work) / case.name
            report = run(case, directory)
            destination = output / case.name
            destination.mkdir(exist_ok=True)
            if (directory / "telemetry.csv").exists():
                shutil.copy2(directory / "telemetry.csv", destination / "telemetry.csv")
            reports.append(report)
    passed = sum(r["passed"] for r in reports)
    result = {
        "schema_version": 1,
        "passed": passed >= 8 and reports[0]["passed"] and reports[1]["passed"],
        "meaning": "Coverage and at least 8/10 safe completed cases, including preselected nominal and left push; not universal robustness.",
        "cases": [asdict(c) for c in CASES],
        "total": len(reports),
        "successful_cases": passed,
        "reports": reports,
        "controller_source_sha256": hashlib.sha256(
            (ROOT / "lab-11-contact-transitions/src/contact_walk.py").read_bytes()
        ).hexdigest(),
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
    parser.add_argument(
        "--video",
        action="store_true",
        help="Record the preselected +10 N lateral case instead of the matrix",
    )
    args = parser.parse_args()
    if args.output is None:
        args.output = (
            Path(__file__).resolve().parents[1]
            / "media"
            / ("demo" if args.video else "benchmark")
        )
    result = (
        run(CASES[1], args.output, video=True) if args.video else evaluate(args.output)
    )
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
