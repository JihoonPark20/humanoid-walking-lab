"""W3: Offline centroidal optimization 실행."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from centroidal_io import make_walking_problem, save_centroidal
from centroidal_optimizer import CentroidalFailure, CentroidalSolver


def main() -> None:
    """높이 변화 보행을 최적화하고 검증한 수치·그림을 저장한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=2)
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--height-amplitude", type=float, default=0.025)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "media" / "offline",
    )
    args = parser.parse_args()
    try:
        cfg, contacts, reference, _ = make_walking_problem(
            args.steps, args.dt, args.height_amplitude
        )
        solver = CentroidalSolver(len(contacts.times) - 1, cfg, terminal_stop=True)
        plan = solver.solve(reference[0], reference, contacts)
        report = save_centroidal(plan, args.output)
        print(json.dumps(report, indent=2, ensure_ascii=False))
    except (ValueError, CentroidalFailure) as error:
        parser.exit(1, f"최적화 실패: {error}\n")


if __name__ == "__main__":
    main()
