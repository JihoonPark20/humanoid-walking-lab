"""Paired RTI sensor experiments within the validated noise and packet-loss envelope."""

from __future__ import annotations

import argparse
import json
import platform
import sys
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for lab in ("lab-7-walking-optimization", "lab-8-centroidal-optimization"):
    sys.path.insert(0, str(ROOT / lab / "src"))
from centroidal_io import make_walking_problem, save_centroidal
from centroidal_optimizer import CentroidalSolver
from centroidal_sensing import SensorConfig
from mpc_experiment import run_centroidal, save_comparison

PROFILES = {
    "noise": SensorConfig(delay_steps=0, dropout_probability=0),
    "noise_delay": SensorConfig(dropout_probability=0),
    "packet_loss": SensorConfig(dropout_probability=0.15),
    "burst_loss": SensorConfig(dropout_probability=0, blackout=(21, 25)),
}


def save_observations(results: dict, output: Path) -> None:
    """Plot truth-scored estimator errors and packet age, including failed rollouts."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    for name, (report, _) in results.items():
        rows = report["sensing"]["observations"]
        data = np.array(
            [
                [
                    r["time_s"],
                    r["com_error_m"],
                    r["momentum_error"],
                    r["age_s"],
                    -1 if r["packet_index"] is None else r["packet_index"],
                ]
                for r in rows
            ]
        )
        np.savetxt(
            output / f"{name}_observations.csv",
            data,
            delimiter=",",
            comments="",
            header="t,com_error_m,momentum_error,age_s,received_index",
        )
        axes[0].plot(data[:, 0], 1000 * data[:, 1], label=name)
        axes[1].step(data[:, 0], data[:, 3], where="post", label=name)
    axes[0].set(
        ylabel="Estimated CoM error (mm)", title="Reduced centroidal sensor experiment"
    )
    axes[1].set(ylabel="Age of last observation (s)", xlabel="Time (s)")
    push_times = {
        result[0]["push_applied_time_s"]
        for result in results.values()
        if result[0]["push_applied_time_s"] is not None
    }
    for ax in axes:
        for push_time in sorted(push_times):
            ax.axvline(push_time, color="black", linestyle=":", alpha=0.5, label="Push")
        ax.grid(alpha=0.2)
        ax.legend()
    fig.tight_layout()
    fig.savefig(output / "observations.png", dpi=130)
    plt.close(fig)


def run_benchmark(output: Path, seeds: tuple[int, ...] = (0, 7, 23)) -> dict:
    """Preserve every paired result and evaluate unchanged physical success criteria."""
    cfg, contacts, reference, _ = make_walking_problem(2, 0.1, 0.015)
    plan = CentroidalSolver(len(contacts.times) - 1, cfg, True).solve(
        reference[0], reference, contacts
    )
    save_centroidal(plan, output / "nominal_plan")
    cases = []
    for name, sensor in PROFILES.items():
        for seed in seeds:
            folder = output / name / f"seed_{seed}"
            results = {}
            for method, gain in (("raw", 1.0), ("filtered", 0.65)):
                report, rows = run_centroidal(
                    plan,
                    "rti",
                    adapt_mass=False,
                    sensor_config=replace(sensor, seed=seed),
                    observer_gain=gain,
                )
                results[method] = report, rows
                cases.append(
                    {
                        "profile": name,
                        "seed": seed,
                        "method": method,
                        "report": report,
                    }
                )
                print(
                    f"{name}/{seed}/{method}: passed={report['passed']}, "
                    f"intervals={report['completed_intervals']}, "
                    f"CoM RMS={report['rms_estimated_com_error_m'] * 1000:.3f} mm",
                    flush=True,
                )
            save_comparison(results, folder, "centroidal")
            save_observations(results, folder)
    summary = {
        "schema_version": 1,
        "model": "reduced centroidal; no MuJoCo sensor simulation",
        "platform": platform.platform(),
        "python": platform.python_version(),
        "seeds": list(seeds),
        "push_m_s": 0.03,
        "push_time_s": 2.1,
        "criteria": "60/60 intervals; final CoM error < 0.03 m; true-state/wrench residual <= 1e-4",
        "all_passed": all(c["report"]["passed"] for c in cases),
        "passed": sum(c["report"]["passed"] for c in cases),
        "total": len(cases),
        "cases": cases,
    }
    (output / "matrix.json").write_text(json.dumps(summary, indent=2) + "\n")
    return summary


def main() -> None:
    """Run the published seed set; return failure if any recommended profile fails."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "media" / "observation_benchmark",
    )
    args = parser.parse_args()
    summary = run_benchmark(args.output)
    print(f"Passed: {summary['passed']}/{summary['total']}")
    if not summary["all_passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
