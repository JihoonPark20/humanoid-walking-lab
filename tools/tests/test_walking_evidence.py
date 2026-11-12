"""Reject plausible-looking videos, altered physical logs and incomplete protocols."""

import csv
import json
import shutil
from pathlib import Path

import pytest

from tools import build_release, walking_evidence

ROOT = Path(__file__).resolve().parents[2]


def test_advanced_recordings_and_protocols_are_valid():
    assert build_release.advanced_walking_errors() == []


@pytest.fixture
def recording(tmp_path):
    source = ROOT / "lab-12-walking-robustness/media/demo"
    for name in ("summary.json", "telemetry.csv", "walk.mp4"):
        shutil.copy2(source / name, tmp_path / name)
    return tmp_path


@pytest.mark.parametrize(
    "mutation",
    [
        "missing_tick",
        "joint_limit",
        "push_removed",
        "landing",
        "no_liftoff",
        "qp_residual",
        "source",
        "video",
    ],
)
def test_forged_physical_success_rejected(recording, mutation):
    summary = json.loads((recording / "summary.json").read_text())
    rows = list(csv.DictReader((recording / "telemetry.csv").open()))
    if mutation == "missing_tick":
        del rows[100]
    elif mutation == "joint_limit":
        rows[100]["joint_violation_rad"] = "0.01"
    elif mutation == "push_removed":
        for row in rows:
            row["push_y_n"] = "0"
    elif mutation == "landing":
        landing = summary["landings"][0]
        row = next(r for r in rows if float(r["t"]) >= landing["end_s"] + 0.1)
        row[f"{landing['side']}_x"] = "10"
    elif mutation == "no_liftoff":
        for row in rows:
            row["left_contact"] = "1"
    elif mutation == "qp_residual":
        rows[100]["residual_base_dynamics"] = "0.03"
    elif mutation == "source":
        summary["source_sha256"] = {}
    else:
        summary["video_sha256"] = "wrong"
    with (recording / "telemetry.csv").open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    # Even a forged matching CSV hash and passed flag cannot bypass the physical gates.
    summary["telemetry_sha256"] = walking_evidence.digest(recording / "telemetry.csv")
    (recording / "summary.json").write_text(json.dumps(summary))
    assert walking_evidence.verify_recording(recording)


@pytest.fixture
def protocols(tmp_path, monkeypatch):
    for number in (11, 12, 13):
        lab = next(ROOT.glob(f"lab-{number}-*"))
        target = tmp_path / lab.name / "media"
        for relative in [
            "demo/summary.json",
            *(["legacy/summary.json"] if number == 11 else ["benchmark/matrix.json"]),
        ]:
            path = target / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(lab / "media" / relative, path)
    monkeypatch.setattr(build_release, "ROOT", tmp_path)
    monkeypatch.setattr(build_release, "verify_recording", lambda *a, **kw: [])
    monkeypatch.setattr(build_release, "verify_trial", lambda *a, **kw: [])
    return tmp_path


@pytest.mark.parametrize(
    "mutation",
    [
        "missing_trial",
        "changed_environment",
        "changed_controller",
        "unbounded_adjustment",
    ],
)
def test_physical_protocol_tampering_rejected(protocols, mutation):
    path = next(protocols.glob("lab-13*/media/benchmark/matrix.json"))
    matrix = json.loads(path.read_text())
    if mutation == "missing_trial":
        matrix["reports"].pop()
    elif mutation == "changed_environment":
        matrix["reports"][0]["case"]["friction"] = 1.5
    elif mutation == "changed_controller":
        matrix["reports"][0]["controller_settings"]["qp_tolerance"] = 0.01
    else:
        row = next(r for r in matrix["reports"] if r["adaptation"])
        row["adaptation"][0]["delta_m"][0] = 1.0
    path.write_text(json.dumps(matrix))
    assert build_release.advanced_walking_errors()
