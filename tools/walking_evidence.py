"""Independent file-level gates for the advanced G1 demonstrations."""

from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_fingerprints():
    """Track the complete controller chain, including the new fixed protocols."""
    files = [
        ROOT / "tools/walking_visuals.py",
        ROOT / "tools/walking_evidence.py",
        ROOT / "requirements.txt",
    ]
    for prefix in (5, 6, 11, 12, 13):
        for lab in ROOT.glob(f"lab-{prefix}-*"):
            files.extend((lab / "src").glob("*.py"))
    return {str(p.relative_to(ROOT)): digest(p) for p in sorted(files)}


def verify_trial(summary, telemetry, *, check_sources=True):
    """Independently reconstruct successful-trial gates from physical samples."""
    errors = []
    if not telemetry.exists() or digest(telemetry) != summary.get("telemetry_sha256"):
        return ["Missing or changed physical telemetry"]
    rows = list(csv.DictReader(telemetry.open()))
    if check_sources and summary.get("source_sha256") != source_fingerprints():
        errors.append("Recording uses a different controller implementation")
    if not summary.get("passed"):
        return errors
    if (
        not rows
        or not summary.get("checks")
        or not all(summary.get("checks", {}).values())
    ):
        errors.append("Recording lacks passing physical gates")
    if rows:
        dt = summary["sample_interval_s"]
        if any(abs(float(r["t"]) - (i + 1) * dt) > 1e-8 for i, r in enumerate(rows)):
            errors.append("Telemetry does not cover every physical integration tick")
        if abs(float(rows[-1]["t"]) - summary["duration_s"]) > dt / 2:
            errors.append("Incomplete physical recording")
        if any(not (int(r["left_contact"]) or int(r["right_contact"])) for r in rows):
            errors.append("Measured flight in physical recording")
        if any(
            int(r["clipped"])
            or int(r["unexpected_contact"])
            or not int(r["qp_converged"])
            for r in rows
        ):
            errors.append(
                "Clipping, unexpected contact or unconverged QP in physical recording"
            )
        if max(float(r["joint_violation_rad"]) for r in rows) > 0.002:
            errors.append("Physical joint limit violation")
        fraction = sum(float(r["zmp_margin_m"]) >= 0 for r in rows) / len(rows)
        if fraction < 0.9 or abs(fraction - summary["zmp_inside_fraction"]) > 1e-10:
            errors.append("ZMP gate disagrees with physical telemetry")
        limits = {
            "base_dynamics": 0.02,
            "contact_acceleration": 0.01,
            "contact_wrench": 0.05,
            "raw_torque": 1e-6,
            "torque_buffer": 0.01,
        }
        for key, limit in limits.items():
            actual = max(float(r[f"residual_{key}"]) for r in rows)
            if actual > limit or abs(actual - summary["qp_violations"][key]) > 1e-9:
                errors.append(f"QP residual gate disagrees: {key}")
        for landing in summary.get("landings", []):
            side = landing["side"]
            target = landing["target"]
            swing = [
                r for r in rows if landing["start_s"] < float(r["t"]) < landing["end_s"]
            ]
            after = next(
                (r for r in rows if float(r["t"]) >= landing["end_s"] + 0.1), None
            )
            if not swing or not any(not int(r[f"{side}_contact"]) for r in swing):
                errors.append("Swing never left the ground")
            elif max(float(r[f"{side}_z"]) - target[2] for r in swing) < 0.008:
                errors.append("Physical swing clearance too small")
            if (
                after is None
                or not int(after[f"{side}_contact"])
                or math.dist([float(after[f"{side}_{axis}"]) for axis in "xyz"], target)
                > 0.03
            ):
                errors.append("Physical landing failed")
        if len(summary.get("landings", [])) != summary["case"]["steps"]:
            errors.append("Missing planned landings")
        if (
            float(rows[-1]["cx"]) - float(rows[0]["cx"])
            < 0.8 * summary["planned_distance_m"] - 0.001
        ):
            errors.append("Insufficient physical forward progress")
        for index, axis in enumerate("xy"):
            impulse = dt * sum(float(r[f"push_{axis}_n"]) for r in rows)
            expected = (
                summary["case"]["push_force"] * summary["case"]["push_duration"]
                if index == summary["case"]["push_axis"]
                else 0.0
            )
            if (
                abs(impulse - expected) > abs(summary["case"]["push_force"]) * dt + 1e-8
                or abs(impulse - summary["applied_impulse_ns"][index]) > 1e-8
            ):
                errors.append("Requested disturbance was not physically applied")
    return errors


def verify_recording(directory, *, require_video=True, check_sources=True):
    """Require a passing physical run and its exact recording."""
    summary = json.loads((directory / "summary.json").read_text())
    errors = verify_trial(
        summary, directory / "telemetry.csv", check_sources=check_sources
    )
    if summary.get("passed") is not True:
        errors.append("Unsuccessful demonstration")
    if require_video:
        video = directory / "walk.mp4"
        if not video.exists() or digest(video) != summary.get("video_sha256"):
            errors.append("Video does not match its physical run")
    return errors
