"""Validate the curated curriculum and package it without local environments/models."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path
from urllib.parse import unquote, urlsplit
from zipfile import ZIP_DEFLATED, ZipFile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.build_gallery import check_comparison
from tools.walking_evidence import verify_recording, verify_trial

ROOT_FILES = {
    "README.md",
    "README.ko.md",
    "README.zh-CN.md",
    "LICENSE",
    "NOTICE",
    "THIRD_PARTY_NOTICES.md",
    "requirements.txt",
    "conftest.py",
    ".gitignore",
}
EXCLUDED = {
    "__pycache__",
    ".pytest_cache",
    ".ruff_cache",
    "mujoco_menagerie",
    "Universal_Robots_ROS2_Description",
    "tasks",
    "ros2_bridge",
}
SUFFIXES = {
    ".py",
    ".md",
    ".xml",
    ".urdf",
    ".csv",
    ".npz",
    ".json",
    ".png",
    ".gif",
    ".mp4",
    ".txt",
    ".sh",
}


def release_files() -> list[Path]:
    """Select only teaching files, measurements, previews, setup and licenses."""
    catalog = json.loads((ROOT / "tools/gallery_catalog.json").read_text())
    files = [ROOT / name for name in ROOT_FILES]
    for directory in ["docs", "tools", *[lab["path"] for lab in catalog["labs"]]]:
        for path in (ROOT / directory).rglob("*"):
            relative = path.relative_to(ROOT)
            if any(part in EXCLUDED or part.startswith(".") for part in relative.parts):
                continue
            if not path.is_file() or not path.resolve().is_relative_to(ROOT):
                continue
            if path.name.endswith(".pending.mp4"):
                continue
            if "third_party" in path.resolve().relative_to(ROOT).parts:
                continue
            if path.suffix in SUFFIXES or path.name.startswith("LICENSE"):
                files.append(path)
    return sorted(set(files))


def documentation_errors(files: list[Path]) -> list[str]:
    """Every public learning note must have same-topic EN/KO/ZH navigation."""
    included = {p.resolve() for p in files}
    errors = []
    canonical = [
        p
        for p in files
        if p.suffix == ".md"
        and not p.name.endswith((".ko.md", ".zh-CN.md"))
        and (
            "docs" in p.relative_to(ROOT).parts or p.name in {"README.md", "GALLERY.md"}
        )
    ]
    for path in canonical:
        variants = [
            path.with_name(path.stem + suffix + ".md")
            for suffix in ("", ".ko", ".zh-CN")
        ]
        expected = f"[English]({variants[0].name}) · [한국어]({variants[1].name}) · [简体中文]({variants[2].name})"
        for variant in variants:
            if variant.resolve() not in included:
                errors.append(f"Missing translated note: {variant.relative_to(ROOT)}")
            elif expected not in variant.read_text():
                errors.append(
                    f"Missing same-topic language switch: {variant.relative_to(ROOT)}"
                )
    return errors


def planning_audit_errors() -> list[str]:
    """Reject a plausible-looking RRT video without genuine obstacle avoidance."""
    lab = ROOT / "lab-4-motion-planning/media"
    path = lab / "planning_audit.json"
    if not path.exists():
        return ["Missing RRT planning audit"]
    audit = json.loads(path.read_text())
    summary = json.loads((lab / "slalom_summary.json").read_text())
    errors = []
    if (
        summary.get("planning_audit_sha256")
        != hashlib.sha256(path.read_bytes()).hexdigest()
    ):
        errors.append("RRT planning audit does not match the recorded execution")
    direct = audit.get("direct_control", {})
    blocked = [
        s
        for s in direct.get("segments", [])
        if s.get("obstacle_blocks_direct_path") is True
        and s.get("direct", {}).get("collision_or_joint_limit_samples", 0) > 0
        and s.get("direct", {}).get("minimum_geometry_clearance_m", 0) < -1e-4
        and s.get("without_obstacles_collision_samples") == 0
    ]
    stages = audit.get("segments", [])
    if (
        audit.get("passed") is not True
        or direct.get("passed") is not True
        or not blocked
        or direct.get("edge_resolution_rad", float("inf")) > 0.005
        or len(stages) != len(direct.get("segments", []))
        or not stages
        or audit.get("velocity_acceleration_limits") is not True
    ):
        errors.append(
            "RRT lacks a blocked-direct-path control and validated planning stages"
        )
    reports = [audit.get("timed_trajectory", {})]
    reports += [stage.get(key, {}) for stage in stages for key in ("raw", "shortcut")]
    for report in reports:
        if (
            report.get("passed") is not True
            or report.get("samples", 0) < 2
            or report.get("collision_or_joint_limit_samples") != 0
            or report.get("clearance_violation_samples") != 0
            or not report.get("minimum_geometry_clearance_m", -1) >= 0.03
            or report.get("required_clearance_m", 0) < 0.03
        ):
            errors.append(
                "RRT raw, shortcut or timed path failed independent clearance validation"
            )
            break
    if (
        summary.get("max_contact_count") != 0
        or not summary.get("min_executed_geometry_clearance_m", -1) >= 0.03
        or summary.get("geometry_clearance_interval_s", float("inf")) > 0.001
    ):
        errors.append("RRT execution failed dense physical collision checks")
    return errors


def planning_benchmark_errors() -> list[str]:
    """Require complete, uncensored multi-environment RRT evidence."""
    path = ROOT / "lab-4-motion-planning/media/planning_benchmark/matrix.json"
    if not path.exists():
        return ["Missing multi-environment RRT benchmark"]
    matrix = json.loads(path.read_text())
    errors = []
    seeds = {0, 7, 23, 42, 91}
    planar = {
        "free",
        "wide_detour",
        "narrow_gap",
        "offset_gates",
        "separating_wall",
        "invalid_goal",
    }
    robot = {f"robot_{seed}" for seed in range(101, 107)}
    expected = {
        (name, algo, seed)
        for name in planar | robot
        for algo in ("rrt", "rrt_star")
        for seed in seeds
    }
    rows = matrix.get("reports", [])
    actual = {(r.get("scene"), r.get("algorithm"), r.get("seed")) for r in rows}
    if actual != expected or len(rows) != len(expected):
        errors.append("RRT benchmark omits or duplicates scene/algorithm/seed trials")
    if (
        matrix.get("passed") is not True
        or not matrix.get("checks")
        or not all(v is True for v in matrix["checks"].values())
    ):
        errors.append("RRT benchmark validity/safety checks failed")
    for name in robot:
        scene = ROOT / f"lab-4-motion-planning/scenarios/{name}.json"
        if (
            not scene.exists()
            or matrix.get("scene_sha256", {}).get(scene.name)
            != hashlib.sha256(scene.read_bytes()).hexdigest()
        ):
            errors.append(f"RRT benchmark has stale scene geometry: {name}")
    for row in rows:
        if row.get("safety_passed") is not True:
            errors.append("RRT benchmark contains an unsafe returned path")
        if row.get("success") != (row.get("status") == "success"):
            errors.append("RRT benchmark has contradictory solution status")
        if row.get("scene") in {"separating_wall", "invalid_goal"} and row.get(
            "success"
        ):
            errors.append("RRT benchmark falsely solved a negative control")
        if row.get("domain") == "robot":
            if row.get("config") != matrix.get("config"):
                errors.append("RRT benchmark uses scene-specific planner settings")
            if row.get("success"):
                audits = row.get("audits", {})
                if set(audits) != {"raw", "shortcut", "timed"} or not all(
                    a.get("passed") is True for a in audits.values()
                ):
                    errors.append(
                        "RRT benchmark solution lacks three independent audits"
                    )
            if row.get("algorithm") == "rrt_star" and row.get("seed") == 0:
                execution = row.get("execution", {})
                if row.get("success") and execution.get("passed") is not True:
                    errors.append(
                        "RRT benchmark lacks the preselected physical execution"
                    )
                if (
                    not row.get("success")
                    and execution.get("status") != "not_run_no_plan"
                ):
                    errors.append("RRT benchmark hides a skipped physical execution")
    return errors


def advanced_walking_errors() -> list[str]:
    """Require complete physical protocols and independently audited new recordings."""
    errors = []
    shared_settings = None
    for number in (11, 12, 13):
        labs = list(ROOT.glob(f"lab-{number}-*"))
        if len(labs) != 1:
            errors.append(f"Missing advanced walking Lab {number}")
            continue
        media = labs[0] / "media"
        for directory in [
            media / "demo",
            *([media / "legacy"] if number == 11 else []),
        ]:
            if not (directory / "summary.json").exists():
                errors.append(f"Missing physical recording evidence: {directory}")
                continue
            errors.extend(
                verify_recording(directory, require_video=directory.name == "demo")
            )
        if number == 11:
            continue
        path = media / "benchmark/matrix.json"
        if not path.exists():
            errors.append(f"Missing physical benchmark: Lab {number}")
            continue
        matrix = json.loads(path.read_text())
        reports = matrix.get("reports", [])
        if (
            matrix.get("passed") is not True
            or len(reports) != 10
            or matrix.get("total") != 10
        ):
            errors.append(f"Invalid physical benchmark coverage: Lab {number}")
        successes = sum(r.get("passed") is True for r in reports)
        if successes != matrix.get("successful_cases"):
            errors.append("Physical benchmark success count is inconsistent")
        if number == 12:
            names = {
                "nominal",
                "push_left",
                "push_right",
                "push_forward",
                "push_backward",
                "friction_04",
                "mass_plus_5pct",
                "mass_minus_5pct",
                "later_push",
                "combined",
            }
            if {
                r.get("case", {}).get("name") for r in reports
            } != names or successes < 8:
                errors.append(
                    "Physical robustness protocol incomplete or below acceptance"
                )
            for name in ("nominal", "push_left"):
                if not any(
                    r.get("case", {}).get("name") == name and r.get("passed")
                    for r in reports
                ):
                    errors.append("Preselected physical robustness case failed")
        else:
            expected = {
                (force, policy)
                for force in (0.0, 10.0, -10.0, 20.0, -20.0)
                for policy in ("fixed", "adaptive")
            }
            if {
                (r.get("case", {}).get("push_force"), r.get("policy")) for r in reports
            } != expected:
                errors.append("Adaptive-footstep paired protocol incomplete")
            if not all(
                r.get("passed") for r in reports if r.get("policy") == "adaptive"
            ):
                errors.append("Adaptive-footstep physical case failed")
            if not any(
                r.get("case", {}).get("push_force") == 10
                and r.get("policy") == "adaptive"
                and r.get("adaptation")
                for r in reports
            ):
                errors.append("Preselected adaptive case did not adjust a footstep")
        for report in reports:
            case = report["case"]
            name = case["name"]
            if number == 12:
                overrides = {
                    "nominal": {},
                    "push_left": {"push_force": 10},
                    "push_right": {"push_force": -10},
                    "push_forward": {"push_force": 10, "push_axis": 0},
                    "push_backward": {"push_force": -10, "push_axis": 0},
                    "friction_04": {"friction": 0.4},
                    "mass_plus_5pct": {"mass_ratio": 1.05},
                    "mass_minus_5pct": {"mass_ratio": 0.95},
                    "later_push": {"push_force": 10, "push_start": 2.5},
                    "combined": {"mass_ratio": 1.05, "friction": 0.4, "push_force": 10},
                }
                changes = overrides.get(name, {})
            else:
                changes = {"push_force": case["push_force"]}
            expected_case = {
                "name": name,
                "steps": 4,
                "step_length": 0.08,
                "friction": 0.6,
                "mass_ratio": 1.0,
                "push_force": 0.0,
                "push_axis": 1,
                "push_start": 1.6,
                "push_duration": 0.15,
            }
            expected_case.update(changes)
            if case != expected_case:
                errors.append("Physical benchmark silently changed an environment")
            settings = report.get("controller_settings")
            if shared_settings is None:
                shared_settings = settings
            if not settings or settings != shared_settings:
                errors.append("Physical benchmarks use case-dependent controllers")
            if report.get("passed") != all(report.get("checks", {}).values()):
                errors.append("Physical acceptance flags contradict recorded checks")
            folder = name if number == 12 else f"{name}_{report['policy']}"
            errors.extend(
                verify_trial(report, media / "benchmark" / folder / "telemetry.csv")
            )
            for event in report.get("adaptation", []):
                if event.get("timing_changed") is not False or any(
                    not lo - 1e-10 <= delta <= hi + 1e-10
                    for delta, lo, hi in zip(
                        event["delta_m"], event["lower_m"], event["upper_m"]
                    )
                ):
                    errors.append(
                        "Footstep adjustment violates its recorded constraints"
                    )
    return errors


def validate(files: list[Path]) -> None:
    """Reject broken local document links and unsuccessful published results."""
    included = {path.resolve() for path in files}
    errors = (
        documentation_errors(files)
        + planning_audit_errors()
        + planning_benchmark_errors()
        + advanced_walking_errors()
    )
    catalog = json.loads((ROOT / "tools/gallery_catalog.json").read_text())
    labs = catalog["labs"]
    if [lab["number"] for lab in labs] != list(range(1, len(labs) + 1)):
        errors.append("Curriculum numbers must run consecutively from 1.")
    for lab in labs:
        if not lab["path"].startswith(f"lab-{lab['number']}-"):
            errors.append(f"Curriculum number does not match folder: {lab['path']}")
        source = ROOT / lab["source"]
        if source.suffix == ".mp4":
            evidence = ROOT / lab["verification"]
            result = json.loads(evidence.read_text())
            checksum = hashlib.sha256(source.read_bytes()).hexdigest()
            if (
                result.get("passed") is not True
                or result.get("video_sha256") != checksum
            ):
                errors.append(f"Video lacks a matching passing result: {lab['source']}")
    for path in files:
        if path.suffix == ".md":
            content = re.sub(r"```.*?```", "", path.read_text(), flags=re.DOTALL)
            links = re.findall(r"\]\(([^)]+)\)", content)
            links += re.findall(r"(?:src|href)=[\"\']([^\"\']+)", content)
            for link in links:
                link = link.strip().split(' "')[0].strip("<>")
                url = urlsplit(link)
                if url.scheme or url.netloc or not url.path:
                    continue
                target = (path.parent / unquote(url.path)).resolve()
                if target not in included and not (
                    target.is_dir() and any(p.is_relative_to(target) for p in included)
                ):
                    errors.append(
                        f"{path.relative_to(ROOT)}: missing release link {link}"
                    )
        elif path.suffix == ".json" and "media" in path.parts:
            data = json.loads(path.read_text())
            if isinstance(data, dict) and any(
                data.get(key) is False for key in ("passed", "all_passed")
            ):
                errors.append(
                    f"{path.relative_to(ROOT)}: unsuccessful published result"
                )
    for lab in (9, 10):
        lab_dir = next(ROOT.glob(f"lab-{lab}-*"))
        names = ["validated_scenarios"]
        if lab == 9:
            names.append("observation_benchmark")
        for name in names:
            matrix = lab_dir / "media" / name / "matrix.json"
            if (
                matrix not in files
                or json.loads(matrix.read_text()).get("all_passed") is not True
            ):
                errors.append(f"Missing passing matrix: {matrix.relative_to(ROOT)}")
    manifest_path = ROOT / "docs/gallery_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    catalog_hash = hashlib.sha256(
        (ROOT / "tools/gallery_catalog.json").read_bytes()
    ).hexdigest()
    if manifest.get("catalog_sha256") != catalog_hash:
        errors.append(
            "Gallery manifest is stale; rebuild previews after changing the catalog"
        )
    records = {r["lab"]: r for r in manifest.get("labs", [])}
    for lab in labs:
        assets = records.get(lab["number"], {}).get("assets", [])
        source = ROOT / lab["source"]
        if source.suffix != ".mp4":
            continue
        recordings = [lab]
        if "comparison" in lab:
            recordings.append(lab["comparison"])
            try:
                check_comparison(lab)
            except (ValueError, KeyError, FileNotFoundError) as exc:
                errors.append(f"Invalid gallery comparison: {lab['path']}: {exc}")
        for recording in recordings:
            matching = [a for a in assets if a.get("source") == recording["source"]]
            if not matching:
                errors.append(f"Missing gallery evidence: {recording['source']}")
                continue
            asset = matching[0]
            output = ROOT / asset["output"]
            clip = asset.get("clip", {})
            expected_clip = {
                "start_s": lab["start_s"],
                "source_duration_s": lab["duration_s"],
                "speed": lab["speed"],
                "crop": lab.get("crop"),
                "layout": "comparison_left_primary_right"
                if "comparison" in lab
                else "single",
            }
            if (
                asset.get("sha256")
                != hashlib.sha256((ROOT / recording["source"]).read_bytes()).hexdigest()
                or asset.get("verification_sha256")
                != hashlib.sha256(
                    (ROOT / recording["verification"]).read_bytes()
                ).hexdigest()
                or output.resolve() not in included
                or not output.exists()
                or asset.get("output_sha256")
                != hashlib.sha256(output.read_bytes()).hexdigest()
                or any(clip.get(key) != value for key, value in expected_clip.items())
            ):
                errors.append(
                    f"Gallery does not match its recording or crop: {recording['source']}"
                )
    if errors:
        raise ValueError("\n".join(errors))


def main() -> None:
    """Check the release or write a portable archive with one top-level folder."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    parser.add_argument(
        "--output", type=Path, default=ROOT.parent / "humanoid-walking-lab.zip"
    )
    args = parser.parse_args()
    files = release_files()
    validate(files)
    print(f"Validated {len(files)} release files.")
    if args.check:
        return
    output = args.output.resolve()
    if output.exists():
        parser.error(f"Output already exists: {output}; choose another --output path.")
    output.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(output, "w", compression=ZIP_DEFLATED, compresslevel=6) as archive:
        for path in files:
            archive.write(path, Path("humanoid-walking-lab") / path.relative_to(ROOT))
    print(f"Created {output} ({output.stat().st_size / 1024**2:.1f} MiB).")


if __name__ == "__main__":
    main()
