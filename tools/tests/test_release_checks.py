"""Release gates must reject missing translations and misleading RRT evidence."""

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "release_checks", TOOLS / "build_release.py"
)
release = importlib.util.module_from_spec(spec)
spec.loader.exec_module(release)


def test_all_learning_notes_have_same_topic_language_switches():
    assert release.documentation_errors(release.release_files()) == []


def test_missing_translation_and_wrong_topic_navigation_fail(tmp_path, monkeypatch):
    monkeypatch.setattr(release, "ROOT", tmp_path)
    path = tmp_path / "docs"
    path.mkdir()
    en, ko, zh = [path / f"NOTE{s}.md" for s in ("", ".ko", ".zh-CN")]
    nav = "[English](NOTE.md) · [한국어](NOTE.ko.md) · [简体中文](NOTE.zh-CN.md)"
    for p in (en, ko, zh):
        p.write_text(nav)
    assert release.documentation_errors([en, ko, zh]) == []
    assert release.documentation_errors([en, ko])
    zh.write_text("[English](../README.md)")
    assert release.documentation_errors([en, ko, zh])


@pytest.fixture
def audit_tree(tmp_path, monkeypatch):
    source = release.ROOT / "lab-4-motion-planning/media"
    target = tmp_path / "lab-4-motion-planning/media"
    target.mkdir(parents=True)
    for name in ("planning_audit.json", "slalom_summary.json"):
        (target / name).write_bytes((source / name).read_bytes())
    monkeypatch.setattr(release, "ROOT", tmp_path)
    return target


def write_audit(target, audit, rehash=True):
    path = target / "planning_audit.json"
    path.write_text(json.dumps(audit))
    if rehash:
        p = target / "slalom_summary.json"
        summary = json.loads(p.read_text())
        summary["planning_audit_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
        p.write_text(json.dumps(summary))


def test_recorded_rrt_evidence_is_accepted(audit_tree):
    assert release.planning_audit_errors() == []


@pytest.mark.parametrize(
    "failure",
    [
        "unblocked",
        "removed_still_collides",
        "shortcut_collision",
        "timed_margin",
        "hash",
    ],
)
def test_false_rrt_success_is_rejected(audit_tree, failure):
    audit = json.loads((audit_tree / "planning_audit.json").read_text())
    if failure in ("unblocked", "removed_still_collides"):
        for segment in audit["direct_control"]["segments"]:
            if failure == "unblocked":
                segment["direct"]["collision_or_joint_limit_samples"] = 0
            else:
                segment["without_obstacles_collision_samples"] = 1
    elif failure == "shortcut_collision":
        audit["segments"][0]["shortcut"]["collision_or_joint_limit_samples"] = 1
    elif failure == "timed_margin":
        audit["timed_trajectory"]["minimum_geometry_clearance_m"] = 0.029
    else:
        audit["tampered"] = True
    write_audit(audit_tree, audit, rehash=failure != "hash")
    assert release.planning_audit_errors()


@pytest.fixture
def benchmark_tree(tmp_path, monkeypatch):
    source = release.ROOT / "lab-4-motion-planning"
    target = tmp_path / "lab-4-motion-planning"
    (target / "media/planning_benchmark").mkdir(parents=True)
    (target / "scenarios").mkdir()
    for path in (source / "scenarios").glob("robot_*.json"):
        (target / "scenarios" / path.name).write_bytes(path.read_bytes())
    matrix = target / "media/planning_benchmark/matrix.json"
    matrix.write_bytes((source / "media/planning_benchmark/matrix.json").read_bytes())
    monkeypatch.setattr(release, "ROOT", tmp_path)
    return matrix


def test_multi_scene_benchmark_is_complete(benchmark_tree):
    assert release.planning_benchmark_errors() == []


@pytest.mark.parametrize(
    "failure",
    [
        "missing_trial",
        "changed_scene",
        "unsafe",
        "different_config",
        "missing_execution",
    ],
)
def test_benchmark_cannot_hide_failures_or_change_cases(benchmark_tree, failure):
    data = json.loads(benchmark_tree.read_text())
    row = next(
        r
        for r in data["reports"]
        if r["domain"] == "robot" and r["algorithm"] == "rrt_star" and r["seed"] == 0
    )
    if failure == "missing_trial":
        data["reports"].pop()
    elif failure == "changed_scene":
        data["scene_sha256"]["robot_101.json"] = "wrong"
    elif failure == "unsafe":
        row["safety_passed"] = False
    elif failure == "different_config":
        row["config"]["max_iter"] = 1
    else:
        row.pop("execution")
    benchmark_tree.write_text(json.dumps(data))
    assert release.planning_benchmark_errors()
