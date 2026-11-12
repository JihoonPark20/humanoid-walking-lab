"""Ensure visual comparisons use matching experiments and valid source crops."""

import copy
import json
from pathlib import Path

import pytest

from tools import build_gallery

ROOT = Path(__file__).resolve().parents[2]


def comparison_entry(number):
    """Read the actual presentation recipe used by the learning repository."""
    catalog = json.loads((ROOT / "tools/gallery_catalog.json").read_text())
    return next(entry for entry in catalog["labs"] if entry["number"] == number)


@pytest.mark.parametrize("number", [11, 12, 13])
def test_comparison_uses_matching_physical_runs(number):
    build_gallery.check_comparison(comparison_entry(number))


@pytest.mark.parametrize("changed", ["controller", "case", "implementation", "video"])
def test_mismatched_comparison_is_rejected(monkeypatch, changed):
    entry = comparison_entry(13)
    actual = build_gallery.recording_result
    other = copy.deepcopy(actual(entry["comparison"]))
    if changed == "controller":
        other["controller_settings"]["step_width"] = 0.3
    elif changed == "case":
        other["case"]["push_start"] += 0.2
    elif changed == "implementation":
        other["source_sha256"] = {}
    else:
        other["video_sha256"] = "stale"
        monkeypatch.setattr(build_gallery.json, "loads", lambda _: other)
        with pytest.raises(ValueError, match="matching passing result"):
            build_gallery.recording_result(entry["comparison"])
        return
    monkeypatch.setattr(
        build_gallery,
        "recording_result",
        lambda item: other if item is entry["comparison"] else actual(item),
    )
    with pytest.raises(ValueError, match="Comparison"):
        build_gallery.check_comparison(entry)


@pytest.mark.parametrize(
    "crop", [[-1, 0, 20, 20], [900, 0, 100, 100], [0, 0, 0, 20], [0, 0, "200", 20]]
)
def test_crop_cannot_invent_pixels(crop):
    with pytest.raises(ValueError, match="Crop"):
        build_gallery.crop_filter(crop, (960, 640))
