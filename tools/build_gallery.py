"""Build compact, traceable previews from recorded experiments, without rerunning physics.

Run from any directory: python tools/build_gallery.py [--lab 9]. Source recordings
are never changed. Clip choices live in gallery_catalog.json; the manifest records
source hashes, exact intervals, speed and generated assets. Requires imageio[ffmpeg]
and Pillow. GIFs are presentation artifacts, not independent performance evidence.
"""

from __future__ import annotations

import argparse
import hashlib
import html
import json
import os
import subprocess
from pathlib import Path

import imageio.v2 as imageio
import imageio_ffmpeg
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
CATALOG = Path(__file__).with_name("gallery_catalog.json")


def digest(path: Path) -> str:
    """Hash source bytes so a preview can be associated with its exact recording."""
    checksum = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            checksum.update(chunk)
    return checksum.hexdigest()


def make_gif(source: Path, target: Path, entry: dict, width: int, fps: int) -> dict:
    """Apply a bounded clip and explicit playback speed using one shared palette."""
    reader = imageio.get_reader(source)
    metadata = reader.get_meta_data()
    reader.close()
    start = entry["start_s"]
    duration = min(entry["duration_s"], metadata["duration"] - start)
    if start < 0 or duration <= 0 or entry["speed"] <= 0:
        raise ValueError(f"Invalid clip: {source}")
    colors = entry.get("palette_colors", 96)
    palette_stats = entry.get("palette_stats", "diff")
    filters = (
        f"setpts=(PTS-STARTPTS)/{entry['speed']},fps={fps},"
        f"scale={width}:-1:flags=lanczos,split[a][b];"
        f"[a]palettegen=max_colors={colors}:stats_mode={palette_stats}:reserve_transparent=0[p];"
        "[b][p]paletteuse=dither=none"
    )
    subprocess.run(
        [
            imageio_ffmpeg.get_ffmpeg_exe(),
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-ss",
            str(start),
            "-t",
            str(duration),
            "-i",
            str(source),
            "-filter_complex",
            filters,
            "-loop",
            "0",
            str(target),
        ],
        check=True,
    )
    with Image.open(target) as gif:
        frames = gif.n_frames
        if frames < 2:
            raise ValueError(f"Preview has no animation: {target}")
    return {
        "start_s": start,
        "source_duration_s": duration,
        "speed": entry["speed"],
        "fps": fps,
        "frames": frames,
        "palette_colors": colors,
        "palette_stats": palette_stats,
        "size_bytes": target.stat().st_size,
    }


def build_lab(entry: dict, width: int, fps: int) -> dict:
    """Render a curated preview and link only explicitly selected teaching results."""
    lab = ROOT / entry["path"]
    source = ROOT / entry["source"]
    assets = []
    if source.suffix == ".mp4":
        evidence = ROOT / entry["verification"]
        result = json.loads(evidence.read_text())
        if result.get("passed") is not True or result.get("video_sha256") != digest(
            source
        ):
            raise ValueError(
                f"Video lacks a matching passing result; rerun its recorder: {source}"
            )
        target = lab / "media" / "gallery" / "hero.gif"
        target.parent.mkdir(parents=True, exist_ok=True)
        clip = make_gif(
            source, target, entry, entry.get("width", width), entry.get("fps", fps)
        )
        assets.append(
            {
                "source": entry["source"],
                "sha256": digest(source),
                "output": str(target.relative_to(ROOT)),
                "clip": clip,
                "verification": entry["verification"],
                "verification_sha256": digest(evidence),
            }
        )
    else:
        target = source
    write_gallery(entry, target)
    return {"lab": entry["number"], "assets": assets}


def write_gallery(entry: dict, target: Path) -> None:
    """Keep language switches on the same gallery and share measured assets."""
    lab = ROOT / entry["path"]
    source = ROOT / entry["source"]
    nav = (
        "[English](GALLERY.md) · [한국어](GALLERY.ko.md) · [简体中文](GALLERY.zh-CN.md)"
    )
    for language, suffix, labels in (
        ("en", "", ("Source recording / image", "Raw data (optional)", "Plots")),
        ("ko", ".ko", ("원본 영상 / 이미지", "원시 데이터 (선택)", "그래프")),
        ("zh", ".zh-CN", ("原始视频 / 图像", "原始数据（可选）", "图表")),
    ):
        text = [
            f"# Lab {entry['number']}: {entry['title'][language]}",
            nav,
            f"![Demo]({target.relative_to(lab).as_posix()})",
            entry["caption"][language],
            f"[{labels[0]}]({source.relative_to(lab).as_posix()})",
        ]
        if entry["plots"]:
            text.append(f"## {labels[2]}")
        for relative in entry["plots"]:
            path = (ROOT / relative).resolve()
            if not path.exists():
                raise FileNotFoundError(path)
            link = Path(os.path.relpath(path, lab)).as_posix()
            text.append(
                f'<img src="{link}" width="640" alt="{html.escape(path.stem)}">'
            )
        if entry["results"]:
            text.append(f"<details>\n<summary>{labels[1]}</summary>")
            links = []
            for relative in entry["results"]:
                path = ROOT / relative
                if not path.exists():
                    raise FileNotFoundError(path)
                links.append(
                    f"- [{path.parent.name}/{path.name}]({path.relative_to(lab).as_posix()})"
                )
            text.append("\n".join(links))
            text.append("</details>")
        (lab / f"GALLERY{suffix}.md").write_text("\n\n".join(text) + "\n")


def main() -> None:
    """Rebuild the curated gallery; adding generated files does not publish new demos."""
    catalog = json.loads(CATALOG.read_text())
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--lab", type=int, choices=[e["number"] for e in catalog["labs"]]
    )
    parser.add_argument(
        "--docs-only",
        action="store_true",
        help="Refresh all three gallery languages without rendering GIFs",
    )
    args = parser.parse_args()
    if args.docs_only:
        for entry in catalog["labs"]:
            if args.lab is None or entry["number"] == args.lab:
                source = ROOT / entry["source"]
                target = (
                    ROOT / entry["path"] / "media/gallery/hero.gif"
                    if source.suffix == ".mp4"
                    else source
                )
                write_gallery(entry, target)
        return
    manifest_path = ROOT / "docs" / "gallery_manifest.json"
    previous = (
        json.loads(manifest_path.read_text())
        if manifest_path.exists()
        else {"labs": []}
    )
    allowed = {e["number"] for e in catalog["labs"]}
    records = (
        {r["lab"]: r for r in previous["labs"] if r["lab"] in allowed}
        if args.lab
        else {}
    )
    for entry in catalog["labs"]:
        if args.lab is None or entry["number"] == args.lab:
            records[entry["number"]] = build_lab(
                entry, catalog["width"], catalog["fps"]
            )
            print(f"Lab {entry['number']}: gallery ready", flush=True)
    manifest_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "catalog_sha256": digest(CATALOG),
                "labs": [records[k] for k in sorted(records)],
            },
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
