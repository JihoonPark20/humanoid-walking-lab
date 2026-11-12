"""Validate the curated curriculum and package it without local environments/models."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path
from urllib.parse import unquote, urlsplit
from zipfile import ZIP_DEFLATED, ZipFile

ROOT = Path(__file__).resolve().parents[1]
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
            if "third_party" in path.resolve().relative_to(ROOT).parts:
                continue
            if path.suffix in SUFFIXES or path.name.startswith("LICENSE"):
                files.append(path)
    return sorted(set(files))


def validate(files: list[Path]) -> None:
    """Reject broken local document links and unsuccessful published results."""
    included = {path.resolve() for path in files}
    errors = []
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
