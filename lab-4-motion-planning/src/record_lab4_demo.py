"""Record the actual capstone execution; metrics and video share one rollout."""

from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parent))
from capstone_demo import run_capstone
from lab4_common import MEDIA_DIR, MIN_CLEARANCE_M, SLALOM_WAYPOINTS


def _add_trail_to_scene(
    scene: mujoco.MjvScene,
    trail: np.ndarray,
    size: float = 0.003,
    rgba: tuple[float, float, float, float] = (0.15, 0.50, 0.95, 0.9),
) -> None:
    """Draw measured positions as visual-only markers, outside collision physics."""
    for point in trail:
        if scene.ngeom >= scene.maxgeom:
            break
        mujoco.mjv_initGeom(
            scene.geoms[scene.ngeom],
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([size, 0, 0]),
            point,
            np.eye(3).flatten(),
            np.array(rgba, dtype=np.float32),
        )
        scene.ngeom += 1


class Recorder:
    """Render only observed states; never compute a second control trajectory."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self.renderer = None
        self.writer = None
        self.frames = 0
        self.trail: list[np.ndarray] = []
        self.camera = mujoco.MjvCamera()
        self.camera.lookat[:] = [0.50, 0.0, 0.40]
        self.camera.distance = 1.45
        self.camera.elevation = -62
        self.camera.azimuth = 100

    def __call__(
        self, model: mujoco.MjModel, data: mujoco.MjData, sample: dict
    ) -> None:
        """Capture the execution state at 30 fps in simulation time."""
        if sample["time"] + 1e-9 < self.frames / 30:
            return
        if self.renderer is None:
            self.renderer = mujoco.Renderer(model, height=480, width=640)
            self.writer = imageio.get_writer(str(self.path), fps=30, codec="libx264")
        self.trail.append(sample["ee_pos"])
        self.renderer.update_scene(data, camera=self.camera)
        _add_trail_to_scene(self.renderer.scene, np.array(self.trail), size=0.0025)
        _add_trail_to_scene(
            self.renderer.scene,
            np.array(SLALOM_WAYPOINTS),
            size=0.008,
            rgba=(0.1, 0.85, 0.3, 1),
        )
        image = Image.fromarray(self.renderer.render())
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 0, 640, 42), fill=(24, 33, 45))
        draw.text(
            (12, 8),
            f"Lab 4 | Slalom | {sample['time']:.2f} / {sample['duration']:.2f} s | 1x",
            fill="white",
        )
        draw.text(
            (12, 25),
            f"Joint error: {sample['joint_error']:.4f} rad | contacts: {data.ncon}",
            fill="white",
        )
        self.writer.append_data(np.asarray(image))
        self.frames += 1

    def close(self) -> None:
        """Close the encoder and renderer."""
        if self.writer is not None:
            self.writer.close()
        if self.renderer is not None:
            self.renderer.close()


def main() -> None:
    """Publish only a complete, collision-free, accurately tracked execution."""
    MEDIA_DIR.mkdir(parents=True, exist_ok=True)
    pending = MEDIA_DIR / "slalom.pending.mp4"
    recorder = Recorder(pending)
    try:
        metrics = run_capstone(frame_callback=recorder)
    finally:
        recorder.close()
    result = metrics["result"]
    contacts = int(np.max(result["contact_counts"]))
    checks = {
        "tracking": metrics["rms"] < 0.05,
        "final_configuration": metrics["final_err"] < 0.05,
        "clearance": metrics["min_clearance"] >= MIN_CLEARANCE_M,
        "no_contacts": contacts == 0,
    }
    summary = {
        "passed": all(checks.values()),
        "checks": checks,
        "rms_error_rad": metrics["rms"],
        "final_error_rad": metrics["final_err"],
        "min_sampled_clearance_m": metrics["min_clearance"],
        "clearance_samples": len(metrics["clr_profile"]),
        "max_contact_count": contacts,
        "duration_s": float(result["time"][-1]),
        "fps": 30,
        "frames": recorder.frames,
    }
    print(json.dumps(summary, indent=2))
    if not summary["passed"]:
        pending.unlink(missing_ok=True)
        raise SystemExit("Slalom recording rejected: execution checks failed.")
    movie = MEDIA_DIR / "lab4_simulation.mp4"
    pending.replace(movie)
    summary["video_sha256"] = hashlib.sha256(movie.read_bytes()).hexdigest()
    (MEDIA_DIR / "slalom_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    np.savetxt(
        MEDIA_DIR / "slalom.csv",
        np.column_stack(
            [
                result["time"],
                result["q_actual"],
                result["q_desired"],
                result["contact_counts"],
            ]
        ),
        delimiter=",",
        header="time,q1,q2,q3,q4,q5,q6,desired1,desired2,desired3,desired4,desired5,desired6,contacts",
        comments="",
    )


if __name__ == "__main__":
    main()
