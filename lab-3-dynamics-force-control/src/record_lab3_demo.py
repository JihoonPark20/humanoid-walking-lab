"""Record the tested line-tracing simulation and publish metrics from that same run."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from collections.abc import Callable
from dataclasses import asdict
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from c1_force_control import HybridGains
from c2_line_trace import plot_line_trace, run_line_trace
from lab3_common import MEDIA_DIR, TABLE_SURFACE_Z

START = np.array([0.40, 0.0])
END = np.array([0.45, 0.0])
APPROACH_HEIGHT = TABLE_SURFACE_Z + 0.04
FPS = 30


def demo_gains() -> HybridGains:
    """Return the documented sliding-contact settings, shared by recording/tests."""
    return HybridGains(K_p=2000, K_d=100, K_fp=0.5, K_fi=3, K_R=60, K_w=1)


def run_demo(
    frame_callback: Callable[[mujoco.MjModel, mujoco.MjData, dict], None] | None = None,
) -> dict[str, np.ndarray]:
    """Run the exact release scenario, with an optional read-only video observer."""
    return run_line_trace(
        START,
        END,
        gains=demo_gains(),
        approach_height=APPROACH_HEIGHT,
        settle_time=1.5,
        trace_duration=6,
        post_time=0.5,
        descent_speed=0.01,
        frame_callback=frame_callback,
    )


def summarize(results: dict[str, np.ndarray]) -> dict:
    """Check completion, raw force, tool attitude and allowed contacts independently."""
    trace = results["phase"] == 2
    if not np.any(trace):
        return {"passed": False, "checks": {"trace_started": False}}
    raw = results["force_raw_z"][trace]
    filtered = results["force_z"][trace]
    xy_error = np.linalg.norm(
        results["ee_pos"][trace, :2] - results["xy_des"][trace], axis=1
    )
    raw_band = float(np.mean(np.abs(raw - 5) < 1))
    filtered_band = float(np.mean(np.abs(filtered - 5) < 1))
    angle = float(np.rad2deg(results["orientation_error"]).max())
    unexpected = int(np.sum(results["unexpected_contacts"]))
    checks = {
        "completed": bool(
            results["phase"][-1] == 3
            and np.allclose(results["xy_des"][-1], END, atol=1e-6)
        ),
        "raw_force_in_band": raw_band > 0.9,
        "filtered_force_in_band": filtered_band > 0.9,
        "peak_contact_force": float(results["force_raw_z"].max()) < 15,
        "xy_tracking": float(xy_error.max()) < 0.005,
        "orientation": angle < 5,
        "probe_only_contact": unexpected == 0,
        "gripper_closed": bool(np.min(results["gripper_angles"]) > 0.65),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "force_target_n": 5,
        "raw_force_in_band_fraction": raw_band,
        "filtered_force_in_band_fraction": filtered_band,
        "trace_mean_force_n": float(raw.mean()),
        "peak_force_n": float(results["force_raw_z"].max()),
        "max_xy_error_m": float(xy_error.max()),
        "max_orientation_error_deg": angle,
        "unexpected_contact_samples": unexpected,
        "min_gripper_driver_angle_rad": float(results["gripper_angles"].min()),
        "duration_s": float(results["time"][-1]),
        "trace_start_s": float(results["time"][trace][0]),
        "trace_end_s": float(results["time"][trace][-1]),
    }


class TraceRecorder:
    """Observe live simulator states without changing control, contacts or timing."""

    def __init__(self, output: Path) -> None:
        self.output = output
        self.renderer = None
        self.writer = None
        self.frames = 0
        self.camera = mujoco.MjvCamera()
        self.camera.lookat[:] = [0.40, 0.0, 0.46]
        self.camera.distance = 0.85
        self.camera.azimuth = 105
        self.camera.elevation = -25

    def __call__(
        self, model: mujoco.MjModel, data: mujoco.MjData, sample: dict
    ) -> None:
        """Render at exact simulation-time intervals, including approach and settling."""
        if sample["time"] + 1e-9 < self.frames / FPS:
            return
        if self.renderer is None:
            self.renderer = mujoco.Renderer(model, height=640, width=960)
            self.writer = imageio.get_writer(str(self.output), fps=FPS, codec="libx264")
        self.renderer.update_scene(data, camera=self.camera)
        scene = self.renderer.scene
        for xy in np.linspace(START, END, 30):
            mujoco.mjv_initGeom(
                scene.geoms[scene.ngeom],
                mujoco.mjtGeom.mjGEOM_SPHERE,
                np.array([0.0018, 0, 0]),
                np.r_[xy, TABLE_SURFACE_Z + 0.001],
                np.eye(3).flatten(),
                np.array([0.1, 0.9, 0.3, 1], dtype=np.float32),
            )
            scene.ngeom += 1
        self.writer.append_data(self.renderer.render())
        self.frames += 1

    def close(self) -> None:
        """Finalize the encoder and free the offscreen context."""
        if self.writer is not None:
            self.writer.close()
        if self.renderer is not None:
            self.renderer.close()


def main() -> None:
    """Record once, evaluate that run, and refuse to publish a failed recording."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=MEDIA_DIR)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    temporary = args.output / "line_trace.pending.mp4"
    recorder = TraceRecorder(temporary)
    try:
        results = run_demo(frame_callback=recorder)
    finally:
        recorder.close()
    summary = summarize(results)
    print(json.dumps(summary, indent=2))
    if not summary["passed"]:
        temporary.unlink(missing_ok=True)
        raise SystemExit("Recording rejected: line-tracing checks failed.")
    movie = args.output / "lab3_simulation.mp4"
    temporary.replace(movie)
    summary.update(
        {
            "video_sha256": hashlib.sha256(movie.read_bytes()).hexdigest(),
            "gains": asdict(demo_gains()),
            "approach_height_m": APPROACH_HEIGHT,
            "fps": FPS,
            "frames": recorder.frames,
            "descent_speed_m_s": 0.01,
            "sliding_friction": 0.05,
            "environment": "Rigid massless probe on a flat low-friction plate; no articulated-finger contact.",
        }
    )
    (args.output / "line_trace_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n"
    )
    columns = [
        results[k]
        for k in (
            "time",
            "phase",
            "force_raw_z",
            "force_z",
            "orientation_error",
            "unexpected_contacts",
        )
    ]
    columns += [results["ee_pos"][:, :2], results["xy_des"]]
    np.savetxt(
        args.output / "line_trace.csv",
        np.column_stack(columns),
        delimiter=",",
        header="time,phase,force_raw_n,force_filtered_n,orientation_rad,unexpected_contacts,x,y,desired_x,desired_y",
        comments="",
    )
    plot_line_trace(results, START, END)


if __name__ == "__main__":
    main()
