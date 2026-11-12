"""Render the six benchmark starting configurations; these are not rollout videos."""

from __future__ import annotations

from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np
from motion_planning import read_scene, scene_checker


def main():
    """Show actual scene geometry without text panels or simulated performance claims."""
    lab = Path(__file__).resolve().parents[1]
    output = lab / "media/planning_benchmark"
    output.mkdir(parents=True, exist_ok=True)
    for path in sorted((lab / "scenarios").glob("robot_*.json")):
        scene = read_scene(path)
        checker = scene_checker(scene)
        checker._forward(np.asarray(scene["start"]))
        model, data = checker.mj_model, checker.mj_data
        model.vis.global_.offwidth = 720
        model.vis.global_.offheight = 480
        camera = mujoco.MjvCamera()
        points = data.geom_xpos[model.geom_bodyid != 0]
        camera.lookat[:] = (points.min(axis=0) + points.max(axis=0)) / 2
        camera.distance = 2.2
        camera.azimuth = 135
        camera.elevation = -25
        renderer = mujoco.Renderer(model, height=480, width=720)
        try:
            renderer.update_scene(data, camera=camera)
            imageio.imwrite(output / f"{path.stem}.png", renderer.render())
        finally:
            renderer.close()
        print(path.stem, flush=True)


if __name__ == "__main__":
    main()
