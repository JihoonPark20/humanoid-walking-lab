"""Animate a validated centroidal plan as a free-body diagram, not a robot rollout."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from centroidal_io import load_centroidal
from centroidal_model import validate_centroidal

from tools.walking_visuals import FPS, Overlay, WalkingVisualizer

SCENE = """<mujoco><visual><global offwidth="640" offheight="560"/>
<headlight ambient="0.5 0.5 0.5" diffuse="0.7 0.7 0.7"/></visual>
<worldbody><light pos="0 -1 3"/><geom name="floor" type="plane" size="3 3 .01" rgba=".13 .20 .28 1"/></worldbody></mujoco>"""


def main() -> None:
    """Render saved CoM, momentum and force samples with their physical units."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--plan",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "media/offline/plan.npz",
    )
    args = parser.parse_args()
    plan = load_centroidal(args.plan)
    validation = validate_centroidal(plan)
    if not validation["valid"]:
        raise SystemExit("Cannot render an invalid centroidal plan")
    model = mujoco.MjModel.from_xml_string(SCENE)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    renderer = WalkingVisualizer(model, 8)
    output = args.plan.parent
    movie = output / "centroidal.mp4"
    writer = imageio.get_writer(str(movie), fps=FPS, codec="libx264")
    try:
        for t in np.arange(0, plan.contacts.times[-1] + 1e-9, 1 / FPS):
            state, wrench = plan.sample(float(t))
            k = min(
                np.searchsorted(plan.contacts.times, t, side="right") - 1,
                len(plan.wrenches) - 1,
            )
            feet = plan.contacts.positions[k]
            active = plan.contacts.active[k]
            patches = []
            for foot, loaded in zip(feet, active):
                if loaded:
                    patches.extend(
                        foot[:2]
                        + np.array(
                            [[-0.05, -0.02], [0.05, -0.02], [0.05, 0.02], [-0.05, 0.02]]
                        )
                    )
            reference = np.array(
                [
                    np.interp(t, plan.contacts.times, plan.reference[:, j])
                    for j in range(3)
                ]
            )
            writer.append_data(
                renderer.render(
                    data,
                    Overlay(
                        time=float(t),
                        com=state[:3],
                        model_only=True,
                        path=plan.states[:, :3],
                        nominal_path=plan.reference[:, :3],
                        target=reference,
                        planned_feet=feet,
                        planned_support=np.asarray(patches),
                        forces=[
                            (feet[i], wrench[i, :3]) for i in range(2) if active[i]
                        ],
                        phase="Optimized reduced model / no robot",
                        values=[
                            f"Fz left / right: {wrench[0, 2]:.0f} / {wrench[1, 2]:.0f} N",
                            f"Height change: {(state[2] - plan.states[0, 2]) * 1000:+.1f} mm",
                            f"Angular momentum: {np.linalg.norm(state[6:]):.4f} kg m2/s",
                        ],
                    ),
                )
            )
    finally:
        writer.close()
        renderer.close()
    result = {
        "passed": bool(validation["valid"]),
        "kind": "reduced-model plan animation",
        "validation": validation,
        "plan_sha256": hashlib.sha256(args.plan.read_bytes()).hexdigest(),
        "video_sha256": hashlib.sha256(movie.read_bytes()).hexdigest(),
        "duration_s": float(plan.contacts.times[-1]),
        "force_arrow_m_per_n": 0.0015,
        "frames": renderer.frames,
        "fps": FPS,
    }
    (output / "animation_summary.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
