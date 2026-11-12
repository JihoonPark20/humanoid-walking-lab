"""Read-only teaching overlays for balance, planning and humanoid control videos.

All markers are MjvScene decorations. Ghosts use a separate MjData and never
replace the simulated robot state. Cyan is measured, violet is planned, amber
is contact ZMP, green is loaded support, and red is an applied disturbance.
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

CYAN = (0.12, 0.88, 1.0, 1.0)
PURPLE = (0.77, 0.51, 1.0, 0.9)
AMBER = (1.0, 0.67, 0.16, 1.0)
GREEN = (0.2, 0.95, 0.62, 0.9)
WHITE = (0.94, 0.97, 1.0, 0.9)
GRAY = (0.56, 0.63, 0.73, 0.65)
RED = (1.0, 0.25, 0.28, 1.0)
WIDTH, HEIGHT, FPS = 960, 640, 30

THEMES = {
    5: ("BALANCE", "Move the CoM; keep both feet planted", "Target CoM", 0.34),
    6: (
        "DCM / ZMP WALKING",
        "Switch support while regulating balance",
        "Target DCM",
        0.55,
    ),
    7: (
        "OFFLINE TRAJECTORY OPTIMIZATION",
        "Choose footsteps and CoM before execution",
        "Planned CoM",
        0.85,
    ),
    8: (
        "CENTROIDAL DYNAMICS",
        "Contact forces move a floating mass",
        "Reference CoM",
        0.65,
    ),
    9: (
        "RECEDING-HORIZON MPC / RTI",
        "Replan from the measured state after a push",
        "Nominal CoM",
        0.65,
    ),
    10: (
        "WHOLE-BODY OPTIMAL CONTROL",
        "Track a joint-space plan with feedback",
        "Nominal CoM",
        0.55,
    ),
}


@dataclass
class Overlay:
    """World-coordinate quantities supplied by the actual experiment (SI units)."""

    time: float
    com: np.ndarray
    phase: str = ""
    values: list[str] = field(default_factory=list)
    target: np.ndarray | None = None
    dcm: np.ndarray | None = None
    path: np.ndarray | None = None
    nominal_path: np.ndarray | None = None
    seed_feet: np.ndarray | None = None
    planned_feet: np.ndarray | None = None
    prediction: np.ndarray | None = None
    prediction_times: np.ndarray | None = None
    ghosts: list[tuple[float, np.ndarray]] = field(default_factory=list)
    forces: list[tuple[np.ndarray, np.ndarray]] = field(default_factory=list)
    push: np.ndarray | None = None
    model_only: bool = False
    planned_support: np.ndarray | None = None


def convex_hull(points: np.ndarray) -> np.ndarray:
    """Return ordered 2D hull vertices, including the degenerate point/line cases."""
    pts = sorted(set(map(tuple, np.asarray(points).reshape(-1, 2))))
    if len(pts) <= 2:
        return np.asarray(pts, dtype=float).reshape(-1, 2)

    def cross(o: tuple, a: tuple, b: tuple) -> float:
        """Signed area of the three points."""
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.asarray(lower[:-1] + upper[:-1])


def ground_contacts(
    model: mujoco.MjModel, data: mujoco.MjData
) -> tuple[np.ndarray, np.ndarray | None, tuple[bool, bool]]:
    """Read loaded ankle/ground contacts and the flat-ground normal-force ZMP.

    This matches the lab's contact-based ZMP gate; it is not a desired ZMP or
    the QP's predicted wrench. No marker is returned during flight.
    """
    points, weights = [], []
    active = [False, False]
    wrench = np.zeros(6)
    for i, c in enumerate(data.contact):
        bodies = [
            model.body(int(model.geom_bodyid[g])).name for g in (c.geom1, c.geom2)
        ]
        foot = next((name for name in bodies if "ankle" in name), None)
        if foot is None:
            continue
        # Only ground-plane contacts, not an ankle touching another robot body.
        if not any(
            model.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE
            for g in (c.geom1, c.geom2)
        ):
            continue
        mujoco.mj_contactForce(model, data, i, wrench)
        weight = abs(float(wrench[0] * c.frame[2]))
        if weight > 1e-6:
            points.append(c.pos[:2].copy())
            weights.append(weight)
            active[0 if foot.startswith("left") else 1] = True
    if not points:
        return np.empty((0, 2)), None, tuple(active)
    return (
        convex_hull(np.asarray(points)),
        np.average(points, axis=0, weights=weights),
        tuple(active),
    )


def ground_point(point: np.ndarray, height: float = 0.012) -> np.ndarray:
    """Project to the horizontal display plane without mutating the source point."""
    return np.array([point[0], point[1], height], dtype=float)


def rgb(color: tuple) -> tuple[int, int, int]:
    """Convert a renderer RGBA color to an opaque diagram RGB color."""
    return tuple(round(255 * c) for c in color[:3])


class WalkingVisualizer:
    """Render a distinct lesson view with a ground-plane diagram and clear legend."""

    def __init__(self, model: mujoco.MjModel, lab: int) -> None:
        self.model = model
        self.lab = lab
        self.frames = 0
        self.title, self.subtitle, self.target_label, self.span = THEMES[lab]
        self.renderer = mujoco.Renderer(model, height=560, width=640)
        self.camera = mujoco.MjvCamera()
        self.camera.distance = 2.05 if lab != 8 else 1.65
        self.camera.elevation = -20 if lab != 8 else -25
        self.camera.azimuth = 180 if lab == 5 else 135
        self.ghost_data = mujoco.MjData(model)
        self.ghost_scene = mujoco.MjvScene(model, maxgeom=1500)
        self.option = mujoco.MjvOption()
        font_path = Path(matplotlib.get_data_path()) / "fonts/ttf/DejaVuSans.ttf"
        self.font = ImageFont.truetype(str(font_path), 15)
        self.small = ImageFont.truetype(str(font_path), 13)
        self.large = ImageFont.truetype(str(font_path), 22)
        self.trail: list[np.ndarray] = []

    def _geom(
        self, kind: int, position: np.ndarray, size: list, color: tuple
    ) -> mujoco.MjvGeom:
        """Append a display-only geom, rejecting capacity overflow explicitly."""
        scene = self.renderer.scene
        if scene.ngeom >= scene.maxgeom:
            raise RuntimeError("Walking overlay exceeded scene capacity")
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(
            geom,
            kind,
            np.asarray(size),
            np.asarray(position),
            np.eye(3).flatten(),
            np.asarray(color, dtype=np.float32),
        )
        geom.category = mujoco.mjtCatBit.mjCAT_DECOR
        geom.castshadow = False
        scene.ngeom += 1
        return geom

    def _line(
        self, a: np.ndarray, b: np.ndarray, color: tuple, width: float = 0.003
    ) -> None:
        """Connect two finite points by a capsule decoration."""
        if np.linalg.norm(np.asarray(a) - b) < 1e-8:
            return
        geom = self._geom(
            mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), [width, 0, 0], color
        )
        mujoco.mjv_connector(
            geom, mujoco.mjtGeom.mjGEOM_CAPSULE, width, np.asarray(a), np.asarray(b)
        )

    def _path(
        self, points: np.ndarray | None, color: tuple, ground: bool = False
    ) -> None:
        """Draw at most 100 segments, keeping the first and last sample."""
        if points is None or len(points) < 2:
            return
        points = np.asarray(points)
        indices = np.linspace(0, len(points) - 1, min(len(points), 101), dtype=int)
        xyz = [ground_point(p) if ground or len(p) == 2 else p for p in points[indices]]
        for a, b in itertools.pairwise(xyz):
            self._line(a, b, color, 0.002)

    def _ring(self, point: np.ndarray, color: tuple, radius: float = 0.025) -> None:
        """Show a CoM ground projection as a ring rather than a filled support disk."""
        angles = np.linspace(0, 2 * np.pi, 25)
        points = np.c_[
            point[0] + radius * np.cos(angles),
            point[1] + radius * np.sin(angles),
            np.full_like(angles, 0.014),
        ]
        self._path(points, color)

    def _footprints(self, feet: np.ndarray | None, color: tuple) -> None:
        """Draw the planner's fixed-yaw contact patches (100 x 40 mm)."""
        if feet is None:
            return
        for point in feet:
            xy = (
                np.array(
                    [
                        [-0.05, -0.02],
                        [0.05, -0.02],
                        [0.05, 0.02],
                        [-0.05, 0.02],
                        [-0.05, -0.02],
                    ]
                )
                + point[:2]
            )
            self._path(xy, color, ground=True)

    def _ghost(self, qpos: np.ndarray, alpha: float) -> None:
        """Copy visual robot geoms from an independent nominal-configuration scene."""
        self.ghost_data.qpos[:] = qpos
        mujoco.mj_forward(self.model, self.ghost_data)
        mujoco.mjv_updateScene(
            self.model,
            self.ghost_data,
            self.option,
            None,
            self.camera,
            mujoco.mjtCatBit.mjCAT_DYNAMIC,
            self.ghost_scene,
        )
        for i in range(self.ghost_scene.ngeom):
            source = self.ghost_scene.geoms[i]
            if source.objtype != mujoco.mjtObj.mjOBJ_GEOM or source.rgba[3] < 0.01:
                continue
            geom = self._geom(
                source.type, source.pos, source.size, (*PURPLE[:3], alpha)
            )
            geom.mat[:] = source.mat
            geom.dataid = source.dataid
            geom.modelrbound = source.modelrbound
            geom.transparent = True
            geom.emission = 0.25

    def render(self, data: mujoco.MjData, overlay: Overlay) -> np.ndarray:
        """Render measured quantities and actual planner samples without stepping physics."""
        com = np.asarray(overlay.com)
        self.camera.lookat[:] = [com[0], com[1], 0.64 if self.lab != 8 else 0.38]
        self.renderer.update_scene(data, camera=self.camera)
        hull, zmp, support = ground_contacts(self.model, data)
        if overlay.model_only:
            hull = (
                np.empty((0, 2))
                if overlay.planned_support is None
                else convex_hull(overlay.planned_support)
            )
        self.trail.append(com.copy())
        self._ring(com, CYAN)
        self._geom(
            mujoco.mjtGeom.mjGEOM_SPHERE,
            com,
            [0.055 if overlay.model_only else 0.027, 0, 0],
            CYAN,
        )
        for a in np.linspace(0, 1, 11)[::2]:
            self._line(
                ground_point(com, 0.02 + a * (com[2] - 0.02)),
                ground_point(com, 0.02 + min(a + 0.05, 1) * (com[2] - 0.02)),
                CYAN,
                0.0015,
            )
        if len(hull) >= 2:
            self._path(np.vstack([hull, hull[0]]), GREEN, ground=True)
        show_zmp = self.lab in (5, 6)
        if zmp is not None and show_zmp:
            self._geom(
                mujoco.mjtGeom.mjGEOM_SPHERE,
                ground_point(zmp, 0.02),
                [0.014, 0, 0],
                AMBER,
            )
        if overlay.dcm is not None:
            self._ring(overlay.dcm, PURPLE, 0.018)
        if overlay.target is not None:
            self._ring(overlay.target, WHITE, 0.014)
        self._path(overlay.nominal_path, GRAY)
        self._path(overlay.path, PURPLE)
        if self.lab in (7, 9):
            self._path(np.asarray(self.trail[-180:]), CYAN)
        self._footprints(overlay.seed_feet, GRAY)
        self._footprints(overlay.planned_feet, PURPLE)
        self._path(overlay.prediction, PURPLE)
        if overlay.prediction is not None:
            for index in np.linspace(
                0,
                len(overlay.prediction) - 1,
                min(4, len(overlay.prediction)),
                dtype=int,
            ):
                self._geom(
                    mujoco.mjtGeom.mjGEOM_SPHERE,
                    overlay.prediction[index],
                    [0.035, 0, 0],
                    (*PURPLE[:3], 0.28),
                )
            self._path(overlay.prediction, (*PURPLE[:3], 0.55), ground=True)
        for offset, qpos in overlay.ghosts:
            self._ghost(qpos, max(0.035, 0.14 - 0.18 * offset))
        for position, force in overlay.forces:
            if np.linalg.norm(force) < 1e-6:
                continue
            arrow = self._geom(
                mujoco.mjtGeom.mjGEOM_ARROW, position, [0.008, 0, 0], GREEN
            )
            mujoco.mjv_connector(
                arrow,
                mujoco.mjtGeom.mjGEOM_ARROW,
                0.008,
                position,
                np.asarray(position) + 0.0015 * np.asarray(force),
            )
        if overlay.push is not None and np.linalg.norm(overlay.push) > 0:
            arrow = self._geom(mujoco.mjtGeom.mjGEOM_ARROW, com, [0.014, 0, 0], RED)
            mujoco.mjv_connector(
                arrow,
                mujoco.mjtGeom.mjGEOM_ARROW,
                0.014,
                com - 0.06 * np.asarray(overlay.push),
                com,
            )
        image = Image.new("RGB", (WIDTH, HEIGHT), (14, 22, 35))
        image.paste(Image.fromarray(self.renderer.render()), (0, 72))
        draw = ImageDraw.Draw(image)
        draw.text(
            (18, 9), f"{self.lab:02d} / {self.title}", font=self.large, fill=rgb(WHITE)
        )
        draw.text((18, 41), self.subtitle, font=self.font, fill=(176, 189, 207))
        draw.text(
            (790, 14), f"t = {overlay.time:05.2f} s", font=self.large, fill=rgb(CYAN)
        )
        self._panel(draw, overlay, hull, zmp if show_zmp else None, support)
        self.frames += 1
        return np.asarray(image)

    def _panel(
        self,
        draw: ImageDraw.ImageDraw,
        o: Overlay,
        hull: np.ndarray,
        zmp: np.ndarray | None,
        support: tuple,
    ) -> None:
        """Add a metrically scaled top view and a short lesson-specific legend."""
        draw.rectangle((640, 72, 959, 639), fill=(20, 31, 47))
        draw.text(
            (658, 84), "GROUND VIEW   /   +x forward", font=self.font, fill=rgb(WHITE)
        )
        center = np.asarray(o.com[:2]).copy()
        if self.lab == 7 and o.planned_feet is not None:
            center = np.mean(o.planned_feet[:, :2], axis=0)
        scale = 240 / self.span

        def project(p: np.ndarray) -> tuple[float, float]:
            """Keep metres and axes consistent: world +x up and +y left."""
            return 800 - (p[1] - center[1]) * scale, 232 - (p[0] - center[0]) * scale

        def polyline(points: np.ndarray | None, color: tuple, width: int = 2) -> None:
            """Draw a world-XY line inside the diagram window."""
            if points is not None and len(points) >= 2:
                projected = [project(p) for p in points]
                # Avoid drawing off-map traces over the text panel.
                for a, b in itertools.pairwise(projected):
                    if all(650 <= v[0] <= 950 and 112 <= v[1] <= 352 for v in (a, b)):
                        draw.line((a, b), fill=rgb(color), width=width)

        for y in range(112, 353, 40):
            draw.line((650, y, 950, y), fill=(40, 52, 69))
        for x in range(660, 951, 40):
            draw.line((x, 112, x, 352), fill=(40, 52, 69))
        if len(hull) >= 3:
            draw.polygon(
                [project(p) for p in hull], fill=(22, 77, 65), outline=rgb(GREEN)
            )
        polyline(o.nominal_path, GRAY)
        polyline(o.path, PURPLE)
        polyline(o.prediction, PURPLE, 3)
        if self.lab in (7, 9):
            polyline(np.asarray(self.trail[-180:]), CYAN)
        for feet, color in ((o.seed_feet, GRAY), (o.planned_feet, PURPLE)):
            if feet is not None:
                for i, p in enumerate(feet):
                    x, y = project(p)
                    draw.rectangle(
                        (
                            x - 0.02 * scale,
                            y - 0.05 * scale,
                            x + 0.02 * scale,
                            y + 0.05 * scale,
                        ),
                        outline=rgb(color),
                        width=2,
                    )
                    if color == PURPLE:
                        draw.text(
                            (x + 8, y - 6), str(i), font=self.small, fill=rgb(color)
                        )
        for point, color, radius in (
            (o.com, CYAN, 8),
            (o.target, WHITE, 5),
            (o.dcm, PURPLE, 6),
            (zmp, AMBER, 4),
        ):
            if point is not None:
                x, y = project(point)
                draw.ellipse(
                    (x - radius, y - radius, x + radius, y + radius),
                    outline=rgb(color),
                    width=3,
                )
        if o.prediction is not None and o.prediction_times is not None:
            for i in np.linspace(
                0, len(o.prediction) - 1, min(3, len(o.prediction)), dtype=int
            ):
                x, y = project(o.prediction[i])
                draw.text(
                    (x + 8, y - 9),
                    f"+{o.prediction_times[i]:.1f}s",
                    font=self.small,
                    fill=rgb(PURPLE),
                )
        if o.push is not None and np.linalg.norm(o.push[:2]) > 0:
            start = np.asarray(project(o.com - 0.06 * o.push))
            end = np.asarray(project(o.com))
            direction = (end - start) / np.linalg.norm(end - start)
            normal = np.array([-direction[1], direction[0]])
            draw.line([tuple(start), tuple(end)], fill=rgb(RED), width=3)
            draw.polygon(
                [
                    tuple(end),
                    tuple(end - 10 * direction + 5 * normal),
                    tuple(end - 10 * direction - 5 * normal),
                ],
                fill=rgb(RED),
            )
            midpoint = (start + end) / 2
            draw.text(
                tuple(midpoint + [0, 8]),
                f"{np.linalg.norm(o.push):.0f} N",
                font=self.small,
                fill=rgb(RED),
            )
        draw.line((666, 340, 666 + 0.1 * scale, 340), fill=rgb(WHITE), width=2)
        draw.text((666, 320), "10 cm", font=self.small, fill=rgb(WHITE))
        legend = [
            (
                CYAN,
                "Planned mass / CoM" if o.model_only else "Measured CoM / ground ring",
            )
        ]
        if self.lab in (5, 6):
            legend.append((AMBER, "Measured contact ZMP"))
        if o.dcm is not None:
            legend.append((PURPLE, "Controller DCM"))
        if o.target is not None:
            legend.append((WHITE, self.target_label))
        if self.lab in (7, 9):
            legend.append(
                (
                    PURPLE,
                    "Optimized plan" if self.lab == 7 else "Updated CoM prediction",
                )
            )
        if self.lab in (7, 9):
            legend.append(
                (GRAY, "Seed footsteps" if self.lab == 7 else "Original nominal plan")
            )
        if o.model_only:
            legend.append((PURPLE, "Optimized CoM / contact patches"))
            legend.append((GRAY, "Reference trajectory"))
            legend.append((GREEN, "Planned forces: 1.5 mm / N"))
        elif self.lab == 10:
            legend.append((PURPLE, "Ghosts: planned joint poses"))
            legend.append((GREEN, "Loaded contact support hull"))
        else:
            legend.append((GREEN, "Loaded contact support hull"))
        if self.lab == 9:
            legend.append((RED, "Applied disturbance"))
        for i, (color, label) in enumerate(legend):
            y = 366 + 21 * i
            draw.line((658, y + 8, 676, y + 8), fill=rgb(color), width=3)
            draw.text((684, y), label, font=self.small, fill=rgb(WHITE))
        values_y = 366 + 21 * len(legend) + 10
        phase = o.phase or (
            "L + R support"
            if all(support)
            else "L support"
            if support[0]
            else "R support"
            if support[1]
            else "No loaded contact"
        )
        draw.text(
            (658, values_y),
            phase,
            font=self.font,
            fill=rgb(
                RED if o.push is not None and np.linalg.norm(o.push) > 0 else GREEN
            ),
        )
        for i, text in enumerate(o.values[:4]):
            draw.text(
                (658, values_y + 24 + 21 * i), text, font=self.small, fill=rgb(WHITE)
            )

    def close(self) -> None:
        """Release the offscreen graphics context."""
        self.renderer.close()
