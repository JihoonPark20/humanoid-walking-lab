"""Validate the filmed scenario, including failures hidden by a mean-force metric."""

from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np
import pinocchio as pin
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from lab3_common import (
    SCENE_TABLE_PATH,
    get_mj_ee_site_id,
    load_mujoco_model,
    load_pinocchio_model,
)
from record_lab3_demo import run_demo, summarize


@pytest.fixture(scope="module")
def rollout() -> dict[str, np.ndarray]:
    """Evaluate the documented video configuration without a graphics backend."""
    return run_demo()


def test_probe_frame_matches_mujoco() -> None:
    """The force-control point must agree in both engines after adding the stylus."""
    model, data = load_mujoco_model(SCENE_TABLE_PATH)
    pm, pd, frame = load_pinocchio_model(contact_probe=True)
    site = get_mj_ee_site_id(model)
    for q in (np.zeros(6), np.array([-1.2, -1.5, 1.3, -1.4, -1.5, 0.2])):
        data.qpos[:6] = q
        mujoco.mj_forward(model, data)
        pin.forwardKinematics(pm, pd, q)
        pin.updateFramePlacements(pm, pd)
        assert np.allclose(pd.oMf[frame].translation, data.site_xpos[site], atol=1e-7)
        assert np.allclose(
            pd.oMf[frame].rotation, data.site_xmat[site].reshape(3, 3), atol=1e-7
        )


def test_filmed_trace_passes_physical_checks(rollout: dict) -> None:
    """Check raw force, complete tracing, tool orientation and probe-only contact."""
    report = summarize(rollout)
    assert report["passed"], report


def test_video_observer_does_not_change_physics(rollout: dict) -> None:
    """The recorder receives measured simulator states from the tested rollout."""
    positions = []
    times = []

    def observe(model: mujoco.MjModel, data: mujoco.MjData, sample: dict) -> None:
        """Inspect the live state as a recorder would, without a second controller."""
        assert np.allclose(
            data.site_xpos[get_mj_ee_site_id(model)], sample["ee_pos"], atol=1e-12
        )
        positions.append(data.qpos[:6].copy())
        times.append(sample["time"])

    observed = run_demo(frame_callback=observe)
    assert np.allclose(positions, rollout["q"], atol=1e-12)
    assert np.allclose(times, rollout["time"], atol=1e-12)
    assert np.allclose(observed["force_raw_z"], rollout["force_raw_z"], atol=1e-12)


def test_filtered_force_cannot_hide_contact_loss(rollout: dict) -> None:
    """A smooth filtered 5 N signal cannot make an intermittent contact pass."""
    bad = {key: value.copy() for key, value in rollout.items()}
    bad["force_raw_z"][bad["phase"] == 2] = 0
    bad["force_z"][bad["phase"] == 2] = 5
    assert not summarize(bad)["passed"]


def test_wrist_contact_is_not_a_valid_trace(rollout: dict) -> None:
    """Position and force accuracy do not excuse non-probe table contact."""
    bad = {key: value.copy() for key, value in rollout.items()}
    bad["unexpected_contacts"][-1] = 1
    assert not summarize(bad)["passed"]
