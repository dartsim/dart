"""Tests for the DART 6 agent capture harness."""

import argparse
import json
import os
import sys
from pathlib import Path

import pytest

import dartpy as dart

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import agent_capture

requires_display = pytest.mark.skipif(
    not os.environ.get("DISPLAY"),
    reason="off-screen GLX capture needs a DISPLAY (run under xvfb-run on "
    "headless hosts)",
)

try:
    _osg = dart.gui.osg
except AttributeError:  # pragma: no cover - depends on build config
    pytest.skip("dartpy built without gui.osg", allow_module_level=True)


def _args(**overrides) -> argparse.Namespace:
    base = dict(
        scene="box_on_ground",
        steps=25,
        width=96,
        height=72,
        fovy_deg=30.0,
        layers=[],
        focus="",
        auto_views=0,
        camera_azimuth=0.7853981633974483,
        camera_elevation=0.5235987755982988,
        camera_distance=None,
        camera_target=[0.0, 0.0, 0.0],
        frame_margin=2.2,
        warmup_frames=10,
        turntable=0,
        motion_frames=0,
        motion_substeps=8,
        motion_orbit_degrees=0.0,
        video=False,
        fps=24,
        out=Path("unused"),
        prefix="capture",
    )
    base.update(overrides)
    return argparse.Namespace(**base)


def test_reproduce_command_records_focus_framing_margin():
    command = agent_capture._reproduce_command(
        _args(focus="box", frame_margin=3.5, camera_distance=None)
    )
    assert "--focus box" in command
    assert "--frame-margin 3.5" in command
    assert "--scene box_on_ground" in command


def test_reproduce_command_prefers_explicit_distance():
    command = agent_capture._reproduce_command(
        _args(focus="box", camera_distance=2.5)
    )
    assert "--camera-distance 2.5" in command
    assert "--frame-margin" not in command


def test_builtin_scenes_construct_and_step():
    dartpy_module = agent_capture._import_dartpy()
    for name, factory in agent_capture._BUILTIN_SCENES.items():
        world = factory(dartpy_module)
        for _ in range(5):
            world.step()
        assert world.getNumSkeletons() >= 2, name


@requires_display
def test_run_capture_smoke_writes_stills_and_sidecar(tmp_path):
    args = _args(
        out=tmp_path,
        prefix="smoke",
        auto_views=1,
        focus="box",
        layers=["contacts", "labels"],
        steps=150,
    )
    try:
        sidecar = agent_capture.run_capture(args)
    except RuntimeError as error:
        if "off-screen GL context" in str(error):
            pytest.skip(str(error))
        raise
    assert sidecar["schema_version"] == "dart.agent_capture/v1"
    stills = [a for a in sidecar["artifacts"] if a["kind"] == "still"]
    assert len(stills) == 1
    assert (tmp_path / stills[0]["path"]).exists()
    assert stills[0]["view_report"]["schema_version"] == "dart.view_report/v1"
    assert "pixi run agent-capture" in sidecar["reproduce"]
    saved = json.loads((tmp_path / "smoke_capture.json").read_text("utf-8"))
    assert saved == sidecar
