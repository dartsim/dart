"""Unit coverage for the deterministic agent capture harness."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import agent_capture
from _image_tools import read_image


def _args(**overrides) -> argparse.Namespace:
    base = dict(
        scene="box_on_ground",
        factory=None,
        steps=25,
        width=96,
        height=72,
        layers=[],
        focus="",
        auto_views=0,
        camera_azimuth=0.7853981633974483,
        camera_elevation=0.5235987755982988,
        camera_distance=None,
        camera_target=[0.0, 0.0, 0.0],
        frame_margin=2.2,
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


def test_reproduce_command_records_focus_framing_margin() -> None:
    command = agent_capture._reproduce_command(
        _args(focus="box", frame_margin=3.5, camera_distance=None)
    )
    assert "--focus box" in command
    assert "--frame-margin 3.5" in command
    assert "--scene box_on_ground" in command
    assert "--steps 25" in command


def test_reproduce_command_prefers_explicit_distance() -> None:
    command = agent_capture._reproduce_command(
        _args(focus="box", camera_distance=2.5)
    )
    assert "--camera-distance 2.5" in command
    assert "--frame-margin" not in command


def test_reproduce_command_records_auto_views() -> None:
    command = agent_capture._reproduce_command(_args(auto_views=3))
    assert "--auto-views 3" in command
    assert "--camera-azimuth" not in command


def test_failed_view_report_rejects_capture() -> None:
    views = [
        {
            "name": "main",
            "report": {"pass": False, "issues": ["off-frame", "too-far"]},
        }
    ]
    with pytest.raises(ValueError, match="main: off-frame, too-far"):
        agent_capture._require_acceptable_views(views)


def test_acceptable_view_reports_pass_capture_gate() -> None:
    agent_capture._require_acceptable_views(
        [{"name": "main", "report": {"pass": True, "issues": []}}]
    )


def test_motion_frame_assessment_rejects_updated_bad_view() -> None:
    class Report:
        def to_json(self):
            return {"pass": False, "issues": ["off-frame"]}

    class Gui:
        @staticmethod
        def assess_view(world, camera, size, focus=None):
            assert size == (96, 72)
            assert focus == "box"
            return Report()

    with pytest.raises(ValueError, match="motion2: off-frame"):
        agent_capture._assess_capture_view(
            Gui(), object(), object(), _args(focus="box"), "motion2"
        )


def test_initial_still_defers_unseeded_motion_trajectory() -> None:
    assert agent_capture._initial_still_layers(
        _args(layers=["labels", "trajectories"], steps=0, motion_frames=3)
    ) == ["labels"]
    assert agent_capture._initial_still_layers(
        _args(layers=["trajectories"], steps=2, motion_frames=3)
    ) == ["trajectories"]
    assert agent_capture._initial_still_layers(
        _args(layers=["labels", "trajectories"], steps=0, turntable=3)
    ) == ["labels"]


def _display_available() -> bool:
    try:
        import dartpy

        return hasattr(dartpy, "gui") and hasattr(dartpy.gui, "OffscreenRenderer")
    except ImportError:
        return False


def _require_usable_display() -> None:
    import ctypes.util
    import os

    if not sys.platform.startswith("linux"):
        return
    if not os.environ.get("DISPLAY"):
        if os.environ.get("DART_REQUIRE_VISUAL_E2E") == "1":
            pytest.fail("visual e2e is required but DISPLAY is unavailable")
        pytest.skip("Filament headless rendering needs DISPLAY or Xvfb")
    if not ctypes.util.find_library("X11"):
        pytest.fail("DISPLAY is configured but the X11 runtime is unavailable")


def _changed_pixel_count(first, second, tolerance: int = 8) -> int:
    assert first.width == second.width
    assert first.height == second.height
    return sum(
        any(
            abs(
                first.pixels[pixel * 3 + channel]
                - second.pixels[pixel * 3 + channel]
            )
            > tolerance
            for channel in range(3)
        )
        for pixel in range(first.pixel_count)
    )


@pytest.mark.skipif(
    not _display_available(),
    reason="agent_capture smoke needs a dartpy GUI build",
)
def test_run_capture_smoke_writes_stills_and_sidecar(tmp_path: Path) -> None:
    _require_usable_display()

    args = _args(
        out=tmp_path,
        prefix="smoke",
        auto_views=1,
        focus="box",
        layers=["contacts", "labels"],
        steps=150,
    )
    sidecar = agent_capture.run_capture(args)
    assert sidecar["schema_version"] == "dart.agent_capture/v1"
    assert sidecar["scene"] == "box_on_ground"
    stills = [a for a in sidecar["artifacts"] if a["kind"] == "still"]
    assert len(stills) == 1
    still = stills[0]
    assert (tmp_path / still["path"]).exists()
    assert still["view_report"]["schema_version"] == "dart.view_report/v1"
    assert "pixi run agent-capture" in sidecar["reproduce"]
    saved = json.loads((tmp_path / "smoke_capture.json").read_text("utf-8"))
    assert saved == sidecar


@pytest.mark.skipif(
    not _display_available(),
    reason="agent_capture visual A/B needs a dartpy GUI build",
)
def test_run_capture_debug_layers_change_pixels_end_to_end(tmp_path: Path) -> None:
    _require_usable_display()
    common = {
        "steps": 250,
        "width": 320,
        "height": 240,
        "focus": "box",
        "auto_views": 1,
    }
    plain = agent_capture.run_capture(
        _args(out=tmp_path / "plain", prefix="plain", layers=[], **common)
    )
    plain_artifact = plain["artifacts"][0]
    plain_image = read_image(tmp_path / "plain" / plain_artifact["path"])

    debug_layers = ["contacts", "collision_bounds", "labels"]
    combined = agent_capture.run_capture(
        _args(
            out=tmp_path / "combined",
            prefix="combined",
            layers=debug_layers,
            **common,
        )
    )
    combined_artifact = combined["artifacts"][0]
    assert combined["layers"] == debug_layers
    assert plain_artifact["camera"] == combined_artifact["camera"]
    combined_image = read_image(
        tmp_path / "combined" / combined_artifact["path"]
    )
    assert _changed_pixel_count(plain_image, combined_image) >= 128

    # Prove each CI-requested layer contributes pixels; a labels-only delta
    # must not hide a missing contacts or collision-bounds renderer.
    for layer in debug_layers:
        debug = agent_capture.run_capture(
            _args(
                out=tmp_path / layer,
                prefix=layer,
                layers=[layer],
                **common,
            )
        )
        debug_artifact = debug["artifacts"][0]
        assert debug["layers"] == [layer]
        assert plain_artifact["camera"] == debug_artifact["camera"]
        debug_image = read_image(tmp_path / layer / debug_artifact["path"])
        assert _changed_pixel_count(plain_image, debug_image) >= 32, layer
