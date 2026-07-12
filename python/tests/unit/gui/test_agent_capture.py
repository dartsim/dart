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
import agent_debug_overlay
from _image_tools import read_image

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
        factory=None,
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


def _changed_pixel_count(first, second, tolerance=8):
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


def test_reproduce_command_records_focus_framing_margin():
    command = agent_capture._reproduce_command(
        _args(focus="box", frame_margin=3.5, camera_distance=None)
    )
    assert "--focus box" in command
    assert "--frame-margin 3.5" in command
    assert "--scene box_on_ground" in command


def test_reproduce_command_records_claim_specific_factory():
    command = agent_capture._reproduce_command(
        _args(factory="claim_scene:make_world")
    )
    assert "--factory claim_scene:make_world" in command
    assert "--scene" not in command


def test_load_factory_resolves_module_callable(tmp_path, monkeypatch):
    (tmp_path / "claim_scene.py").write_text(
        "def make_world():\n    return 'world'\nvalue = 3\n", encoding="utf-8"
    )
    monkeypatch.syspath_prepend(str(tmp_path))

    assert agent_capture._load_factory("claim_scene:make_world")() == "world"
    with pytest.raises(ValueError, match="module:callable"):
        agent_capture._load_factory("claim_scene")
    with pytest.raises(TypeError, match="did not resolve to a callable"):
        agent_capture._load_factory("claim_scene:value")


@pytest.mark.parametrize(
    ("factory", "message"),
    [
        ("missing_factory_module:make_world", "No module named"),
        ("agent_capture:missing_factory", "has no attribute"),
        ("agent_capture:SCHEMA_VERSION", "did not resolve to a callable"),
    ],
)
def test_main_reports_invalid_factory_without_traceback(
    tmp_path, capsys, factory, message
):
    result = agent_capture.main(
        ["--factory", factory, "--out", str(tmp_path / "capture")]
    )

    captured = capsys.readouterr()
    assert result == 2
    assert captured.out == ""
    assert captured.err.startswith("error: ")
    assert message in captured.err
    assert "Traceback" not in captured.err


def test_reproduce_command_prefers_explicit_distance():
    command = agent_capture._reproduce_command(
        _args(focus="box", camera_distance=2.5)
    )
    assert "--camera-distance 2.5" in command
    assert "--frame-margin" not in command


def test_reproduce_command_records_custom_warmup():
    # Warmup frames affect OSG initialization before the screenshot; a
    # non-default value must survive into the recorded reproduce command.
    command = agent_capture._reproduce_command(_args(warmup_frames=25))
    assert "--warmup-frames 25" in command
    default = agent_capture._reproduce_command(_args())
    assert "--warmup-frames" not in default


def test_failed_view_report_rejects_capture():
    views = [
        {
            "name": "main",
            "report": {"pass": False, "issues": ["off-frame", "too-far"]},
        }
    ]
    with pytest.raises(ValueError, match="main: off-frame, too-far"):
        agent_capture._require_acceptable_views(views)


def test_acceptable_view_reports_pass_capture_gate():
    agent_capture._require_acceptable_views(
        [{"name": "main", "report": {"pass": True, "issues": []}}]
    )


def test_motion_frame_assessment_gates_moved_subject():
    # Motion sequences re-run the view-quality gate per frame against the
    # advanced world: a camera that frames the subject passes, one aimed away
    # from it aborts the capture with the frame name in the error.
    world = agent_capture._BUILTIN_SCENES["box_on_ground"](
        agent_capture._import_dartpy()
    )
    args = _args(focus="box", width=320, height=240)

    import agent_view_quality as avq

    good = avq.frame_body(world, "box", azimuth=0.8, elevation=0.45)
    report = agent_capture._assess_capture_view(world, good, args, "motion0")
    assert report["pass"] is True

    away = avq.orbit_camera([50.0, 50.0, 0.2], 2.0, 0.8, 0.45)
    with pytest.raises(ValueError, match="motion1"):
        agent_capture._assess_capture_view(world, away, args, "motion1")


def test_prestep_layers_defer_unseeded_trajectory():
    assert agent_capture._prestep_layers(
        _args(layers=["labels", "trajectories"], steps=0, turntable=3)
    ) == ["labels"]
    assert agent_capture._prestep_layers(
        _args(layers=["trajectories"], steps=2, turntable=3)
    ) == ["trajectories"]


def test_builtin_scenes_construct_and_step():
    dartpy_module = agent_capture._import_dartpy()
    for name, factory in agent_capture._BUILTIN_SCENES.items():
        world = factory(dartpy_module)
        for _ in range(5):
            world.step()
        assert world.getNumSkeletons() >= 2, name


@requires_display
def test_run_capture_smoke_writes_stills_and_sidecar(tmp_path, monkeypatch):
    (tmp_path / "claim_capture_scene.py").write_text(
        "import agent_capture\n"
        "def make_world():\n"
        "    return agent_capture._BUILTIN_SCENES['box_on_ground'](\n"
        "        agent_capture._import_dartpy())\n",
        encoding="utf-8",
    )
    monkeypatch.syspath_prepend(str(tmp_path))
    args = _args(
        out=tmp_path,
        prefix="smoke",
        factory="claim_capture_scene:make_world",
        auto_views=1,
        focus="box",
        layers=["contacts", "labels"],
        steps=250,
    )
    try:
        sidecar = agent_capture.run_capture(args)
    except RuntimeError as error:
        if "off-screen GL context" in str(error):
            pytest.fail(str(error))
        raise
    assert sidecar["schema_version"] == "dart.agent_capture/v1"
    stills = [a for a in sidecar["artifacts"] if a["kind"] == "still"]
    assert len(stills) == 1
    assert (tmp_path / stills[0]["path"]).exists()
    assert stills[0]["view_report"]["schema_version"] == "dart.view_report/v1"
    assert "pixi run agent-capture" in sidecar["reproduce"]
    assert "--factory claim_capture_scene:make_world" in sidecar["reproduce"]
    saved = json.loads((tmp_path / "smoke_capture.json").read_text("utf-8"))
    assert saved == sidecar


@requires_display
def test_run_capture_debug_layers_change_pixels_end_to_end(tmp_path):
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
    contact_pixels = sum(
        tuple(combined_image.pixels[pixel * 3 : pixel * 3 + 3])
        == agent_debug_overlay.CONTACT_POINT_RGB
        for pixel in range(combined_image.pixel_count)
    )
    assert contact_pixels >= 4

    # Prove every CI-requested OSG layer contributes pixels independently;
    # contacts must not mask a broken collision-bounds or labels path.
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
