from __future__ import annotations

import argparse
import importlib.util
import json
import pathlib

import pytest

_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SPEC = importlib.util.spec_from_file_location(
    "capture_py_demo", _ROOT / "scripts" / "capture_py_demo.py"
)
assert _SPEC is not None
assert _SPEC.loader is not None
capture_py_demo = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(capture_py_demo)


def _write_ppm(
    path: pathlib.Path, pixels: bytes, width: int = 2, height: int = 1
) -> None:
    path.write_bytes(f"P6\n{width} {height}\n255\n".encode() + pixels)


def _workspace_pixels(width: int = 320, height: int = 240) -> bytes:
    pixels = bytearray([150] * width * height * 3)

    def fill(x0: int, x1: int, y0: int, y1: int, value: int) -> None:
        for y in range(y0, y1):
            for x in range(x0, x1):
                offset = (y * width + x) * 3
                pixels[offset : offset + 3] = bytes([value, value, value])

    fill(0, width, 0, int(height * 0.18), 25)
    fill(0, int(width * 0.24), int(height * 0.22), int(height * 0.90), 45)
    fill(int(width * 0.73), width, int(height * 0.22), int(height * 0.90), 45)
    fill(0, width, int(height * 0.89), height, 25)
    fill(
        int(width * 0.30),
        int(width * 0.68),
        int(height * 0.30),
        int(height * 0.78),
        170,
    )
    return bytes(pixels)


def test_ppm_to_png_conversion_and_blank_guard(tmp_path: pathlib.Path) -> None:
    ppm = tmp_path / "capture.ppm"
    png = tmp_path / "capture.png"
    _write_ppm(ppm, bytes([0, 0, 0, 255, 0, 0]))

    assert capture_py_demo.ppm_has_nonzero_pixels(ppm)
    capture_py_demo.convert_ppm_to_png(ppm, png)
    assert png.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")

    blank = tmp_path / "blank.ppm"
    _write_ppm(blank, bytes(6))
    assert not capture_py_demo.ppm_has_nonzero_pixels(blank)


def test_ppm_image_evidence_reports_machine_checkable_stats(
    tmp_path: pathlib.Path,
) -> None:
    ppm = tmp_path / "evidence.ppm"
    _write_ppm(
        ppm,
        bytes(
            [
                0,
                0,
                0,
                255,
                255,
                255,
                255,
                0,
                0,
                0,
                255,
                0,
            ]
        ),
        width=2,
        height=2,
    )

    evidence = capture_py_demo.ppm_image_evidence(ppm)

    assert evidence["width"] == 2
    assert evidence["height"] == 2
    assert evidence["pixel_count"] == 4
    assert evidence["nonzero_pixels"] == 3
    assert evidence["nonzero_channels"] == 5
    assert evidence["unique_rgb_count"] == 4
    assert evidence["rgb_channel_variance"] > 0.0
    assert evidence["luminance_variance"] > 0.0
    assert evidence["docked_workspace"] is False


def test_show_ui_detector_rejects_scene_only_frames(
    tmp_path: pathlib.Path,
) -> None:
    no_ui = tmp_path / "no_ui.ppm"
    workspace = tmp_path / "workspace.ppm"
    width, height = 320, 240
    _write_ppm(no_ui, bytes([150] * width * height * 3), width, height)
    _write_ppm(workspace, _workspace_pixels(width, height), width, height)

    assert not capture_py_demo.ppm_has_docked_workspace_regions(no_ui)
    assert capture_py_demo.ppm_has_docked_workspace_regions(workspace)


def test_show_ui_frame_sequence_drops_warmup_frames(
    tmp_path: pathlib.Path,
) -> None:
    frames = tmp_path / "frames"
    frames.mkdir()
    width, height = 320, 240
    _write_ppm(
        frames / "frame_000001.ppm",
        bytes([150] * width * height * 3),
        width,
        height,
    )
    _write_ppm(
        frames / "frame_000002.ppm", _workspace_pixels(width, height), width, height
    )
    _write_ppm(
        frames / "frame_000003.ppm", _workspace_pixels(width, height), width, height
    )

    ready_frames, dropped = capture_py_demo._prepare_frame_sequence(frames, True)

    assert dropped == 1
    assert [frame.name for frame in ready_frames] == [
        "frame_000001.ppm",
        "frame_000002.ppm",
    ]
    assert all(
        capture_py_demo.ppm_has_docked_workspace_regions(frame)
        for frame in ready_frames
    )


def test_visual_capture_manifest_records_image_evidence(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "capture"
    width, height = 320, 240

    def fake_run_demo(demo_args: list[str]) -> int:
        screenshot = pathlib.Path(
            demo_args[demo_args.index("--screenshot") + 1]
        )
        frames = pathlib.Path(demo_args[demo_args.index("--out") + 1])
        scene_metrics = pathlib.Path(
            demo_args[demo_args.index("--capture-metrics-event-log") + 1]
        )
        frames.mkdir(parents=True)
        _write_ppm(
            screenshot,
            _workspace_pixels(width, height),
            width,
            height,
        )
        _write_ppm(
            frames / "frame_000001.ppm",
            bytes([150] * width * height * 3),
            width,
            height,
        )
        _write_ppm(
            frames / "frame_000002.ppm",
            _workspace_pixels(width, height),
            width,
            height,
        )
        _write_ppm(
            frames / "frame_000003.ppm",
            _workspace_pixels(width, height),
            width,
            height,
        )
        scene_metrics.write_text(
            "\n".join(
                [
                    json.dumps(
                        {
                            "event": "scene_capture_metrics",
                            "frame": 1,
                            "metrics": {"step_ms": 1.2, "status": "settling"},
                            "scene": "rigid_body",
                            "source": "py-demo-scene",
                        },
                        sort_keys=True,
                    ),
                    json.dumps(
                        {
                            "event": "scene_capture_metrics",
                            "frame": 3,
                            "metrics": {"step_ms": 2.4, "status": "standing"},
                            "scene": "rigid_body",
                            "source": "py-demo-scene",
                        },
                        sort_keys=True,
                    ),
                ]
            )
            + "\n"
        )
        return 0

    monkeypatch.setattr(capture_py_demo, "_run_demo", fake_run_demo)

    rc = capture_py_demo.main(
        [
            "--scene",
            "rigid_body",
            "--frames",
            "3",
            "--width",
            str(width),
            "--height",
            str(height),
            "--show-ui",
            "--output-dir",
            str(output),
        ]
    )

    assert rc == 0
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["capture"] == {
        "converted_frames": 2,
        "height": height,
        "requested_frames": 3,
        "width": width,
    }
    assert manifest["ui_ready"] == {
        "dropped_warmup_frames": 1,
        "required": True,
    }
    assert pathlib.Path(manifest["artifacts"]["screenshot"]).is_file()
    assert pathlib.Path(manifest["artifacts"]["scene_metrics_events"]).is_file()
    assert len(list((output / "png_frames").glob("frame_*.png"))) == 2
    assert manifest["scene_metrics"] == {
        "event_count": 2,
        "latest": {
            "event": "scene_capture_metrics",
            "frame": 3,
            "metrics": {"status": "standing", "step_ms": 2.4},
            "scene": "rigid_body",
            "source": "py-demo-scene",
        },
    }
    evidence = manifest["visual_evidence"]
    for key in ("screenshot", "first_frame"):
        stats = evidence[key]
        assert stats["width"] == width
        assert stats["height"] == height
        assert stats["pixel_count"] == width * height
        assert stats["nonzero_pixels"] == width * height
        assert stats["unique_rgb_count"] > 1
        assert stats["rgb_channel_variance"] > 0.0
        assert stats["luminance_variance"] > 0.0
        assert stats["docked_workspace"] is True


def test_linux_render_env_defaults_to_software_gl(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(capture_py_demo.sys, "platform", "linux")
    monkeypatch.delenv("LIBGL_ALWAYS_SOFTWARE", raising=False)
    monkeypatch.delenv("MESA_LOADER_DRIVER_OVERRIDE", raising=False)

    capture_py_demo._apply_stable_linux_render_env()

    assert capture_py_demo.os.environ["LIBGL_ALWAYS_SOFTWARE"] == "1"
    assert capture_py_demo.os.environ["MESA_LOADER_DRIVER_OVERRIDE"] == "llvmpipe"


def test_linux_render_env_preserves_overrides(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(capture_py_demo.sys, "platform", "linux")
    monkeypatch.setenv("LIBGL_ALWAYS_SOFTWARE", "0")
    monkeypatch.setenv("MESA_LOADER_DRIVER_OVERRIDE", "custom")

    capture_py_demo._apply_stable_linux_render_env()

    assert capture_py_demo.os.environ["LIBGL_ALWAYS_SOFTWARE"] == "0"
    assert capture_py_demo.os.environ["MESA_LOADER_DRIVER_OVERRIDE"] == "custom"


def test_visual_capture_default_scene_matches_py_demos_front_door() -> None:
    from examples.demos.runner import DEFAULT_INITIAL_SCENE_ID

    assert capture_py_demo.parse_args([]).scene == DEFAULT_INITIAL_SCENE_ID
    assert DEFAULT_INITIAL_SCENE_ID == "rigid_body"


def test_visual_capture_rejects_noop_backend(tmp_path: pathlib.Path) -> None:
    args = argparse.Namespace(
        backend="noop",
        allow_noop=False,
        scene="articulated",
        frames=1,
        width=2,
        height=1,
        show_ui=False,
    )

    with pytest.raises(ValueError, match="noop"):
        capture_py_demo.build_demo_args(
            args, tmp_path / "capture.ppm", tmp_path / "frames"
        )


def test_visual_capture_forwards_scripted_demo_switch(
    tmp_path: pathlib.Path,
) -> None:
    args = argparse.Namespace(
        backend="opengl",
        allow_noop=False,
        scene="rigid_ipc_slide",
        frames=6,
        width=640,
        height=360,
        show_ui=True,
        switch_scene="rigid_ipc_incline",
        switch_frame=2,
        event_log=tmp_path / "events.jsonl",
    )

    demo_args = capture_py_demo.build_demo_args(
        args, tmp_path / "capture.ppm", tmp_path / "frames"
    )

    assert "--scripted-demo-switch" in demo_args
    assert "2:rigid_ipc_incline" in demo_args
    assert "--scripted-demo-event-log" in demo_args
    assert str(tmp_path / "events.jsonl") in demo_args


def test_visual_capture_forwards_scene_metrics_log(
    tmp_path: pathlib.Path,
) -> None:
    args = argparse.Namespace(
        backend="opengl",
        allow_noop=False,
        scene="rigid_body",
        frames=3,
        width=640,
        height=360,
        show_ui=True,
        capture_metrics_event_log=tmp_path / "scene_metrics.jsonl",
    )

    demo_args = capture_py_demo.build_demo_args(
        args, tmp_path / "capture.ppm", tmp_path / "frames"
    )

    assert "--capture-metrics-event-log" in demo_args
    assert str(tmp_path / "scene_metrics.jsonl") in demo_args


def test_visual_capture_forwards_scripted_force_drag(
    tmp_path: pathlib.Path,
) -> None:
    args = argparse.Namespace(
        backend="opengl",
        allow_noop=False,
        scene="rigid_ipc_slide",
        frames=12,
        width=640,
        height=360,
        show_ui=True,
        force_drag_target="ipc_slide_box_visual",
        force_drag_frame=2,
        force_drag_frames=5,
        force_drag_delta=(0.8, 0.0, 0.2),
        event_log=tmp_path / "events.jsonl",
    )

    demo_args = capture_py_demo.build_demo_args(
        args, tmp_path / "capture.ppm", tmp_path / "frames"
    )

    assert "--scripted-force-drag" in demo_args
    assert "2:ipc_slide_box_visual:0.8,0,0.2:5" in demo_args
    assert "--scripted-demo-event-log" in demo_args
    assert str(tmp_path / "events.jsonl") in demo_args


def test_visual_capture_forwards_scripted_pointer_force_drag(
    tmp_path: pathlib.Path,
) -> None:
    args = argparse.Namespace(
        backend="opengl",
        allow_noop=False,
        scene="rigid_ipc_slide",
        frames=12,
        width=640,
        height=360,
        show_ui=True,
        force_drag_target="",
        force_drag_pixel=(320.0, 180.0),
        force_drag_delta_pixels=(140.0, -50.0),
        force_drag_frame=2,
        force_drag_frames=5,
        force_drag_delta=(0.8, 0.0, 0.2),
        event_log=tmp_path / "events.jsonl",
    )

    demo_args = capture_py_demo.build_demo_args(
        args, tmp_path / "capture.ppm", tmp_path / "frames"
    )

    assert "--scripted-pointer-force-drag" in demo_args
    assert "2:320,180:140,-50:5" in demo_args
    assert "--scripted-demo-event-log" in demo_args
    assert str(tmp_path / "events.jsonl") in demo_args
