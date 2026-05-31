from __future__ import annotations

import argparse
import importlib.util
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


def test_visual_capture_rejects_noop_backend(tmp_path: pathlib.Path) -> None:
    args = argparse.Namespace(
        backend="noop",
        allow_noop=False,
        scene="hello_world",
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
        scene="sx_rigid_ipc_slide",
        frames=6,
        width=640,
        height=360,
        show_ui=True,
        switch_scene="sx_rigid_ipc_incline",
        switch_frame=2,
        event_log=tmp_path / "events.jsonl",
    )

    demo_args = capture_py_demo.build_demo_args(
        args, tmp_path / "capture.ppm", tmp_path / "frames"
    )

    assert "--scripted-demo-switch" in demo_args
    assert "2:sx_rigid_ipc_incline" in demo_args
    assert "--scripted-demo-event-log" in demo_args
    assert str(tmp_path / "events.jsonl") in demo_args


def test_visual_capture_forwards_scripted_force_drag(
    tmp_path: pathlib.Path,
) -> None:
    args = argparse.Namespace(
        backend="opengl",
        allow_noop=False,
        scene="sx_rigid_ipc_slide",
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
        scene="sx_rigid_ipc_slide",
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
