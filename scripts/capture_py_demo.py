#!/usr/bin/env python3
"""Capture Python demo screenshots and optional videos for visual debugging."""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import shutil
import struct
import subprocess
import sys
import tempfile
import time
import zlib


def read_ppm(path: pathlib.Path) -> tuple[int, int, bytes]:
    data = path.read_bytes()
    parts = data.split(b"\n", 3)
    if len(parts) != 4 or parts[0] != b"P6":
        raise ValueError(f"{path} is not a binary PPM")
    width, height = (int(part) for part in parts[1].split())
    if parts[2] != b"255":
        raise ValueError(f"{path} is not an 8-bit PPM")
    expected = width * height * 3
    pixels = parts[3]
    if len(pixels) < expected:
        raise ValueError(f"{path} is truncated")
    return width, height, pixels[:expected]


def ppm_has_nonzero_pixels(path: pathlib.Path) -> bool:
    _, _, pixels = read_ppm(path)
    return any(channel != 0 for channel in pixels)


def _png_chunk(kind: bytes, payload: bytes) -> bytes:
    checksum = zlib.crc32(kind + payload) & 0xFFFFFFFF
    return (
        struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", checksum)
    )


def write_png(path: pathlib.Path, width: int, height: int, rgb: bytes) -> None:
    rows = [b"\x00" + rgb[y * width * 3 : (y + 1) * width * 3] for y in range(height)]
    raw = b"".join(rows)
    png = (
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + _png_chunk(b"IDAT", zlib.compress(raw, 6))
        + _png_chunk(b"IEND", b"")
    )
    path.write_bytes(png)


def convert_ppm_to_png(ppm: pathlib.Path, png: pathlib.Path) -> None:
    width, height, pixels = read_ppm(ppm)
    png.parent.mkdir(parents=True, exist_ok=True)
    write_png(png, width, height, pixels)


def _repo_root() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parents[1]


def _default_output_dir(scene: str) -> pathlib.Path:
    safe = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in scene)
    if not safe:
        safe = "scene"
    return pathlib.Path(tempfile.gettempdir()) / "dart_py_demo_capture" / safe


def _safe_stem(value: str) -> str:
    safe = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in value)
    return safe or "scene"


def _capture_stem(args: argparse.Namespace) -> str:
    switch_scene = getattr(args, "switch_scene", "")
    if switch_scene:
        return f"{_safe_stem(args.scene)}_to_{_safe_stem(switch_scene)}"
    return _safe_stem(args.scene)


def _write_json(path: pathlib.Path, payload: dict[str, object]) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def _append_event(
    path: pathlib.Path, start_time: float, event: str, **fields: object
) -> None:
    payload: dict[str, object] = {
        "event": event,
        "source": "capture",
        "t_ms": round((time.monotonic() - start_time) * 1000.0, 3),
    }
    payload.update(fields)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, sort_keys=True) + "\n")


def _ffmpeg_path() -> str | None:
    found = shutil.which("ffmpeg")
    if found is not None:
        return found
    bundled = _repo_root() / ".pixi" / "envs" / "gazebo" / "bin" / "ffmpeg"
    if bundled.is_file():
        return str(bundled)
    return None


def build_demo_args(
    args: argparse.Namespace, screenshot: pathlib.Path, frames: pathlib.Path
) -> list[str]:
    if args.backend == "noop" and not args.allow_noop:
        raise ValueError("--backend noop is not valid for visual capture")

    demo_args = [
        "--scene",
        args.scene,
        "--headless",
        "--frames",
        str(args.frames),
        "--width",
        str(args.width),
        "--height",
        str(args.height),
        "--screenshot",
        str(screenshot),
        "--out",
        str(frames),
    ]
    if args.show_ui:
        demo_args.append("--show-ui")
    if args.backend:
        demo_args.extend(["--backend", args.backend])
    switch_scene = getattr(args, "switch_scene", "")
    if switch_scene:
        demo_args.extend(
            [
                "--scripted-demo-switch",
                f"{getattr(args, 'switch_frame', 2)}:{switch_scene}",
            ]
        )
        event_log = getattr(args, "event_log", None)
        if event_log is not None:
            demo_args.extend(["--scripted-demo-event-log", str(event_log)])
    return demo_args


def _run_demo(demo_args: list[str]) -> int:
    root = _repo_root()
    pixi_env = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    build_type = os.environ.get("BUILD_TYPE", "Release")
    build_python_dir = root / "build" / pixi_env / "cpp" / build_type / "python"
    if build_python_dir.is_dir() and str(build_python_dir) not in sys.path:
        sys.path.insert(0, str(build_python_dir))
    python_dir = root / "python"
    if str(python_dir) not in sys.path:
        sys.path.append(str(python_dir))

    from examples.demos.registry import make_demo_scenes
    from examples.demos.runner import run

    return run(demo_args, make_demo_scenes())


def _encode_video(frames: pathlib.Path, output: pathlib.Path, fps: int) -> bool:
    ffmpeg = _ffmpeg_path()
    if ffmpeg is None:
        return False
    output.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            ffmpeg,
            "-y",
            "-framerate",
            str(fps),
            "-i",
            str(frames / "frame_%06d.ppm"),
            "-pix_fmt",
            "yuv420p",
            str(output),
        ],
        check=True,
    )
    return True


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture a Python demo through the real Filament viewer."
    )
    parser.add_argument("--scene", default="hello_world")
    parser.add_argument("--frames", type=int, default=24)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--backend", default="")
    parser.add_argument("--allow-noop", action="store_true")
    parser.add_argument("--show-ui", action="store_true")
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--output-dir", type=pathlib.Path)
    parser.add_argument(
        "--switch-scene",
        default="",
        help="Request a demo switch during capture and record switch events.",
    )
    parser.add_argument(
        "--switch-frame",
        type=int,
        default=2,
        help="Rendered frame count after which --switch-scene is requested.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.frames < 1:
        raise SystemExit("--frames must be positive")
    if args.width < 1 or args.height < 1:
        raise SystemExit("--width and --height must be positive")
    if args.switch_frame < 1:
        raise SystemExit("--switch-frame must be positive")
    if args.switch_scene and args.frames <= args.switch_frame:
        raise SystemExit("--frames must be greater than --switch-frame")

    output_dir = args.output_dir or _default_output_dir(args.scene)
    output_dir.mkdir(parents=True, exist_ok=True)
    capture_stem = _capture_stem(args)
    screenshot_ppm = output_dir / f"{capture_stem}.ppm"
    screenshot_png = output_dir / f"{capture_stem}.png"
    frames_dir = output_dir / "frames"
    png_frames_dir = output_dir / "png_frames"
    manifest = output_dir / "manifest.json"
    events = output_dir / "events.jsonl"
    for path in (frames_dir, png_frames_dir):
        if path.exists():
            shutil.rmtree(path)
    for path in (screenshot_ppm, screenshot_png, manifest, events):
        if path.exists():
            path.unlink()

    if args.switch_scene:
        args.event_log = events
    start_time = time.monotonic()
    demo_args = build_demo_args(args, screenshot_ppm, frames_dir)
    _write_json(
        manifest,
        {
            "schema_version": 1,
            "scene": args.scene,
            "switch_scene": args.switch_scene or None,
            "switch_frame": args.switch_frame if args.switch_scene else None,
            "artifacts": {
                "events": str(events) if args.switch_scene else None,
                "frames": str(png_frames_dir),
                "screenshot": str(screenshot_png),
            },
        },
    )
    rc = _run_demo(demo_args)
    if args.switch_scene:
        _append_event(events, start_time, "capture_run_finished", return_code=rc)
    if rc != 0:
        return rc
    if not screenshot_ppm.is_file():
        raise SystemExit(f"demo did not write {screenshot_ppm}")
    if not ppm_has_nonzero_pixels(screenshot_ppm):
        raise SystemExit(f"{screenshot_ppm} contains only zero-valued pixels")

    convert_ppm_to_png(screenshot_ppm, screenshot_png)
    converted_frames = 0
    for ppm in sorted(frames_dir.glob("frame_*.ppm")):
        if not ppm_has_nonzero_pixels(ppm):
            raise SystemExit(f"{ppm} contains only zero-valued pixels")
        convert_ppm_to_png(ppm, png_frames_dir / f"{ppm.stem}.png")
        converted_frames += 1

    print(f"screenshot: {screenshot_png}")
    if converted_frames:
        print(f"frames: {png_frames_dir} ({converted_frames} PNG files)")
    if args.switch_scene:
        _append_event(
            events,
            start_time,
            "artifacts_written",
            converted_frames=converted_frames,
            screenshot=str(screenshot_png),
        )
        print(f"events: {events}")
    print(f"manifest: {manifest}")
    if args.video:
        video = output_dir / f"{capture_stem}.mp4"
        if _encode_video(frames_dir, video, args.fps):
            print(f"video: {video}")
        else:
            print("video: skipped (ffmpeg not found)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
