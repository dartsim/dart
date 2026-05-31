#!/usr/bin/env python3
"""Capture Python demo screenshots and optional videos for visual debugging."""

from __future__ import annotations

import argparse
import os
import pathlib
import shutil
import struct
import subprocess
import sys
import tempfile
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
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.frames < 1:
        raise SystemExit("--frames must be positive")
    if args.width < 1 or args.height < 1:
        raise SystemExit("--width and --height must be positive")

    output_dir = args.output_dir or _default_output_dir(args.scene)
    output_dir.mkdir(parents=True, exist_ok=True)
    screenshot_ppm = output_dir / f"{args.scene}.ppm"
    screenshot_png = output_dir / f"{args.scene}.png"
    frames_dir = output_dir / "frames"
    png_frames_dir = output_dir / "png_frames"
    for path in (frames_dir, png_frames_dir):
        if path.exists():
            shutil.rmtree(path)
    for path in (screenshot_ppm, screenshot_png):
        if path.exists():
            path.unlink()

    demo_args = build_demo_args(args, screenshot_ppm, frames_dir)
    rc = _run_demo(demo_args)
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
    if args.video:
        video = output_dir / f"{args.scene}.mp4"
        if _encode_video(frames_dir, video, args.fps):
            print(f"video: {video}")
        else:
            print("video: skipped (ffmpeg not found)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
