#!/usr/bin/env python3
"""Capture Python demo screenshots and optional videos for visual debugging."""

from __future__ import annotations

import argparse
import json
import math
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


def ppm_image_evidence(path: pathlib.Path) -> dict[str, object]:
    width, height, pixels = read_ppm(path)
    pixel_count = width * height
    nonzero_pixels = 0
    nonzero_channels = 0
    unique_colors: set[int] = set()
    channel_total = 0.0
    channel_square_total = 0.0
    luminance_total = 0.0
    luminance_square_total = 0.0

    for offset in range(0, len(pixels), 3):
        red = pixels[offset]
        green = pixels[offset + 1]
        blue = pixels[offset + 2]
        if red or green or blue:
            nonzero_pixels += 1
        nonzero_channels += int(red != 0) + int(green != 0) + int(blue != 0)
        unique_colors.add((red << 16) | (green << 8) | blue)
        channel_total += red + green + blue
        channel_square_total += red * red + green * green + blue * blue
        luminance = 0.2126 * red + 0.7152 * green + 0.0722 * blue
        luminance_total += luminance
        luminance_square_total += luminance * luminance

    channel_count = pixel_count * 3
    if channel_count:
        channel_mean = channel_total / channel_count
        channel_variance = max(
            0.0, channel_square_total / channel_count - channel_mean * channel_mean
        )
    else:
        channel_mean = 0.0
        channel_variance = 0.0
    if pixel_count:
        luminance_mean = luminance_total / pixel_count
        luminance_variance = max(
            0.0,
            luminance_square_total / pixel_count - luminance_mean * luminance_mean,
        )
    else:
        luminance_mean = 0.0
        luminance_variance = 0.0

    return {
        "width": width,
        "height": height,
        "pixel_count": pixel_count,
        "nonzero_pixels": nonzero_pixels,
        "nonzero_channels": nonzero_channels,
        "unique_rgb_count": len(unique_colors),
        "rgb_channel_mean": round(channel_mean, 3),
        "rgb_channel_variance": round(channel_variance, 3),
        "mean_luminance": round(luminance_mean, 3),
        "luminance_variance": round(luminance_variance, 3),
        "docked_workspace": _has_docked_workspace_regions(width, height, pixels),
    }


def _mean_luminance(
    pixels: bytes,
    width: int,
    x0: int,
    x1: int,
    y0: int,
    y1: int,
) -> float:
    total = 0.0
    count = 0
    for y in range(y0, y1):
        row = y * width * 3
        for x in range(x0, x1):
            offset = row + x * 3
            red, green, blue = pixels[offset : offset + 3]
            total += 0.2126 * red + 0.7152 * green + 0.0722 * blue
            count += 1
    if count == 0:
        return 0.0
    return total / count


def _scaled_bounds(
    width: int,
    height: int,
    x0: float,
    x1: float,
    y0: float,
    y1: float,
) -> tuple[int, int, int, int]:
    return (
        max(0, min(width, int(width * x0))),
        max(0, min(width, int(width * x1))),
        max(0, min(height, int(height * y0))),
        max(0, min(height, int(height * y1))),
    )


def _has_docked_workspace_regions(width: int, height: int, pixels: bytes) -> bool:
    if width < 32 or height < 32:
        return False

    def region(x0: float, x1: float, y0: float, y1: float) -> float:
        ix0, ix1, iy0, iy1 = _scaled_bounds(width, height, x0, x1, y0, y1)
        return _mean_luminance(pixels, width, ix0, ix1, iy0, iy1)

    top = region(0.0, 1.0, 0.0, 0.16)
    left = region(0.0, 0.24, 0.25, 0.88)
    right = region(0.73, 1.0, 0.25, 0.88)
    bottom = region(0.0, 1.0, 0.89, 1.0)
    center = region(0.28, 0.70, 0.28, 0.80)

    return (
        top < 95.0
        and left < 105.0
        and right < 105.0
        and bottom < 95.0
        and center > 85.0
        and center > left + 25.0
        and center > right + 25.0
    )


def ppm_has_docked_workspace_regions(path: pathlib.Path) -> bool:
    width, height, pixels = read_ppm(path)
    return _has_docked_workspace_regions(width, height, pixels)


def _prepare_frame_sequence(
    frames: pathlib.Path, require_docked_ui: bool
) -> tuple[list[pathlib.Path], int]:
    frame_paths = sorted(frames.glob("frame_*.ppm"))
    if not require_docked_ui or not frame_paths:
        return frame_paths, 0

    first_ready = next(
        (
            index
            for index, frame in enumerate(frame_paths)
            if ppm_has_docked_workspace_regions(frame)
        ),
        None,
    )
    if first_ready is None:
        raise SystemExit(
            f"{frames} does not contain a frame with the docked ImGui workspace"
        )

    ready_frames = frame_paths[first_ready:]
    missing_ui = [
        frame for frame in ready_frames if not ppm_has_docked_workspace_regions(frame)
    ]
    if missing_ui:
        raise SystemExit(
            f"{missing_ui[0]} lost the docked ImGui workspace after it became visible"
        )

    if first_ready == 0:
        return frame_paths, 0

    filtered = frames.parent / f"{frames.name}_ui_ready"
    if filtered.exists():
        shutil.rmtree(filtered)
    filtered.mkdir(parents=True)
    for index, frame in enumerate(ready_frames, start=1):
        shutil.copy2(frame, filtered / f"frame_{index:06d}.ppm")
    shutil.rmtree(frames)
    filtered.rename(frames)
    return sorted(frames.glob("frame_*.ppm")), first_ready


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


def _apply_stable_linux_render_env() -> None:
    if not sys.platform.startswith("linux"):
        return
    os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
    os.environ.setdefault("MESA_LOADER_DRIVER_OVERRIDE", "llvmpipe")
    egl_vendor = pathlib.Path("/usr/share/glvnd/egl_vendor.d/50_mesa.json")
    if egl_vendor.is_file():
        os.environ.setdefault("__EGL_VENDOR_LIBRARY_FILENAMES", str(egl_vendor))


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
    force_drag_target = getattr(args, "force_drag_target", "")
    if force_drag_target:
        return f"{_safe_stem(args.scene)}_force_{_safe_stem(force_drag_target)}"
    force_drag_pixel = getattr(args, "force_drag_pixel", None)
    if force_drag_pixel is not None:
        x, y = force_drag_pixel
        return f"{_safe_stem(args.scene)}_force_pixel_{x:g}_{y:g}"
    return _safe_stem(args.scene)


def _parse_vector3(text: str) -> tuple[float, float, float]:
    parts = text.split(",")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected <x>,<y>,<z>")
    try:
        vector = (float(parts[0]), float(parts[1]), float(parts[2]))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("expected finite numeric vector") from exc
    if not all(math.isfinite(component) for component in vector):
        raise argparse.ArgumentTypeError("expected finite numeric vector")
    return vector


def _parse_vector2(text: str) -> tuple[float, float]:
    parts = text.split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("expected <x>,<y>")
    try:
        vector = (float(parts[0]), float(parts[1]))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("expected finite numeric vector") from exc
    if not all(math.isfinite(component) for component in vector):
        raise argparse.ArgumentTypeError("expected finite numeric vector")
    return vector


def _format_vector3(value: tuple[float, float, float]) -> str:
    return ",".join(f"{component:g}" for component in value)


def _format_vector2(value: tuple[float, float]) -> str:
    return ",".join(f"{component:g}" for component in value)


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


def _read_scene_metrics_summary(path: pathlib.Path) -> dict[str, object] | None:
    if not path.is_file():
        return None

    event_count = 0
    latest: dict[str, object] | None = None
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            text = line.strip()
            if not text:
                continue
            try:
                payload = json.loads(text)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_number} is not valid JSON") from exc
            if not isinstance(payload, dict):
                continue
            if payload.get("event") != "scene_capture_metrics":
                continue
            event_count += 1
            latest = payload

    if latest is None:
        return None
    return {"event_count": event_count, "latest": latest}


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
    capture_metrics_event_log = getattr(args, "capture_metrics_event_log", None)
    if capture_metrics_event_log:
        demo_args.extend(
            ["--capture-metrics-event-log", str(capture_metrics_event_log)]
        )
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
    force_drag_target = getattr(args, "force_drag_target", "")
    if force_drag_target:
        demo_args.extend(
            [
                "--scripted-force-drag",
                (
                    f"{getattr(args, 'force_drag_frame', 2)}:"
                    f"{force_drag_target}:"
                    f"{_format_vector3(getattr(args, 'force_drag_delta'))}:"
                    f"{getattr(args, 'force_drag_frames', 8)}"
                ),
            ]
        )
        event_log = getattr(args, "event_log", None)
        if event_log is not None and "--scripted-demo-event-log" not in demo_args:
            demo_args.extend(["--scripted-demo-event-log", str(event_log)])
    force_drag_pixel = getattr(args, "force_drag_pixel", None)
    if force_drag_pixel is not None:
        demo_args.extend(
            [
                "--scripted-pointer-force-drag",
                (
                    f"{getattr(args, 'force_drag_frame', 2)}:"
                    f"{_format_vector2(force_drag_pixel)}:"
                    f"{_format_vector2(getattr(args, 'force_drag_delta_pixels'))}:"
                    f"{getattr(args, 'force_drag_frames', 8)}"
                ),
            ]
        )
        event_log = getattr(args, "event_log", None)
        if event_log is not None and "--scripted-demo-event-log" not in demo_args:
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
    parser.add_argument("--scene", default="rigid_body")
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
    parser.add_argument(
        "--force-drag-target",
        default="",
        help="Renderable name or id:<number> to scripted-force-drag in capture.",
    )
    parser.add_argument(
        "--force-drag-frame",
        type=int,
        default=2,
        help="Rendered frame count after which --force-drag-target starts.",
    )
    parser.add_argument(
        "--force-drag-frames",
        type=int,
        default=8,
        help="Number of frames to keep the scripted force drag active.",
    )
    parser.add_argument(
        "--force-drag-delta",
        type=_parse_vector3,
        default=(0.8, 0.0, 0.2),
        help="World-space target offset for scripted force drag as <x>,<y>,<z>.",
    )
    parser.add_argument(
        "--force-drag-pixel",
        type=_parse_vector2,
        help="Framebuffer pixel <x>,<y> where scripted force drag starts.",
    )
    parser.add_argument(
        "--force-drag-delta-pixels",
        type=_parse_vector2,
        default=(180.0, -60.0),
        help="Framebuffer pixel drag delta as <dx>,<dy>.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    _apply_stable_linux_render_env()
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.frames < 1:
        raise SystemExit("--frames must be positive")
    if args.width < 1 or args.height < 1:
        raise SystemExit("--width and --height must be positive")
    if args.switch_frame < 1:
        raise SystemExit("--switch-frame must be positive")
    if args.switch_scene and args.frames <= args.switch_frame:
        raise SystemExit("--frames must be greater than --switch-frame")
    has_force_drag = bool(args.force_drag_target) or args.force_drag_pixel is not None
    if args.switch_scene and has_force_drag:
        raise SystemExit("--switch-scene and force-drag capture cannot be combined")
    if args.force_drag_target and args.force_drag_pixel is not None:
        raise SystemExit(
            "--force-drag-target and --force-drag-pixel cannot be combined"
        )
    if args.force_drag_frame < 1:
        raise SystemExit("--force-drag-frame must be positive")
    if args.force_drag_frames < 1:
        raise SystemExit("--force-drag-frames must be positive")
    if has_force_drag and args.frames <= (
        args.force_drag_frame + args.force_drag_frames
    ):
        raise SystemExit(
            "--frames must be greater than --force-drag-frame + --force-drag-frames"
        )

    output_dir = args.output_dir or _default_output_dir(args.scene)
    output_dir.mkdir(parents=True, exist_ok=True)
    capture_stem = _capture_stem(args)
    screenshot_ppm = output_dir / f"{capture_stem}.ppm"
    screenshot_png = output_dir / f"{capture_stem}.png"
    frames_dir = output_dir / "frames"
    png_frames_dir = output_dir / "png_frames"
    manifest = output_dir / "manifest.json"
    events = output_dir / "events.jsonl"
    scene_metrics_events = output_dir / "scene_metrics.jsonl"
    for path in (frames_dir, png_frames_dir):
        if path.exists():
            shutil.rmtree(path)
    for path in (
        screenshot_ppm,
        screenshot_png,
        manifest,
        events,
        scene_metrics_events,
    ):
        if path.exists():
            path.unlink()

    needs_event_log = bool(args.switch_scene or has_force_drag)
    if needs_event_log:
        args.event_log = events
    args.capture_metrics_event_log = scene_metrics_events
    start_time = time.monotonic()
    demo_args = build_demo_args(args, screenshot_ppm, frames_dir)
    capture_metadata: dict[str, object] = {
        "converted_frames": 0,
        "requested_frames": args.frames,
        "width": args.width,
        "height": args.height,
    }
    visual_evidence: dict[str, object] = {
        "first_frame": None,
        "screenshot": None,
    }
    manifest_payload: dict[str, object] = {
        "schema_version": 1,
        "scene": args.scene,
        "switch_scene": args.switch_scene or None,
        "switch_frame": args.switch_frame if args.switch_scene else None,
        "force_drag": (
            {
                "delta": list(args.force_drag_delta),
                "duration_frames": args.force_drag_frames,
                "start_frame": args.force_drag_frame,
                "target": args.force_drag_target,
            }
            if args.force_drag_target
            else (
                {
                    "delta_pixels": list(args.force_drag_delta_pixels),
                    "duration_frames": args.force_drag_frames,
                    "pixel": list(args.force_drag_pixel),
                    "start_frame": args.force_drag_frame,
                }
                if args.force_drag_pixel is not None
                else None
            )
        ),
        "artifacts": {
            "events": str(events) if needs_event_log else None,
            "frames": str(png_frames_dir),
            "scene_metrics_events": None,
            "screenshot": str(screenshot_png),
        },
        "capture": capture_metadata,
        "scene_metrics": None,
        "show_ui": args.show_ui,
        "ui_ready": None,
        "visual_evidence": visual_evidence,
    }
    _write_json(manifest, manifest_payload)
    rc = _run_demo(demo_args)
    if needs_event_log:
        _append_event(events, start_time, "capture_run_finished", return_code=rc)
    if rc != 0:
        return rc
    if not screenshot_ppm.is_file():
        raise SystemExit(f"demo did not write {screenshot_ppm}")
    if not ppm_has_nonzero_pixels(screenshot_ppm):
        raise SystemExit(f"{screenshot_ppm} contains only zero-valued pixels")
    require_docked_ui = args.show_ui and not args.allow_noop
    if require_docked_ui and not ppm_has_docked_workspace_regions(screenshot_ppm):
        raise SystemExit(f"{screenshot_ppm} does not show the docked ImGui workspace")

    convert_ppm_to_png(screenshot_ppm, screenshot_png)
    frame_paths, dropped_warmup_frames = _prepare_frame_sequence(
        frames_dir, require_docked_ui
    )
    converted_frames = 0
    for ppm in frame_paths:
        if not ppm_has_nonzero_pixels(ppm):
            raise SystemExit(f"{ppm} contains only zero-valued pixels")
        convert_ppm_to_png(ppm, png_frames_dir / f"{ppm.stem}.png")
        converted_frames += 1
    capture_metadata["converted_frames"] = converted_frames
    visual_evidence["screenshot"] = ppm_image_evidence(screenshot_ppm)
    visual_evidence["first_frame"] = (
        ppm_image_evidence(frame_paths[0]) if frame_paths else None
    )
    manifest_payload["scene_metrics"] = _read_scene_metrics_summary(
        scene_metrics_events
    )
    manifest_payload["artifacts"]["scene_metrics_events"] = (
        str(scene_metrics_events) if scene_metrics_events.is_file() else None
    )
    manifest_payload["ui_ready"] = {
        "dropped_warmup_frames": dropped_warmup_frames,
        "required": require_docked_ui,
    }
    _write_json(manifest, manifest_payload)

    print(f"screenshot: {screenshot_png}")
    if converted_frames:
        print(f"frames: {png_frames_dir} ({converted_frames} PNG files)")
    if needs_event_log:
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
