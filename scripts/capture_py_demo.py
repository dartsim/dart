#!/usr/bin/env python3
"""Capture Python demo screenshots and optional videos for visual debugging."""

from __future__ import annotations

import argparse
import html
import json
import math
import os
import pathlib
import shlex
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


RIGID_WORKFLOW_CAPTURE_SPECS: tuple[tuple[str, int, int, int, bool], ...] = (
    ("rigid_body", 24, 960, 540, True),
    ("rigid_body_modes", 72, 960, 540, True),
    ("rigid_free_flight", 96, 960, 540, True),
    ("rigid_frame_hierarchy", 72, 960, 540, True),
    ("rigid_external_loads", 72, 960, 540, True),
    ("rigid_link_point_loads", 72, 960, 540, True),
    ("rigid_timestep_sensitivity", 96, 960, 540, True),
    ("rigid_step_diagnostics", 72, 960, 540, True),
    ("rigid_contact_scale_budget", 72, 960, 540, True),
    ("rigid_restitution_ladder", 96, 960, 540, True),
    ("rigid_material_mixing", 72, 960, 540, True),
    ("rigid_contact_inspector", 24, 960, 540, True),
    ("rigid_collision_query_options", 24, 960, 540, True),
    ("rigid_collision_casts", 48, 960, 540, True),
    ("rigid_solver_compare", 24, 960, 540, True),
    ("rigid_executor_equivalence", 24, 960, 540, True),
    ("rigid_contact_solver_compare", 72, 960, 540, True),
    ("contact", 144, 960, 540, True),
    ("rigid_friction_threshold", 24, 960, 540, True),
    ("rigid_spin_roll_coupling", 96, 960, 540, True),
    ("rigid_stack_stability", 24, 960, 540, True),
    ("rigid_contact_manipulation", 72, 960, 540, True),
    ("rigid_kinematic_driver", 72, 960, 540, True),
    ("rigid_kinematic_normal_push", 72, 960, 540, True),
    ("rigid_fixed_joint", 24, 960, 540, True),
    ("rigid_joint_breakage", 48, 960, 540, True),
    ("rigid_distance_spring", 72, 960, 540, True),
    ("rigid_limited_joints", 24, 960, 540, True),
    ("rigid_joint_motor_limits", 72, 960, 540, True),
    ("rigid_joint_passive_parameters", 120, 960, 540, True),
    ("rigid_screw_joint_pitch", 96, 960, 540, True),
    ("rigid_multibody_dynamics_terms", 96, 960, 540, True),
    ("rigid_link_center_of_mass", 72, 960, 540, True),
    ("rigid_link_jacobian", 96, 960, 540, True),
    ("rigid_multibody_solver_family", 72, 960, 540, True),
    ("rigid_loop_closure", 72, 960, 540, True),
)


RIGID_WORKFLOW_RELATED_CAPTURE_SPECS: tuple[tuple[str, int, int, int, bool], ...] = (
    ("floating_base", 72, 960, 540, True),
    ("articulated", 72, 960, 540, True),
    ("rigid_ipc_tunnel", 24, 960, 540, True),
    ("rigid_ipc_edge_drop", 72, 960, 540, True),
    ("diff_drone_liftoff", 96, 960, 540, True),
    ("avbd_rigid_fixed_joint_contact", 72, 960, 540, True),
    ("avbd_rigid_breakable_joint", 72, 960, 540, True),
    ("avbd_rigid_spherical_breakable_joint", 72, 960, 540, True),
    ("avbd_rigid_revolute_motor", 72, 960, 540, True),
    ("avbd_rigid_prismatic_motor", 72, 960, 540, True),
)


RIGID_WORKFLOW_IPC_SHELF_CAPTURE_SPECS: tuple[tuple[str, int, int, int, bool], ...] = (
    ("rigid_ipc", 72, 960, 540, True),
    ("rigid_ipc_slide", 72, 960, 540, True),
    ("rigid_ipc_incline", 72, 960, 540, True),
    ("rigid_ipc_pile", 72, 960, 540, True),
)


RIGID_WORKFLOW_PACKET_CAPTURE_SPECS: tuple[tuple[str, int, int, int, bool], ...] = (
    ("rigid_ipc_stack_packet", 24, 960, 540, True),
)


def rigid_workflow_capture_specs() -> tuple[tuple[str, int, int, int, bool], ...]:
    return RIGID_WORKFLOW_CAPTURE_SPECS


def rigid_workflow_related_capture_specs() -> (
    tuple[tuple[str, int, int, int, bool], ...]
):
    return RIGID_WORKFLOW_RELATED_CAPTURE_SPECS


def rigid_workflow_ipc_shelf_capture_specs() -> (
    tuple[tuple[str, int, int, int, bool], ...]
):
    return RIGID_WORKFLOW_IPC_SHELF_CAPTURE_SPECS


def rigid_workflow_packet_capture_specs() -> (
    tuple[tuple[str, int, int, int, bool], ...]
):
    return RIGID_WORKFLOW_PACKET_CAPTURE_SPECS


def _rigid_workflow_guidance_by_scene() -> dict[str, dict[str, object]]:
    python_dir = _repo_root() / "python"
    if str(python_dir) not in sys.path:
        sys.path.append(str(python_dir))
    try:
        from examples.demos.runner import RIGID_VISUAL_WORKFLOW_GUIDES
    except Exception:
        return {}

    guidance: dict[str, dict[str, object]] = {}
    for scene_id, guide in RIGID_VISUAL_WORKFLOW_GUIDES.items():
        guidance[str(scene_id)] = {
            "workflow_label": guide.label,
            "user_question": guide.question,
            "try_first": guide.try_first,
            "inspect": list(guide.inspect),
            "healthy_signal": guide.healthy_signal,
            "scope": guide.scope,
        }
    return guidance


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
    first: dict[str, object] | None = None
    latest: dict[str, object] | None = None
    metric_key_counts: dict[str, int] = {}
    numeric_ranges: dict[str, dict[str, float]] = {}
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
            if first is None:
                first = payload
            latest = payload
            metrics = payload.get("metrics")
            if isinstance(metrics, dict):
                for key, value in metrics.items():
                    if not isinstance(key, str):
                        continue
                    metric_key_counts[key] = metric_key_counts.get(key, 0) + 1
                    if isinstance(value, bool) or not isinstance(value, int | float):
                        continue
                    range_entry = numeric_ranges.setdefault(
                        key, {"max": float(value), "min": float(value)}
                    )
                    range_entry["max"] = max(range_entry["max"], float(value))
                    range_entry["min"] = min(range_entry["min"], float(value))

    if latest is None:
        return None
    return {
        "event_count": event_count,
        "first": first,
        "latest": latest,
        "metric_key_counts": dict(sorted(metric_key_counts.items())),
        "numeric_ranges": dict(sorted(numeric_ranges.items())),
    }


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
    parser.add_argument(
        "--rigid-workflow",
        action="store_true",
        help="Capture or plan the full PLAN-103 rigid visual workflow.",
    )
    parser.add_argument(
        "--include-related",
        action="store_true",
        help=(
            "With --rigid-workflow, also capture non-numbered related-evidence "
            "shelf routes."
        ),
    )
    parser.add_argument(
        "--include-ipc-shelf",
        action="store_true",
        help=(
            "With --rigid-workflow, also capture direct metric-backed Rigid IPC "
            "shelf scenes."
        ),
    )
    parser.add_argument(
        "--include-packets",
        action="store_true",
        help=(
            "With --rigid-workflow, also capture non-numbered capture-first "
            "rigid evidence packets."
        ),
    )
    parser.add_argument(
        "--workflow-start-row",
        type=int,
        help=(
            "With --rigid-workflow, start at this 1-based workflow row for "
            "targeted reruns."
        ),
    )
    parser.add_argument(
        "--workflow-end-row",
        type=int,
        help=(
            "With --rigid-workflow, stop at this 1-based workflow row for "
            "targeted reruns."
        ),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Write the rigid workflow capture plan without rendering.",
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


def _workflow_output_dir(args: argparse.Namespace) -> pathlib.Path:
    return args.output_dir or _default_output_dir("rigid_workflow")


def _workflow_scene_output_dir(
    output_dir: pathlib.Path, order: int, scene: str
) -> pathlib.Path:
    return output_dir / "scenes" / f"{order:02d}_{_safe_stem(scene)}"


def _workflow_capture_specs(
    args: argparse.Namespace,
) -> tuple[tuple[str, int, int, int, bool], ...]:
    specs = list(rigid_workflow_capture_specs())
    if getattr(args, "include_related", False):
        specs.extend(rigid_workflow_related_capture_specs())
    if getattr(args, "include_ipc_shelf", False):
        specs.extend(rigid_workflow_ipc_shelf_capture_specs())
    if getattr(args, "include_packets", False):
        specs.extend(rigid_workflow_packet_capture_specs())
    return tuple(specs)


def _workflow_requested_include_flags(
    args: argparse.Namespace,
) -> dict[str, bool]:
    return {
        "include_related": bool(getattr(args, "include_related", False)),
        "include_ipc_shelf": bool(getattr(args, "include_ipc_shelf", False)),
        "include_packets": bool(getattr(args, "include_packets", False)),
    }


def _workflow_row_bounds(args: argparse.Namespace, total_count: int) -> tuple[int, int]:
    start = getattr(args, "workflow_start_row", None)
    end = getattr(args, "workflow_end_row", None)
    if start is None:
        start = 1
    if end is None:
        end = total_count
    if start < 1:
        raise SystemExit("--workflow-start-row must be >= 1")
    if end < 1:
        raise SystemExit("--workflow-end-row must be >= 1")
    if start > end:
        raise SystemExit("--workflow-start-row must be <= --workflow-end-row")
    if end > total_count:
        raise SystemExit(
            f"--workflow-end-row must be <= {total_count} for this workflow packet"
        )
    return start, end


def _workflow_scene_argv(
    args: argparse.Namespace,
    order: int,
    spec: tuple[str, int, int, int, bool],
    output_dir: pathlib.Path,
) -> list[str]:
    scene, frames, width, height, show_ui = spec
    scene_output = _workflow_scene_output_dir(output_dir, order, scene)
    scene_argv = [
        "--scene",
        scene,
        "--frames",
        str(frames),
        "--width",
        str(width),
        "--height",
        str(height),
        "--output-dir",
        str(scene_output),
    ]
    if show_ui:
        scene_argv.append("--show-ui")
    if args.backend:
        scene_argv.extend(["--backend", args.backend])
    if args.allow_noop:
        scene_argv.append("--allow-noop")
    return scene_argv


def _public_command(argv: list[str]) -> str:
    return "pixi run py-demo-capture -- " + " ".join(shlex.quote(arg) for arg in argv)


def _workflow_plan_entries(
    args: argparse.Namespace, output_dir: pathlib.Path
) -> list[dict[str, object]]:
    entries: list[dict[str, object]] = []
    specs = _workflow_capture_specs(args)
    numbered_count = len(rigid_workflow_capture_specs())
    related_count = (
        len(rigid_workflow_related_capture_specs())
        if getattr(args, "include_related", False)
        else 0
    )
    ipc_shelf_count = (
        len(rigid_workflow_ipc_shelf_capture_specs())
        if getattr(args, "include_ipc_shelf", False)
        else 0
    )
    count = len(specs)
    start_row, end_row = _workflow_row_bounds(args, count)
    guidance_by_scene = _rigid_workflow_guidance_by_scene()
    for order, spec in enumerate(specs, start=1):
        if order < start_row or order > end_row:
            continue
        scene, frames, width, height, show_ui = spec
        scene_output = _workflow_scene_output_dir(output_dir, order, scene)
        argv = _workflow_scene_argv(args, order, spec, output_dir)
        if order <= numbered_count:
            workflow_group = "numbered"
        elif order <= numbered_count + related_count:
            workflow_group = "related_evidence"
        elif order <= numbered_count + related_count + ipc_shelf_count:
            workflow_group = "rigid_ipc_shelf"
        else:
            workflow_group = "capture_first_packet"
        entries.append(
            {
                "order": order,
                "count": count,
                "workflow_group": workflow_group,
                "scene": scene,
                "frames": frames,
                "width": width,
                "height": height,
                "show_ui": show_ui,
                "output_dir": str(scene_output),
                "manifest": str(scene_output / "manifest.json"),
                "command": _public_command(argv),
                "status": "planned",
            }
        )
        if workflow_group == "numbered":
            entries[-1].update(guidance_by_scene.get(scene, {}))
    return entries


def _workflow_review_index_path(output_dir: pathlib.Path) -> pathlib.Path:
    return output_dir / "review_index.html"


def _workflow_href(target: pathlib.Path, base_dir: pathlib.Path) -> str:
    try:
        href = os.path.relpath(target, start=base_dir)
    except ValueError:
        href = str(target)
    return href.replace(os.sep, "/")


def _workflow_scene_manifest_summary(
    capture: dict[str, object], output_dir: pathlib.Path
) -> dict[str, object]:
    manifest_value = capture.get("manifest")
    if not isinstance(manifest_value, str):
        return {}
    manifest_path = pathlib.Path(manifest_value)
    summary: dict[str, object] = {
        "manifest_href": _workflow_href(manifest_path, output_dir),
    }
    if not manifest_path.is_file():
        return summary

    try:
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        summary["read_error"] = str(exc)
        return summary
    if not isinstance(payload, dict):
        summary["read_error"] = "manifest payload is not an object"
        return summary

    artifacts = payload.get("artifacts")
    if isinstance(artifacts, dict):
        screenshot = artifacts.get("screenshot")
        if isinstance(screenshot, str) and screenshot:
            screenshot_path = pathlib.Path(screenshot)
            if not screenshot_path.is_absolute():
                screenshot_path = manifest_path.parent / screenshot_path
            summary["screenshot_href"] = _workflow_href(screenshot_path, output_dir)
        frames = artifacts.get("frames")
        if isinstance(frames, str) and frames:
            frames_path = pathlib.Path(frames)
            if not frames_path.is_absolute():
                frames_path = manifest_path.parent / frames_path
            summary["frames_href"] = _workflow_href(frames_path, output_dir)

    scene_metrics = payload.get("scene_metrics")
    if isinstance(scene_metrics, dict):
        metric_key_counts = scene_metrics.get("metric_key_counts")
        if isinstance(metric_key_counts, dict):
            summary["metric_keys"] = sorted(
                key for key in metric_key_counts if isinstance(key, str)
            )
        latest = scene_metrics.get("latest")
        if isinstance(latest, dict):
            metrics = latest.get("metrics")
            if isinstance(metrics, dict):
                comparison_axis = metrics.get("comparison_axis")
                if isinstance(comparison_axis, str):
                    summary["comparison_axis"] = comparison_axis
                held_fixed = metrics.get("held_fixed")
                if isinstance(held_fixed, dict):
                    summary["held_fixed_keys"] = sorted(
                        key for key in held_fixed if isinstance(key, str)
                    )

    return summary


def _workflow_badge(label: str, value: object) -> str:
    return (
        '<span class="badge">'
        f"<strong>{html.escape(label)}</strong> {html.escape(str(value))}"
        "</span>"
    )


def _workflow_detail(label: str, value: object) -> str:
    return f"<dt>{html.escape(label)}</dt>" f"<dd>{html.escape(str(value))}</dd>"


def _workflow_link(label: str, href: object) -> str:
    return (
        f'<a href="{html.escape(str(href), quote=True)}">' f"{html.escape(label)}</a>"
    )


def _workflow_review_card(capture: dict[str, object], output_dir: pathlib.Path) -> str:
    summary = _workflow_scene_manifest_summary(capture, output_dir)
    order = capture.get("order", "?")
    count = capture.get("count", "?")
    scene = str(capture.get("scene", "unknown"))
    status = str(capture.get("status", "planned"))
    title = f"{order}/{count} {scene}"
    command = str(capture.get("command", ""))
    image_href = summary.get("screenshot_href")
    if isinstance(image_href, str):
        image = (
            f'<a class="thumb" href="{html.escape(image_href, quote=True)}">'
            f'<img src="{html.escape(image_href, quote=True)}" '
            f'alt="{html.escape(scene, quote=True)} screenshot" loading="lazy">'
            "</a>"
        )
    else:
        image = (
            '<div class="thumb placeholder">'
            f"{html.escape(status.replace('_', ' '))}"
            "</div>"
        )

    links: list[str] = []
    manifest_href = summary.get("manifest_href")
    if isinstance(manifest_href, str):
        links.append(_workflow_link("manifest", manifest_href))
    frames_href = summary.get("frames_href")
    if isinstance(frames_href, str):
        links.append(_workflow_link("frames", frames_href))
    if isinstance(image_href, str):
        links.append(_workflow_link("screenshot", image_href))
    link_html = " | ".join(links) if links else "manifest pending"

    details = [
        _workflow_detail("group", capture.get("workflow_group", "")),
        _workflow_detail("frames", capture.get("frames", "")),
        _workflow_detail(
            "size",
            f"{capture.get('width', '')}x{capture.get('height', '')}",
        ),
        _workflow_detail("ui", "docked" if capture.get("show_ui") else "headless"),
    ]
    workflow_label = capture.get("workflow_label")
    if isinstance(workflow_label, str) and workflow_label:
        details.insert(0, _workflow_detail("role", workflow_label))
    user_question = capture.get("user_question")
    if isinstance(user_question, str) and user_question:
        details.append(_workflow_detail("question", user_question))
    try_first = capture.get("try_first")
    if isinstance(try_first, str) and try_first:
        details.append(_workflow_detail("try first", try_first))
    inspect = capture.get("inspect")
    if isinstance(inspect, list) and inspect:
        details.append(
            _workflow_detail("inspect", " / ".join(str(item) for item in inspect))
        )
    healthy_signal = capture.get("healthy_signal")
    if isinstance(healthy_signal, str) and healthy_signal:
        details.append(_workflow_detail("healthy", healthy_signal))
    scope = capture.get("scope")
    if isinstance(scope, str) and scope:
        details.append(_workflow_detail("scope", scope))
    return_code = capture.get("return_code")
    if return_code is not None:
        details.append(_workflow_detail("return code", return_code))
    failure_reason = capture.get("failure_reason")
    if failure_reason is not None:
        details.append(_workflow_detail("failure", failure_reason))
    comparison_axis = summary.get("comparison_axis")
    if isinstance(comparison_axis, str):
        details.append(_workflow_detail("axis", comparison_axis))
    held_fixed_keys = summary.get("held_fixed_keys")
    if isinstance(held_fixed_keys, list) and held_fixed_keys:
        details.append(_workflow_detail("held fixed", ", ".join(held_fixed_keys)))
    metric_keys = summary.get("metric_keys")
    if isinstance(metric_keys, list) and metric_keys:
        shown = ", ".join(str(key) for key in metric_keys[:8])
        if len(metric_keys) > 8:
            shown = f"{shown}, +{len(metric_keys) - 8} more"
        details.append(_workflow_detail("metrics", shown))
    read_error = summary.get("read_error")
    if isinstance(read_error, str):
        details.append(_workflow_detail("manifest error", read_error))

    status_class = "".join(ch if ch.isalnum() else "-" for ch in status.lower())
    return f"""
<article class="card {html.escape(status_class, quote=True)}">
  <div class="card-header">
    <h2>{html.escape(title)}</h2>
    <span class="status">{html.escape(status)}</span>
  </div>
  {image}
  <div class="card-body">
    <p class="links">{link_html}</p>
    <dl>{''.join(details)}</dl>
    <pre>{html.escape(command)}</pre>
  </div>
</article>
"""


def _write_workflow_review_index(
    output_dir: pathlib.Path,
    *,
    dry_run: bool,
    status: str,
    captures: list[dict[str, object]],
    started_at: float,
) -> pathlib.Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    index = _workflow_review_index_path(output_dir)
    completed = sum(1 for capture in captures if capture.get("status") == "captured")
    failed = sum(1 for capture in captures if capture.get("status") == "failed")
    elapsed_s = round(time.monotonic() - started_at, 3)
    cards = "\n".join(
        _workflow_review_card(capture, output_dir) for capture in captures
    )
    badges = " ".join(
        [
            _workflow_badge("status", status),
            _workflow_badge("captures", len(captures)),
            _workflow_badge("complete", completed),
            _workflow_badge("failed", failed),
            _workflow_badge("elapsed s", elapsed_s),
            _workflow_badge("mode", "plan" if dry_run else "capture"),
        ]
    )
    index.write_text(
        f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>DART Rigid Workflow Review Index</title>
  <style>
    :root {{
      color-scheme: light;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #f6f8fa;
      color: #24292f;
    }}
    body {{
      margin: 0;
    }}
    header {{
      padding: 24px;
      background: #ffffff;
      border-bottom: 1px solid #d0d7de;
    }}
    h1 {{
      margin: 0 0 12px;
      font-size: 24px;
      font-weight: 650;
    }}
    h2 {{
      margin: 0;
      font-size: 15px;
      font-weight: 650;
    }}
    .summary {{
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      margin: 12px 0;
    }}
    .badge {{
      display: inline-flex;
      gap: 6px;
      align-items: center;
      padding: 4px 8px;
      border: 1px solid #d0d7de;
      border-radius: 6px;
      background: #f6f8fa;
      font-size: 12px;
    }}
    main {{
      padding: 16px;
    }}
    .grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
      gap: 12px;
    }}
    .card {{
      background: #ffffff;
      border: 1px solid #d0d7de;
      border-radius: 6px;
      overflow: hidden;
    }}
    .card-header {{
      display: flex;
      justify-content: space-between;
      gap: 8px;
      align-items: center;
      padding: 10px 12px;
      border-bottom: 1px solid #d0d7de;
    }}
    .status {{
      flex: 0 0 auto;
      padding: 2px 8px;
      border-radius: 999px;
      background: #ddf4ff;
      color: #0550ae;
      font-size: 12px;
      font-weight: 650;
    }}
    .captured .status {{
      background: #dafbe1;
      color: #116329;
    }}
    .failed .status {{
      background: #ffebe9;
      color: #cf222e;
    }}
    .thumb {{
      display: flex;
      align-items: center;
      justify-content: center;
      aspect-ratio: 16 / 9;
      background: #d8dee4;
      color: #57606a;
      text-decoration: none;
      font-size: 13px;
      font-weight: 650;
    }}
    .thumb img {{
      width: 100%;
      height: 100%;
      object-fit: cover;
      display: block;
    }}
    .card-body {{
      padding: 12px;
    }}
    .links {{
      margin: 0 0 10px;
      font-size: 13px;
    }}
    dl {{
      display: grid;
      grid-template-columns: max-content 1fr;
      gap: 4px 10px;
      margin: 0 0 10px;
      font-size: 13px;
    }}
    dt {{
      color: #57606a;
      font-weight: 650;
    }}
    dd {{
      margin: 0;
      overflow-wrap: anywhere;
    }}
    pre {{
      margin: 0;
      padding: 8px;
      border-radius: 6px;
      background: #f6f8fa;
      overflow-wrap: anywhere;
      white-space: pre-wrap;
      font-size: 12px;
      line-height: 1.35;
    }}
    a {{
      color: #0969da;
    }}
  </style>
</head>
<body>
  <header>
    <h1>DART rigid workflow review index</h1>
    <div class="summary">{badges}</div>
    <p>Generated by <code>scripts/capture_py_demo.py</code>. Open this file after
    <code>py-demo-capture -- --rigid-workflow</code> to inspect the selected
    rigid visual-verification captures from one page.</p>
    <p>{_workflow_link("workflow manifest", "manifest.json")}</p>
  </header>
  <main>
    <section class="grid">
{cards}
    </section>
  </main>
</body>
</html>
""",
        encoding="utf-8",
    )
    return index


def _write_workflow_manifest(
    output_dir: pathlib.Path,
    *,
    dry_run: bool,
    status: str,
    captures: list[dict[str, object]],
    started_at: float,
    requested_include_flags: dict[str, bool] | None = None,
) -> pathlib.Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest = output_dir / "manifest.json"
    review_index = _write_workflow_review_index(
        output_dir,
        dry_run=dry_run,
        status=status,
        captures=captures,
        started_at=started_at,
    )
    completed = sum(1 for capture in captures if capture.get("status") == "captured")
    failed = sum(1 for capture in captures if capture.get("status") == "failed")
    selected_include_related = any(
        capture.get("workflow_group") == "related_evidence" for capture in captures
    )
    selected_include_ipc_shelf = any(
        capture.get("workflow_group") == "rigid_ipc_shelf" for capture in captures
    )
    selected_include_packets = any(
        capture.get("workflow_group") == "capture_first_packet" for capture in captures
    )
    requested_include_flags = requested_include_flags or {
        "include_related": selected_include_related,
        "include_ipc_shelf": selected_include_ipc_shelf,
        "include_packets": selected_include_packets,
    }
    capture_orders = [
        int(capture["order"])
        for capture in captures
        if isinstance(capture.get("order"), int)
    ]
    workflow_total_count = (
        max(
            int(capture.get("count", len(captures)))
            for capture in captures
            if isinstance(capture.get("count"), int)
        )
        if captures
        else 0
    )
    _write_json(
        manifest,
        {
            "schema_version": 1,
            "workflow": "rigid_visual_verification",
            "include_related": requested_include_flags["include_related"],
            "include_ipc_shelf": requested_include_flags["include_ipc_shelf"],
            "include_packets": requested_include_flags["include_packets"],
            "selected_include_related": selected_include_related,
            "selected_include_ipc_shelf": selected_include_ipc_shelf,
            "selected_include_packets": selected_include_packets,
            "workflow_total_count": workflow_total_count,
            "workflow_row_start": min(capture_orders) if capture_orders else None,
            "workflow_row_end": max(capture_orders) if capture_orders else None,
            "dry_run": dry_run,
            "status": status,
            "capture_count": len(captures),
            "completed_count": completed,
            "failed_count": failed,
            "elapsed_s": round(time.monotonic() - started_at, 3),
            "output_dir": str(output_dir),
            "artifacts": {
                "review_index": str(review_index),
            },
            "captures": captures,
        },
    )
    return manifest


def _run_scene_capture_from_argv(argv: list[str]) -> int:
    return subprocess.run(
        [sys.executable, str(pathlib.Path(__file__).resolve()), *argv],
        check=False,
    ).returncode


def _validate_rigid_workflow_args(args: argparse.Namespace) -> None:
    has_force_drag = bool(args.force_drag_target) or args.force_drag_pixel is not None
    if args.switch_scene or has_force_drag:
        raise SystemExit(
            "--rigid-workflow cannot be combined with scripted switch or force drag"
        )
    if args.video:
        raise SystemExit("--video is only supported for single-scene captures")


def _run_rigid_workflow(args: argparse.Namespace) -> int:
    _validate_rigid_workflow_args(args)
    output_dir = _workflow_output_dir(args)
    started_at = time.monotonic()
    requested_include_flags = _workflow_requested_include_flags(args)
    specs = _workflow_capture_specs(args)
    _workflow_row_bounds(args, len(specs))
    captures = _workflow_plan_entries(args, output_dir)
    if args.dry_run:
        manifest = _write_workflow_manifest(
            output_dir,
            dry_run=True,
            status="planned",
            captures=captures,
            started_at=started_at,
            requested_include_flags=requested_include_flags,
        )
        for capture in captures:
            print(capture["command"], flush=True)
        print(f"manifest: {manifest}", flush=True)
        return 0

    manifest = _write_workflow_manifest(
        output_dir,
        dry_run=False,
        status="running",
        captures=captures,
        started_at=started_at,
        requested_include_flags=requested_include_flags,
    )
    for capture in captures:
        order = int(capture["order"])
        spec = specs[order - 1]
        scene = str(capture["scene"])
        print(f"[{order}/{len(specs)}] capturing {scene}", flush=True)
        rc = _run_scene_capture_from_argv(
            _workflow_scene_argv(args, order, spec, output_dir)
        )
        capture["return_code"] = rc
        scene_manifest = pathlib.Path(str(capture["manifest"]))
        capture["manifest_exists"] = scene_manifest.is_file()
        if rc == 0 and scene_manifest.is_file():
            capture["status"] = "captured"
            status = "running"
        else:
            capture["status"] = "failed"
            capture["failure_reason"] = "return_code" if rc != 0 else "missing_manifest"
            status = "failed"
        manifest = _write_workflow_manifest(
            output_dir,
            dry_run=False,
            status=status,
            captures=captures,
            started_at=started_at,
            requested_include_flags=requested_include_flags,
        )
        if capture["status"] == "failed":
            print(f"manifest: {manifest}", flush=True)
            return rc if rc != 0 else 1

    manifest = _write_workflow_manifest(
        output_dir,
        dry_run=False,
        status="complete",
        captures=captures,
        started_at=started_at,
        requested_include_flags=requested_include_flags,
    )
    print(f"workflow manifest: {manifest}", flush=True)
    return 0


def main(argv: list[str] | None = None) -> int:
    _apply_stable_linux_render_env()
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.include_related and not args.rigid_workflow:
        raise SystemExit("--include-related requires --rigid-workflow")
    if args.include_ipc_shelf and not args.rigid_workflow:
        raise SystemExit("--include-ipc-shelf requires --rigid-workflow")
    if args.include_packets and not args.rigid_workflow:
        raise SystemExit("--include-packets requires --rigid-workflow")
    if (
        args.workflow_start_row is not None or args.workflow_end_row is not None
    ) and not args.rigid_workflow:
        raise SystemExit(
            "--workflow-start-row/--workflow-end-row require --rigid-workflow"
        )
    if args.rigid_workflow:
        return _run_rigid_workflow(args)
    if args.dry_run:
        raise SystemExit("--dry-run requires --rigid-workflow")
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
