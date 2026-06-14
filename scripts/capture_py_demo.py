#!/usr/bin/env python3
"""Capture Python demo screenshots and optional videos for visual debugging."""

from __future__ import annotations

import argparse
import html
import importlib.util
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
    ("rigid_body", 180, 960, 540, True),
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
    ("rigid_joint_motor_limits", 96, 960, 540, True),
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
    ("diff_pre_contact_surrogate", 24, 960, 540, True),
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
    ("rigid_ipc_heavy_stack_packet", 12, 960, 540, True),
)

_RIGID_WORKFLOW_IPC_SHELF_GUIDANCE_BY_SCENE: dict[str, dict[str, object]] = {
    "rigid_ipc": {
        "workflow_label": "Rigid IPC shelf",
        "user_question": (
            "Can a free box settle above static ground on the rigid IPC barrier?"
        ),
        "try_first": (
            "Use the direct shelf packet when the numbered workflow needs "
            "basic IPC barrier evidence."
        ),
        "inspect": [
            "box height",
            "barrier gap",
            "contact count",
            "step timing",
            "barrier-held status",
        ],
        "healthy_signal": (
            "Barrier gap stays nonnegative while speed settles and timing "
            "remains finite."
        ),
        "scope": (
            "Direct Rigid IPC shelf row; not a numbered World Rigid Body "
            "workflow row."
        ),
    },
    "rigid_ipc_slide": {
        "workflow_label": "Rigid IPC shelf",
        "user_question": "How does rigid IPC friction brake tangential slide?",
        "try_first": (
            "Use this direct shelf row when the question is IPC friction "
            "rather than broad solver-family comparison."
        ),
        "inspect": [
            "tangential travel",
            "box speed",
            "barrier gap",
            "contact count",
            "friction-braked status",
        ],
        "healthy_signal": (
            "Tangential speed decays while the contact gap stays finite."
        ),
        "scope": (
            "Direct Rigid IPC shelf row; not a numbered World Rigid Body "
            "workflow row."
        ),
    },
    "rigid_ipc_incline": {
        "workflow_label": "Rigid IPC shelf",
        "user_question": (
            "Can rigid IPC keep contact while a box slides down an inclined ramp?"
        ),
        "try_first": (
            "Use this direct shelf row for tilted-ramp IPC contact behavior "
            "before changing solver families."
        ),
        "inspect": [
            "down-slope speed",
            "down-slope travel",
            "ramp gap",
            "contact count",
            "step timing",
        ],
        "healthy_signal": (
            "Ramp gap stays finite while downslope motion and timing remain "
            "observable."
        ),
        "scope": (
            "Direct Rigid IPC shelf row; not a numbered World Rigid Body "
            "workflow row."
        ),
    },
    "rigid_ipc_pile": {
        "workflow_label": "Rigid IPC shelf",
        "user_question": (
            "Can direct Rigid IPC captures show multi-box pile "
            "gap/contact/timing evidence?"
        ),
        "try_first": (
            "Use this direct shelf row for a live-budget IPC pile before "
            "moving to capture-first stack packets."
        ),
        "inspect": [
            "box count",
            "minimum clearance",
            "maximum speed",
            "contact count",
            "step timing",
        ],
        "healthy_signal": (
            "Pile clearances remain finite and the live-budget timing is "
            "visible in the manifest."
        ),
        "scope": (
            "Direct Rigid IPC shelf row; not a numbered World Rigid Body "
            "workflow row or benchmark packet."
        ),
    },
}

_RIGID_WORKFLOW_PACKET_GUIDANCE_BY_SCENE: dict[str, dict[str, object]] = {
    "rigid_ipc_stack_packet": {
        "workflow_label": "Capture-first packet",
        "user_question": (
            "Can a four-box IPC stack stay separated, ordered, and finite "
            "beyond the live demo budget?"
        ),
        "try_first": (
            "Use this packet when the live workflow should cite heavier IPC "
            "stack evidence without making a performance parity claim."
        ),
        "inspect": [
            "friction",
            "box count",
            "frame-budget threshold",
            "min clearance",
            "contact count",
            "top drift",
            "height error",
            "max speed",
            "wall time",
            "benchmark pointer",
        ],
        "healthy_signal": (
            "The stack remains separated, ordered, and finite, with benchmark "
            "ownership recorded for throughput context."
        ),
        "scope": (
            "Capture-first stress packet; not a numbered workflow row and not "
            "a solver-performance parity claim."
        ),
    },
    "rigid_ipc_heavy_stack_packet": {
        "workflow_label": "Capture-first packet",
        "user_question": (
            "How does a taller, top-heavy IPC stack behave beyond the live demo "
            "budget?"
        ),
        "try_first": (
            "Use this packet when four boxes are not enough to inspect IPC "
            "stack stress, but keep it out of the live numbered workflow."
        ),
        "inspect": [
            "friction",
            "box count",
            "top mass",
            "frame-budget threshold",
            "min clearance",
            "contact count",
            "top drift",
            "height error",
            "max speed",
            "wall time",
            "benchmark pointer",
        ],
        "healthy_signal": (
            "The taller mass-gradient stack stays finite and separated enough "
            "for capture evidence, with benchmark ownership recorded for "
            "throughput context."
        ),
        "scope": (
            "Taller capture-first stress packet; not a numbered workflow row "
            "and not a solver-performance parity claim."
        ),
    },
}


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
    guidance: dict[str, dict[str, object]] = {}
    runner_module_name = "_dart_capture_py_demo_runner_guidance"
    runner_path = _repo_root() / "python" / "examples" / "demos" / "runner.py"
    try:
        spec = importlib.util.spec_from_file_location(runner_module_name, runner_path)
        if spec is None or spec.loader is None:
            raise ImportError(f"could not load {runner_path}")
        runner_module = importlib.util.module_from_spec(spec)
        sys.modules[runner_module_name] = runner_module
        spec.loader.exec_module(runner_module)
        RIGID_VISUAL_WORKFLOW_GUIDES = getattr(
            runner_module, "RIGID_VISUAL_WORKFLOW_GUIDES", {}
        )
        rigid_workflow_related_evidence_by_scene = getattr(
            runner_module, "rigid_workflow_related_evidence_by_scene", None
        )
    except Exception:
        sys.modules.pop(runner_module_name, None)
        RIGID_VISUAL_WORKFLOW_GUIDES = {}
        rigid_workflow_related_evidence_by_scene = None

    for scene_id, guide in RIGID_VISUAL_WORKFLOW_GUIDES.items():
        guidance[str(scene_id)] = {
            "workflow_index": guide.index,
            "workflow_count": guide.count,
            "workflow_label": guide.label,
            "user_question": guide.question,
            "workflow_phase": guide.workflow_phase,
            "focus_axis": guide.focus_axis,
            "try_first": guide.try_first,
            "inspect": list(guide.inspect),
            "healthy_signal": guide.healthy_signal,
            "scope": guide.scope,
        }
        if guide.deferred_api_caveats:
            guidance[str(scene_id)]["deferred_api_caveats"] = list(
                guide.deferred_api_caveats
            )
    if rigid_workflow_related_evidence_by_scene is not None:
        for source_scene, entries in rigid_workflow_related_evidence_by_scene().items():
            source_guide = RIGID_VISUAL_WORKFLOW_GUIDES.get(source_scene)
            if source_guide is None:
                source_label = str(source_scene)
            else:
                source_label = (
                    f"{source_guide.index:02d}/{source_guide.count:02d} "
                    f"{source_guide.label}"
                )
            for entry in entries:
                guidance[str(entry.scene_id)] = {
                    "workflow_label": "Related evidence",
                    "user_question": entry.label,
                    "try_first": (
                        "Open this row from "
                        f"{source_label} when the numbered workflow points to "
                        "the related shelf."
                    ),
                    "inspect": [entry.shelf, entry.reason],
                    "healthy_signal": (
                        "Related route remains scoped and metric-backed; use "
                        "the numbered source row for the main workflow claim."
                    ),
                    "scope": entry.reason,
                    "related_source_row": str(source_scene),
                    "related_source_label": source_label,
                    "related_shelf": entry.shelf,
                }
    guidance.update(_RIGID_WORKFLOW_IPC_SHELF_GUIDANCE_BY_SCENE)
    guidance.update(_RIGID_WORKFLOW_PACKET_GUIDANCE_BY_SCENE)
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


_SOLVER_IDENTITY_KEYS = (
    "solver",
    "solver_pair",
    "contact_solver_method",
    "same_solver",
    "executor",
    "executor_pair",
    "held_fixed",
)


_SOLVER_FAMILY_IDENTITY_KEYS = ("solver", "solver_pair")
_SOLVER_CONTEXT_IDENTITY_KEYS = (
    "contact_solver_method",
    "same_solver",
    "executor",
    "executor_pair",
    "held_fixed",
)


def _json_like_value(value: object) -> object | None:
    if value is None or isinstance(value, str | bool | int | float):
        return value
    if isinstance(value, list):
        converted = [_json_like_value(item) for item in value]
        return converted if all(item is not None for item in converted) else None
    if isinstance(value, dict):
        converted_dict: dict[str, object] = {}
        for key, item in value.items():
            if not isinstance(key, str):
                continue
            converted_item = _json_like_value(item)
            if converted_item is not None:
                converted_dict[key] = converted_item
        return converted_dict or None
    return None


def _valid_resolved_solver_identity(
    identity: dict[str, object],
) -> dict[str, object] | None:
    has_solver_family = any(key in identity for key in _SOLVER_FAMILY_IDENTITY_KEYS)
    has_context = any(key in identity for key in _SOLVER_CONTEXT_IDENTITY_KEYS)
    if not has_solver_family or not has_context:
        return None
    return identity


def _resolved_solver_identity_from_metrics(
    metrics: dict[str, object],
) -> dict[str, object] | None:
    identity: dict[str, object] = {}
    for key in _SOLVER_IDENTITY_KEYS:
        if key not in metrics:
            continue
        value = _json_like_value(metrics[key])
        if value is not None:
            identity[key] = value
    identity["source"] = "scene_capture_metrics.latest.metrics"
    return _valid_resolved_solver_identity(identity)


def _read_scene_resolved_solver_identity(
    manifest_path: pathlib.Path,
) -> dict[str, object] | None:
    if not manifest_path.is_file():
        return None
    try:
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict):
        return None
    identity = payload.get("resolved_solver_identity")
    if isinstance(identity, dict):
        filtered = {
            key: value for key, value in identity.items() if isinstance(key, str)
        }
        return _valid_resolved_solver_identity(filtered)
    scene_metrics = payload.get("scene_metrics")
    if isinstance(scene_metrics, dict):
        latest = scene_metrics.get("latest")
        if isinstance(latest, dict):
            metrics = latest.get("metrics")
            if isinstance(metrics, dict):
                return _resolved_solver_identity_from_metrics(metrics)
    return None


def _read_scene_metrics_evidence(
    manifest_path: pathlib.Path,
) -> dict[str, object] | None:
    if not manifest_path.is_file():
        return None
    try:
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict):
        return None
    scene_metrics = payload.get("scene_metrics")
    if not isinstance(scene_metrics, dict):
        return None
    latest = scene_metrics.get("latest")
    if not isinstance(latest, dict):
        return None
    metrics = latest.get("metrics")
    if not isinstance(metrics, dict) or not metrics:
        return None

    evidence: dict[str, object] = {
        "latest_metric_count": len(metrics),
    }
    event_count = scene_metrics.get("event_count")
    if isinstance(event_count, int):
        evidence["event_count"] = event_count
    metric_key_counts = scene_metrics.get("metric_key_counts")
    if isinstance(metric_key_counts, dict):
        evidence["metric_key_count"] = len(metric_key_counts)
    return evidence


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
    latest_metrics: dict[str, object] | None = None
    if isinstance(latest, dict):
        metrics = latest.get("metrics")
        if isinstance(metrics, dict):
            latest_metrics = metrics
    summary: dict[str, object] = {
        "event_count": event_count,
        "first": first,
        "latest": latest,
        "metric_key_counts": dict(sorted(metric_key_counts.items())),
        "numeric_ranges": dict(sorted(numeric_ranges.items())),
    }
    if latest_metrics is not None:
        resolved_solver_identity = _resolved_solver_identity_from_metrics(
            latest_metrics
        )
        if resolved_solver_identity is not None:
            summary["resolved_solver_identity"] = resolved_solver_identity
    return summary


def _read_scene_metadata(path: pathlib.Path) -> dict[str, object] | None:
    if not path.is_file():
        return None

    latest_metadata: dict[str, object] | None = None
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
            if payload.get("event") != "scene_capture_metadata":
                continue
            metadata = payload.get("metadata")
            if isinstance(metadata, dict):
                latest_metadata = metadata
    return latest_metadata


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
        "--continue-on-failure",
        action="store_true",
        help=(
            "With --rigid-workflow, keep capturing later rows after a row "
            "fails. The final workflow manifest and exit code still report "
            "failure when any row fails."
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
    if args.video:
        scene_argv.extend(["--video", "--fps", str(args.fps)])
    return scene_argv


def _workflow_rerun_output_dir(
    output_dir: pathlib.Path, order: int, scene: str
) -> pathlib.Path:
    return output_dir / "reruns" / f"{order:02d}_{scene}"


def _workflow_row_rerun_argv(
    args: argparse.Namespace,
    order: int,
    scene: str,
    output_dir: pathlib.Path,
) -> list[str]:
    rerun_argv = ["--rigid-workflow"]
    if getattr(args, "include_related", False):
        rerun_argv.append("--include-related")
    if getattr(args, "include_ipc_shelf", False):
        rerun_argv.append("--include-ipc-shelf")
    if getattr(args, "include_packets", False):
        rerun_argv.append("--include-packets")
    if getattr(args, "continue_on_failure", False):
        rerun_argv.append("--continue-on-failure")
    rerun_argv.extend(
        [
            "--workflow-start-row",
            str(order),
            "--workflow-end-row",
            str(order),
        ]
    )
    if args.backend:
        rerun_argv.extend(["--backend", args.backend])
    if args.allow_noop:
        rerun_argv.append("--allow-noop")
    if args.video:
        rerun_argv.extend(["--video", "--fps", str(args.fps)])
    rerun_argv.extend(
        [
            "--output-dir",
            str(_workflow_rerun_output_dir(output_dir, order, scene)),
        ]
    )
    return rerun_argv


def _workflow_command_argv(
    args: argparse.Namespace, output_dir: pathlib.Path
) -> list[str]:
    workflow_argv = ["--rigid-workflow"]
    if getattr(args, "include_related", False):
        workflow_argv.append("--include-related")
    if getattr(args, "include_ipc_shelf", False):
        workflow_argv.append("--include-ipc-shelf")
    if getattr(args, "include_packets", False):
        workflow_argv.append("--include-packets")
    if getattr(args, "continue_on_failure", False):
        workflow_argv.append("--continue-on-failure")
    start = getattr(args, "workflow_start_row", None)
    if start is not None:
        workflow_argv.extend(["--workflow-start-row", str(start)])
    end = getattr(args, "workflow_end_row", None)
    if end is not None:
        workflow_argv.extend(["--workflow-end-row", str(end)])
    if args.dry_run:
        workflow_argv.append("--dry-run")
    if args.backend:
        workflow_argv.extend(["--backend", args.backend])
    if args.allow_noop:
        workflow_argv.append("--allow-noop")
    if args.video:
        workflow_argv.extend(["--video", "--fps", str(args.fps)])
    workflow_argv.extend(["--output-dir", str(output_dir)])
    return workflow_argv


def _public_command(argv: list[str]) -> str:
    return "pixi run py-demo-capture -- " + " ".join(shlex.quote(arg) for arg in argv)


def _viewer_command(scene: str, width: int, height: int, backend: str = "") -> str:
    argv = ["--scene", scene, "--width", str(width), "--height", str(height)]
    if backend:
        argv.extend(["--backend", backend])
    return "pixi run py-demos -- " + " ".join(shlex.quote(arg) for arg in argv)


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
                "workflow_rerun_command": _public_command(
                    _workflow_row_rerun_argv(args, order, scene, output_dir)
                ),
                "viewer_command": _viewer_command(scene, width, height, args.backend),
                "status": "planned",
            }
        )
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


def _workflow_manifest_artifact_path(
    artifact: str, manifest_path: pathlib.Path, output_dir: pathlib.Path
) -> pathlib.Path:
    artifact_path = pathlib.Path(artifact)
    if artifact_path.is_absolute():
        return artifact_path
    manifest_relative = manifest_path.parent / artifact_path
    if manifest_relative.exists():
        return manifest_relative
    output_relative = output_dir / artifact_path
    if output_relative.exists():
        return output_relative
    if artifact_path.exists():
        return artifact_path
    return manifest_relative


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
            screenshot_path = _workflow_manifest_artifact_path(
                screenshot, manifest_path, output_dir
            )
            summary["screenshot_href"] = _workflow_href(screenshot_path, output_dir)
        frames = artifacts.get("frames")
        if isinstance(frames, str) and frames:
            frames_path = _workflow_manifest_artifact_path(
                frames, manifest_path, output_dir
            )
            summary["frames_href"] = _workflow_href(frames_path, output_dir)
        video = artifacts.get("video")
        if isinstance(video, str) and video:
            video_path = _workflow_manifest_artifact_path(
                video, manifest_path, output_dir
            )
            summary["video_href"] = _workflow_href(video_path, output_dir)

    top_level_identity = payload.get("resolved_solver_identity")
    if isinstance(top_level_identity, dict):
        summary["resolved_solver_identity"] = {
            key: value
            for key, value in top_level_identity.items()
            if isinstance(key, str)
        }

    scene_metrics = payload.get("scene_metrics")
    if isinstance(scene_metrics, dict):
        resolved_solver_identity = scene_metrics.get("resolved_solver_identity")
        if isinstance(resolved_solver_identity, dict):
            summary["resolved_solver_identity"] = {
                key: value
                for key, value in resolved_solver_identity.items()
                if isinstance(key, str)
            }
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
                    held_fixed_values = _workflow_metric_key_values(held_fixed)
                    if held_fixed_values:
                        summary["held_fixed_values"] = held_fixed_values
                controls = metrics.get("controls")
                if isinstance(controls, dict):
                    control_values = _workflow_metric_key_values(controls)
                    if control_values:
                        summary["control_values"] = control_values
                backend_diagnostics = _workflow_backend_diagnostics(metrics)
                if backend_diagnostics:
                    summary["backend_diagnostics"] = backend_diagnostics
                metric_highlights = _workflow_metric_highlights(metrics)
                if metric_highlights:
                    summary["metric_highlights"] = metric_highlights
                if "resolved_solver_identity" not in summary:
                    resolved_solver_identity = _resolved_solver_identity_from_metrics(
                        metrics
                    )
                    if resolved_solver_identity is not None:
                        summary["resolved_solver_identity"] = resolved_solver_identity

    scene_metadata = payload.get("scene_metadata")
    if isinstance(scene_metadata, dict):
        replay_timeline = scene_metadata.get("replay_timeline")
        if isinstance(replay_timeline, dict):
            label = replay_timeline.get("signal_label", replay_timeline.get("label"))
            if isinstance(label, str) and label:
                summary["replay_timeline_label"] = label
            summary["replay_timeline_has_signal"] = bool(
                replay_timeline.get("has_signal")
            )
            summary["replay_timeline_has_markers"] = bool(
                replay_timeline.get("has_markers")
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


_WORKFLOW_METRIC_HIGHLIGHT_KEYS = (
    "baseline_max_speed",
    "baseline_min_height",
    "baseline_energy",
    "baseline_scene_contact_count",
    "baseline_step_ms",
    "dynamic_body_count",
    "drift_position_error",
    "drift_momentum_drift",
    "drift_speed",
    "arc_position_error",
    "arc_momentum_residual",
    "arc_energy_drift",
    "arc_height",
    "spin_momentum_ratio",
    "spin_energy_ratio",
    "dynamic_displacement_x",
    "dynamic_height",
    "dynamic_speed",
    "kinematic_error",
    "kinematic_x",
    "world_error",
    "relative_error",
    "orientation_error",
    "sensor_x",
    "sensor_y",
    "sensor_z",
    "parent",
    "light_force_accel_x",
    "heavy_force_accel_x",
    "linear_impulse_momentum_x",
    "linear_impulse_speed",
    "low_inertia_angular_accel_z",
    "high_inertia_angular_accel_z",
    "angular_impulse_momentum_z",
    "angular_impulse_speed",
    "static_drift",
    "center_world_accel_x",
    "double_world_accel_x",
    "local_frame_world_accel_y",
    "offcenter_yaw_accel",
    "pulse_applied_count",
    "base_time_step_ms",
    "fine_freefall_error",
    "medium_freefall_error",
    "coarse_freefall_error",
    "coarse_error_ratio",
    "fine_clearance",
    "coarse_clearance",
    "budget_ms",
    "dense_single_ratio",
    "restitution_scale",
    "dead_rebound_height",
    "middle_rebound_height",
    "high_rebound_height",
    "high_upward_velocity",
    "dead_contact_count",
    "middle_contact_count",
    "high_contact_count",
    "effective_restitution",
    "expected_restitution_rule",
    "effective_friction",
    "expected_friction_rule",
    "bounce_body_high_rebound",
    "bounce_surface_high_rebound",
    "slide_body_high_speed_loss",
    "slide_surface_high_speed_loss",
    "target_travel_divergence",
    "si_caveat_target_travel",
    "ipc_normal_max_depth",
    "travel_divergence",
    "sequential_impulse_target_travel",
    "ipc_target_travel",
    "sequential_impulse_max_contact_count",
    "ipc_min_gap",
    "ipc_grip_box_travel",
    "ipc_grip_speed_ratio",
    "ipc_slip_slip",
    "si_caveat_driver_travel",
    "ipc_grip_min_abs_support_gap",
    "fixed_joint_translation_error",
    "fixed_joint_orientation_error",
    "fixed_joint_payload_speed",
    "fixed_joint_payload_angular_speed",
    "breakage_payload_release_distance",
    "breakage_broken",
    "breakage_captured_offset_error",
    "breakage_payload_speed",
    "breakage_status",
    "distance_spring_free_abs_stretch",
    "distance_spring_soft_abs_stretch",
    "distance_spring_stiff_abs_stretch",
    "distance_spring_offset_abs_stretch",
    "distance_spring_offset_angular_speed",
    "distance_spring_max_sprung_abs_stretch",
    "one_dof_hinge_radius_error",
    "one_dof_hinge_z_error",
    "one_dof_slider_orthogonal_error",
    "one_dof_hinge_yaw",
    "one_dof_slider_axis_travel",
    "one_dof_hinge_angular_speed",
    "one_dof_slider_axis_speed",
    "joint_motor_speed",
    "joint_motor_expected_speed",
    "joint_motor_speed_error",
    "joint_motor_position_limit_angle",
    "joint_motor_position_limit_error",
    "joint_motor_force_position_gap",
    "joint_motor_force_acceleration_gap",
    "passive_joint_spring_energy",
    "passive_joint_damped_energy",
    "passive_joint_damped_energy_ratio",
    "passive_joint_slip_speed",
    "passive_joint_armature_position_gap",
    "passive_joint_armature_acceleration_gap",
    "screw_joint_zero_pitch_axial_travel",
    "screw_joint_fine_pitch",
    "screw_joint_coarse_pitch",
    "screw_joint_reverse_pitch",
    "screw_joint_coarse_fine_travel_gap",
    "screw_joint_reverse_angle",
    "screw_joint_fine_acceleration_error",
    "multibody_dynamics_single_mass_diag0",
    "multibody_dynamics_coupled_coupling",
    "multibody_dynamics_heavy_tau_gap",
    "multibody_dynamics_coupled_heavy_response_gap",
    "multibody_dynamics_heavy_response_ratio",
    "multibody_dynamics_max_inverse_residual",
    "multibody_dynamics_max_impulse_residual",
    "link_com_centered_gravity_torque",
    "link_com_positive_gravity_torque",
    "link_com_negative_gravity_torque",
    "link_com_positive_negative_angle_sum",
    "link_com_high_mass_matrix_ratio",
    "link_com_high_acceleration_ratio",
    "link_com_max_acceleration_error",
    "link_jacobian_linear_speed",
    "link_jacobian_angular_speed",
    "link_jacobian_world_body_gap",
    "link_jacobian_finite_difference_error",
    "link_jacobian_tau0",
    "link_jacobian_tau1",
    "link_jacobian_power_error",
    "multibody_solver_residual_only_residual",
    "multibody_solver_solved_residual",
    "multibody_solver_residual_solve_ratio",
    "multibody_solver_semi_residual",
    "multibody_solver_variational_residual",
    "multibody_solver_solved_tip_error",
    "multibody_solver_max_step_ms",
    "loop_closure_point_residual_ratio",
    "loop_closure_distance_residual_ratio",
    "loop_closure_rigid_residual_ratio",
    "loop_closure_distance_solved_distance_error",
    "loop_closure_distance_solved_tip_error",
    "loop_closure_rigid_residual_orientation_error",
    "loop_closure_rigid_solved_orientation_error",
    "selected_contact_count",
    "total_contact_count",
    "selected_max_depth",
    "first_pair",
    "first_shape_indices",
    "max_depth",
    "active_contact_count",
    "baseline_contact_count",
    "filtered_contact_count",
    "ignored_contact_count",
    "ignored_pair_key",
    "option_contact_count",
    "option_filtered_contact_count",
    "ray_hit_count",
    "ray_first_fraction",
    "sphere_hit_count",
    "sphere_first_toi",
    "capsule_hit_count",
    "capsule_first_toi",
    "sphere_margin",
    "capsule_margin",
    "linear_speed",
    "angular_speed",
    "body_x",
    "body_y",
    "body_z",
    "spin_command",
    "dofs",
    "link_count",
    "max_joint_speed",
    "forearm_height",
    "shoulder_speed",
    "wrist_speed",
    "shoulder_damping",
    "wrist_damping",
    "wrist_position_norm",
    "shoulder_position",
    "min_tunnel_margin",
    "max_wall_crossing",
    "capture_first",
    "frame_budget_ms",
    "box_count",
    "min_clearance",
    "clearance",
    "top_drift",
    "height_error",
    "max_speed",
    "max_contact_count",
    "contact_count",
    "max_step_ms",
    "step_ms",
    "tunnel_margin",
    "launch_speed",
    "min_barrier_gap",
    "tilt_deg",
    "max_tilt_deg",
    "vertical_speed",
    "box_height",
    "box_speed",
    "box_vx",
    "box_x",
    "initial_speed",
    "travel",
    "max_travel",
    "min_speed",
    "down_slope_speed",
    "down_slope_travel",
    "max_down_slope_speed",
    "max_down_slope_travel",
    "min_ramp_gap",
    "ramp_gap",
    "mean_height",
    "min_mean_height",
    "span_x",
    "max_span_x",
    "min_history_clearance",
    "max_history_speed",
    "top_mass",
    "benchmark",
    "anchor_offset_error",
    "break_force",
    "broken",
    "orientation_drift",
    "payload_height",
    "payload_speed",
    "reset_break_force",
    "target_speed",
    "measured_speed",
    "abs_speed_error",
    "speed_error",
    "axis_position",
    "orthogonal_drift",
    "max_torque",
    "max_force",
    "actuator",
    "aware_final_z",
    "aware_target_error",
    "aware_loss",
    "aware_thrust",
    "aware_last_thrust_gradient",
    "naive_final_z",
    "naive_target_error",
    "naive_loss",
    "target_error_gap",
    "thrust_gap",
    "height_gap",
    "surrogate_next_z",
    "surrogate_dvzprime_dvz",
    "surrogate_block_magnitude",
    "surrogate_delta_dvzprime_dvz",
    "post_step_clearance",
    "forward_state_max_abs_diff",
    "thresholds_pass",
    "differentiable_available",
    "contact_solver_method",
    "gradient_modes",
    "status",
    "friction",
    "solver_pair",
    "executor_pair",
    "contact_policy_pair",
    "case_pair",
    "solver_label",
    "solver",
    "executor",
    "contact_policy",
    "same_solver",
    "position_divergence",
    "velocity_divergence",
    "contact_delta",
    "sequential_contact_count",
    "parallel_contact_count",
    "sequential_step_ms",
    "parallel_step_ms",
    "below_distance",
    "above_distance",
    "controlled_threshold_delta",
    "matched_roll_contact_slip",
    "slide_to_roll_spin_delta",
    "low_friction_slip_contact_slip",
    "top_x_divergence",
    "sequential_impulse_min_clearance",
    "ipc_min_clearance",
    "ipc_top_drift",
    "ipc_normal_target_travel",
    "ipc_heavy_max_depth",
    "si_caveat_contact_count",
)

_WORKFLOW_NESTED_METRIC_HIGHLIGHT_KEYS = {
    "divergence": (
        "current_x",
        "max_x",
        "current_pose",
        "max_pose",
        "samples",
    ),
    "history": (
        "max_position_divergence",
        "max_velocity_divergence",
        "max_contact_delta",
        "max_sequential_step_ms",
        "max_parallel_step_ms",
        "samples",
    ),
}


def _workflow_metric_label(key: str) -> str:
    return key.replace("_", " ")


def _workflow_metric_value(value: object) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int | float):
        return f"{float(value):.6g}"
    if isinstance(value, str):
        return value
    if isinstance(value, list | tuple):
        return " / ".join(_workflow_metric_value(item) for item in value[:4])
    return str(value)


def _workflow_metric_key_values(values: dict[object, object]) -> list[str]:
    pairs: list[str] = []
    for key, value in sorted(values.items(), key=lambda item: str(item[0])):
        if not isinstance(key, str):
            continue
        pairs.append(f"{_workflow_metric_label(key)}={_workflow_metric_value(value)}")
    return pairs[:8]


_WORKFLOW_BACKEND_DIAGNOSTIC_LANE_ORDER = (
    "single",
    "contact",
    "stack",
)


def _workflow_numeric_metric(value: object) -> int | float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int | float):
        return value
    return None


def _workflow_lane_backend_diagnostics(label: str, metrics: object) -> str | None:
    if not isinstance(metrics, dict):
        return None
    if not any(
        key in metrics
        for key in (
            "profile_status",
            "accelerated_backend_active",
            "accelerated_stage_count",
            "max_workers",
            "top_stage",
            "stage_ms",
        )
    ):
        return None

    parts: list[str] = []
    profile_status = metrics.get("profile_status")
    if isinstance(profile_status, str) and profile_status:
        parts.append(profile_status)

    accelerated = metrics.get("accelerated_backend_active")
    if isinstance(accelerated, bool):
        parts.append(f"backend {'active' if accelerated else 'inactive'}")
    accelerated_stage_count = _workflow_numeric_metric(
        metrics.get("accelerated_stage_count")
    )
    if accelerated_stage_count is not None:
        parts.append(
            f"{_workflow_metric_value(accelerated_stage_count)} accelerated stages"
        )

    max_workers = _workflow_numeric_metric(metrics.get("max_workers"))
    if max_workers is not None:
        parts.append(f"workers {_workflow_metric_value(max_workers)}")

    top_stage = metrics.get("top_stage")
    top_stage_ms = _workflow_numeric_metric(metrics.get("top_stage_ms"))
    if isinstance(top_stage, str) and top_stage and top_stage != "none":
        top_stage_text = f"top {top_stage}"
        if top_stage_ms is not None:
            top_stage_text = (
                f"{top_stage_text} {_workflow_metric_value(top_stage_ms)} ms"
            )
        parts.append(top_stage_text)

    stage_ms = _workflow_numeric_metric(metrics.get("stage_ms"))
    if stage_ms is not None:
        parts.append(f"stage {_workflow_metric_value(stage_ms)} ms")

    if not parts:
        return None
    return f"{label}: {', '.join(parts[:6])}"


def _workflow_backend_diagnostics(metrics: dict[str, object]) -> list[str]:
    lanes = metrics.get("lanes")
    if not isinstance(lanes, dict):
        return []
    ordered_keys = [
        key for key in _WORKFLOW_BACKEND_DIAGNOSTIC_LANE_ORDER if key in lanes
    ]
    ordered_keys.extend(
        sorted(
            key
            for key in lanes
            if isinstance(key, str)
            and key not in _WORKFLOW_BACKEND_DIAGNOSTIC_LANE_ORDER
        )
    )
    diagnostics: list[str] = []
    for key in ordered_keys:
        if not isinstance(key, str):
            continue
        diagnostic = _workflow_lane_backend_diagnostics(key, lanes.get(key))
        if diagnostic is not None:
            diagnostics.append(diagnostic)
    return diagnostics[:4]


def _workflow_nested_metric_highlight(label: str, value: object) -> str | None:
    if not isinstance(value, dict):
        return None
    keys = _WORKFLOW_NESTED_METRIC_HIGHLIGHT_KEYS.get(label, ())
    parts = [
        f"{_workflow_metric_label(key)}={_workflow_metric_value(value[key])}"
        for key in keys
        if key in value
    ]
    if not parts:
        return None
    return f"{_workflow_metric_label(label)}: {', '.join(parts[:5])}"


def _workflow_metric_highlights(metrics: dict[str, object]) -> list[str]:
    highlights = [
        f"{_workflow_metric_label(key)}: {_workflow_metric_value(metrics[key])}"
        for key in _WORKFLOW_METRIC_HIGHLIGHT_KEYS
        if key in metrics
    ]
    for key in _WORKFLOW_NESTED_METRIC_HIGHLIGHT_KEYS:
        highlight = _workflow_nested_metric_highlight(key, metrics.get(key))
        if highlight is not None:
            highlights.append(highlight)
    return highlights[:8]


_WORKFLOW_GROUP_LABELS = {
    "numbered": "numbered",
    "related_evidence": "related",
    "rigid_ipc_shelf": "ipc shelf",
    "capture_first_packet": "packets",
}

_WORKFLOW_GUIDANCE_REQUIRED_FIELDS = (
    "workflow_label",
    "user_question",
    "try_first",
    "inspect",
    "healthy_signal",
    "scope",
)
_WORKFLOW_NUMBERED_GUIDANCE_REQUIRED_FIELDS = (
    "workflow_phase",
    "focus_axis",
)


def _workflow_group_labels(captures: list[dict[str, object]]) -> str:
    labels: list[str] = []
    for capture in captures:
        group = capture.get("workflow_group")
        if not isinstance(group, str):
            continue
        label = _WORKFLOW_GROUP_LABELS.get(group, group)
        if label not in labels:
            labels.append(label)
    return ", ".join(labels) if labels else "none"


def _workflow_requested_group_labels(
    requested_include_flags: dict[str, bool] | None,
    *,
    fallback: str,
) -> str:
    if requested_include_flags is None:
        return fallback
    labels = ["numbered"]
    if requested_include_flags.get("include_related", False):
        labels.append("related")
    if requested_include_flags.get("include_ipc_shelf", False):
        labels.append("ipc shelf")
    if requested_include_flags.get("include_packets", False):
        labels.append("packets")
    return ", ".join(labels)


def _workflow_guidance_missing(
    captures: list[dict[str, object]],
) -> list[dict[str, object]]:
    missing: list[dict[str, object]] = []
    for capture in captures:
        missing_fields: list[str] = []
        required_fields = list(_WORKFLOW_GUIDANCE_REQUIRED_FIELDS)
        if capture.get("workflow_group") == "numbered":
            required_fields.extend(_WORKFLOW_NUMBERED_GUIDANCE_REQUIRED_FIELDS)
        for field in required_fields:
            value = capture.get(field)
            if field == "inspect":
                if not isinstance(value, list) or not value:
                    missing_fields.append(field)
            elif not isinstance(value, str) or not value:
                missing_fields.append(field)
        if missing_fields:
            missing.append(
                {
                    "order": capture.get("order"),
                    "scene": capture.get("scene"),
                    "workflow_group": capture.get("workflow_group"),
                    "missing_fields": missing_fields,
                }
            )
    return missing


def _workflow_guidance_status(missing: list[dict[str, object]]) -> str:
    if not missing:
        return "complete"
    return f"missing {len(missing)}"


def _workflow_guidance_warning(missing: list[dict[str, object]]) -> str:
    if not missing:
        return ""
    items: list[str] = []
    for entry in missing:
        order = entry.get("order", "?")
        scene = entry.get("scene", "unknown")
        fields = entry.get("missing_fields", [])
        if isinstance(fields, list):
            field_text = ", ".join(str(field) for field in fields)
        else:
            field_text = str(fields)
        items.append(
            "<li>"
            f"{html.escape(str(order))} {html.escape(str(scene))}: "
            f"{html.escape(field_text)}"
            "</li>"
        )
    return (
        '<section class="guidance-warning">'
        "<h2>Rows Missing Guidance</h2>"
        "<p>These selected rows are missing self-description fields in the "
        "workflow manifest and should be fixed before relying on the packet "
        "as reviewer-facing evidence.</p>"
        f"<ul>{''.join(items)}</ul>"
        "</section>"
    )


def _workflow_failed_rows(
    captures: list[dict[str, object]],
) -> list[dict[str, object]]:
    failed_rows: list[dict[str, object]] = []
    for capture in captures:
        if capture.get("status") != "failed":
            continue
        failed_rows.append(
            {
                "order": capture.get("order"),
                "count": capture.get("count"),
                "scene": capture.get("scene"),
                "workflow_group": capture.get("workflow_group"),
                "workflow_label": capture.get("workflow_label"),
                "failure_reason": capture.get("failure_reason", "unknown"),
                "return_code": capture.get("return_code"),
                "manifest_exists": capture.get("manifest_exists"),
                "manifest": capture.get("manifest"),
                "command": capture.get("command"),
                "workflow_rerun_command": capture.get("workflow_rerun_command"),
            }
        )
    return failed_rows


def _attach_workflow_resolved_solver_identities(
    captures: list[dict[str, object]],
) -> None:
    for capture in captures:
        if capture.get("status") != "captured":
            capture.pop("resolved_solver_identity", None)
            continue
        manifest_value = capture.get("manifest")
        if not isinstance(manifest_value, str):
            capture.pop("resolved_solver_identity", None)
            continue
        identity = _read_scene_resolved_solver_identity(pathlib.Path(manifest_value))
        if identity is None:
            capture.pop("resolved_solver_identity", None)
        else:
            capture["resolved_solver_identity"] = identity


def _attach_workflow_scene_metrics_evidence(
    captures: list[dict[str, object]],
) -> None:
    for capture in captures:
        if capture.get("status") != "captured":
            capture.pop("scene_metrics_evidence", None)
            continue
        manifest_value = capture.get("manifest")
        if not isinstance(manifest_value, str):
            capture.pop("scene_metrics_evidence", None)
            continue
        evidence = _read_scene_metrics_evidence(pathlib.Path(manifest_value))
        if evidence is None:
            capture.pop("scene_metrics_evidence", None)
        else:
            capture["scene_metrics_evidence"] = evidence


def _workflow_missing_solver_identity_rows(
    captures: list[dict[str, object]], *, dry_run: bool
) -> list[dict[str, object]]:
    if dry_run:
        return []
    missing_rows: list[dict[str, object]] = []
    for capture in captures:
        if capture.get("status") != "captured":
            continue
        if isinstance(capture.get("resolved_solver_identity"), dict):
            continue
        missing_rows.append(
            {
                "order": capture.get("order"),
                "count": capture.get("count"),
                "scene": capture.get("scene"),
                "workflow_group": capture.get("workflow_group"),
                "workflow_label": capture.get("workflow_label"),
                "manifest": capture.get("manifest"),
            }
        )
    return missing_rows


def _workflow_missing_scene_metrics_rows(
    captures: list[dict[str, object]], *, dry_run: bool
) -> list[dict[str, object]]:
    if dry_run:
        return []
    missing_rows: list[dict[str, object]] = []
    for capture in captures:
        if capture.get("status") != "captured":
            continue
        if isinstance(capture.get("scene_metrics_evidence"), dict):
            continue
        missing_rows.append(
            {
                "order": capture.get("order"),
                "count": capture.get("count"),
                "scene": capture.get("scene"),
                "workflow_group": capture.get("workflow_group"),
                "workflow_label": capture.get("workflow_label"),
                "manifest": capture.get("manifest"),
            }
        )
    return missing_rows


def _workflow_solver_identity_status(
    missing: list[dict[str, object]], *, dry_run: bool
) -> str:
    if dry_run:
        return "not required"
    if not missing:
        return "complete"
    return f"missing {len(missing)}"


def _workflow_scene_metrics_status(
    missing: list[dict[str, object]], *, dry_run: bool
) -> str:
    if dry_run:
        return "not required"
    if not missing:
        return "complete"
    return f"missing {len(missing)}"


def _workflow_solver_identity_warning(missing: list[dict[str, object]]) -> str:
    if not missing:
        return ""
    items: list[str] = []
    for entry in missing:
        order = entry.get("order", "?")
        scene = entry.get("scene", "unknown")
        manifest = entry.get("manifest", "")
        manifest_html = ""
        if isinstance(manifest, str) and manifest:
            manifest_html = (
                '<p class="command-label">scene manifest</p>'
                f"<pre>{html.escape(manifest)}</pre>"
            )
        items.append(
            "<li>"
            f"{html.escape(str(order))} {html.escape(str(scene))}"
            f"{manifest_html}"
            "</li>"
        )
    return (
        '<section class="solver-identity-warning">'
        "<h2>Rows Missing Solver Identity</h2>"
        "<p>These captured rows did not publish a resolved solver identity in "
        "their scene manifest. Treat the packet as incomplete DART 7 harness "
        "evidence until each row records the solver/contact/executor "
        "configuration that actually ran.</p>"
        f"<ul>{''.join(items)}</ul>"
        "</section>"
    )


def _workflow_scene_metrics_warning(missing: list[dict[str, object]]) -> str:
    if not missing:
        return ""
    items: list[str] = []
    for entry in missing:
        order = entry.get("order", "?")
        scene = entry.get("scene", "unknown")
        manifest = entry.get("manifest", "")
        manifest_html = ""
        if isinstance(manifest, str) and manifest:
            manifest_html = (
                '<p class="command-label">scene manifest</p>'
                f"<pre>{html.escape(manifest)}</pre>"
            )
        items.append(
            "<li>"
            f"{html.escape(str(order))} {html.escape(str(scene))}"
            f"{manifest_html}"
            "</li>"
        )
    return (
        '<section class="scene-metrics-warning">'
        "<h2>Rows Missing Scene Metrics</h2>"
        "<p>These captured rows did not publish latest scene metrics in their "
        "scene manifest. Treat the packet as incomplete DART 7 harness "
        "evidence until each row records the runtime metrics that make the "
        "visual result reviewable.</p>"
        f"<ul>{''.join(items)}</ul>"
        "</section>"
    )


def _workflow_failure_summary(failed_rows: list[dict[str, object]]) -> str:
    if not failed_rows:
        return ""
    items: list[str] = []
    for entry in failed_rows:
        order = entry.get("order", "?")
        count = entry.get("count", "?")
        scene = entry.get("scene", "unknown")
        reason = entry.get("failure_reason", "unknown")
        return_code = entry.get("return_code")
        rc_text = f" return code {return_code}" if return_code is not None else ""
        workflow_command = entry.get("workflow_rerun_command")
        workflow_command_html = ""
        if isinstance(workflow_command, str) and workflow_command:
            workflow_command_html = (
                '<p class="command-label">rerun workflow row</p>'
                f"<pre>{html.escape(workflow_command)}</pre>"
            )
        command = entry.get("command")
        command_html = ""
        if isinstance(command, str) and command:
            command_html = (
                '<p class="command-label">capture scene directly</p>'
                f"<pre>{html.escape(command)}</pre>"
            )
        items.append(
            "<li>"
            f"<strong>{html.escape(str(order))}/{html.escape(str(count))} "
            f"{html.escape(str(scene))}</strong>: "
            f"{html.escape(str(reason))}{html.escape(rc_text)}"
            f"{workflow_command_html}"
            f"{command_html}"
            "</li>"
        )
    return (
        '<section class="failure-summary">'
        "<h2>Failed Rows</h2>"
        "<p>These selected rows failed during the workflow packet. Inspect or "
        "rerun them before relying on the packet as complete evidence.</p>"
        f"<ul>{''.join(items)}</ul>"
        "</section>"
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
    video_href = summary.get("video_href")
    if isinstance(video_href, str):
        links.append(_workflow_link("video", video_href))
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
    workflow_phase = capture.get("workflow_phase")
    if isinstance(workflow_phase, str) and workflow_phase:
        details.append(_workflow_detail("phase", workflow_phase))
    focus_axis = capture.get("focus_axis")
    if isinstance(focus_axis, str) and focus_axis:
        details.append(_workflow_detail("focus axis", focus_axis))
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
    deferred_api_caveats = capture.get("deferred_api_caveats")
    if isinstance(deferred_api_caveats, list) and deferred_api_caveats:
        details.append(
            _workflow_detail(
                "deferred API caveat",
                " / ".join(str(caveat) for caveat in deferred_api_caveats),
            )
        )
    return_code = capture.get("return_code")
    if return_code is not None:
        details.append(_workflow_detail("return code", return_code))
    failure_reason = capture.get("failure_reason")
    if failure_reason is not None:
        details.append(_workflow_detail("failure", failure_reason))
    comparison_axis = summary.get("comparison_axis")
    if isinstance(comparison_axis, str):
        details.append(_workflow_detail("axis", comparison_axis))
    resolved_solver_identity = summary.get("resolved_solver_identity")
    if isinstance(resolved_solver_identity, dict):
        identity_values = _workflow_metric_key_values(
            {
                key: value
                for key, value in resolved_solver_identity.items()
                if key != "source"
            }
        )
        if identity_values:
            details.append(
                _workflow_detail("resolved solver", ", ".join(identity_values[:6]))
            )
    held_fixed_values = summary.get("held_fixed_values")
    if isinstance(held_fixed_values, list) and held_fixed_values:
        details.append(_workflow_detail("held fixed", ", ".join(held_fixed_values)))
    control_values = summary.get("control_values")
    if isinstance(control_values, list) and control_values:
        shown = ", ".join(str(item) for item in control_values[:6])
        if len(control_values) > 6:
            shown = f"{shown}, +{len(control_values) - 6} more"
        details.append(_workflow_detail("controls", shown))
    backend_diagnostics = summary.get("backend_diagnostics")
    if isinstance(backend_diagnostics, list) and backend_diagnostics:
        shown = " / ".join(str(item) for item in backend_diagnostics[:4])
        details.append(_workflow_detail("backend diagnostics", shown))
    replay_timeline_label = summary.get("replay_timeline_label")
    if isinstance(replay_timeline_label, str):
        replay_tracks: list[str] = []
        if summary.get("replay_timeline_has_signal"):
            replay_tracks.append("signal")
        if summary.get("replay_timeline_has_markers"):
            replay_tracks.append("markers")
        replay_detail = replay_timeline_label
        if replay_tracks:
            replay_detail = f"{replay_detail} ({', '.join(replay_tracks)})"
        details.append(_workflow_detail("replay", replay_detail))
    metric_keys = summary.get("metric_keys")
    if isinstance(metric_keys, list) and metric_keys:
        shown = ", ".join(str(key) for key in metric_keys[:8])
        if len(metric_keys) > 8:
            shown = f"{shown}, +{len(metric_keys) - 8} more"
        details.append(_workflow_detail("metrics", shown))
    metric_highlights = summary.get("metric_highlights")
    if isinstance(metric_highlights, list) and metric_highlights:
        shown = " / ".join(str(item) for item in metric_highlights[:6])
        if len(metric_highlights) > 6:
            shown = f"{shown} / +{len(metric_highlights) - 6} more"
        details.append(_workflow_detail("latest signals", shown))
    read_error = summary.get("read_error")
    if isinstance(read_error, str):
        details.append(_workflow_detail("manifest error", read_error))

    viewer_command = capture.get("viewer_command")
    command_blocks: list[str] = []
    if isinstance(viewer_command, str) and viewer_command:
        command_blocks.append(
            '<p class="command-label">open live</p>'
            f"<pre>{html.escape(viewer_command)}</pre>"
        )
    workflow_rerun_command = capture.get("workflow_rerun_command")
    if isinstance(workflow_rerun_command, str) and workflow_rerun_command:
        command_blocks.append(
            '<p class="command-label">rerun workflow row</p>'
            f"<pre>{html.escape(workflow_rerun_command)}</pre>"
        )
    command_blocks.append(
        '<p class="command-label">capture evidence</p>'
        f"<pre>{html.escape(command)}</pre>"
    )

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
    {''.join(command_blocks)}
  </div>
</article>
"""


def _workflow_row_span(rows: list[int]) -> str:
    if not rows:
        return "none"
    sorted_rows = sorted(set(rows))
    ranges: list[str] = []
    start = sorted_rows[0]
    previous = sorted_rows[0]
    for row in sorted_rows[1:]:
        if row == previous + 1:
            previous = row
            continue
        ranges.append(str(start) if start == previous else f"{start}-{previous}")
        start = previous = row
    ranges.append(str(start) if start == previous else f"{start}-{previous}")
    return ", ".join(ranges)


def _workflow_phase_summary(
    captures: list[dict[str, object]],
) -> list[dict[str, object]]:
    phases: dict[str, dict[str, object]] = {}
    for capture in captures:
        if capture.get("workflow_group") != "numbered":
            continue
        phase = capture.get("workflow_phase")
        if not isinstance(phase, str) or not phase:
            continue
        entry = phases.setdefault(
            phase,
            {
                "phase": phase,
                "row_count": 0,
                "rows": [],
                "row_span": "",
                "scenes": [],
                "focus_axes": [],
                "status_counts": {
                    "planned": 0,
                    "captured": 0,
                    "failed": 0,
                },
                "planned_count": 0,
                "captured_count": 0,
                "failed_count": 0,
            },
        )
        rows = entry["rows"]
        if isinstance(rows, list) and isinstance(capture.get("order"), int):
            rows.append(int(capture["order"]))
            entry["row_span"] = _workflow_row_span(rows)
        scenes = entry["scenes"]
        scene = capture.get("scene")
        if isinstance(scenes, list) and isinstance(scene, str):
            scenes.append(scene)
        focus_axes = entry["focus_axes"]
        focus_axis = capture.get("focus_axis")
        if (
            isinstance(focus_axes, list)
            and isinstance(focus_axis, str)
            and focus_axis
            and focus_axis not in focus_axes
        ):
            focus_axes.append(focus_axis)
        if isinstance(entry["row_count"], int):
            entry["row_count"] += 1
        status = capture.get("status")
        status_key = status if isinstance(status, str) and status else "unknown"
        status_counts = entry["status_counts"]
        if isinstance(status_counts, dict):
            status_counts[status_key] = int(status_counts.get(status_key, 0)) + 1
        count_key = f"{status_key}_count"
        if isinstance(entry.get(count_key), int):
            entry[count_key] = int(entry[count_key]) + 1
    return list(phases.values())


def _workflow_limited_list_text(values: object, *, limit: int = 5) -> str:
    if not isinstance(values, list):
        return ""
    string_values = [value for value in values if isinstance(value, str)]
    if len(string_values) > limit:
        return (
            ", ".join(string_values[:limit]) + f", +{len(string_values) - limit} more"
        )
    return ", ".join(string_values)


def _workflow_phase_status_text(entry: dict[str, object]) -> str:
    status_counts = entry.get("status_counts")
    if not isinstance(status_counts, dict):
        return ""
    parts: list[str] = []
    for status in ("captured", "failed", "planned"):
        count = status_counts.get(status)
        if isinstance(count, int) and count:
            parts.append(f"{status} {count}")
    extra_statuses = sorted(
        status
        for status, count in status_counts.items()
        if status not in {"captured", "failed", "planned"}
        and isinstance(status, str)
        and isinstance(count, int)
        and count
    )
    for status in extra_statuses:
        parts.append(f"{status} {status_counts[status]}")
    return ", ".join(parts) if parts else "none"


def _workflow_phase_map(summary: list[dict[str, object]]) -> str:
    if not summary:
        return ""

    rows: list[str] = []
    for entry in summary:
        phase = str(entry.get("phase", ""))
        row_span = str(entry.get("row_span", "none"))
        row_count = int(entry.get("row_count", 0))
        status_text = _workflow_phase_status_text(entry)
        focus_axis_text = _workflow_limited_list_text(entry.get("focus_axes"))
        scene_text = _workflow_limited_list_text(entry.get("scenes"))
        rows.append(
            "      <tr>"
            f"<td>{html.escape(row_span)}</td>"
            f"<td>{html.escape(phase)}</td>"
            f"<td>{row_count}</td>"
            f"<td>{html.escape(status_text)}</td>"
            f"<td>{html.escape(focus_axis_text)}</td>"
            f"<td>{html.escape(scene_text)}</td>"
            "</tr>"
        )

    return f"""
    <section class="phase-map">
      <h2>Workflow Phase Map</h2>
      <table>
        <thead>
          <tr><th>Rows</th><th>Phase</th><th>Count</th><th>Status</th><th>Focus axes</th><th>Selected scenes</th></tr>
        </thead>
        <tbody>
{chr(10).join(rows)}
        </tbody>
      </table>
    </section>"""


def _workflow_deferred_api_summary(
    captures: list[dict[str, object]],
) -> list[dict[str, object]]:
    summary: list[dict[str, object]] = []
    for capture in captures:
        caveats = capture.get("deferred_api_caveats")
        if not isinstance(caveats, list):
            continue
        caveat_values = [
            caveat for caveat in caveats if isinstance(caveat, str) and caveat
        ]
        if not caveat_values:
            continue
        row = capture.get("order")
        scene = capture.get("scene")
        workflow_label = capture.get("workflow_label")
        summary.append(
            {
                "row": int(row) if isinstance(row, int) else None,
                "scene": scene if isinstance(scene, str) else "",
                "workflow_label": (
                    workflow_label if isinstance(workflow_label, str) else ""
                ),
                "caveat_count": len(caveat_values),
                "caveats": caveat_values,
            }
        )
    return summary


def _workflow_deferred_api_map(summary: list[dict[str, object]]) -> str:
    if not summary:
        return ""

    rows: list[str] = []
    for entry in summary:
        row = entry.get("row")
        row_text = str(row) if isinstance(row, int) else "n/a"
        scene = str(entry.get("scene", ""))
        workflow_label = str(entry.get("workflow_label", ""))
        caveats = entry.get("caveats")
        caveat_text = _workflow_limited_list_text(caveats, limit=3)
        rows.append(
            "      <tr>"
            f"<td>{html.escape(row_text)}</td>"
            f"<td>{html.escape(scene)}</td>"
            f"<td>{html.escape(workflow_label)}</td>"
            f"<td>{html.escape(caveat_text)}</td>"
            "</tr>"
        )

    return f"""
    <section class="deferred-api-map">
      <h2>Deferred API Caveats</h2>
      <p>These selected rows are nearest current verifier rows for user queries
      whose public DART 7 API surface is still unavailable.</p>
      <table>
        <thead>
          <tr><th>Row</th><th>Scene</th><th>Route</th><th>Caveat</th></tr>
        </thead>
        <tbody>
{chr(10).join(rows)}
        </tbody>
      </table>
    </section>"""


def _write_workflow_review_index(
    output_dir: pathlib.Path,
    *,
    dry_run: bool,
    status: str,
    captures: list[dict[str, object]],
    started_at: float,
    requested_include_flags: dict[str, bool] | None = None,
    continue_on_failure: bool = False,
    workflow_command: str = "",
) -> pathlib.Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    index = _workflow_review_index_path(output_dir)
    completed = sum(1 for capture in captures if capture.get("status") == "captured")
    failed = sum(1 for capture in captures if capture.get("status") == "failed")
    elapsed_s = round(time.monotonic() - started_at, 3)
    cards = "\n".join(
        _workflow_review_card(capture, output_dir) for capture in captures
    )
    selected_groups = _workflow_group_labels(captures)
    requested_groups = _workflow_requested_group_labels(
        requested_include_flags,
        fallback=selected_groups,
    )
    guidance_missing = _workflow_guidance_missing(captures)
    guidance_warning = _workflow_guidance_warning(guidance_missing)
    solver_identity_missing = _workflow_missing_solver_identity_rows(
        captures, dry_run=dry_run
    )
    solver_identity_warning = _workflow_solver_identity_warning(solver_identity_missing)
    scene_metrics_missing = _workflow_missing_scene_metrics_rows(
        captures, dry_run=dry_run
    )
    scene_metrics_warning = _workflow_scene_metrics_warning(scene_metrics_missing)
    failed_rows = _workflow_failed_rows(captures)
    failure_summary = _workflow_failure_summary(failed_rows)
    phase_summary = _workflow_phase_summary(captures)
    phase_map = _workflow_phase_map(phase_summary)
    deferred_api_summary = _workflow_deferred_api_summary(captures)
    deferred_api_map = _workflow_deferred_api_map(deferred_api_summary)
    workflow_command_html = (
        "<p><strong>workflow command</strong><br>"
        f"<code>{html.escape(workflow_command)}</code></p>"
        if workflow_command
        else ""
    )
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
    row_span = (
        f"{min(capture_orders)}-{max(capture_orders)} / {workflow_total_count}"
        if capture_orders
        else f"none / {workflow_total_count}"
    )
    badges = " ".join(
        [
            _workflow_badge("status", status),
            _workflow_badge("captures", len(captures)),
            _workflow_badge("rows", row_span),
            _workflow_badge("requested groups", requested_groups),
            _workflow_badge("selected groups", selected_groups),
            _workflow_badge("phases", len(phase_summary)),
            _workflow_badge("deferred API caveats", len(deferred_api_summary)),
            _workflow_badge("guidance", _workflow_guidance_status(guidance_missing)),
            _workflow_badge(
                "solver identity",
                _workflow_solver_identity_status(
                    solver_identity_missing, dry_run=dry_run
                ),
            ),
            _workflow_badge(
                "scene metrics",
                _workflow_scene_metrics_status(scene_metrics_missing, dry_run=dry_run),
            ),
            _workflow_badge("complete", completed),
            _workflow_badge("failed", failed),
            _workflow_badge("elapsed s", elapsed_s),
            _workflow_badge("mode", "plan" if dry_run else "capture"),
            _workflow_badge(
                "failure mode",
                "continue" if continue_on_failure else "fail fast",
            ),
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
    .phase-map {{
      margin-top: 16px;
    }}
    .phase-map h2 {{
      margin: 0 0 8px;
    }}
    .phase-map table {{
      width: 100%;
      border-collapse: collapse;
      font-size: 13px;
    }}
    .phase-map th,
    .phase-map td {{
      padding: 6px 8px;
      border: 1px solid #d0d7de;
      text-align: left;
      vertical-align: top;
    }}
    .phase-map th {{
      background: #f6f8fa;
      color: #57606a;
      font-weight: 650;
    }}
    .guidance-warning {{
      margin: 0 0 16px;
      padding: 12px;
      border: 1px solid #f0b429;
      border-radius: 6px;
      background: #fff8c5;
    }}
    .solver-identity-warning {{
      margin: 0 0 16px;
      padding: 12px;
      border: 1px solid #f0b429;
      border-radius: 6px;
      background: #fff8c5;
    }}
    .scene-metrics-warning {{
      margin: 0 0 16px;
      padding: 12px;
      border: 1px solid #f0b429;
      border-radius: 6px;
      background: #fff8c5;
    }}
    .failure-summary {{
      margin: 0 0 16px;
      padding: 12px;
      border: 1px solid #ff8182;
      border-radius: 6px;
      background: #ffebe9;
    }}
    .failure-summary h2,
    .guidance-warning h2,
    .solver-identity-warning h2,
    .scene-metrics-warning h2 {{
      margin-bottom: 8px;
    }}
    .failure-summary p,
    .guidance-warning p,
    .solver-identity-warning p,
    .scene-metrics-warning p {{
      margin: 0 0 8px;
    }}
    .failure-summary ul,
    .guidance-warning ul,
    .solver-identity-warning ul,
    .scene-metrics-warning ul {{
      margin: 0;
      padding-left: 20px;
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
    code {{
      overflow-wrap: anywhere;
    }}
    .command-label {{
      margin: 8px 0 4px;
      color: #57606a;
      font-size: 12px;
      font-weight: 650;
      text-transform: uppercase;
      letter-spacing: 0;
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
    {workflow_command_html}
    <p>{_workflow_link("workflow manifest", "manifest.json")}</p>
    {phase_map}
    {deferred_api_map}
  </header>
  <main>
    {failure_summary}
    {guidance_warning}
    {solver_identity_warning}
    {scene_metrics_warning}
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
    continue_on_failure: bool = False,
    workflow_command: str = "",
) -> pathlib.Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest = output_dir / "manifest.json"
    _attach_workflow_resolved_solver_identities(captures)
    _attach_workflow_scene_metrics_evidence(captures)
    review_index = _write_workflow_review_index(
        output_dir,
        dry_run=dry_run,
        status=status,
        captures=captures,
        started_at=started_at,
        requested_include_flags=requested_include_flags,
        continue_on_failure=continue_on_failure,
        workflow_command=workflow_command,
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
    guidance_missing = _workflow_guidance_missing(captures)
    failed_rows = _workflow_failed_rows(captures)
    solver_identity_missing = _workflow_missing_solver_identity_rows(
        captures, dry_run=dry_run
    )
    scene_metrics_missing = _workflow_missing_scene_metrics_rows(
        captures, dry_run=dry_run
    )
    solver_identity_count = sum(
        1
        for capture in captures
        if isinstance(capture.get("resolved_solver_identity"), dict)
    )
    scene_metrics_count = sum(
        1
        for capture in captures
        if isinstance(capture.get("scene_metrics_evidence"), dict)
    )
    phase_summary = _workflow_phase_summary(captures)
    deferred_api_summary = _workflow_deferred_api_summary(captures)
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
            "workflow_phase_summary": phase_summary,
            "deferred_api_caveat_count": len(deferred_api_summary),
            "deferred_api_caveat_summary": deferred_api_summary,
            "continue_on_failure": continue_on_failure,
            "dry_run": dry_run,
            "status": status,
            "command": workflow_command,
            "capture_count": len(captures),
            "completed_count": completed,
            "failed_count": failed,
            "failed_rows": failed_rows,
            "guidance_complete": not guidance_missing,
            "guidance_missing_count": len(guidance_missing),
            "guidance_missing_rows": guidance_missing,
            "resolved_solver_identity_complete": (
                None
                if dry_run
                else (
                    completed == len(captures)
                    and failed == 0
                    and not solver_identity_missing
                )
            ),
            "resolved_solver_identity_count": solver_identity_count,
            "resolved_solver_identity_missing_count": len(solver_identity_missing),
            "resolved_solver_identity_missing_rows": solver_identity_missing,
            "scene_metrics_complete": (
                None
                if dry_run
                else completed == len(captures)
                and failed == 0
                and not scene_metrics_missing
            ),
            "scene_metrics_count": scene_metrics_count,
            "scene_metrics_missing_count": len(scene_metrics_missing),
            "scene_metrics_missing_rows": scene_metrics_missing,
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


def _run_rigid_workflow(args: argparse.Namespace) -> int:
    _validate_rigid_workflow_args(args)
    output_dir = _workflow_output_dir(args)
    started_at = time.monotonic()
    requested_include_flags = _workflow_requested_include_flags(args)
    specs = _workflow_capture_specs(args)
    _workflow_row_bounds(args, len(specs))
    captures = _workflow_plan_entries(args, output_dir)
    workflow_command = _public_command(_workflow_command_argv(args, output_dir))
    if args.dry_run:
        manifest = _write_workflow_manifest(
            output_dir,
            dry_run=True,
            status="planned",
            captures=captures,
            started_at=started_at,
            requested_include_flags=requested_include_flags,
            continue_on_failure=args.continue_on_failure,
            workflow_command=workflow_command,
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
        continue_on_failure=args.continue_on_failure,
        workflow_command=workflow_command,
    )
    first_failure_rc = 0
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
            status = "running" if args.continue_on_failure else "failed"
            if first_failure_rc == 0:
                first_failure_rc = rc if rc != 0 else 1
        manifest = _write_workflow_manifest(
            output_dir,
            dry_run=False,
            status=status,
            captures=captures,
            started_at=started_at,
            requested_include_flags=requested_include_flags,
            continue_on_failure=args.continue_on_failure,
            workflow_command=workflow_command,
        )
        if capture["status"] == "failed":
            print(f"manifest: {manifest}", flush=True)
            if not args.continue_on_failure:
                return rc if rc != 0 else 1

    _attach_workflow_resolved_solver_identities(captures)
    _attach_workflow_scene_metrics_evidence(captures)
    solver_identity_missing = _workflow_missing_solver_identity_rows(
        captures, dry_run=False
    )
    scene_metrics_missing = _workflow_missing_scene_metrics_rows(
        captures, dry_run=False
    )
    final_status = (
        "failed"
        if first_failure_rc or solver_identity_missing or scene_metrics_missing
        else "complete"
    )
    manifest = _write_workflow_manifest(
        output_dir,
        dry_run=False,
        status=final_status,
        captures=captures,
        started_at=started_at,
        requested_include_flags=requested_include_flags,
        continue_on_failure=args.continue_on_failure,
        workflow_command=workflow_command,
    )
    print(f"workflow manifest: {manifest}", flush=True)
    if first_failure_rc:
        return first_failure_rc
    return 1 if solver_identity_missing or scene_metrics_missing else 0


def main(argv: list[str] | None = None) -> int:
    _apply_stable_linux_render_env()
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.include_related and not args.rigid_workflow:
        raise SystemExit("--include-related requires --rigid-workflow")
    if args.include_ipc_shelf and not args.rigid_workflow:
        raise SystemExit("--include-ipc-shelf requires --rigid-workflow")
    if args.include_packets and not args.rigid_workflow:
        raise SystemExit("--include-packets requires --rigid-workflow")
    if args.continue_on_failure and not args.rigid_workflow:
        raise SystemExit("--continue-on-failure requires --rigid-workflow")
    if args.video and args.fps < 1:
        raise SystemExit("--fps must be positive")
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
    workflow_guidance = _rigid_workflow_guidance_by_scene().get(args.scene)
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
            "video": None,
        },
        "capture": capture_metadata,
        "resolved_solver_identity": None,
        "scene_metrics": None,
        "scene_metadata": None,
        "show_ui": args.show_ui,
        "ui_ready": None,
        "visual_evidence": visual_evidence,
        "workflow_guidance": workflow_guidance,
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
    scene_metrics_summary = _read_scene_metrics_summary(scene_metrics_events)
    manifest_payload["scene_metrics"] = scene_metrics_summary
    manifest_payload["resolved_solver_identity"] = (
        scene_metrics_summary.get("resolved_solver_identity")
        if isinstance(scene_metrics_summary, dict)
        else None
    )
    manifest_payload["scene_metadata"] = _read_scene_metadata(scene_metrics_events)
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
            manifest_payload["artifacts"]["video"] = str(video)
            _write_json(manifest, manifest_payload)
            print(f"video: {video}")
        else:
            print("video: skipped (ffmpeg not found)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
