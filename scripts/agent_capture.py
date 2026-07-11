"""Deterministic headless capture harness for agent visual evidence.

Drives a DART 7 world through the offscreen renderer with active camera
control: explicit orbit parameters, focus-framed reframing, auto-selected
viewpoints (view-quality scored), multi-view sets, turntables, and camera
paths for video. Debug layers expose internal state (frames, contacts,
velocities, trajectories, labels) beyond the plain render. Every capture
writes a sidecar JSON that fully reproduces it: scene id, steps, camera
parameters, layers, sizes, and the view-quality report.

Reuses the built-in scene registry from trajectory_record.py so text and
visual evidence come from identical worlds.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any

from trajectory_record import _BUILTIN_SCENES, _load_factory

SCHEMA_VERSION = "dart.agent_capture/v1"


def _import_dartpy() -> Any:
    try:
        import dartpy
    except ImportError as error:  # pragma: no cover - environment specific
        raise RuntimeError(
            "agent_capture needs a dartpy build with GUI support on PYTHONPATH; "
            "run through `pixi run agent-capture`"
        ) from error
    if not hasattr(dartpy, "gui") or not hasattr(dartpy.gui, "OffscreenRenderer"):
        raise RuntimeError("this dartpy build has no GUI offscreen renderer")
    return dartpy


def _write_png_rgba(path: Path, pixels: Any) -> None:
    import numpy as np

    array = np.asarray(pixels, dtype=np.uint8)
    if array.ndim != 3 or array.shape[2] != 4:
        raise ValueError("expected an (H, W, 4) RGBA array")
    import struct
    import zlib

    height, width = array.shape[:2]
    raw = b"".join(b"\x00" + array[row].tobytes() for row in range(height))

    def chunk(kind: bytes, payload: bytes) -> bytes:
        return (
            struct.pack(">I", len(payload))
            + kind
            + payload
            + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)
        )

    header = struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0)
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", header)
        + chunk(b"IDAT", zlib.compress(raw))
        + chunk(b"IEND", b"")
    )


def _camera_from_args(dart: Any, args: argparse.Namespace, world: Any) -> Any:
    gui = dart.gui
    if args.focus and args.camera_distance is None:
        return gui.frame_body(
            world,
            args.focus,
            azimuth=args.camera_azimuth,
            elevation=args.camera_elevation,
            size=(args.width, args.height),
            margin=args.frame_margin,
        )
    return gui.orbit_camera(
        azimuth=args.camera_azimuth,
        elevation=args.camera_elevation,
        distance=args.camera_distance if args.camera_distance is not None else 3.0,
        target=tuple(args.camera_target),
    )


def _capture_view(
    dart: Any,
    world: Any,
    camera: Any,
    args: argparse.Namespace,
    tracker: Any,
    contacts: list[Any],
    layers: list[str] | None = None,
) -> Any:
    gui = dart.gui
    active_layers = args.layers if layers is None else layers
    if active_layers:
        options = gui.DebugDrawOptions()
        scene = gui.debug_scene_for_world(
            world,
            options=options,
            layers=tuple(active_layers),
            trajectories=tracker,
            contacts=contacts,
        )
        return gui.render_annotated(
            world, camera, (args.width, args.height), debug=scene
        )
    import numpy as np

    image = gui.render(world, camera, (args.width, args.height))
    return np.array(memoryview(image), copy=True)


def _initial_still_layers(args: argparse.Namespace) -> list[str]:
    layers = list(args.layers)
    if args.motion_frames > 0 and args.steps < 2:
        layers = [layer for layer in layers if layer != "trajectories"]
    return layers


def _camera_json(camera: Any) -> dict[str, Any]:
    import numpy as np

    return {
        "azimuth": float(camera.yaw),
        "elevation": float(camera.pitch),
        "distance": float(camera.distance),
        "target": [float(v) for v in np.asarray(camera.target).reshape(3)],
    }


def _encode_video(frame_dir: Path, pattern: str, out: Path, fps: int) -> bool:
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        print("ffmpeg not found; skipping video encode", file=sys.stderr)
        return False
    command = [
        ffmpeg,
        "-y",
        "-loglevel",
        "error",
        "-framerate",
        str(fps),
        "-i",
        str(frame_dir / pattern),
        "-pix_fmt",
        "yuv420p",
        "-vf",
        "pad=ceil(iw/2)*2:ceil(ih/2)*2",
        str(out),
    ]
    try:
        subprocess.run(command, check=True)
    except subprocess.CalledProcessError as error:
        # The frames on disk are still valid evidence; keep the capture and
        # its sidecar instead of aborting the whole run.
        print(f"ffmpeg encode failed ({error}); keeping frames", file=sys.stderr)
        return False
    return True


def _require_acceptable_views(views: list[dict[str, Any]]) -> None:
    failures = []
    for view in views:
        report = view["report"]
        if not bool(report.get("pass", False)):
            issues = ", ".join(str(issue) for issue in report.get("issues", []))
            failures.append(f"{view['name']}: {issues or 'view assessment failed'}")
    if failures:
        raise ValueError(
            "capture rejected because view quality failed ("
            + "; ".join(failures)
            + "); adjust the camera or use --auto-views"
        )


def _assess_capture_view(gui, world, camera, args, name: str) -> dict[str, Any]:
    report = gui.assess_view(
        world, camera, (args.width, args.height), focus=args.focus or None
    ).to_json()
    _require_acceptable_views([{"name": name, "report": report}])
    return report


def run_capture(args: argparse.Namespace) -> dict[str, Any]:
    dart = _import_dartpy()
    gui = dart.gui

    if args.factory:
        factory = _load_factory(args.factory)
        scene_id = args.factory
    else:
        if args.scene not in _BUILTIN_SCENES:
            raise ValueError(
                f"unknown scene {args.scene!r}; available: {sorted(_BUILTIN_SCENES)}"
            )
        factory = _BUILTIN_SCENES[args.scene]
        scene_id = args.scene
    world = factory()

    tracker = gui.TrajectoryTracker(world) if "trajectories" in args.layers else None
    for _ in range(args.steps):
        world.step()
        if tracker is not None:
            tracker.sample()
    contacts = list(world.collide()) if "contacts" in args.layers else []

    out_dir = args.out
    out_dir.mkdir(parents=True, exist_ok=True)

    views: list[dict[str, Any]] = []
    artifacts: list[dict[str, Any]] = []

    if args.auto_views > 0:
        choices = gui.select_viewpoints(
            world,
            (args.width, args.height),
            focus=args.focus or None,
            count=args.auto_views,
        )
        for index, choice in enumerate(choices):
            views.append(
                {
                    "name": f"auto{index}",
                    "camera": choice.camera,
                    "report": choice.report.to_json(),
                    "reason": choice.reason,
                }
            )
    else:
        camera = _camera_from_args(dart, args, world)
        report = gui.assess_view(
            world, camera, (args.width, args.height), focus=args.focus or None
        )
        views.append(
            {
                "name": "main",
                "camera": camera,
                "report": report.to_json(),
                "reason": "explicit camera parameters",
            }
        )

    # View assessment is a capture gate, not advisory metadata. Refuse to
    # render or publish artifacts from a camera that the report rejects.
    _require_acceptable_views(views)

    for view in views:
        pixels = _capture_view(
            dart,
            world,
            view["camera"],
            args,
            tracker,
            contacts,
            layers=_initial_still_layers(args),
        )
        still_path = out_dir / f"{args.prefix}_{view['name']}.png"
        _write_png_rgba(still_path, pixels)
        artifacts.append(
            {
                "kind": "still",
                "view": view["name"],
                "path": still_path.name,
                "camera": _camera_json(view["camera"]),
                "view_report": view["report"],
                "reason": view["reason"],
            }
        )

    if args.turntable > 0:
        base_camera = views[0]["camera"]
        frame_dir = out_dir / f"{args.prefix}_turntable"
        frame_dir.mkdir(parents=True, exist_ok=True)
        for frame in range(args.turntable):
            camera = gui.orbit_camera(
                azimuth=float(base_camera.yaw) + math.tau * frame / args.turntable,
                elevation=float(base_camera.pitch),
                distance=float(base_camera.distance),
                target=base_camera.target,
            )
            pixels = _capture_view(dart, world, camera, args, tracker, contacts)
            _write_png_rgba(frame_dir / f"turn{frame:04d}.png", pixels)
        entry: dict[str, Any] = {
            "kind": "turntable",
            "view": views[0]["name"],
            "path": frame_dir.name,
            "frames": args.turntable,
        }
        if args.video:
            video_path = out_dir / f"{args.prefix}_turntable.mp4"
            if _encode_video(frame_dir, "turn%04d.png", video_path, args.fps):
                entry["video"] = video_path.name
        artifacts.append(entry)

    if args.motion_frames > 0:
        # Temporal capture: keep stepping and record frames from the primary
        # view (with optional simultaneous orbit for camera-motion video).
        frame_dir = out_dir / f"{args.prefix}_motion"
        frame_dir.mkdir(parents=True, exist_ok=True)
        base_camera = views[0]["camera"]
        sweep = math.radians(args.motion_orbit_degrees)
        motion_reports = []
        for frame in range(args.motion_frames):
            for _ in range(args.motion_substeps):
                world.step()
                if tracker is not None:
                    tracker.sample()
            contacts = list(world.collide()) if "contacts" in args.layers else []
            camera = gui.orbit_camera(
                azimuth=float(base_camera.yaw)
                + sweep * frame / max(args.motion_frames - 1, 1),
                elevation=float(base_camera.pitch),
                distance=float(base_camera.distance),
                target=base_camera.target,
            )
            motion_reports.append(
                _assess_capture_view(gui, world, camera, args, f"motion{frame}")
            )
            pixels = _capture_view(dart, world, camera, args, tracker, contacts)
            _write_png_rgba(frame_dir / f"frame{frame:04d}.png", pixels)
        entry = {
            "kind": "motion",
            "view": views[0]["name"],
            "path": frame_dir.name,
            "frames": args.motion_frames,
            "substeps": args.motion_substeps,
            "orbit_degrees": args.motion_orbit_degrees,
            "view_reports": motion_reports,
        }
        if args.video:
            video_path = out_dir / f"{args.prefix}_motion.mp4"
            if _encode_video(frame_dir, "frame%04d.png", video_path, args.fps):
                entry["video"] = video_path.name
        artifacts.append(entry)

    sidecar = {
        "schema_version": SCHEMA_VERSION,
        "scene": scene_id,
        "steps": args.steps,
        "size": [args.width, args.height],
        "layers": list(args.layers),
        "focus": args.focus or None,
        "auto_views": args.auto_views,
        "turntable": args.turntable,
        "motion_frames": args.motion_frames,
        "artifacts": artifacts,
        "reproduce": _reproduce_command(args),
        "pass": True,
    }
    sidecar_path = out_dir / f"{args.prefix}_capture.json"
    sidecar_path.write_text(
        json.dumps(sidecar, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return sidecar


def _reproduce_command(args: argparse.Namespace) -> str:
    parts = ["pixi run agent-capture --"]
    if args.factory:
        parts.append(f"--factory {args.factory}")
    else:
        parts.append(f"--scene {args.scene}")
    parts.append(f"--steps {args.steps}")
    parts.append(f"--width {args.width} --height {args.height}")
    if args.layers:
        parts.append("--layers " + " ".join(args.layers))
    if args.focus:
        parts.append(f"--focus {args.focus}")
    if args.auto_views:
        parts.append(f"--auto-views {args.auto_views}")
    else:
        parts.append(
            f"--camera-azimuth {args.camera_azimuth} "
            f"--camera-elevation {args.camera_elevation}"
        )
        if args.camera_distance is not None:
            parts.append(f"--camera-distance {args.camera_distance}")
        elif args.focus:
            # Focus framing derives the camera from --frame-margin; without
            # it the reproduce command would use a different distance.
            parts.append(f"--frame-margin {args.frame_margin}")
        target = " ".join(str(v) for v in args.camera_target)
        parts.append(f"--camera-target {target}")
    if args.turntable:
        parts.append(f"--turntable {args.turntable}")
    if args.motion_frames:
        parts.append(
            f"--motion-frames {args.motion_frames} "
            f"--motion-substeps {args.motion_substeps} "
            f"--motion-orbit-degrees {args.motion_orbit_degrees}"
        )
    if args.video:
        parts.append(f"--video --fps {args.fps}")
    parts.append(f"--out <dir> --prefix {args.prefix}")
    return " ".join(parts)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", default="box_on_ground")
    parser.add_argument("--factory", default=None, help="module:callable world factory")
    parser.add_argument("--steps", type=int, default=0, help="steps before capture")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=960)
    parser.add_argument(
        "--layers",
        nargs="*",
        default=[],
        help="debug layers (see dart.gui.DEBUG_LAYERS); empty renders plain",
    )
    parser.add_argument("--focus", default="", help="body name to frame/assess")
    parser.add_argument(
        "--auto-views",
        type=int,
        default=0,
        help="pick N viewpoints by view-quality score instead of explicit camera",
    )
    parser.add_argument("--camera-azimuth", type=float, default=0.7853981633974483)
    parser.add_argument("--camera-elevation", type=float, default=0.5235987755982988)
    parser.add_argument("--camera-distance", type=float, default=None)
    parser.add_argument("--camera-target", nargs=3, type=float, default=[0.0, 0.0, 0.0])
    parser.add_argument("--frame-margin", type=float, default=2.2)
    parser.add_argument("--turntable", type=int, default=0, help="turntable frames")
    parser.add_argument(
        "--motion-frames", type=int, default=0, help="temporal frames while stepping"
    )
    parser.add_argument("--motion-substeps", type=int, default=8)
    parser.add_argument(
        "--motion-orbit-degrees",
        type=float,
        default=0.0,
        help="camera azimuth sweep across the motion capture",
    )
    parser.add_argument("--video", action="store_true", help="encode MP4 via ffmpeg")
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--prefix", default="capture")
    args = parser.parse_args(argv)

    try:
        sidecar = run_capture(args)
    except (OSError, ValueError, RuntimeError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    print(json.dumps(sidecar, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
