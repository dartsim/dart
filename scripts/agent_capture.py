"""Deterministic headless capture harness for DART 6 agent visual evidence.

Drives a DART 6 world through the OSG offscreen path (WP-ASV
`Viewer.captureOffscreen`) with active camera control: explicit orbit
parameters, focus-framed reframing, view-quality-scored auto viewpoints,
turntables, and motion sequences with optional camera sweep. Debug layers
(contacts, body frames, velocities, trajectories, labels) are composited
onto each capture in image space — DART 7 draws its overlay always-on-top,
so the composite carries the same information. Every run writes a sidecar
JSON recording camera parameters, layers, view reports, and the exact
reproduction command.

Requires a GLX-capable X server (run under ``xvfb-run`` on headless hosts);
see docs/ai/verification.md.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any, Callable

import agent_debug_overlay as ado
import agent_view_quality as avq
import numpy as np

SCHEMA_VERSION = "dart.agent_capture/v1"


def _import_dartpy() -> Any:
    try:
        import dartpy
    except ImportError as error:  # pragma: no cover - environment specific
        raise RuntimeError(
            "agent_capture needs dartpy with the OSG GUI on PYTHONPATH; run "
            "through `pixi run agent-capture`"
        ) from error
    if not hasattr(dartpy.gui, "osg"):
        raise RuntimeError("this dartpy build has no gui.osg module")
    return dartpy


def _box_skeleton(
    dart: Any,
    name: str,
    size: tuple[float, float, float],
    position: tuple[float, float, float],
    *,
    static: bool = False,
    color: tuple[float, float, float] = (0.35, 0.55, 0.85),
) -> Any:
    skeleton = dart.dynamics.Skeleton(name)
    if static:
        joint, body = skeleton.createWeldJointAndBodyNodePair()
    else:
        joint, body = skeleton.createFreeJointAndBodyNodePair()
    body.setName(name)
    shape = dart.dynamics.BoxShape(np.asarray(size, dtype=float))
    shape_node = body.createShapeNode(shape)
    shape_node.createVisualAspect().setColor(list(color))
    shape_node.createCollisionAspect()
    shape_node.createDynamicsAspect()
    if static:
        transform = dart.math.Isometry3()
        transform.set_translation(list(position))
        joint.setTransformFromParentBodyNode(transform)
    else:
        joint.setPositions(
            np.concatenate([[0.0, 0.0, 0.0], np.asarray(position, dtype=float)])
        )
    return skeleton


def _make_box_on_ground(dart: Any) -> Any:
    world = dart.simulation.World()
    world.addSkeleton(
        _box_skeleton(
            dart, "ground", (2.0, 2.0, 0.1), (0.0, 0.0, -0.05), static=True,
            color=(0.75, 0.75, 0.78),
        )
    )
    world.addSkeleton(
        _box_skeleton(dart, "box", (0.2, 0.2, 0.2), (0.0, 0.0, 0.35))
    )
    return world


def _make_box_stack(dart: Any) -> Any:
    world = dart.simulation.World()
    world.addSkeleton(
        _box_skeleton(
            dart, "ground", (2.0, 2.0, 0.1), (0.0, 0.0, -0.05), static=True,
            color=(0.75, 0.75, 0.78),
        )
    )
    for index, height in enumerate((0.11, 0.34, 0.57)):
        world.addSkeleton(
            _box_skeleton(
                dart,
                f"stack{index}",
                (0.22 - 0.04 * index, 0.22 - 0.04 * index, 0.2),
                (0.0, 0.0, height),
                color=(0.35 + 0.18 * index, 0.5, 0.85 - 0.18 * index),
            )
        )
    return world


def _make_two_body_contact(dart: Any) -> Any:
    world = dart.simulation.World()
    world.addSkeleton(
        _box_skeleton(
            dart, "ground", (1.2, 1.2, 0.1), (0.0, 0.0, -0.05), static=True,
            color=(0.75, 0.75, 0.78),
        )
    )
    world.addSkeleton(_box_skeleton(dart, "boxA", (0.2, 0.2, 0.2), (0.0, 0.0, 0.11)))
    world.addSkeleton(
        _box_skeleton(dart, "boxB", (0.16, 0.16, 0.16), (0.05, 0.04, 0.4))
    )
    return world


_BUILTIN_SCENES: dict[str, Callable[[Any], Any]] = {
    "box_on_ground": _make_box_on_ground,
    "box_stack": _make_box_stack,
    "two_body_contact": _make_two_body_contact,
}


def _camera_from_args(args: argparse.Namespace, world: Any) -> avq.AgentCamera:
    if args.focus and args.camera_distance is None:
        return avq.frame_body(
            world,
            args.focus,
            azimuth=args.camera_azimuth,
            elevation=args.camera_elevation,
            fovy_deg=args.fovy_deg,
            margin=args.frame_margin,
        )
    return avq.orbit_camera(
        tuple(args.camera_target),
        args.camera_distance if args.camera_distance is not None else 3.0,
        azimuth=args.camera_azimuth,
        elevation=args.camera_elevation,
        fovy_deg=args.fovy_deg,
    )


def _sweep_camera(
    camera: avq.AgentCamera, azimuth_offset: float
) -> avq.AgentCamera:
    offset = np.asarray(camera.eye) - np.asarray(camera.center)
    distance_xy = math.hypot(offset[0], offset[1])
    azimuth = math.atan2(offset[1], offset[0]) + azimuth_offset
    eye = np.asarray(camera.center) + np.array(
        [
            distance_xy * math.cos(azimuth),
            distance_xy * math.sin(azimuth),
            offset[2],
        ]
    )
    return avq.AgentCamera(
        eye=eye,
        center=np.asarray(camera.center),
        up=np.asarray(camera.up),
        fovy_deg=camera.fovy_deg,
    )


def _capture_view(
    viewer: Any,
    world: Any,
    camera: avq.AgentCamera,
    path: Path,
    args: argparse.Namespace,
    tracker: Any,
    contacts: list[Any],
) -> bool:
    ok = viewer.captureOffscreen(
        str(path),
        camera.eye,
        camera.center,
        camera.up,
        width=args.width,
        height=args.height,
        fovYDeg=camera.fovy_deg,
        warmupFrames=args.warmup_frames,
    )
    if not ok:
        return False
    if args.layers:
        scene = ado.build_overlay(
            world,
            layers=tuple(args.layers),
            contacts=contacts if "contacts" in args.layers else None,
            trajectories=tracker,
        )
        ado.composite_overlay_file(path, scene, camera)
    return True


def _camera_json(camera: avq.AgentCamera) -> dict[str, Any]:
    return camera.to_json()


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
        print(f"ffmpeg encode failed ({error}); keeping frames", file=sys.stderr)
        return False
    return True


def run_capture(args: argparse.Namespace) -> dict[str, Any]:
    dart = _import_dartpy()
    if args.scene not in _BUILTIN_SCENES:
        raise ValueError(
            f"unknown scene {args.scene!r}; available: {sorted(_BUILTIN_SCENES)}"
        )
    world = _BUILTIN_SCENES[args.scene](dart)

    tracker = (
        ado.TrajectoryTracker(world) if "trajectories" in args.layers else None
    )
    for _ in range(args.steps):
        world.step()
        if tracker is not None:
            tracker.sample()
    contacts = (
        list(world.getLastCollisionResult().getContacts())
        if "contacts" in args.layers
        else []
    )

    viewer = dart.gui.osg.ImGuiViewer()
    viewer.addWorldNode(dart.gui.osg.WorldNode(world))

    out_dir = args.out
    out_dir.mkdir(parents=True, exist_ok=True)

    views: list[dict[str, Any]] = []
    if args.auto_views > 0:
        choices = avq.select_viewpoints(
            world,
            (args.width, args.height),
            focus=args.focus or None,
            count=args.auto_views,
            fovy_deg=args.fovy_deg,
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
        camera = _camera_from_args(args, world)
        report = avq.assess_view(
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

    artifacts: list[dict[str, Any]] = []
    for view in views:
        still_path = out_dir / f"{args.prefix}_{view['name']}.png"
        if not _capture_view(
            viewer, world, view["camera"], still_path, args, tracker, contacts
        ):
            raise RuntimeError(
                "captureOffscreen failed: no off-screen GL context (needs a "
                "usable DISPLAY; run under xvfb-run on headless hosts)"
            )
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
        base = views[0]["camera"]
        frame_dir = out_dir / f"{args.prefix}_turntable"
        frame_dir.mkdir(parents=True, exist_ok=True)
        for frame in range(args.turntable):
            camera = _sweep_camera(base, math.tau * frame / args.turntable)
            if not _capture_view(
                viewer,
                world,
                camera,
                frame_dir / f"turn{frame:04d}.png",
                args,
                tracker,
                contacts,
            ):
                raise RuntimeError("captureOffscreen failed during turntable")
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
        base = views[0]["camera"]
        frame_dir = out_dir / f"{args.prefix}_motion"
        frame_dir.mkdir(parents=True, exist_ok=True)
        sweep = math.radians(args.motion_orbit_degrees)
        for frame in range(args.motion_frames):
            for _ in range(args.motion_substeps):
                world.step()
                if tracker is not None:
                    tracker.sample()
            contacts = (
                list(world.getLastCollisionResult().getContacts())
                if "contacts" in args.layers
                else []
            )
            camera = _sweep_camera(
                base, sweep * frame / max(args.motion_frames - 1, 1)
            )
            if not _capture_view(
                viewer,
                world,
                camera,
                frame_dir / f"frame{frame:04d}.png",
                args,
                tracker,
                contacts,
            ):
                raise RuntimeError("captureOffscreen failed during motion capture")
        entry = {
            "kind": "motion",
            "view": views[0]["name"],
            "path": frame_dir.name,
            "frames": args.motion_frames,
            "substeps": args.motion_substeps,
            "orbit_degrees": args.motion_orbit_degrees,
        }
        if args.video:
            video_path = out_dir / f"{args.prefix}_motion.mp4"
            if _encode_video(frame_dir, "frame%04d.png", video_path, args.fps):
                entry["video"] = video_path.name
        artifacts.append(entry)

    sidecar = {
        "schema_version": SCHEMA_VERSION,
        "scene": args.scene,
        "steps": args.steps,
        "size": [args.width, args.height],
        "fovy_deg": args.fovy_deg,
        "layers": list(args.layers),
        "focus": args.focus or None,
        "auto_views": args.auto_views,
        "turntable": args.turntable,
        "motion_frames": args.motion_frames,
        "artifacts": artifacts,
        "reproduce": _reproduce_command(args),
        "pass": True,
    }
    (out_dir / f"{args.prefix}_capture.json").write_text(
        json.dumps(sidecar, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return sidecar


def _reproduce_command(args: argparse.Namespace) -> str:
    parts = ["pixi run agent-capture --"]
    parts.append(f"--scene {args.scene}")
    parts.append(f"--steps {args.steps}")
    parts.append(f"--width {args.width} --height {args.height}")
    if args.fovy_deg != avq.DEFAULT_FOVY_DEG:
        parts.append(f"--fovy-deg {args.fovy_deg}")
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
    parser.add_argument("--steps", type=int, default=0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=960)
    parser.add_argument("--fovy-deg", type=float, default=avq.DEFAULT_FOVY_DEG)
    parser.add_argument(
        "--layers",
        nargs="*",
        default=[],
        help=f"debug layers to composite (available: {list(ado.DEBUG_LAYERS)})",
    )
    parser.add_argument("--focus", default="")
    parser.add_argument("--auto-views", type=int, default=0)
    parser.add_argument("--camera-azimuth", type=float, default=math.tau / 8.0)
    parser.add_argument("--camera-elevation", type=float, default=math.tau / 12.0)
    parser.add_argument("--camera-distance", type=float, default=None)
    parser.add_argument(
        "--camera-target", nargs=3, type=float, default=[0.0, 0.0, 0.0]
    )
    parser.add_argument("--frame-margin", type=float, default=2.2)
    parser.add_argument("--warmup-frames", type=int, default=10)
    parser.add_argument("--turntable", type=int, default=0)
    parser.add_argument("--motion-frames", type=int, default=0)
    parser.add_argument("--motion-substeps", type=int, default=8)
    parser.add_argument("--motion-orbit-degrees", type=float, default=0.0)
    parser.add_argument("--video", action="store_true")
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
