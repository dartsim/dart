"""Retrospective visual verification for merged PR #2984.

Reconstructs the visual evidence #2984 ("Improve renderer fidelity and
promote debug visuals into dart::gui") described but never attached, using
the agent evidence workflow: deterministic offscreen captures tied to
explicit claims (see docs/design/agent_sim_verification.md).

  C1  Per-shape PBR (metallic/roughness) produces visibly distinct materials.
  C2  high_fidelity offscreen rendering differs from the default reduced-
      fidelity headless output on a real GPU (post-processing restored).
  C3  Debug visuals reach the headless path: joint-axis/velocity-style
      overlay lines, contact markers/normals, frames, and labels render
      offscreen with world-derived composition (the follow-up #2984 deferred).

Run: pixi run retro-2984-evidence -- --out <dir>
Then: pixi run evidence-select -- <dir>/candidates.json --out <dir>/selection.json
      pixi run evidence-publish -- <dir>/selection.json ...
"""

from __future__ import annotations

import argparse
import json
import struct
import zlib
from pathlib import Path

import dartpy as dart
import numpy as np
from _image_tools import ImageData, diff_heatmap, side_by_side, write_image


def write_png_rgba(path: Path, array: np.ndarray) -> None:
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


def _sphere_descriptor(
    identifier: int,
    name: str,
    position: tuple[float, float, float],
    metallic: float,
    roughness: float,
) -> object:
    gui = dart.gui
    descriptor = gui.RenderableDescriptor()
    descriptor.id = identifier
    descriptor.shape_frame_name = name
    descriptor.shape_node_name = name
    descriptor.geometry.kind = gui.ShapeKind.Ellipsoid
    descriptor.geometry.size = np.array([0.5, 0.5, 0.5])
    descriptor.geometry.local_bounds_min = np.array([-0.25, -0.25, -0.25])
    descriptor.geometry.local_bounds_max = np.array([0.25, 0.25, 0.25])
    descriptor.geometry.has_local_bounds = True
    descriptor.material.rgba = np.array([0.75, 0.2, 0.15, 1.0])
    descriptor.material.metallic = metallic
    descriptor.material.roughness = roughness
    transform = np.eye(4)
    transform[:3, 3] = position
    descriptor.world_transform = transform
    descriptor.render_resource_version = 1
    return descriptor


def capture_pbr_sweep(out_dir: Path, size: tuple[int, int]) -> dict:
    """C1: metallic (rows) x roughness (columns) sphere sweep."""
    gui = dart.gui
    descriptors = []
    identifier = 0xB0000
    metallic_values = (0.0, 0.5, 1.0)
    roughness_values = (0.1, 0.45, 0.9)
    spacing = 0.7
    for row, metallic in enumerate(metallic_values):
        for column, roughness in enumerate(roughness_values):
            descriptors.append(
                _sphere_descriptor(
                    identifier,
                    f"m{metallic:g}_r{roughness:g}",
                    (
                        (column - 1) * spacing,
                        0.0,
                        (1 - row) * spacing,
                    ),
                    metallic,
                    roughness,
                )
            )
            identifier += 1
    camera = gui.orbit_camera(
        azimuth=-1.5707963267948966, elevation=0.0, distance=3.4, target=(0, 0, 0)
    )
    renderer = gui.OffscreenRenderer(width=size[0], height=size[1])
    image = renderer.render(descriptors, camera)
    pixels = np.array(memoryview(image), copy=True)
    # Column/row captions drawn in the margin keep the sweep self-describing.
    gui.draw_text(pixels, "ROUGHNESS 0.1 / 0.45 / 0.9 (LEFT-RIGHT)", (10, 8), scale=2)
    gui.draw_text(
        pixels, "METALLIC 0.0 / 0.5 / 1.0 (TOP-BOTTOM)", (10, size[1] - 20), scale=2
    )
    path = out_dir / "pbr_sweep.png"
    write_png_rgba(path, pixels)
    return {
        "path": path.name,
        "kind": "still",
        "claims": ["C1"],
        "caption": "Per-shape PBR sweep: metallic rows x roughness columns",
        "observe": (
            "top row stays dielectric-diffuse; bottom row turns metallic with "
            "sharp highlights at low roughness (bottom-left) fading to matte "
            "metal (bottom-right)"
        ),
        "quality": 0.9,
        "azimuth": -1.5707963267948966,
    }


def capture_fidelity_pair(
    out_dir: Path, size: tuple[int, int]
) -> tuple[list[dict], dict, object]:
    """C2: identical scene, default vs high-fidelity offscreen renderer.

    Returns (candidate entries, diff statistics, camera used).
    """
    gui = dart.gui
    world = dart.World()
    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -0.05))
    ground.is_static = True
    ground.set_collision_shape(dart.CollisionShape.box((1.6, 1.6, 0.1)))
    box = world.add_rigid_body("box", position=(0.0, 0.0, 0.35))
    box.set_collision_shape(dart.CollisionShape.box((0.25, 0.25, 0.25)))
    ball = world.add_rigid_body("ball", position=(0.35, 0.25, 0.2))
    ball.set_collision_shape(dart.CollisionShape.sphere(0.14))

    from dartpy import _world_render_bridge as bridge

    descriptors = bridge._renderables_from_world(world)
    # A whole-scene fit leaves the subject too small to judge shading
    # differences (assess_view flags it), so frame the box tightly instead.
    camera = gui.frame_body(
        world, "box", azimuth=0.9, elevation=0.42, size=size, margin=4.0
    )
    report = gui.assess_view(world, camera, size, focus="box")
    (out_dir / "fidelity_view_report.json").write_text(
        report.to_json_text() + "\n", encoding="utf-8"
    )

    outputs = {}
    for label, high_fidelity in (("default", False), ("high_fidelity", True)):
        renderer = gui.OffscreenRenderer(
            width=size[0], height=size[1], high_fidelity=high_fidelity
        )
        image = renderer.render(descriptors, camera)
        outputs[label] = np.array(memoryview(image), copy=True)
        write_png_rgba(out_dir / f"fidelity_{label}.png", outputs[label])

    def as_rgb(array: np.ndarray) -> ImageData:
        rgb = np.ascontiguousarray(array[..., :3])
        return ImageData(
            path=Path("memory"),
            width=rgb.shape[1],
            height=rgb.shape[0],
            pixels=rgb.tobytes(),
        )

    default_rgb = as_rgb(outputs["default"])
    high_rgb = as_rgb(outputs["high_fidelity"])
    composite = side_by_side(
        [default_rgb, high_rgb], labels=["DEFAULT HEADLESS", "HIGH FIDELITY"]
    )
    write_image(out_dir / "fidelity_compare.png", composite)
    heat, stats = diff_heatmap(default_rgb, high_rgb)
    write_image(out_dir / "fidelity_diff.png", heat)
    (out_dir / "fidelity_stats.json").write_text(
        json.dumps(stats, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    entry = {
        "path": "fidelity_compare.png",
        "kind": "composite",
        "claims": ["C2"],
        "caption": (
            "Same scene and camera: default reduced-fidelity headless render "
            "(left) vs DART_GUI_HIGH_FIDELITY-quality offscreen render (right)"
        ),
        "observe": (
            "right panel shows ambient occlusion in the box/ground crease and "
            "smoother anti-aliased edges; fidelity_stats.json quantifies the "
            "pixel delta"
        ),
        "quality": 0.85,
        "azimuth": float(camera.yaw),
    }
    return [entry], stats, camera


def capture_debug_overlay(out_dir: Path, size: tuple[int, int]) -> dict:
    """C3: world-derived debug layers on the headless path."""
    gui = dart.gui
    world = dart.World()
    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -0.05))
    ground.is_static = True
    ground.set_collision_shape(dart.CollisionShape.box((1.6, 1.6, 0.1)))
    box = world.add_rigid_body("box", position=(0.0, 0.0, 0.6))
    box.set_collision_shape(dart.CollisionShape.box((0.22, 0.22, 0.22)))
    ball = world.add_rigid_body("ball", position=(0.12, 0.1, 1.1))
    ball.set_collision_shape(dart.CollisionShape.sphere(0.12))

    tracker = gui.TrajectoryTracker(world, bodies=["ball"])
    for _ in range(700):
        world.step()
        tracker.sample()
    contacts = world.collide()

    camera = gui.frame_body(world, "box", azimuth=0.9, elevation=0.35, margin=3.2)
    report = gui.assess_view(world, camera, size, focus="box")
    options = gui.DebugDrawOptions()
    options.body_frame_axis_length = 0.25
    scene = gui.debug_scene_for_world(
        world,
        options=options,
        layers=("body_frames", "contacts", "trajectories", "velocities", "labels"),
        trajectories=tracker,
        contacts=contacts,
    )
    pixels = gui.render_annotated(world, camera, size, debug=scene)
    path = out_dir / "debug_overlay.png"
    write_png_rgba(path, pixels)
    (out_dir / "debug_overlay_view_report.json").write_text(
        report.to_json_text() + "\n", encoding="utf-8"
    )
    return {
        "path": path.name,
        "kind": "still",
        "claims": ["C3"],
        "caption": (
            "World-derived debug layers rendered offscreen: frames, contact "
            "markers/normals, ball trajectory, labels"
        ),
        "observe": (
            "RGB frame triads on each body, yellow contact crosses with "
            "orange normal arrows at the resting interfaces, the ball's fall "
            "trajectory polyline, and name labels"
        ),
        "quality": float(report.score),
        "azimuth": float(camera.yaw),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args(argv)
    size = (args.width, args.height)
    args.out.mkdir(parents=True, exist_ok=True)

    artifacts = [capture_pbr_sweep(args.out, size)]
    fidelity_entries, fidelity_stats, fidelity_camera = capture_fidelity_pair(
        args.out, size
    )
    artifacts.append(fidelity_entries[0])
    artifacts.append(capture_debug_overlay(args.out, size))

    manifest = {
        "schema_version": "dart.evidence_candidates/v1",
        "claims": [
            {
                "id": "C1",
                "text": (
                    "Per-shape PBR metallic/roughness parameters change the "
                    "rendered material appearance (dielectric vs metal, "
                    "gloss vs matte)"
                ),
            },
            {
                "id": "C2",
                "text": (
                    "high_fidelity offscreen rendering restores windowed-"
                    "quality post-processing relative to the default "
                    "headless output"
                ),
            },
            {
                "id": "C3",
                "text": (
                    "Debug visuals render on the headless path from "
                    "world-derived layers (frames, contacts, trajectories, "
                    "velocities, labels)"
                ),
            },
        ],
        "artifacts": [entry for entry in artifacts if entry.get("claims")],
        "fidelity_stats": fidelity_stats,
    }
    (args.out / "candidates.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({"out": str(args.out), "artifacts": len(artifacts)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
