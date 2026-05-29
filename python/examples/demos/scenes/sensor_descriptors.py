"""Sensor depth/segmentation (stub): expose the dartpy.gui descriptor surface.

The headless ``dartpy.gui`` submodule exposes renderable descriptors (geometry,
material, world transform) that a downstream pipeline can render into depth,
segmentation, or RGB images. This scene builds a small world and surfaces the
descriptor-extraction path so users can wire their own offline renderer. Full
in-process depth/segmentation rendering ties to PLAN-090's fidelity-profile
seam and is intentionally NOT implemented here.
"""

from __future__ import annotations

import numpy as np

import dartpy as dart
import dartpy.gui as dgui  # noqa: F401 — surfaces the headless GUI submodule

from ..runner import PythonDemoScene, SceneSetup


def _make_target_box() -> dart.Skeleton:
    skel = dart.Skeleton("target")
    joint, body = skel.create_free_joint_and_body_node_pair()
    transform = np.eye(4)
    transform[:3, 3] = (0.0, 0.0, 0.6)
    joint.set_transform_from_parent_body_node(transform)
    shape = dart.BoxShape(np.array([0.2, 0.2, 0.2]))
    sn = body.create_shape_node(shape)
    sn.create_visual_aspect().set_color([0.20, 0.85, 0.55])
    sn.create_collision_aspect()
    sn.create_dynamics_aspect()
    body.set_inertia(dart.Inertia(
        1.0,
        np.zeros(3),
        dart.BoxShape.compute_inertia_of(np.array([0.2, 0.2, 0.2]), 1.0),
    ))
    return skel


def _make_ground() -> dart.Skeleton:
    skel = dart.Skeleton("ground")
    joint, body = skel.create_weld_joint_and_body_node_pair()
    t = np.eye(4)
    t[:3, 3] = (0.0, 0.0, -0.05)
    joint.set_transform_from_parent_body_node(t)
    shape = dart.BoxShape(np.array([4.0, 4.0, 0.1]))
    sn = body.create_shape_node(shape)
    sn.create_visual_aspect().set_color([0.7, 0.7, 0.75])
    sn.create_collision_aspect()
    sn.create_dynamics_aspect()
    return skel


def build() -> SceneSetup:
    world = dart.World("sensor_descriptors")
    world.set_gravity([0.0, 0.0, -9.81])
    world.add_skeleton(_make_target_box())
    world.add_skeleton(_make_ground())

    # Probe the dartpy.gui descriptor surface without depending on any
    # particular method name; downstream code wires its own renderer once the
    # PLAN-090 fidelity-profile camera-sensor seam is reachable from Python.
    gui_surface = [name for name in dir(dgui) if not name.startswith("_")]

    return SceneSetup(world=world, info={
        "dartpy_gui_symbols": gui_surface,
        "rendering_stub": True,
        "depth_segmentation_status": (
            "deferred to PLAN-090 fidelity profile seam; the headless "
            "dartpy.gui submodule exposes renderable descriptors today"
        ),
    })


SCENE = PythonDemoScene(
    id="sensor_descriptors",
    title="Sensor Descriptors (depth/seg stub)",
    category="Control & Modern",
    summary="A scene + the headless dartpy.gui descriptor surface (stub).",
    build=build,
)
