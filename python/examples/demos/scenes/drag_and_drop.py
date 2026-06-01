"""Drag-and-drop scene: an interactive frame with a child draggable + axis markers.

Mirrors examples/demos/scenes/drag_and_drop.cpp at a high level. The C++
scene attaches a gizmo to the ``interactive frame`` so the user can
drag/rotate it in the viewer; the Python mirror builds the same scene
layout but does not register a gizmo (the gizmo binding is not yet
exposed on dartpy.gui). The scene still demonstrates frame parenting
(the draggable is a child of the interactive frame) and SimpleFrame
markers for the world axes.
"""

from __future__ import annotations

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup


def _translation(x: float, y: float, z: float) -> np.ndarray:
    tf = np.eye(4)
    tf[:3, 3] = (x, y, z)
    return tf


def _add_marker(
    world: "dart.World", name: str, position, color
) -> None:
    frame = dart.SimpleFrame(dart.Frame.world(), name)
    frame.set_shape(dart.BoxShape(np.array([0.2, 0.2, 0.2])))
    frame.create_visual_aspect().set_color(list(color))
    frame.set_relative_transform(_translation(*position))
    world.add_simple_frame(frame)


def build() -> SceneSetup:
    world = dart.simulation.World("drag_and_drop")
    world.set_gravity([0.0, 0.0, 0.0])

    anchor = dart.SimpleFrame(dart.Frame.world(), "interactive frame")
    anchor.set_relative_transform(_translation(4.0, -4.0, 0.0))
    world.add_simple_frame(anchor)

    draggable = anchor.spawn_child_simple_frame("draggable")
    draggable.set_shape(dart.BoxShape(np.array([1.0, 1.0, 1.0])))
    draggable.create_visual_aspect().set_color([0.9, 0.0, 0.0])
    draggable.set_relative_transform(_translation(-4.0, 4.0, 0.0))
    world.add_simple_frame(draggable)

    _add_marker(world, "X", (8.0, 0.0, 0.0), (0.9, 0.0, 0.0))
    _add_marker(world, "Y", (0.0, 8.0, 0.0), (0.0, 0.9, 0.0))
    _add_marker(world, "Z", (0.0, 0.0, 8.0), (0.0, 0.0, 0.9))

    return SceneSetup(world=world, info={})


SCENE = PythonDemoScene(
    id="drag_and_drop",
    title="Drag and Drop",
    category="Visualization",
    summary="Frame parenting demo (interactive gizmo is C++-only).",
    build=build,
)
