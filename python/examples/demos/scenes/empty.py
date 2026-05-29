"""Empty scaffold scene: a few SimpleFrames in a gravity-free world.

Mirrors examples/demos/scenes/empty.cpp. Drops the keyboard actions, panels,
and pre/post hook counters — those are GUI-only scaffolding for the
interactive C++ host.
"""

from __future__ import annotations

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup


def _transform(translation: tuple[float, float, float]) -> np.ndarray:
    tf = np.eye(4)
    tf[:3, 3] = translation
    return tf


def _box_frame(
    name: str,
    parent: "dart.Frame",
    translation: tuple[float, float, float],
    size: tuple[float, float, float],
    color: tuple[float, float, float],
) -> "dart.SimpleFrame":
    frame = dart.SimpleFrame(parent, name, _transform(translation))
    frame.set_shape(dart.BoxShape(np.array(size)))
    frame.create_visual_aspect().set_color(list(color))
    return frame


def build() -> SceneSetup:
    world = dart.World("dartsim_empty")
    world.set_gravity([0.0, 0.0, 0.0])

    anchor = _box_frame(
        "interactive frame", dart.Frame.world(),
        (4.0, -4.0, 0.0), (0.45, 0.45, 0.45), (0.95, 0.70, 0.15))
    world.add_simple_frame(anchor)

    draggable = _box_frame(
        "draggable", anchor,
        (-4.0, 4.0, 0.0), (1.0, 1.0, 1.0), (0.90, 0.00, 0.00))
    world.add_simple_frame(draggable)

    world.add_simple_frame(_box_frame(
        "X", dart.Frame.world(), (8.0, 0.0, 0.0), (0.25, 0.25, 0.25), (0.90, 0.0, 0.0)))
    world.add_simple_frame(_box_frame(
        "Y", dart.Frame.world(), (0.0, 8.0, 0.0), (0.25, 0.25, 0.25), (0.0, 0.90, 0.0)))
    world.add_simple_frame(_box_frame(
        "Z", dart.Frame.world(), (0.0, 0.0, 8.0), (0.25, 0.25, 0.25), (0.0, 0.0, 0.90)))

    return SceneSetup(world=world, info={})


SCENE = PythonDemoScene(
    id="empty",
    title="Empty Scaffold",
    category="Getting Started",
    summary="Minimal SimpleFrame hierarchy with axis markers.",
    build=build,
)
