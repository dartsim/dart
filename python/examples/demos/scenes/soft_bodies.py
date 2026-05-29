"""Soft bodies scene: a soft-body world loaded from softBodies.skel.

Mirrors examples/demos/scenes/soft_bodies.cpp. Drops the recorded-state
playback (the C++ scene's interactive history-keyed scrubber); the Python
mirror just simulates forward like every other Python scene.
"""

from __future__ import annotations

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup

_URI = "dart://sample/skel/softBodies.skel"


def build() -> SceneSetup:
    world = dart.utils.SkelParser.read_world(_URI)
    if world is None:
        raise RuntimeError(f"Failed to load {_URI}")
    return SceneSetup(world=world, info={})


SCENE = PythonDemoScene(
    id="soft_bodies",
    title="Soft Bodies",
    category="Soft Bodies",
    summary="Soft-body simulation loaded from softBodies.skel.",
    build=build,
)
