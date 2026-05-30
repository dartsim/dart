"""Rigid cubes scene: a stack of dynamic cubes loaded from cubes.skel.

Mirrors examples/demos/scenes/rigid_cubes.cpp. The C++ scene applies an
interactive directional force via keyboard; the Python mirror is headless,
so it just lets gravity settle the stack (deterministic for smoke testing).
"""

from __future__ import annotations

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup
from ._z_up import reorient_to_z_up


_URI = "dart://sample/skel/cubes.skel"


def build() -> SceneSetup:
    world = dart.io.SkelParser.read_world(_URI)
    if world is None:
        raise RuntimeError(f"Failed to load {_URI}")
    # cubes.skel is authored Y-up; reorient to the canonical Z-up convention.
    reorient_to_z_up(world)
    skeletons = [world.get_skeleton(i).get_name() for i in range(world.get_num_skeletons())]
    return SceneSetup(world=world, info={"skeletons": skeletons})


SCENE = PythonDemoScene(
    id="rigid_cubes",
    title="Rigid Cubes",
    category="Rigid Body",
    summary="A stack of rigid cubes loaded from cubes.skel.",
    build=build,
)
