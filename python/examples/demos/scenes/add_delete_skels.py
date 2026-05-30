"""Add/delete skeletons scene: spawn cubes deterministically.

Mirrors examples/demos/scenes/add_delete_skels.cpp. The C++ scene exposes a
keyboard action that spawns randomly-positioned cubes on demand; the Python
mirror is headless, so it spawns a fixed set of 6 cubes at deterministic
positions/sizes/colors (the build is reproducible across runs).
"""

from __future__ import annotations

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup
from ._z_up import reorient_to_z_up

_GROUND_URI = "dart://sample/skel/ground.skel"
_GROUND_NAME = "add_delete_ground"
_CUBE_PREFIX = "spawned_cube_"
_NUM_CUBES = 6


def _create_cube(name: str, position: tuple, size: tuple, color: tuple, mass: float = 0.1) -> "dart.Skeleton":
    skel = dart.Skeleton(name)
    _, body = skel.create_free_joint_and_body_node_pair()
    tf = np.eye(4)
    tf[:3, 3] = position
    body.get_parent_joint().set_transform_from_parent_body_node(tf)

    size_arr = np.array(size)
    shape = dart.BoxShape(size_arr)
    sn = body.create_shape_node(shape)
    sn.create_visual_aspect().set_color(list(color))
    sn.create_collision_aspect()
    sn.create_dynamics_aspect()
    body.set_inertia(dart.Inertia(
        mass,
        np.zeros(3),
        dart.BoxShape.compute_inertia_of(size_arr, mass),
    ))
    return skel


def build() -> SceneSetup:
    world = dart.io.SkelParser.read_world(_GROUND_URI)
    if world is None:
        raise RuntimeError(f"Failed to load {_GROUND_URI}")

    ground = world.get_skeleton(0)
    if ground is not None:
        ground.set_name(_GROUND_NAME)

    for i in range(_NUM_CUBES):
        position = (-1.0 + 0.4 * i, 0.6 + 0.15 * i, -0.5 + 0.2 * i)
        size = 0.18 + 0.04 * (i % 3)
        color = ((i % 3) / 3.0, ((i + 1) % 3) / 3.0, ((i + 2) % 3) / 3.0)
        world.add_skeleton(_create_cube(
            f"{_CUBE_PREFIX}{i}", position, (size, size, size), color))

    # ground.skel and the spawn poses above are authored Y-up; reorient the
    # whole world (ground + cubes) to the canonical Z-up convention.
    reorient_to_z_up(world)

    return SceneSetup(world=world, info={})


SCENE = PythonDemoScene(
    id="add_delete_skels",
    title="Add / Delete Skeletons",
    category="Rigid Body",
    summary="Spawn cubes at runtime (deterministic Python mirror).",
    build=build,
)
