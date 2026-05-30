"""Collision sandbox: static visual showcase of canonical shape pairs.

The C++ ``collision_sandbox`` scene displays the dart-collision-native
``pair_registry`` catalog (sphere/sphere, sphere/box, box/box,
capsule/box, etc.) laid out in a grid. The Python mirror is a
hand-curated equivalent — the pair registry itself lives in C++ test
infrastructure that is not bound to dartpy, so this Python scene
redeclares the same canonical pairs locally to keep the visual
catalog intact.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup


_COLUMN_SPACING = 3.5
_COLUMNS = 3
_ORIGIN_Z = 0.0
_CONTACT_COLOR = np.array([0.20, 0.55, 0.90, 0.85])
_DISTANCE_COLOR = np.array([0.90, 0.45, 0.20, 0.85])


@dataclass(frozen=True)
class _Pair:
    name: str
    make_a: Callable[[], "dart.Shape"]
    make_b: Callable[[], "dart.Shape"]
    offset_a: tuple[float, float, float] = (0.0, 0.0, 0.0)
    offset_b: tuple[float, float, float] = (0.0, 0.0, 0.0)
    supports_contact: bool = True


_PAIRS: tuple[_Pair, ...] = (
    _Pair(
        "sphere_sphere",
        make_a=lambda: dart.SphereShape(0.25),
        make_b=lambda: dart.SphereShape(0.25),
        offset_a=(-0.30, 0.0, 0.0),
        offset_b=(0.30, 0.0, 0.0),
    ),
    _Pair(
        "sphere_box",
        make_a=lambda: dart.SphereShape(0.20),
        make_b=lambda: dart.BoxShape(np.array([0.4, 0.4, 0.4])),
        offset_a=(-0.40, 0.0, 0.0),
        offset_b=(0.20, 0.0, 0.0),
    ),
    _Pair(
        "box_box",
        make_a=lambda: dart.BoxShape(np.array([0.4, 0.4, 0.4])),
        make_b=lambda: dart.BoxShape(np.array([0.4, 0.4, 0.4])),
        offset_a=(-0.30, 0.0, 0.0),
        offset_b=(0.30, 0.0, 0.0),
    ),
    _Pair(
        "capsule_box",
        make_a=lambda: dart.CapsuleShape(0.18, 0.5),
        make_b=lambda: dart.BoxShape(np.array([0.4, 0.4, 0.4])),
        offset_a=(-0.40, 0.0, 0.0),
        offset_b=(0.30, 0.0, 0.0),
    ),
    _Pair(
        "capsule_capsule",
        make_a=lambda: dart.CapsuleShape(0.16, 0.5),
        make_b=lambda: dart.CapsuleShape(0.16, 0.5),
        offset_a=(-0.30, 0.0, 0.0),
        offset_b=(0.30, 0.0, 0.0),
    ),
    _Pair(
        "cylinder_cylinder",
        make_a=lambda: dart.CylinderShape(0.20, 0.4),
        make_b=lambda: dart.CylinderShape(0.20, 0.4),
        offset_a=(-0.30, 0.0, 0.0),
        offset_b=(0.30, 0.0, 0.0),
    ),
    _Pair(
        "sphere_capsule",
        make_a=lambda: dart.SphereShape(0.20),
        make_b=lambda: dart.CapsuleShape(0.18, 0.5),
        offset_a=(-0.30, 0.0, 0.0),
        offset_b=(0.30, 0.0, 0.0),
    ),
    _Pair(
        "plane_sphere",
        make_a=lambda: dart.BoxShape(np.array([1.5, 1.5, 0.04])),
        make_b=lambda: dart.SphereShape(0.20),
        offset_a=(0.0, 0.0, -0.2),
        offset_b=(0.0, 0.0, 0.2),
    ),
    _Pair(
        "ellipsoid_sphere",
        make_a=lambda: dart.EllipsoidShape(np.array([0.35, 0.2, 0.2])),
        make_b=lambda: dart.SphereShape(0.18),
        offset_a=(-0.30, 0.0, 0.0),
        offset_b=(0.30, 0.0, 0.0),
        supports_contact=False,
    ),
)


def _add_frame(
    world: "dart.World", name: str, shape: "dart.Shape", translation, rgba
) -> None:
    frame = dart.SimpleFrame(dart.Frame.World(), name)
    frame.set_shape(shape)
    frame.create_visual_aspect().set_rgba(rgba)
    tf = np.eye(4)
    tf[:3, 3] = translation
    frame.set_relative_transform(tf)
    world.add_simple_frame(frame)


def build() -> SceneSetup:
    world = dart.simulation.World("collision_sandbox")
    world.set_gravity([0.0, 0.0, 0.0])

    ground = dart.SimpleFrame(dart.Frame.World(), "collision_sandbox_ground")
    ground.set_shape(dart.BoxShape(np.array([20.0, 20.0, 0.05])))
    ground.create_visual_aspect().set_rgba(np.array([0.78, 0.78, 0.78, 1.0]))
    ground_tf = np.eye(4)
    ground_tf[2, 3] = -1.5
    ground.set_relative_transform(ground_tf)
    world.add_simple_frame(ground)

    for index, pair in enumerate(_PAIRS):
        col = index % _COLUMNS
        row = index // _COLUMNS
        column_origin = np.array([col * _COLUMN_SPACING, row * _COLUMN_SPACING, _ORIGIN_Z])
        rgba = _CONTACT_COLOR if pair.supports_contact else _DISTANCE_COLOR
        _add_frame(
            world,
            f"{pair.name}_A",
            pair.make_a(),
            column_origin + np.array(pair.offset_a),
            rgba,
        )
        _add_frame(
            world,
            f"{pair.name}_B",
            pair.make_b(),
            column_origin + np.array(pair.offset_b),
            rgba * 0.85 + np.array([0.0, 0.0, 0.0, 0.15]),
        )
    return SceneSetup(world=world, info={})


SCENE = PythonDemoScene(
    id="collision_sandbox",
    title="Collision Sandbox",
    category="Collision",
    summary="Static showcase of canonical shape pair tests.",
    build=build,
)
