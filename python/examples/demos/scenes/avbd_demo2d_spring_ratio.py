"""AVBD port of the avbd-demo2d Spring Ratio source scene."""

from __future__ import annotations

from collections import deque
from typing import Any

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 1.0 / 60.0
_GRAVITY = -10.0
_FRICTION = 0.5
_THICKNESS = 0.2
_LINK_COUNT = 8
_SPRING_COUNT = _LINK_COUNT - 1
_LINK_SIZE_2D = np.array([1.0, 0.5])
_LINK_SIZE = np.array([1.0, 0.5, _THICKNESS])
_PARENT_ANCHOR = np.array([0.5, 0.0, 0.0])
_CHILD_ANCHOR = np.array([-0.5, 0.0, 0.0])
_LOW_STIFFNESS = 1.0e3
_HIGH_STIFFNESS = 1.0e6
_SPRING_REST_LENGTH = 0.1
_SOURCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo2d",
    "repository": "https://github.com/savant117/avbd-demo2d",
    "revision": "74699a11f858",
    "dimension": 2,
    "scene_index": 10,
    "scene_name": "Spring Ratio",
    "scene_count": 19,
    "scene_builder": "sceneSpringsRatio",
    "solver_defaults": {
        "time_step": _TIME_STEP,
        "gravity_axis": "y",
        "gravity": _GRAVITY,
        "iterations": 10,
    },
    "source_shapes": {
        "spring_link": {
            "count": _LINK_COUNT,
            "size": tuple(_LINK_SIZE_2D),
            "dynamic_density": 1.0,
            "static_endpoint_density": 0.0,
            "friction": _FRICTION,
            "first_position": (0.0, 10.0, 0.0),
            "x_spacing": 4.0,
        },
    },
    "source_constraints": {
        "radial_distance_springs": {
            "count": _SPRING_COUNT,
            "parent_anchor": (0.5, 0.0),
            "child_anchor": (-0.5, 0.0),
            "rest_length": _SPRING_REST_LENGTH,
            "low_stiffness": _LOW_STIFFNESS,
            "high_stiffness": _HIGH_STIFFNESS,
            "low_stiffness_count": 3,
            "high_stiffness_count": 4,
        },
    },
    "expected_counts": {
        "rigid_bodies": _LINK_COUNT,
        "dynamic_bodies": _LINK_COUNT - 2,
        "static_bodies": 2,
        "joints": 0,
        "distance_springs": _SPRING_COUNT,
        "collision_shapes": _LINK_COUNT,
    },
}


def _box_mass_2d(size_xy: np.ndarray, density: float) -> float:
    return float(np.prod(size_xy) * density)


def _full_box_inertia(size: np.ndarray, mass: float) -> np.ndarray:
    return np.diag(
        [
            mass * float(size[1] * size[1] + size[2] * size[2]) / 12.0,
            mass * float(size[0] * size[0] + size[2] * size[2]) / 12.0,
            mass * float(size[0] * size[0] + size[1] * size[1]) / 12.0,
        ]
    )


def _source_link_position(index: int) -> np.ndarray:
    return np.array([4.0 * float(index), 10.0, 0.0])


def _spring_stiffness(child_index: int) -> float:
    return _LOW_STIFFNESS if child_index % 2 == 0 else _HIGH_STIFFNESS


def _source_row() -> dict[str, Any]:
    return {
        **_SOURCE_ROW,
        "solver_defaults": dict(_SOURCE_ROW["solver_defaults"]),
        "source_shapes": {
            name: dict(shape)
            for name, shape in _SOURCE_ROW["source_shapes"].items()
        },
        "source_constraints": {
            name: dict(constraint)
            for name, constraint in _SOURCE_ROW["source_constraints"].items()
        },
        "expected_counts": dict(_SOURCE_ROW["expected_counts"]),
    }


def _add_source_box(
    world: sx.World,
    name: str,
    *,
    density: float,
    position: np.ndarray,
    is_static: bool = False,
) -> sx.RigidBody:
    body = world.add_rigid_body(name, position=tuple(position))
    body.is_static = is_static
    body.friction = _FRICTION
    body.set_collision_shape(sx.CollisionShape.box(0.5 * _LINK_SIZE))
    if not is_static:
        mass = _box_mass_2d(_LINK_SIZE_2D, density)
        body.mass = mass
        body.inertia = _full_box_inertia(_LINK_SIZE, mass)
    return body


def _anchor_world_position(body: sx.RigidBody, local_anchor: np.ndarray) -> np.ndarray:
    rotation = np.asarray(body.rotation, dtype=float).reshape(3, 3)
    translation = np.asarray(body.translation, dtype=float).reshape(3)
    return translation + rotation @ local_anchor


def build() -> SceneSetup:
    world = sx.World(time_step=_TIME_STEP, gravity=(0.0, _GRAVITY, 0.0))

    links: list[sx.RigidBody] = []
    for index in range(_LINK_COUNT):
        is_static = index == 0 or index == _LINK_COUNT - 1
        links.append(
            _add_source_box(
                world,
                f"avbd_demo2d_spring_ratio_link_{index:02d}",
                density=0.0 if is_static else 1.0,
                position=_source_link_position(index),
                is_static=is_static,
            )
        )

    springs: list[dict[str, Any]] = []
    ignored_collision_pairs: list[tuple[sx.RigidBody, sx.RigidBody]] = []
    for child_index, (parent, child) in enumerate(zip(links, links[1:]), start=1):
        stiffness = _spring_stiffness(child_index)
        name = f"avbd_demo2d_spring_ratio_radial_{child_index - 1:02d}"
        world.add_rigid_body_distance_spring(
            name,
            parent,
            child,
            _SPRING_REST_LENGTH,
            stiffness,
            parent_anchor=tuple(_PARENT_ANCHOR),
            child_anchor=tuple(_CHILD_ANCHOR),
        )
        world.set_collision_pair_ignored(parent, child)
        ignored_collision_pairs.append((parent, child))
        springs.append(
            {
                "name": name,
                "parent": parent.name,
                "child": child.name,
                "parent_anchor": tuple(_PARENT_ANCHOR),
                "child_anchor": tuple(_CHILD_ANCHOR),
                "rest_length": _SPRING_REST_LENGTH,
                "stiffness": stiffness,
            }
        )

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_demo2d_spring_ratio_render")
    for index, link in enumerate(links):
        is_endpoint = index == 0 or index == _LINK_COUNT - 1
        bridge.add_rigid_body_visual(
            link,
            dart.BoxShape(_LINK_SIZE),
            (0.30, 0.38, 0.44) if is_endpoint else (0.28, 0.58, 0.72),
            name=f"avbd_demo2d_spring_ratio_link_{index:02d}_visual",
        )
    bridge.sync()

    max_length_history: deque[float] = deque(maxlen=160)
    middle_y_history: deque[float] = deque(maxlen=160)
    middle = links[_LINK_COUNT // 2]
    initial_middle_y = float(np.asarray(middle.translation, dtype=float).reshape(3)[1])

    def spring_lengths() -> list[float]:
        lengths = []
        for parent, child in zip(links, links[1:]):
            lengths.append(
                float(
                    np.linalg.norm(
                        _anchor_world_position(parent, _PARENT_ANCHOR)
                        - _anchor_world_position(child, _CHILD_ANCHOR)
                    )
                )
            )
        return lengths

    def build_panel(builder: object, context: object) -> None:
        lengths = spring_lengths()
        max_length = max(lengths)
        middle_y = float(np.asarray(middle.translation, dtype=float).reshape(3)[1])
        max_length_history.append(max_length)
        middle_y_history.append(middle_y)

        builder.text("source corpus: avbd-demo2d Spring Ratio")
        builder.text("source scene: sceneSpringsRatio, index 10 of 19")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"distance springs: {len(springs)}")
        builder.text(f"ignored collision pairs: {len(ignored_collision_pairs)}")
        builder.text(f"max spring length: {max_length:.3f} m")
        builder.text(f"middle-link dy: {middle_y - initial_middle_y:.3f} m")
        builder.text(f"world time: {world.time:.3f} s")
        builder.plot_lines("Max spring length", list(max_length_history))
        builder.plot_lines("Middle y", list(middle_y_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Demo2D Spring Ratio", build_panel)],
        info={
            "sx_world": world,
            "links": links,
            "springs": springs,
            "ignored_collision_pairs": ignored_collision_pairs,
            "source_demo_row": "avbd-demo2d spring ratio",
            "source_demo_reference": _source_row(),
            "spring_lengths": spring_lengths,
        },
    )


SCENE = PythonDemoScene(
    id="avbd_demo2d_spring_ratio",
    title="AVBD Demo2D Spring Ratio (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary=(
        "A source-demo Spring Ratio row port with alternating radial "
        "distance-spring stiffness."
    ),
    build=build,
)
