"""AVBD port of the avbd-demo2d Cards source scene."""

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
_FRICTION = 0.7
_THICKNESS = 0.2
_LEVELS = 5
_CARD_COUNT = 40
_HORIZONTAL_CARD_COUNT = 10
_LEANING_CARD_COUNT = 30
_CARD_HEIGHT = 0.2 * 2.0
_CARD_THICKNESS = 0.001 * 2.0
_ANGLE_POSITIVE = 25.0 * 3.14159 / 180.0
_ANGLE_NEGATIVE = -25.0 * 3.14159 / 180.0
_ANGLE_HORIZONTAL = 0.5 * 3.14159
_GROUND_SIZE_2D = np.array([80.0, 4.0])
_GROUND_SIZE = np.array([80.0, 4.0, _THICKNESS])
_CARD_SIZE_2D = np.array([_CARD_THICKNESS, _CARD_HEIGHT])
_CARD_SIZE = np.array([_CARD_THICKNESS, _CARD_HEIGHT, _THICKNESS])
_SOURCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo2d",
    "repository": "https://github.com/savant117/avbd-demo2d",
    "revision": "74699a11f858",
    "dimension": 2,
    "scene_index": 5,
    "scene_name": "Cards",
    "scene_count": 19,
    "scene_builder": "sceneCards",
    "solver_defaults": {
        "time_step": _TIME_STEP,
        "gravity_axis": "y",
        "gravity": _GRAVITY,
        "iterations": 10,
    },
    "source_shapes": {
        "ground": {
            "size": tuple(_GROUND_SIZE_2D),
            "density": 0.0,
            "friction": _FRICTION,
            "position": (0.0, -2.0, 0.0),
        },
        "cards": {
            "count": _CARD_COUNT,
            "levels": _LEVELS,
            "horizontal_count": _HORIZONTAL_CARD_COUNT,
            "leaning_count": _LEANING_CARD_COUNT,
            "size": tuple(_CARD_SIZE_2D),
            "density": 1.0,
            "friction": _FRICTION,
            "angle_positive": _ANGLE_POSITIVE,
            "angle_negative": _ANGLE_NEGATIVE,
            "angle_horizontal": _ANGLE_HORIZONTAL,
            "first_position": (0.25, 0.36, _ANGLE_HORIZONTAL),
            "last_position": (0.875, 1.62, _ANGLE_POSITIVE),
        },
    },
    "expected_counts": {
        "rigid_bodies": 41,
        "dynamic_bodies": 40,
        "static_bodies": 1,
        "joints": 0,
        "collision_shapes": 41,
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


def _z_axis_orientation(angle: float) -> tuple[float, float, float, float]:
    return (float(np.cos(0.5 * angle)), 0.0, 0.0, float(np.sin(0.5 * angle)))


def _source_row() -> dict[str, Any]:
    return {
        **_SOURCE_ROW,
        "solver_defaults": dict(_SOURCE_ROW["solver_defaults"]),
        "source_shapes": {
            name: dict(shape)
            for name, shape in _SOURCE_ROW["source_shapes"].items()
        },
        "expected_counts": dict(_SOURCE_ROW["expected_counts"]),
    }


def _source_card_specs() -> list[dict[str, Any]]:
    specs: list[dict[str, Any]] = []
    remaining = _LEVELS
    x0 = 0.0
    y = _CARD_HEIGHT * 0.5 - 0.02
    level = 0
    while remaining:
        x = x0
        for card_index in range(remaining):
            if card_index != remaining - 1:
                specs.append(
                    {
                        "level": level,
                        "kind": "horizontal",
                        "position": np.array(
                            [x + 0.25, y + _CARD_HEIGHT * 0.5 - 0.02, 0.0]
                        ),
                        "angle": _ANGLE_HORIZONTAL,
                    }
                )

            specs.append(
                {
                    "level": level,
                    "kind": "leaning_negative",
                    "position": np.array([x, y, 0.0]),
                    "angle": _ANGLE_NEGATIVE,
                }
            )
            x += 0.175

            specs.append(
                {
                    "level": level,
                    "kind": "leaning_positive",
                    "position": np.array([x, y, 0.0]),
                    "angle": _ANGLE_POSITIVE,
                }
            )
            x += 0.175

        y += _CARD_HEIGHT - 0.04
        x0 += 0.175
        remaining -= 1
        level += 1

    return specs


def _add_source_box(
    world: sx.World,
    name: str,
    *,
    size: np.ndarray,
    size_2d: np.ndarray,
    density: float,
    position: np.ndarray,
    angle: float = 0.0,
    is_static: bool = False,
) -> sx.RigidBody:
    body = world.add_rigid_body(
        name,
        position=tuple(position),
        orientation=_z_axis_orientation(angle),
    )
    body.is_static = is_static
    body.friction = _FRICTION
    body.set_collision_shape(sx.CollisionShape.box(0.5 * size))
    if not is_static:
        mass = _box_mass_2d(size_2d, density)
        body.mass = mass
        body.inertia = _full_box_inertia(size, mass)
    return body


def build() -> SceneSetup:
    world = sx.World(time_step=_TIME_STEP, gravity=(0.0, _GRAVITY, 0.0))

    ground = _add_source_box(
        world,
        "avbd_demo2d_cards_ground",
        size=_GROUND_SIZE,
        size_2d=_GROUND_SIZE_2D,
        density=0.0,
        position=np.array([0.0, -2.0, 0.0]),
        is_static=True,
    )

    card_specs = _source_card_specs()
    cards: list[sx.RigidBody] = []
    for index, spec in enumerate(card_specs):
        cards.append(
            _add_source_box(
                world,
                f"avbd_demo2d_cards_card_{index}",
                size=_CARD_SIZE,
                size_2d=_CARD_SIZE_2D,
                density=1.0,
                position=spec["position"],
                angle=float(spec["angle"]),
            )
        )

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_demo2d_cards_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_GROUND_SIZE),
        (0.38, 0.38, 0.40),
        name="avbd_demo2d_cards_ground_visual",
    )
    for index, card in enumerate(cards):
        level = int(card_specs[index]["level"])
        kind = str(card_specs[index]["kind"])
        blend = float(level) / float(max(1, _LEVELS - 1))
        if kind == "horizontal":
            color = (0.38 + 0.24 * blend, 0.61 - 0.18 * blend, 0.78)
        else:
            color = (0.83 - 0.34 * blend, 0.46 + 0.20 * blend, 0.24 + 0.34 * blend)
        bridge.add_rigid_body_visual(
            card,
            dart.BoxShape(_CARD_SIZE),
            color,
            name=f"avbd_demo2d_cards_card_{index}_visual",
        )
    bridge.sync()

    contact_history: deque[float] = deque(maxlen=160)
    top_card = cards[-1]
    initial_top_y = float(np.asarray(top_card.translation, dtype=float).reshape(3)[1])

    def build_panel(builder: object, context: object) -> None:
        top_y = float(np.asarray(top_card.translation, dtype=float).reshape(3)[1])
        contacts = len(world.collide())
        contact_history.append(float(contacts))

        builder.text("source corpus: avbd-demo2d Cards")
        builder.text("source scene: sceneCards, index 5 of 19")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"collision shapes: {1 + len(cards)}")
        builder.text(f"contacts: {contacts}")
        builder.text(f"top-card dy: {top_y - initial_top_y:.3f} m")
        builder.text(f"world time: {world.time:.3f} s")
        builder.plot_lines("Contacts", list(contact_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Demo2D Cards", build_panel)],
        info={
            "sx_world": world,
            "ground": ground,
            "cards": cards,
            "card_specs": card_specs,
            "card_size": _CARD_SIZE,
            "source_demo_row": "avbd-demo2d cards",
            "source_demo_reference": _source_row(),
        },
    )


SCENE = PythonDemoScene(
    id="avbd_demo2d_cards",
    title="AVBD Demo2D Cards (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A source-demo Cards row port with a thin-card rigid contact tower.",
    build=build,
)
