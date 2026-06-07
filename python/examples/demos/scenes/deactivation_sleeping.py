"""DART 7 body deactivation: sleeping bodies wake on active contact.

The scene keeps a row of quiet rigid bodies below the deactivation thresholds
long enough for them to sleep, then sends an active striker through the row so
contact wakes each target. A separate quiet reference body stays untouched and
sleeping, making the wake path visible in the GUI diagnostics.
"""

from __future__ import annotations

from collections import deque
from typing import Any

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 1.0 / 120.0
_BOX_HALF = np.array([0.16, 0.16, 0.16], dtype=float)
_LANE_HALF = np.array([2.2, 0.48, 0.02], dtype=float)
_BODY_Z = 0.18

_STATIC_COLOR = (0.36, 0.38, 0.40)
_SLEEPING_COLOR = (0.18, 0.58, 0.82)
_ACTIVE_COLOR = (0.95, 0.53, 0.18)
_STRIKER_COLOR = (0.88, 0.22, 0.18)


def _full_extents(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * np.asarray(half_extents, dtype=float)


def _body_speed(body: object) -> float:
    linear = np.asarray(getattr(body, "linear_velocity", np.zeros(3)), dtype=float)
    angular = np.asarray(getattr(body, "angular_velocity", np.zeros(3)), dtype=float)
    return float(max(np.linalg.norm(linear), np.linalg.norm(angular)))


def _set_frame_color(frame: object, color: tuple[float, float, float]) -> None:
    visual = frame.get_visual_aspect(False)
    if visual is not None:
        visual.set_color(list(color))


def _make_sleep_options(enabled: bool = True, time_until_sleep: float = 0.10):
    return sx.DeactivationOptions(
        enabled=enabled,
        linear_speed_threshold=0.015,
        angular_speed_threshold=0.015,
        generalized_speed_threshold=0.015,
        time_until_sleep=time_until_sleep,
        wake_threshold_scale=2.0,
    )


def build() -> SceneSetup:
    world = sx.World(
        time_step=_TIME_STEP,
        gravity=(0.0, 0.0, 0.0),
        deactivation_options=_make_sleep_options(),
    )

    lane = world.add_rigid_body("sleep_lane", position=(0.50, 0.0, -0.03))
    lane.is_static = True
    lane.set_collision_shape(sx.CollisionShape.box(_LANE_HALF))

    sleepers = []
    for i, x in enumerate((0.0, 0.52, 1.04)):
        body = world.add_rigid_body(
            f"sleep_candidate_{i}",
            position=(x, 0.0, _BODY_Z),
        )
        body.mass = 1.0
        body.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))
        body.friction = 0.2
        body.restitution = 0.0
        sleepers.append(body)

    quiet_reference = world.add_rigid_body(
        "quiet_reference",
        position=(0.52, 0.65, _BODY_Z),
    )
    quiet_reference.mass = 1.0
    quiet_reference.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))
    quiet_reference.friction = 0.2
    quiet_reference.restitution = 0.0
    sleepers.append(quiet_reference)

    striker = world.add_rigid_body(
        "wake_striker",
        position=(-1.25, 0.0, _BODY_Z),
        linear_velocity=(1.35, 0.0, 0.0),
    )
    striker.mass = 1.0
    striker.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))
    striker.friction = 0.2
    striker.restitution = 0.0

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="deactivation_sleeping_render")
    bridge.render_world.set_time_step(_TIME_STEP)
    bridge.add_rigid_body_visual(
        lane,
        dart.BoxShape(_full_extents(_LANE_HALF)),
        _STATIC_COLOR,
        name="sleep_lane_visual",
    )

    body_visuals: list[tuple[Any, Any]] = []
    for body in sleepers:
        frame = bridge.add_rigid_body_visual(
            body,
            dart.BoxShape(_full_extents(_BOX_HALF)),
            _SLEEPING_COLOR,
            name=f"{body.name}_visual",
        )
        body_visuals.append((body, frame))

    striker_frame = bridge.add_rigid_body_visual(
        striker,
        dart.BoxShape(_full_extents(_BOX_HALF)),
        _STRIKER_COLOR,
        name="wake_striker_visual",
    )
    body_visuals.append((striker, striker_frame))

    dynamic_bodies = [*sleepers, striker]
    sleeping_history: deque[float] = deque(maxlen=180)
    contact_history: deque[float] = deque(maxlen=180)
    striker_speed_history: deque[float] = deque(maxlen=180)

    def sync_visuals() -> None:
        for body, frame in body_visuals:
            if body is striker:
                color = _STRIKER_COLOR if not body.is_sleeping else _SLEEPING_COLOR
            elif body.is_sleeping:
                color = _SLEEPING_COLOR
            else:
                color = _ACTIVE_COLOR
            _set_frame_color(frame, color)
        bridge.sync()

    def renderable_provider() -> list[object]:
        sync_visuals()
        return bridge.render_world.renderable_provider()

    def set_deactivation_options(
        *, enabled: bool | None = None, time_until_sleep: float | None = None
    ) -> None:
        options = world.deactivation_options
        if enabled is not None:
            options.enabled = bool(enabled)
        if time_until_sleep is not None:
            options.time_until_sleep = float(time_until_sleep)
        world.deactivation_options = options

    def _table_cell(builder: object, value: str) -> None:
        if builder.table_next_column():
            builder.text(value)

    def build_panel(builder: object, context: object) -> None:
        contacts = len(world.collide())
        sleeping_count = sum(1 for body in sleepers if body.is_sleeping)
        striker_speed = _body_speed(striker)
        sleeping_history.append(float(sleeping_count))
        contact_history.append(float(contacts))
        striker_speed_history.append(striker_speed)

        options = world.deactivation_options
        builder.text("solver: sequential impulse")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"deactivation: {'on' if options.enabled else 'off'}")
        builder.text(f"sleeping candidates: {sleeping_count}/{len(sleepers)}")
        builder.text(f"contacts: {contacts}")
        changed, enabled = builder.checkbox("Enable deactivation", bool(options.enabled))
        if changed:
            set_deactivation_options(enabled=bool(enabled))
            options = world.deactivation_options
        changed, sleep_after = builder.slider(
            "Sleep after", float(options.time_until_sleep), 0.02, 0.40
        )
        if changed:
            set_deactivation_options(time_until_sleep=float(sleep_after))

        builder.plot_lines("Sleeping bodies", list(sleeping_history))
        builder.plot_lines("Contacts", list(contact_history))
        builder.plot_lines("Striker speed", list(striker_speed_history))
        builder.separator()
        if builder.begin_table("Sleep state table", ["Body", "State", "Group", "Speed"]):
            for body in dynamic_bodies:
                builder.table_next_row()
                _table_cell(builder, body.name)
                _table_cell(builder, "sleeping" if body.is_sleeping else "active")
                _table_cell(builder, str(body.deactivation_group_index))
                _table_cell(builder, f"{_body_speed(body):.3f}")
            builder.end_table()
        builder.separator()
        bridge.build_control_panel(builder, context)

    sync_visuals()

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("Body Deactivation", build_panel)],
        renderable_provider=renderable_provider,
        info={
            "sx_world": world,
            "sleep_candidates": sleepers,
            "quiet_reference": quiet_reference,
            "striker": striker,
            "replay_sync": sync_visuals,
        },
    )


SCENE = PythonDemoScene(
    id="deactivation_sleeping",
    title="Body Deactivation",
    category="World Rigid Body",
    summary="Sleeping DART 7 rigid bodies wake only when active contact reaches them.",
    build=build,
)
