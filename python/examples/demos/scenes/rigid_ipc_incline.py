"""Rigid IPC inclined slide: a box slides down a tilted static ramp under
gravity and friction through the rigid implicit-barrier (IPC)
solver (PLAN-082).

The ramp and box are both rotated about the y-axis so the contact is a tilted
face-face barrier; lagged Coulomb friction sets how fast the box accelerates
down-slope. WorldRenderBridge mirrors the bodies into a parallel
render World for rendering.
"""

from __future__ import annotations

import math
from collections import deque
import time

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_RAMP_HALF = (2.0, 1.0, 0.1)
_BOX_HALF = (0.2, 0.2, 0.2)
_TILT_RAD = math.radians(20.0)
_FRICTION = 0.2
_HISTORY = 120


def _full(half: tuple[float, float, float]) -> np.ndarray:
    return np.array([2.0 * half[0], 2.0 * half[1], 2.0 * half[2]])


def _tilt_quaternion() -> tuple[float, float, float, float]:
    # Rotation about the y-axis by the tilt angle, as (w, x, y, z).
    return (math.cos(_TILT_RAD / 2.0), 0.0, math.sin(_TILT_RAD / 2.0), 0.0)


def _down_slope_axis() -> np.ndarray:
    axis = np.array([math.cos(_TILT_RAD), 0.0, -math.sin(_TILT_RAD)])
    return axis / np.linalg.norm(axis)


def build() -> SceneSetup:
    world = sx.World()
    world.rigid_body_solver = sx.RigidBodySolver.IPC
    world.time_step = 0.005

    tilt = _tilt_quaternion()

    ramp_options = sx.RigidBodyOptions()
    ramp_options.is_static = True
    ramp_options.position = (0.0, 0.0, 0.0)
    ramp_options.orientation = tilt
    ramp = world.add_rigid_body("ipc_incline_ramp", ramp_options)
    ramp.set_collision_shape(sx.CollisionShape.box(_RAMP_HALF))
    ramp.friction = _FRICTION

    # Place the box just above the ramp's top face along its (tilted) normal.
    normal = (math.sin(_TILT_RAD), 0.0, math.cos(_TILT_RAD))
    offset = _RAMP_HALF[2] + _BOX_HALF[2] + 0.004
    box_options = sx.RigidBodyOptions()
    box_options.mass = 1.0
    box_options.orientation = tilt
    box_options.position = (normal[0] * offset, 0.0, normal[2] * offset)
    box = world.add_rigid_body("ipc_incline_box", box_options)
    box.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))
    box.friction = _FRICTION

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="rigid_ipc_incline_render")
    bridge.add_rigid_body_visual(
        ramp,
        dart.BoxShape(_full(_RAMP_HALF)),
        (0.7, 0.7, 0.7),
        name="ipc_incline_ramp_visual",
    )
    bridge.add_rigid_body_visual(
        box,
        dart.BoxShape(_full(_BOX_HALF)),
        (0.95, 0.65, 0.15),
        name="ipc_incline_box_visual",
    )
    bridge.sync()

    down_slope = _down_slope_axis()
    initial_position = np.asarray(box.translation, dtype=float)
    ramp_normal = np.array([math.sin(_TILT_RAD), 0.0, math.cos(_TILT_RAD)])
    contact_offset = _RAMP_HALF[2] + _BOX_HALF[2]
    speed_history: deque[float] = deque(maxlen=_HISTORY)
    gap_history: deque[float] = deque(maxlen=_HISTORY)
    travel_history: deque[float] = deque(maxlen=_HISTORY)
    contact_history: deque[float] = deque(maxlen=_HISTORY)
    step_ms_history: deque[float] = deque(maxlen=_HISTORY)
    last_metrics: dict[str, object] = {}

    def sample_metrics(step_ms: float | None = None) -> dict[str, object]:
        translation = np.asarray(box.translation, dtype=float)
        velocity = np.asarray(box.linear_velocity, dtype=float)
        down_slope_speed = float(np.dot(velocity, down_slope))
        down_slope_travel = float(np.dot(translation - initial_position, down_slope))
        ramp_gap = float(np.dot(translation, ramp_normal) - contact_offset)
        contact_count = float(len(world.collide()))
        if ramp_gap <= 0.01 and down_slope_speed > 0.05:
            status = "barrier-slide"
        elif ramp_gap <= 0.01:
            status = "barrier-held"
        elif down_slope_speed > 0.05:
            status = "sliding"
        else:
            status = "released"
        return {
            "box_x": float(translation[0]),
            "box_z": float(translation[2]),
            "contact_count": contact_count,
            "down_slope_speed": down_slope_speed,
            "down_slope_travel": down_slope_travel,
            "ramp_gap": ramp_gap,
            "status": status,
            "step_ms": float(step_ms or 0.0),
        }

    def record_metrics(step_ms: float | None = None) -> dict[str, object]:
        metrics = sample_metrics(step_ms)
        last_metrics.clear()
        last_metrics.update(metrics)
        speed_history.append(float(metrics["down_slope_speed"]))
        gap_history.append(float(metrics["ramp_gap"]))
        travel_history.append(float(metrics["down_slope_travel"]))
        contact_history.append(float(metrics["contact_count"]))
        step_ms_history.append(float(metrics["step_ms"]))
        return metrics

    record_metrics(0.0)

    def pre_step() -> None:
        start = time.perf_counter()
        bridge.pre_step()
        record_metrics((time.perf_counter() - start) * 1000.0)

    def capture_metrics() -> dict[str, object]:
        metrics = dict(last_metrics or sample_metrics())
        speeds = list(speed_history)
        gaps = list(gap_history)
        travels = list(travel_history)
        contacts = list(contact_history)
        step_values = list(step_ms_history)
        return {
            "row": "rigid_ipc_incline",
            "solver": "rigid_ipc",
            "executor": "World.step default",
            "scope": "tilted_face_friction_slide",
            "time_step_ms": float(world.time_step) * 1000.0,
            "world_time": float(world.time),
            "friction": float(box.friction),
            "tilt_deg": math.degrees(_TILT_RAD),
            **metrics,
            "history_samples": float(len(speeds)),
            "min_ramp_gap": min(gaps, default=float(metrics["ramp_gap"])),
            "max_down_slope_speed": max(
                speeds, default=float(metrics["down_slope_speed"])
            ),
            "max_down_slope_travel": max(
                travels, default=float(metrics["down_slope_travel"])
            ),
            "max_contact_count": max(contacts, default=0.0),
            "max_step_ms": max(step_values, default=0.0),
        }

    def build_panel(builder: object, context: object) -> None:
        metrics = dict(last_metrics or sample_metrics())
        builder.text("solver: rigid IPC")
        builder.text(f"tilt: {math.degrees(_TILT_RAD):.1f} deg")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(
            f"down-slope speed: {float(metrics['down_slope_speed']):.3f} m/s"
        )
        builder.text(f"ramp gap: {float(metrics['ramp_gap']):.4f} m")
        builder.text(f"travel: {float(metrics['down_slope_travel']):.3f} m")
        builder.text(f"status: {metrics['status']}")
        changed, friction = builder.slider("Friction", float(box.friction), 0.0, 1.0)
        if changed:
            box.friction = float(friction)
            ramp.friction = float(friction)
        builder.plot_lines("Speed", list(speed_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("Rigid IPC Incline", build_panel)],
        info={
            CAPTURE_METRICS_INFO_KEY: capture_metrics,
            "replay_sync": bridge.sync,
            "replay_live_step_is_stateless": True,
            "sx_world": world,
            "rigid_body_solver": "ipc",
        },
    )


SCENE = PythonDemoScene(
    id="rigid_ipc_incline",
    title="Rigid IPC Inclined Slide",
    category="Rigid IPC",
    summary="A box slides down a tilted ramp under friction via the rigid IPC barrier solver.",
    build=build,
)
