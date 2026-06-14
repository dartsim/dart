"""Rigid IPC friction slide: a box slides across static ground and is braked to
rest by friction, all through the rigid implicit-barrier (IPC)
solver (PLAN-082).

This scene complements ``rigid_ipc`` (a box settling straight down). Here the
box starts in contact with a tangential velocity, so the smooth contact barrier
holds it above the ground while lagged Coulomb friction decelerates the slide
without it penetrating or freezing. WorldRenderBridge mirrors the box and ground
into a parallel render World for rendering.
"""

from __future__ import annotations

from collections import deque
import time

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

# sx CollisionShape.box takes half-extents; dart.BoxShape takes full extents.
_GROUND_HALF = (3.0, 1.0, 0.25)
_BOX_HALF = (0.25, 0.25, 0.25)
_FRICTION = 0.5
_INITIAL_SPEED = 2.0
_HISTORY = 120


def _full(half: tuple[float, float, float]) -> np.ndarray:
    return np.array([2.0 * half[0], 2.0 * half[1], 2.0 * half[2]])


def build() -> SceneSetup:
    world = sx.World()
    world.rigid_body_solver = sx.RigidBodySolver.IPC
    world.time_step = 0.005

    # Static ground slab (long in x to give the box room to slide), top at z = 0.
    ground = world.add_rigid_body(
        "ipc_slide_ground", position=(-1.0, 0.0, -_GROUND_HALF[2])
    )
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))
    ground.friction = _FRICTION

    # Box resting on the ground with a tangential velocity; friction brakes it.
    box = world.add_rigid_body("ipc_slide_box", position=(-1.5, 0.0, 0.255))
    box.mass = 1.0
    box.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))
    box.linear_velocity = (_INITIAL_SPEED, 0.0, 0.0)
    box.friction = _FRICTION

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="rigid_ipc_slide_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_full(_GROUND_HALF)),
        (0.7, 0.7, 0.7),
        name="ipc_slide_ground_visual",
    )
    bridge.add_rigid_body_visual(
        box,
        dart.BoxShape(_full(_BOX_HALF)),
        (0.20, 0.55, 0.90),
        name="ipc_slide_box_visual",
    )
    bridge.sync()

    speed_history: deque[float] = deque(maxlen=_HISTORY)
    clearance_history: deque[float] = deque(maxlen=_HISTORY)
    distance_history: deque[float] = deque(maxlen=_HISTORY)
    contact_history: deque[float] = deque(maxlen=_HISTORY)
    step_ms_history: deque[float] = deque(maxlen=_HISTORY)
    last_metrics: dict[str, object] = {}

    def sample_metrics(step_ms: float | None = None) -> dict[str, object]:
        translation = np.asarray(box.translation, dtype=float)
        velocity = np.asarray(box.linear_velocity, dtype=float)
        speed = float(np.linalg.norm(velocity))
        clearance = float(translation[2] - _BOX_HALF[2])
        travel = float(translation[0] + 1.5)
        contact_count = float(len(world.collide()))
        if speed <= 0.05 and clearance <= 0.01:
            status = "braked"
        elif clearance <= 0.01:
            status = "barrier-slide"
        elif speed > 0.05:
            status = "coasting"
        else:
            status = "held"
        return {
            "box_x": float(translation[0]),
            "box_speed": speed,
            "box_vx": float(velocity[0]),
            "clearance": clearance,
            "contact_count": contact_count,
            "status": status,
            "step_ms": float(step_ms or 0.0),
            "travel": travel,
        }

    def record_metrics(step_ms: float | None = None) -> dict[str, object]:
        metrics = sample_metrics(step_ms)
        last_metrics.clear()
        last_metrics.update(metrics)
        speed_history.append(float(metrics["box_speed"]))
        clearance_history.append(float(metrics["clearance"]))
        distance_history.append(float(metrics["travel"]))
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
        clearances = list(clearance_history)
        distances = list(distance_history)
        contacts = list(contact_history)
        step_values = list(step_ms_history)
        return {
            "row": "rigid_ipc_slide",
            "solver": "rigid_ipc",
            "executor": "World.step default",
            "scope": "friction_braked_tangential_slide",
            "time_step_ms": float(world.time_step) * 1000.0,
            "world_time": float(world.time),
            "controls": {
                "friction": float(box.friction),
            },
            "friction": float(box.friction),
            "initial_speed": float(_INITIAL_SPEED),
            **metrics,
            "history_samples": float(len(speeds)),
            "min_speed": min(speeds, default=float(metrics["box_speed"])),
            "max_speed": max(speeds, default=float(metrics["box_speed"])),
            "min_clearance": min(clearances, default=float(metrics["clearance"])),
            "max_travel": max(distances, default=float(metrics["travel"])),
            "max_contact_count": max(contacts, default=0.0),
            "max_step_ms": max(step_values, default=0.0),
        }

    def build_panel(builder: object, context: object) -> None:
        metrics = dict(last_metrics or sample_metrics())
        builder.text("solver: rigid IPC")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"time step: {world.time_step:.4f} s")
        builder.text(f"box speed: {float(metrics['box_speed']):.3f} m/s")
        builder.text(f"barrier gap: {float(metrics['clearance']):.4f} m")
        builder.text(f"travel: {float(metrics['travel']):.3f} m")
        builder.text(f"status: {metrics['status']}")
        changed, friction = builder.slider("Friction", float(box.friction), 0.0, 1.0)
        if changed:
            box.friction = float(friction)
            ground.friction = float(friction)
        builder.plot_lines("Box speed", list(speed_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("Rigid IPC Slide", build_panel)],
        info={
            CAPTURE_METRICS_INFO_KEY: capture_metrics,
            "replay_sync": bridge.sync,
            "replay_live_step_is_stateless": True,
            "sx_world": world,
            "rigid_body_solver": "ipc",
        },
    )


SCENE = PythonDemoScene(
    id="rigid_ipc_slide",
    title="Rigid IPC Friction Slide",
    category="Rigid IPC",
    summary="A box slides across static ground and is friction-braked to rest "
    "via the rigid IPC barrier solver.",
    build=build,
)
