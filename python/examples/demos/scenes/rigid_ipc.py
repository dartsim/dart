"""Rigid IPC contact: a free box settles onto static ground via the rigid
implicit-barrier (IPC) solver (PLAN-082).

World owns the physics with the opt-in rigid IPC solver selected
(`RigidBodySolver.IPC`), so the falling box is held above the ground by the
smooth contact barrier rather than by sequential impulses. WorldRenderBridge
mirrors the box and ground into a parallel render World for rendering.
"""

from __future__ import annotations

from collections import deque
import time

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

# sx CollisionShape.box takes half-extents; dart.BoxShape takes full extents.
_GROUND_HALF = (2.0, 2.0, 0.25)
_BOX_HALF = (0.25, 0.25, 0.25)
_HISTORY = 120


def _full(half: tuple[float, float, float]) -> np.ndarray:
    return np.array([2.0 * half[0], 2.0 * half[1], 2.0 * half[2]])


def build() -> SceneSetup:
    world = sx.World()
    world.rigid_body_solver = sx.RigidBodySolver.IPC
    world.time_step = 0.005

    # Static ground slab with its top face at z = 0.
    ground = world.add_rigid_body("ipc_ground", position=(0.0, 0.0, -_GROUND_HALF[2]))
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))

    # Free box released just above the ground; the IPC barrier catches it.
    box = world.add_rigid_body("ipc_box", position=(0.0, 0.0, 0.6))
    box.mass = 1.0
    box.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="rigid_ipc_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_full(_GROUND_HALF)),
        (0.7, 0.7, 0.7),
        name="ipc_ground_visual",
    )
    bridge.add_rigid_body_visual(
        box, dart.BoxShape(_full(_BOX_HALF)), (0.90, 0.45, 0.20), name="ipc_box_visual"
    )
    bridge.sync()

    height_history: deque[float] = deque(maxlen=_HISTORY)
    speed_history: deque[float] = deque(maxlen=_HISTORY)
    clearance_history: deque[float] = deque(maxlen=_HISTORY)
    contact_history: deque[float] = deque(maxlen=_HISTORY)
    step_ms_history: deque[float] = deque(maxlen=_HISTORY)
    last_metrics: dict[str, object] = {}

    def sample_metrics(step_ms: float | None = None) -> dict[str, object]:
        height = float(np.asarray(box.translation, dtype=float)[2])
        speed = float(np.linalg.norm(np.asarray(box.linear_velocity, dtype=float)))
        clearance = height - _BOX_HALF[2]
        contact_count = float(len(world.collide()))
        if clearance <= 0.01 and speed <= 0.05:
            status = "barrier-held"
        elif clearance <= 0.01:
            status = "near-barrier"
        elif speed > 0.05:
            status = "falling"
        else:
            status = "released"
        return {
            "box_height": height,
            "box_speed": speed,
            "clearance": float(clearance),
            "contact_count": contact_count,
            "status": status,
            "step_ms": float(step_ms or 0.0),
        }

    def record_metrics(step_ms: float | None = None) -> dict[str, object]:
        metrics = sample_metrics(step_ms)
        last_metrics.clear()
        last_metrics.update(metrics)
        height_history.append(float(metrics["box_height"]))
        speed_history.append(float(metrics["box_speed"]))
        clearance_history.append(float(metrics["clearance"]))
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
        heights = list(height_history)
        speeds = list(speed_history)
        clearances = list(clearance_history)
        contacts = list(contact_history)
        step_values = list(step_ms_history)
        return {
            "row": "rigid_ipc",
            "solver": "rigid_ipc",
            "executor": "World.step default",
            "scope": "basic_box_ground_barrier_settle",
            "time_step_ms": float(world.time_step) * 1000.0,
            "world_time": float(world.time),
            "controls": {
                "friction": float(box.friction),
            },
            "friction": float(box.friction),
            **metrics,
            "history_samples": float(len(heights)),
            "min_height": min(heights, default=float(metrics["box_height"])),
            "min_clearance": min(clearances, default=float(metrics["clearance"])),
            "max_speed": max(speeds, default=float(metrics["box_speed"])),
            "max_contact_count": max(contacts, default=0.0),
            "max_step_ms": max(step_values, default=0.0),
        }

    def build_panel(builder: object, context: object) -> None:
        metrics = dict(last_metrics or sample_metrics())

        builder.text("solver: rigid IPC")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"time step: {world.time_step:.4f} s")
        builder.text(f"box height: {float(metrics['box_height']):.3f} m")
        builder.text(f"box speed: {float(metrics['box_speed']):.3f} m/s")
        builder.text(f"barrier gap: {float(metrics['clearance']):.4f} m")
        builder.text(f"status: {metrics['status']}")
        changed, friction = builder.slider("Friction", float(box.friction), 0.0, 1.0)
        if changed:
            box.friction = float(friction)
            ground.friction = float(friction)
        builder.plot_lines("Box height", list(height_history))
        builder.plot_lines("Box speed", list(speed_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("Rigid IPC Contact", build_panel)],
        info={
            CAPTURE_METRICS_INFO_KEY: capture_metrics,
            "replay_sync": bridge.sync,
            "replay_live_step_is_stateless": True,
            "sx_world": world,
            "rigid_body_solver": "ipc",
        },
    )


SCENE = PythonDemoScene(
    id="rigid_ipc",
    title="Rigid IPC Contact",
    category="Rigid IPC",
    summary="A free box settles on static ground via the rigid IPC barrier solver.",
    build=build,
)
