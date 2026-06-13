"""Rigid IPC pile: three free boxes dropped from staggered heights land and
settle into a pile on static ground through the rigid implicit-barrier (IPC)
solver (PLAN-082).

Unlike the neatly aligned ``rigid_ipc_stack``, the boxes start spread out and
at different heights, so they fall, collide, and settle into an irregular pile --
exercising several simultaneous body-ground and body-body barrier contacts.
WorldRenderBridge mirrors the bodies into a parallel render World for
rendering.
"""

from __future__ import annotations

from collections import deque
import time

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_GROUND_HALF = (2.0, 2.0, 0.25)
_BOX_HALF = (0.15, 0.15, 0.15)
# (x, z) drop positions and per-box colors.
_DROPS = ((-0.35, 0.5), (0.0, 0.7), (0.35, 0.9))
_COLORS = ((0.90, 0.45, 0.20), (0.30, 0.70, 0.45), (0.25, 0.50, 0.85))
_FRICTION = 0.45
_HISTORY = 120


def _full(half: tuple[float, float, float]) -> np.ndarray:
    return np.array([2.0 * half[0], 2.0 * half[1], 2.0 * half[2]])


def build() -> SceneSetup:
    world = sx.World()
    world.rigid_body_solver = sx.RigidBodySolver.IPC
    world.time_step = 0.005

    ground = world.add_rigid_body(
        "ipc_pile_ground", position=(0.0, 0.0, -_GROUND_HALF[2])
    )
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))
    ground.friction = _FRICTION

    boxes = []
    for i, (x, z) in enumerate(_DROPS):
        box = world.add_rigid_body(f"ipc_pile_box{i}", position=(x, 0.0, z))
        box.mass = 1.0
        box.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))
        box.friction = _FRICTION
        boxes.append(box)

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="rigid_ipc_pile_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_full(_GROUND_HALF)),
        (0.7, 0.7, 0.7),
        name="ipc_pile_ground_visual",
    )
    for i, box in enumerate(boxes):
        bridge.add_rigid_body_visual(
            box,
            dart.BoxShape(_full(_BOX_HALF)),
            _COLORS[i],
            name=f"ipc_pile_box{i}_visual",
        )
    bridge.sync()

    speed_history: deque[float] = deque(maxlen=_HISTORY)
    height_history: deque[float] = deque(maxlen=_HISTORY)
    clearance_history: deque[float] = deque(maxlen=_HISTORY)
    contact_history: deque[float] = deque(maxlen=_HISTORY)
    span_history: deque[float] = deque(maxlen=_HISTORY)
    step_ms_history: deque[float] = deque(maxlen=_HISTORY)
    last_metrics: dict[str, object] = {}

    def sample_metrics(step_ms: float | None = None) -> dict[str, object]:
        speeds = [
            float(np.linalg.norm(np.asarray(box.linear_velocity, dtype=float)))
            for box in boxes
        ]
        positions = [np.asarray(box.translation, dtype=float) for box in boxes]
        heights = [float(position[2]) for position in positions]
        clearances = [height - _BOX_HALF[2] for height in heights]
        xs = [float(position[0]) for position in positions]
        max_speed = max(speeds) if speeds else 0.0
        mean_height = float(np.mean(heights)) if heights else 0.0
        min_clearance = min(clearances) if clearances else 0.0
        span_x = max(xs) - min(xs) if xs else 0.0
        contact_count = float(len(world.collide()))
        if min_clearance <= 0.01 and max_speed <= 0.05:
            status = "settled"
        elif min_clearance <= 0.01:
            status = "barrier-pile"
        else:
            status = "falling"
        return {
            "box_count": float(len(boxes)),
            "contact_count": contact_count,
            "max_speed": max_speed,
            "mean_height": mean_height,
            "min_clearance": float(min_clearance),
            "min_height": min(heights) if heights else 0.0,
            "span_x": float(span_x),
            "status": status,
            "step_ms": float(step_ms or 0.0),
        }

    def record_metrics(step_ms: float | None = None) -> dict[str, object]:
        metrics = sample_metrics(step_ms)
        last_metrics.clear()
        last_metrics.update(metrics)
        speed_history.append(float(metrics["max_speed"]))
        height_history.append(float(metrics["mean_height"]))
        clearance_history.append(float(metrics["min_clearance"]))
        contact_history.append(float(metrics["contact_count"]))
        span_history.append(float(metrics["span_x"]))
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
        heights = list(height_history)
        clearances = list(clearance_history)
        contacts = list(contact_history)
        spans = list(span_history)
        step_values = list(step_ms_history)
        return {
            "row": "rigid_ipc_pile",
            "solver": "rigid_ipc",
            "executor": "World.step default",
            "scope": "multi_box_barrier_pile",
            "time_step_ms": float(world.time_step) * 1000.0,
            "world_time": float(world.time),
            "controls": {
                "friction": float(ground.friction),
            },
            "friction": float(ground.friction),
            **metrics,
            "history_samples": float(len(speeds)),
            "max_history_speed": max(speeds, default=float(metrics["max_speed"])),
            "min_mean_height": min(heights, default=float(metrics["mean_height"])),
            "min_history_clearance": min(
                clearances, default=float(metrics["min_clearance"])
            ),
            "max_contact_count": max(contacts, default=0.0),
            "max_span_x": max(spans, default=float(metrics["span_x"])),
            "max_step_ms": max(step_values, default=0.0),
        }

    def build_panel(builder: object, context: object) -> None:
        metrics = dict(last_metrics or sample_metrics())
        builder.text("solver: rigid IPC")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"time step: {world.time_step:.4f} s")
        builder.text(
            f"boxes: {len(boxes)} | max speed: {float(metrics['max_speed']):.3f} m/s"
        )
        builder.text(f"mean height: {float(metrics['mean_height']):.3f} m")
        builder.text(f"min barrier gap: {float(metrics['min_clearance']):.4f} m")
        builder.text(f"contact count: {float(metrics['contact_count']):.0f}")
        builder.text(f"status: {metrics['status']}")
        changed, friction = builder.slider("Friction", float(ground.friction), 0.0, 1.0)
        if changed:
            ground.friction = float(friction)
            for box in boxes:
                box.friction = float(friction)
        builder.plot_lines("Max speed", list(speed_history))
        builder.plot_lines("Mean height", list(height_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("Rigid IPC Pile", build_panel)],
        info={
            CAPTURE_METRICS_INFO_KEY: capture_metrics,
            "replay_sync": bridge.sync,
            "replay_live_step_is_stateless": True,
            "sx_world": world,
            "rigid_body_solver": "ipc",
        },
    )


SCENE = PythonDemoScene(
    id="rigid_ipc_pile",
    title="Rigid IPC Box Pile",
    category="Rigid IPC",
    summary="Three boxes drop and settle into a pile via the rigid IPC barrier solver.",
    build=build,
)
