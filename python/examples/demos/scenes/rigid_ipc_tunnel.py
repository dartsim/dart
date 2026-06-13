"""Rigid IPC intersection-free guarantee: a fast box is hurled at a thin static
wall and stopped dead by continuous collision detection instead of tunneling
through it -- the headline property of intersection-free rigid IPC (PLAN-082).

A single discrete contact step would let an 8 m/s box jump past a thin wall
between frames; the rigid IPC conservative line search refuses any step that
would cross the wall, so the box is always caught. WorldRenderBridge mirrors the
bodies into a parallel render World for rendering.
"""

from __future__ import annotations

from collections import deque
import time

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_WALL_HALF = (0.05, 1.0, 1.0)
_BOX_HALF = (0.2, 0.2, 0.2)
_IMPACT_SPEED = 8.0


def _full(half: tuple[float, float, float]) -> np.ndarray:
    return np.array([2.0 * half[0], 2.0 * half[1], 2.0 * half[2]])


def build() -> SceneSetup:
    world = sx.World()
    world.rigid_body_solver = sx.RigidBodySolver.IPC
    world.time_step = 0.01

    wall = world.add_rigid_body("ipc_tunnel_wall", position=(1.0, 0.0, 0.0))
    wall.is_static = True
    wall.set_collision_shape(sx.CollisionShape.box(_WALL_HALF))

    box = world.add_rigid_body("ipc_tunnel_box", position=(0.0, 0.0, 0.0))
    box.mass = 1.0
    box.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))
    box.linear_velocity = (_IMPACT_SPEED, 0.0, 0.0)

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="rigid_ipc_tunnel_render")
    bridge.add_rigid_body_visual(
        wall,
        dart.BoxShape(_full(_WALL_HALF)),
        (0.7, 0.7, 0.7),
        name="ipc_tunnel_wall_visual",
    )
    bridge.add_rigid_body_visual(
        box,
        dart.BoxShape(_full(_BOX_HALF)),
        (0.90, 0.25, 0.25),
        name="ipc_tunnel_box_visual",
    )
    bridge.sync()

    wall_left_face = 1.0 - _WALL_HALF[0]
    wall_right_face = 1.0 + _WALL_HALF[0]
    speed_history: deque[float] = deque(maxlen=120)
    clearance_history: deque[float] = deque(maxlen=120)
    tunnel_margin_history: deque[float] = deque(maxlen=120)
    step_ms_history: deque[float] = deque(maxlen=120)
    contact_history: deque[float] = deque(maxlen=120)
    last_metrics: dict[str, object] = {}

    def sample_metrics(step_ms: float | None = None) -> dict[str, object]:
        velocity = np.asarray(box.linear_velocity, dtype=float)
        translation = np.asarray(box.translation, dtype=float)
        box_x = float(translation[0])
        leading_face = box_x + _BOX_HALF[0]
        trailing_face = box_x - _BOX_HALF[0]
        clearance = float(wall_left_face - leading_face)
        tunnel_margin = float(wall_right_face - trailing_face)
        contact_count = float(len(world.collide()))
        speed_x = float(velocity[0])
        speed = float(np.linalg.norm(velocity))
        if tunnel_margin < 0.0:
            status = "tunneled"
        elif clearance <= 0.01 and abs(speed_x) < 0.5:
            status = "barrier-held"
        elif clearance <= 0.05:
            status = "barrier-active"
        else:
            status = "approaching"
        return {
            "box_x": box_x,
            "box_speed": speed,
            "box_vx": speed_x,
            "clearance": clearance,
            "contact_count": contact_count,
            "leading_face_x": leading_face,
            "status": status,
            "step_ms": float(step_ms or 0.0),
            "trailing_face_x": trailing_face,
            "tunnel_margin": tunnel_margin,
            "wall_left_face_x": float(wall_left_face),
            "wall_right_face_x": float(wall_right_face),
        }

    def record_metrics(step_ms: float | None = None) -> dict[str, object]:
        metrics = sample_metrics(step_ms)
        last_metrics.clear()
        last_metrics.update(metrics)
        speed_history.append(float(metrics["box_vx"]))
        clearance_history.append(float(metrics["clearance"]))
        tunnel_margin_history.append(float(metrics["tunnel_margin"]))
        step_ms_history.append(float(metrics["step_ms"]))
        contact_history.append(float(metrics["contact_count"]))
        return metrics

    record_metrics(0.0)

    def pre_step() -> None:
        start = time.perf_counter()
        bridge.pre_step()
        record_metrics((time.perf_counter() - start) * 1000.0)

    def capture_metrics() -> dict[str, object]:
        metrics = dict(last_metrics or sample_metrics())
        clearance_values = list(clearance_history)
        tunnel_margin_values = list(tunnel_margin_history)
        speed_values = list(speed_history)
        step_values = list(step_ms_history)
        contact_values = list(contact_history)
        return {
            "row": "rigid_ipc_tunnel",
            "related_source_row": "rigid_solver_compare",
            "solver": "rigid_ipc",
            "executor": "World.step default",
            "scope": "focused_no_tunneling_capability",
            "time_step_ms": float(world.time_step) * 1000.0,
            "world_time": float(world.time),
            "launch_speed": float(_IMPACT_SPEED),
            "wall_half_extent_x": float(_WALL_HALF[0]),
            "box_half_extent_x": float(_BOX_HALF[0]),
            **metrics,
            "min_clearance": min(clearance_values, default=float(metrics["clearance"])),
            "min_tunnel_margin": min(
                tunnel_margin_values, default=float(metrics["tunnel_margin"])
            ),
            "max_wall_crossing": max(
                (max(-value, 0.0) for value in clearance_values),
                default=max(-float(metrics["clearance"]), 0.0),
            ),
            "max_contact_count": max(contact_values, default=0.0),
            "max_step_ms": max(step_values, default=0.0),
            "min_box_vx": min(speed_values, default=float(metrics["box_vx"])),
            "history_samples": float(len(clearance_values)),
        }

    def build_panel(builder: object, context: object) -> None:
        metrics = dict(last_metrics or sample_metrics())
        builder.text("solver: rigid IPC")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"time step: {world.time_step:.4f} s")
        builder.text(f"launch speed: {_IMPACT_SPEED:.1f} m/s")
        builder.text(f"box vx: {float(metrics['box_vx']):.3f} m/s")
        builder.text(f"wall clearance: {float(metrics['clearance']):.3f} m")
        builder.text(f"tunnel margin: {float(metrics['tunnel_margin']):.3f} m")
        builder.text(f"status: {metrics['status']}")
        builder.plot_lines("Box vx", list(speed_history))
        builder.plot_lines("Clearance", list(clearance_history))
        builder.plot_lines("Tunnel margin", list(tunnel_margin_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("Rigid IPC Tunnel", build_panel)],
        info={
            CAPTURE_METRICS_INFO_KEY: capture_metrics,
            "replay_sync": bridge.sync,
            "replay_live_step_is_stateless": True,
            "rigid_body_solver": "ipc",
            "rigid_ipc_tunnel_capture_first": False,
            "sx_world": world,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_ipc_tunnel",
    title="Rigid IPC No-Tunneling",
    category="Rigid IPC",
    summary=(
        "A fast box is stopped by a thin wall via the rigid IPC continuous "
        "collision detection (no tunneling)."
    ),
    build=build,
)
