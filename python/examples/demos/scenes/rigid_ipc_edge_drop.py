"""Rigid IPC degenerate edge drop: a cube is released balanced on one edge and
tips through a degenerate edge-on-face barrier configuration under the World
rigid implicit-barrier (IPC) solver (PLAN-082).

This reproduces the spirit of the rigid IPC paper's unit-test / Erleben
degenerate-case figures (Figs. 16-17): the barrier and conservative CCD keep the
contact intersection-free even though the cube first reaches the ground barrier
along a single edge (a degenerate, non-face-aligned configuration) rather than a
flat face.

The cube starts rotated 45 degrees about the x-axis so its lower edge reaches
the ground barrier first; a small angular kick makes the near-edge response
visible while the barrier keeps a small positive separation.
WorldRenderBridge mirrors the bodies into a parallel render World for
rendering.
"""

from __future__ import annotations

from collections import deque
import math
import time

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_GROUND_HALF = (2.0, 2.0, 0.15)
_CUBE_HALF = (0.25, 0.25, 0.25)
_FRICTION = 0.6
_HISTORY = 120
_INITIAL_ANGULAR_SPEED = 0.5


def _full(half: tuple[float, float, float]) -> np.ndarray:
    return np.array([2.0 * half[0], 2.0 * half[1], 2.0 * half[2]])


def _box_min_z(transform: np.ndarray, half: tuple[float, float, float]) -> float:
    corners = [
        np.array([sx_, sy_, sz_, 1.0], dtype=float)
        for sx_ in (-half[0], half[0])
        for sy_ in (-half[1], half[1])
        for sz_ in (-half[2], half[2])
    ]
    return float(min((transform @ corner)[2] for corner in corners))


def _tilt_from_upright(transform: np.ndarray) -> float:
    local_z = np.asarray(transform[:3, 2], dtype=float)
    norm = float(np.linalg.norm(local_z))
    if norm <= 0.0:
        return 0.0
    alignment = abs(float(np.dot(local_z / norm, np.array([0.0, 0.0, 1.0]))))
    return float(math.acos(max(-1.0, min(1.0, alignment))))


def build() -> SceneSetup:
    world = sx.World()
    world.rigid_body_solver = sx.RigidBodySolver.IPC
    world.time_step = 0.005

    # Static ground slab with its top face at z = 0.
    ground = world.add_rigid_body(
        "ipc_edge_ground", position=(0.0, 0.0, -_GROUND_HALF[2])
    )
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))
    ground.friction = _FRICTION

    # Cube rotated 45 degrees about x so it descends onto a single bottom edge.
    # The half-diagonal of the y-z face is half * sqrt(2); start the lowest edge
    # a few millimetres above the ground.
    half = _CUBE_HALF[2]
    edge_height = half * math.sqrt(2.0)
    tilt = (math.cos(math.pi / 8.0), math.sin(math.pi / 8.0), 0.0, 0.0)  # 45 deg / 2

    cube_options = sx.RigidBodyOptions()
    cube_options.mass = 1.0
    cube_options.position = (0.0, 0.0, edge_height + 0.06)
    cube_options.orientation = tilt
    cube = world.add_rigid_body("ipc_edge_cube", cube_options)
    cube.set_collision_shape(sx.CollisionShape.box(_CUBE_HALF))
    cube.friction = _FRICTION
    cube.angular_velocity = (_INITIAL_ANGULAR_SPEED, 0.0, 0.0)

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="rigid_ipc_edge_drop_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_full(_GROUND_HALF)),
        (0.72, 0.72, 0.74),
        name="ipc_edge_ground_visual",
    )
    bridge.add_rigid_body_visual(
        cube,
        dart.BoxShape(_full(_CUBE_HALF)),
        (0.85, 0.30, 0.32),
        name="ipc_edge_cube_visual",
    )
    bridge.sync()

    clearance_history: deque[float] = deque(maxlen=_HISTORY)
    contact_history: deque[float] = deque(maxlen=_HISTORY)
    tilt_history: deque[float] = deque(maxlen=_HISTORY)
    angular_speed_history: deque[float] = deque(maxlen=_HISTORY)
    step_ms_history: deque[float] = deque(maxlen=_HISTORY)
    last_metrics: dict[str, object] = {}

    def sample_metrics(step_ms: float | None = None) -> dict[str, object]:
        transform = np.asarray(cube.transform, dtype=float)
        translation = np.asarray(cube.translation, dtype=float)
        linear_velocity = np.asarray(cube.linear_velocity, dtype=float)
        angular_velocity = np.asarray(cube.angular_velocity, dtype=float)
        min_corner_z = _box_min_z(transform, _CUBE_HALF)
        clearance = float(min_corner_z)
        contact_count = float(len(world.collide()))
        tilt_rad = _tilt_from_upright(transform)
        angular_speed = float(np.linalg.norm(angular_velocity))
        vertical_speed = float(linear_velocity[2])
        near_edge_barrier = clearance <= 0.01
        if near_edge_barrier and angular_speed > 0.05:
            status = "edge-barrier"
        elif near_edge_barrier and abs(vertical_speed) <= 0.02:
            status = "barrier-held"
        elif clearance > 0.01 and contact_count == 0.0:
            status = "airborne"
        elif contact_count > 0.0 and tilt_rad > 0.20 and angular_speed > 0.05:
            status = "edge-contact"
        elif contact_count > 0.0 and angular_speed <= 0.05:
            status = "settled"
        elif contact_count > 0.0:
            status = "barrier-contact"
        else:
            status = "approaching"
        return {
            "angular_speed": angular_speed,
            "clearance": clearance,
            "contact_count": contact_count,
            "cube_z": float(translation[2]),
            "status": status,
            "step_ms": float(step_ms or 0.0),
            "tilt_deg": math.degrees(tilt_rad),
            "tilt_rad": tilt_rad,
            "vertical_speed": vertical_speed,
        }

    def record_metrics(step_ms: float | None = None) -> dict[str, object]:
        metrics = sample_metrics(step_ms)
        last_metrics.clear()
        last_metrics.update(metrics)
        clearance_history.append(float(metrics["clearance"]))
        contact_history.append(float(metrics["contact_count"]))
        tilt_history.append(float(metrics["tilt_deg"]))
        angular_speed_history.append(float(metrics["angular_speed"]))
        step_ms_history.append(float(metrics["step_ms"]))
        return metrics

    record_metrics(0.0)

    def pre_step() -> None:
        start = time.perf_counter()
        bridge.pre_step()
        record_metrics((time.perf_counter() - start) * 1000.0)

    def capture_metrics() -> dict[str, object]:
        metrics = dict(last_metrics or sample_metrics())
        clearance_values = list(clearance_history)
        contact_values = list(contact_history)
        tilt_values = list(tilt_history)
        angular_values = list(angular_speed_history)
        step_values = list(step_ms_history)
        return {
            "row": "rigid_ipc_edge_drop",
            "related_source_row": "rigid_solver_compare",
            "solver": "rigid_ipc",
            "executor": "World.step default",
            "scope": "degenerate_edge_contact_capability",
            "time_step_ms": float(world.time_step) * 1000.0,
            "world_time": float(world.time),
            "friction": float(_FRICTION),
            "initial_angular_speed": float(_INITIAL_ANGULAR_SPEED),
            **metrics,
            "history_samples": float(len(clearance_values)),
            "min_barrier_gap": min(
                clearance_values, default=float(metrics["clearance"])
            ),
            "min_clearance": min(clearance_values, default=float(metrics["clearance"])),
            "max_contact_count": max(contact_values, default=0.0),
            "max_tilt_deg": max(tilt_values, default=float(metrics["tilt_deg"])),
            "min_tilt_deg": min(tilt_values, default=float(metrics["tilt_deg"])),
            "max_angular_speed": max(
                angular_values, default=float(metrics["angular_speed"])
            ),
            "max_step_ms": max(step_values, default=0.0),
        }

    def build_panel(builder: object, context: object) -> None:
        metrics = dict(last_metrics or sample_metrics())
        builder.text("solver: rigid IPC")
        builder.text("related row: rigid_solver_compare")
        builder.text("scope: degenerate edge barrier")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"time step: {world.time_step:.4f} s")
        builder.text(f"initial spin: {_INITIAL_ANGULAR_SPEED:.2f} rad/s")
        builder.text(f"contact count: {float(metrics['contact_count']):.0f}")
        builder.text(f"barrier gap: {float(metrics['clearance']):.4f} m")
        builder.text(f"tilt: {float(metrics['tilt_deg']):.1f} deg")
        builder.text(f"angular speed: {float(metrics['angular_speed']):.3f} rad/s")
        builder.text(f"status: {metrics['status']}")
        builder.plot_lines("Barrier gap", list(clearance_history))
        builder.plot_lines("Tilt deg", list(tilt_history))
        builder.plot_lines("Angular speed", list(angular_speed_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("Rigid IPC Edge Drop", build_panel)],
        info={
            CAPTURE_METRICS_INFO_KEY: capture_metrics,
            "physics_world": world,
            "replay_sync": bridge.sync,
            "replay_live_step_is_stateless": True,
            "rigid_body_solver": "ipc",
            "sx_world": world,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_ipc_edge_drop",
    title="Rigid IPC Degenerate Edge Drop",
    category="Rigid IPC",
    summary="A cube tips through a degenerate edge-on-ground barrier "
    "configuration while staying intersection-free via rigid IPC.",
    build=build,
)
