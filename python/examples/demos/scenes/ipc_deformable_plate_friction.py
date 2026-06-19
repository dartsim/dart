"""Friction across a barrier-only box plate (IPC deformable solver).

A deformable strip rests on a wide static box plate and is shoved across its top
face. The box is opted into *barrier-only* mode -- it keeps its clamped-log
contact barrier (and so participates in friction) but is excluded from the
surface-CCD line-search limiter, which otherwise scales the whole step and masks
tangential sliding. With friction the strip is brought to rest; without it the
strip coasts across the plate. This is the CCD-free path to box/sphere obstacle
friction (PLAN-081 M5).

DART-native showcase -- not a faithful IPC paper-figure reproduction.
"""

from __future__ import annotations

from collections import deque

import dartpy as sx
import numpy as np

from .._ipc_deformable_bridge import IpcDeformableBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_PLATE_CENTER = (0.0, 0.0, 0.0)
_PLATE_HALF = (2.0, 1.0, 0.5)  # top face at z = 0.5
_FRICTION = 0.8


def _build_strip() -> tuple["sx.DeformableBodyOptions", list[tuple[int, int]]]:
    options = sx.DeformableBodyOptions()
    top_z = _PLATE_HALF[2] + 0.012
    positions = [np.array([-0.8 + 0.15 * i, 0.0, top_z]) for i in range(5)]
    options.positions = positions
    options.masses = [0.1] * 5
    options.edge_stiffness = 200.0
    options.damping = 1.0
    options.velocities = [np.array([2.0, 0.0, 0.0]) for _ in range(5)]
    edges = [(i, i + 1) for i in range(4)]
    options.edges = [
        sx.DeformableEdge(a, b, float(np.linalg.norm(positions[a] - positions[b])))
        for a, b in edges
    ]
    options.material.friction_coefficient = _FRICTION
    return options, edges


def build() -> SceneSetup:
    options, edges = _build_strip()

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 0.004

    plate = world.add_rigid_body("plate", position=_PLATE_CENTER)
    plate.is_static = True
    plate.set_collision_shape(sx.CollisionShape.box(_PLATE_HALF))
    policy = plate.deformable_obstacle_policy
    policy.surface_obstacle = True
    plate.deformable_obstacle_policy = policy
    policy = plate.deformable_obstacle_policy
    policy.barrier_only = True
    plate.deformable_obstacle_policy = policy

    body = world.add_deformable_body("plate_strip", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_plate_friction")
    bridge.add_rigid_box_visual(
        _PLATE_CENTER,
        tuple(2.0 * h for h in _PLATE_HALF),
        (0.37, 0.40, 0.43),
        name="plate_visual",
    )
    bridge.add_deformable_visual(body, name="plate_strip", edges=edges)

    speed_history = deque(maxlen=120)
    clearance_history = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        positions = np.asarray(
            [body.node_position(i) for i in range(int(body.node_count))],
            dtype=float,
        )
        velocities = np.asarray(
            [body.node_velocity(i) for i in range(int(body.node_count))],
            dtype=float,
        )
        if positions.size:
            plate_top = _PLATE_CENTER[2] + _PLATE_HALF[2]
            clearance = float(np.min(positions[:, 2] - plate_top))
            tangent_speed = float(np.mean(np.abs(velocities[:, 0])))
            clearance_history.append(clearance)
            speed_history.append(tangent_speed)
            builder.text(f"friction coefficient: {_FRICTION:.2f}")
            builder.text(f"x speed: {tangent_speed:.3f} m/s")
            builder.text(f"plate clearance: {clearance:.4f} m")
        diagnostics = world.last_deformable_solver_diagnostics
        builder.text(
            f"solver iters: {diagnostics.solver_iterations} | "
            f"friction dissipation: {diagnostics.friction_dissipation:.4f}"
        )
        builder.separator()
        bridge.build_diagnostics_panel(builder, context)
        if speed_history:
            builder.separator()
            builder.plot_lines("X speed", list(speed_history))
            builder.plot_lines("Plate clearance", list(clearance_history))

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        panels=[ScenePanel("IPC Plate Friction", build_panel)],
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_plate_friction",
    title="Deformable Friction on Box Plate (IPC)",
    category="IPC Deformable",
    summary="A deformable strip shoved across a barrier-only box plate is brought "
    "to rest by Coulomb friction.",
    build=build,
)
