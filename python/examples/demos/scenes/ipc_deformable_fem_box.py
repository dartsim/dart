"""FEM slab draping over a box obstacle (IPC deformable solver).

A free tetrahedral FEM slab is dropped onto a static box ("table") resting on
the ground. The box is opted in as a deformable obstacle, so its clamped-log
barrier (along the outward surface normal, a first-class projected-Newton term)
keeps the slab outside the box while the ground barrier catches the overhanging
edges: the slab drapes over the flat top and down the sides intersection-free.
This exercises stable neo-Hookean FEM elasticity (PLAN-081 M1) with the box
obstacle barrier and the ground barrier in one solve.

DART-native FEM showcase -- not a faithful IPC paper-figure reproduction.
"""

from __future__ import annotations

from collections import deque

import numpy as np
import dartpy as sx

from .._ipc_deformable_bridge import IpcDeformableBridge, build_fem_bar
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_BOX_HALF = (0.16, 0.16, 0.1)
_BOX_CENTER = (0.0, 0.0, _BOX_HALF[2])  # resting on the ground top (z = 0)
_GROUND_CENTER = (0.0, 0.0, -0.06)
_GROUND_HALF = (1.2, 1.2, 0.06)


def build() -> SceneSetup:
    options, edges = build_fem_bar(
        cells_x=8,
        cells_y=8,
        cells_z=1,
        cell_size=0.06,
        origin=(-0.24, -0.24, 0.4),
        youngs_modulus=4.0e4,
        poisson_ratio=0.3,
        density=1.0e3,
    )
    options.fixed_nodes = []

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 0.004

    ground = world.add_rigid_body("ground", position=_GROUND_CENTER)
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))
    policy = ground.deformable_obstacle_policy
    policy.ground_barrier = True
    ground.deformable_obstacle_policy = policy

    box = world.add_rigid_body("box", position=_BOX_CENTER)
    box.is_static = True
    box.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))
    policy = box.deformable_obstacle_policy
    policy.surface_obstacle = True
    box.deformable_obstacle_policy = policy

    body = world.add_deformable_body("fem_box", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_fem_box")
    bridge.add_rigid_box_visual(
        _GROUND_CENTER,
        tuple(2.0 * h for h in _GROUND_HALF),
        (0.37, 0.40, 0.43),
        name="ground_visual",
    )
    bridge.add_rigid_box_visual(
        _BOX_CENTER,
        tuple(2.0 * h for h in _BOX_HALF),
        (0.52, 0.45, 0.34),
        name="box_visual",
    )
    bridge.add_deformable_visual(body, name="fem_box", edges=edges)

    box_clearance_history = deque(maxlen=120)
    ground_clearance_history = deque(maxlen=120)
    span_z_history = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        positions = np.asarray(
            [body.node_position(i) for i in range(int(body.node_count))],
            dtype=float,
        )
        builder.text("material: stable neo-Hookean FEM")
        builder.text("contact: box obstacle + ground barrier")
        if positions.size:
            ground_top = _GROUND_CENTER[2] + _GROUND_HALF[2]
            box_top = _BOX_CENTER[2] + _BOX_HALF[2]
            over_box = (
                (np.abs(positions[:, 0] - _BOX_CENTER[0]) <= _BOX_HALF[0])
                & (np.abs(positions[:, 1] - _BOX_CENTER[1]) <= _BOX_HALF[1])
            )
            ground_clearance = float(np.min(positions[:, 2]) - ground_top)
            box_clearance = (
                float(np.min(positions[over_box, 2] - box_top))
                if np.any(over_box)
                else ground_clearance
            )
            span_z = float(np.max(positions[:, 2]) - np.min(positions[:, 2]))
            box_clearance_history.append(box_clearance)
            ground_clearance_history.append(ground_clearance)
            span_z_history.append(span_z)
            builder.text(f"box clearance: {box_clearance:.4f} m")
            builder.text(f"ground clearance: {ground_clearance:.4f} m")
            builder.text(f"vertical span: {span_z:.3f} m")
        builder.separator()
        bridge.build_diagnostics_panel(builder, context)
        if box_clearance_history:
            builder.separator()
            builder.plot_lines("Box clearance", list(box_clearance_history))
            builder.plot_lines("Ground clearance", list(ground_clearance_history))
            builder.plot_lines("Span z", list(span_z_history))

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        panels=[ScenePanel("IPC FEM Box", build_panel)],
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_fem_box",
    title="Deformable FEM over Box (IPC)",
    category="IPC Deformable",
    summary="A FEM slab drapes over a box obstacle via the box barrier + ground barrier.",
    build=build,
)
