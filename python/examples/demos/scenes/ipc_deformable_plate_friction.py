"""Friction across a barrier-only box plate (experimental IPC deformable solver).

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

import dartpy.simulation_experimental as sx
import numpy as np

from .._ipc_deformable_bridge import IpcDeformableBridge
from ..runner import PythonDemoScene, SceneSetup

_PLATE_CENTER = (0.0, 0.0, 0.0)
_PLATE_HALF = (2.0, 1.0, 0.5)  # top face at z = 0.5


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
    options.material.friction_coefficient = 0.8
    return options, edges


def build() -> SceneSetup:
    options, edges = _build_strip()

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 0.004

    plate = world.add_rigid_body("plate", position=_PLATE_CENTER)
    plate.is_static = True
    plate.set_collision_shape(sx.CollisionShape.box(_PLATE_HALF))
    plate.is_deformable_surface_ccd_obstacle = True
    plate.is_deformable_obstacle_barrier_only = True

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

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_plate_friction",
    title="Deformable Friction on Box Plate (IPC)",
    category="IPC Deformable (sx)",
    summary="A deformable strip shoved across a barrier-only box plate is brought "
    "to rest by Coulomb friction.",
    build=build,
)
