"""Rigid IPC degenerate edge drop: a cube is released balanced on one edge and
lands in a degenerate edge-on-face contact before tipping to rest, through the
World rigid implicit-barrier (IPC) solver (PLAN-082).

This reproduces the spirit of the rigid IPC paper's unit-test / Erleben
degenerate-case figures (Figs. 16-17): the barrier and conservative CCD keep the
contact intersection-free even though the cube first touches the ground along a
single edge (a degenerate, non-face-aligned contact) rather than a flat face.

The cube starts rotated 45 degrees about the x-axis so its lower edge meets the
ground first; friction then lets it tip down onto a face and settle.
WorldRenderBridge mirrors the bodies into a parallel render World for
rendering.
"""

from __future__ import annotations

import math

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, SceneSetup

_GROUND_HALF = (2.0, 2.0, 0.15)
_CUBE_HALF = (0.25, 0.25, 0.25)
_FRICTION = 0.6


def _full(half: tuple[float, float, float]) -> np.ndarray:
    return np.array([2.0 * half[0], 2.0 * half[1], 2.0 * half[2]])


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

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"physics_world": world, "rigid_body_solver": "ipc"},
    )


SCENE = PythonDemoScene(
    id="rigid_ipc_edge_drop",
    title="Rigid IPC Degenerate Edge Drop",
    category="Rigid IPC",
    summary="A cube lands on one edge in a degenerate contact and tips to rest, "
    "staying intersection-free via the rigid IPC barrier solver.",
    build=build,
)
