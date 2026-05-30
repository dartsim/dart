"""Rigid IPC intersection-free guarantee: a fast box is hurled at a thin static
wall and stopped dead by continuous collision detection instead of tunneling
through it -- the headline property of intersection-free rigid IPC (PLAN-082).

A single discrete contact step would let an 8 m/s box jump past a thin wall
between frames; the rigid IPC conservative line search refuses any step that
would cross the wall, so the box is always caught. SxRenderBridge mirrors the
bodies into a parallel dart.simulation.World for rendering.
"""

from __future__ import annotations

import dartpy as dart
import dartpy.simulation_experimental as sx
import numpy as np

from .._sx_bridge import SxRenderBridge
from ..runner import PythonDemoScene, SceneSetup

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

    bridge = SxRenderBridge(world, name="sx_rigid_ipc_tunnel_render")
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

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "rigid_body_solver": "ipc"},
    )


SCENE = PythonDemoScene(
    id="sx_rigid_ipc_tunnel",
    title="Rigid IPC No-Tunneling (sx)",
    category="Experimental",
    summary="A fast box is stopped by a thin wall via the rigid IPC continuous collision detection "
    "(no tunneling).",
    build=build,
)
