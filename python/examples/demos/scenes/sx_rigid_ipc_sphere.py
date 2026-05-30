"""Rigid IPC sphere drop: a free sphere settles onto static ground through the
experimental rigid implicit-barrier (IPC) solver (PLAN-082).

This complements ``sx_rigid_ipc`` (a box) by exercising the solver's curved
collision shape: the sphere is triangulated into a surface mesh and held above
the ground by the smooth contact barrier. SxRenderBridge mirrors the sphere and
ground into a parallel dart.simulation.World for rendering.
"""

from __future__ import annotations

import dartpy as dart
import dartpy.simulation_experimental as sx
import numpy as np

from .._sx_bridge import SxRenderBridge
from ..runner import PythonDemoScene, SceneSetup

_GROUND_HALF = (2.0, 2.0, 0.25)
_RADIUS = 0.25


def _full(half: tuple[float, float, float]) -> np.ndarray:
    return np.array([2.0 * half[0], 2.0 * half[1], 2.0 * half[2]])


def build() -> SceneSetup:
    world = sx.World()
    world.rigid_body_solver = sx.RigidBodySolver.IPC
    world.time_step = 0.005

    ground = world.add_rigid_body(
        "ipc_sphere_ground", position=(0.0, 0.0, -_GROUND_HALF[2])
    )
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))

    sphere = world.add_rigid_body("ipc_sphere", position=(0.0, 0.0, 0.6))
    sphere.mass = 1.0
    sphere.set_collision_shape(sx.CollisionShape.sphere(_RADIUS))

    world.enter_simulation_mode()

    bridge = SxRenderBridge(world, name="sx_rigid_ipc_sphere_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_full(_GROUND_HALF)),
        (0.7, 0.7, 0.7),
        name="ipc_sphere_ground_visual",
    )
    bridge.add_rigid_body_visual(
        sphere, dart.SphereShape(_RADIUS), (0.85, 0.30, 0.55), name="ipc_sphere_visual"
    )
    bridge.sync()

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "rigid_body_solver": "ipc"},
    )


SCENE = PythonDemoScene(
    id="sx_rigid_ipc_sphere",
    title="Rigid IPC Sphere Drop (sx)",
    category="Experimental",
    summary="A free sphere settles on static ground via the rigid IPC barrier solver.",
    build=build,
)
