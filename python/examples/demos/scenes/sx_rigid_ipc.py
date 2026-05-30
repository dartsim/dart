"""Rigid IPC contact: a free box settles onto static ground via the experimental
rigid implicit-barrier (IPC) solver (PLAN-082).

sx::World owns the physics with the opt-in rigid IPC solver selected
(`RigidBodySolver.IPC`), so the falling box is held above the ground by the
smooth contact barrier rather than by sequential impulses. SxRenderBridge mirrors
the box and ground into a parallel dart.simulation.World for rendering.
"""

from __future__ import annotations

import numpy as np

import dartpy as dart
import dartpy.simulation_experimental as sx

from .._sx_bridge import SxRenderBridge
from ..runner import PythonDemoScene, SceneSetup

# sx CollisionShape.box takes half-extents; dart.BoxShape takes full extents.
_GROUND_HALF = (2.0, 2.0, 0.25)
_BOX_HALF = (0.25, 0.25, 0.25)


def _full(half: tuple[float, float, float]) -> np.ndarray:
    return np.array([2.0 * half[0], 2.0 * half[1], 2.0 * half[2]])


def build() -> SceneSetup:
    world = sx.World()
    world.rigid_body_solver = sx.RigidBodySolver.IPC
    world.time_step = 0.005

    # Static ground slab with its top face at z = 0.
    ground = world.add_rigid_body(
        "ipc_ground", position=(0.0, 0.0, -_GROUND_HALF[2]))
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))

    # Free box released just above the ground; the IPC barrier catches it.
    box = world.add_rigid_body("ipc_box", position=(0.0, 0.0, 0.6))
    box.mass = 1.0
    box.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))

    world.enter_simulation_mode()

    bridge = SxRenderBridge(world, name="sx_rigid_ipc_render")
    bridge.add_rigid_body_visual(
        ground, dart.BoxShape(_full(_GROUND_HALF)), (0.7, 0.7, 0.7),
        name="ipc_ground_visual")
    bridge.add_rigid_body_visual(
        box, dart.BoxShape(_full(_BOX_HALF)), (0.90, 0.45, 0.20),
        name="ipc_box_visual")
    bridge.sync()

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "rigid_body_solver": "ipc"},
    )


SCENE = PythonDemoScene(
    id="sx_rigid_ipc",
    title="Rigid IPC Contact (sx)",
    category="Experimental",
    summary="A free box settles on static ground via the rigid IPC barrier solver.",
    build=build,
)
