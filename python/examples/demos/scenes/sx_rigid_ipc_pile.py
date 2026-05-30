"""Rigid IPC pile: three free boxes dropped from staggered heights land and
settle into a pile on static ground through the experimental rigid
implicit-barrier (IPC) solver (PLAN-082).

Unlike the neatly aligned ``sx_rigid_ipc_stack``, the boxes start spread out and
at different heights, so they fall, collide, and settle into an irregular pile --
exercising several simultaneous body-ground and body-body barrier contacts.
SxRenderBridge mirrors the bodies into a parallel dart.simulation.World for
rendering.
"""

from __future__ import annotations

import dartpy as dart
import dartpy.simulation_experimental as sx
import numpy as np

from .._sx_bridge import SxRenderBridge
from ..runner import PythonDemoScene, SceneSetup

_GROUND_HALF = (2.0, 2.0, 0.25)
_BOX_HALF = (0.15, 0.15, 0.15)
# (x, z) drop positions and per-box colors.
_DROPS = ((-0.35, 0.5), (0.0, 0.7), (0.35, 0.9))
_COLORS = ((0.90, 0.45, 0.20), (0.30, 0.70, 0.45), (0.25, 0.50, 0.85))


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

    boxes = []
    for i, (x, z) in enumerate(_DROPS):
        box = world.add_rigid_body(f"ipc_pile_box{i}", position=(x, 0.0, z))
        box.mass = 1.0
        box.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))
        boxes.append(box)

    world.enter_simulation_mode()

    bridge = SxRenderBridge(world, name="sx_rigid_ipc_pile_render")
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

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "rigid_body_solver": "ipc"},
    )


SCENE = PythonDemoScene(
    id="sx_rigid_ipc_pile",
    title="Rigid IPC Box Pile (sx)",
    category="Experimental",
    summary="Three boxes drop and settle into a pile via the rigid IPC barrier solver.",
    build=build,
)
