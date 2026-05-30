"""Rigid IPC box stack: three free boxes settle into a stable stack on static
ground through the experimental rigid implicit-barrier (IPC) solver (PLAN-082).

This exercises multiple dynamic bodies in one solve with simultaneous
body-ground and body-body barrier contacts, all held apart by a single
scene-level adaptive barrier stiffness. SxRenderBridge mirrors the bodies into a
parallel dart.simulation.World for rendering.
"""

from __future__ import annotations

import dartpy as dart
import dartpy.simulation_experimental as sx
import numpy as np

from .._sx_bridge import SxRenderBridge
from ..runner import PythonDemoScene, SceneSetup

_GROUND_HALF = (2.0, 2.0, 0.25)
_BOX_HALF = (0.25, 0.25, 0.25)
_COLORS = ((0.90, 0.45, 0.20), (0.30, 0.70, 0.45), (0.25, 0.50, 0.85))
_Z_CENTERS = (0.252, 0.754, 1.256)


def _full(half: tuple[float, float, float]) -> np.ndarray:
    return np.array([2.0 * half[0], 2.0 * half[1], 2.0 * half[2]])


def build() -> SceneSetup:
    world = sx.World()
    world.rigid_body_solver = sx.RigidBodySolver.IPC
    world.time_step = 0.005

    ground = world.add_rigid_body(
        "ipc_stack_ground", position=(0.0, 0.0, -_GROUND_HALF[2])
    )
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))

    # Boxes pre-stacked just in contact (centers 0.502 apart for 0.5-tall boxes);
    # gravity settles them into a stable stack via the barriers.
    boxes = []
    for i, z_center in enumerate(_Z_CENTERS):
        box = world.add_rigid_body(f"ipc_stack_box{i}", position=(0.0, 0.0, z_center))
        box.mass = 1.0
        box.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))
        boxes.append(box)

    world.enter_simulation_mode()

    bridge = SxRenderBridge(world, name="sx_rigid_ipc_stack_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_full(_GROUND_HALF)),
        (0.7, 0.7, 0.7),
        name="ipc_stack_ground_visual",
    )
    for i, box in enumerate(boxes):
        bridge.add_rigid_body_visual(
            box,
            dart.BoxShape(_full(_BOX_HALF)),
            _COLORS[i],
            name=f"ipc_stack_box{i}_visual",
        )
    bridge.sync()

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "rigid_body_solver": "ipc"},
    )


SCENE = PythonDemoScene(
    id="sx_rigid_ipc_stack",
    title="Rigid IPC Box Stack (sx)",
    category="Experimental",
    summary="A three-box stack settles stably on static ground via the rigid IPC barrier solver.",
    build=build,
)
