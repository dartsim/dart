"""Rigid IPC friction slide: a box slides across static ground and is braked to
rest by friction, all through the rigid implicit-barrier (IPC)
solver (PLAN-082).

This scene complements ``rigid_ipc`` (a box settling straight down). Here the
box starts in contact with a tangential velocity, so the smooth contact barrier
holds it above the ground while lagged Coulomb friction decelerates the slide
without it penetrating or freezing. WorldRenderBridge mirrors the box and ground
into a parallel render World for rendering.
"""

from __future__ import annotations

from collections import deque

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

# sx CollisionShape.box takes half-extents; dart.BoxShape takes full extents.
_GROUND_HALF = (3.0, 1.0, 0.25)
_BOX_HALF = (0.25, 0.25, 0.25)
_FRICTION = 0.5
_INITIAL_SPEED = 2.0


def _full(half: tuple[float, float, float]) -> np.ndarray:
    return np.array([2.0 * half[0], 2.0 * half[1], 2.0 * half[2]])


def build() -> SceneSetup:
    world = sx.World()
    world.rigid_body_solver = sx.RigidBodySolver.IPC
    world.time_step = 0.005

    # Static ground slab (long in x to give the box room to slide), top at z = 0.
    ground = world.add_rigid_body(
        "ipc_slide_ground", position=(-1.0, 0.0, -_GROUND_HALF[2])
    )
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))
    ground.friction = _FRICTION

    # Box resting on the ground with a tangential velocity; friction brakes it.
    box = world.add_rigid_body("ipc_slide_box", position=(-1.5, 0.0, 0.255))
    box.mass = 1.0
    box.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))
    box.linear_velocity = (_INITIAL_SPEED, 0.0, 0.0)
    box.friction = _FRICTION

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="rigid_ipc_slide_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_full(_GROUND_HALF)),
        (0.7, 0.7, 0.7),
        name="ipc_slide_ground_visual",
    )
    bridge.add_rigid_body_visual(
        box,
        dart.BoxShape(_full(_BOX_HALF)),
        (0.20, 0.55, 0.90),
        name="ipc_slide_box_visual",
    )
    bridge.sync()

    speed_history: deque[float] = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        speed = float(np.linalg.norm(np.asarray(box.linear_velocity, dtype=float)))
        speed_history.append(speed)
        builder.text("solver: rigid IPC")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"time step: {world.time_step:.4f} s")
        builder.text(f"box speed: {speed:.3f} m/s")
        changed, friction = builder.slider("Friction", float(box.friction), 0.0, 1.0)
        if changed:
            box.friction = float(friction)
            ground.friction = float(friction)
        builder.plot_lines("Box speed", list(speed_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("Rigid IPC Slide", build_panel)],
        info={"sx_world": world, "rigid_body_solver": "ipc"},
    )


SCENE = PythonDemoScene(
    id="rigid_ipc_slide",
    title="Rigid IPC Friction Slide",
    category="Rigid IPC",
    summary="A box slides across static ground and is friction-braked to rest "
    "via the rigid IPC barrier solver.",
    build=build,
)
