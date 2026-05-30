"""Experimental rigid-body scene: four spheres and a box dropping onto a ground plate.

Mirrors examples/demos/scenes/experimental_rigid_body_gui.cpp via the
SxRenderBridge: an sx::World owns the physics (RigidBody + collision
shape), and a parallel dart.simulation.World owns the SimpleFrame
visuals the C++ viewer renders. The bridge advances sx physics in
``pre_step`` and copies each body's world transform onto its render
frame.
"""

from __future__ import annotations

import numpy as np

import dartpy as dart
import dartpy.simulation_experimental as sx

from .._sx_bridge import SxRenderBridge
from ..runner import PythonDemoScene, SceneSetup


def _visual_for(shape_kind: str, *, radius: float = 0.0, half_extents=None) -> "dart.Shape":
    if shape_kind == "sphere":
        return dart.SphereShape(radius)
    if shape_kind == "box":
        return dart.BoxShape(2.0 * np.asarray(half_extents, dtype=float))
    raise ValueError(f"unknown shape kind: {shape_kind}")


def build() -> SceneSetup:
    world = sx.World()
    world.time_step = 1.0 / 120.0

    bridge = SxRenderBridge(world, name="experimental_rigid_body_render")
    bridge.render_world.set_time_step(1.0 / 120.0)

    ground_opts = sx.RigidBodyOptions()
    ground_opts.position = np.array([0.0, 0.0, -0.08])
    ground_opts.is_static = True
    ground = world.add_rigid_body("ground", ground_opts)
    ground.set_collision_shape(sx.CollisionShape.box(np.array([1.5, 1.5, 0.04])))
    ground.friction = 0.85
    bridge.add_rigid_body_visual(
        ground,
        _visual_for("box", half_extents=np.array([1.5, 1.5, 0.04])),
        (0.42, 0.45, 0.48),
        name="ground_visual",
    )

    for i in range(4):
        opts = sx.RigidBodyOptions()
        opts.mass = 1.0 + 0.25 * i
        opts.position = np.array([-0.75 + 0.5 * i, 0.18 * (i % 2), 1.2 + 0.45 * i])
        opts.linear_velocity = np.array([0.45 - 0.25 * i, 0.0, 0.0])
        opts.angular_velocity = np.array([0.0, 0.4 + 0.2 * i, 0.25])

        radius = 0.18 + 0.03 * i
        body = world.add_rigid_body(f"falling_sphere_{i}", opts)
        body.set_collision_shape(sx.CollisionShape.sphere(radius))
        body.restitution = 0.15
        body.friction = 0.85
        bridge.add_rigid_body_visual(
            body,
            _visual_for("sphere", radius=radius),
            (0.16 + 0.12 * i, 0.42, 0.92 - 0.1 * i),
            name=f"sphere_{i}_visual",
        )

    box_opts = sx.RigidBodyOptions()
    box_opts.mass = 2.0
    box_opts.position = np.array([0.85, -0.35, 2.2])
    box_opts.angular_velocity = np.array([0.2, -0.35, 0.1])
    box = world.add_rigid_body("falling_box", box_opts)
    box.set_collision_shape(sx.CollisionShape.box(np.array([0.1, 0.12, 0.08])))
    box.restitution = 0.15
    box.friction = 0.85
    bridge.add_rigid_body_visual(
        box,
        _visual_for("box", half_extents=np.array([0.1, 0.12, 0.08])),
        (0.93, 0.56, 0.18),
        name="box_visual",
    )

    world.enter_simulation_mode()
    bridge.sync()

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world},
    )


SCENE = PythonDemoScene(
    id="experimental_rigid_body_gui",
    title="Experimental Rigid Body (sx)",
    category="Experimental",
    summary="Four spheres + a box dropping under the experimental world.",
    build=build,
)
