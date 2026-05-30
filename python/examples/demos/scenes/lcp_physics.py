"""LCP physics: extreme mass-ratio stability scenario.

Mirrors examples/demos/scenes/lcp_physics.cpp at a high level. The C++
scene presents five LCP benchmark scenarios switchable from a UI panel
(mass_ratio, box_stack, ball_drop, dominos, inclined_plane); the
headless Python mirror runs the most distinctive default scenario
(mass_ratio: a 1000:1 mass-ratio stack) so the LCP solver is
exercised under a stressful contact configuration.

The C++ scene also lets the user switch between the Dantzig and PGS
LCP solvers at runtime; the Python mirror picks the constructor's
primary LCP solver (Dantzig) once at world creation and keeps it for
the lifetime of the scene.
"""

from __future__ import annotations

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup

_GROUND_THICKNESS = 0.1
_BOX_SIZE = 0.5


def _ground() -> "dart.Skeleton":
    ground = dart.Skeleton("ground")
    _joint, body = ground.create_weld_joint_and_body_node_pair()
    box = dart.BoxShape([20.0, _GROUND_THICKNESS, 20.0])
    node = body.create_shape_node(box)
    node.create_visual_aspect().set_color([0.8, 0.8, 0.8])
    node.create_collision_aspect()
    node.create_dynamics_aspect()
    tf = np.eye(4)
    tf[1, 3] = -0.5 * _GROUND_THICKNESS
    body.get_parent_joint().set_transform_from_parent_body_node(tf)
    return ground


def _box(
    name: str, size, mass: float, position, color
) -> "dart.Skeleton":
    skel = dart.Skeleton(name)
    joint, body = skel.create_free_joint_and_body_node_pair()
    tf = np.eye(4)
    tf[:3, 3] = position
    joint.set_transform_from_parent_body_node(tf)

    size_vec = np.asarray(size, dtype=float)
    shape = dart.BoxShape(size_vec)
    node = body.create_shape_node(shape)
    visual = node.create_visual_aspect()
    visual.set_color(list(color))
    node.create_collision_aspect()
    node.create_dynamics_aspect()
    body.set_inertia(
        dart.Inertia(
            mass,
            np.zeros(3),
            dart.BoxShape.compute_inertia_of(size_vec, mass),
        )
    )
    return skel


def build() -> SceneSetup:
    config = dart.simulation.WorldConfig("lcp_physics")
    config.primary_lcp_solver = dart.simulation.LcpSolverType.Dantzig
    world = dart.simulation.World(config)
    world.set_time_step(1.0 / 1000.0)
    world.set_gravity([0.0, -9.81, 0.0])

    world.add_skeleton(_ground())
    world.add_skeleton(
        _box(
            "light_box",
            [_BOX_SIZE, _BOX_SIZE, _BOX_SIZE],
            1.0,
            [0.0, 0.5 * _BOX_SIZE, 0.0],
            [0.4, 0.8, 0.4],
        )
    )
    world.add_skeleton(
        _box(
            "heavy_box",
            [_BOX_SIZE, _BOX_SIZE, _BOX_SIZE],
            1000.0,
            [0.0, _BOX_SIZE * 1.6, 0.0],
            [0.8, 0.2, 0.2],
        )
    )

    return SceneSetup(world=world, info={})


SCENE = PythonDemoScene(
    id="lcp_physics",
    title="LCP Physics (Mass Ratio)",
    category="Constraints & Joints",
    summary="Extreme 1000:1 mass-ratio stack exercising the LCP contact solver.",
    build=build,
)
