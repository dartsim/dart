"""FEM slab draping over a box obstacle (experimental IPC deformable solver).

A free tetrahedral FEM slab is dropped onto a static box ("table") resting on
the ground. The box is opted in as a deformable obstacle, so its clamped-log
barrier (along the outward surface normal, a first-class projected-Newton term)
keeps the slab outside the box while the ground barrier catches the overhanging
edges: the slab drapes over the flat top and down the sides intersection-free.
This exercises stable neo-Hookean FEM elasticity (PLAN-081 M1) with the box
obstacle barrier and the ground barrier in one solve.

DART-native FEM showcase -- not a faithful IPC paper-figure reproduction.
"""

from __future__ import annotations

import dartpy.simulation_experimental as sx

from .._ipc_deformable_bridge import IpcDeformableBridge, build_fem_bar
from ..runner import PythonDemoScene, SceneSetup

_BOX_HALF = (0.16, 0.16, 0.1)
_BOX_CENTER = (0.0, 0.0, _BOX_HALF[2])  # resting on the ground top (z = 0)
_GROUND_CENTER = (0.0, 0.0, -0.06)
_GROUND_HALF = (1.2, 1.2, 0.06)


def build() -> SceneSetup:
    options, edges = build_fem_bar(
        cells_x=8,
        cells_y=8,
        cells_z=1,
        cell_size=0.06,
        origin=(-0.24, -0.24, 0.4),
        youngs_modulus=4.0e4,
        poisson_ratio=0.3,
        density=1.0e3,
    )
    options.fixed_nodes = []

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 0.004

    ground = world.add_rigid_body("ground", position=_GROUND_CENTER)
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))
    ground.is_deformable_ground_barrier = True

    box = world.add_rigid_body("box", position=_BOX_CENTER)
    box.is_static = True
    box.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))
    box.is_deformable_surface_ccd_obstacle = True

    body = world.add_deformable_body("fem_box", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_fem_box")
    bridge.add_rigid_box_visual(
        _GROUND_CENTER,
        tuple(2.0 * h for h in _GROUND_HALF),
        (0.37, 0.40, 0.43),
        name="ground_visual",
    )
    bridge.add_rigid_box_visual(
        _BOX_CENTER,
        tuple(2.0 * h for h in _BOX_HALF),
        (0.52, 0.45, 0.34),
        name="box_visual",
    )
    bridge.add_deformable_visual(body, name="fem_box", edges=edges)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_fem_box",
    title="Deformable FEM over Box (IPC)",
    category="IPC Deformable (sx)",
    summary="A FEM slab drapes over a box obstacle via the box barrier + ground barrier.",
    build=build,
)
