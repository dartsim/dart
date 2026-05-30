"""FEM slab draping over a sphere obstacle (experimental IPC deformable solver).

A free tetrahedral FEM slab is dropped onto a static sphere resting on the
ground. The sphere is opted in as a deformable obstacle, so its radial
clamped-log barrier (now a first-class projected-Newton term) pushes the slab's
nodes out along the surface normal while the ground barrier catches the
overhanging edges: the slab drapes over the curved obstacle intersection-free.
This combines stable neo-Hookean FEM elasticity (PLAN-081 M1) with the sphere
obstacle barrier and the ground barrier in one solve.

DART-native FEM showcase -- not a faithful IPC paper-figure reproduction.
"""

from __future__ import annotations

import dartpy.simulation_experimental as sx

from .._ipc_deformable_bridge import IpcDeformableBridge, build_fem_bar
from ..runner import PythonDemoScene, SceneSetup

_SPHERE_RADIUS = 0.12
_SPHERE_CENTER = (0.0, 0.0, _SPHERE_RADIUS)  # resting on the ground top (z = 0)
_GROUND_CENTER = (0.0, 0.0, -0.06)
_GROUND_HALF = (1.2, 1.2, 0.06)


def build() -> SceneSetup:
    options, edges = build_fem_bar(
        cells_x=8,
        cells_y=8,
        cells_z=1,
        cell_size=0.06,
        origin=(-0.24, -0.24, 0.42),
        youngs_modulus=3.0e4,
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

    sphere = world.add_rigid_body("sphere", position=_SPHERE_CENTER)
    sphere.is_static = True
    sphere.set_collision_shape(sx.CollisionShape.sphere(_SPHERE_RADIUS))
    sphere.is_deformable_surface_ccd_obstacle = True

    body = world.add_deformable_body("fem_sphere", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_fem_sphere")
    bridge.add_rigid_box_visual(
        _GROUND_CENTER,
        tuple(2.0 * h for h in _GROUND_HALF),
        (0.37, 0.40, 0.43),
        name="ground_visual",
    )
    bridge.add_rigid_sphere_visual(
        _SPHERE_CENTER, _SPHERE_RADIUS, (0.52, 0.45, 0.34), name="sphere_visual"
    )
    bridge.add_deformable_visual(body, name="fem_sphere", edges=edges)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_fem_sphere",
    title="Deformable FEM over Sphere (IPC)",
    category="IPC Deformable (sx)",
    summary="A FEM slab drapes over a sphere obstacle via the radial barrier + ground barrier.",
    build=build,
)
