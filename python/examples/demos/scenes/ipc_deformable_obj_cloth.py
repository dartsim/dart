"""Cloth loaded from a Wavefront .obj draping under gravity (experimental IPC).

A mass-spring cloth membrane is loaded from a bundled Wavefront ``.obj`` triangle
mesh (the new ``load_obj_triangle_mesh`` importer, PLAN-081 M3 asset pipeline),
pinned along one edge, and released so it sags and drapes under gravity, its free
edge settling onto the static ground barrier below. This is the surface-mesh
counterpart of the volumetric ``.msh`` FEM showcase, exercising the ``.obj``
importer feeding a real deformable solve plus the ground contact barrier.

DART-native showcase -- not a faithful IPC paper-figure reproduction.
"""

from __future__ import annotations

from pathlib import Path

import dartpy.simulation_experimental as sx

from .._ipc_deformable_bridge import IpcDeformableBridge, build_cloth_from_obj
from ..runner import PythonDemoScene, SceneSetup

_CLOTH_PATH = Path(__file__).resolve().parent.parent / "assets" / "cloth_grid.obj"
_GROUND_CENTER = (0.0, 0.0, -0.26)
_GROUND_HALF = (1.2, 1.2, 0.06)


def build() -> SceneSetup:
    # Lift the flat sheet (authored in the z = 0 plane) and let it drape.
    options, edges = build_cloth_from_obj(
        _CLOTH_PATH,
        mass=0.02,
        edge_stiffness=200.0,
        damping=1.0,
        translate=(0.0, 0.0, 0.2),
    )

    # Pin the sheet's far (max-y) edge so the rest sags and drapes off it.
    ys = [float(p[1]) for p in options.positions]
    y_max = max(ys)
    options.fixed_nodes = [i for i, y in enumerate(ys) if y > y_max - 1e-4]

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 0.004

    ground = world.add_rigid_body("ground", position=_GROUND_CENTER)
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))
    ground.is_deformable_ground_barrier = True

    body = world.add_deformable_body("obj_cloth", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_obj_cloth")
    bridge.add_rigid_box_visual(
        _GROUND_CENTER,
        tuple(2.0 * h for h in _GROUND_HALF),
        (0.37, 0.40, 0.43),
        name="ground_visual",
    )
    bridge.add_deformable_visual(body, name="obj_cloth", edges=edges)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_obj_cloth",
    title="Deformable .obj Cloth (IPC)",
    category="IPC Deformable (sx)",
    summary="A cloth loaded from a Wavefront .obj is pinned at one edge and "
    "drapes under gravity onto the ground barrier.",
    build=build,
)
