"""FEM cantilever bar (experimental IPC deformable solver).

A tetrahedralized beam is pinned at one end and sags under gravity, held by
stable neo-Hookean *finite-element* volumetric elasticity (PLAN-081 M1) rather
than the mass-spring edge model the other scenes use. This is the first DART
deformable showcase driven by true FEM elasticity -- the keystone capability for
the IPC paper's volumetric scenes (e.g. Fig. 4 rod twist / Fig. 14 mat twist).

DART-native FEM showcase -- not a faithful IPC paper-figure reproduction (no
codimensional contact or friction here; this isolates the volumetric elasticity).
"""

from __future__ import annotations

import dartpy.simulation_experimental as sx

from .._ipc_deformable_bridge import IpcDeformableBridge, build_fem_bar
from ..runner import PythonDemoScene, SceneSetup


def build() -> SceneSetup:
    options, edges = build_fem_bar(
        cells_x=10,
        cells_y=1,
        cells_z=1,
        cell_size=0.12,
        origin=(-0.6, -0.06, 0.75),
        youngs_modulus=5.0e6,
        poisson_ratio=0.3,
        density=1.0e3,
    )

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 0.005
    body = world.add_deformable_body("fem_bar", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_fem_bar")
    bridge.add_deformable_visual(body, name="fem_bar", edges=edges)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_fem_bar",
    title="Deformable FEM Bar (IPC)",
    category="IPC Deformable (sx)",
    summary="A tetrahedral FEM cantilever sags under gravity via stable neo-Hookean elasticity.",
    build=build,
)
