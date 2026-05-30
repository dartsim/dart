"""FEM twisting bar (experimental IPC deformable solver).

A tetrahedralized beam is gripped at both ends and counter-rotated about its
long axis, then released; the stable neo-Hookean FEM core stores the shear and
untwists elastically. This is the volumetric-twist counterpart of the IPC
paper's Fig. 4 (rod twist) and Fig. 14 (mat twist) stress tests, driven by the
opt-in FEM elasticity (PLAN-081 M1) plus scripted Dirichlet boundary conditions.

DART-native FEM showcase -- not a faithful paper-figure reproduction (no
self-contact buckling, codimensional contact, or friction); it isolates
large-shear volumetric elasticity.
"""

from __future__ import annotations

import dartpy.simulation_experimental as sx

from .._ipc_deformable_bridge import IpcDeformableBridge, build_fem_twist_bar
from ..runner import PythonDemoScene, SceneSetup


def build() -> SceneSetup:
    options, edges = build_fem_twist_bar(
        cells_x=10,
        cells_y=2,
        cells_z=2,
        cell_size=0.1,
        origin=(-0.5, -0.1, 0.7),
        youngs_modulus=8.0e5,
        twist_rate=0.35,
        twist_end_time=1.4,
        poisson_ratio=0.3,
        density=1.0e3,
    )

    world = sx.World()
    world.gravity = [0.0, 0.0, 0.0]  # isolate the twist from gravity sag
    world.time_step = 0.005
    body = world.add_deformable_body("fem_twist", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_fem_twist")
    bridge.add_deformable_visual(body, name="fem_twist", edges=edges)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_fem_twist",
    title="Deformable FEM Twist (IPC)",
    category="IPC Deformable (sx)",
    summary="A tetrahedral FEM bar is twisted at both ends and untwists elastically.",
    build=build,
)
