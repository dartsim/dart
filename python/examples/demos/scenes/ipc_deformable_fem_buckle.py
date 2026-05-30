"""FEM self-contact under compression (experimental IPC deformable solver).

A slender tetrahedralized beam is gripped at both ends and the grips are driven
toward each other, so the soft FEM core buckles and folds onto itself. Where the
folding surface would pass through itself, the IPC clamped-log self-contact
barrier holds it apart at a positive separation -- self-collision is the heart of
the IPC method, here on a volumetric FEM body. This is a DART-native step toward
the paper's self-collision stress tests (e.g. the mat/rod buckling figures),
driven by the opt-in FEM elasticity (PLAN-081) plus scripted Dirichlet boundary
conditions and the always-on surface self-contact barrier.

DART-native FEM showcase -- not a faithful paper-figure reproduction; it isolates
volumetric FEM elasticity colliding with itself under a prescribed compression.
"""

from __future__ import annotations

import dartpy.simulation_experimental as sx

from .._ipc_deformable_bridge import IpcDeformableBridge, build_fem_compression_bar
from ..runner import PythonDemoScene, SceneSetup


def build() -> SceneSetup:
    options, edges = build_fem_compression_bar(
        cells_x=24,
        cells_y=2,
        cells_z=2,
        cell_size=0.05,
        origin=(-0.6, -0.05, 0.5),
        youngs_modulus=2.0e4,
        compression_rate=0.12,
        compression_end_time=3.0,
        poisson_ratio=0.3,
        density=1.0e3,
    )

    world = sx.World()
    world.gravity = [
        0.0,
        0.0,
        -3.0,
    ]  # gentle downward bias breaks the buckling symmetry
    world.time_step = 0.004
    body = world.add_deformable_body("fem_buckle", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_fem_buckle")
    bridge.add_deformable_visual(body, name="fem_buckle", edges=edges)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_fem_buckle",
    title="Deformable FEM Self-Contact (IPC)",
    category="IPC Deformable (sx)",
    summary="A tetrahedral FEM beam is compressed end-to-end until it buckles "
    "and folds onto itself, held apart by the self-contact barrier.",
    build=build,
)
