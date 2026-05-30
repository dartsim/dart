"""FEM twisting bar with the fixed-corotational material (experimental IPC).

Same gripped-and-counter-rotated tetrahedral beam as the stable neo-Hookean
twist demo, but driven by the *fixed-corotational* material -- the IPC paper's
other isotropic elasticity model (Li et al. 2020). Fixed-corotational measures
strain as ``||F - R||`` against the polar-decomposition rotation ``R``, so it
stores pure shear energy under twist and untwists elastically when released.
This is the volumetric-twist counterpart of the paper's Fig. 4 (rod twist) /
Fig. 14 (mat twist) stress tests, here exercising the opt-in fixed-corotational
FEM path (PLAN-081) plus scripted Dirichlet boundary conditions.

DART-native FEM showcase -- not a faithful paper-figure reproduction (no
self-contact buckling, codimensional contact, or friction); it isolates the
fixed-corotational material under large shear.
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
        use_fixed_corotational=True,
    )

    world = sx.World()
    world.gravity = [0.0, 0.0, 0.0]  # isolate the twist from gravity sag
    world.time_step = 0.005
    body = world.add_deformable_body("fcr_twist", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_fcr_twist")
    bridge.add_deformable_visual(body, name="fcr_twist", edges=edges)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_fcr_twist",
    title="Deformable FCR Twist (IPC)",
    category="IPC Deformable (sx)",
    summary="A tetrahedral fixed-corotational FEM bar is twisted at both ends "
    "and untwists elastically.",
    build=build,
)
