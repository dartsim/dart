"""FEM cantilever loaded from a GMSH .msh file (experimental IPC deformable solver).

A tetrahedral beam is loaded from a GMSH ASCII ``.msh`` mesh file via
``dartpy.simulation_experimental.load_gmsh_tet_mesh`` (the new tet-mesh
importer), opted in to stable neo-Hookean FEM elasticity, pinned at its clamped
end, and released to sag under gravity. This demonstrates the file-loading path
for FEM bodies -- the gateway to driving the solver from external tet meshes
rather than procedurally built grids (PLAN-081 M4).

DART-native FEM showcase -- not a faithful IPC paper-figure reproduction.
"""

from __future__ import annotations

from pathlib import Path

import dartpy.simulation_experimental as sx

from .._ipc_deformable_bridge import IpcDeformableBridge
from ..runner import PythonDemoScene, SceneSetup

_MESH_PATH = Path(__file__).resolve().parent.parent / "assets" / "fem_bar.msh"


def build() -> SceneSetup:
    options = sx.load_gmsh_tet_mesh(str(_MESH_PATH))
    options.material.youngs_modulus = 5.0e6
    options.material.poisson_ratio = 0.3
    options.material.density = 1.0e3
    options.material.use_finite_element_elasticity = True

    # Pin the clamped end (the nodes on the minimum-x face).
    positions = options.positions
    min_x = min(float(p[0]) for p in positions)
    options.fixed_nodes = [
        i for i, p in enumerate(positions) if abs(float(p[0]) - min_x) < 1e-6
    ]

    # Wireframe edges from the loaded tetrahedra.
    edges: set[tuple[int, int]] = set()
    for tet in options.tetrahedra:
        nodes = (tet.node_a, tet.node_b, tet.node_c, tet.node_d)
        for a in range(4):
            for b in range(a + 1, 4):
                u, v = nodes[a], nodes[b]
                edges.add((u, v) if u < v else (v, u))

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 0.005
    body = world.add_deformable_body("fem_msh", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_fem_msh")
    bridge.add_deformable_visual(body, name="fem_msh", edges=sorted(edges))

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_fem_msh",
    title="Deformable FEM from .msh (IPC)",
    category="IPC Deformable (sx)",
    summary="A FEM cantilever loaded from a GMSH .msh tet mesh sags under gravity.",
    build=build,
)
