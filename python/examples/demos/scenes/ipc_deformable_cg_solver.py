"""Large FEM cantilever solved with the iterative CG linear solver (PLAN-081 M7).

A denser tetrahedral beam (a few hundred nodes) is pinned at one end and sags
under gravity, held by stable neo-Hookean *finite-element* elasticity. Unlike the
other FEM scenes, this body opts in to the **iterative** projected-Newton linear
solve (``use_iterative_linear_solver``): a Jacobi-preconditioned conjugate
gradient replaces the sparse Cholesky factorization. CG never factorizes, so its
memory stays near O(nnz) and it scales to far larger meshes -- the keystone of
the IPC paper's large-scale volumetric benchmarks (Fig. 23 / Table 1). On a mesh
this size both solvers reach the same equilibrium; the CG path is what carries
the solver to mesh sizes the direct factorization cannot.

DART-native FEM/scale showcase -- not a faithful IPC paper-figure reproduction
(this isolates the volumetric elasticity and the iterative linear solve).
"""

from __future__ import annotations

import dartpy.simulation_experimental as sx

from .._ipc_deformable_bridge import IpcDeformableBridge, build_fem_bar
from ..runner import PythonDemoScene, SceneSetup


def build() -> SceneSetup:
    options, edges = build_fem_bar(
        cells_x=14,
        cells_y=3,
        cells_z=3,
        cell_size=0.07,
        origin=(-0.6, -0.105, 0.75),
        # A moderate modulus keeps the elasticity Hessian well-conditioned
        # relative to the inertia term, so the Jacobi-preconditioned CG converges
        # in a handful of iterations -- the regime where the iterative solve pays
        # off on large meshes.
        youngs_modulus=2.0e5,
        poisson_ratio=0.3,
        density=1.0e3,
    )
    # Drive this body through the matrix-light conjugate-gradient Newton solve
    # instead of the sparse Cholesky factorization.
    options.material.use_iterative_linear_solver = True

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 0.005
    body = world.add_deformable_body("fem_cg_bar", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_cg_solver")
    bridge.add_deformable_visual(body, name="fem_cg_bar", edges=edges)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_cg_solver",
    title="Deformable FEM CG Solver (IPC)",
    category="IPC Deformable (sx)",
    summary="A large tetrahedral FEM cantilever sags under gravity, solved by the iterative conjugate-gradient Newton path.",
    build=build,
)
