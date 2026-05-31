"""Stiff FEM cube settling on a ground barrier via the iterative CG solve (PLAN-081 M7).

A dense, stiff tetrahedral FEM cube falls under gravity onto a static ground
barrier, squashes on impact, and settles intersection-free -- driven by the
**iterative** projected-Newton linear solve (``use_iterative_linear_solver``).
Stiff barrier contact makes the Newton Hessian ill-conditioned, which is exactly
where a diagonal (Jacobi) conjugate gradient stalls; the incomplete-Cholesky
preconditioner collapses the CG iteration count so the matrix-light iterative
solve carries the contact without falling back to steepest descent. CG never
factorizes, so its memory stays near O(nnz) -- the scaling axis of the IPC
paper's large-scale contact benchmarks (Fig. 23 / Table 1).

DART-native FEM/scale showcase -- not a faithful IPC paper-figure reproduction
(single analytic ground barrier; this isolates the volumetric elasticity, stiff
contact, and the incomplete-Cholesky-preconditioned iterative linear solve).
"""

from __future__ import annotations

import dartpy.simulation_experimental as sx

from .._ipc_deformable_bridge import IpcDeformableBridge, build_fem_bar
from ..runner import PythonDemoScene, SceneSetup

_GROUND_CENTER = (0.0, 0.0, -0.06)
_GROUND_HALF = (1.2, 1.2, 0.06)  # top face at z = 0.0


def build() -> SceneSetup:
    # A dense, stiff cube of FEM tetrahedra released above the barrier.
    options, edges = build_fem_bar(
        cells_x=5,
        cells_y=5,
        cells_z=5,
        cell_size=0.05,
        origin=(-0.125, -0.125, 0.34),
        # A stiff modulus makes the contact Hessian ill-conditioned -- the regime
        # where the incomplete-Cholesky preconditioner keeps CG converging.
        youngs_modulus=1.0e6,
        poisson_ratio=0.3,
        density=1.0e3,
    )
    options.fixed_nodes = []
    # Drive this body through the incomplete-Cholesky-preconditioned CG Newton
    # solve instead of the sparse Cholesky factorization.
    options.material.use_iterative_linear_solver = True

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 0.004

    ground = world.add_rigid_body("ground", position=_GROUND_CENTER)
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))
    ground.is_deformable_ground_barrier = True

    body = world.add_deformable_body("fem_cg_contact", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_cg_contact")
    bridge.add_rigid_box_visual(
        _GROUND_CENTER,
        tuple(2.0 * h for h in _GROUND_HALF),
        (0.37, 0.40, 0.43),
        name="ground_visual",
    )
    bridge.add_deformable_visual(body, name="fem_cg_contact", edges=edges)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_cg_contact",
    title="Deformable FEM CG Contact (IPC)",
    category="IPC Deformable (sx)",
    summary="A stiff FEM cube settles on an IPC ground barrier via the incomplete-Cholesky CG Newton solve.",
    build=build,
)
