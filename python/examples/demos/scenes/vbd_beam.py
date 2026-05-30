"""VBD beam: a tetrahedral cantilever sagging under gravity, solved by VBD.

A Stable Neo-Hookean tetrahedral bar (stacked unit cubes, six Kuhn tets each)
pinned at one end. It is the reproducible single-body analogue of the VBD
paper's twisting-beam scene: the experimental World runs the contact-free
tetrahedral body through the VBD inner solver with the body's Lame parameters
(from the material), and the free end sags under gravity.
"""

from __future__ import annotations

import numpy as np

import dartpy.simulation_experimental as sx

from .._sx_bridge import SxRenderBridge
from ..runner import PythonDemoScene, SceneSetup

# Six Kuhn tetrahedra tiling one cube's eight corners (n = 4*di + 2*dj + dk).
_KUHN = (
    (0, 1, 3, 7),
    (0, 3, 2, 7),
    (0, 2, 6, 7),
    (0, 6, 4, 7),
    (0, 4, 5, 7),
    (0, 5, 1, 7),
)


def _make_beam_options(length_cubes: int) -> "sx.DeformableBodyOptions":
    options = sx.DeformableBodyOptions()
    spacing = 0.15

    def index(i: int, j: int, k: int) -> int:
        return (i * 2 + j) * 2 + k

    positions = []
    masses = []
    fixed = []
    for i in range(length_cubes + 1):
        for j in range(2):
            for k in range(2):
                positions.append(
                    np.array([spacing * i, spacing * j, spacing * k]))
                masses.append(0.2)
                if i == 0:  # pin the root face
                    fixed.append(index(i, j, k))
    options.positions = positions
    options.masses = masses
    options.fixed_nodes = fixed

    tetrahedra = []
    for c in range(length_cubes):
        corner = []
        for n in range(8):
            di = (n >> 2) & 1
            dj = (n >> 1) & 1
            dk = n & 1
            corner.append(index(c + di, dj, dk))
        for tet in _KUHN:
            tetrahedra.append(
                sx.DeformableTetrahedron(
                    corner[tet[0]],
                    corner[tet[1]],
                    corner[tet[2]],
                    corner[tet[3]]))
    options.tetrahedra = tetrahedra

    material = sx.DeformableMaterialProperties()
    material.youngs_modulus = 3.0e4
    material.poisson_ratio = 0.3
    options.material = material
    options.damping = 2.0
    return options


def build() -> SceneSetup:
    length_cubes = 8
    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 1.0 / 120.0
    world.add_deformable_body("vbd_beam", _make_beam_options(length_cubes))

    solver = sx.DeformableSolverOptions()
    solver.iterations = 30
    world.configure_deformable_solver("vbd_beam", solver)
    world.enter_simulation_mode()

    bridge = SxRenderBridge(world, name="vbd_beam_render")
    body = world.get_deformable_body("vbd_beam")
    bridge.add_deformable_visual(
        body, (0.85, 0.45, 0.20), radius=0.025, fixed_color=(0.30, 0.65, 0.95))
    bridge.sync()

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "nodes": (length_cubes + 1) * 4},
    )


SCENE = PythonDemoScene(
    id="vbd_beam",
    title="VBD Beam (sx)",
    category="Experimental",
    summary="A tetrahedral cantilever beam sagging under gravity, solved by VBD.",
    build=build,
)
