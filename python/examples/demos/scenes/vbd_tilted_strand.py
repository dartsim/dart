"""VBD tilted strand: the TinyVBD reference stress scene, solved by VBD.

Reproduces the TinyVBD default benchmark scene -- a 20-vertex strand tilted 30
degrees and pinned at one end, with very stiff springs (1e8), a 1:1000 mass
ratio between the strand and its heavy free end, a 1/60 s step, and 100 VBD
sweeps. It is the runnable, visible companion to the
``bm_vbd_block_descent`` ``BM_VbdTinyStrandStep`` CPU benchmark: the same
parameters the in-repo VBD solver is benchmarked against the TinyVBD reference
on. The stiff springs plus the heavy free end are the stress that motivates
VBD's per-vertex block solve.
"""

from __future__ import annotations

import dartpy.simulation_experimental as sx
import numpy as np

from .._sx_bridge import SxRenderBridge
from ..runner import PythonDemoScene, SceneSetup

_NUM_VERTS = 20
_SPACING = 0.05
_TAN_ANGLE = 0.57735  # 30 degrees


def _make_strand_options() -> "sx.DeformableBodyOptions":
    options = sx.DeformableBodyOptions()
    positions = []
    masses = []
    for i in range(_NUM_VERTS):
        # Pinned at node 0; the strand extends in +x and tilts up in +z, then
        # sags under gravity. The heavy free end (1000x mass) against the stiff
        # springs is the high mass-ratio stress the TinyVBD scene exercises.
        positions.append(np.array([i * _SPACING, 0.0, 1.5 + i * _SPACING * _TAN_ANGLE]))
        masses.append(1000.0 if i == _NUM_VERTS - 1 else 1.0)
    options.positions = positions
    options.masses = masses
    options.fixed_nodes = [0]

    edges = []
    for i in range(_NUM_VERTS - 1):  # structural springs
        edges.append(sx.DeformableEdge(i, i + 1, -1.0))
    for i in range(_NUM_VERTS - 2):  # skip (bending) springs (i, i+2)
        edges.append(sx.DeformableEdge(i, i + 2, -1.0))
    options.edges = edges
    options.edge_stiffness = 1.0e8
    options.damping = 1.0
    return options


def build() -> SceneSetup:
    world = sx.World()
    world.gravity = [0.0, 0.0, -10.0]
    world.time_step = 1.0 / 60.0
    world.add_deformable_body("vbd_tilted_strand", _make_strand_options())

    solver = sx.DeformableSolverOptions()
    solver.iterations = 100
    world.configure_deformable_solver("vbd_tilted_strand", solver)
    world.enter_simulation_mode()

    bridge = SxRenderBridge(world, name="vbd_tilted_strand_render")
    body = world.get_deformable_body("vbd_tilted_strand")
    bridge.add_deformable_visual(
        body, (0.90, 0.40, 0.55), radius=0.02, fixed_color=(0.30, 0.65, 0.95)
    )
    bridge.sync()

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "nodes": _NUM_VERTS},
    )


SCENE = PythonDemoScene(
    id="vbd_tilted_strand",
    title="VBD Tilted Strand (sx)",
    category="Experimental",
    summary=(
        "The TinyVBD reference stress scene (stiff 20-vertex strand, 1:1000 "
        "mass ratio), solved by VBD."
    ),
    build=build,
)
