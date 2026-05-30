"""Strand loaded from a .seg segment mesh hanging under gravity (experimental).

A one-dimensional mass-spring strand is loaded from a bundled ``.seg`` segment
mesh (the new ``load_seg_line_mesh`` importer, PLAN-081 M3 asset pipeline), pinned
at one end, and released so it hangs and swings under gravity -- the codimensional
(1D) counterpart of the ``.obj`` cloth and ``.msh`` FEM showcases.

DART-native showcase -- not a faithful IPC paper-figure reproduction.
"""

from __future__ import annotations

from pathlib import Path

import dartpy.simulation_experimental as sx

from .._ipc_deformable_bridge import IpcDeformableBridge, build_strand_from_seg
from ..runner import PythonDemoScene, SceneSetup

_STRAND_PATH = Path(__file__).resolve().parent.parent / "assets" / "strand.seg"


def build() -> SceneSetup:
    options, edges = build_strand_from_seg(
        _STRAND_PATH,
        mass=0.05,
        edge_stiffness=150.0,
        damping=1.0,
        translate=(-0.33, 0.0, 0.1),
    )
    # Pin the first vertex so the strand hangs from it.
    options.fixed_nodes = [0]

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 0.005
    body = world.add_deformable_body("seg_strand", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_seg_strand")
    bridge.add_deformable_visual(body, name="seg_strand", edges=edges)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_seg_strand",
    title="Deformable .seg Strand (IPC)",
    category="IPC Deformable (sx)",
    summary="A mass-spring strand loaded from a .seg segment mesh hangs and "
    "swings from a pinned end.",
    build=build,
)
