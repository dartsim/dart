"""Strand loaded from a .seg segment mesh hanging under gravity (experimental).

A one-dimensional mass-spring strand is loaded from a bundled ``.seg`` segment
mesh (the new ``load_seg_line_mesh`` importer, PLAN-081 M3 asset pipeline), pinned
at one end, and released so it hangs and swings under gravity -- the codimensional
(1D) counterpart of the ``.obj`` cloth and ``.msh`` FEM showcases.

DART-native showcase -- not a faithful IPC paper-figure reproduction.
"""

from __future__ import annotations

from collections import deque
from pathlib import Path

import numpy as np
import dartpy.simulation_experimental as sx

from .._ipc_deformable_bridge import IpcDeformableBridge, build_strand_from_seg
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

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

    tip_index = int(body.node_count) - 1
    tip_drop_history = deque(maxlen=120)
    span_z_history = deque(maxlen=120)
    speed_history = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        positions = np.asarray(
            [body.node_position(i) for i in range(int(body.node_count))],
            dtype=float,
        )
        velocities = np.asarray(
            [body.node_velocity(i) for i in range(int(body.node_count))],
            dtype=float,
        )
        builder.text("asset: strand.seg")
        builder.text("pins: first vertex")
        if positions.size:
            tip_drop = float(positions[0, 2] - positions[tip_index, 2])
            span_z = float(np.max(positions[:, 2]) - np.min(positions[:, 2]))
            mean_speed = (
                float(np.mean(np.linalg.norm(velocities, axis=1)))
                if velocities.size
                else 0.0
            )
            tip_drop_history.append(tip_drop)
            span_z_history.append(span_z)
            speed_history.append(mean_speed)
            builder.text(f"tip drop: {tip_drop:.3f} m")
            builder.text(f"vertical span: {span_z:.3f} m")
            builder.text(f"mean node speed: {mean_speed:.3f} m/s")
        builder.separator()
        bridge.build_diagnostics_panel(builder, context)
        if tip_drop_history:
            builder.separator()
            builder.plot_lines("Tip drop", list(tip_drop_history))
            builder.plot_lines("Span z", list(span_z_history))
            builder.plot_lines("Mean speed", list(speed_history))

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        panels=[ScenePanel("IPC SEG Strand", build_panel)],
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_seg_strand",
    title="Deformable .seg Strand (IPC)",
    category="IPC Deformable",
    summary="A mass-spring strand loaded from a .seg segment mesh hangs and "
    "swings from a pinned end.",
    build=build,
)
