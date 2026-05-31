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

from collections import deque

import numpy as np
import dartpy.simulation_experimental as sx

from .._ipc_deformable_bridge import IpcDeformableBridge, build_fem_compression_bar
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_COMPRESSION_RATE = 0.12
_COMPRESSION_END_TIME = 3.0


def build() -> SceneSetup:
    options, edges = build_fem_compression_bar(
        cells_x=24,
        cells_y=2,
        cells_z=2,
        cell_size=0.05,
        origin=(-0.6, -0.05, 0.5),
        youngs_modulus=2.0e4,
        compression_rate=_COMPRESSION_RATE,
        compression_end_time=_COMPRESSION_END_TIME,
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

    span_history = {
        "x": deque(maxlen=120),
        "y": deque(maxlen=120),
        "z": deque(maxlen=120),
    }

    def build_panel(builder: object, context: object) -> None:
        progress = min(1.0, float(world.time) / _COMPRESSION_END_TIME)
        builder.text(f"compression: {100.0 * progress:.0f}%")
        builder.text(f"drive speed: {_COMPRESSION_RATE:.2f} m/s")
        builder.text("boundary: opposing end-face Dirichlet drives")
        positions = np.asarray(
            [body.node_position(i) for i in range(int(body.node_count))],
            dtype=float,
        )
        if positions.size:
            span = np.max(positions, axis=0) - np.min(positions, axis=0)
            span_history["x"].append(float(span[0]))
            span_history["y"].append(float(span[1]))
            span_history["z"].append(float(span[2]))
            builder.text(
                f"span xyz: {span[0]:.3f} x {span[1]:.3f} x {span[2]:.3f} m"
            )
        diagnostics = world.last_deformable_solver_diagnostics
        builder.text(
            f"solver iters: {diagnostics.solver_iterations} | "
            f"contacts: {diagnostics.self_contact_barrier_active_contacts}"
        )
        builder.separator()
        bridge.build_diagnostics_panel(builder, context)
        if span_history["x"]:
            builder.separator()
            builder.plot_lines("Span x", list(span_history["x"]))
            builder.plot_lines("Span y", list(span_history["y"]))
            builder.plot_lines("Span z", list(span_history["z"]))

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        panels=[ScenePanel("IPC FEM Buckle", build_panel)],
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
