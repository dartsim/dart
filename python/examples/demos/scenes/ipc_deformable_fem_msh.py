"""FEM cantilever loaded from a GMSH .msh file (IPC deformable solver).

A tetrahedral beam is loaded from a GMSH ASCII ``.msh`` mesh file via
``dartpy.load_gmsh_tet_mesh`` (the new tet-mesh
importer), opted in to stable neo-Hookean FEM elasticity, pinned at its clamped
end, and released to sag under gravity. This demonstrates the file-loading path
for FEM bodies -- the gateway to driving the solver from external tet meshes
rather than procedurally built grids (PLAN-081 M4).

DART-native FEM showcase -- not a faithful IPC paper-figure reproduction.
"""

from __future__ import annotations

from collections import deque
from pathlib import Path

import numpy as np
import dartpy as sx

from .._ipc_deformable_bridge import IpcDeformableBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

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

    initial_positions = np.asarray(options.positions, dtype=float)
    free_end_indices = np.flatnonzero(
        np.isclose(initial_positions[:, 0], np.max(initial_positions[:, 0]))
    )
    free_end_height0 = float(np.mean(initial_positions[free_end_indices, 2]))
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
        builder.text("asset: fem_bar.msh")
        builder.text("material: stable neo-Hookean FEM")
        builder.text(f"pins: min-x face ({len(options.fixed_nodes)} nodes)")
        if positions.size:
            free_end_height = float(np.mean(positions[free_end_indices, 2]))
            tip_drop = free_end_height0 - free_end_height
            span_z = float(np.max(positions[:, 2]) - np.min(positions[:, 2]))
            mean_speed = (
                float(np.mean(np.linalg.norm(velocities, axis=1)))
                if velocities.size
                else 0.0
            )
            tip_drop_history.append(tip_drop)
            span_z_history.append(span_z)
            speed_history.append(mean_speed)
            builder.text(f"free-end height: {free_end_height:.3f} m")
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
        panels=[ScenePanel("IPC FEM MSH", build_panel)],
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_fem_msh",
    title="Deformable FEM from .msh (IPC)",
    category="IPC Deformable",
    summary="A FEM cantilever loaded from a GMSH .msh tet mesh sags under gravity.",
    build=build,
)
