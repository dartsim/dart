"""FEM cube dropped onto a ground barrier (IPC deformable solver).

A free tetrahedral FEM cube falls under gravity onto a static ground barrier,
squashes on impact, and settles intersection-free on the barrier surface. This
is the first DART showcase combining stable neo-Hookean *volumetric* FEM
elasticity (PLAN-081 M1) with the landed IPC clamped-log ground barrier contact
-- a step toward the paper's volumetric drop scenes (e.g. Fig. 12 mass/stiffness
drops).

DART-native FEM showcase -- not a faithful paper-figure reproduction (single
analytic ground barrier; no codimensional obstacles or friction).
"""

from __future__ import annotations

from collections import deque

import numpy as np
import dartpy as sx

from .._ipc_deformable_bridge import IpcDeformableBridge, build_fem_bar
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_GROUND_CENTER = (0.0, 0.0, -0.06)
_GROUND_HALF = (1.2, 1.2, 0.06)  # top face at z = 0.0


def build() -> SceneSetup:
    # A cube of FEM tetrahedra, released above the barrier (no pinned nodes).
    options, edges = build_fem_bar(
        cells_x=3,
        cells_y=3,
        cells_z=3,
        cell_size=0.08,
        origin=(-0.12, -0.12, 0.34),
        youngs_modulus=1.5e5,
        poisson_ratio=0.3,
        density=1.0e3,
    )
    options.fixed_nodes = []

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 0.004

    ground = world.add_rigid_body("ground", position=_GROUND_CENTER)
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))
    policy = ground.deformable_obstacle_policy
    policy.ground_barrier = True
    ground.deformable_obstacle_policy = policy

    body = world.add_deformable_body("fem_drop", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_fem_drop")
    bridge.add_rigid_box_visual(
        _GROUND_CENTER,
        tuple(2.0 * h for h in _GROUND_HALF),
        (0.37, 0.40, 0.43),
        name="ground_visual",
    )
    bridge.add_deformable_visual(body, name="fem_drop", edges=edges)

    clearance_history = deque(maxlen=120)
    center_height_history = deque(maxlen=120)
    span_z_history = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        positions = np.asarray(
            [body.node_position(i) for i in range(int(body.node_count))],
            dtype=float,
        )
        builder.text("material: stable neo-Hookean FEM")
        builder.text("contact: ground barrier")
        if positions.size:
            ground_top = _GROUND_CENTER[2] + _GROUND_HALF[2]
            clearance = float(np.min(positions[:, 2]) - ground_top)
            center_height = float(np.mean(positions[:, 2]))
            span_z = float(np.max(positions[:, 2]) - np.min(positions[:, 2]))
            clearance_history.append(clearance)
            center_height_history.append(center_height)
            span_z_history.append(span_z)
            builder.text(f"ground clearance: {clearance:.4f} m")
            builder.text(f"cube center height: {center_height:.3f} m")
            builder.text(f"vertical span: {span_z:.3f} m")
        builder.separator()
        bridge.build_diagnostics_panel(builder, context)
        if clearance_history:
            builder.separator()
            builder.plot_lines("Ground clearance", list(clearance_history))
            builder.plot_lines("Cube center height", list(center_height_history))
            builder.plot_lines("Span z", list(span_z_history))

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        panels=[ScenePanel("IPC FEM Drop", build_panel)],
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_fem_drop",
    title="Deformable FEM Drop (IPC)",
    category="IPC Deformable",
    summary="A FEM cube falls and settles on an IPC ground barrier without penetrating.",
    build=build,
)
