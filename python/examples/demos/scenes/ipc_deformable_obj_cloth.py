"""Cloth loaded from a Wavefront .obj draping under gravity (IPC).

A mass-spring cloth membrane is loaded from a bundled Wavefront ``.obj`` triangle
mesh (the new ``load_obj_triangle_mesh`` importer, PLAN-081 M3 asset pipeline),
pinned along one edge, and released so it sags and drapes under gravity, its free
edge settling onto the static ground barrier below. This is the surface-mesh
counterpart of the volumetric ``.msh`` FEM showcase, exercising the ``.obj``
importer feeding a real deformable solve plus the ground contact barrier.

DART-native showcase -- not a faithful IPC paper-figure reproduction.
"""

from __future__ import annotations

from collections import deque
from pathlib import Path

import numpy as np
import dartpy as sx

from .._ipc_deformable_bridge import IpcDeformableBridge, build_cloth_from_obj
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_CLOTH_PATH = Path(__file__).resolve().parent.parent / "assets" / "cloth_grid.obj"
_GROUND_CENTER = (0.0, 0.0, -0.26)
_GROUND_HALF = (1.2, 1.2, 0.06)


def build() -> SceneSetup:
    # Lift the flat sheet (authored in the z = 0 plane) and let it drape.
    options, edges = build_cloth_from_obj(
        _CLOTH_PATH,
        mass=0.02,
        edge_stiffness=200.0,
        damping=1.0,
        translate=(0.0, 0.0, 0.2),
    )

    # Pin the sheet's far (max-y) edge so the rest sags and drapes off it.
    ys = [float(p[1]) for p in options.positions]
    y_max = max(ys)
    options.fixed_nodes = [i for i, y in enumerate(ys) if y > y_max - 1e-4]
    fixed_nodes = list(options.fixed_nodes)

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 0.004

    ground = world.add_rigid_body("ground", position=_GROUND_CENTER)
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))
    policy = ground.deformable_obstacle_policy
    policy.ground_barrier = True
    ground.deformable_obstacle_policy = policy

    body = world.add_deformable_body("obj_cloth", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_obj_cloth")
    bridge.add_rigid_box_visual(
        _GROUND_CENTER,
        tuple(2.0 * h for h in _GROUND_HALF),
        (0.37, 0.40, 0.43),
        name="ground_visual",
    )
    bridge.add_deformable_visual(body, name="obj_cloth", edges=edges)

    sag_history = deque(maxlen=120)
    clearance_history = deque(maxlen=120)
    span_z_history = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        positions = np.asarray(
            [body.node_position(i) for i in range(int(body.node_count))],
            dtype=float,
        )
        builder.text("asset: cloth_grid.obj")
        builder.text(f"pins: max-y edge ({len(fixed_nodes)} nodes)")
        if positions.size:
            ground_top = _GROUND_CENTER[2] + _GROUND_HALF[2]
            pinned_z = (
                float(np.mean(positions[fixed_nodes, 2]))
                if fixed_nodes
                else float(np.max(positions[:, 2]))
            )
            min_z = float(np.min(positions[:, 2]))
            sag = pinned_z - min_z
            clearance = min_z - ground_top
            span_z = float(np.max(positions[:, 2]) - min_z)
            sag_history.append(sag)
            clearance_history.append(clearance)
            span_z_history.append(span_z)
            builder.text(f"cloth sag: {sag:.3f} m")
            builder.text(f"ground clearance: {clearance:.4f} m")
            builder.text(f"vertical span: {span_z:.3f} m")
        builder.separator()
        bridge.build_diagnostics_panel(builder, context)
        if sag_history:
            builder.separator()
            builder.plot_lines("Cloth sag", list(sag_history))
            builder.plot_lines("Ground clearance", list(clearance_history))
            builder.plot_lines("Span z", list(span_z_history))

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        panels=[ScenePanel("IPC OBJ Cloth", build_panel)],
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_obj_cloth",
    title="Deformable .obj Cloth (IPC)",
    category="IPC Deformable",
    summary="A cloth loaded from a Wavefront .obj is pinned at one edge and "
    "drapes under gravity onto the ground barrier.",
    build=build,
)
