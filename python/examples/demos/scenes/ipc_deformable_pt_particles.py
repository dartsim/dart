"""Particles loaded from a .pt point set dropping onto the ground (experimental).

A cloud of free deformable particles is loaded from a bundled ``.pt`` point set
(the new ``load_point_set`` importer, PLAN-081 M3 asset pipeline) and dropped onto
the static ground barrier: with no springs or tetrahedra each node is an inertial
point mass, so the cloud falls under gravity and stacks on the ground barrier
intersection-free -- the codimensional (0D) counterpart of the ``.obj`` cloth and
``.seg`` strand showcases.

DART-native showcase -- not a faithful IPC paper-figure reproduction.
"""

from __future__ import annotations

from collections import deque
from pathlib import Path

import numpy as np
import dartpy as sx

from .._ipc_deformable_bridge import IpcDeformableBridge, build_particles_from_pt
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_POINTS_PATH = Path(__file__).resolve().parent.parent / "assets" / "particles.pt"
_GROUND_CENTER = (0.0, 0.0, -0.06)
_GROUND_HALF = (1.0, 1.0, 0.06)


def build() -> SceneSetup:
    options = build_particles_from_pt(
        _POINTS_PATH,
        mass=0.02,
        translate=(0.0, 0.0, 0.0),
    )

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 0.005

    ground = world.add_rigid_body("ground", position=_GROUND_CENTER)
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))
    policy = ground.deformable_obstacle_policy
    policy.ground_barrier = True
    ground.deformable_obstacle_policy = policy

    body = world.add_deformable_body("pt_particles", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_pt_particles")
    bridge.add_rigid_box_visual(
        _GROUND_CENTER,
        tuple(2.0 * h for h in _GROUND_HALF),
        (0.37, 0.40, 0.43),
        name="ground_visual",
    )
    bridge.add_deformable_visual(body, name="pt_particles", edges=[])

    initial_min_z = min(body.node_position(i)[2] for i in range(int(body.node_count)))
    clearance_history = deque(maxlen=120)
    fall_history = deque(maxlen=120)
    cloud_height_history = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        positions = np.asarray(
            [body.node_position(i) for i in range(int(body.node_count))],
            dtype=float,
        )
        builder.text("asset: particles.pt")
        builder.text("model: free point masses")
        if positions.size:
            ground_top = _GROUND_CENTER[2] + _GROUND_HALF[2]
            min_z = float(np.min(positions[:, 2]))
            max_z = float(np.max(positions[:, 2]))
            clearance = min_z - ground_top
            fall = float(initial_min_z - min_z)
            cloud_height = max_z - min_z
            clearance_history.append(clearance)
            fall_history.append(fall)
            cloud_height_history.append(cloud_height)
            builder.text(f"ground clearance: {clearance:.4f} m")
            builder.text(f"fall distance: {fall:.3f} m")
            builder.text(f"cloud height: {cloud_height:.3f} m")
        builder.separator()
        bridge.build_diagnostics_panel(builder, context)
        if clearance_history:
            builder.separator()
            builder.plot_lines("Ground clearance", list(clearance_history))
            builder.plot_lines("Fall distance", list(fall_history))
            builder.plot_lines("Cloud height", list(cloud_height_history))

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        panels=[ScenePanel("IPC PT Particles", build_panel)],
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_pt_particles",
    title="Deformable .pt Particles (IPC)",
    category="IPC Deformable",
    summary="A cloud of point particles loaded from a .pt file falls and stacks "
    "on the ground barrier.",
    build=build,
)
