"""FEM twisting bar (IPC deformable solver).

A tetrahedralized beam is gripped at both ends and counter-rotated about its
long axis, then released; the stable neo-Hookean FEM core stores the shear and
untwists elastically. This is the volumetric-twist counterpart of the IPC
paper's Fig. 4 (rod twist) and Fig. 14 (mat twist) stress tests, driven by the
opt-in FEM elasticity (PLAN-081 M1) plus scripted Dirichlet boundary conditions.

DART-native FEM showcase -- not a faithful paper-figure reproduction (no
self-contact buckling, codimensional contact, or friction); it isolates
large-shear volumetric elasticity.
"""

from __future__ import annotations

from collections import deque

import numpy as np
import dartpy.simulation_experimental as sx

from .._ipc_deformable_bridge import IpcDeformableBridge, build_fem_twist_bar
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_TWIST_END_TIME = 1.4


def build() -> SceneSetup:
    options, edges = build_fem_twist_bar(
        cells_x=10,
        cells_y=2,
        cells_z=2,
        cell_size=0.1,
        origin=(-0.5, -0.1, 0.7),
        youngs_modulus=8.0e5,
        twist_rate=0.35,
        twist_end_time=_TWIST_END_TIME,
        poisson_ratio=0.3,
        density=1.0e3,
    )

    world = sx.World()
    world.gravity = [0.0, 0.0, 0.0]  # isolate the twist from gravity sag
    world.time_step = 0.005
    body = world.add_deformable_body("fem_twist", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_fem_twist")
    bridge.add_deformable_visual(body, name="fem_twist", edges=edges)

    span_y_history = deque(maxlen=120)
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
        phase = "driving" if float(world.time) <= _TWIST_END_TIME else "released"
        builder.text("material: stable neo-Hookean FEM")
        builder.text(f"twist phase: {phase}")
        builder.text("gravity: off")
        if positions.size:
            span = np.max(positions, axis=0) - np.min(positions, axis=0)
            mean_speed = (
                float(np.mean(np.linalg.norm(velocities, axis=1)))
                if velocities.size
                else 0.0
            )
            span_y_history.append(float(span[1]))
            span_z_history.append(float(span[2]))
            speed_history.append(mean_speed)
            builder.text(f"span xyz: {span[0]:.3f} x {span[1]:.3f} x {span[2]:.3f} m")
            builder.text(f"mean node speed: {mean_speed:.3f} m/s")
        builder.separator()
        bridge.build_diagnostics_panel(builder, context)
        if span_y_history:
            builder.separator()
            builder.plot_lines("Span y", list(span_y_history))
            builder.plot_lines("Span z", list(span_z_history))
            builder.plot_lines("Mean speed", list(speed_history))

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        panels=[ScenePanel("IPC FEM Twist", build_panel)],
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_fem_twist",
    title="Deformable FEM Twist (IPC)",
    category="IPC Deformable",
    summary="A tetrahedral FEM bar is twisted at both ends and untwists elastically.",
    build=build,
)
