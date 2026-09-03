"""VBD tilted strand: a TinyVBD-inspired stress smoke, solved by VBD.

It keeps TinyVBD's 20 vertices, 30-degree tilt, structural stiffness 1e8,
1:1000 tip mass ratio, 1/60 s step, and 100 sweeps, but is not source-matched.
DART currently gives skip springs the structural stiffness instead of 100,
uses uniform spacing instead of TinyVBD's doubled final segment, adds damping,
and uses a different local Hessian/solve policy. It is the visible companion to
the equally unmatched ``BM_VbdTinyStrandStep`` smoke and must not be used for a
TinyVBD speedup or parity claim.
"""

from __future__ import annotations

from collections import deque

import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_NUM_VERTS = 20
_SPACING = 0.05
_TAN_ANGLE = 0.57735  # 30 degrees
# This spring-only stress scene has no surface triangles. Its valid-pair bound
# is zero, so the explicit evidence cap is the documented nonzero floor.
_SURFACE_CONTACT_CANDIDATE_CAPACITY = 1


def _make_strand_options() -> "sx.DeformableBodyOptions":
    options = sx.DeformableBodyOptions()
    positions = []
    masses = []
    for i in range(_NUM_VERTS):
        # Pinned at node 0; the strand extends in +x and tilts up in +z, then
        # sags under gravity. The heavy free end (1000x mass) against the stiff
        # springs is the high mass-ratio stress the TinyVBD scene exercises.
        positions.append(np.array([i * _SPACING, 0.0, 1.1 + i * _SPACING * _TAN_ANGLE]))
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
    options.surface_contact_candidate_capacity = (
        _SURFACE_CONTACT_CANDIDATE_CAPACITY
    )
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

    bridge = WorldRenderBridge(world, name="vbd_tilted_strand_render")
    body = world.get_deformable_body("vbd_tilted_strand")
    bridge.add_deformable_visual(
        body,
        (0.90, 0.40, 0.55),
        radius=0.04,
        fixed_color=(0.30, 0.65, 0.95),
        thickness=6.0,
    )
    bridge.sync()

    initial_positions = np.asarray(
        [body.node_position(i) for i in range(int(body.node_count))],
        dtype=float,
    )
    initial_tip_height = float(initial_positions[-1, 2])
    initial_end_to_end = float(
        np.linalg.norm(initial_positions[-1] - initial_positions[0])
    )
    tip_drop_history = deque(maxlen=120)
    stretch_history = deque(maxlen=120)
    speed_history = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        del context
        positions = np.asarray(
            [body.node_position(i) for i in range(int(body.node_count))],
            dtype=float,
        )
        velocities = np.asarray(
            [body.node_velocity(i) for i in range(int(body.node_count))],
            dtype=float,
        )
        builder.text("inspired by TinyVBD tilted strand; not source-matched")
        builder.text(f"nodes: {_NUM_VERTS} | pins: 1 | mass ratio: 1:1000")
        builder.text("edge stiffness: 1.00e8 | VBD sweeps: 100")
        if positions.size:
            tip_height = float(positions[-1, 2])
            tip_drop = initial_tip_height - tip_height
            end_to_end = float(np.linalg.norm(positions[-1] - positions[0]))
            stretch = end_to_end - initial_end_to_end
            mean_speed = (
                float(np.mean(np.linalg.norm(velocities, axis=1)))
                if velocities.size
                else 0.0
            )
            tip_drop_history.append(tip_drop)
            stretch_history.append(stretch)
            speed_history.append(mean_speed)
            builder.text(f"free-end height: {tip_height:.3f} m")
            builder.text(f"free-end drop: {tip_drop:.3f} m")
            builder.text(f"end-to-end stretch: {stretch:.5f} m")
            builder.text(f"mean node speed: {mean_speed:.3f} m/s")
        diagnostics = getattr(world, "last_deformable_solver_diagnostics", None)
        if diagnostics is not None:
            builder.text(f"VBD sweeps: {diagnostics.vbd_sweeps}")
        if tip_drop_history:
            builder.separator()
            builder.plot_lines("Free-end drop", list(tip_drop_history))
            builder.plot_lines("End-to-end stretch", list(stretch_history))
            builder.plot_lines("Mean speed", list(speed_history))

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        panels=[ScenePanel("VBD Tilted Strand", build_panel)],
        info={"sx_world": world, "nodes": _NUM_VERTS},
    )


SCENE = PythonDemoScene(
    id="vbd_tilted_strand",
    title="VBD Tilted Strand",
    category="Vertex Block Descent",
    summary=(
        "A TinyVBD-inspired but unmatched stiff 20-vertex strand with a "
        "1:1000 tip-mass ratio, solved by VBD."
    ),
    build=build,
)
