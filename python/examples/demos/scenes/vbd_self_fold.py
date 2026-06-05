"""VBD surface self-contact: one body's two cloth layers stay apart.

A single deformable body carries two horizontal cloth layers -- a wide pinned
bottom layer and a smaller free top layer released just above it. Under gravity
the top layer drops onto the bottom layer; because both are surfaces of the same
body, the World VBD path's point-triangle / edge-edge self-contact barrier holds
them at a positive separation instead of letting one pass through the other.
This isolates VBD surface self-collision (no obstacles, no second body): the
capability that lets a VBD cloth or volumetric body fold against itself without
interpenetrating.
"""

from __future__ import annotations

from collections import deque

import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_BOTTOM_SIDE = 7
_TOP_SIDE = 5
_LAYER_SPACING = 0.09
_BOTTOM_HEIGHT = 0.0
_TOP_INITIAL_HEIGHT = 0.07


def _add_layer(state, base, side, spacing, height, mass, is_fixed):
    """Append a `side`x`side` cloth grid (nodes, springs, surface triangles) to
    the running ``state`` lists, returning the next free node index. Lists are
    built locally and assigned to the options once (the bindings' vector getters
    return copies, so in-place appends on the options would not persist)."""
    positions, masses, fixed, edges, triangles = state
    half = 0.5 * spacing * (side - 1)

    def idx(col, row):
        return base + row * side + col

    for row in range(side):
        for col in range(side):
            positions.append(
                np.array([spacing * col - half, spacing * row - half, height])
            )
            masses.append(mass)
            if is_fixed:
                fixed.append(idx(col, row))
    for row in range(side):
        for col in range(side):
            if col + 1 < side:
                edges.append(sx.DeformableEdge(idx(col, row), idx(col + 1, row), -1.0))
            if row + 1 < side:
                edges.append(sx.DeformableEdge(idx(col, row), idx(col, row + 1), -1.0))
            if col + 1 < side and row + 1 < side:
                # Two triangles per quad form the collidable surface; a shear
                # spring along one diagonal keeps the layer from racking.
                edges.append(
                    sx.DeformableEdge(idx(col, row), idx(col + 1, row + 1), -1.0)
                )
                triangles.append(
                    sx.DeformableSurfaceTriangle(
                        idx(col, row), idx(col + 1, row), idx(col + 1, row + 1)
                    )
                )
                triangles.append(
                    sx.DeformableSurfaceTriangle(
                        idx(col, row), idx(col + 1, row + 1), idx(col, row + 1)
                    )
                )
    return base + side * side


def _make_two_layer_options():
    state = ([], [], [], [], [])  # positions, masses, fixed, edges, triangles
    # Wide pinned bottom layer at z = 0.
    nxt = _add_layer(
        state,
        0,
        side=_BOTTOM_SIDE,
        spacing=_LAYER_SPACING,
        height=_BOTTOM_HEIGHT,
        mass=1.0,
        is_fixed=True,
    )
    # Smaller free top layer released just above it.
    _add_layer(
        state,
        nxt,
        side=_TOP_SIDE,
        spacing=_LAYER_SPACING,
        height=_TOP_INITIAL_HEIGHT,
        mass=0.05,
        is_fixed=False,
    )

    positions, masses, fixed, edges, triangles = state
    options = sx.DeformableBodyOptions()
    options.positions = positions
    options.masses = masses
    options.fixed_nodes = fixed
    options.edges = edges
    options.surface_triangles = triangles
    options.edge_stiffness = 300.0
    options.damping = 0.8
    return options


def build() -> SceneSetup:
    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 0.003
    world.add_deformable_body("self_fold", _make_two_layer_options())

    solver = sx.DeformableSolverOptions()
    solver.iterations = 60
    world.configure_deformable_solver("self_fold", solver)
    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="vbd_self_fold")
    body = world.get_deformable_body("self_fold")
    bridge.add_deformable_visual(
        body,
        (0.85, 0.45, 0.55),
        radius=0.02,
        fixed_color=(0.30, 0.65, 0.95),
        thickness=3.0,
    )
    bridge.sync()

    bottom_count = _BOTTOM_SIDE * _BOTTOM_SIDE
    bottom_indices = np.arange(bottom_count)
    top_indices = np.arange(bottom_count, int(body.node_count))
    clearance_history = deque(maxlen=120)
    top_drop_history = deque(maxlen=120)
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
        builder.text("solver: VBD surface self-contact")
        builder.text(f"bottom layer: {_BOTTOM_SIDE} x {_BOTTOM_SIDE} pinned")
        builder.text(f"top layer: {_TOP_SIDE} x {_TOP_SIDE} free")
        if positions.size:
            bottom_max_z = float(np.max(positions[bottom_indices, 2]))
            top_min_z = float(np.min(positions[top_indices, 2]))
            top_mean_z = float(np.mean(positions[top_indices, 2]))
            clearance = top_min_z - bottom_max_z
            top_drop = _TOP_INITIAL_HEIGHT - top_mean_z
            top_speeds = (
                np.linalg.norm(velocities[top_indices], axis=1)
                if velocities.size
                else np.zeros(0)
            )
            mean_top_speed = float(np.mean(top_speeds)) if top_speeds.size else 0.0
            clearance_history.append(clearance)
            top_drop_history.append(top_drop)
            speed_history.append(mean_top_speed)
            builder.text(f"layer clearance: {clearance:.5f} m")
            builder.text(f"top mean height: {top_mean_z:.4f} m")
            builder.text(f"top layer drop: {top_drop:.4f} m")
            builder.text(f"mean top speed: {mean_top_speed:.3f} m/s")
        diagnostics = getattr(world, "last_deformable_solver_diagnostics", None)
        if diagnostics is not None:
            builder.text(
                f"solver iters: {diagnostics.solver_iterations} | "
                f"self contacts: {diagnostics.self_contact_barrier_active_contacts}"
            )
            min_distance = float(diagnostics.min_active_contact_distance)
            if np.isfinite(min_distance):
                builder.text(f"min active distance: {min_distance:.5f} m")
        if clearance_history:
            builder.separator()
            builder.plot_lines("Layer clearance", list(clearance_history))
            builder.plot_lines("Top layer drop", list(top_drop_history))
            builder.plot_lines("Top speed", list(speed_history))

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        panels=[ScenePanel("VBD Self Contact", build_panel)],
        info={"sx_world": world, "nodes": int(body.node_count)},
    )


SCENE = PythonDemoScene(
    id="vbd_self_fold",
    title="VBD Surface Self-Contact",
    category="Vertex Block Descent",
    summary="One body's free top cloth layer settles on its pinned bottom layer "
    "without passing through it, via VBD surface self-contact.",
    build=build,
)
