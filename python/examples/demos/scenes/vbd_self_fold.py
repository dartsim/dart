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

import dartpy.simulation_experimental as sx
import numpy as np

from .._sx_bridge import SxRenderBridge
from ..runner import PythonDemoScene, SceneSetup


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
        state, 0, side=7, spacing=0.09, height=0.0, mass=1.0, is_fixed=True
    )
    # Smaller free top layer released just above it.
    _add_layer(state, nxt, side=5, spacing=0.09, height=0.07, mass=0.05, is_fixed=False)

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

    bridge = SxRenderBridge(world, name="vbd_self_fold")
    body = world.get_deformable_body("self_fold")
    bridge.add_deformable_visual(
        body,
        (0.85, 0.45, 0.55),
        radius=0.02,
        fixed_color=(0.30, 0.65, 0.95),
        thickness=3.0,
    )
    bridge.sync()

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "nodes": int(body.node_count)},
    )


SCENE = PythonDemoScene(
    id="vbd_self_fold",
    title="VBD Surface Self-Contact (sx)",
    category="Experimental",
    summary="One body's free top cloth layer settles on its pinned bottom layer "
    "without passing through it, via VBD surface self-contact.",
    build=build,
)
