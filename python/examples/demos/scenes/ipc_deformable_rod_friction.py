"""Friction against a capsule rod obstacle (experimental IPC deformable solver).

A short deformable strip rests on top of a static horizontal capsule rod and is
shoved along the rod's axis. Lagged smoothed Coulomb friction against the rod's
clamped-log contact barrier (PLAN-081 M5, mesh-vs-obstacle friction) opposes the
tangential slide, bringing the strip to rest -- whereas a frictionless strip
would coast along the rod. The capsule obstacle is barrier-only (no surface CCD),
so the tangential slide is unconstrained and the friction force is what stops it.

DART-native showcase -- not a faithful IPC paper-figure reproduction.
"""

from __future__ import annotations

import math

import dartpy.simulation_experimental as sx
import numpy as np

from .._ipc_deformable_bridge import IpcDeformableBridge
from ..runner import PythonDemoScene, SceneSetup

_ROD_RADIUS = 0.2
_ROD_HALF_HEIGHT = 3.0
_ROD_CENTER = (0.0, 0.0, 0.0)
# Lay the capsule axis (body z) along world y, via a -90 deg rotation about x.
_ROD_ORIENTATION = (math.cos(-math.pi / 4), math.sin(-math.pi / 4), 0.0, 0.0)


def _build_strip() -> tuple["sx.DeformableBodyOptions", list[tuple[int, int]]]:
    # A 5-node strip lying along the rod axis (y), resting on the rod top.
    options = sx.DeformableBodyOptions()
    top_z = _ROD_RADIUS + 0.012
    positions = [np.array([0.0, -0.4 + 0.2 * i, top_z]) for i in range(5)]
    options.positions = positions
    options.masses = [0.1] * 5
    options.edge_stiffness = 200.0
    options.damping = 1.0
    options.velocities = [np.array([0.0, 2.0, 0.0]) for _ in range(5)]
    edges = [(i, i + 1) for i in range(4)]
    options.edges = [
        sx.DeformableEdge(a, b, float(np.linalg.norm(positions[a] - positions[b])))
        for a, b in edges
    ]
    # Coulomb friction against the rod opposes the axial shove.
    options.material.friction_coefficient = 0.8
    return options, edges


def build() -> SceneSetup:
    options, edges = _build_strip()

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 0.004

    rod = world.add_rigid_body(
        "rod", position=_ROD_CENTER, orientation=_ROD_ORIENTATION
    )
    rod.is_static = True
    rod.set_collision_shape(sx.CollisionShape.capsule(_ROD_RADIUS, _ROD_HALF_HEIGHT))
    rod.is_deformable_surface_ccd_obstacle = True

    body = world.add_deformable_body("rod_strip", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_rod_friction")
    bridge.add_rigid_box_visual(
        _ROD_CENTER,
        (2.0 * _ROD_RADIUS, 2.0 * (_ROD_HALF_HEIGHT + _ROD_RADIUS), 2.0 * _ROD_RADIUS),
        (0.52, 0.45, 0.34),
        name="rod_visual",
    )
    bridge.add_deformable_visual(body, name="rod_strip", edges=edges)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_rod_friction",
    title="Deformable Friction on Capsule Rod (IPC)",
    category="IPC Deformable (sx)",
    summary="A deformable strip shoved along a capsule rod is brought to rest by "
    "Coulomb friction against the rod obstacle.",
    build=build,
)
