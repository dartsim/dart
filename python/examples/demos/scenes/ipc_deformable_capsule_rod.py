"""Cloth draping over a capsule rod obstacle (IPC deformable solver).

A mass-spring cloth (loaded from the bundled ``.obj`` mesh) is dropped over a
static horizontal capsule -- a rod/wire -- opted in as a deformable obstacle. The
capsule's clamped-log barrier (PLAN-081 M3 codimensional collision objects) pushes
the cloth nodes out along the radial normal from the rod axis, so the sheet drapes
over the curved rod and its two halves hang down on either side, intersection-free.
Unlike the sphere/box surface-CCD obstacles, the capsule obstacle is barrier-only,
so the connected sheet drapes freely rather than being limited as a rigid body.
A DART-native step toward the IPC paper's Fig. 18 codimensional-roller theme.

DART-native showcase -- not a faithful IPC paper-figure reproduction.
"""

from __future__ import annotations

from collections import deque
import math
from pathlib import Path

import dartpy as sx
import numpy as np

from .._ipc_deformable_bridge import IpcDeformableBridge, build_cloth_from_obj
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_CLOTH_PATH = Path(__file__).resolve().parent.parent / "assets" / "cloth_grid.obj"
_ROD_RADIUS = 0.1
_ROD_HALF_HEIGHT = 0.45
_ROD_CENTER = (0.0, 0.0, 0.25)
# Lay the capsule axis (body z) along world y, via a -90 deg rotation about x.
_ROD_ORIENTATION = (math.cos(-math.pi / 4), math.sin(-math.pi / 4), 0.0, 0.0)
_ROD_AXIS = np.array([0.0, 1.0, 0.0])


def _capsule_clearance(position: np.ndarray) -> float:
    """Signed distance from a point to the static world-y capsule rod."""

    center = np.asarray(_ROD_CENTER, dtype=float)
    offset = position - center
    axial = float(
        np.clip(np.dot(offset, _ROD_AXIS), -_ROD_HALF_HEIGHT, _ROD_HALF_HEIGHT)
    )
    closest = center + axial * _ROD_AXIS
    return float(np.linalg.norm(position - closest) - _ROD_RADIUS)


def build() -> SceneSetup:
    options, edges = build_cloth_from_obj(
        _CLOTH_PATH,
        mass=0.02,
        edge_stiffness=200.0,
        damping=1.0,
        translate=(0.0, 0.0, 0.5),
    )

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 0.004

    rod = world.add_rigid_body(
        "rod", position=_ROD_CENTER, orientation=_ROD_ORIENTATION
    )
    rod.is_static = True
    rod.set_collision_shape(sx.CollisionShape.capsule(_ROD_RADIUS, _ROD_HALF_HEIGHT))
    policy = rod.deformable_obstacle_policy
    policy.surface_obstacle = True
    rod.deformable_obstacle_policy = policy

    body = world.add_deformable_body("capsule_cloth", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_capsule_rod")
    # Approximate the horizontal rod with an axis-aligned box visual along y.
    bridge.add_rigid_box_visual(
        _ROD_CENTER,
        (2.0 * _ROD_RADIUS, 2.0 * (_ROD_HALF_HEIGHT + _ROD_RADIUS), 2.0 * _ROD_RADIUS),
        (0.52, 0.45, 0.34),
        name="rod_visual",
    )
    bridge.add_deformable_visual(body, name="capsule_cloth", edges=edges)

    clearance_history = deque(maxlen=120)
    sag_history = deque(maxlen=120)
    balance_history = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        positions = np.asarray(
            [body.node_position(i) for i in range(int(body.node_count))],
            dtype=float,
        )
        builder.text("obstacle: capsule rod radial barrier")
        builder.text(f"rod radius: {_ROD_RADIUS:.3f} m")
        if positions.size:
            clearances = np.asarray(
                [_capsule_clearance(position) for position in positions], dtype=float
            )
            min_clearance = float(np.min(clearances))
            min_z = float(np.min(positions[:, 2]))
            left = positions[positions[:, 0] < _ROD_CENTER[0]]
            right = positions[positions[:, 0] >= _ROD_CENTER[0]]
            left_mean_z = float(np.mean(left[:, 2])) if left.size else min_z
            right_mean_z = float(np.mean(right[:, 2])) if right.size else min_z
            balance = left_mean_z - right_mean_z
            clearance_history.append(min_clearance)
            sag_history.append(min_z)
            balance_history.append(balance)
            builder.text(f"rod clearance: {min_clearance:.4f} m")
            builder.text(f"lowest cloth z: {min_z:.3f} m")
            builder.text(f"left/right mean-z delta: {balance:.3f} m")
        diagnostics = world.last_deformable_solver_diagnostics
        builder.text(
            f"solver iters: {diagnostics.solver_iterations} | "
            f"contacts: {diagnostics.converged_active_contact_count}"
        )
        builder.separator()
        bridge.build_diagnostics_panel(builder, context)
        if clearance_history:
            builder.separator()
            builder.plot_lines("Rod clearance", list(clearance_history))
            builder.plot_lines("Lowest cloth z", list(sag_history))
            builder.plot_lines("Left-right balance", list(balance_history))

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        panels=[ScenePanel("IPC Capsule Rod", build_panel)],
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_capsule_rod",
    title="Deformable Cloth over Capsule Rod (IPC)",
    category="IPC Deformable",
    summary="A cloth drapes over a static capsule rod obstacle via its "
    "codimensional radial barrier.",
    build=build,
)
