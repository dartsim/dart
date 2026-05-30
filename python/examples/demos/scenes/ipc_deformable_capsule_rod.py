"""Cloth draping over a capsule rod obstacle (experimental IPC deformable solver).

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

import math
from pathlib import Path

import dartpy.simulation_experimental as sx

from .._ipc_deformable_bridge import IpcDeformableBridge, build_cloth_from_obj
from ..runner import PythonDemoScene, SceneSetup

_CLOTH_PATH = Path(__file__).resolve().parent.parent / "assets" / "cloth_grid.obj"
_ROD_RADIUS = 0.1
_ROD_HALF_HEIGHT = 0.45
_ROD_CENTER = (0.0, 0.0, 0.25)
# Lay the capsule axis (body z) along world y, via a -90 deg rotation about x.
_ROD_ORIENTATION = (math.cos(-math.pi / 4), math.sin(-math.pi / 4), 0.0, 0.0)


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
    rod.is_deformable_surface_ccd_obstacle = True

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

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_capsule_rod",
    title="Deformable Cloth over Capsule Rod (IPC)",
    category="IPC Deformable (sx)",
    summary="A cloth drapes over a static capsule rod obstacle via its "
    "codimensional radial barrier.",
    build=build,
)
