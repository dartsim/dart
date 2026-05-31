"""VBD cloth draping over a sphere obstacle, solved by VBD.

A free mass-spring cloth is dropped onto a static sphere resting on the ground.
The sphere is opted in as a deformable obstacle, so the World VBD path's
per-vertex obstacle barrier pushes the cloth's nodes out along the sphere's
surface normal while the ground barrier catches the overhanging edges -- the
cloth drapes over the curved obstacle without sinking through it. This showcases
the VBD static sphere/box obstacle handling: previously any obstacle forced the
body off the VBD path onto the default solver.
"""

from __future__ import annotations

import dartpy as dart
import dartpy.simulation_experimental as sx
import numpy as np

from .._sx_bridge import SxRenderBridge
from ..runner import PythonDemoScene, SceneSetup

_SPHERE_RADIUS = 0.32
_SPHERE_CENTER = (0.0, 0.0, _SPHERE_RADIUS)  # resting on the ground top (z = 0)
_GROUND_CENTER = (0.0, 0.0, -0.5)
_GROUND_HALF = (2.0, 2.0, 0.5)


def _make_cloth_options(side: int, spacing: float, height: float):
    options = sx.DeformableBodyOptions()
    half = 0.5 * spacing * (side - 1)

    positions = []
    masses = []
    for row in range(side):
        for col in range(side):
            positions.append(
                np.array([spacing * col - half, spacing * row - half, height])
            )
            masses.append(0.02)
    options.positions = positions
    options.masses = masses  # free cloth: no pinned nodes

    def index(col: int, row: int) -> int:
        return row * side + col

    edges = []
    for row in range(side):
        for col in range(side):
            if col + 1 < side:
                edges.append(
                    sx.DeformableEdge(index(col, row), index(col + 1, row), -1.0)
                )
            if row + 1 < side:
                edges.append(
                    sx.DeformableEdge(index(col, row), index(col, row + 1), -1.0)
                )
            if col + 1 < side and row + 1 < side:  # shear springs
                edges.append(
                    sx.DeformableEdge(index(col, row), index(col + 1, row + 1), -1.0)
                )
                edges.append(
                    sx.DeformableEdge(index(col + 1, row), index(col, row + 1), -1.0)
                )
    options.edges = edges
    options.edge_stiffness = 200.0
    options.damping = 0.6
    return options


def _add_static_visual(bridge, center, shape, color, name):
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(center, dtype=float)
    frame = dart.SimpleFrame(dart.Frame.world(), name, transform)
    frame.set_shape(shape)
    frame.create_visual_aspect().set_color(list(color))
    bridge.render_world.add_simple_frame(frame)


def build() -> SceneSetup:
    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 0.005

    ground = world.add_rigid_body("ground", position=_GROUND_CENTER)
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))
    ground.is_deformable_ground_barrier = True

    sphere = world.add_rigid_body("sphere", position=_SPHERE_CENTER)
    sphere.is_static = True
    sphere.set_collision_shape(sx.CollisionShape.sphere(_SPHERE_RADIUS))
    sphere.is_deformable_surface_ccd_obstacle = True

    # A free cloth slightly wider than the sphere, released just above its top.
    world.add_deformable_body(
        "drape",
        _make_cloth_options(side=11, spacing=0.08, height=_SPHERE_RADIUS * 2.0 + 0.15),
    )

    solver = sx.DeformableSolverOptions()
    solver.iterations = 40
    solver.ground_contact_stiffness = 5.0e3  # enables the VBD obstacle + ground penalty
    world.configure_deformable_solver("drape", solver)
    world.enter_simulation_mode()

    bridge = SxRenderBridge(world, name="vbd_obstacle_drape")
    body = world.get_deformable_body("drape")
    bridge.add_deformable_visual(body, (0.25, 0.62, 0.85), radius=0.02, thickness=3.0)
    _add_static_visual(
        bridge,
        _SPHERE_CENTER,
        dart.SphereShape(_SPHERE_RADIUS),
        (0.55, 0.45, 0.35),
        "sphere_visual",
    )
    _add_static_visual(
        bridge,
        _GROUND_CENTER,
        dart.BoxShape(np.array([2.0 * h for h in _GROUND_HALF])),
        (0.37, 0.40, 0.43),
        "ground_visual",
    )
    bridge.sync()

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "nodes": 11 * 11},
    )


SCENE = PythonDemoScene(
    id="vbd_obstacle_drape",
    title="VBD Cloth over Sphere (sx)",
    category="Experimental",
    summary="A VBD cloth drapes over a static sphere obstacle without sinking through it.",
    build=build,
)
