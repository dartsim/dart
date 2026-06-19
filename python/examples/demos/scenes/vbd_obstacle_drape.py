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

from collections import deque

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_SPHERE_RADIUS = 0.32
_SPHERE_CENTER = (0.0, 0.0, _SPHERE_RADIUS)  # resting on the ground top (z = 0)
_GROUND_CENTER = (0.0, 0.0, -0.5)
_GROUND_HALF = (2.0, 2.0, 0.5)
_CLOTH_SIDE = 11
_CLOTH_SPACING = 0.08
_CLOTH_HEIGHT = _SPHERE_RADIUS * 2.0 + 0.15


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
    frame = dart.SimpleFrame(dart.gui.world_render_frame(), name, transform)
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
    policy = ground.deformable_obstacle_policy
    policy.ground_barrier = True
    ground.deformable_obstacle_policy = policy

    sphere = world.add_rigid_body("sphere", position=_SPHERE_CENTER)
    sphere.is_static = True
    sphere.set_collision_shape(sx.CollisionShape.sphere(_SPHERE_RADIUS))
    policy = sphere.deformable_obstacle_policy
    policy.surface_obstacle = True
    sphere.deformable_obstacle_policy = policy

    # A free cloth slightly wider than the sphere, released just above its top.
    world.add_deformable_body(
        "drape",
        _make_cloth_options(
            side=_CLOTH_SIDE,
            spacing=_CLOTH_SPACING,
            height=_CLOTH_HEIGHT,
        ),
    )

    solver = sx.DeformableSolverOptions()
    solver.iterations = 40
    solver.ground_contact_stiffness = 5.0e3  # enables the VBD obstacle + ground penalty
    world.configure_deformable_solver("drape", solver)
    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="vbd_obstacle_drape")
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

    sphere_center = np.asarray(_SPHERE_CENTER, dtype=float)
    sphere_clearance_history = deque(maxlen=120)
    ground_clearance_history = deque(maxlen=120)
    drape_depth_history = deque(maxlen=120)

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
        builder.text("solver: VBD with static obstacle barrier")
        builder.text(f"cloth: {_CLOTH_SIDE} x {_CLOTH_SIDE} nodes")
        builder.text(f"sphere radius: {_SPHERE_RADIUS:.2f} m")
        if positions.size:
            sphere_clearance = float(
                np.min(np.linalg.norm(positions - sphere_center, axis=1))
                - _SPHERE_RADIUS
            )
            ground_clearance = float(np.min(positions[:, 2]))
            drape_depth = _CLOTH_HEIGHT - ground_clearance
            mean_speed = (
                float(np.mean(np.linalg.norm(velocities, axis=1)))
                if velocities.size
                else 0.0
            )
            sphere_clearance_history.append(sphere_clearance)
            ground_clearance_history.append(ground_clearance)
            drape_depth_history.append(drape_depth)
            builder.text(f"sphere clearance: {sphere_clearance:.4f} m")
            builder.text(f"ground clearance: {ground_clearance:.4f} m")
            builder.text(f"drape depth: {drape_depth:.3f} m")
            builder.text(f"mean node speed: {mean_speed:.3f} m/s")
        diagnostics = getattr(world, "last_deformable_solver_diagnostics", None)
        if diagnostics is not None:
            builder.text(
                f"solver iters: {diagnostics.solver_iterations} | "
                f"contacts: {diagnostics.converged_active_contact_count}"
            )
        if sphere_clearance_history:
            builder.separator()
            builder.plot_lines("Sphere clearance", list(sphere_clearance_history))
            builder.plot_lines("Ground clearance", list(ground_clearance_history))
            builder.plot_lines("Drape depth", list(drape_depth_history))

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        panels=[ScenePanel("VBD Sphere Drape", build_panel)],
        info={"sx_world": world, "nodes": _CLOTH_SIDE * _CLOTH_SIDE},
    )


SCENE = PythonDemoScene(
    id="vbd_obstacle_drape",
    title="VBD Cloth over Sphere",
    category="Vertex Block Descent",
    summary="A VBD cloth drapes over a static sphere obstacle without sinking through it.",
    build=build,
)
