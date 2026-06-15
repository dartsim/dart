"""Articulated public loop-closure preview for the DART 7 World facade."""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy as sx
from dartpy._dartpy import dynamics as _dyn

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_NUM_LINKS = 4
_LINK_LENGTH = 0.56
_LINK_MASS = 0.55


def _translation(x: float, y: float, z: float) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = (x, y, z)
    return transform


def _rotation_y(angle: float) -> np.ndarray:
    transform = np.eye(4)
    c = float(np.cos(angle))
    s = float(np.sin(angle))
    transform[:3, :3] = ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))
    return transform


def _isometry(transform: np.ndarray) -> dart.Isometry3:
    isometry = dart.Isometry3()
    isometry.set_matrix(transform)
    return isometry


def build() -> SceneSetup:
    world = sx.World()
    world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    world.gravity = (0.0, 0.0, -9.81)
    world.time_step = 0.005

    robot = world.add_multibody("variational_endpoint_chain")
    base = robot.add_link("base")

    links = []
    parent = base
    tip_target_transform = np.eye(4)
    for i in range(_NUM_LINKS):
        offset = 0.0 if i == 0 else _LINK_LENGTH
        initial_position = 0.18 * (-1.0 if i % 2 else 1.0)
        link = robot.add_link(
            f"link{i}",
            parent=parent,
            joint=sx.JointSpec(
                name=f"hinge{i}",
                type=sx.JointType.REVOLUTE,
                axis=(0.0, 1.0, 0.0),
                transform_from_parent=_translation(offset, 0.0, 0.0),
            ),
        )
        link.mass = _LINK_MASS
        ixx = 0.5 * _LINK_MASS * (0.05**2)
        itrans = _LINK_MASS * (_LINK_LENGTH**2) / 12.0
        link.inertia = ((ixx, 0.0, 0.0), (0.0, itrans, 0.0), (0.0, 0.0, itrans))
        link.parent_joint.position = [initial_position]
        links.append(link)
        parent = link
        tip_target_transform = (
            tip_target_transform @ _rotation_y(initial_position) @ _translation(offset, 0.0, 0.0)
        )

    tip_target = tip_target_transform[:3, 3]
    closure = world.add_loop_closure(
        "variational_endpoint_tip_closure",
        sx.LoopClosureSpec(
            frame_a=links[-1],
            frame_b=base,
            family=sx.LoopClosureFamily.POINT,
            offset_b=_translation(*tip_target),
        ),
    )
    closure.dynamics = sx.ClosureDynamicsPolicy.SOLVE

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="variational_endpoint_loop_closure_render")
    bridge.add_link_visual(
        base,
        dart.BoxShape(np.array([0.18, 0.18, 0.18])),
        (0.38, 0.40, 0.44),
        name="variational_endpoint_base_visual",
    )
    palette = [
        (0.20, 0.55, 0.90),
        (0.30, 0.72, 0.48),
        (0.92, 0.56, 0.20),
        (0.67, 0.40, 0.78),
    ]
    link_shape = dart.BoxShape(np.array([_LINK_LENGTH, 0.075, 0.075]))
    for i, link in enumerate(links):
        bridge.add_link_visual(
            link,
            link_shape,
            palette[i % len(palette)],
            name=f"variational_endpoint_link{i}_visual",
        )
    target_frame = dart.SimpleFrame(
        _dyn.Frame.world(),
        "variational_endpoint_target_visual",
        _isometry(_translation(*tip_target)),
    )
    target_frame.set_shape(dart.BoxShape(np.array([0.11, 0.11, 0.11])))
    target_frame.create_visual_aspect().set_color([0.92, 0.86, 0.22])
    bridge.render_world.add_simple_frame(target_frame)
    bridge.sync()

    tip_height_history: deque[float] = deque(maxlen=140)
    joint_speed_history: deque[float] = deque(maxlen=140)
    residual_history: deque[float] = deque(maxlen=140)

    def build_panel(builder: object, context: object) -> None:
        speeds = [
            float(np.linalg.norm(np.asarray(link.parent_joint.velocity, dtype=float)))
            for link in links
        ]
        tip_position = np.asarray(links[-1].translation, dtype=float).reshape(3)
        tip_height = float(tip_position[2])
        closure_residual = float(np.linalg.norm(tip_position - tip_target))
        tip_height_history.append(tip_height)
        joint_speed_history.append(max(speeds) if speeds else 0.0)
        residual_history.append(closure_residual)

        builder.text("solver: articulated sx world")
        builder.text("endpoint class: multibody link")
        builder.text("constraint path: public variational loop closure")
        builder.text("constraint family: POINT")
        builder.text(f"links: {robot.num_links}")
        builder.text(f"dofs: {robot.num_dofs}")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"tip height: {tip_height:.3f} m")
        builder.text(f"closure residual: {closure_residual:.4f} m")
        builder.plot_lines("Tip height", list(tip_height_history))
        builder.plot_lines("Max joint speed", list(joint_speed_history))
        builder.plot_lines("Closure residual", list(residual_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("Variational Endpoint Closure", build_panel)],
        info={
            "sx_world": world,
            "endpoint_kind": "multibody_link",
            "constraint_path": "public_variational_loop_closure",
            "constraint_family": "point",
            "dofs": robot.num_dofs,
        },
    )


SCENE = PythonDemoScene(
    id="variational_endpoint_loop_closure",
    title="Variational Endpoint Loop Closure (sx)",
    category="Variational Integrators",
    summary="A multibody-link point closure in the articulated variational solve path.",
    build=build,
)
