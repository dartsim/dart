"""Rigid-body one-DOF joint scene for the World facade."""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy.simulation_experimental as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_BASE_HALF = np.array([0.16, 0.16, 0.16])
_HINGE_HALF = np.array([0.28, 0.12, 0.12])
_SLIDER_HALF = np.array([0.18, 0.18, 0.22])
_HINGE_ANCHOR = np.array([-0.85, 0.0, 1.0])
_SLIDER_ANCHOR = np.array([0.85, 0.0, 1.0])


def _translation(x: float, y: float, z: float) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = (x, y, z)
    return transform


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * np.asarray(half_extents, dtype=float)


def build() -> SceneSetup:
    world = sx.World(time_step=0.005, gravity=(0.0, 0.0, 0.0))

    hinge_base = world.add_rigid_body("hinge_base", position=tuple(_HINGE_ANCHOR))
    hinge_base.is_static = True
    hinge_payload = world.add_rigid_body(
        "hinge_payload", position=tuple(_HINGE_ANCHOR + np.array([0.42, 0.0, 0.0]))
    )
    hinge_payload.mass = 1.0
    hinge_payload.angular_velocity = (0.0, 0.0, 1.4)
    hinge_joint = world.add_rigid_body_revolute_joint(
        "hinge_base_to_payload",
        hinge_base,
        hinge_payload,
        axis=(0.0, 0.0, 1.0),
    )

    slider_base = world.add_rigid_body("slider_base", position=tuple(_SLIDER_ANCHOR))
    slider_base.is_static = True
    slider_payload = world.add_rigid_body(
        "slider_payload", position=tuple(_SLIDER_ANCHOR + np.array([0.0, 0.0, 0.55]))
    )
    slider_payload.mass = 1.0
    slider_payload.linear_velocity = (0.0, 0.0, 0.45)
    slider_joint = world.add_rigid_body_prismatic_joint(
        "slider_base_to_payload",
        slider_base,
        slider_payload,
        axis=(0.0, 0.0, 1.0),
    )

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="rigid_limited_joints_render")
    bridge.add_rigid_body_visual(
        hinge_base,
        dart.BoxShape(_full(_BASE_HALF)),
        (0.30, 0.33, 0.38),
        name="hinge_base_visual",
    )
    bridge.add_rigid_body_visual(
        hinge_payload,
        dart.BoxShape(_full(_HINGE_HALF)),
        (0.88, 0.34, 0.18),
        name="hinge_payload_visual",
    )
    bridge.add_rigid_body_visual(
        slider_base,
        dart.BoxShape(_full(_BASE_HALF)),
        (0.30, 0.33, 0.38),
        name="slider_base_visual",
    )
    bridge.add_rigid_body_visual(
        slider_payload,
        dart.BoxShape(_full(_SLIDER_HALF)),
        (0.22, 0.54, 0.86),
        name="slider_payload_visual",
    )

    hinge_axis = dart.SimpleFrame(
        dart.Frame.world(),
        "hinge_axis_visual",
        _translation(*(_HINGE_ANCHOR + np.array([0.0, 0.0, 0.35]))),
    )
    hinge_axis.set_shape(dart.CylinderShape(0.025, 0.7))
    hinge_axis.create_visual_aspect().set_color([0.96, 0.78, 0.22])
    bridge.render_world.add_simple_frame(hinge_axis)

    slider_axis = dart.SimpleFrame(
        dart.Frame.world(),
        "slider_axis_visual",
        _translation(*(_SLIDER_ANCHOR + np.array([0.0, 0.0, 0.55]))),
    )
    slider_axis.set_shape(dart.CylinderShape(0.02, 1.1))
    slider_axis.create_visual_aspect().set_color([0.22, 0.78, 0.58])
    bridge.render_world.add_simple_frame(slider_axis)
    bridge.sync()

    hinge_error_history: deque[float] = deque(maxlen=120)
    slider_error_history: deque[float] = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        hinge_offset = np.asarray(hinge_payload.translation, dtype=float) - (
            _HINGE_ANCHOR + np.array([0.42, 0.0, 0.0])
        )
        slider_translation = np.asarray(slider_payload.translation, dtype=float)
        slider_orthogonal = slider_translation[:2] - _SLIDER_ANCHOR[:2]
        hinge_error = float(np.linalg.norm(hinge_offset[:2]))
        slider_error = float(np.linalg.norm(slider_orthogonal))
        hinge_error_history.append(hinge_error)
        slider_error_history.append(slider_error)

        builder.text("revolute: z-axis hinge")
        builder.text(f"hinge joint: {hinge_joint.name}")
        builder.text(f"hinge xy error: {hinge_error:.4f} m")
        builder.text("prismatic: z-axis slider")
        builder.text(f"slider joint: {slider_joint.name}")
        builder.text(f"slider orthogonal error: {slider_error:.4f} m")
        builder.text(f"rigid-body joints: {world.num_rigid_body_joints}")
        builder.text(f"world time: {world.time:.3f} s")
        builder.plot_lines("Hinge xy error", list(hinge_error_history))
        builder.plot_lines("Slider xy error", list(slider_error_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("Rigid One-DOF Joints", build_panel)],
        info={
            "sx_world": world,
            "hinge_joint": hinge_joint,
            "slider_joint": slider_joint,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_limited_joints",
    title="Rigid One-DOF Joints",
    category="World Rigid Body",
    summary="Revolute and prismatic rigid-body joints use captured AVBD rows.",
    build=build,
)
