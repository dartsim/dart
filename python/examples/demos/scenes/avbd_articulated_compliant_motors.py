"""Finite-stiffness AVBD articulated movable-pair motor showcase."""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 0.005
_START_STIFFNESS = 20.0
_LINEAR_STIFFNESS = 2000.0
_ANGULAR_STIFFNESS = 2000.0
_HINGE_SPEED = 0.8
_SLIDER_SPEED = 0.45
_MAX_EFFORT = 800.0
_COMMAND_SWITCH_SECONDS = 0.30
_AUTO_RESET_SECONDS = 1.20
_HINGE_CENTER = np.array([-1.55, 0.0, 1.05])
_SLIDER_CENTER = np.array([1.55, 0.0, 1.05])
_PARENT_ANCHOR = np.array([0.45, 0.0, 0.0])
_CHILD_ANCHOR = np.array([-0.45, 0.0, 0.0])


def _translation(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = position
    return transform


def _rotation_angle(rotation: np.ndarray) -> float:
    cosine = 0.5 * (float(np.trace(rotation)) - 1.0)
    return float(np.arccos(np.clip(cosine, -1.0, 1.0)))


def _link_rotation(link: sx.Link) -> np.ndarray:
    return np.asarray(link.rotation, dtype=float).reshape(3, 3)


def _link_translation(link: sx.Link) -> np.ndarray:
    return np.asarray(link.translation, dtype=float).reshape(3)


def _world_point(link: sx.Link, local_point: np.ndarray) -> np.ndarray:
    return _link_rotation(link) @ local_point + _link_translation(link)


def _configure_finite_motor(
    joint: sx.Joint, target_speed: float
) -> sx.Joint:
    joint.actuator_type = sx.ActuatorType.VELOCITY
    joint.command_velocity = [target_speed]
    joint.set_effort_limits([-_MAX_EFFORT], [_MAX_EFFORT])
    policy = joint.constraint_projection_policy
    policy.start_stiffness = _START_STIFFNESS
    policy.linear_stiffness = _LINEAR_STIFFNESS
    policy.angular_stiffness = _ANGULAR_STIFFNESS
    joint.constraint_projection_policy = policy
    return joint


def build() -> SceneSetup:
    world = sx.World(time_step=_TIME_STEP, gravity=(0.0, 0.0, 0.0))
    world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )

    robot = world.add_multibody("avbd_articulated_compliant_motors")
    base = robot.add_link("avbd_articulated_compliant_motor_base")

    initial_positions = {
        "hinge_parent": _HINGE_CENTER - _PARENT_ANCHOR,
        "hinge_child": _HINGE_CENTER - _CHILD_ANCHOR,
        "slider_parent": _SLIDER_CENTER - _PARENT_ANCHOR,
        "slider_child": _SLIDER_CENTER - _CHILD_ANCHOR,
    }
    links: dict[str, sx.Link] = {}
    for name, position in initial_positions.items():
        link = robot.add_link(
            f"avbd_articulated_compliant_{name}",
            parent=base,
            joint=sx.JointSpec(
                name=f"avbd_articulated_compliant_{name}_floating",
                type=sx.JointType.FLOATING,
            ),
        )
        link.mass = 1.0 if name.endswith("parent") else 1.2
        link.inertia = ((0.16, 0.0, 0.0), (0.0, 0.22, 0.0), (0.0, 0.0, 0.28))
        link.parent_joint.position = [*position, 0.0, 0.0, 0.0]
        links[name] = link

    hinge = _configure_finite_motor(
        world.add_joint(
            links["hinge_parent"],
            links["hinge_child"],
            sx.JointSpec(
                name="avbd_articulated_compliant_hinge_motor",
                type=sx.JointType.REVOLUTE,
                axis=(0.0, 0.0, 1.0),
                parent_anchor=_PARENT_ANCHOR.tolist(),
                child_anchor=_CHILD_ANCHOR.tolist(),
            ),
        ),
        _HINGE_SPEED,
    )
    slider = _configure_finite_motor(
        world.add_joint(
            links["slider_parent"],
            links["slider_child"],
            sx.JointSpec(
                name="avbd_articulated_compliant_slider_motor",
                type=sx.JointType.PRISMATIC,
                axis=(1.0, 0.0, 0.0),
                parent_anchor=_PARENT_ANCHOR.tolist(),
                child_anchor=_CHILD_ANCHOR.tolist(),
            ),
        ),
        _SLIDER_SPEED,
    )
    joints = {"hinge": hinge, "slider": slider}

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(
        world, name="avbd_articulated_compliant_motors_render"
    )
    colors = {
        "hinge_parent": (0.22, 0.52, 0.84),
        "hinge_child": (0.95, 0.57, 0.16),
        "slider_parent": (0.28, 0.70, 0.46),
        "slider_child": (0.73, 0.42, 0.86),
    }
    shapes = {
        "hinge_parent": dart.BoxShape(np.array([0.82, 0.18, 0.18])),
        "hinge_child": dart.BoxShape(np.array([0.82, 0.18, 0.18])),
        "slider_parent": dart.BoxShape(np.array([0.58, 0.28, 0.22])),
        "slider_child": dart.BoxShape(np.array([0.58, 0.28, 0.22])),
    }
    for name, link in links.items():
        bridge.add_link_visual(
            link,
            shapes[name],
            colors[name],
            name=f"avbd_articulated_compliant_{name}_visual",
        )

    for name, center in (
        ("hinge", _HINGE_CENTER),
        ("slider", _SLIDER_CENTER),
    ):
        marker = dart.SimpleFrame(
            dart.gui.world_render_frame(),
            f"avbd_articulated_compliant_{name}_anchor",
            _translation(center),
        )
        marker.set_shape(dart.SphereShape(0.075))
        marker.create_visual_aspect().set_color([0.86, 0.86, 0.80])
        bridge.render_world.add_simple_frame(marker)

    hinge_axis = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        "avbd_articulated_compliant_motor_hinge_axis",
        _translation(_HINGE_CENTER),
    )
    hinge_axis.set_shape(dart.CylinderShape(0.025, 0.75))
    hinge_axis.create_visual_aspect().set_color([0.92, 0.38, 0.18])
    bridge.render_world.add_simple_frame(hinge_axis)

    slider_rail = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        "avbd_articulated_compliant_motor_slider_rail",
        _translation(_SLIDER_CENTER),
    )
    slider_rail.set_shape(dart.BoxShape(np.array([2.6, 0.035, 0.035])))
    slider_rail.create_visual_aspect().set_color([0.74, 0.78, 0.84])
    bridge.render_world.add_simple_frame(slider_rail)

    replay_state = {
        "enabled": True,
        "last_reset": 0.0,
        "direction": 1.0,
        "cycles": 0,
    }

    def metrics() -> dict[str, float]:
        hinge_parent_rotation = _link_rotation(links["hinge_parent"])
        hinge_child_rotation = _link_rotation(links["hinge_child"])
        hinge_relative_rotation = (
            hinge_parent_rotation.T @ hinge_child_rotation
        )
        hinge_anchor_delta = _world_point(
            links["hinge_child"], _CHILD_ANCHOR
        ) - _world_point(links["hinge_parent"], _PARENT_ANCHOR)

        slider_parent_rotation = _link_rotation(links["slider_parent"])
        slider_child_rotation = _link_rotation(links["slider_child"])
        slider_relative_rotation = (
            slider_parent_rotation.T @ slider_child_rotation
        )
        slider_axis_world = slider_parent_rotation @ np.array([1.0, 0.0, 0.0])
        slider_anchor_delta = _world_point(
            links["slider_child"], _CHILD_ANCHOR
        ) - _world_point(links["slider_parent"], _PARENT_ANCHOR)
        slider_travel = float(slider_anchor_delta @ slider_axis_world)

        return {
            "hinge_angle": float(
                np.arctan2(
                    hinge_relative_rotation[1, 0],
                    hinge_relative_rotation[0, 0],
                )
            ),
            "hinge_anchor_error": float(np.linalg.norm(hinge_anchor_delta)),
            "hinge_axis_tilt": float(
                np.linalg.norm(
                    hinge_parent_rotation @ np.array([0.0, 0.0, 1.0])
                    - hinge_child_rotation @ np.array([0.0, 0.0, 1.0])
                )
            ),
            "slider_travel": slider_travel,
            "slider_transverse_error": float(
                np.linalg.norm(
                    slider_anchor_delta - slider_travel * slider_axis_world
                )
            ),
            "slider_rotation_error": _rotation_angle(
                slider_relative_rotation
            ),
        }

    def apply_commands() -> None:
        direction = float(replay_state["direction"])
        hinge.command_velocity = [direction * _HINGE_SPEED]
        slider.command_velocity = [direction * _SLIDER_SPEED]

    def reset_pairs() -> None:
        for name, link in links.items():
            link.parent_joint.position = [
                *initial_positions[name],
                0.0,
                0.0,
                0.0,
            ]
            link.parent_joint.velocity = [0.0] * 6
        replay_state["last_reset"] = float(world.time)
        replay_state["direction"] = 1.0
        replay_state["cycles"] = int(replay_state["cycles"]) + 1
        apply_commands()
        bridge.sync()

    def replay_capture_state() -> dict[str, object]:
        return dict(replay_state)

    def replay_restore_state(state: dict[str, object]) -> None:
        replay_state["enabled"] = bool(state.get("enabled", True))
        replay_state["last_reset"] = float(state.get("last_reset", 0.0))
        replay_state["direction"] = float(state.get("direction", 1.0))
        replay_state["cycles"] = int(state.get("cycles", 0))
        apply_commands()
        bridge.sync()

    def pre_step() -> None:
        elapsed = float(world.time) - float(replay_state["last_reset"])
        if bool(replay_state["enabled"]):
            switch_index = int(elapsed / _COMMAND_SWITCH_SECONDS)
            replay_state["direction"] = 1.0 if switch_index % 2 == 0 else -1.0
            if elapsed >= _AUTO_RESET_SECONDS:
                reset_pairs()
        apply_commands()
        bridge.pre_step()

    bridge.sync()

    hinge_history: deque[float] = deque(maxlen=160)
    slider_history: deque[float] = deque(maxlen=160)
    residual_history: deque[float] = deque(maxlen=160)

    def build_panel(builder: object, context: object) -> None:
        values = metrics()
        hinge_history.append(values["hinge_angle"])
        slider_history.append(values["slider_travel"])
        residual_history.append(
            max(
                values["hinge_anchor_error"],
                values["slider_transverse_error"],
            )
        )

        builder.text("solver: AVBD finite masked rows + bounded motor rows")
        builder.text("endpoints: two movable same-multibody link pairs")
        builder.text(f"start stiffness: {_START_STIFFNESS:.1f}")
        builder.text(f"linear/angular cap: {_LINEAR_STIFFNESS:.1f}")
        builder.text(f"effort cap: {_MAX_EFFORT:.1f}")
        builder.text(f"command direction: {int(replay_state['direction']):+d}")
        builder.text(f"hinge angle: {values['hinge_angle']:.3f} rad")
        builder.text(
            f"hinge anchor error: {values['hinge_anchor_error']:.4f} m"
        )
        builder.text(f"hinge axis tilt: {values['hinge_axis_tilt']:.4f}")
        builder.text(f"slider travel: {values['slider_travel']:.3f} m")
        builder.text(
            "slider transverse error: "
            f"{values['slider_transverse_error']:.4f} m"
        )
        builder.text(
            f"slider rotation error: {values['slider_rotation_error']:.4f} rad"
        )
        builder.text(f"replay cycles: {int(replay_state['cycles'])}")
        if builder.button("Reset motor pairs"):
            reset_pairs()
        changed, enabled = builder.checkbox(
            "Loop replay",
            bool(replay_state["enabled"]),
        )
        if changed:
            replay_state["enabled"] = bool(enabled)
            replay_state["last_reset"] = float(world.time)
        builder.plot_lines("Hinge angle", list(hinge_history))
        builder.plot_lines("Slider travel", list(slider_history))
        builder.plot_lines("Max constrained residual", list(residual_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Articulated Compliant Motors", build_panel)],
        info={
            "sx_world": world,
            "base": base,
            "links": links,
            "joints": joints,
            "metrics": metrics,
            "reset_pairs": reset_pairs,
            "replay_state": replay_state,
            "replay_capture_state": replay_capture_state,
            "replay_restore_state": replay_restore_state,
            "time_step": _TIME_STEP,
            "start_stiffness": _START_STIFFNESS,
            "linear_stiffness": _LINEAR_STIFFNESS,
            "angular_stiffness": _ANGULAR_STIFFNESS,
            "hinge_speed": _HINGE_SPEED,
            "slider_speed": _SLIDER_SPEED,
            "max_effort": _MAX_EFFORT,
            "command_switch_seconds": _COMMAND_SWITCH_SECONDS,
            "auto_reset_seconds": _AUTO_RESET_SECONDS,
        },
    )


SCENE = PythonDemoScene(
    id="avbd_articulated_compliant_motors",
    title="AVBD Articulated Compliant Motors (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="Finite-stiffness same-multibody revolute and prismatic pairs "
    "share their free coordinates with bounded velocity-motor rows.",
    build=build,
)
