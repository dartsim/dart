"""Finite-stiffness AVBD articulated joint-mask showcase."""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 0.002
_START_STIFFNESS = 10.0
_LINEAR_STIFFNESS = 1000.0
_ANGULAR_STIFFNESS = 1000.0
_AUTO_RESET_SECONDS = 1.2
_ANCHORS = {
    "spherical": np.array([-2.2, 0.0, 1.2]),
    "revolute": np.array([0.0, 0.0, 1.2]),
    "prismatic": np.array([2.2, 0.0, 1.2]),
}
_INITIAL_VELOCITIES = {
    "spherical": np.array([0.8, 0.0, 0.0, 0.0, 0.0, 1.2]),
    "revolute": np.array([0.5, 0.3, 0.0, 0.5, 0.3, 1.0]),
    "prismatic": np.array([1.0, 0.5, 0.0, 0.0, 0.4, 0.6]),
}


def _translation(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = position
    return transform


def _rotation_angle(rotation: np.ndarray) -> float:
    cosine = 0.5 * (float(np.trace(rotation)) - 1.0)
    return float(np.arccos(np.clip(cosine, -1.0, 1.0)))


def _configure_finite_joint(joint: sx.Joint) -> sx.Joint:
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

    robot = world.add_multibody("avbd_articulated_compliant_joints")
    base = robot.add_link("avbd_articulated_compliant_base")
    bodies: dict[str, sx.Link] = {}
    joints: dict[str, sx.Joint] = {}
    types = {
        "spherical": sx.JointType.SPHERICAL,
        "revolute": sx.JointType.REVOLUTE,
        "prismatic": sx.JointType.PRISMATIC,
    }
    axes = {
        "spherical": (0.0, 0.0, 1.0),
        "revolute": (0.0, 0.0, 1.0),
        "prismatic": (1.0, 0.0, 0.0),
    }

    for name in ("spherical", "revolute", "prismatic"):
        body = robot.add_link(
            f"avbd_articulated_compliant_{name}_body",
            parent=base,
            joint=sx.JointSpec(
                name=f"avbd_articulated_compliant_{name}_floating",
                type=sx.JointType.FLOATING,
            ),
        )
        body.mass = 2.0
        body.inertia = ((0.2, 0.0, 0.0), (0.0, 0.25, 0.0), (0.0, 0.0, 0.3))
        body.parent_joint.position = [*_ANCHORS[name], 0.0, 0.0, 0.0]
        body.parent_joint.velocity = _INITIAL_VELOCITIES[name].tolist()

        joint = world.add_joint(
            body,
            sx.JointSpec(
                name=f"avbd_articulated_compliant_{name}_joint",
                type=types[name],
                axis=axes[name],
                parent_anchor=_ANCHORS[name].tolist(),
                child_anchor=(0.0, 0.0, 0.0),
            ),
        )
        joint.actuator_type = sx.ActuatorType.PASSIVE
        bodies[name] = body
        joints[name] = _configure_finite_joint(joint)

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(
        world, name="avbd_articulated_compliant_joints_render"
    )
    body_shapes = {
        "spherical": dart.BoxShape(np.array([0.72, 0.30, 0.22])),
        "revolute": dart.BoxShape(np.array([0.90, 0.20, 0.20])),
        "prismatic": dart.BoxShape(np.array([0.54, 0.34, 0.24])),
    }
    colors = {
        "spherical": (0.25, 0.66, 0.92),
        "revolute": (0.95, 0.61, 0.16),
        "prismatic": (0.34, 0.73, 0.39),
    }
    for name, body in bodies.items():
        bridge.add_link_visual(
            body,
            body_shapes[name],
            colors[name],
            name=f"avbd_articulated_compliant_{name}_visual",
        )

    for name, anchor in _ANCHORS.items():
        marker = dart.SimpleFrame(
            dart.gui.world_render_frame(),
            f"avbd_articulated_compliant_{name}_anchor",
            _translation(anchor),
        )
        marker.set_shape(dart.SphereShape(0.075))
        marker.create_visual_aspect().set_color([0.82, 0.82, 0.78])
        bridge.render_world.add_simple_frame(marker)

    hinge_axis = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        "avbd_articulated_compliant_hinge_axis",
        _translation(_ANCHORS["revolute"]),
    )
    hinge_axis.set_shape(dart.CylinderShape(0.025, 1.0))
    hinge_axis.create_visual_aspect().set_color([0.92, 0.38, 0.18])
    bridge.render_world.add_simple_frame(hinge_axis)

    slider_rail = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        "avbd_articulated_compliant_slider_rail",
        _translation(_ANCHORS["prismatic"]),
    )
    slider_rail.set_shape(dart.BoxShape(np.array([2.4, 0.035, 0.035])))
    slider_rail.create_visual_aspect().set_color([0.72, 0.76, 0.82])
    bridge.render_world.add_simple_frame(slider_rail)

    initial_rotations = {
        name: np.asarray(body.rotation, dtype=float).reshape(3, 3).copy()
        for name, body in bodies.items()
    }
    replay_state = {"enabled": True, "last_reset": 0.0, "cycles": 0}

    def reset_joints() -> None:
        for name, body in bodies.items():
            body.parent_joint.position = [*_ANCHORS[name], 0.0, 0.0, 0.0]
            body.parent_joint.velocity = _INITIAL_VELOCITIES[name].tolist()
        replay_state["last_reset"] = float(world.time)
        replay_state["cycles"] = int(replay_state["cycles"]) + 1
        bridge.sync()

    def metrics() -> dict[str, float]:
        spherical_rotation = np.asarray(
            bodies["spherical"].rotation, dtype=float
        ).reshape(3, 3)
        revolute_rotation = np.asarray(
            bodies["revolute"].rotation, dtype=float
        ).reshape(3, 3)
        prismatic_rotation = np.asarray(
            bodies["prismatic"].rotation, dtype=float
        ).reshape(3, 3)
        socket_displacement = (
            np.asarray(bodies["spherical"].translation, dtype=float).reshape(3)
            - _ANCHORS["spherical"]
        )
        hinge_displacement = (
            np.asarray(bodies["revolute"].translation, dtype=float).reshape(3)
            - _ANCHORS["revolute"]
        )
        slider_displacement = (
            np.asarray(bodies["prismatic"].translation, dtype=float).reshape(3)
            - _ANCHORS["prismatic"]
        )
        return {
            "socket_anchor_error": float(np.linalg.norm(socket_displacement)),
            "socket_free_rotation": _rotation_angle(
                spherical_rotation @ initial_rotations["spherical"].T
            ),
            "hinge_anchor_error": float(np.linalg.norm(hinge_displacement)),
            "hinge_axis_tilt": float(
                np.linalg.norm(
                    revolute_rotation @ np.array([0.0, 0.0, 1.0])
                    - np.array([0.0, 0.0, 1.0])
                )
            ),
            "hinge_free_rotation": float(
                abs(np.arctan2(revolute_rotation[1, 0], revolute_rotation[0, 0]))
            ),
            "slider_axis_travel": float(slider_displacement[0]),
            "slider_transverse_error": float(
                np.linalg.norm(slider_displacement[1:])
            ),
            "slider_rotation_error": _rotation_angle(
                prismatic_rotation @ initial_rotations["prismatic"].T
            ),
        }

    def replay_capture_state() -> dict[str, object]:
        return dict(replay_state)

    def replay_restore_state(state: dict[str, object]) -> None:
        replay_state["enabled"] = bool(state.get("enabled", True))
        replay_state["last_reset"] = float(state.get("last_reset", 0.0))
        replay_state["cycles"] = int(state.get("cycles", 0))
        bridge.sync()

    def pre_step() -> None:
        if (
            bool(replay_state["enabled"])
            and world.time - float(replay_state["last_reset"])
            >= _AUTO_RESET_SECONDS
        ):
            reset_joints()
        bridge.pre_step()

    bridge.sync()

    socket_history: deque[float] = deque(maxlen=160)
    hinge_history: deque[float] = deque(maxlen=160)
    slider_history: deque[float] = deque(maxlen=160)

    def build_panel(builder: object, context: object) -> None:
        values = metrics()
        socket_history.append(values["socket_free_rotation"])
        hinge_history.append(values["hinge_free_rotation"])
        slider_history.append(values["slider_axis_travel"])

        builder.text("solver: AVBD articulated finite-stiffness joint rows")
        builder.text(
            "rows: spherical translation; revolute translation/tilt; "
            "prismatic transverse/rotation"
        )
        builder.text(f"start stiffness: {_START_STIFFNESS:.1f}")
        builder.text(f"linear cap: {_LINEAR_STIFFNESS:.1f}")
        builder.text(f"angular cap: {_ANGULAR_STIFFNESS:.1f}")
        builder.text(f"socket anchor error: {values['socket_anchor_error']:.4f} m")
        builder.text(
            f"socket free rotation: {values['socket_free_rotation']:.3f} rad"
        )
        builder.text(f"hinge anchor error: {values['hinge_anchor_error']:.4f} m")
        builder.text(f"hinge axis tilt: {values['hinge_axis_tilt']:.4f}")
        builder.text(
            f"hinge free rotation: {values['hinge_free_rotation']:.3f} rad"
        )
        builder.text(f"slider travel: {values['slider_axis_travel']:.3f} m")
        builder.text(
            f"slider transverse error: {values['slider_transverse_error']:.4f} m"
        )
        builder.text(
            f"slider rotation error: {values['slider_rotation_error']:.4f} rad"
        )
        builder.text(f"replay cycles: {int(replay_state['cycles'])}")
        if builder.button("Reset joint trio"):
            reset_joints()
        changed, enabled = builder.checkbox(
            "Loop replay",
            bool(replay_state["enabled"]),
        )
        if changed:
            replay_state["enabled"] = bool(enabled)
            replay_state["last_reset"] = float(world.time)
        builder.plot_lines("Socket free rotation", list(socket_history))
        builder.plot_lines("Hinge free rotation", list(hinge_history))
        builder.plot_lines("Slider axis travel", list(slider_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Articulated Compliant Joints", build_panel)],
        info={
            "sx_world": world,
            "base": base,
            "bodies": bodies,
            "joints": joints,
            "anchors": {name: anchor.copy() for name, anchor in _ANCHORS.items()},
            "start_stiffness": _START_STIFFNESS,
            "linear_stiffness": _LINEAR_STIFFNESS,
            "angular_stiffness": _ANGULAR_STIFFNESS,
            "auto_reset_seconds": _AUTO_RESET_SECONDS,
            "metrics": metrics,
            "reset_joints": reset_joints,
            "replay_state": replay_state,
            "replay_capture_state": replay_capture_state,
            "replay_restore_state": replay_restore_state,
        },
    )


SCENE = PythonDemoScene(
    id="avbd_articulated_compliant_joints",
    title="AVBD Articulated Compliant Joints (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="Finite-stiffness spherical, revolute, and prismatic articulated "
    "rows resist only their constrained coordinates.",
    build=build,
)
