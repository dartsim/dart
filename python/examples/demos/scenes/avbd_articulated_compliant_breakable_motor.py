"""Finite-stiffness AVBD articulated break/load/reset lifecycle showcase."""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 0.005
_START_STIFFNESS = 100.0
_LINEAR_STIFFNESS = 100.0
_ANGULAR_STIFFNESS = 100.0
_TARGET_SPEED = 0.8
_MAX_EFFORT = 6.0
_WEAK_BREAK_FORCE = 9.0
_STRONG_BREAK_FORCE = 100.0
_PRESTRAIN = 0.08
_LATERAL_FORCE = 200.0
_PHASE_SECONDS = 0.25
_CYCLE_SECONDS = 3.0 * _PHASE_SECONDS
_CENTER = np.array([0.0, 0.0, 1.0])
_PARENT_ANCHOR = np.array([0.45, 0.0, 0.0])
_CHILD_ANCHOR = np.array([-0.45, 0.0, 0.0])
_AXIS = np.array([1.0, 0.0, 0.0])
_PHASE_LABELS = (
    "weak: break",
    "strong reset: intact",
    "weak re-arm: break",
)


def _transform(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = position
    return transform


def _normalized_or(vector: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1.0e-12:
        return fallback
    return vector / norm


def _connector_transform(start: np.ndarray, end: np.ndarray) -> np.ndarray:
    x_axis = _normalized_or(end - start, np.array([1.0, 0.0, 0.0]))
    reference = np.array([0.0, 0.0, 1.0])
    if abs(float(reference @ x_axis)) > 0.95:
        reference = np.array([0.0, 1.0, 0.0])
    y_axis = _normalized_or(np.cross(reference, x_axis), np.array([0.0, 1.0, 0.0]))
    z_axis = np.cross(x_axis, y_axis)
    transform = _transform(0.5 * (start + end))
    transform[:3, 0] = x_axis
    transform[:3, 1] = y_axis
    transform[:3, 2] = z_axis
    return transform


def _link_rotation(link: sx.Link) -> np.ndarray:
    return np.asarray(link.rotation, dtype=float).reshape(3, 3)


def _link_translation(link: sx.Link) -> np.ndarray:
    return np.asarray(link.translation, dtype=float).reshape(3)


def _world_point(link: sx.Link, local_point: np.ndarray) -> np.ndarray:
    return _link_rotation(link) @ local_point + _link_translation(link)


def build() -> SceneSetup:
    world = sx.World(time_step=_TIME_STEP, gravity=(0.0, 0.0, 0.0))
    world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )

    robot = world.add_multibody("avbd_articulated_compliant_breakable_motor")
    base = robot.add_link("avbd_articulated_compliant_breakable_base")
    parent_position = _CENTER - _PARENT_ANCHOR
    child_position = _CENTER - _CHILD_ANCHOR

    def add_floating_link(name: str, position: np.ndarray, mass: float) -> sx.Link:
        link = robot.add_link(
            name,
            parent=base,
            joint=sx.JointSpec(
                name=f"{name}_floating",
                type=sx.JointType.FLOATING,
            ),
        )
        link.mass = mass
        link.inertia = ((0.16, 0.0, 0.0), (0.0, 0.22, 0.0), (0.0, 0.0, 0.28))
        link.parent_joint.position = [*position, 0.0, 0.0, 0.0]
        return link

    parent = add_floating_link(
        "avbd_articulated_compliant_breakable_parent",
        parent_position,
        100.0,
    )
    child = add_floating_link(
        "avbd_articulated_compliant_breakable_child",
        child_position,
        120.0,
    )

    joint = world.add_joint(
        parent,
        child,
        sx.JointSpec(
            name="avbd_articulated_compliant_breakable_slider",
            type=sx.JointType.PRISMATIC,
            axis=_AXIS.tolist(),
            parent_anchor=_PARENT_ANCHOR.tolist(),
            child_anchor=_CHILD_ANCHOR.tolist(),
        ),
    )
    joint.actuator_type = sx.ActuatorType.VELOCITY
    joint.command_velocity = [_TARGET_SPEED]
    joint.set_effort_limits([-_MAX_EFFORT], [_MAX_EFFORT])
    policy = joint.constraint_projection_policy
    policy.start_stiffness = _START_STIFFNESS
    policy.linear_stiffness = _LINEAR_STIFFNESS
    policy.angular_stiffness = _ANGULAR_STIFFNESS
    joint.constraint_projection_policy = policy
    joint.break_force = _WEAK_BREAK_FORCE

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(
        world, name="avbd_articulated_compliant_breakable_motor_render"
    )
    bridge.add_link_visual(
        parent,
        dart.BoxShape(np.array([0.82, 0.20, 0.20])),
        (0.22, 0.52, 0.84),
        name="avbd_articulated_compliant_breakable_parent_visual",
    )
    bridge.add_link_visual(
        child,
        dart.BoxShape(np.array([0.82, 0.20, 0.20])),
        (0.95, 0.57, 0.16),
        name="avbd_articulated_compliant_breakable_child_visual",
    )

    rail = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        "avbd_articulated_compliant_breakable_rail",
        _transform(_CENTER),
    )
    rail.set_shape(dart.BoxShape(np.array([2.6, 0.035, 0.035])))
    rail_visual = rail.create_visual_aspect()
    rail_visual.set_color([0.74, 0.78, 0.84])
    bridge.render_world.add_simple_frame(rail)

    anchor_marker = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        "avbd_articulated_compliant_breakable_anchor",
        _transform(_CENTER),
    )
    anchor_marker.set_shape(dart.SphereShape(0.075))
    anchor_marker.create_visual_aspect().set_color([0.86, 0.86, 0.80])
    bridge.render_world.add_simple_frame(anchor_marker)

    connector = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        "avbd_articulated_compliant_breakable_connector",
        _connector_transform(_CENTER, _CENTER),
    )
    connector.set_shape(dart.BoxShape(np.array([0.02, 0.055, 0.055])))
    connector_visual = connector.create_visual_aspect()
    connector_visual.set_color([0.35, 0.82, 0.45])
    bridge.render_world.add_simple_frame(connector)

    replay_state: dict[str, object] = {
        "enabled": True,
        "cycle_start": 0.0,
        "phase_index": 0,
        "cycles": 0,
        "transitions": 0,
    }

    def reset_loaded_pose(break_force: float, phase_index: int) -> None:
        parent.parent_joint.position = [*parent_position, 0.0, 0.0, 0.0]
        child.parent_joint.position = [
            *(child_position + _PRESTRAIN * np.array([0.0, 1.0, 0.0])),
            0.0,
            0.0,
            0.0,
        ]
        parent.parent_joint.velocity = [0.0] * 6
        child.parent_joint.velocity = [0.0] * 6
        joint.command_velocity = [_TARGET_SPEED]
        joint.break_force = float(break_force)
        joint.reset_breakage()
        replay_state["phase_index"] = int(phase_index)
        replay_state["transitions"] = int(replay_state["transitions"]) + 1
        world.update_kinematics()
        bridge.sync()

    def reset_strong_joint() -> None:
        reset_loaded_pose(_STRONG_BREAK_FORCE, 1)

    def rearm_weak_joint() -> None:
        reset_loaded_pose(_WEAK_BREAK_FORCE, 2)

    def start_weak_cycle() -> None:
        reset_loaded_pose(_WEAK_BREAK_FORCE, 0)

    def anchor_delta() -> np.ndarray:
        return _world_point(child, _CHILD_ANCHOR) - _world_point(
            parent, _PARENT_ANCHOR
        )

    def sample_metrics() -> dict[str, float | str]:
        delta = anchor_delta()
        axis_travel = float(delta @ _AXIS)
        transverse = delta - axis_travel * _AXIS
        phase_index = int(replay_state["phase_index"])
        return {
            "axis_travel": axis_travel,
            "transverse_residual": float(np.linalg.norm(transverse)),
            "broken": 1.0 if joint.is_broken else 0.0,
            "break_force": float(joint.break_force),
            "world_time": float(world.time),
            "phase": _PHASE_LABELS[phase_index],
            "cycles": float(replay_state["cycles"]),
            "transitions": float(replay_state["transitions"]),
        }

    transverse_history: deque[float] = deque(maxlen=180)
    travel_history: deque[float] = deque(maxlen=180)
    broken_history: deque[float] = deque(maxlen=180)
    _last_metrics: dict[str, float | str] = {}

    def sync_visuals() -> None:
        start = _world_point(parent, _PARENT_ANCHOR)
        end = _world_point(child, _CHILD_ANCHOR)
        distance = max(float(np.linalg.norm(end - start)), 0.02)
        connector.set_transform(_connector_transform(start, end))
        connector.set_shape(dart.BoxShape(np.array([distance, 0.055, 0.055])))
        if joint.is_broken:
            connector_visual.set_color([0.95, 0.18, 0.13])
        elif int(replay_state["phase_index"]) == 1:
            connector_visual.set_color([0.25, 0.86, 0.42])
        else:
            connector_visual.set_color([0.94, 0.72, 0.18])

    def record_metrics() -> dict[str, float | str]:
        _last_metrics.clear()
        _last_metrics.update(sample_metrics())
        transverse_history.append(float(_last_metrics["transverse_residual"]))
        travel_history.append(float(_last_metrics["axis_travel"]))
        broken_history.append(float(_last_metrics["broken"]))
        return _last_metrics

    def capture_metrics() -> dict[str, object]:
        if not _last_metrics:
            record_metrics()
        broken_values = list(broken_history)
        transverse_values = list(transverse_history)
        return {
            "row": "avbd_articulated_compliant_breakable_motor",
            "solver": "avbd_variational_articulated",
            "executor": "World.step default",
            "constraint": "finite_and_motor_physical_load_fracture_lifecycle",
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(world.time),
            "joint_name": str(joint.name),
            "phase": str(_last_metrics["phase"]),
            "status": "broken" if joint.is_broken else "intact",
            "break_force": float(joint.break_force),
            "weak_break_force": _WEAK_BREAK_FORCE,
            "strong_break_force": _STRONG_BREAK_FORCE,
            "start_stiffness": _START_STIFFNESS,
            "linear_stiffness": _LINEAR_STIFFNESS,
            "angular_stiffness": _ANGULAR_STIFFNESS,
            "max_effort": _MAX_EFFORT,
            "prestrain": _PRESTRAIN,
            "axis_travel": float(_last_metrics["axis_travel"]),
            "transverse_residual": float(_last_metrics["transverse_residual"]),
            "broken": float(_last_metrics["broken"]),
            "history": {
                "samples": float(len(broken_values)),
                "saw_broken": max(broken_values, default=0.0),
                "saw_intact": 1.0 - min(broken_values, default=0.0),
                "max_transverse_residual": max(transverse_values, default=0.0),
                "cycles": float(replay_state["cycles"]),
                "transitions": float(replay_state["transitions"]),
            },
        }

    def replay_capture_state() -> dict[str, object]:
        return dict(replay_state)

    def replay_restore_state(state: dict[str, object]) -> None:
        replay_state.update(state)
        joint.command_velocity = [_TARGET_SPEED]
        bridge.sync()
        record_metrics()
        sync_visuals()

    def pre_step() -> None:
        if bool(replay_state["enabled"]):
            elapsed = float(world.time) - float(replay_state["cycle_start"])
            if elapsed >= _CYCLE_SECONDS:
                replay_state["cycle_start"] = float(world.time)
                replay_state["cycles"] = int(replay_state["cycles"]) + 1
                start_weak_cycle()
                elapsed = 0.0
            phase_index = min(int(elapsed / _PHASE_SECONDS), 2)
            if phase_index != int(replay_state["phase_index"]):
                if phase_index == 1:
                    reset_strong_joint()
                elif phase_index == 2:
                    rearm_weak_joint()

        parent.apply_force((0.0, -_LATERAL_FORCE, 0.0))
        child.apply_force((0.0, _LATERAL_FORCE, 0.0))
        joint.command_velocity = [_TARGET_SPEED]
        bridge.pre_step()
        record_metrics()
        sync_visuals()

    start_weak_cycle()
    record_metrics()
    sync_visuals()

    def build_panel(builder: object, context: object) -> None:
        metrics = _last_metrics or record_metrics()
        builder.text("solver: AVBD finite rows + bounded motor load")
        builder.text(f"phase: {metrics['phase']}")
        builder.text(f"state: {'broken' if joint.is_broken else 'intact'}")
        builder.text(f"break threshold: {joint.break_force:.1f} N")
        builder.text(f"finite stiffness: {_LINEAR_STIFFNESS:.1f} N/m")
        builder.text(f"motor effort cap: {_MAX_EFFORT:.1f} N")
        builder.text(f"prestrain: {_PRESTRAIN:.3f} m")
        builder.text(
            "transverse residual: "
            f"{float(metrics['transverse_residual']):.4f} m"
        )
        builder.text(f"axis travel: {float(metrics['axis_travel']):.4f} m")
        builder.text(f"replay cycles: {int(replay_state['cycles'])}")
        if builder.button("Reset strong"):
            reset_strong_joint()
        if builder.button("Re-arm weak"):
            rearm_weak_joint()
        changed, enabled = builder.checkbox(
            "Loop fracture lifecycle",
            bool(replay_state["enabled"]),
        )
        if changed:
            replay_state["enabled"] = bool(enabled)
            replay_state["cycle_start"] = float(world.time)
        builder.plot_lines("Transverse residual", list(transverse_history))
        builder.plot_lines("Slider travel", list(travel_history))
        builder.plot_lines("Broken", list(broken_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[
            ScenePanel(
                "AVBD Articulated Compliant Breakable Motor",
                build_panel,
            )
        ],
        info={
            "sx_world": world,
            "base": base,
            "parent": parent,
            "child": child,
            "joint": joint,
            "axis": _AXIS.copy(),
            "metrics": sample_metrics,
            "replay_state": replay_state,
            "replay_capture_state": replay_capture_state,
            "replay_restore_state": replay_restore_state,
            "reset_loaded_pose": reset_loaded_pose,
            "reset_strong_joint": reset_strong_joint,
            "rearm_weak_joint": rearm_weak_joint,
            "start_weak_cycle": start_weak_cycle,
            "time_step": _TIME_STEP,
            "start_stiffness": _START_STIFFNESS,
            "linear_stiffness": _LINEAR_STIFFNESS,
            "angular_stiffness": _ANGULAR_STIFFNESS,
            "target_speed": _TARGET_SPEED,
            "max_effort": _MAX_EFFORT,
            "weak_break_force": _WEAK_BREAK_FORCE,
            "strong_break_force": _STRONG_BREAK_FORCE,
            "prestrain": _PRESTRAIN,
            "phase_seconds": _PHASE_SECONDS,
            CAPTURE_METRICS_INFO_KEY: capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="avbd_articulated_compliant_breakable_motor",
    title="AVBD Articulated Compliant Breakable Motor (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A finite same-multibody prismatic motor breaks from its combined "
    "solver-row load metric, resets strong, and fractures again when re-armed.",
    build=build,
)
