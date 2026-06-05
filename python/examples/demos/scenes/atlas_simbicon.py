"""Atlas SIMBICON preview on the experimental World.

This scene stages the old `atlas_simbicon` flagship as a World-native controller
preview: Atlas is loaded through `sx.add_skeleton`, then a four-state SIMBICON
target cycle drives the named leg joints directly through World joint state.
The full dynamically robust walking controller remains deferred until the
documented lateral-balance gap is closed.
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup
from ._robot_model_world import (
    add_ground_visual,
    add_robot_link_visuals,
    apply_joint_targets,
    load_atlas_world,
)

_DEG = math.pi / 180.0

_BASE_POSE = {
    "back_bky": 4.75 * _DEG,
    "l_leg_hpy": -0.18,
    "r_leg_hpy": -0.18,
    "l_leg_kny": 0.30,
    "r_leg_kny": 0.30,
    "l_leg_aky": -0.16,
    "r_leg_aky": -0.16,
    "l_arm_shx": -0.55,
    "r_arm_shx": 0.55,
    "l_arm_ely": 1.05,
    "r_arm_ely": 1.05,
}

_STATE_DURATIONS = (0.30, 0.55, 0.30, 0.55)
_STATE_NAMES = (
    "right swing up",
    "right swing down",
    "left swing up",
    "left swing down",
)


def _state_targets(state: int) -> dict[str, float]:
    targets = dict(_BASE_POSE)
    right_swing = state < 2
    swing_prefix = "r" if right_swing else "l"
    stance_prefix = "l" if right_swing else "r"
    if state in (0, 2):
        targets[f"{swing_prefix}_leg_hpy"] = -0.50
        targets[f"{swing_prefix}_leg_kny"] = 1.10
        targets[f"{swing_prefix}_leg_aky"] = -0.60
        targets[f"{stance_prefix}_leg_kny"] = 0.05
    else:
        targets[f"{swing_prefix}_leg_hpy"] = 0.10
        targets[f"{swing_prefix}_leg_kny"] = 0.05
        targets[f"{swing_prefix}_leg_aky"] = -0.15
        targets[f"{stance_prefix}_leg_kny"] = 0.10
    return targets


class _AtlasSimbiconPreview:
    def __init__(self, robot: object) -> None:
        self.robot = robot
        self.state = 0
        self.state_time = 0.0
        self.speed = 1.0
        self._previous_targets = dict(_BASE_POSE)
        self._current_targets = _state_targets(0)

    @property
    def state_name(self) -> str:
        return _STATE_NAMES[self.state]

    @property
    def phase(self) -> float:
        return min(1.0, self.state_time / _STATE_DURATIONS[self.state])

    def reset(self) -> None:
        self.state = 0
        self.state_time = 0.0
        self._previous_targets = dict(_BASE_POSE)
        self._current_targets = _state_targets(0)
        apply_joint_targets(self.robot, self._current_targets)

    def step(self, dt: float) -> None:
        self.state_time += max(0.0, dt * self.speed)
        duration = _STATE_DURATIONS[self.state]
        if self.state_time >= duration:
            self.state = (self.state + 1) % len(_STATE_DURATIONS)
            self.state_time = 0.0
            self._previous_targets = self._current_targets
            self._current_targets = _state_targets(self.state)

        phase = 0.5 - 0.5 * math.cos(math.pi * self.phase)
        names = set(self._previous_targets) | set(self._current_targets)
        blended = {
            name: (1.0 - phase) * self._previous_targets.get(name, 0.0)
            + phase * self._current_targets.get(name, 0.0)
            for name in names
        }
        apply_joint_targets(self.robot, blended)


def build() -> SceneSetup:
    world, robot = load_atlas_world(_BASE_POSE)
    bridge = WorldRenderBridge(world, name="atlas_simbicon_render")
    add_robot_link_visuals(bridge, robot, prefix="atlas_simbicon")
    add_ground_visual(bridge, name="atlas_simbicon_ground_visual")
    bridge.sync()

    controller = _AtlasSimbiconPreview(robot)
    controller.reset()
    pelvis = robot.get_link("pelvis")
    right_foot = robot.get_link("r_foot")
    left_foot = robot.get_link("l_foot")
    pelvis_history: deque[float] = deque(maxlen=120)
    swing_history: deque[float] = deque(maxlen=120)

    def pre_step() -> None:
        controller.step(world.time_step)
        world.update_kinematics()
        bridge.pre_step()

    def build_panel(builder: object, context: object) -> None:
        if pelvis is not None:
            pelvis_history.append(float(np.asarray(pelvis.translation)[2]))
        swing = right_foot if controller.state < 2 else left_foot
        if swing is not None:
            swing_history.append(float(np.asarray(swing.translation)[2]))

        builder.text("controller: SIMBICON target preview")
        builder.text(f"state: {controller.state_name}")
        builder.text(f"phase: {controller.phase:.2f}")
        changed, speed = builder.slider(
            "Playback speed", float(controller.speed), 0.0, 2.0
        )
        if changed:
            controller.speed = float(speed)
        if builder.button("Reset gait"):
            controller.reset()
            world.update_kinematics()
            bridge.sync()
        builder.plot_lines("Pelvis height", list(pelvis_history))
        builder.plot_lines("Swing foot height", list(swing_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("Atlas SIMBICON", build_panel)],
        info={
            "sx_world": world,
            "robot": "atlas",
            "controller": "simbicon_target_preview",
            "dofs": robot.num_dofs,
        },
    )


SCENE = PythonDemoScene(
    id="atlas_simbicon",
    title="Atlas SIMBICON",
    category="Control & IK",
    summary="Atlas cycling SIMBICON gait targets on the experimental World.",
    build=build,
)
