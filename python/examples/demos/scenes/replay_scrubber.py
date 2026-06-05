"""Simulation replay scrubber for the experimental World.

The scene records a World rollout once, restores frame zero, then lets the
user scrub the already-simulated states with a timestep-resolution slider.
Scrubbing calls ``World.restore_replay_frame``; it does not re-run physics.
"""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_REPLAY_STEPS = 180
_TIME_STEP = 0.01


def build() -> SceneSetup:
    world = sx.World(time_step=_TIME_STEP)
    world.gravity = np.array([0.0, 0.0, -9.81])

    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -0.08))
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box((3.0, 1.2, 0.08)))
    ground.friction = 0.7

    ball = world.add_rigid_body(
        "ball",
        position=(-1.0, 0.0, 1.15),
        linear_velocity=(1.4, 0.0, 0.0),
    )
    ball.set_collision_shape(sx.CollisionShape.sphere(0.12))
    ball.restitution = 0.55
    ball.friction = 0.25

    world.replay_recording_enabled = True
    world.step(_REPLAY_STEPS)
    world.restore_replay_frame(0)

    bridge = WorldRenderBridge(world, name="replay_scrubber_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(np.array([6.0, 2.4, 0.16])),
        (0.55, 0.55, 0.58),
        name="replay_ground",
    )
    bridge.add_rigid_body_visual(
        ball,
        dart.SphereShape(0.12),
        (0.20, 0.58, 0.92),
        name="replay_ball",
    )
    bridge.sync()

    playback = {
        "frame": 0,
        "playing": False,
    }
    height_history: deque[float] = deque(maxlen=120)

    def restore_frame(index: int) -> None:
        count = int(world.replay_frame_count)
        if count <= 0:
            return
        clamped = max(0, min(index, count - 1))
        world.restore_replay_frame(clamped)
        playback["frame"] = clamped
        bridge.sync()

    def pre_step() -> None:
        if not playback["playing"]:
            return
        next_frame = int(playback["frame"]) + 1
        if next_frame >= int(world.replay_frame_count):
            playback["playing"] = False
            return
        restore_frame(next_frame)

    def build_panel(builder: object, context: object) -> None:
        frame_count = int(world.replay_frame_count)
        current = int(playback["frame"])
        height = float(np.asarray(ball.translation, dtype=float).reshape(3)[2])
        height_history.append(height)

        builder.text("recording: on")
        builder.text(f"frames: {frame_count}")
        builder.text(f"time step: {world.time_step:.3f} s")
        builder.text(f"time: {world.time:.3f} s")
        builder.text(f"simulation frame: {world.frame}")
        if height_history:
            builder.plot_lines("Ball height", list(height_history))
        builder.separator()

        max_frame = max(0, frame_count - 1)
        changed, selected = builder.slider(
            "Replay frame", float(current), 0.0, float(max_frame)
        )
        if changed:
            restore_frame(int(round(selected)))

        if builder.button("|<"):
            playback["playing"] = False
            restore_frame(0)
        builder.same_line()
        if builder.button("<"):
            playback["playing"] = False
            restore_frame(current - 1)
        builder.same_line()
        if builder.button("Pause" if playback["playing"] else "Play"):
            playback["playing"] = not bool(playback["playing"])
        builder.same_line()
        if builder.button(">"):
            playback["playing"] = False
            restore_frame(current + 1)
        builder.same_line()
        if builder.button(">|"):
            playback["playing"] = False
            restore_frame(max_frame)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        panels=[ScenePanel("Replay Scrubber", build_panel)],
        info={"sx_world": world, "recorded_frames": world.replay_frame_count},
    )


SCENE = PythonDemoScene(
    id="replay_scrubber",
    title="Replay Scrubber",
    category="Simulation Replay",
    summary="Scrub an already-recorded experimental World at timestep resolution.",
    build=build,
)
