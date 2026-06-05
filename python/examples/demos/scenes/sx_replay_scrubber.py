"""Simulation replay scrubber for the experimental World.

The scene records an sx::World rollout once, restores frame zero, then lets the
user scrub the already-simulated states with a timestep-resolution slider.
Scrubbing calls ``World.restore_replay_frame``; it does not re-run physics.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import dartpy as dart
import dartpy.simulation_experimental as sx

from .._sx_bridge import SxRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_REPLAY_STEPS = 180
_TIME_STEP = 0.01
_PLAYBACK_RATE_LABELS = (
    "1 frame/tick",
    "2 frames/tick",
    "4 frames/tick",
    "8 frames/tick",
)
_PLAYBACK_RATE_STEPS = (1, 2, 4, 8)


@dataclass(frozen=True)
class _ReplayFrameSample:
    index: int
    time: float
    simulation_frame: int
    ball_x: float
    ball_height: float


def _body_translation(body: object) -> np.ndarray:
    return np.asarray(body.translation, dtype=float).reshape(3)


def _collect_replay_samples(
    world: sx.World, ball: object
) -> list[_ReplayFrameSample]:
    samples: list[_ReplayFrameSample] = []
    for index in range(int(world.replay_frame_count)):
        world.restore_replay_frame(index)
        translation = _body_translation(ball)
        samples.append(
            _ReplayFrameSample(
                index=index,
                time=float(world.get_replay_frame_time(index)),
                simulation_frame=int(world.get_replay_simulation_frame(index)),
                ball_x=float(translation[0]),
                ball_height=float(translation[2]),
            )
        )
    return samples


def _sample_at(
    samples: list[_ReplayFrameSample], frame: int
) -> _ReplayFrameSample | None:
    if not samples:
        return None
    return samples[max(0, min(frame, len(samples) - 1))]


def _frame_mark_series(frame_count: int) -> list[float]:
    if frame_count <= 0:
        return []
    interval = max(1, frame_count // 12)
    return [
        1.0 if index in {0, frame_count - 1} or index % interval == 0 else 0.0
        for index in range(frame_count)
    ]


def _cursor_series(frame_count: int, frame: int) -> list[float]:
    if frame_count <= 0:
        return []
    clamped = max(0, min(frame, frame_count - 1))
    values = [0.0] * frame_count
    values[clamped] = 1.0
    return values


def _replay_duration(samples: list[_ReplayFrameSample]) -> float:
    if len(samples) < 2:
        return 0.0
    return max(0.0, samples[-1].time - samples[0].time)


def _add_table_row(builder: object, label: str, value: str) -> None:
    builder.table_next_row()
    if builder.table_next_column():
        builder.text(label)
    if builder.table_next_column():
        builder.text(value)


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
    samples = _collect_replay_samples(world, ball)
    height_values = [sample.ball_height for sample in samples]
    frame_marks = _frame_mark_series(len(samples))
    duration = _replay_duration(samples)
    world.restore_replay_frame(0)

    bridge = SxRenderBridge(world, name="sx_replay_scrubber_render")
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
        "loop": False,
        "rate_index": 0,
    }

    def restore_frame(index: int) -> None:
        count = len(samples)
        if count <= 0:
            return
        clamped = max(0, min(index, count - 1))
        world.restore_replay_frame(clamped)
        playback["frame"] = clamped
        bridge.sync()

    def pre_step() -> None:
        if not playback["playing"]:
            return
        frame_count = len(samples)
        if frame_count <= 0:
            playback["playing"] = False
            return
        rate_index = max(
            0, min(int(playback["rate_index"]), len(_PLAYBACK_RATE_STEPS) - 1)
        )
        next_frame = int(playback["frame"]) + _PLAYBACK_RATE_STEPS[rate_index]
        if next_frame >= frame_count:
            if playback["loop"]:
                next_frame = next_frame % frame_count
            else:
                next_frame = frame_count - 1
                playback["playing"] = False
        restore_frame(next_frame)

    def build_panel(builder: object, context: object) -> None:
        frame_count = len(samples)
        current = int(playback["frame"])
        sample = _sample_at(samples, current)
        status = "playing" if playback["playing"] else "paused"
        frame_time = sample.time if sample is not None else float(world.time)

        builder.text(
            f"frame {current + 1} / {frame_count}  "
            f"time {frame_time:.3f} s  {status}  "
            f"range 0.000 s - {duration:.3f} s  "
            f"step {world.time_step:.3f} s"
        )
        builder.separator()

        max_frame = max(0, frame_count - 1)
        if builder.button("<<"):
            playback["playing"] = False
            restore_frame(0)
        builder.same_line()
        if builder.button("-10"):
            playback["playing"] = False
            restore_frame(current - 10)
        builder.same_line()
        if builder.button("-1"):
            playback["playing"] = False
            restore_frame(current - 1)
        builder.same_line()
        if builder.button("Pause" if playback["playing"] else "Play"):
            playback["playing"] = not bool(playback["playing"])
        builder.same_line()
        if builder.button("+1"):
            playback["playing"] = False
            restore_frame(current + 1)
        builder.same_line()
        if builder.button("+10"):
            playback["playing"] = False
            restore_frame(current + 10)
        builder.same_line()
        if builder.button(">>"):
            playback["playing"] = False
            restore_frame(max_frame)

        builder.same_line()
        changed, loop = builder.checkbox("Loop playback", bool(playback["loop"]))
        if changed:
            playback["loop"] = loop
        builder.same_line()
        changed, rate_index = builder.select(
            "Rate",
            int(playback["rate_index"]),
            list(_PLAYBACK_RATE_LABELS),
        )
        if changed:
            playback["rate_index"] = max(
                0, min(int(rate_index), len(_PLAYBACK_RATE_STEPS) - 1)
            )

        builder.separator()
        current = int(playback["frame"])
        cursor = _cursor_series(frame_count, current)
        changed, selected = builder.timeline(
            "Timeline##replay_timeline",
            float(current),
            0.0,
            float(max_frame),
            value_track=height_values,
            marker_track=frame_marks,
            cursor_track=cursor,
            value_track_label="Ball height",
        )
        if changed:
            restore_frame(int(round(selected)))

        sample = _sample_at(samples, int(playback["frame"]))
        if (
            sample is not None
            and builder.collapsing_header("Cursor details", False)
            and builder.begin_table("Cursor details table", ["Track", "Value"])
        ):
            _add_table_row(builder, "world time", f"{sample.time:.3f} s")
            _add_table_row(builder, "simulation frame", str(sample.simulation_frame))
            _add_table_row(builder, "ball x", f"{sample.ball_x:.3f} m")
            _add_table_row(builder, "ball height", f"{sample.ball_height:.3f} m")
            builder.end_table()

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        panels=[
            ScenePanel(
                "Replay Timeline",
                build_panel,
                dock_side="bottom",
                initial_size=(960.0, 320.0),
                horizontal_scrollbar=True,
            )
        ],
        info={
            "sx_world": world,
            "recorded_frames": world.replay_frame_count,
            "timeline_frame_count": len(samples),
            "timeline_duration": duration,
            "timeline_ball_height": height_values,
        },
    )


SCENE = PythonDemoScene(
    id="sx_replay_scrubber",
    title="Replay Timeline (sx)",
    category="Simulation Replay (sx)",
    summary="Inspect and scrub an already-recorded experimental World timeline.",
    build=build,
)
