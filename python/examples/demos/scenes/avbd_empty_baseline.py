"""AVBD empty-scene baseline for source-demo corpus smoke coverage."""

from __future__ import annotations

from collections import deque
from typing import Any

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_AXIS_LENGTH = 0.7
_AXIS_HALF_WIDTH = 0.012
_SOURCE_TIME_STEP = 1.0 / 60.0
_SOURCE_GRAVITY = -10.0
_SOURCE_ITERATIONS = 10
_SOURCE_EMPTY_REFERENCES: tuple[dict[str, Any], ...] = (
    {
        "demo": "avbd-demo2d",
        "repository": "https://github.com/savant117/avbd-demo2d",
        "revision": "74699a11f858",
        "dimension": 2,
        "scene_index": 0,
        "scene_name": "Empty",
        "scene_count": 19,
        "scene_builder": "sceneEmpty",
        "scene_effect": "solver->clear()",
        "solver_defaults": {
            "time_step": _SOURCE_TIME_STEP,
            "gravity_axis": "y",
            "gravity": _SOURCE_GRAVITY,
            "iterations": _SOURCE_ITERATIONS,
        },
        "expected_counts": {
            "rigid_bodies": 0,
            "joints": 0,
            "springs": 0,
            "motors": 0,
        },
    },
    {
        "demo": "avbd-demo3d",
        "repository": "https://github.com/savant117/avbd-demo3d",
        "revision": "7701bd427d55",
        "dimension": 3,
        "scene_index": 0,
        "scene_name": "Empty",
        "scene_count": 14,
        "scene_builder": "sceneEmpty",
        "scene_effect": "solver->clear()",
        "solver_defaults": {
            "time_step": _SOURCE_TIME_STEP,
            "gravity_axis": "z",
            "gravity": _SOURCE_GRAVITY,
            "iterations": _SOURCE_ITERATIONS,
        },
        "expected_counts": {
            "rigid_bodies": 0,
            "joints": 0,
            "springs": 0,
            "motors": 0,
        },
    },
)


def _source_empty_reference_rows() -> tuple[dict[str, Any], ...]:
    rows: list[dict[str, Any]] = []
    for row in _SOURCE_EMPTY_REFERENCES:
        rows.append(
            {
                **row,
                "solver_defaults": dict(row["solver_defaults"]),
                "expected_counts": dict(row["expected_counts"]),
            }
        )
    return tuple(rows)


def _normalized_or(vector: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1.0e-12:
        return fallback
    return vector / norm


def _axis_transform(axis: np.ndarray) -> np.ndarray:
    x_axis = _normalized_or(axis, np.array([1.0, 0.0, 0.0]))
    reference = np.array([0.0, 0.0, 1.0])
    if abs(float(reference @ x_axis)) > 0.95:
        reference = np.array([0.0, 1.0, 0.0])
    y_axis = _normalized_or(np.cross(reference, x_axis), np.array([0.0, 1.0, 0.0]))
    z_axis = np.cross(x_axis, y_axis)

    transform = np.eye(4)
    transform[:3, 0] = x_axis
    transform[:3, 1] = y_axis
    transform[:3, 2] = z_axis
    transform[:3, 3] = 0.5 * _AXIS_LENGTH * x_axis
    return transform


def build() -> SceneSetup:
    world = sx.World(time_step=_SOURCE_TIME_STEP, gravity=(0.0, 0.0, _SOURCE_GRAVITY))
    world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_empty_baseline_render")
    markers = []
    axes = (
        ("x", np.array([1.0, 0.0, 0.0]), (0.92, 0.24, 0.18)),
        ("y", np.array([0.0, 1.0, 0.0]), (0.24, 0.70, 0.30)),
        ("z", np.array([0.0, 0.0, 1.0]), (0.25, 0.52, 0.92)),
    )
    for label, axis, color in axes:
        marker = dart.SimpleFrame(
            dart.gui.world_render_frame(),
            f"avbd_empty_baseline_{label}_axis",
            _axis_transform(axis),
        )
        marker.set_shape(
            dart.BoxShape(np.array([_AXIS_LENGTH, _AXIS_HALF_WIDTH, _AXIS_HALF_WIDTH]))
        )
        marker.create_visual_aspect().set_color(color)
        bridge.render_world.add_simple_frame(marker)
        markers.append(marker)

    bridge.sync()

    time_history: deque[float] = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        time_history.append(float(world.time))

        builder.text("source corpus: avbd-demo2d/3d empty baseline")
        builder.text("source defaults: dt=1/60, |g|=10, 10 iterations")
        builder.text("solver: variational integrator, no simulated bodies")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"multibodies: {world.num_multibodies}")
        builder.plot_lines("Time", list(time_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Empty Baseline", build_panel)],
        info={
            "sx_world": world,
            "markers": markers,
            "source_demo_rows": ("avbd-demo2d empty", "avbd-demo3d empty"),
            "source_demo_reference_rows": _source_empty_reference_rows(),
            "replay_sync": bridge.sync,
            "replay_live_step_is_stateless": True,
        },
    )


SCENE = PythonDemoScene(
    id="avbd_empty_baseline",
    title="AVBD Empty Baseline (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="An empty variational-integrator World baseline for the AVBD "
    "source-demo corpus harness.",
    build=build,
)
