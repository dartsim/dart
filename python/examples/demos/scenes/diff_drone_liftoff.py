"""Differentiable drone lift-off / saddle-escape scene (paper experiment, World).

Reproduces the Nimble paper's complementarity-aware saddle-escape experiment
(arXiv:2103.16021, Section VII-C, Fig 8) from
``tests/unit/simulation/diff/test_diff_paper_experiments.cpp``
(the ``DroneLiftOff...`` test / ``optimizeDrone`` / ``buildDroneWorld``).

A single free rigid-body "drone" rests on the ground, an active *clamping*
contact. A constant vertical thrust is applied each step through the rollout
control matrix (column 2, the z-force). The thrust is optimized by plain
gradient descent so the drone reaches a target hover height. The whole-rollout
reverse-mode VJP (``sx.diff.rollout`` + ``RolloutTrajectory.gradients``) supplies
the gradient, and the contact-gradient mode selects the gradient regime:

- ``ContactGradientMode.ANALYTIC`` (naive): while the drone rests, the true
  gradient ``dq'/dtau`` through the clamping contact is 0, so SGD stalls in the
  saddle — the thrust never grows and the drone stays grounded at z ~ 0.2.
- ``ContactGradientMode.COMPLEMENTARITY_AWARE``: the gradient is non-zero in the
  clamping saddle, so SGD raises the thrust past gravity, the drone lifts off
  and climbs toward the target (z ~ 1.38, near the 1.5 m target).

The scene makes the escape legible: a yellow target-height marker at z=1.5, the
ground plane, the COMPLEMENTARITY_AWARE drone (red sphere) animated climbing
from the ground to the target by replaying its rollout heights frame by frame,
a faint gray GHOST drone showing the naive (ANALYTIC) result staying grounded at
z ~ 0.2 (static, for contrast), and a vertical breadcrumb trail of the aware
drone's climb.

Graceful degradation: the differentiable bindings (``sx.diff.rollout`` /
``RolloutTrajectory``) only exist in a ``DART_BUILD_DIFF=ON`` build. When they
are absent (or any optimization step fails) the scene falls back to a static
drone resting at the ground height so it still builds and renders. ``build()``
never raises when diff is unavailable; the status is recorded in ``info["note"]``.
"""

from __future__ import annotations

from typing import Any

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

# Mirror the C++ test physics. Iterations/lr match optimizeDrone(..., 400, 4.0).
_TIME_STEP = 1e-2
_HORIZON = 150
_MAX_ITERS = 400
_LEARNING_RATE = 4.0
_DRONE_RADIUS = 0.2
_DRONE_MASS = 1.0
_TARGET_Z = 1.5
# Drone resting height: centre at the radius minus a sub-allowance penetration
# so the ground contact is active and clamping at the start (the saddle).
_REST_Z = _DRONE_RADIUS - 5e-5
_THRUST_COLUMN = 2  # vertical (z) force column of the rollout control matrix.
_Z_STATE_ROW = 2  # final-state z position row in the World state vector.


def _translation(translation: Any) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(translation, dtype=float)
    return transform


def _build_drone_world(mode: Any) -> "sx.World | None":
    """Build the differentiable drone world, or None if diff is unavailable.

    Mirrors ``buildDroneWorld``: ground box (half-extents (10,10,0.5)) at
    z=-0.5 so its top face is at z=0; a free rigid-body drone (mass 1.0, sphere
    radius 0.2) resting at z=0.2-5e-5; BOXED_LCP solver; differentiable; with
    the given contact-gradient ``mode``.
    """

    try:
        world = sx.World(
            time_step=_TIME_STEP,
            gravity=(0.0, 0.0, -9.81),
            differentiable=True,
            contact_solver_method=sx.ContactSolverMethod.BOXED_LCP,
            contact_gradient_mode=mode,
        )
    except Exception:  # noqa: BLE001 - diff World / kw may be unsupported.
        return None

    # Ground: makeBox takes HALF-extents, so a 0.5 half-height box at z=-0.5 has
    # its top face at z=0; the drone (radius 0.2) then rests with centre at z=0.2.
    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -0.5))
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box((10.0, 10.0, 0.5)))
    ground.friction = 0.0

    drone = world.add_rigid_body(
        "drone", mass=_DRONE_MASS, position=(0.0, 0.0, _REST_Z)
    )
    drone.set_collision_shape(sx.CollisionShape.sphere(_DRONE_RADIUS))
    drone.friction = 0.0

    return world


def _rollout_heights(trajectory: Any) -> list[float]:
    """Extract the drone's z height at each recorded state row."""

    states = np.asarray(trajectory.states)
    return [float(states[i, _Z_STATE_ROW]) for i in range(states.shape[0])]


def _sample_plot_values(values: list[float], limit: int = 120) -> list[float]:
    if len(values) <= limit:
        return [float(value) for value in values]
    indices = np.linspace(0, len(values) - 1, limit, dtype=int)
    return [float(values[int(index)]) for index in indices]


def _finite_float(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if np.isfinite(result) else default


def _history_stats(values: Any) -> dict[str, float]:
    samples = [_finite_float(value) for value in values or []]
    if not samples:
        samples = [0.0]
    return {
        "samples": float(len(samples)),
        "first": samples[0],
        "latest": samples[-1],
        "min": min(samples),
        "max": max(samples),
    }


def _optimize_drone(diff: Any, mode: Any) -> dict[str, Any]:
    """Gradient-descend a constant vertical thrust for the given gradient mode.

    Mirrors ``optimizeDrone``: loss = 0.5*(target_z - final_z)^2; the thrust
    gradient is the whole-rollout VJP summed over the (shared) per-step z-force
    component; ``thrust -= lr*grad`` clamped non-negative. Returns the converged
    thrust, final height, loss and the final rollout's frame-by-frame heights.
    Raises on any diff failure; the caller guards it.
    """

    thrust = 0.0  # start at rest on the ground (the clamping saddle).
    loss = float("inf")
    final_z = _REST_Z
    final_heights: list[float] = []
    thrust_history: list[float] = []
    loss_history: list[float] = []
    final_z_history: list[float] = []
    thrust_grad_history: list[float] = []

    for _ in range(_MAX_ITERS):
        # Fresh world each iteration so the rollout starts from the same state.
        world = _build_drone_world(mode)
        if world is None:
            raise RuntimeError("differentiable drone World unavailable")
        world.enter_simulation_mode()

        x0 = np.asarray(world.state_vector, dtype=float)
        state_size = x0.shape[0]
        efforts = int(world.num_efforts)
        controls = np.zeros((_HORIZON, efforts), dtype=float)
        controls[:, _THRUST_COLUMN] = thrust  # constant vertical (z) force.

        trajectory = diff.rollout(world, x0, controls, _HORIZON)
        heights = _rollout_heights(trajectory)
        if not heights:
            raise RuntimeError("rollout produced no states")
        final_z = heights[-1]
        residual = final_z - _TARGET_Z
        loss = 0.5 * residual * residual

        # dL/d(final z): only the z position row of the final state carries it.
        final_state_grad = np.zeros(state_size, dtype=float)
        final_state_grad[_Z_STATE_ROW] = residual
        _initial_state_grad, control_grads = trajectory.gradients(final_state_grad)
        control_grads = np.asarray(control_grads, dtype=float)
        # The single decision variable (shared thrust) is the sum over steps of
        # the z-force control gradient column.
        thrust_grad = float(np.sum(control_grads[:, _THRUST_COLUMN]))
        thrust_history.append(float(thrust))
        loss_history.append(float(loss))
        final_z_history.append(float(final_z))
        thrust_grad_history.append(float(thrust_grad))
        thrust -= _LEARNING_RATE * thrust_grad
        if thrust < 0.0:
            thrust = 0.0  # physical thrust is non-negative.

        final_heights = heights

    return {
        "thrust": thrust,
        "final_z": final_z,
        "loss": loss,
        "heights": final_heights,
        "thrust_history": thrust_history,
        "loss_history": loss_history,
        "final_z_history": final_z_history,
        "thrust_grad_history": thrust_grad_history,
    }


def _make_marker(
    render_world: Any,
    name: str,
    shape: Any,
    color: tuple[float, float, float],
    position: Any,
) -> Any:
    frame = dart.SimpleFrame(
        dart.gui.world_render_frame(), name, _translation(position)
    )
    frame.set_shape(shape)
    frame.create_visual_aspect().set_color(list(color))
    render_world.add_simple_frame(frame)
    return frame


def build(
    panel_title: str = "Diff Drone Lift-Off",
    mode_text: str = "mode: complementarity-aware replay",
) -> SceneSetup:
    diff = getattr(sx, "diff", None)
    has_rollout = diff is not None and hasattr(diff, "rollout")

    optimized = False
    note = ""
    aware: dict[str, Any] = {}
    naive: dict[str, Any] = {}

    if has_rollout:
        try:
            naive = _optimize_drone(diff, sx.ContactGradientMode.ANALYTIC)
            aware = _optimize_drone(diff, sx.ContactGradientMode.COMPLEMENTARITY_AWARE)
            optimized = True
            note = (
                f"saddle escape: naive thrust={naive['thrust']:.2f} "
                f"height={naive['final_z']:.2f} | aware thrust={aware['thrust']:.2f} "
                f"height={aware['final_z']:.2f} (target z={_TARGET_Z})"
            )
        except Exception:  # noqa: BLE001 - never raise from build().
            optimized = False

    if not optimized:
        # Fallback: a static drone resting at the ground height with a flat
        # height trail, so the scene still builds and renders without diff.
        note = "diff bindings unavailable - showing static grounded drone"
        aware = {
            "thrust": 0.0,
            "final_z": _REST_Z,
            "loss": 0.0,
            "heights": [_REST_Z],
            "thrust_history": [0.0],
            "loss_history": [0.0],
            "final_z_history": [_REST_Z],
            "thrust_grad_history": [0.0],
        }
        naive = {
            "thrust": 0.0,
            "final_z": _REST_Z,
            "loss": 0.0,
            "heights": [_REST_Z],
            "thrust_history": [0.0],
            "loss_history": [0.0],
            "final_z_history": [_REST_Z],
            "thrust_grad_history": [0.0],
        }

    aware_heights = aware.get("heights") or [_REST_Z]
    naive_z = float(naive.get("final_z", _REST_Z))

    # Render-only world; replay the precomputed heights through a playhead. Reuse
    # WorldRenderBridge only for its zero-gravity render_world plumbing.
    bridge = WorldRenderBridge(sx.World(), name="diff_drone_render")
    render_world = bridge.render_world
    render_world.set_time_step(_TIME_STEP)

    # Ground plane (purely visual; physics ground lives in the rollout world).
    _make_marker(
        render_world,
        "ground_visual",
        dart.BoxShape(np.array([4.0, 4.0, 0.1])),
        (0.40, 0.43, 0.46),
        (0.0, 0.0, -0.05),
    )
    # Yellow target-height marker at z=1.5 (a thin disk the drone climbs toward).
    _make_marker(
        render_world,
        "target_marker",
        dart.BoxShape(np.array([0.8, 0.8, 0.03])),
        (0.95, 0.78, 0.16),
        (0.0, 0.0, _TARGET_Z),
    )

    # Faint gray GHOST drone: the naive (ANALYTIC) result, staying grounded at
    # z ~ 0.2 — static, for contrast against the climbing aware drone.
    _make_marker(
        render_world,
        "naive_ghost",
        dart.SphereShape(_DRONE_RADIUS),
        (0.55, 0.58, 0.62),
        (0.45, 0.0, naive_z),
    )

    # Vertical breadcrumb trail of the aware drone's climb (small blue spheres).
    for i in range(0, len(aware_heights), 6):
        _make_marker(
            render_world,
            f"climb_trail_{i}",
            dart.SphereShape(0.05),
            (0.20, 0.55, 0.95),
            (0.0, 0.0, aware_heights[i]),
        )

    # Animated COMPLEMENTARITY_AWARE drone (red sphere) climbing to the target.
    drone_frame = _make_marker(
        render_world,
        "drone_visual",
        dart.SphereShape(_DRONE_RADIUS),
        (0.93, 0.30, 0.22),
        (0.0, 0.0, aware_heights[0]),
    )

    playhead = {"i": 0}
    playback = {"stride": 1}
    height_plot = _sample_plot_values([float(value) for value in aware_heights])
    aware_loss_plot = _sample_plot_values(
        [float(value) for value in aware.get("loss_history", [aware.get("loss", 0.0)])]
    )
    aware_thrust_plot = _sample_plot_values(
        [
            float(value)
            for value in aware.get("thrust_history", [aware.get("thrust", 0.0)])
        ]
    )
    aware_grad_plot = _sample_plot_values(
        [float(value) for value in aware.get("thrust_grad_history", [0.0])]
    )
    naive_loss_plot = _sample_plot_values(
        [float(value) for value in naive.get("loss_history", [naive.get("loss", 0.0)])]
    )

    def capture_metrics() -> dict[str, Any]:
        current_index = playhead["i"] if aware_heights else 0
        current_z = _finite_float(
            aware_heights[current_index] if aware_heights else _REST_Z,
            _REST_Z,
        )
        aware_final_z = _finite_float(aware.get("final_z", _REST_Z), _REST_Z)
        naive_final_z = _finite_float(naive.get("final_z", _REST_Z), _REST_Z)
        aware_thrust = _finite_float(aware.get("thrust", 0.0))
        naive_thrust = _finite_float(naive.get("thrust", 0.0))
        aware_loss = _finite_float(aware.get("loss", 0.0))
        naive_loss = _finite_float(naive.get("loss", 0.0))
        height_gap = aware_final_z - naive_final_z
        aware_target_error = abs(_TARGET_Z - aware_final_z)
        naive_target_error = abs(_TARGET_Z - naive_final_z)
        aware_height_stats = _history_stats(aware.get("final_z_history", []))
        if aware_height_stats["samples"] <= 1.0:
            aware_height_stats = _history_stats(aware_heights)
        aware_thrust_stats = _history_stats(aware.get("thrust_history", []))
        aware_loss_stats = _history_stats(aware.get("loss_history", []))
        aware_grad_stats = _history_stats(aware.get("thrust_grad_history", []))
        naive_loss_stats = _history_stats(naive.get("loss_history", []))
        status = "fallback"
        if optimized:
            status = (
                "saddle_escape"
                if aware_final_z > naive_final_z + 1e-3
                else "optimized_no_escape"
            )

        return {
            "row": "diff_drone_liftoff",
            "category": "Differentiable",
            "related_source_row": "rigid_contact_solver_compare",
            "solver": "boxed_lcp_contact_gradient_modes",
            "scope": "rigid_contact_gradient_saddle_escape",
            "contact_solver_method": "BOXED_LCP",
            "status": status,
            "optimized": optimized,
            "gradient_modes": ["ANALYTIC", "COMPLEMENTARITY_AWARE"],
            "time_step_ms": _TIME_STEP * 1000.0,
            "horizon": float(_HORIZON),
            "max_iters": float(_MAX_ITERS),
            "learning_rate": _LEARNING_RATE,
            "target_z": _TARGET_Z,
            "rest_z": _REST_Z,
            "drone_radius": _DRONE_RADIUS,
            "drone_mass": _DRONE_MASS,
            "playhead_index": float(current_index),
            "steps": float(len(aware_heights)),
            "current_z": current_z,
            "naive_mode": "ANALYTIC",
            "aware_mode": "COMPLEMENTARITY_AWARE",
            "naive_thrust": naive_thrust,
            "naive_final_z": naive_final_z,
            "naive_loss": naive_loss,
            "aware_thrust": aware_thrust,
            "aware_final_z": aware_final_z,
            "aware_loss": aware_loss,
            "height_gap": height_gap,
            "aware_target_error": aware_target_error,
            "naive_target_error": naive_target_error,
            "target_error_gap": naive_target_error - aware_target_error,
            "thrust_gap": aware_thrust - naive_thrust,
            "aware_min_height": aware_height_stats["min"],
            "aware_max_height": aware_height_stats["max"],
            "aware_max_thrust": aware_thrust_stats["max"],
            "aware_last_thrust_gradient": aware_grad_stats["latest"],
            "aware_min_loss": aware_loss_stats["min"],
            "naive_min_loss": naive_loss_stats["min"],
            "aware_history_samples": float(len(aware_thrust_plot)),
            "naive_history_samples": float(len(naive_loss_plot)),
            "history": {
                "aware_height": aware_height_stats,
                "aware_thrust": aware_thrust_stats,
                "aware_loss": aware_loss_stats,
                "aware_thrust_gradient": aware_grad_stats,
                "naive_loss": naive_loss_stats,
            },
            "modes": {
                "ANALYTIC": {
                    "thrust": naive_thrust,
                    "final_z": naive_final_z,
                    "loss": naive_loss,
                    "target_error": naive_target_error,
                },
                "COMPLEMENTARITY_AWARE": {
                    "thrust": aware_thrust,
                    "final_z": aware_final_z,
                    "loss": aware_loss,
                    "target_error": aware_target_error,
                },
            },
        }

    def pre_step() -> None:
        if not aware_heights:
            return
        z = aware_heights[playhead["i"]]
        drone_frame.set_transform(_translation((0.0, 0.0, z)))
        stride = max(1, int(playback["stride"]))
        playhead["i"] = (playhead["i"] + stride) % len(aware_heights)

    def build_panel(builder: object, context: object) -> None:
        del context
        current_index = playhead["i"] if aware_heights else 0
        current_z = float(aware_heights[current_index]) if aware_heights else _REST_Z
        builder.text(mode_text)
        builder.text(f"optimized: {'yes' if optimized else 'fallback'}")
        builder.text(
            f"frame: {current_index + 1}/{len(aware_heights)} | z={current_z:.3f} m"
        )
        builder.text(f"target z: {_TARGET_Z:.3f} m")
        changed, stride_value = builder.slider(
            "Playback stride", float(playback["stride"]), 1.0, 8.0
        )
        if changed:
            playback["stride"] = max(1, int(round(stride_value)))
        if builder.button("Reset replay"):
            playhead["i"] = 0
            drone_frame.set_transform(_translation((0.0, 0.0, aware_heights[0])))
        builder.separator()
        builder.text(
            f"ANALYTIC: thrust={float(naive.get('thrust', 0.0)):.2f} N | "
            f"final z={naive_z:.3f} m | loss={float(naive.get('loss', 0.0)):.4f}"
        )
        builder.text(
            f"AWARE: thrust={float(aware.get('thrust', 0.0)):.2f} N | "
            f"final z={float(aware.get('final_z', _REST_Z)):.3f} m | "
            f"loss={float(aware.get('loss', 0.0)):.4f}"
        )
        builder.plot_lines("Aware height", height_plot)
        builder.plot_lines("Aware thrust", aware_thrust_plot)
        builder.plot_lines("Aware loss", aware_loss_plot)
        builder.plot_lines("Aware thrust gradient", aware_grad_plot)
        builder.plot_lines("Analytic loss", naive_loss_plot)

    return SceneSetup(
        world=render_world,
        pre_step=pre_step,
        panels=[ScenePanel(panel_title, build_panel)],
        info={
            "optimized": optimized,
            "steps": len(aware_heights),
            "note": note,
            "gradient_modes": ["ANALYTIC", "COMPLEMENTARITY_AWARE"],
            "target_z": _TARGET_Z,
            "naive_thrust": float(naive.get("thrust", 0.0)),
            "naive_height": naive_z,
            "aware_thrust": float(aware.get("thrust", 0.0)),
            "aware_height": float(aware.get("final_z", _REST_Z)),
            "aware_history_count": len(aware_thrust_plot),
            "naive_history_count": len(naive_loss_plot),
            CAPTURE_METRICS_INFO_KEY: capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="diff_drone_liftoff",
    title="Differentiable Drone Lift-Off / Saddle Escape",
    category="Differentiable",
    summary=(
        "Reproduces the Nimble paper's complementarity-aware saddle-escape "
        "experiment (VII-C, Fig 8): a drone resting on the ground optimizes a "
        "vertical thrust by gradient descent. Naive (ANALYTIC) gradients stall "
        "in the clamping saddle (gray ghost stays grounded); "
        "COMPLEMENTARITY_AWARE gradients escape and the red drone lifts off "
        "toward the yellow target height. Falls back to a static grounded drone "
        "when the differentiable bindings are absent."
    ),
    build=build,
)
