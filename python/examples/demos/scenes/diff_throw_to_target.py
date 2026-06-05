"""Differentiable throw-to-target scene (paper experiment, World).

Mirrors the throw-to-target trajectory optimization from
``tests/unit/simulation/experimental/diff/test_diff_optimization.cpp`` and the
C++ GUI demo ``examples/experimental_differentiable_gui/main.cpp``: a ballistic
projectile is thrown under gravity and its INITIAL VELOCITY is optimized by
plain gradient descent so the converged rollout lands on a fixed target. The
gradient comes from ``sx.diff.rollout`` plus ``RolloutTrajectory.gradients``
(the whole-rollout reverse-mode VJP).

The scene visualizes a ground plane, the yellow target marker, the faint gray
pre-optimization arc (rest throw, which falls short), the blue optimized-arc
trail, and the red projectile sphere that animates the converged rollout frame
by frame.

Graceful degradation: the differentiable bindings (``sx.diff.rollout`` /
``RolloutTrajectory``) only exist in a ``DART_BUILD_DIFF=ON`` build. When they
are absent the scene falls back to animating the UN-optimized (rest-throw)
trajectory of the SAME scene so it still builds and renders. ``build()`` never
raises when diff is unavailable.
"""

from __future__ import annotations

from typing import Any

import numpy as np

import dartpy as dart
import dartpy.simulation_experimental as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

# Small DOF / short horizon / few iters so the headless cycle smoke stays fast.
_TIME_STEP = 1e-2
_STEPS = 60
_MAX_ITERS = 120
_LEARNING_RATE = 0.5
_CONVERGED_LOSS = 1e-4
_TARGET = np.array([3.0, -2.0, 4.0])
_PROJECTILE_RADIUS = 0.2


def _translation(translation: Any) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(translation, dtype=float)
    return transform


def _build_throw_world() -> "sx.World | None":
    """Build the differentiable throw scene, or None if diff is unavailable.

    The differentiable World requires ``differentiable=True``; if the build was
    produced without the differentiable slice the constructor may reject the
    flag, so the whole construction is guarded.
    """

    try:
        world = sx.World(
            time_step=_TIME_STEP,
            gravity=(0.0, 0.0, -9.81),
            differentiable=True,
            contact_solver_method=sx.ContactSolverMethod.BOXED_LCP,
        )
    except Exception:  # noqa: BLE001 - diff World may be unsupported.
        return None

    # Static ground far below: never contacted over the horizon, so the rollout
    # stays a smooth contact-free ballistic arc (one differentiable regime).
    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -100.0))
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box((50.0, 50.0, 0.5)))
    ground.friction = 0.0

    projectile = world.add_rigid_body(
        "projectile", mass=1.0, position=(0.0, 0.0, 5.0)
    )
    projectile.set_collision_shape(sx.CollisionShape.sphere(_PROJECTILE_RADIUS))
    projectile.friction = 0.0

    return world


def _positions_of(states: np.ndarray) -> list[np.ndarray]:
    """Extract the (px, py, pz) of each recorded state row."""

    return [np.asarray(states[i, :3], dtype=float) for i in range(states.shape[0])]


def _sample_plot_values(values: list[float], limit: int = 120) -> list[float]:
    if len(values) <= limit:
        return [float(value) for value in values]
    indices = np.linspace(0, len(values) - 1, limit, dtype=int)
    return [float(values[int(index)]) for index in indices]


def _rollout_positions(
    diff: Any, world: "sx.World", x0: np.ndarray, controls: np.ndarray
) -> list[np.ndarray]:
    trajectory = diff.rollout(world, x0, controls, _STEPS)
    return _positions_of(np.asarray(trajectory.states))


def _optimize_throw(diff: Any, world: "sx.World") -> dict[str, Any]:
    """Gradient-descend the initial velocity so the final position hits target.

    Returns a dict with the pre-/post-optimization position trails, the
    recovered velocity, the final loss and the iteration count. Falls back to
    the un-optimized trail (and marks ``optimized=False``) on any diff failure.
    """

    initial_state = np.asarray(world.state_vector, dtype=float)
    state_size = initial_state.shape[0]
    efforts = int(world.num_efforts)
    controls = np.zeros((_STEPS, efforts), dtype=float)

    def run(velocity: np.ndarray) -> Any:
        x0 = initial_state.copy()
        x0[3:6] = velocity
        return diff.rollout(world, x0, controls, _STEPS)

    # Pre-optimization reference: rest throw, which simply falls short.
    pre_trail = _positions_of(np.asarray(run(np.zeros(3)).states))

    velocity = np.zeros(3, dtype=float)
    loss = float("inf")
    iters = 0
    for iters in range(_MAX_ITERS):
        trajectory = run(velocity)
        states = np.asarray(trajectory.states)
        final_pos = np.asarray(states[-1, :3], dtype=float)
        residual = final_pos - _TARGET
        loss = 0.5 * float(residual @ residual)
        if loss < _CONVERGED_LOSS:
            break

        # dL/dx_final: only the position rows carry the residual.
        final_state_grad = np.zeros(state_size, dtype=float)
        final_state_grad[:3] = residual
        initial_state_grad, _control_grads = trajectory.gradients(final_state_grad)
        # The decision variable is the initial velocity (state rows 3..5).
        velocity = velocity - _LEARNING_RATE * np.asarray(
            initial_state_grad, dtype=float
        )[3:6]

    final_trajectory = run(velocity)
    post_trail = _positions_of(np.asarray(final_trajectory.states))

    return {
        "optimized": True,
        "pre_trail": pre_trail,
        "post_trail": post_trail,
        "velocity": velocity,
        "loss": loss,
        "iters": iters,
    }


def _make_marker(
    render_world: Any,
    name: str,
    shape: Any,
    color: tuple[float, float, float],
    position: Any,
) -> None:
    frame = dart.SimpleFrame(dart.Frame.world(), name, _translation(position))
    frame.set_shape(shape)
    frame.create_visual_aspect().set_color(list(color))
    render_world.add_simple_frame(frame)


def build() -> SceneSetup:
    world = _build_throw_world()

    optimized = False
    trail: list[np.ndarray] = []
    pre_trail: list[np.ndarray] = []
    note = ""
    optimization = {
        "iters": 0,
        "loss": float("inf"),
        "velocity": np.zeros(3, dtype=float),
    }

    diff = getattr(sx, "diff", None)
    has_rollout = diff is not None and hasattr(diff, "rollout")

    if world is not None and has_rollout:
        try:
            world.enter_simulation_mode()
            result = _optimize_throw(diff, world)
            optimized = result["optimized"]
            trail = result["post_trail"]
            pre_trail = result["pre_trail"]
            optimization = result
            note = (
                f"optimized v0 in {result['iters']} iters, "
                f"loss={result['loss']:.2e}"
            )
        except Exception:  # noqa: BLE001 - never raise from build().
            optimized = False

    if not optimized:
        # Fallback: animate the UN-optimized (rest-throw) ballistic arc so the
        # scene still builds and renders without the differentiable bindings.
        note = "diff bindings unavailable - showing un-optimized rest throw"
        trail = _fallback_trail()
        pre_trail = []

    # A render-only world (no sx physics tracked); we replay the precomputed
    # trajectory through a playhead, so reuse WorldRenderBridge only for its
    # zero-gravity render_world plumbing.
    bridge = WorldRenderBridge(
        world if world is not None else sx.World(),
        name="diff_throw_render",
    )
    render_world = bridge.render_world
    render_world.set_time_step(_TIME_STEP)

    # Visual ground plane near the bottom of the arc (purely visual; the physics
    # ground sits far below to keep the rollout contact-free).
    _make_marker(
        render_world,
        "ground_visual",
        dart.BoxShape(np.array([8.0, 8.0, 0.1])),
        (0.40, 0.43, 0.46),
        (1.5, -1.0, 0.0),
    )
    # Target marker.
    _make_marker(
        render_world,
        "target_marker",
        dart.SphereShape(0.18),
        (0.95, 0.78, 0.16),
        _TARGET,
    )
    # Faint pre-optimization (rest-throw) trail.
    for i in range(0, len(pre_trail), 5):
        _make_marker(
            render_world,
            f"pre_opt_{i}",
            dart.SphereShape(0.05),
            (0.55, 0.58, 0.62),
            pre_trail[i],
        )
    # Optimized-arc trail.
    for i in range(0, len(trail), 4):
        _make_marker(
            render_world,
            f"opt_trail_{i}",
            dart.SphereShape(0.07),
            (0.18, 0.55, 0.95),
            trail[i],
        )

    # Animated projectile sphere replaying the converged rollout.
    start = trail[0] if trail else np.array([0.0, 0.0, 5.0])
    projectile_frame = dart.SimpleFrame(
        dart.Frame.world(), "projectile_visual", _translation(start)
    )
    projectile_frame.set_shape(dart.SphereShape(_PROJECTILE_RADIUS))
    projectile_frame.create_visual_aspect().set_color([0.93, 0.30, 0.22])
    render_world.add_simple_frame(projectile_frame)

    playhead = {"i": 0}
    playback = {"stride": 1}
    distance_plot = _sample_plot_values(
        [float(np.linalg.norm(position - _TARGET)) for position in trail]
    )
    height_plot = _sample_plot_values([float(position[2]) for position in trail])
    final_distance = (
        float(np.linalg.norm(np.asarray(trail[-1], dtype=float) - _TARGET))
        if trail
        else 0.0
    )

    def pre_step() -> None:
        if not trail:
            return
        projectile_frame.set_transform(_translation(trail[playhead["i"]]))
        stride = max(1, int(playback["stride"]))
        playhead["i"] = (playhead["i"] + stride) % len(trail)

    def build_panel(builder: object, context: object) -> None:
        del context
        current_index = playhead["i"] if trail else 0
        current_position = (
            np.asarray(trail[current_index], dtype=float)
            if trail
            else np.array([0.0, 0.0, 5.0])
        )
        current_distance = float(np.linalg.norm(current_position - _TARGET))
        velocity = np.asarray(optimization.get("velocity", np.zeros(3)), dtype=float)

        builder.text("mode: initial-velocity replay")
        builder.text(f"optimized: {'yes' if optimized else 'fallback'}")
        builder.text(
            f"frame: {current_index + 1}/{len(trail)} | "
            f"target distance={current_distance:.3f} m"
        )
        builder.text(
            f"target xyz: {_TARGET[0]:.2f}, {_TARGET[1]:.2f}, {_TARGET[2]:.2f} m"
        )
        builder.text(f"final distance: {final_distance:.3f} m")
        if optimized:
            builder.text(
                "initial velocity: "
                f"{velocity[0]:.2f}, {velocity[1]:.2f}, {velocity[2]:.2f} m/s"
            )
            builder.text(
                f"iters: {int(optimization.get('iters', 0))} | "
                f"loss={float(optimization.get('loss', 0.0)):.2e}"
            )
        else:
            builder.text(note)
        changed, stride_value = builder.slider(
            "Playback stride", float(playback["stride"]), 1.0, 8.0
        )
        if changed:
            playback["stride"] = max(1, int(round(stride_value)))
        if builder.button("Reset replay") and trail:
            playhead["i"] = 0
            projectile_frame.set_transform(_translation(trail[0]))
        if distance_plot:
            builder.separator()
            builder.plot_lines("Target distance", distance_plot)
            builder.plot_lines("Projectile height", height_plot)

    return SceneSetup(
        world=render_world,
        pre_step=pre_step,
        panels=[ScenePanel("Diff Throw Target", build_panel)],
        info={
            "optimized": optimized,
            "steps": len(trail),
            "note": note,
            "target": _TARGET.tolist(),
            "final_distance": final_distance,
        },
    )


def _fallback_trail() -> list[np.ndarray]:
    """Analytic rest-throw ballistic arc for the diff-OFF fallback.

    Reproduces the same projectile (start at z=5, no initial velocity, gravity
    -9.81) without invoking any differentiable surface, so the scene renders an
    animated arc even when ``sx.diff.rollout`` is missing.
    """

    g = np.array([0.0, 0.0, -9.81])
    position = np.array([0.0, 0.0, 5.0])
    velocity = np.zeros(3)
    trail = [position.copy()]
    for _ in range(_STEPS):
        velocity = velocity + g * _TIME_STEP
        position = position + velocity * _TIME_STEP
        trail.append(position.copy())
    return trail


SCENE = PythonDemoScene(
    id="diff_throw_to_target",
    title="Differentiable Throw-to-Target",
    category="Differentiable",
    summary=(
        "Gradient descent through sx.diff.rollout optimizes a projectile's "
        "initial velocity to hit a target (falls back to the un-optimized "
        "arc when the differentiable bindings are absent)."
    ),
    build=build,
)
