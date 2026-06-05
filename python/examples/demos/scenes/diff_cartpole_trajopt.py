"""Differentiable cartpole trajectory-optimization scene (World).

Mirrors the cartpole experiment from
``tests/unit/simulation/experimental/diff/test_diff_paper_experiments.cpp``
(Nimble paper Section VII-B): a cart that slides along world X (prismatic,
actuated) carries a pole that hinges about world Y (revolute, unactuated). A
per-step cart-force sequence is optimized by plain gradient descent so the cart
reaches a target X position at the final step. The per-step joint-space
Jacobians (``state_jacobian`` / ``control_jacobian``) come from
``World.get_step_derivatives()``; the reverse pass is a manual adjoint over the
chained per-step Jacobians (the World state vector covers only rigid bodies, so
a multibody rollout is driven through its joints rather than ``diff.rollout``).

The scene animates the controlled rollout: the cart box slides toward the
target marker while the pole swings, replaying the recorded joint states frame
by frame.

Graceful degradation: ``get_step_derivatives`` only produces useful analytic
Jacobians in a ``DART_BUILD_DIFF=ON`` differentiable build. When the
differentiable World or step derivatives are unavailable, the scene falls back
to animating the UN-optimized (zero-force) rollout of the SAME cartpole so it
still builds and renders. ``build()`` never raises when diff is absent.
"""

from __future__ import annotations

from typing import Any

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

# Small system / short horizon / few iters so the headless cycle smoke is fast.
_TIME_STEP = 1e-2
_STEPS = 50
_MAX_ITERS = 200
_LEARNING_RATE = 2.0
_CONVERGED_LOSS = 1e-4
_TARGET = 0.5  # desired final cart X position (metres)
_CART_DOF = 0
_STATE_SIZE = 4  # [cart_x, pole_th, cart_v, pole_w]
_POLE_LENGTH = 0.5

_CART_EXTENT = np.array([0.3, 0.2, 0.15])
_POLE_EXTENT = np.array([_POLE_LENGTH, 0.06, 0.06])


def _translation(translation: Any) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(translation, dtype=float)
    return transform


def _build_cartpole(differentiable: bool) -> "sx.World | None":
    """Build the cartpole world, or None if construction fails."""

    try:
        if differentiable:
            world = sx.World(
                time_step=_TIME_STEP,
                gravity=(0.0, 0.0, -9.81),
                differentiable=True,
            )
        else:
            world = sx.World(time_step=_TIME_STEP, gravity=(0.0, 0.0, -9.81))
    except Exception:  # noqa: BLE001 - diff World may be unsupported.
        return None

    cartpole = world.add_multibody("cartpole")
    base = cartpole.add_link("base")

    cart = cartpole.add_link(
        "cart",
        parent=base,
        joint=sx.JointSpec(
            name="slide", type=sx.JointType.PRISMATIC, axis=(1.0, 0.0, 0.0)
        ),
    )
    cart.mass = 1.0
    cart.inertia = ((0.01, 0.0, 0.0), (0.0, 0.01, 0.0), (0.0, 0.0, 0.01))

    pole = cartpole.add_link(
        "pole",
        parent=cart,
        joint=sx.JointSpec(
            name="hinge",
            type=sx.JointType.REVOLUTE,
            axis=(0.0, 1.0, 0.0),
            transform_from_parent=_translation((_POLE_LENGTH, 0.0, 0.0)),
        ),
    )
    pole.mass = 0.5
    pole.inertia = ((0.01, 0.0, 0.0), (0.0, 0.01, 0.0), (0.0, 0.0, 0.01))

    world.enter_simulation_mode()
    return world


def _read_state(cartpole: Any) -> np.ndarray:
    """Read [q; q̇] (joints in construction order) for the cartpole."""

    q: list[float] = []
    qdot: list[float] = []
    for joint in cartpole.joints:
        q.extend(np.asarray(joint.position, dtype=float).reshape(-1).tolist())
        qdot.extend(np.asarray(joint.velocity, dtype=float).reshape(-1).tolist())
    return np.asarray(q + qdot, dtype=float)


def _sample_plot_values(values: list[float], limit: int = 120) -> list[float]:
    if len(values) <= limit:
        return [float(value) for value in values]
    indices = np.linspace(0, len(values) - 1, limit, dtype=int)
    return [float(values[int(index)]) for index in indices]


def _rollout_cartpole(
    world: Any, cart_force: np.ndarray, want_derivatives: bool
) -> dict[str, Any]:
    """Roll out the cartpole driven by a per-step cart force.

    Records the joint state after each step (and, if requested, the per-step
    Jacobians for the reverse pass). Returns a dict with ``states`` and, when
    available, ``derivatives``.
    """

    cartpole = world.get_multibody("cartpole")
    states = [_read_state(cartpole)]
    derivatives: list[Any] = []

    for t in range(cart_force.shape[0]):
        joints = cartpole.joints
        joints[0].force = [float(cart_force[t])]
        joints[1].force = [0.0]
        world.step()
        states.append(_read_state(cartpole))
        if want_derivatives:
            derivatives.append(world.get_step_derivatives())

    return {"states": states, "derivatives": derivatives}


def _optimize_cartpole() -> dict[str, Any]:
    """Gradient-descend a cart-force sequence so the cart reaches the target.

    Returns a dict with the recorded final-rollout states (size-4 each), the
    converged controls, the final loss, the iteration count and ``optimized``.
    Raises on any diff failure; the caller guards it.
    """

    controls = np.zeros(_STEPS, dtype=float)
    loss = float("inf")
    iters = 0

    for iters in range(_MAX_ITERS):
        # Fresh world each iteration so the rollout starts from the same state.
        world = _build_cartpole(differentiable=True)
        if world is None:
            raise RuntimeError("differentiable cartpole World unavailable")
        rollout = _rollout_cartpole(world, controls, want_derivatives=True)
        if len(rollout["derivatives"]) != _STEPS:
            raise RuntimeError("step derivatives unavailable")

        cart_final = float(rollout["states"][-1][_CART_DOF])
        residual = cart_final - _TARGET
        loss = 0.5 * residual * residual
        if loss < _CONVERGED_LOSS:
            break

        # Adjoint reverse pass accumulating dL/dτ_t. Only the final state
        # carries a loss, on the cart-position row.
        adjoint = np.zeros(_STATE_SIZE, dtype=float)
        adjoint[_CART_DOF] = residual
        control_grad = np.zeros(_STEPS, dtype=float)
        for t in range(_STEPS - 1, -1, -1):
            d = rollout["derivatives"][t]
            control_jacobian = np.asarray(d.control_jacobian, dtype=float)
            state_jacobian = np.asarray(d.state_jacobian, dtype=float)
            # dL/dτ_t = controlJacobianᵀ adjoint, take the cart column.
            control_grad[t] = float(control_jacobian[:, _CART_DOF] @ adjoint)
            adjoint = state_jacobian.T @ adjoint

        controls = controls - _LEARNING_RATE * control_grad

    final_world = _build_cartpole(differentiable=True)
    if final_world is None:
        raise RuntimeError("differentiable cartpole World unavailable")
    final_rollout = _rollout_cartpole(final_world, controls, want_derivatives=False)

    return {
        "optimized": True,
        "states": final_rollout["states"],
        "controls": controls,
        "loss": loss,
        "iters": iters,
    }


def _unoptimized_cartpole() -> dict[str, Any]:
    """Zero-force (uncontrolled) rollout used as the diff-OFF fallback."""

    world = _build_cartpole(differentiable=False)
    if world is None:
        # Last-resort: an empty trajectory so build() still returns a scene.
        return {"optimized": False, "states": [], "controls": np.zeros(_STEPS)}
    controls = np.zeros(_STEPS, dtype=float)
    rollout = _rollout_cartpole(world, controls, want_derivatives=False)
    return {"optimized": False, "states": rollout["states"], "controls": controls}


def _cart_transform(cart_x: float) -> np.ndarray:
    return _translation((cart_x, 0.0, 0.0))


def _pole_transform(cart_x: float, pole_theta: float) -> np.ndarray:
    """Pose the pole box: hinged at the cart, rotated by theta about world Y.

    The pole link sits a half-length out along its local +X from the hinge; the
    hinge is at the cart's far end (pole offset = +pole_length along X).
    """

    transform = np.eye(4)
    cos_t = float(np.cos(pole_theta))
    sin_t = float(np.sin(pole_theta))
    # Rotation about Y by theta.
    transform[0, 0] = cos_t
    transform[0, 2] = sin_t
    transform[2, 0] = -sin_t
    transform[2, 2] = cos_t
    hinge = np.array([cart_x + _POLE_LENGTH, 0.0, 0.0])
    # Place the pole-box center half a length out along the rotated local +X.
    local_offset = np.array([0.5 * _POLE_LENGTH, 0.0, 0.0])
    transform[:3, 3] = hinge + transform[:3, :3] @ local_offset
    return transform


def _make_marker(
    render_world: Any,
    name: str,
    shape: Any,
    color: tuple[float, float, float],
    transform: np.ndarray,
) -> Any:
    frame = dart.SimpleFrame(dart.gui.world_render_frame(), name, transform)
    frame.set_shape(shape)
    frame.create_visual_aspect().set_color(list(color))
    render_world.add_simple_frame(frame)
    return frame


def _cart_trail(
    render_world: Any,
    prefix: str,
    states: list[np.ndarray],
    color: tuple[float, float, float],
    radius: float,
    every: int = 4,
) -> None:
    """Drop breadcrumb spheres along the cart's X path so the trajectory the
    optimizer produced is visible at a glance (not only while it animates)."""

    for i in range(0, len(states), every):
        cart_x = float(states[i][_CART_DOF])
        _make_marker(
            render_world,
            f"{prefix}_{i}",
            dart.SphereShape(radius),
            color,
            _translation((cart_x, 0.0, 0.05)),
        )


def build() -> SceneSetup:
    diff = getattr(sx, "diff", None)
    # The cartpole path uses World.get_step_derivatives (analytic articulated
    # Jacobians), which only produce useful gradients in a differentiable
    # build. ``sx.diff`` presence is the same gate used by the throw scene.
    diff_available = diff is not None and hasattr(diff, "rollout")

    result: dict[str, Any]
    note = ""
    if diff_available:
        try:
            result = _optimize_cartpole()
            note = (
                f"optimized cart force in {result['iters']} iters, "
                f"loss={result['loss']:.2e}"
            )
        except Exception:  # noqa: BLE001 - never raise from build().
            result = _unoptimized_cartpole()
            note = "diff optimization failed - showing uncontrolled rollout"
    else:
        result = _unoptimized_cartpole()
        note = "diff bindings unavailable - showing uncontrolled rollout"

    states = result["states"]

    # Render-only world; replay the recorded joint states through a playhead.
    bridge = WorldRenderBridge(sx.World(), name="diff_cartpole_render")
    render_world = bridge.render_world
    render_world.set_time_step(_TIME_STEP)

    # Target marker for the cart's final X position (thin vertical post).
    _make_marker(
        render_world,
        "cart_target",
        dart.BoxShape(np.array([0.04, 0.3, 1.0])),
        (0.95, 0.78, 0.16),
        _translation((_TARGET, 0.0, 0.5)),
    )
    # Visual rail along which the cart slides.
    _make_marker(
        render_world,
        "rail",
        dart.BoxShape(np.array([3.0, 0.05, 0.02])),
        (0.40, 0.43, 0.46),
        _translation((0.0, 0.0, 0.0)),
    )

    cart_x0 = float(states[0][_CART_DOF]) if states else 0.0
    pole_th0 = float(states[0][1]) if states else 0.0

    # Debugging visualization — make the optimizer's effect legible (the loop
    # otherwise just replays the converged motion). For contrast, overlay the
    # UN-optimized (zero-force) cart path, where the cart barely moves, against
    # the optimized blue path that reaches the yellow target post.
    if result["optimized"]:
        try:
            ghost_states = _unoptimized_cartpole()["states"]
        except Exception:  # noqa: BLE001 - viz only; never raise from build().
            ghost_states = []
        _cart_trail(
            render_world, "uncontrolled", ghost_states, (0.55, 0.58, 0.62), 0.035
        )
    # Blue breadcrumbs: the optimized cart path driving toward the target.
    _cart_trail(render_world, "optimized_path", states, (0.20, 0.55, 0.95), 0.05)
    # Green: the cart's start position.
    _make_marker(
        render_world,
        "cart_start",
        dart.SphereShape(0.07),
        (0.30, 0.80, 0.45),
        _translation((cart_x0, 0.0, 0.05)),
    )

    cart_frame = _make_marker(
        render_world,
        "cart_visual",
        dart.BoxShape(_CART_EXTENT),
        (0.30, 0.50, 0.80),
        _cart_transform(cart_x0),
    )
    pole_frame = _make_marker(
        render_world,
        "pole_visual",
        dart.BoxShape(_POLE_EXTENT),
        (0.90, 0.65, 0.30),
        _pole_transform(cart_x0, pole_th0),
    )

    playhead = {"i": 0}
    playback = {"stride": 1}
    controls = np.asarray(result.get("controls", np.zeros(_STEPS)), dtype=float)
    cart_positions = [float(state[_CART_DOF]) for state in states]
    target_errors = [float(position - _TARGET) for position in cart_positions]
    pole_angles = [float(state[1]) for state in states]
    force_plot = _sample_plot_values([float(value) for value in controls])
    cart_plot = _sample_plot_values(cart_positions)
    error_plot = _sample_plot_values(target_errors)
    pole_plot = _sample_plot_values(pole_angles)
    final_cart_x = cart_positions[-1] if cart_positions else 0.0
    final_error = final_cart_x - _TARGET
    peak_force = float(np.max(np.abs(controls))) if controls.size else 0.0

    def pre_step() -> None:
        if not states:
            return
        state = states[playhead["i"]]
        cart_x = float(state[_CART_DOF])
        pole_theta = float(state[1])
        cart_frame.set_transform(_cart_transform(cart_x))
        pole_frame.set_transform(_pole_transform(cart_x, pole_theta))
        stride = max(1, int(playback["stride"]))
        playhead["i"] = (playhead["i"] + stride) % len(states)

    def build_panel(builder: object, context: object) -> None:
        del context
        current_index = playhead["i"] if states else 0
        current_state = (
            np.asarray(states[current_index], dtype=float)
            if states
            else np.zeros(_STATE_SIZE, dtype=float)
        )
        current_force = (
            float(controls[min(current_index, controls.shape[0] - 1)])
            if controls.size
            else 0.0
        )

        builder.text("mode: cart-force replay")
        builder.text(f"optimized: {'yes' if result['optimized'] else 'fallback'}")
        builder.text(
            f"frame: {current_index + 1}/{len(states)} | "
            f"cart x={float(current_state[_CART_DOF]):.3f} m"
        )
        builder.text(
            f"target x: {_TARGET:.3f} m | final error={final_error:.3f} m"
        )
        builder.text(
            f"pole angle: {float(current_state[1]):.3f} rad | "
            f"cart force={current_force:.3f} N"
        )
        builder.text(f"peak force: {peak_force:.3f} N")
        if result["optimized"]:
            builder.text(
                f"iters: {int(result.get('iters', 0))} | "
                f"loss={float(result.get('loss', 0.0)):.2e}"
            )
        else:
            builder.text(note)
        changed, stride_value = builder.slider(
            "Playback stride", float(playback["stride"]), 1.0, 8.0
        )
        if changed:
            playback["stride"] = max(1, int(round(stride_value)))
        if builder.button("Reset replay") and states:
            playhead["i"] = 0
            first_state = states[0]
            cart_frame.set_transform(_cart_transform(float(first_state[_CART_DOF])))
            pole_frame.set_transform(
                _pole_transform(float(first_state[_CART_DOF]), float(first_state[1]))
            )
        if cart_plot:
            builder.separator()
            builder.plot_lines("Cart x", cart_plot)
            builder.plot_lines("Target error", error_plot)
            builder.plot_lines("Pole angle", pole_plot)
            builder.plot_lines("Cart force", force_plot)

    return SceneSetup(
        world=render_world,
        pre_step=pre_step,
        panels=[ScenePanel("Diff Cartpole TrajOpt", build_panel)],
        info={
            "optimized": result["optimized"],
            "steps": len(states),
            "note": note,
            "target": _TARGET,
            "final_error": final_error,
        },
    )


SCENE = PythonDemoScene(
    id="diff_cartpole_trajopt",
    title="Differentiable Cartpole Trajectory Opt",
    category="Differentiable",
    summary=(
        "Gradient descent through World.get_step_derivatives optimizes a "
        "cart-force sequence to drive the cart to the yellow target. Blue "
        "breadcrumbs trace the optimized cart path; faint gray shows the "
        "un-optimized (zero-force) path that barely moves. Falls back to the "
        "uncontrolled rollout when the differentiable bindings are absent."
    ),
    build=build,
)
