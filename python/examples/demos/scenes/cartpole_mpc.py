"""MPC-in-the-loop (minimal): LQR-stabilized cart-pole.

The infinite-horizon LQR is the degenerate (unconstrained, LTI, quadratic-cost)
form of MPC; the controller computes a torque each step from a hand-tuned gain
matrix applied to the cart-pole state. Swap the gain for a full MPC solver
(e.g. ``cvxpy``/``scipy``) without changing the scene shape.
"""

from __future__ import annotations

from collections import deque

import numpy as np

from ..runner import PythonDemoScene, ScenePanel, SceneSetup
from .cartpole_gym_env import _build_cart_pole_world

# Hand-tuned LQR-style gain for [cart_pos, cart_vel, pole_angle, pole_angvel].
# Sign convention: u = -K @ x stabilizes the upright pole.
_K = np.array([-1.0, -2.0, 30.0, 5.0])
_MAX_FORCE = 20.0


def build() -> SceneSetup:
    world, cart, pole = _build_cart_pole_world()
    last_force = {"value": 0.0}
    cart_history = deque(maxlen=120)
    angle_history = deque(maxlen=120)
    force_history = deque(maxlen=120)

    def state() -> np.ndarray:
        return np.array([
            cart.get_position(0),
            cart.get_velocity(0),
            pole.get_position(0),
            pole.get_velocity(0),
        ])

    def pre_step() -> None:
        command = float(np.clip(-_K @ state(), -_MAX_FORCE, _MAX_FORCE))
        last_force["value"] = command
        cart.set_force(0, command)

    def build_panel(builder: object, context: object) -> None:
        current = state()
        cart_history.append(float(current[0]))
        angle_history.append(float(current[2]))
        force_history.append(float(last_force["value"]))
        builder.text("controller: LQR as MPC baseline")
        builder.text(f"force limit: +/-{_MAX_FORCE:.1f} N")
        builder.text(f"command force: {last_force['value']:.3f} N")
        builder.text(f"cart x: {current[0]:.3f} m | v: {current[1]:.3f} m/s")
        builder.text(f"pole angle: {current[2]:.3f} rad | w: {current[3]:.3f} rad/s")
        builder.separator()
        builder.plot_lines("Cart x", list(cart_history))
        builder.plot_lines("Pole angle", list(angle_history))
        builder.plot_lines("Force", list(force_history))

    return SceneSetup(
        world=world,
        pre_step=pre_step,
        panels=[ScenePanel("Cart-pole MPC", build_panel)],
        info={"controller": "LQR", "gains": _K.tolist()},
    )


SCENE = PythonDemoScene(
    id="cartpole_mpc",
    title="Cart-pole MPC (LQR)",
    category="Control & Modern",
    summary="Cart-pole stabilized upright by an LQR (degenerate MPC) controller.",
    build=build,
)
