"""MPC-in-the-loop (minimal): LQR-stabilized cart-pole.

The infinite-horizon LQR is the degenerate (unconstrained, LTI, quadratic-cost)
form of MPC; the controller computes a torque each step from a hand-tuned gain
matrix applied to the cart-pole state. Swap the gain for a full MPC solver
(e.g. ``cvxpy``/``scipy``) without changing the scene shape.
"""

from __future__ import annotations

import numpy as np

from ..runner import PythonDemoScene, SceneSetup
from .cartpole_gym_env import _build_cart_pole_world

# Hand-tuned LQR-style gain for [cart_pos, cart_vel, pole_angle, pole_angvel].
# Sign convention: u = -K @ x stabilizes the upright pole.
_K = np.array([-1.0, -2.0, 30.0, 5.0])
_MAX_FORCE = 20.0


def build() -> SceneSetup:
    world, cart, pole = _build_cart_pole_world()

    def step(n: int) -> None:
        for _ in range(max(0, n)):
            state = np.array([
                cart.get_position(0),
                cart.get_velocity(0),
                pole.get_position(0),
                pole.get_velocity(0),
            ])
            force = float(np.clip(-_K @ state, -_MAX_FORCE, _MAX_FORCE))
            cart.set_force(0, force)
            world.step()

    return SceneSetup(world=world, step=step,
                      info={"controller": "LQR", "gains": _K.tolist()})


SCENE = PythonDemoScene(
    id="cartpole_mpc",
    title="Cart-pole MPC (LQR)",
    category="Control & Modern",
    summary="Cart-pole stabilized upright by an LQR (degenerate MPC) controller.",
    build=build,
)
