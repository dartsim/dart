"""RL gym-style wrapper (minimal): a cart-pole environment.

Exposes a Gymnasium-style ``CartPoleEnv`` with ``reset()`` / ``step(action)`` /
``action_space`` / ``observation_space`` over a cart + pole built on DART's
World. The class is intentionally dependency-free (no ``gymnasium`` import) so
the demos surface stays light; users plug it into their own training loop.
The scene's ``info["env"]`` is the env instance so tests and notebooks can drive
it without rebuilding the world.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup

_CART_EXTENT = (0.4, 0.2, 0.15)
_POLE_EXTENT = (0.04, 0.04, 0.6)
_MAX_FORCE = 10.0


def _build_cart_pole_world() -> tuple[dart.World, Any, Any]:
    """Return (world, cart_joint, pole_joint)."""
    world = dart.World("cartpole")
    world.set_gravity([0.0, 0.0, -9.81])

    skel = dart.Skeleton("cartpole")
    cart_joint, cart_body = skel.create_prismatic_joint_and_body_node_pair()
    cart_joint.set_axis(np.array([1.0, 0.0, 0.0]))
    cart_shape = dart.BoxShape(np.array(_CART_EXTENT))
    cn = cart_body.create_shape_node(cart_shape)
    cn.create_visual_aspect().set_color([0.30, 0.50, 0.80])
    cn.create_collision_aspect()
    cn.create_dynamics_aspect()
    cart_body.set_inertia(dart.Inertia(
        1.0,
        np.zeros(3),
        dart.BoxShape.compute_inertia_of(np.array(_CART_EXTENT), 1.0),
    ))

    pole_joint, pole_body = skel.create_revolute_joint_and_body_node_pair(
        cart_body)
    pole_joint.set_axis(np.array([0.0, 1.0, 0.0]))
    pole_joint.set_position(0, 0.05)  # small tilt
    transform = np.eye(4)
    transform[:3, 3] = (0.0, 0.0, _POLE_EXTENT[2] * 0.5)
    pole_joint.set_transform_from_parent_body_node(transform)
    pole_shape = dart.BoxShape(np.array(_POLE_EXTENT))
    pn = pole_body.create_shape_node(pole_shape)
    pn.create_visual_aspect().set_color([0.90, 0.65, 0.30])
    pn.create_collision_aspect()
    pn.create_dynamics_aspect()
    pole_body.set_inertia(dart.Inertia(
        0.1,
        np.zeros(3),
        dart.BoxShape.compute_inertia_of(np.array(_POLE_EXTENT), 0.1),
    ))

    world.add_skeleton(skel)
    return world, cart_joint, pole_joint


@dataclass
class _Space:
    shape: tuple[int, ...]
    low: float
    high: float


class CartPoleEnv:
    """A minimal Gymnasium-style env wrapping a DART cart-pole world."""

    def __init__(self) -> None:
        self.world, self.cart, self.pole = _build_cart_pole_world()
        self.action_space = _Space(shape=(1,), low=-_MAX_FORCE, high=_MAX_FORCE)
        self.observation_space = _Space(shape=(4,), low=-math.inf, high=math.inf)
        self._terminal_angle = 0.4  # ~23 degrees

    def reset(self) -> np.ndarray:
        self.cart.set_position(0, 0.0)
        self.cart.set_velocity(0, 0.0)
        self.pole.set_position(0, 0.05)
        self.pole.set_velocity(0, 0.0)
        self.world.set_time(0.0)
        return self._observation()

    def step(self, action: float) -> tuple[np.ndarray, float, bool, dict]:
        force = float(np.clip(action, -_MAX_FORCE, _MAX_FORCE))
        self.cart.set_force(0, force)
        self.world.step()
        obs = self._observation()
        terminated = bool(abs(obs[2]) > self._terminal_angle)
        reward = 0.0 if terminated else 1.0
        return obs, reward, terminated, {}

    def _observation(self) -> np.ndarray:
        return np.array([
            self.cart.get_position(0),
            self.cart.get_velocity(0),
            self.pole.get_position(0),
            self.pole.get_velocity(0),
        ])


def build() -> SceneSetup:
    env = CartPoleEnv()
    env.reset()

    def step(n: int) -> None:
        for _ in range(max(0, n)):
            env.step(0.0)  # passive: no control input

    return SceneSetup(world=env.world, step=step,
                      info={"env": env, "controller": "passive"})


SCENE = PythonDemoScene(
    id="cartpole_gym_env",
    title="Cart-pole RL Env",
    category="Control & Modern",
    summary="Gymnasium-style env over a DART cart-pole (passive step).",
    build=build,
)
