"""Plane lock for the ports of the two-dimensional reference demos.

The ``avbd-demo2d`` reference solver is two-dimensional: a body has a
position in the plane and one rotation angle. The ports build unit-thickness
boxes in the three-dimensional world, and nothing in that world knows about
the plane: the contact manifolds between two boxes are not exactly symmetric
in ``z``, so stacked and resting bodies slowly tilt, then tumble out of the
plane after a second or two. The lock removes the out-of-plane state of every
dynamic body before each step (``z`` translation and velocity, the tilt of the
orientation, and the ``x``/``y`` angular velocity) and keeps the in-plane
rotation angle, which is exactly the state the reference solver integrates.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Iterable
from typing import Any

import numpy as np


def lock_to_xy_plane(bodies: Iterable[Any]) -> int:
    """Project the dynamic ``bodies`` onto the XY plane in place.

    Static bodies are left alone. A body is only written back when one of its
    out-of-plane components is nonzero, so a body that already lies in the
    plane is not touched. Returns the number of bodies that were written.
    """

    written = 0
    for body in bodies:
        if body.is_static:
            continue
        transform = np.array(body.transform, dtype=float).reshape(4, 4)
        rotation = transform[:3, :3]
        angle = math.atan2(rotation[1, 0], rotation[0, 0])
        cosine = math.cos(angle)
        sine = math.sin(angle)
        planar = np.eye(4)
        planar[0, 0] = cosine
        planar[0, 1] = -sine
        planar[1, 0] = sine
        planar[1, 1] = cosine
        planar[0, 3] = transform[0, 3]
        planar[1, 3] = transform[1, 3]
        changed = False
        if not np.array_equal(planar, transform):
            body.transform = planar
            changed = True
        linear = np.array(body.linear_velocity, dtype=float).reshape(3)
        if linear[2] != 0.0:
            linear[2] = 0.0
            body.linear_velocity = linear
            changed = True
        angular = np.array(body.angular_velocity, dtype=float).reshape(3)
        if angular[0] != 0.0 or angular[1] != 0.0:
            angular[0] = 0.0
            angular[1] = 0.0
            body.angular_velocity = angular
            changed = True
        if changed:
            written += 1
    return written


def plane_locked_pre_step(bridge: Any, bodies: Iterable[Any]) -> Callable[[], None]:
    """Return a scene ``pre_step`` that locks the ``bodies`` to the plane and
    then steps through ``bridge.pre_step``.

    The returned callable keeps the bridge in its closure, so the shared replay
    controls find the bridge's ``sync`` the way they do for a bare
    ``bridge.pre_step``. The lock is a pure projection of the current state, so
    scenes that use it declare ``replay_live_step_is_stateless``.
    """

    locked_bodies = tuple(bodies)

    def locked_pre_step() -> None:
        lock_to_xy_plane(locked_bodies)
        bridge.pre_step()

    return locked_pre_step
