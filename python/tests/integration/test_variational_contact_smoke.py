"""Headless smoke for the variational-contact showcase.

Build the ``variational_contact`` scene and step the underlying World
under the variational integrator while the pendulum falls onto the ground plane.
Assert the compliant ground contact (configured through the World surface --
``Multibody.set_ground_contact`` / ``add_ground_contact_point``) is stable: every
VI step converges (``pre_step`` raises otherwise), the tip stays finite, swings
down to the plane, and is caught near it instead of tunneling through.
"""

from __future__ import annotations

import importlib
import pathlib
import sys

import numpy as np
import pytest

# Put python/ on sys.path so the demos package is importable.
_PYTHON_DIR = pathlib.Path(__file__).resolve().parents[2]
if str(_PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(_PYTHON_DIR))


def _sx():
    try:
        module = importlib.import_module("dartpy.simulation_experimental")
    except ModuleNotFoundError:
        pytest.skip("DART_BUILD_SIMULATION_EXPERIMENTAL is disabled")
    if not hasattr(module, "World"):
        pytest.skip("DART_BUILD_SIMULATION_EXPERIMENTAL is disabled")
    return module


def _tip_z(tip, tip_local) -> float:
    transform = tip.transform
    matrix = np.asarray(
        transform.matrix() if hasattr(transform, "matrix") else transform
    )
    world = matrix @ np.array([tip_local[0], tip_local[1], tip_local[2], 1.0])
    return float(world[2])


def test_variational_contact_catches_tip_on_ground() -> None:
    _sx()  # skip cleanly if experimental is unavailable
    from examples.demos.scenes.variational_contact import build

    setup = build()
    tip = setup.info["tip"]
    tip_local = setup.info["tip_local"]
    ground_z = float(setup.info["ground_z"])

    # pre_step advances the VI (raising on non-convergence) and re-syncs the
    # render mirror. 400 steps * 2 ms = 0.8 s: enough to fall and get caught.
    min_tip_z = _tip_z(tip, tip_local)
    for _ in range(400):
        setup.pre_step()
        z = _tip_z(tip, tip_local)
        assert np.isfinite(z)
        min_tip_z = min(min_tip_z, z)

    # The pendulum swung down to the ground plane...
    assert min_tip_z < ground_z + 0.05, (
        f"tip never reached the ground plane z={ground_z}: min {min_tip_z}"
    )
    # ...and compliant contact caught it -- the tip is held near the plane (a
    # bounded penalty penetration), not tunneling far below it.
    assert min_tip_z > ground_z - 0.15, (
        f"tip tunneled through the ground: min {min_tip_z}"
    )
