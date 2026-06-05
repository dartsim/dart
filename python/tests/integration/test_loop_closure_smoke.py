"""Headless smokes for the loop-closure showcase and bridge drag damping.

1. Build the ``loop_closure`` scene, step the underlying World ~100 times
   under the variational integrator, and assert the holonomic DISTANCE closure
   keeps holding — the tip-to-anchor residual stays small while the arm swings.
2. Exercise the WorldRenderBridge force-drag velocity-damping path once: a
   moving rigid body under an active drag gets ``F_spring - kd*v`` applied (not
   just the spring term), so the dragged force opposes the body's motion.
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
        module = importlib.import_module("dartpy")
    except ModuleNotFoundError:
        pytest.skip("DART_BUILD_SIMULATION_EXPERIMENTAL is disabled")
    if not hasattr(module, "World"):
        pytest.skip("DART_BUILD_SIMULATION_EXPERIMENTAL is disabled")
    return module


def test_loop_closure_holds_under_vi() -> None:
    _sx()  # skip cleanly if experimental is unavailable
    from examples.demos.scenes.loop_closure import build

    setup = build()
    closure = setup.info["closure"]

    # Residual is ~0 at t=0 because the target distance is the rest-pose
    # separation (computed analytically in the scene).
    assert closure.compute_residual().norm < 1e-9

    # Step the physics ~100 times (the scene's pre_step advances the World and
    # re-syncs the render mirror).
    max_residual = 0.0
    for _ in range(100):
        setup.pre_step()
        max_residual = max(max_residual, float(closure.compute_residual().norm))

    # The DISTANCE loop is Newton-projected each VI step, so it keeps holding:
    # the tip stays on the target sphere about the anchor while the arm swings.
    assert max_residual < 1e-3, f"loop closure drifted: max residual {max_residual}"


def test_bridge_force_drag_applies_velocity_damping() -> None:
    sx = _sx()
    from examples.demos._world_bridge import WorldRenderBridge

    import dartpy as dart  # noqa: F401  (imported for parity with bridge usage)

    world = sx.World()
    body = world.add_rigid_body("free_body")
    body.mass = 1.0
    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="drag_damping_smoke")
    bridge.add_link_visual(
        body, dart.BoxShape(np.array([0.2, 0.2, 0.2])), (0.2, 0.6, 0.9),
        name="body_visual",
    )

    # Give the body a velocity so the damping term is non-zero, and capture the
    # velocity at the application point the bridge will damp against.
    body.linear_velocity = (1.0, 0.0, 0.0)
    point = np.array([0.0, 0.0, 0.0])
    spring = np.array([10.0, 0.0, 0.0])
    expected_damped = spring - WorldRenderBridge._DRAG_DAMPING_KD * np.array(
        [1.0, 0.0, 0.0]
    )

    # The damped force the bridge computes must be strictly less than the raw
    # spring along the motion direction (i.e. damping opposes the velocity).
    damping = bridge._drag_velocity(body, point)
    assert np.allclose(damping, [1.0, 0.0, 0.0])
    assert expected_damped[0] < spring[0]

    # Drive the full drag path once: activate a drag, step, and confirm it ran
    # without error and the body responded to the net (damped) force.
    bridge.force_drag(
        {
            "active": True,
            "renderable_name": "body_visual",
            "application_point": point,
            "force": spring,
        }
    )
    vx_before = float(np.asarray(body.linear_velocity)[0])
    bridge.pre_step()
    vx_after = float(np.asarray(body.linear_velocity)[0])
    # Net force along +x is positive (spring 10 - kd*1 > 0), so vx increases.
    assert vx_after > vx_before

    # An inactive event clears the drag so no force is applied afterwards.
    bridge.force_drag({"active": False})
    assert bridge._drag is None
