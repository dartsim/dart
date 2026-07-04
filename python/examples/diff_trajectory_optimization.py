#!/usr/bin/env python3
"""Differentiable trajectory optimization: steer a projectile to a target.

A standalone, torch-free worked example for DART's opt-in differentiable
simulation (PLAN-110). A single rigid body starts at rest above a distant ground
plane. The optimizer uses ``sx.diff.rollout`` and the rollout VJP returned by
``RolloutTrajectory.gradients`` to tune a per-step force sequence so the final
position reaches a fixed target.

The setup deliberately stays contact-free over the horizon, so the optimization
exercises the smooth differentiable path and the control Jacobian is exact. This
is the headless counterpart to the interactive differentiable trajectory scenes
under ``python/examples/demos/scenes/``.

Run it directly -- this is a standalone program, not a ``-m examples.demos`` GUI
scene::

    PYTHONPATH=build/default/cpp/Release/python:python \\
        .pixi/envs/default/bin/python python/examples/diff_trajectory_optimization.py

Exit-code contract: the script exits 0 in exactly two cases -- (1) the expected
DART 7 ``dartpy`` imports successfully but differentiable support is not compiled
(``DART_BUILD_DIFF=OFF``, the default pixi build), where it prints a clear
message instead of raising; and (2) a successful trajectory optimization on a
``DART_BUILD_DIFF=ON`` build. Every other outcome exits nonzero: a ``dartpy``
import/setup failure, a wrong or stale ``dartpy`` that lacks the DART 7
World/differentiable surface, a failure of the differentiable API, a non-finite
result, or non-convergence.
"""

from __future__ import annotations

import sys
from typing import Any

import numpy as np

# --- Problem setup ----------------------------------------------------------
TIME_STEP = 1e-2
NUM_STEPS = 60
GRAVITY = (0.0, 0.0, -9.81)
PROJECTILE_MASS = 1.0
PROJECTILE_RADIUS = 0.2
START_POSITION = (0.0, 0.0, 5.0)
TARGET_POSITION = np.array([3.0, -2.0, 4.0], dtype=float)

# --- Optimizer --------------------------------------------------------------
LEARNING_RATE = 1000.0
MAX_ITERS = 80
LOSS_TOL = 1e-8
ACCEPT_DISTANCE = 2e-2

STATE_SIZE = 2 * 3  # [position(3); linear velocity(3)] for one rigid body


def _build_world(sx: Any) -> Any:
    """Build a fresh differentiable World for the projectile task."""

    world = sx.World(
        time_step=TIME_STEP,
        gravity=GRAVITY,
        differentiable=True,
        contact_solver_method=sx.ContactSolverMethod.BOXED_LCP,
    )

    # Static ground far below the projectile. It gives the scene ordinary
    # collision content while keeping the optimized horizon contact-free.
    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -100.0))
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(np.array([50.0, 50.0, 0.5])))
    ground.friction = 0.0

    projectile = world.add_rigid_body(
        "projectile", mass=PROJECTILE_MASS, position=START_POSITION
    )
    projectile.set_collision_shape(sx.CollisionShape.sphere(PROJECTILE_RADIUS))
    projectile.friction = 0.0
    return world


def _diff_built(sx: Any) -> bool:
    """Whether differentiable rollout support is compiled in."""

    diff = getattr(sx, "diff", None)
    return diff is not None and hasattr(diff, "rollout")


_REQUIRED_DART7_SYMBOLS = ("World", "diff", "CollisionShape", "ContactSolverMethod")


def _diff_off_is_genuine(sx: Any) -> tuple[bool, str]:
    """Confirm absent diff rollout bindings mean a genuine DIFF-off build."""

    missing = [name for name in _REQUIRED_DART7_SYMBOLS if not hasattr(sx, name)]
    if missing:
        return False, f"imported dartpy is missing DART 7 symbols {missing}"
    try:
        world = _build_world(sx)
        world.step()
        world.get_step_derivatives()
    except Exception as exc:  # noqa: BLE001 - classify the expected OFF signal.
        message = str(exc)
        if "DART_BUILD_DIFF" in message or "differentiable support" in message:
            return True, ""
        return False, f"differentiable API failed unexpectedly: {exc}"
    return False, "differentiable API returned derivatives on an OFF build"


def _loss_and_grad(sx: Any, controls: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    """Run one rollout and return (loss, final position, dL/dcontrols)."""

    world = _build_world(sx)
    initial_state = np.asarray(world.state_vector, dtype=float)
    if initial_state.shape != (STATE_SIZE,):
        raise ValueError(
            f"expected projectile state shape {(STATE_SIZE,)}, got {initial_state.shape}"
        )
    if controls.shape != (NUM_STEPS, int(world.num_efforts)):
        raise ValueError(
            "control sequence has shape "
            f"{controls.shape}, expected {(NUM_STEPS, int(world.num_efforts))}"
        )

    trajectory = sx.diff.rollout(world, initial_state, controls, NUM_STEPS)
    states = np.asarray(trajectory.states, dtype=float)
    if states.shape != (NUM_STEPS + 1, STATE_SIZE):
        raise ValueError(
            f"rollout returned states with shape {states.shape}, "
            f"expected {(NUM_STEPS + 1, STATE_SIZE)}"
        )

    final_position = states[-1, :3]
    residual = final_position - TARGET_POSITION
    loss = 0.5 * float(residual @ residual)

    final_state_grad = np.zeros(STATE_SIZE, dtype=float)
    final_state_grad[:3] = residual
    _initial_state_grad, control_grad = trajectory.gradients(final_state_grad)
    control_grad = np.asarray(control_grad, dtype=float)
    if control_grad.shape != controls.shape:
        raise ValueError(
            f"rollout returned control gradient shape {control_grad.shape}, "
            f"expected {controls.shape}"
        )
    return loss, final_position, control_grad


def _optimize_controls(sx: Any) -> dict[str, Any]:
    """Gradient-descend the per-step force sequence."""

    probe_world = _build_world(sx)
    efforts = int(probe_world.num_efforts)
    if efforts != 3:
        raise ValueError(f"expected one projectile with 3 efforts, got {efforts}")

    controls = np.zeros((NUM_STEPS, efforts), dtype=float)
    loss = float("inf")
    final_position = np.zeros(3, dtype=float)

    print("target position:", _format_vec(TARGET_POSITION), "m")
    print(f"{'iter':>4}  {'loss':>13}  {'distance [m]':>13}  {'max |force|':>13}")

    for iters in range(1, MAX_ITERS + 1):
        loss, final_position, control_grad = _loss_and_grad(sx, controls)
        distance = float(np.linalg.norm(final_position - TARGET_POSITION))
        max_force = float(np.max(np.abs(controls)))
        print(f"{iters:>4}  {loss:>13.5e}  {distance:>13.5e}  {max_force:>13.5e}")

        if not np.isfinite(loss) or not np.all(np.isfinite(control_grad)):
            raise FloatingPointError("trajectory optimization produced non-finite values")
        if loss < LOSS_TOL:
            break
        controls = controls - LEARNING_RATE * control_grad

    return {
        "controls": controls,
        "final_position": final_position,
        "loss": loss,
        "iters": iters,
    }


def _format_vec(values: np.ndarray) -> str:
    return "(" + ", ".join(f"{float(value):.4f}" for value in values) + ")"


def main() -> int:
    try:
        import dartpy as sx
    except Exception as exc:  # noqa: BLE001 - a dartpy import/setup failure is real.
        print(f"[error] dartpy could not be imported: {exc}")
        print(
            "  Set PYTHONPATH to the built dartpy; the example cannot run "
            "without it."
        )
        return 1

    if not _diff_built(sx):
        genuine, reason = _diff_off_is_genuine(sx)
        if not genuine:
            print(
                "[error] differentiable rollout bindings are absent and this is "
                "not a valid DART_BUILD_DIFF=OFF dartpy build."
            )
            print(f"  reason: {reason}")
            print(
                "  Point PYTHONPATH at the freshly built dartpy; a stale or "
                "partial dartpy may be shadowing the build tree."
            )
            return 1
        print(
            "[diff-unavailable] differentiable simulation is not compiled into "
            "this build."
        )
        print(
            "  Rebuild with the DART_BUILD_DIFF CMake option ON to run the "
            "trajectory optimization."
        )
        print("  (The default pixi build ships with DART_BUILD_DIFF=OFF.)")
        return 0

    print(
        "differentiable simulation ENABLED - optimizing a force trajectory\n"
    )
    try:
        result = _optimize_controls(sx)
    except Exception as exc:  # noqa: BLE001 - surface DIFF-on regressions loudly.
        print(
            "[error] the differentiable run failed on a DART_BUILD_DIFF=ON "
            f"build: {exc}"
        )
        return 1

    final_position = np.asarray(result["final_position"], dtype=float)
    if not np.all(np.isfinite(final_position)):
        print(
            "[error] the differentiable run produced a non-finite final position "
            f"({_format_vec(final_position)}) on a DART_BUILD_DIFF=ON build."
        )
        return 1

    distance = float(np.linalg.norm(final_position - TARGET_POSITION))
    max_force = float(np.max(np.abs(np.asarray(result["controls"], dtype=float))))
    print()
    print(
        f"final position: {_format_vec(final_position)} m "
        f"(target {_format_vec(TARGET_POSITION)} m)"
    )
    print(
        f"converged in {result['iters']} iters, final loss "
        f"{result['loss']:.5e}, max |force| {max_force:.3f} N"
    )
    if distance < ACCEPT_DISTANCE:
        print(
            f"PASS: reached the target within {ACCEPT_DISTANCE:.2e} m "
            "using rollout control gradients."
        )
        return 0
    print(
        f"FAIL: final distance {distance:.5e} m exceeds "
        f"{ACCEPT_DISTANCE:.2e} m."
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
