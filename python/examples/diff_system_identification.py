#!/usr/bin/env python3
"""Differentiable system identification: recover an unknown rigid-body mass.

A standalone, torch-free worked example for DART's opt-in differentiable
simulation (PLAN-110). A single rigid body is pushed by a *known* constant
force; from a short observed trajectory we recover its *unknown* mass by plain
gradient descent on the per-step mass parameter Jacobian returned by
``World.get_step_derivatives()``.

The setup is deliberately the simplest one that isolates the mass signal: zero
gravity and a constant applied force, so the whole trajectory depends on the
mass only through ``F / m`` and the smooth (contact-free) differentiable step is
exact. The analytic mass gradient matches the closed form ``dv'/dm = -dt F / m^2``
(see ``python/tests/unit/simulation/test_diff.py``).

Run it directly -- this is a standalone program, not a ``-m examples.demos`` GUI
scene::

    PYTHONPATH=build/default/cpp/Release/python:python \\
        .pixi/envs/default/bin/python python/examples/diff_system_identification.py

Graceful degradation: differentiable support is an opt-in build flag
(``DART_BUILD_DIFF``), and the default pixi build ships with it OFF. When it is
unavailable the script prints a clear message and exits 0 instead of raising.
"""

from __future__ import annotations

import sys
from typing import Any

import numpy as np

# --- Problem setup ----------------------------------------------------------
TIME_STEP = 1e-2
NUM_STEPS = 40
GRAVITY = (0.0, 0.0, 0.0)  # forced (not free-falling): isolate the mass signal
APPLIED_FORCE = (4.0, -3.0, 2.0)  # known constant force [N]
INITIAL_VELOCITY = (0.1, -0.2, 0.3)
TRUE_MASS = 2.0  # the planted value the optimizer must recover [kg]
INITIAL_MASS_GUESS = 5.0  # a deliberately wrong starting point [kg]

# --- Optimizer --------------------------------------------------------------
LEARNING_RATE = 0.2  # plain gradient descent step (tuned for this setup)
MAX_ITERS = 200
LOSS_TOL = 1e-12  # stop when the trajectory-matching loss is negligible
MASS_STEP_TOL = 1e-9  # stop when the mass update stalls
ACCEPT_REL_TOL = 1e-3  # success bar for the recovered mass

STATE_SIZE = 2 * 3  # [position(3); linear velocity(3)] for one rigid body


def _build_world(sx: Any, mass: float) -> tuple[Any, Any]:
    """Build a fresh differentiable World holding one forced rigid body.

    A single dynamic sphere of ``mass`` starts at a fixed pose and velocity and
    is pushed by ``APPLIED_FORCE`` each step. A static ground sits far below so
    the short rollout stays contact-free and exercises the smooth differentiable
    path. Returns ``(world, body)``.
    """
    world = sx.World(
        time_step=TIME_STEP,
        gravity=GRAVITY,
        differentiable=True,
        contact_solver_method=sx.ContactSolverMethod.BOXED_LCP,
    )

    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -50.0))
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(np.array([5.0, 5.0, 0.5])))
    ground.friction = 0.0

    body = world.add_rigid_body("body", mass=mass, position=(0.0, 0.0, 5.0))
    body.set_collision_shape(sx.CollisionShape.sphere(0.5))
    body.friction = 0.0
    body.linear_velocity = np.array(INITIAL_VELOCITY, dtype=float)
    return world, body


def _rollout(world: Any, body: Any, want_derivatives: bool) -> tuple[
    list[np.ndarray], list[Any]
]:
    """Roll the forced body out for ``NUM_STEPS``, recording each state.

    When ``want_derivatives`` is set, also record the per-step
    ``StepDerivatives`` (state and parameter Jacobians) used by the reverse pass.
    """
    states = [np.asarray(world.state_vector, dtype=float)]
    derivatives: list[Any] = []
    force = np.asarray(APPLIED_FORCE, dtype=float)
    for _ in range(NUM_STEPS):
        body.force = force
        world.step()
        states.append(np.asarray(world.state_vector, dtype=float))
        if want_derivatives:
            derivatives.append(world.get_step_derivatives())
    return states, derivatives


def _diff_available(sx: Any) -> tuple[bool, str]:
    """Detect whether differentiable simulation is compiled in.

    Mirrors the cartpole-scene detection: ``sx.diff.rollout`` only exists in a
    ``DART_BUILD_DIFF=ON`` build, and the parameter/Jacobian calls throw when
    diff support is not compiled. ``is_differentiable`` is *not* a reliable
    signal -- it reflects the requested opt-in even when support is absent -- so
    we actually exercise the diff calls inside a guard.
    """
    diff = getattr(sx, "diff", None)
    if diff is None or not hasattr(diff, "rollout"):
        return False, "sx.diff differentiable bindings are not built"
    try:
        world, body = _build_world(sx, TRUE_MASS)
        world.add_differentiable_parameter(body, sx.PhysicalParameter.MASS)
        world.step()
        jacobian = np.asarray(world.get_step_derivatives().parameter_jacobian)
    except Exception as exc:  # noqa: BLE001 - any diff failure -> graceful exit
        return False, f"differentiable step derivatives unavailable: {exc}"
    if jacobian.shape != (STATE_SIZE, 1):
        return False, f"unexpected parameter_jacobian shape {jacobian.shape}"
    return True, ""


def _identify_mass(sx: Any) -> dict[str, Any]:
    """Recover the planted mass by gradient descent on the parameter Jacobian."""
    # Ground-truth trajectory generated with the true (unknown to the optimizer)
    # mass. Both rollouts start from the identical construction state, so the
    # only thing that differs is the mass we are solving for.
    gt_world, gt_body = _build_world(sx, TRUE_MASS)
    observed, _ = _rollout(gt_world, gt_body, want_derivatives=False)

    mass = INITIAL_MASS_GUESS
    loss = float("inf")
    iters = 0
    print(f"true mass (planted): {TRUE_MASS:.6f} kg")
    print(f"initial guess:       {mass:.6f} kg")
    print(f"{'iter':>4}  {'loss':>13}  {'mass [kg]':>12}  {'dL/dm':>13}")

    for iters in range(1, MAX_ITERS + 1):
        # Fresh world each iteration so the rollout restarts from the same state.
        world, body = _build_world(sx, mass)
        world.add_differentiable_parameter(body, sx.PhysicalParameter.MASS)
        predicted, derivatives = _rollout(world, body, want_derivatives=True)

        residuals = [predicted[t] - observed[t] for t in range(NUM_STEPS + 1)]
        loss = 0.5 * float(sum(r @ r for r in residuals))

        # Adjoint reverse pass over the chained per-step Jacobians (mirrors
        # diff_cartpole_trajopt._optimize_cartpole). The mass enters every step,
        # so dL/dm accumulates parameter_jacobian_t^T * adjoint_{t+1} across the
        # whole rollout while the adjoint is propagated by state_jacobian^T.
        adjoint = residuals[NUM_STEPS].copy()
        grad = 0.0
        for t in range(NUM_STEPS - 1, -1, -1):
            derivative = derivatives[t]
            parameter_jacobian = np.asarray(
                derivative.parameter_jacobian, dtype=float
            )
            state_jacobian = np.asarray(derivative.state_jacobian, dtype=float)
            grad += float(parameter_jacobian[:, 0] @ adjoint)
            adjoint = state_jacobian.T @ adjoint
            if t >= 1:
                adjoint = adjoint + residuals[t]

        print(f"{iters:>4}  {loss:>13.5e}  {mass:>12.6f}  {grad:>13.5e}")
        if loss < LOSS_TOL:
            break
        new_mass = max(mass - LEARNING_RATE * grad, 1e-3)  # keep mass physical
        if abs(new_mass - mass) < MASS_STEP_TOL:
            mass = new_mass
            break
        mass = new_mass

    return {"mass": mass, "loss": loss, "iters": iters}


def main() -> int:
    try:
        import dartpy as sx
    except Exception as exc:  # noqa: BLE001 - missing dartpy -> graceful exit
        print(f"[diff-unavailable] dartpy could not be imported: {exc}")
        print("  Set PYTHONPATH to the built dartpy and retry. Exiting 0.")
        return 0

    available, message = _diff_available(sx)
    if not available:
        print(
            "[diff-unavailable] differentiable simulation is not enabled in "
            "this build."
        )
        print(f"  reason: {message}")
        print(
            "  Rebuild with the DART_BUILD_DIFF CMake option ON to run the "
            "system-identification optimization."
        )
        print("  (The default pixi build ships with DART_BUILD_DIFF=OFF.)")
        return 0

    print(
        "differentiable simulation ENABLED - recovering mass by gradient "
        "descent\n"
    )
    result = _identify_mass(sx)
    recovered = float(result["mass"])
    rel_err = abs(recovered - TRUE_MASS) / TRUE_MASS
    print()
    print(
        f"recovered mass: {recovered:.6f} kg "
        f"(true {TRUE_MASS:.6f} kg, relative error {rel_err:.2e})"
    )
    print(
        f"converged in {result['iters']} iters, "
        f"final loss {result['loss']:.5e}"
    )
    if rel_err < ACCEPT_REL_TOL:
        print(
            f"PASS: recovered the planted mass within {ACCEPT_REL_TOL:.0e} "
            "relative tolerance."
        )
        return 0
    print(f"WARN: did not reach {ACCEPT_REL_TOL:.0e} relative tolerance.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
