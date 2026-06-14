"""Tests for the dartpy PyTorch bridge for differentiable simulation.

Covers PLAN-110 WS3 first slice: the ``sx.diff`` torch.autograd bridge plus the
``get_step_derivatives`` / ``apply_step_vjp`` / ``state_vector`` bindings.

The import / ImportError-guard tests run in every configuration. The numeric
binding tests require differentiable support to be compiled (``DART_BUILD_DIFF``)
and are skipped otherwise. The torch gradient test runs only when torch is
importable in the environment.
"""

from __future__ import annotations

import importlib
import os
from pathlib import Path

import numpy as np
import pytest


def _read_cache_flag(flag: str) -> str | None:
    """Read a boolean CMake cache flag value, if a cache file is found."""
    build_type = (
        os.environ.get("BUILD_TYPE") or os.environ.get("CMAKE_BUILD_TYPE") or "Release"
    )
    repo_root = Path(__file__).resolve().parents[4]
    pixi_env = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    candidates: list[Path] = []
    if os.environ.get("CMAKE_BUILD_DIR"):
        candidates.append(Path(os.environ["CMAKE_BUILD_DIR"]) / "CMakeCache.txt")
    candidates.extend(
        [
            repo_root / "build" / pixi_env / "cpp" / build_type / "CMakeCache.txt",
            repo_root / "build" / pixi_env / "cpp" / "CMakeCache.txt",
        ]
    )
    for cache in candidates:
        if not cache.is_file():
            continue
        for line in cache.read_text(encoding="utf-8").splitlines():
            if line.startswith(f"{flag}:"):
                return line.rsplit("=", 1)[-1].strip().lower()
    return None


def _cache_reports_diff_disabled() -> bool:
    override = os.environ.get("DART_BUILD_DIFF_OVERRIDE")
    if override and override.lower() in {"0", "false", "no", "off"}:
        return True
    value = _read_cache_flag("DART_BUILD_DIFF")
    # Treat unknown as disabled: the default build resets DART_BUILD_DIFF=OFF.
    return value in {None, "0", "false", "no", "off"}


def _simulation():
    try:
        module = importlib.import_module("dartpy")
    except ModuleNotFoundError as exc:
        raise AssertionError(
            "dartpy should be available with the DART 7 World stack"
        ) from exc
    if not hasattr(module, "World"):
        raise AssertionError("dartpy imported but did not expose World")
    return module


def _torch_available() -> bool:
    return importlib.util.find_spec("torch") is not None


def _build_frictionless_sphere_scene(sx):
    """Sphere resting on a static ground, BoxedLcp + differentiable."""
    world = sx.World(
        time_step=1e-3,
        differentiable=True,
        contact_solver_method=sx.ContactSolverMethod.BOXED_LCP,
    )
    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -0.5))
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(np.array([5.0, 5.0, 0.5])))
    ground.friction = 0.0

    sphere = world.add_rigid_body("sphere", mass=2.0, position=(0.0, 0.0, 0.5 - 5e-5))
    sphere.set_collision_shape(sx.CollisionShape.sphere(0.5))
    sphere.friction = 0.0
    sphere.linear_velocity = np.array([0.0, 0.0, -0.2])
    return world


# ==============================================================================
# Import-safety tests: run in every configuration, including without torch.
# ==============================================================================
def test_diff_submodule_is_importable_without_torch():
    sx = _simulation()
    assert hasattr(sx, "diff"), "sx.diff should always exist as an attribute"
    assert hasattr(sx.diff, "timestep")


def test_diff_timestep_requires_torch_with_clear_message():
    sx = _simulation()
    if _torch_available():
        pytest.skip("torch is available; the ImportError guard is not exercised")
    with pytest.raises(ImportError, match="torch is required for sx.diff"):
        sx.diff.timestep(object(), object(), object())


def test_world_exposes_differentiable_construction_flags():
    sx = _simulation()
    assert hasattr(sx, "ContactSolverMethod")
    assert hasattr(sx, "StepDerivatives")
    assert hasattr(sx, "StepGradient")

    world = sx.World(
        time_step=2e-3,
        differentiable=True,
        contact_solver_method=sx.ContactSolverMethod.BOXED_LCP,
    )
    assert world.is_differentiable is True
    assert world.contact_solver_method == sx.ContactSolverMethod.BOXED_LCP

    default_world = sx.World()
    assert default_world.is_differentiable is False
    assert (
        default_world.contact_solver_method == sx.ContactSolverMethod.SEQUENTIAL_IMPULSE
    )


def test_contact_gradient_mode_reflects_through_world():
    sx = _simulation()
    assert hasattr(sx, "ContactGradientMode")

    # Default is ANALYTIC.
    default_world = sx.World()
    assert default_world.contact_gradient_mode == sx.ContactGradientMode.ANALYTIC

    for mode in (
        sx.ContactGradientMode.ANALYTIC,
        sx.ContactGradientMode.COMPLEMENTARITY_AWARE,
        sx.ContactGradientMode.PRE_CONTACT_SURROGATE,
    ):
        world = sx.World(
            differentiable=True,
            contact_solver_method=sx.ContactSolverMethod.BOXED_LCP,
            contact_gradient_mode=mode,
        )
        assert world.contact_gradient_mode == mode

    # The property is writable (mode only affects the backward pass).
    world = sx.World(
        differentiable=True,
        contact_solver_method=sx.ContactSolverMethod.BOXED_LCP,
    )
    world.contact_gradient_mode = sx.ContactGradientMode.COMPLEMENTARITY_AWARE
    assert world.contact_gradient_mode == sx.ContactGradientMode.COMPLEMENTARITY_AWARE


def _build_pre_contact_surrogate_scene(sx, mode):
    world = sx.World(
        time_step=1e-3,
        differentiable=True,
        contact_solver_method=sx.ContactSolverMethod.BOXED_LCP,
        contact_gradient_mode=mode,
    )
    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -0.5))
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(np.array([5.0, 5.0, 0.5])))
    ground.friction = 0.0

    sphere = world.add_rigid_body("sphere", mass=2.0, position=(0.0, 0.0, 1.0))
    sphere.set_collision_shape(sx.CollisionShape.sphere(0.5))
    sphere.friction = 0.0
    sphere.linear_velocity = np.array([0.0, 0.0, -0.5])
    return world


def test_pre_contact_surrogate_public_step_derivatives():
    sx = _simulation()
    if _cache_reports_diff_disabled():
        pytest.skip("DART_BUILD_DIFF is disabled")

    analytic_world = _build_pre_contact_surrogate_scene(
        sx, sx.ContactGradientMode.ANALYTIC
    )
    surrogate_world = _build_pre_contact_surrogate_scene(
        sx, sx.ContactGradientMode.PRE_CONTACT_SURROGATE
    )
    assert len(analytic_world.collide()) == 0
    assert len(surrogate_world.collide()) == 0

    analytic_world.step()
    surrogate_world.step()
    np.testing.assert_allclose(
        np.asarray(analytic_world.state_vector),
        np.asarray(surrogate_world.state_vector),
        atol=1e-12,
    )

    analytic_jac = np.asarray(analytic_world.get_step_derivatives().state_jacobian)
    surrogate_jac = np.asarray(surrogate_world.get_step_derivatives().state_jacobian)
    freefall = np.zeros((6, 6))
    freefall[:3, :3] = np.eye(3)
    freefall[:3, 3:] = 1e-3 * np.eye(3)
    freefall[3:, 3:] = np.eye(3)

    np.testing.assert_allclose(analytic_jac, freefall, atol=1e-9)
    assert np.max(np.abs(surrogate_jac - freefall)) > 1e-3
    assert surrogate_jac[5, 5] < 0.5
    assert surrogate_jac[3, 3] == pytest.approx(1.0, abs=1e-9)


def test_state_and_control_vectors_roundtrip():
    sx = _simulation()
    world = _build_frictionless_sphere_scene(sx)

    assert world.num_dofs == 3
    assert world.num_efforts == 3

    state = world.state_vector
    assert state.shape == (6,)
    np.testing.assert_allclose(state[:3], [0.0, 0.0, 0.5 - 5e-5])
    np.testing.assert_allclose(state[3:], [0.0, 0.0, -0.2])

    new_state = np.array([0.1, 0.2, 0.6, 1.0, -1.0, 0.5])
    world.state_vector = new_state
    np.testing.assert_allclose(world.state_vector, new_state)

    new_control = np.array([0.3, -0.4, 0.5])
    world.control_vector = new_control
    np.testing.assert_allclose(world.control_vector, new_control)


# ==============================================================================
# Numeric tests: require differentiable support to be compiled.
# ==============================================================================
def test_get_step_derivatives_and_vjp_match():
    sx = _simulation()
    if _cache_reports_diff_disabled():
        pytest.skip("DART_BUILD_DIFF is disabled")

    world = _build_frictionless_sphere_scene(sx)
    assert len(world.collide()) >= 1
    world.step()

    derivatives = world.get_step_derivatives()
    state_jac = np.asarray(derivatives.state_jacobian)
    control_jac = np.asarray(derivatives.control_jacobian)
    assert state_jac.shape == (6, 6)
    assert control_jac.shape == (6, 3)

    rng = np.random.RandomState(7)
    grad = rng.rand(6)
    result = world.apply_step_vjp(grad)
    np.testing.assert_allclose(np.asarray(result.state), state_jac.T @ grad, atol=1e-12)
    np.testing.assert_allclose(
        np.asarray(result.control), control_jac.T @ grad, atol=1e-12
    )


def test_get_step_derivatives_throws_when_not_differentiable():
    sx = _simulation()
    world = sx.World(time_step=1e-3, differentiable=False)
    with pytest.raises(Exception, match="differentiable"):
        world.get_step_derivatives()


def _build_freefall_sphere_scene(sx, mass):
    """Sphere in free fall (no contact), BoxedLcp + differentiable."""
    world = sx.World(
        time_step=1e-3,
        differentiable=True,
        contact_solver_method=sx.ContactSolverMethod.BOXED_LCP,
    )
    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -0.5))
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(np.array([5.0, 5.0, 0.5])))
    ground.friction = 0.0

    sphere = world.add_rigid_body("sphere", mass=mass, position=(0.0, 0.0, 1.0))
    sphere.set_collision_shape(sx.CollisionShape.sphere(0.5))
    sphere.friction = 0.0
    sphere.linear_velocity = np.array([0.1, -0.2, 0.3])
    sphere.force = np.array([0.5, -0.3, 0.7])
    return world


def test_add_differentiable_parameter_mass_jacobian():
    """The mass parameter Jacobian is exposed and matches FD-of-step.

    Torch-independent: drives get_step_derivatives() directly.
    """
    sx = _simulation()
    if _cache_reports_diff_disabled():
        pytest.skip("DART_BUILD_DIFF is disabled")

    dt = 1e-3
    mass = 2.0
    force = np.array([0.5, -0.3, 0.7])

    world = _build_freefall_sphere_scene(sx, mass)
    sphere = world.get_rigid_body("sphere")
    world.add_differentiable_parameter(sphere, sx.PhysicalParameter.MASS)
    assert world.num_differentiable_parameters == 1
    assert len(world.collide()) == 0  # free fall

    world.step()
    derivatives = world.get_step_derivatives()
    param_jac = np.asarray(derivatives.parameter_jacobian)
    assert param_jac.shape == (6, 1)

    # Closed form: only F/m depends on m, so dv'/dm = -dt F / m^2.
    d_vel = -dt * force / (mass * mass)
    expected = np.concatenate([dt * d_vel, d_vel])
    np.testing.assert_allclose(param_jac[:, 0], expected, atol=1e-6)


def test_parameter_jacobian_empty_without_registration():
    sx = _simulation()
    if _cache_reports_diff_disabled():
        pytest.skip("DART_BUILD_DIFF is disabled")

    world = _build_frictionless_sphere_scene(sx)
    assert world.num_differentiable_parameters == 0
    world.step()
    derivatives = world.get_step_derivatives()
    param_jac = np.asarray(derivatives.parameter_jacobian)
    assert param_jac.size == 0
    # State/control Jacobians are still produced.
    assert np.asarray(derivatives.state_jacobian).shape == (6, 6)


def test_add_differentiable_parameter_center_of_mass_throws():
    sx = _simulation()
    if _cache_reports_diff_disabled():
        pytest.skip("DART_BUILD_DIFF is disabled")

    world = _build_frictionless_sphere_scene(sx)
    sphere = world.get_rigid_body("sphere")
    # CENTER_OF_MASS is still reserved (the rigid-body step does not read the
    # body-local COM, so its Jacobian is identically zero); it must throw.
    with pytest.raises(Exception):
        world.add_differentiable_parameter(sphere, sx.PhysicalParameter.CENTER_OF_MASS)
    assert world.num_differentiable_parameters == 0
    # INERTIA and FRICTION are supported and register successfully.
    world.add_differentiable_parameter(sphere, sx.PhysicalParameter.INERTIA)
    world.add_differentiable_parameter(sphere, sx.PhysicalParameter.FRICTION)
    assert world.num_differentiable_parameters == 2


# ==============================================================================
# Framework-neutral (torch-free) rollout test: uses the numpy sx.diff.rollout,
# NOT sx.diff.timestep. Requires differentiable support to be compiled.
# ==============================================================================
def _freefall_initial_state_and_world(sx, mass=2.0):
    """Two free-falling spheres well above the ground (smooth, no contact)."""
    world = sx.World(
        time_step=1e-3,
        differentiable=True,
        contact_solver_method=sx.ContactSolverMethod.BOXED_LCP,
    )
    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -50.0))
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(np.array([5.0, 5.0, 0.5])))
    ground.friction = 0.0

    sphere = world.add_rigid_body("sphere", mass=mass, position=(0.0, 0.0, 5.0))
    sphere.set_collision_shape(sx.CollisionShape.sphere(0.5))
    sphere.friction = 0.0
    sphere.linear_velocity = np.array([0.1, -0.2, 0.3])
    return world


def test_rollout_states_shape_and_namespace():
    sx = _simulation()
    if _cache_reports_diff_disabled():
        pytest.skip("DART_BUILD_DIFF is disabled")

    assert hasattr(sx.diff, "rollout"), "sx.diff.rollout should exist when built"
    assert hasattr(sx.diff, "RolloutTrajectory")

    steps = 6
    world = _freefall_initial_state_and_world(sx)
    state_dim = 2 * world.num_dofs
    initial_state = np.asarray(world.state_vector)
    controls = np.zeros((steps, world.num_efforts))

    trajectory = sx.diff.rollout(world, initial_state, controls, steps)
    states = np.asarray(trajectory.states)
    assert states.shape == (steps + 1, state_dim)
    np.testing.assert_allclose(states[0], initial_state)
    assert trajectory.num_steps == steps


def test_rollout_vjp_matches_finite_difference():
    """Whole-rollout VJP {initial_state_grad, control_grads} == central FD.

    Torch-independent: drives sx.diff.rollout / trajectory.gradients directly.
    Loss = sum(final state). A fresh world is rebuilt per FD probe so each
    probe starts from an identical configuration.
    """
    sx = _simulation()
    if _cache_reports_diff_disabled():
        pytest.skip("DART_BUILD_DIFF is disabled")

    steps = 6

    def build():
        return _freefall_initial_state_and_world(sx)

    world0 = build()
    num_efforts = world0.num_efforts
    initial_state = np.asarray(world0.state_vector)
    assert len(world0.collide()) == 0  # smooth free-fall regime

    rng = np.random.RandomState(3)
    controls = 0.05 * rng.standard_normal((steps, num_efforts))

    def rollout_loss(state, ctrl):
        world = build()
        traj = sx.diff.rollout(world, state, ctrl, steps)
        return float(np.asarray(traj.states)[-1].sum())

    trajectory = sx.diff.rollout(build(), initial_state, controls, steps)
    final_state_grad = np.ones(2 * world0.num_dofs)
    initial_state_grad, control_grads = trajectory.gradients(final_state_grad)
    initial_state_grad = np.asarray(initial_state_grad)
    control_grads = np.asarray(control_grads)
    assert initial_state_grad.shape == (2 * world0.num_dofs,)
    assert control_grads.shape == (steps, num_efforts)

    worst_state_rel = 0.0
    worst_control_rel = 0.0
    for h in (1e-5, 1e-6, 1e-7):
        for i in range(initial_state.size):
            plus = initial_state.copy()
            minus = initial_state.copy()
            plus[i] += h
            minus[i] -= h
            fd = (rollout_loss(plus, controls) - rollout_loss(minus, controls)) / (
                2.0 * h
            )
            analytic = initial_state_grad[i]
            rel = abs(fd - analytic) / max(abs(analytic), 1e-9)
            worst_state_rel = max(worst_state_rel, rel)
            assert rel < 1e-4, f"initial_state_grad[{i}] h={h} fd={fd} an={analytic}"

        for t in range(steps):
            for j in range(num_efforts):
                plus = controls.copy()
                minus = controls.copy()
                plus[t, j] += h
                minus[t, j] -= h
                fd = (
                    rollout_loss(initial_state, plus)
                    - rollout_loss(initial_state, minus)
                ) / (2.0 * h)
                analytic = control_grads[t, j]
                rel = abs(fd - analytic) / max(abs(analytic), 1e-9)
                worst_control_rel = max(worst_control_rel, rel)
                assert (
                    rel < 1e-4
                ), f"control_grads[{t}][{j}] h={h} fd={fd} an={analytic}"

    print(
        f"[rollout-FD] worst initial_state_grad rel={worst_state_rel:.3e} "
        f"worst control_grads rel={worst_control_rel:.3e}"
    )


# ==============================================================================
# Torch gradient test: runs only when torch is importable in the environment.
# ==============================================================================
def test_diff_timestep_autograd_matches_jacobians():
    sx = _simulation()
    if not _torch_available():
        pytest.skip("torch is not installed in this environment")
    if _cache_reports_diff_disabled():
        pytest.skip("DART_BUILD_DIFF is disabled")

    import torch

    world = _build_frictionless_sphere_scene(sx)

    state = torch.tensor(world.state_vector, dtype=torch.float64, requires_grad=True)
    action = torch.zeros(world.num_efforts, dtype=torch.float64, requires_grad=True)

    next_state = sx.diff.timestep(world, state, action)
    loss = next_state.sum()
    loss.backward()

    # The analytic step Jacobians for the stepped configuration.
    derivatives = world.get_step_derivatives()
    state_jac = np.asarray(derivatives.state_jacobian)
    control_jac = np.asarray(derivatives.control_jacobian)

    # d(loss)/d(next_state) = ones, so grads are Jᵀ · 1 = column sums of J.
    ones = np.ones(state_jac.shape[0])
    np.testing.assert_allclose(
        state.grad.detach().cpu().numpy(), state_jac.T @ ones, atol=1e-4
    )
    np.testing.assert_allclose(
        action.grad.detach().cpu().numpy(), control_jac.T @ ones, atol=1e-4
    )
