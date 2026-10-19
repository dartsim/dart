"""The Python posture objective preserves native residuals and IK callbacks."""

import gc
import sys
import weakref
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from examples.demos.models.relaxed_posture import RelaxedPosture


def test_relaxed_posture_original_bounded_and_ideal_residuals():
    posture = RelaxedPosture(
        [0.0, 0.5, -0.25], [-1.0, -1.0, -np.inf], [1.0, 1.0, np.inf], [2.0, 3.0, 4.0]
    )
    assert isinstance(posture, dart.optimizer.Function)
    assert posture.eval([-2.0, 2.0, 0.0]) == 6.5
    np.testing.assert_array_equal(
        posture.eval_gradient([-2.0, 2.0, 0.0]), [-2.0, 3.0, 0.0]
    )
    assert posture.eval([0.0, 0.5, -0.25]) == 0.0
    np.testing.assert_array_equal(posture.eval_gradient([0.0, 0.5, -0.25]), np.zeros(3))
    posture.enforce_ideal_posture = True
    assert posture.eval([-2.0, 2.0, 0.0]) == 18.625
    np.testing.assert_array_equal(
        posture.eval_gradient([-2.0, 2.0, 0.0]), [-4.0, 4.5, 1.0]
    )


@pytest.mark.parametrize("input_size,gradient_size", [(1, 5), (5, 1), (0, 5), (5, 0)])
def test_relaxed_posture_callback_truncates_and_zero_fills(input_size, gradient_size):
    posture = RelaxedPosture(np.zeros(3), np.zeros(3), np.zeros(3), np.ones(3))
    x = np.full(input_size, 2.0)
    gradient = np.full(gradient_size, 9.0)
    posture.evalGradient(x, gradient)
    expected = np.zeros(gradient_size)
    expected[: min(input_size, gradient_size, 3)] = 2.0
    np.testing.assert_array_equal(gradient, expected)
    np.testing.assert_array_equal(
        posture._residual(x),
        np.r_[np.full(min(input_size, 3), 2.0), np.zeros(max(0, 3 - input_size))],
    )


def test_relaxed_posture_owns_inputs_and_gradient_results():
    inputs = [np.zeros(2), np.full(2, -1.0), np.ones(2), np.full(2, 2.0)]
    posture = RelaxedPosture(*inputs)
    for value in inputs:
        value[:] = 99.0
    gradient = posture.eval_gradient([-2.0, 2.0])
    np.testing.assert_array_equal(gradient, [-2.0, 2.0])
    gradient[:] = 99.0
    np.testing.assert_array_equal(posture.eval_gradient([-2.0, 2.0]), [-2.0, 2.0])


def test_relaxed_posture_whole_body_objective_retention():
    skeleton = dart.dynamics.Skeleton()
    skeleton.createFreeJointAndBodyNodePair()
    objective = RelaxedPosture(np.zeros(6), np.full(6, -1.0), np.ones(6), np.ones(6))
    objective.enforce_ideal_posture = True
    reference = weakref.ref(objective)
    ik = skeleton.getIK(True)
    ik.setObjective(objective)
    del objective
    gc.collect()
    assert ik.getObjective() is reference()
    assert ik.getObjective().enforce_ideal_posture
    skeleton.setPosition(0, 0.25)
    assert ik.getObjective().eval(skeleton.getPositions()) == 0.03125
    ik.setObjective(None)
    gc.collect()
    assert ik.getObjective() is None
    assert reference() is None


def test_relaxed_posture_owner_cycle_collects():
    skeleton = dart.dynamics.Skeleton()
    skeleton.createFreeJointAndBodyNodePair()
    ik = skeleton.getIK(True)
    objective = RelaxedPosture(np.zeros(6), np.full(6, -1.0), np.ones(6), np.ones(6))
    objective.owner = ik
    ik.setObjective(objective)
    references = weakref.ref(objective), weakref.ref(ik), weakref.ref(skeleton)
    del objective, ik, skeleton
    gc.collect()
    assert all(reference() is None for reference in references)


def test_relaxed_posture_whole_body_solver_uses_weighted_residual():
    skeleton = dart.dynamics.Skeleton()
    skeleton.createFreeJointAndBodyNodePair()
    skeleton.setPositions(np.full(6, 0.2))
    ik = skeleton.getIK(True)
    objective = RelaxedPosture(np.zeros(6), np.zeros(6), np.zeros(6), np.full(6, 2.0))
    ik.setObjective(objective)
    solver = dart.optimizer.GradientDescentSolver(ik.getProblem())
    solver.setStepSize(0.25)
    solver.setNumMaxIterations(0)
    solver.setMaxAttempts(1)
    ik.setSolver(solver)
    assert not ik.solveAndApply(True)
    np.testing.assert_allclose(
        skeleton.getPositions(), np.full(6, 0.1), atol=1e-15, rtol=0
    )


@pytest.mark.parametrize(
    "ideal,lower,upper,weights",
    [
        ([0.0], [], [1.0], [1.0]),
        ([np.nan], [-1.0], [1.0], [1.0]),
        ([np.inf], [-1.0], [1.0], [1.0]),
        ([0.0], [np.nan], [1.0], [1.0]),
        ([0.0], [-1.0], [np.nan], [1.0]),
        ([0.0], [2.0], [1.0], [1.0]),
        ([0.0], [-1.0], [1.0], [-1.0]),
        ([0.0], [-1.0], [1.0], [np.inf]),
        ([[0.0]], [-1.0], [1.0], [1.0]),
    ],
)
def test_relaxed_posture_constructor_validation(ideal, lower, upper, weights):
    with pytest.raises(ValueError):
        RelaxedPosture(ideal, lower, upper, weights)


@pytest.mark.parametrize("positions", [[], [0.0, 0.0], [np.nan], [np.inf], [[0.0]]])
def test_relaxed_posture_evaluation_validation(positions):
    posture = RelaxedPosture([0.0], [-np.inf], [np.inf], [0.01])
    for evaluate in (posture.eval, posture.eval_gradient):
        with pytest.raises(ValueError):
            evaluate(positions)
