import math

import dartpy as dart
import numpy as np
import pytest


# Problem reference: http://ab-initio.mit.edu/wiki/index.php/NLopt_Tutorial
class SampleObjFunc(dart.optimizer.Function):
    def eval(self, x):
        return math.sqrt(x[1])

    def evalGradient(self, x, grad):
        grad[0] = 0
        grad[1] = 0.5 / (math.sqrt(x[1]) + 0.000001)


def test_modular_function_accepts_callable():
    function = dart.optimizer.ModularFunction()
    function.setCostFunction(lambda x: x[0] + 2 * x[1])
    assert function.eval([3, 4]) == pytest.approx(11)


def test_function_without_gradient_uses_native_fallback():
    class EvalOnlyFunction(dart.optimizer.Function):
        def eval(self, x):
            return float(x[0] ** 2)

    problem = dart.optimizer.Problem(1)
    problem.setInitialGuess([0.0])
    objective = EvalOnlyFunction()
    problem.setObjective(objective)
    solver = dart.optimizer.GradientDescentSolver(problem)
    solver.setNumMaxIterations(1)

    assert solver.solve() is True


def test_function_gradient_override_is_called():
    calls = []

    class GradientFunction(dart.optimizer.Function):
        def eval(self, x):
            return float(x[0] ** 2)

        def evalGradient(self, x, grad):
            calls.append("gradient")
            grad[:] = 2 * x

    problem = dart.optimizer.Problem(1)
    problem.setInitialGuess([0.0])
    objective = GradientFunction()
    problem.setObjective(objective)
    solver = dart.optimizer.GradientDescentSolver(problem)
    solver.setNumMaxIterations(1)

    assert solver.solve() is True
    assert calls == ["gradient"]


def test_override_arguments_are_read_only_but_gradient_is_writable():
    seen = {}

    class CheckingFunction(dart.optimizer.Function):
        def eval(self, x):
            seen["x"] = x.flags.writeable
            return float(x[0] ** 2)

        def evalGradient(self, x, grad):
            seen["grad"] = grad.flags.writeable
            grad[:] = 2 * x

    problem = dart.optimizer.Problem(1)
    problem.setInitialGuess([1.0])
    problem.setObjective(CheckingFunction())
    solver = dart.optimizer.GradientDescentSolver(problem)
    solver.setNumMaxIterations(1)
    solver.solve()
    assert seen == {"x": False, "grad": True}


def test_gradient_callbacks_retain_independent_arrays_and_apply_updates():
    gradients = []
    snapshots = []
    positions = []

    class RetainingFunction(dart.optimizer.Function):
        def eval(self, x):
            return float(x @ x)

        def evalGradient(self, x, grad):
            for previous, expected in zip(gradients, snapshots):
                np.testing.assert_array_equal(previous, expected)
            assert grad.flags.writeable and grad.flags.owndata
            grad[:] = 2 * x
            gradients.append(grad)
            snapshots.append(grad.copy())
            positions.append(x.copy())

    problem = dart.optimizer.Problem(2)
    problem.setInitialGuess([1.0, 2.0])
    problem.setObjective(RetainingFunction())
    solver = dart.optimizer.GradientDescentSolver(problem)
    solver.setNumMaxIterations(2)
    solver.setStepSize(0.1)
    solver.setMaxAttempts(1)
    solver.setPerturbationStep(0)
    solver.solve()
    np.testing.assert_allclose(positions, [[1, 2], [0.8, 1.6], [0.64, 1.28]])
    assert not np.shares_memory(gradients[0], gradients[1])
    del solver, problem
    for gradient, expected in zip(gradients, snapshots):
        np.testing.assert_array_equal(gradient, expected)


def test_gradient_callback_rejects_changed_output_size():
    class ResizingFunction(dart.optimizer.Function):
        def eval(self, x):
            return float(x @ x)

        def evalGradient(self, x, grad):
            grad.resize(grad.size + 1, refcheck=False)

    problem = dart.optimizer.Problem(1)
    problem.setInitialGuess([1.0])
    problem.setObjective(ResizingFunction())
    solver = dart.optimizer.GradientDescentSolver(problem)
    solver.setNumMaxIterations(1)
    with pytest.raises(ValueError, match="gradient.*size"):
        solver.solve()


def test_gradient_descent_solver():
    prob = dart.optimizer.Problem(2)
    assert prob.getDimension() == 2

    prob.setLowerBounds([-1e100, 0])
    prob.setInitialGuess([1.234, 5.678])

    assert prob.getInitialGuess()[0] == pytest.approx(1.234)

    obj = SampleObjFunc()
    prob.setObjective(obj)

    solver = dart.optimizer.GradientDescentSolver(prob)
    success = solver.solve()
    assert success is True

    min_f = prob.getOptimumValue()
    opt_x = prob.getOptimalSolution()

    assert min_f == pytest.approx(0, 1e-6)
    assert len(opt_x) == prob.getDimension()
    assert opt_x[0] == pytest.approx(1.234, 0.0)
    assert opt_x[1] == pytest.approx(0, solver.getTolerance())


if __name__ == "__main__":
    pytest.main()
