import math

import dartpy as dart
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
