import math

import dartpy as dart
import pytest


# Problem reference: http://ab-initio.mit.edu/wiki/index.php/NLopt_Tutorial
class SampleObjFunc(dart.optimization.Function):
    def eval(self, x):
        return math.sqrt(x[1])

    def evalGradient(self, x, grad):
        grad[0] = 0
        grad[1] = 0.5 / (math.sqrt(x[1]) + 0.000001)


class SampleConstFunc(dart.optimization.Function):
    def __init__(self, a, b):
        super(SampleConstFunc, self).__init__()
        self.a = a
        self.b = b

    def eval(self, x):
        return (self.a * x[0] + self.b) * (self.a * x[0] + self.b) * (
            self.a * x[0] + self.b
        ) - x[1]

    def evalGradient(self, x, grad):
        grad[0] = 3 * self.a * (self.a * x[0] + self.b) * (self.a * x[0] + self.b)
        grad[1] = -1.0


def test_gradient_descent_solver():
    prob = dart.optimization.Problem(2)
    assert prob.getDimension() == 2

    prob.setLowerBounds([-1e100, 0])
    prob.setInitialGuess([1.234, 5.678])

    assert prob.getInitialGuess()[0] == pytest.approx(1.234)

    obj = SampleObjFunc()
    prob.setObjective(obj)

    solver = dart.optimization.GradientDescentSolver(prob)
    success = solver.solve()
    assert success is True

    min_f = prob.getOptimumValue()
    opt_x = prob.getOptimalSolution()

    assert min_f == pytest.approx(0, 1e-6)
    assert len(opt_x) == prob.getDimension()
    assert opt_x[0] == pytest.approx(1.234, 0.0)
    assert opt_x[1] == pytest.approx(0, solver.getTolerance())


def test_nlopt_solver():
    if not hasattr(dart.optimization, "NloptSolver"):
        return

    prob = dart.optimization.Problem(2)
    assert prob.getDimension() == 2

    prob.setLowerBounds([-1e100, 0])
    prob.setInitialGuess([1.234, 5.678])

    assert prob.getInitialGuess()[0] == pytest.approx(1.234)

    obj = SampleObjFunc()
    prob.setObjective(obj)

    const1 = SampleConstFunc(2, 0)
    const2 = SampleConstFunc(-1, 1)
    prob.addIneqConstraint(const1)
    prob.addIneqConstraint(const2)

    solver = dart.optimization.NloptSolver(prob)
    solver.setAlgorithm(dart.optimization.NloptSolver.Algorithm.LD_MMA)
    success = solver.solve()
    assert success is True

    min_f = prob.getOptimumValue()
    opt_x = prob.getOptimalSolution()

    assert min_f == pytest.approx(0.544330847, 1e-5)
    assert len(opt_x) == prob.getDimension()
    assert opt_x[0] == pytest.approx(0.333334, 1e-5)
    assert opt_x[1] == pytest.approx(0.296296, 1e-5)


if __name__ == "__main__":
    pytest.main()
