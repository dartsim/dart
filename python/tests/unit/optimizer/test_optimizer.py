import math

import dartpy_nb as dart
import pytest


class SampleObjFunc(dart.optimizer.Function):
    def eval(self, x):
        return math.sqrt(x[1])

    def evalGradient(self, x, grad):
        grad[0] = 0
        grad[1] = 0.5 / (math.sqrt(x[1]) + 0.000001)


class SampleConstFunc(dart.optimizer.Function):
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
