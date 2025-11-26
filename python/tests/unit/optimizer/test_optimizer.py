import math

import dartpy as dart
import pytest


class SampleObjFunc(dart.Function):
    def eval(self, x):
        return math.sqrt(x[1])

    def evalGradient(self, x, grad):
        grad[0] = 0
        grad[1] = 0.5 / (math.sqrt(x[1]) + 0.000001)


class SampleConstFunc(dart.Function):
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
    prob = dart.Problem(2)
    assert prob.get_dimension() == 2

    prob.set_lower_bounds([-1e100, 0])
    prob.set_initial_guess([1.234, 5.678])

    assert prob.get_initial_guess()[0] == pytest.approx(1.234)

    obj = SampleObjFunc()
    prob.set_objective(obj)

    solver = dart.GradientDescentSolver(prob)
    success = solver.solve()
    assert success is True

    min_f = prob.get_optimum_value()
    opt_x = prob.get_optimal_solution()

    assert min_f == pytest.approx(0, 1e-6)
    assert len(opt_x) == prob.get_dimension()
    assert opt_x[0] == pytest.approx(1.234, 0.0)
    assert opt_x[1] == pytest.approx(0, solver.get_tolerance())


if __name__ == "__main__":
    pytest.main()
