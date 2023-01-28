import math
import platform

import dartpy as dart
import numpy as np
import pytest


def test_solve_for_free_joint():
    """
    Very simple test of InverseKinematics module, applied to a FreeJoint to
    ensure that the target is reachable
    """

    skel = dart.dynamics.Skeleton()
    [joint0, body0] = skel.createFreeJointAndBodyNodePair()

    ik = body0.getOrCreateIK()
    assert ik.isActive()

    tf = dart.math.Isometry3()
    tf.set_translation([0, 0, 0.8])
    tf.set_rotation(dart.math.AngleAxis(math.pi / 8.0, [0, 1, 0]).to_rotation_matrix())
    ik.getTarget().setTransform(tf)

    error_method = ik.getErrorMethod()
    assert error_method.getMethodName() == "TaskSpaceRegion"
    [lb, ub] = error_method.getBounds()
    assert len(lb) is 6
    assert len(ub) is 6
    error_method.setBounds(np.ones(6) * -1e-8, np.ones(6) * 1e-8)
    [lb, ub] = error_method.getBounds()
    assert lb == pytest.approx(-1e-8)
    assert ub == pytest.approx(1e-8)

    solver = ik.getSolver()
    solver.setNumMaxIterations(100)

    prob = ik.getProblem()

    tf_actual = ik.getTarget().getTransform().matrix()
    tf_expected = body0.getTransform().matrix()
    assert not np.isclose(tf_actual, tf_expected).all()

    success = solver.solve()
    assert success

    tf_actual = ik.getTarget().getTransform().matrix()
    tf_expected = body0.getTransform().matrix()
    assert np.isclose(tf_actual, tf_expected).all()


class FailingSolver(dart.optimization.Solver):
    def __init__(self, constant):
        super(FailingSolver, self).__init__()
        self.constant = constant

    def solve(self):
        problem = self.getProblem()
        if problem is None:
            print(
                "[FailingSolver::solve] Attempting to solve a nullptr problem! We will return false."
            )
            return False

        dim = problem.getDimension()
        wrong_solution = np.ones(dim) * self.constant
        problem.setOptimalSolution(wrong_solution)

        return False

    def getType(self):
        return "FailingSolver"

    def clone(self):
        return FailingSolver(self.constant)


def test_do_not_apply_solution_on_failure():
    skel = dart.dynamics.Skeleton()
    [joint, body] = skel.createFreeJointAndBodyNodePair()

    ik = body.getIK(True)
    solver = FailingSolver(10)
    ik.setSolver(solver)

    dofs = skel.getNumDofs()
    skel.resetPositions()

    assert not ik.solveAndApply(allowIncompleteResult=False)
    assert np.isclose(skel.getPositions(), np.zeros(dofs)).all()

    assert not ik.solveAndApply(allowIncompleteResult=True)
    assert not np.isclose(skel.getPositions(), np.zeros(dofs)).all()


if __name__ == "__main__":
    pytest.main()
