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

    skel = dart.Skeleton()
    [joint0, body0] = skel.create_free_joint_and_body_node_pair()

    ik = body0.get_or_create_ik()
    assert ik.is_active()

    tf = dart.Isometry3()
    tf.set_translation([0, 0, 0.8])
    tf.set_rotation(dart.AngleAxis(math.pi / 8.0, [0, 1, 0]).to_rotation_matrix())
    ik.get_target().set_transform(tf)

    error_method = ik.get_error_method()
    assert error_method.get_method_name() == "TaskSpaceRegion"
    [lb, ub] = error_method.get_bounds()
    assert len(lb) == 6
    assert len(ub) == 6
    error_method.set_bounds(np.ones(6) * -1e-8, np.ones(6) * 1e-8)
    [lb, ub] = error_method.get_bounds()
    assert lb == pytest.approx(-1e-8)
    assert ub == pytest.approx(1e-8)

    solver = ik.get_solver()
    solver.set_num_max_iterations(100)

    prob = ik.get_problem()

    tf_actual = ik.get_target().get_transform().matrix()
    tf_expected = body0.get_transform().matrix()
    assert not np.isclose(tf_actual, tf_expected).all()

    success = solver.solve()
    assert success

    tf_actual = ik.get_target().get_transform().matrix()
    tf_expected = body0.get_transform().matrix()
    assert np.isclose(tf_actual, tf_expected).all()


class FailingSolver(dart.Solver):
    def __init__(self, constant):
        super(FailingSolver, self).__init__()
        self.constant = constant

    def solve(self):
        problem = self.get_problem()
        if problem is None:
            print(
                "[FailingSolver::solve] Attempting to solve a nullptr problem! We will return false."
            )
            return False

        dim = problem.get_dimension()
        wrong_solution = np.ones(dim) * self.constant
        problem.set_optimal_solution(wrong_solution)

        return False

    def getType(self):
        return "FailingSolver"

    def clone(self):
        return FailingSolver(self.constant)


def test_do_not_apply_solution_on_failure():
    skel = dart.Skeleton()
    [joint, body] = skel.create_free_joint_and_body_node_pair()

    ik = body.get_ik(True)
    solver = FailingSolver(10)
    ik.set_solver(solver)

    dofs = skel.get_num_dofs()
    skel.reset_positions()

    assert not ik.solve_and_apply(False)
    assert np.isclose(skel.get_positions(), np.zeros(dofs)).all()

    assert not ik.solve_and_apply(True)
    assert not np.isclose(skel.get_positions(), np.zeros(dofs)).all()


if __name__ == "__main__":
    pytest.main()
