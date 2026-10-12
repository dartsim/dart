import math
import platform

import dartpy as dart
import numpy as np
import pytest


class SsikLikeSolution:
    def __init__(self, q, validity=None):
        self.q = np.asarray(q)
        self.fk_residual = 0.0
        self.refinement_used = False
        if validity is not None:
            self.validity = validity


def _create_free_joint_ik():
    skel = dart.dynamics.Skeleton()
    [joint, body] = skel.createFreeJointAndBodyNodePair()
    ik = body.getOrCreateIK()
    dofs = list(range(skel.getNumDofs()))
    ik.setDofs(dofs)
    return skel, body, ik, dofs


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
    assert len(lb) == 6
    assert len(ub) == 6
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


def test_python_analytical_accepts_ssik_like_solutions():
    skel, body, ik, dofs = _create_free_joint_ik()
    expected = np.linspace(0.1, 0.6, len(dofs))
    seen_targets = []

    def solve(target):
        seen_targets.append(np.asarray(target))
        return [SsikLikeSolution(expected)]

    analytical = ik.setPythonAnalytical(solve, dofs, "ssik.prebuilt.fake_ik")

    assert ik.getDofs() == dofs
    assert ik.getGradientMethod().getMethodName() == "ssik.prebuilt.fake_ik"
    assert ik.getAnalytical() is not None
    assert analytical.getDofs() == dofs

    target = dart.math.Isometry3()
    target.set_translation([0.1, 0.2, 0.3])

    solutions = analytical.getSolutions(target)

    assert len(solutions) == 1
    assert np.allclose(solutions[0].mConfig, expected)
    assert solutions[0].mValidity == dart.dynamics.InverseKinematicsAnalytical.VALID
    assert len(seen_targets) == 1
    assert seen_targets[0].shape == (4, 4)
    assert np.allclose(seen_targets[0], target.matrix())


def test_python_analytical_accepts_tuple_and_solution_objects():
    skel, body, ik, dofs = _create_free_joint_ik()
    valid = np.linspace(0.2, 0.7, len(dofs))
    out_of_reach = np.linspace(-0.2, -0.7, len(dofs))

    def solve(target):
        return [
            (out_of_reach, dart.dynamics.InverseKinematicsAnalytical.OUT_OF_REACH),
            dart.dynamics.InverseKinematicsAnalyticalSolution(valid),
        ]

    analytical = ik.setPythonAnalytical(solve, dofs)
    solutions = analytical.getSolutions(dart.math.Isometry3())

    assert len(solutions) == 2
    assert solutions[0].mValidity == dart.dynamics.InverseKinematicsAnalytical.VALID
    assert np.allclose(solutions[0].mConfig, valid)
    assert (
        solutions[1].mValidity == dart.dynamics.InverseKinematicsAnalytical.OUT_OF_REACH
    )
    assert np.allclose(solutions[1].mConfig, out_of_reach)


def test_python_analytical_can_leave_extra_dofs_to_dart_ik():
    skel, body, ik, dofs = _create_free_joint_ik()
    analytical_dofs = dofs[:3]

    def solve(target):
        return [np.zeros(len(analytical_dofs))]

    analytical = ik.setPythonAnalytical(solve, analytical_dofs, "partial_ssik")
    analytical.setExtraDofUtilization(
        dart.dynamics.InverseKinematicsAnalytical.PRE_AND_POST_ANALYTICAL
    )

    assert ik.getDofs() == dofs
    assert analytical.getDofs() == analytical_dofs
    assert ik.getAnalytical() is not None
    assert ik.getGradientMethod().getMethodName() == "partial_ssik"
    assert (
        analytical.getExtraDofUtilization()
        == dart.dynamics.InverseKinematicsAnalytical.PRE_AND_POST_ANALYTICAL
    )


def test_python_analytical_rejects_wrong_solution_dimension():
    skel, body, ik, dofs = _create_free_joint_ik()

    def solve(target):
        return [np.zeros(len(dofs) - 1)]

    analytical = ik.setPythonAnalytical(solve, dofs)

    with pytest.raises(ValueError, match="DOFs were registered"):
        analytical.getSolutions(dart.math.Isometry3())


def test_python_analytical_rejects_out_of_range_dof():
    skel, body, ik, dofs = _create_free_joint_ik()

    def solve(target):
        return []

    # A nonexistent DOF index (e.g. a stale solver chain for another skeleton)
    # must raise instead of segfaulting later in checkSolutionJointLimits().
    with pytest.raises(ValueError, match="out of range"):
        ik.setPythonAnalytical(solve, [len(dofs) + 5])


@pytest.mark.parametrize("shift_dofs", [False, True])
def test_python_analytical_copy_uses_jacobian_dls(shift_dofs, capfd):
    skel = dart.dynamics.Skeleton("original")
    joint, body = skel.createPrismaticJointAndBodyNodePair()
    joint.setAxis([1, 0, 0])
    ik = body.createShapeNode(dart.dynamics.SphereShape(0.1)).getOrCreateIK()
    defaults = ik.getErrorMethod().getErrorMethodProperties()
    ik.setPythonAnalytical(lambda target: [], [0], "stored_dofs")
    ik.getGradientMethod().setComponentWeights(np.array([0.75]))
    ik.getErrorMethod().setBounds(np.full(6, -1e-8), np.full(6, 1e-8))
    ik.getErrorMethod().setComputeFromCenter(False)
    ik.getSolver().setNumMaxIterations(100)
    capfd.readouterr()

    if shift_dofs:
        copy = dart.dynamics.Skeleton("shifted")
        base = copy.createPrismaticJointAndBodyNodePair()[1]
        body.copyTo(copy, base)
        copied_body = copy.getBodyNode(1)
    else:
        copy = body.copyAs("same")
        copied_body = copy.getBodyNode(0)
    warning = capfd.readouterr().err
    assert warning.count("uses JacobianDLS") == 1
    assert "Set the analytical solver up again for the copy." in warning

    copied_ik = copied_body.getShapeNode(0).getIK()
    assert copied_ik.getDofs() == [int(shift_dofs)]
    assert copied_ik.getAnalytical() is None
    assert copied_ik.getGradientMethod().getMethodName() == "JacobianDLS"
    assert copied_ik.getGradientMethod().getComponentWeights() == pytest.approx([0.75])
    error = copied_ik.getErrorMethod()
    assert error.getErrorLengthClamp() == defaults.mErrorLengthClamp
    assert error.getErrorMethodProperties().mErrorWeights == pytest.approx(
        defaults.mErrorWeights
    )
    lower, upper = error.getBounds()
    assert lower == pytest.approx(np.full(6, -1e-8))
    assert upper == pytest.approx(np.full(6, 1e-8))
    assert not error.isComputingFromCenter()
    target = dart.math.Isometry3()
    target.set_translation([0.8, 0, 0])
    copied_ik.getTarget().setTransform(target)
    assert copied_ik.solveAndApply(False)
    assert copied_body.getWorldTransform().matrix() == pytest.approx(
        target.matrix(), abs=1e-7
    )
    assert skel.getPositions() == pytest.approx([0])
    assert ik.getGradientMethod().getMethodName() == "stored_dofs"


def test_python_analytical_clone_skeleton_keeps_solver_and_solves():
    skel = dart.dynamics.Skeleton("original")
    joint, body = skel.createPrismaticJointAndBodyNodePair()
    joint.setAxis([1, 0, 0])
    ik = body.createShapeNode(dart.dynamics.SphereShape(0.1)).getOrCreateIK()
    ik.setPythonAnalytical(lambda target: [[target[0, 3]]], [0], "prismatic")
    ik.getErrorMethod().setBounds(np.full(6, -1e-8), np.full(6, 1e-8))
    clone = skel.clone()
    cloned_ik = clone.getBodyNode(0).getShapeNode(0).getIK()
    assert cloned_ik.getAnalytical() is not None
    assert cloned_ik.getGradientMethod().getMethodName() == "prismatic"
    target = dart.math.Isometry3()
    target.set_translation([0.8, 0, 0])
    cloned_ik.getTarget().setTransform(target)
    assert cloned_ik.solveAndApply(False)
    assert clone.getPositions() == pytest.approx([0.8], abs=1e-7)
    assert skel.getPositions() == pytest.approx([0])


class FailingSolver(dart.optimizer.Solver):
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
