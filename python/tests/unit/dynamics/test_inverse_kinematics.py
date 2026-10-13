import math
import platform
import signal
import subprocess
import sys
import textwrap

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


def test_whole_body_ik_reaches_end_effector_target_on_three_link_arm():
    skel = dart.dynamics.Skeleton()
    assert skel.getIK() is None
    body = None
    for index in range(3):
        joint, body = skel.createRevoluteJointAndBodyNodePair(body)
        joint.setAxis([0, 0, 1])
        offset = dart.math.Isometry3()
        offset.set_translation([0.4 if index else 0.0, 0, 0])
        joint.setTransformFromParentBodyNode(offset)
    ee = body.createEndEffector("tip")
    offset = dart.math.Isometry3()
    offset.set_translation([0.4, 0, 0])
    ee.setRelativeTransform(offset)

    skel.setPositions([0.25, -0.4, 0.3])
    target = ee.getTransform()
    skel.setPositions([0.05, -0.15, 0.1])
    assert not np.allclose(ee.getTransform().matrix(), target.matrix())
    assert ee.getIK() is None
    ik = ee.getIK(True)
    ik.getTarget().setTransform(target)
    ik.getErrorMethod().setBounds(np.full(6, -1e-6), np.full(6, 1e-6))

    whole_body = skel.getIK(True)
    assert isinstance(whole_body, dart.dynamics.WholeBodyIK)
    assert skel.getIK() is whole_body
    assert skel.getIK(True) is whole_body
    assert isinstance(whole_body, dart.dynamics.HierarchicalIK)
    assert isinstance(whole_body, dart.common.Subject)
    assert whole_body.getSkeleton().getNumDofs() == 3
    assert whole_body.getProblem().getDimension() == 3
    whole_body.getSolver().setNumMaxIterations(2000)
    whole_body.refreshIKHierarchy()
    assert whole_body.solveAndApply(True)
    np.testing.assert_allclose(ee.getTransform().matrix(), target.matrix(), atol=1e-5)


def test_end_effector_lookup_transforms_and_support_geometry():
    skel = dart.dynamics.Skeleton()
    _, body = skel.createFreeJointAndBodyNodePair()
    ee = body.createEndEffector()
    assert ee.getName() == "EndEffector"
    assert skel.getNumEndEffectors() == 1
    assert skel.getEndEffector(0).getName() == ee.getName()
    assert skel.getEndEffector(ee.getName()).getName() == ee.getName()
    assert skel.getEndEffector("missing") is None

    current = dart.math.Isometry3()
    current.set_translation([0.1, 0.2, 0.3])
    default = dart.math.Isometry3()
    default.set_translation([0.4, 0.5, 0.6])
    ee.setRelativeTransform(current)
    ee.setDefaultRelativeTransform(default)
    np.testing.assert_allclose(ee.getRelativeTransform().matrix(), current.matrix())
    ee.resetRelativeTransform()
    np.testing.assert_allclose(ee.getRelativeTransform().matrix(), default.matrix())
    ee.setDefaultRelativeTransform(current, True)
    np.testing.assert_allclose(ee.getRelativeTransform().matrix(), current.matrix())

    assert not ee.hasSupport()
    assert ee.getSupport() is None
    support = ee.getSupport(True)
    assert ee.hasSupport()
    assert isinstance(support, dart.dynamics.Support)
    points = [[-0.2, 0, -0.1], [0.2, 0, -0.1], [0.2, 0, 0.1], [-0.2, 0, 0.1]]
    support.setGeometry(points)
    geometry = support.getGeometry()
    assert isinstance(geometry, list)
    np.testing.assert_allclose(geometry, points)
    support.setActive(False)
    assert not support.isActive()
    support.setActive()
    assert support.isActive()
    support.setGeometry([])
    assert support.getGeometry() == []


@pytest.mark.parametrize("balance_method", ["SHIFT_SUPPORT", "SHIFT_COM"])
@pytest.mark.parametrize(
    "error_method", ["FROM_CENTROID", "FROM_EDGE", "OPTIMIZE_BALANCE"]
)
def test_balance_constraint_methods_evaluate_finite_error(balance_method, error_method):
    skel = dart.dynamics.Skeleton()
    skel.setGravity([0, -9.81, 0])
    _, body = skel.createFreeJointAndBodyNodePair()
    support = body.createEndEffector("foot").getSupport(True)
    support.setGeometry([[-1, 0, -1], [1, 0, -1], [1, 0, 1], [-1, 0, 1]])
    support.setActive()
    balance = dart.constraint.BalanceConstraint(skel.getIK(True))
    assert isinstance(balance, dart.optimizer.Function)
    cls = dart.constraint.BalanceConstraint
    assert balance.getBalanceMethod() == cls.BalanceMethod.SHIFT_SUPPORT
    assert balance.getErrorMethod() == cls.ErrorMethod.FROM_CENTROID
    assert not hasattr(cls, "SHIFT_SUPPORT")
    assert not hasattr(cls, "FROM_CENTROID")
    balance = cls(
        skel.getIK(),
        balanceMethod=getattr(cls.BalanceMethod, balance_method),
        errorMethod=getattr(cls.ErrorMethod, error_method),
    )
    balance.setBalanceMethod(getattr(cls.BalanceMethod, balance_method))
    balance.setErrorMethod(getattr(cls.ErrorMethod, error_method))
    assert balance.getBalanceMethod() == getattr(cls.BalanceMethod, balance_method)
    assert balance.getErrorMethod() == getattr(cls.ErrorMethod, error_method)
    balance.setOptimizationTolerance(2e-5)
    balance.setPseudoInverseDamping(0.125)
    assert balance.getOptimizationTolerance() == pytest.approx(2e-5)
    assert balance.getPseudoInverseDamping() == pytest.approx(0.125)
    # Put the COM beyond the support polygon, so eval exercises each error method.
    skel.setPositions([0, 0, 0, 2, 0, 0])
    support_points = [[-3, 0, -1], [-1, 0, -1], [-1, 0, 1], [-3, 0, 1]]
    support.setGeometry(support_points)
    value = balance.eval(skel.getPositions())
    assert np.isfinite(value)
    assert value > 0


def test_whole_body_ik_objective_accessors():
    class Objective(dart.optimizer.Function):
        def eval(self, x):
            return float(np.dot(x, x))

    skel = dart.dynamics.Skeleton()
    skel.createFreeJointAndBodyNodePair()
    ik = skel.getIK(True)
    assert ik.getObjective() is None
    assert ik.getNullSpaceObjective() is None
    objective = Objective()
    ik.setObjective(objective)
    ik.setNullSpaceObjective(objective)
    assert ik.getObjective() is objective
    assert ik.getNullSpaceObjective() is objective
    assert ik.getObjective().eval(np.ones(6)) == 6
    ik.setObjective(None)
    ik.setNullSpaceObjective(None)
    assert ik.getObjective() is None
    assert ik.getNullSpaceObjective() is None


@pytest.mark.parametrize("clone_method", ["clone", "cloneWholeBodyIK"])
def test_whole_body_ik_clone_without_solver(clone_method):
    # Keep a null-solver regression from crashing the whole pytest process.
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            textwrap.dedent(
                f"""
                import dartpy as dart
                skeleton = dart.dynamics.Skeleton()
                skeleton.createFreeJointAndBodyNodePair()
                ik = skeleton.getIK(True)
                ik.setSolver(None)
                target = dart.dynamics.Skeleton()
                target.createFreeJointAndBodyNodePair()
                cloned = ik.{clone_method}(target)
                assert cloned.getSolver() is None
                assert cloned.getProblem().getDimension() == 6
                """
            ),
        ],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize("clone_method", ["clone", "cloneWholeBodyIK"])
def test_whole_body_ik_clone_preserves_objectives(clone_method):
    class Objective(dart.optimizer.Function):
        def eval(self, x):
            return float(np.dot(x, x))

    skel = dart.dynamics.Skeleton()
    skel.createFreeJointAndBodyNodePair()
    ik = skel.getIK(True)
    objective = Objective()
    null_space_objective = Objective()
    ik.setObjective(objective)
    ik.setNullSpaceObjective(null_space_objective)

    target = dart.dynamics.Skeleton()
    target.createFreeJointAndBodyNodePair()
    cloned = getattr(ik, clone_method)(target)
    assert cloned.getObjective() is objective
    assert cloned.getNullSpaceObjective() is null_space_objective

    ik.setObjective(None)
    ik.setNullSpaceObjective(None)
    assert cloned.getObjective() is objective
    assert cloned.getNullSpaceObjective() is null_space_objective
    assert cloned.getObjective().eval(np.ones(6)) == 6
    assert cloned.getNullSpaceObjective().eval(np.ones(6)) == 6


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


@pytest.mark.parametrize("whole_body", [False, True])
def test_do_not_apply_solution_on_failure(whole_body):
    skel = dart.dynamics.Skeleton()
    [joint, body] = skel.createFreeJointAndBodyNodePair()

    ik = skel.getIK(True) if whole_body else body.getIK(True)
    solver = FailingSolver(10)
    ik.setSolver(solver)

    dofs = skel.getNumDofs()
    skel.resetPositions()

    assert not ik.solveAndApply(allowIncompleteResult=False)
    assert np.isclose(skel.getPositions(), np.zeros(dofs)).all()

    assert not ik.solveAndApply(allowIncompleteResult=True)
    assert not np.isclose(skel.getPositions(), np.zeros(dofs)).all()


@pytest.mark.parametrize("whole_body", [False, True])
@pytest.mark.parametrize("allow_incomplete", [None, False, True])
def test_cleared_solver_does_not_apply_an_empty_solution(whole_body, allow_incomplete):
    call = (
        "ik.solveAndApply()"
        if allow_incomplete is None
        else f"ik.solveAndApply(allowIncompleteResult={allow_incomplete!r})"
    )
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            textwrap.dedent(
                f"""
                import dartpy as dart
                import numpy as np
                skeleton = dart.dynamics.Skeleton()
                body = skeleton.createFreeJointAndBodyNodePair()[1]
                ik = skeleton.getIK(True) if {whole_body!r} else body.getIK(True)
                positions = np.linspace(0.1, 0.6, 6)
                skeleton.setPositions(positions)
                ik.setSolver(None)
                assert ik.getSolver() is None
                assert not {call}
                np.testing.assert_array_equal(skeleton.getPositions(), positions)
                """
            ),
        ],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, result.stderr
    assert "setPositions" not in result.stderr, result.stderr


EXPIRED_IK_CASES = {
    "stale_solver": """
        import gc
        import dartpy as dart
        skeleton = dart.dynamics.Skeleton()
        skeleton.createFreeJointAndBodyNodePair()
        ik = skeleton.getIK(True)
        solver = ik.getSolver()
        solver.setTolerance(0.0)
        del ik, skeleton
        gc.collect()
        solver.solve()
    """,
    "hierarchy_constraint_as_objective": """
        import gc
        import dartpy as dart
        skeleton = dart.dynamics.Skeleton()
        skeleton.createFreeJointAndBodyNodePair()
        ik = skeleton.getIK(True)
        problem = dart.optimizer.Problem(6)
        problem.setObjective(ik.getProblem().getEqConstraint(0))
        solver = dart.optimizer.GradientDescentSolver(problem)
        del ik, skeleton
        gc.collect()
        solver.solve()
    """,
}


@pytest.mark.parametrize("case", sorted(EXPIRED_IK_CASES))
def test_solving_after_the_whole_body_ik_expires_does_not_crash(case):
    # These segfaulted before HierarchicalIK::Constraint::evalGradient checked
    # its weak module; run them in a child so a regression cannot kill pytest.
    result = subprocess.run(
        [sys.executable, "-c", textwrap.dedent(EXPIRED_IK_CASES[case])],
        capture_output=True,
        text=True,
        timeout=60,
    )
    if result.returncode != 0:
        # Builds with native assertions stop at the DART_ASSERT that follows
        # the reported misuse (SIGABRT, or exit 3 from the MSVC runtime).
        assert result.returncode in {-signal.SIGABRT, 3}, result.stderr
        assert "expired HierarchicalIK" in result.stderr, result.stderr


if __name__ == "__main__":
    pytest.main()
