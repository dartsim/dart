from __future__ import annotations

import numpy as np
import pytest

import dartpy as dart

SOLVER_TYPES = (
    dart.DantzigSolver,
    dart.LemkeSolver,
    dart.BaraffSolver,
    dart.DirectSolver,
    dart.PgsSolver,
    dart.SymmetricPsorSolver,
    dart.JacobiSolver,
    dart.RedBlackGaussSeidelSolver,
    dart.BlockedJacobiSolver,
    dart.BgsSolver,
    dart.NncgSolver,
    dart.SubspaceMinimizationSolver,
    dart.ApgdSolver,
    dart.TgsSolver,
    dart.MinimumMapNewtonSolver,
    dart.FischerBurmeisterNewtonSolver,
    dart.PenalizedFischerBurmeisterNewtonSolver,
    dart.InteriorPointSolver,
    dart.MprgpSolver,
    dart.ShockPropagationSolver,
    dart.StaggeringSolver,
    dart.AdmmSolver,
    dart.SapSolver,
    dart.BoxedSemiSmoothNewtonSolver,
)

BOXED_SOLVERS = {
    dart.DantzigSolver,
    dart.PgsSolver,
    dart.SymmetricPsorSolver,
    dart.JacobiSolver,
    dart.RedBlackGaussSeidelSolver,
    dart.BlockedJacobiSolver,
    dart.BgsSolver,
    dart.NncgSolver,
    dart.SubspaceMinimizationSolver,
    dart.ApgdSolver,
    dart.TgsSolver,
    dart.ShockPropagationSolver,
    dart.StaggeringSolver,
    dart.AdmmSolver,
    dart.SapSolver,
    dart.BoxedSemiSmoothNewtonSolver,
}

FINDEX_SOLVERS = {
    dart.DantzigSolver,
    dart.PgsSolver,
    dart.SymmetricPsorSolver,
    dart.JacobiSolver,
    dart.RedBlackGaussSeidelSolver,
    dart.BlockedJacobiSolver,
    dart.BgsSolver,
    dart.NncgSolver,
    dart.SubspaceMinimizationSolver,
    dart.ApgdSolver,
    dart.TgsSolver,
    dart.ShockPropagationSolver,
    dart.StaggeringSolver,
    dart.AdmmSolver,
    dart.SapSolver,
    dart.BoxedSemiSmoothNewtonSolver,
}


def test_lcp_problem_constructors_classify_standard_boxed_and_findex() -> None:
    A = np.eye(2)
    b = np.array([1.0, 2.0])

    standard = dart.LcpProblem(A, b)
    assert standard.size() == 2
    assert standard.get_type() == dart.LcpProblemType.STANDARD
    assert standard.is_standard_lcp()
    assert standard.is_boxed_lcp()
    assert not standard.has_friction_index()
    np.testing.assert_allclose(standard.lo, [0.0, 0.0])
    assert np.isinf(standard.hi).all()
    np.testing.assert_array_equal(standard.findex, [-1, -1])

    boxed = dart.LcpProblem(A, b, np.array([-1.0, 0.0]), np.array([1.0, 3.0]))
    assert boxed.get_type() == dart.LcpProblemType.BOXED
    assert not boxed.is_standard_lcp()
    assert boxed.is_boxed_lcp()
    assert not boxed.has_friction_index()

    findex = dart.LcpProblem(
        A,
        b,
        np.array([0.0, -1.0]),
        np.array([np.inf, 1.0]),
        np.array([-1, 0], dtype=np.int32),
    )
    assert findex.get_type() == dart.LcpProblemType.FRICTION_INDEX
    assert not findex.is_standard_lcp()
    assert not findex.is_boxed_lcp()
    assert findex.has_friction_index()
    assert dart.lcp_problem_type_to_string(dart.LcpProblemType.FRICTION_INDEX)


def test_lcp_problem_rejects_findex_with_invalid_bound_dimensions() -> None:
    problem = dart.LcpProblem(
        np.eye(2),
        np.array([1.0, 2.0]),
        np.array([0.0]),
        np.array([np.inf, 1.0]),
        np.array([-1, 0], dtype=np.int32),
    )

    assert problem.get_type() == dart.LcpProblemType.INVALID
    assert not problem.is_standard_lcp()
    assert not problem.is_boxed_lcp()
    assert not problem.has_friction_index()
    assert not dart.DantzigSolver().supports_problem(problem)


@pytest.mark.parametrize("solver_type", SOLVER_TYPES)
def test_all_lcp_solvers_are_available_from_dartpy_math(solver_type: type) -> None:
    problem = dart.LcpProblem(np.eye(3), np.array([1.0, 0.5, 2.0]))
    options = dart.LcpOptions.high_accuracy()
    options.max_iterations = 1000

    solver = solver_type()
    supports_boxed = solver_type in BOXED_SOLVERS
    supports_findex = solver_type in FINDEX_SOLVERS

    result, solution = solver.solve(problem, options=options)

    assert solver.get_name()
    assert solver.get_category()
    assert solver.supports_standard_lcp()
    assert solver.supports_boxed_lcp() is supports_boxed
    assert solver.supports_friction_index() is supports_findex
    assert solver.supports_problem(problem)
    assert result.status in (
        dart.LcpSolverStatus.SUCCESS,
        dart.LcpSolverStatus.MAX_ITERATIONS,
    )
    assert dart.lcp_solver_status_to_string(result.status)
    np.testing.assert_allclose(solution, [1.0, 0.5, 2.0], atol=1e-6)


@pytest.mark.parametrize("solver_type", SOLVER_TYPES)
def test_lcp_solver_capabilities_classify_problem_forms(solver_type: type) -> None:
    A = np.eye(2)
    standard = dart.LcpProblem(A, np.array([1.0, 2.0]))
    boxed = dart.LcpProblem(A, np.array([1.0, 2.0]), np.array([-1.0, 0.0]), np.ones(2))
    findex = dart.LcpProblem(
        A,
        np.array([1.0, 2.0]),
        np.array([0.0, -1.0]),
        np.array([np.inf, 1.0]),
        np.array([-1, 0], dtype=np.int32),
    )

    solver = solver_type()
    supports_boxed = solver_type in BOXED_SOLVERS
    supports_findex = solver_type in FINDEX_SOLVERS

    assert solver.supports_problem(standard)
    assert solver.supports_problem(boxed) is supports_boxed
    assert solver.supports_problem(findex) is supports_findex


def test_lcp_solver_rejects_wrong_initial_guess_size() -> None:
    problem = dart.LcpProblem(np.eye(2), np.array([1.0, 2.0]))
    solver = dart.DantzigSolver()

    with pytest.raises(ValueError, match="initial_guess"):
        solver.solve(problem, initial_guess=np.zeros(3))


def test_lcp_solver_rejects_non_finite_problem_data() -> None:
    A = np.eye(2)
    A[0, 1] = np.nan
    problem = dart.LcpProblem(A, np.array([1.0, 2.0]))

    result, _ = dart.DantzigSolver().solve(problem)

    assert result.status == dart.LcpSolverStatus.INVALID_PROBLEM
    assert "non-finite" in result.message
