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
    assert standard.is_valid()
    assert standard.get_validation_message() == ""
    assert standard.get_type() == dart.LcpProblemType.STANDARD
    assert standard.is_standard_lcp()
    assert standard.is_boxed_lcp()
    assert not standard.has_friction_index()
    assert standard.get_friction_index_row_count() == 0
    assert standard.get_friction_index_contact_count() == 0
    np.testing.assert_allclose(standard.lo, [0.0, 0.0])
    assert np.isinf(standard.hi).all()
    np.testing.assert_array_equal(standard.findex, [-1, -1])

    boxed = dart.LcpProblem(A, b, np.array([-1.0, 0.0]), np.array([1.0, 3.0]))
    assert boxed.get_type() == dart.LcpProblemType.BOXED
    assert not boxed.is_standard_lcp()
    assert boxed.is_boxed_lcp()
    assert not boxed.has_friction_index()
    assert boxed.get_friction_index_row_count() == 0
    assert boxed.get_friction_index_contact_count() == 0

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
    assert findex.get_friction_index_row_count() == 1
    assert findex.get_friction_index_contact_count() == 1
    assert dart.lcp_problem_type_to_string(dart.LcpProblemType.FRICTION_INDEX)


def test_lcp_problem_exposes_findex_contact_metadata() -> None:
    findex = dart.LcpProblem(
        np.eye(6),
        np.ones(6),
        np.array([0.0, -1.0, -1.0, 0.0, -0.5, -0.5]),
        np.array([np.inf, 1.0, 1.0, np.inf, 0.5, 0.5]),
        np.array([-1, 0, 0, -1, 3, 3], dtype=np.int32),
    )

    assert findex.get_type() == dart.LcpProblemType.FRICTION_INDEX
    assert findex.get_friction_index_row_count() == 4
    assert findex.get_friction_index_contact_count() == 2


def test_lcp_problem_rejects_findex_with_invalid_bound_dimensions() -> None:
    problem = dart.LcpProblem(
        np.eye(2),
        np.array([1.0, 2.0]),
        np.array([0.0]),
        np.array([np.inf, 1.0]),
        np.array([-1, 0], dtype=np.int32),
    )

    assert problem.get_type() == dart.LcpProblemType.INVALID
    assert not problem.is_valid()
    assert "dimensions inconsistent" in problem.get_validation_message()
    assert not problem.is_standard_lcp()
    assert not problem.is_boxed_lcp()
    assert not problem.has_friction_index()
    assert problem.get_friction_index_row_count() == 0
    assert problem.get_friction_index_contact_count() == 0
    assert not dart.DantzigSolver().supports_problem(problem)


def test_lcp_problem_rejects_invalid_matrix_dimensions() -> None:
    problem = dart.LcpProblem(np.eye(1), np.array([1.0, 2.0]))

    assert problem.get_type() == dart.LcpProblemType.INVALID
    assert not problem.is_valid()
    assert "dimensions inconsistent" in problem.get_validation_message()
    assert not problem.is_standard_lcp()
    assert not problem.is_boxed_lcp()
    assert not problem.has_friction_index()
    assert not dart.DantzigSolver().supports_problem(problem)


@pytest.mark.parametrize(
    ("lo", "hi"),
    (
        (np.array([0.0, np.inf]), np.ones(2)),
        (np.zeros(2), np.array([1.0, -np.inf])),
    ),
)
def test_lcp_problem_rejects_invalid_infinite_bound_direction(
    lo: np.ndarray, hi: np.ndarray
) -> None:
    problem = dart.LcpProblem(np.eye(2), np.array([1.0, 2.0]), lo, hi)

    assert problem.get_type() == dart.LcpProblemType.INVALID
    assert not problem.is_valid()
    assert "invalid infinity direction" in problem.get_validation_message()
    assert not problem.is_boxed_lcp()
    assert not dart.DantzigSolver().supports_problem(problem)

    result, _ = dart.DantzigSolver().solve(problem)

    assert result.status == dart.LcpSolverStatus.INVALID_PROBLEM


@pytest.mark.parametrize(
    "findex",
    (
        np.array([-1, 2], dtype=np.int32),
        np.array([-1, 1], dtype=np.int32),
    ),
)
def test_lcp_problem_rejects_invalid_findex_references(
    findex: np.ndarray,
) -> None:
    problem = dart.LcpProblem(
        np.eye(2),
        np.array([1.0, 2.0]),
        np.array([0.0, -1.0]),
        np.array([np.inf, 1.0]),
        findex,
    )

    assert problem.get_type() == dart.LcpProblemType.INVALID
    assert not problem.is_valid()
    assert "Friction index" in problem.get_validation_message()
    assert not problem.is_boxed_lcp()
    assert not problem.has_friction_index()
    assert not dart.DantzigSolver().supports_problem(problem)


def test_lcp_problem_rejects_negative_findex_coefficient() -> None:
    problem = dart.LcpProblem(
        np.eye(2),
        np.array([1.0, 2.0]),
        np.array([0.0, -1.0]),
        np.array([np.inf, -0.5]),
        np.array([-1, 0], dtype=np.int32),
    )

    assert problem.get_type() == dart.LcpProblemType.INVALID
    assert not problem.is_valid()
    assert "non-negative" in problem.get_validation_message()
    assert not problem.has_friction_index()
    assert not dart.DantzigSolver().supports_problem(problem)

    result, _ = dart.DantzigSolver().solve(problem)
    assert result.status == dart.LcpSolverStatus.INVALID_PROBLEM


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
    assert solver.supports_standard_lcp() is (
        solver_type is not dart.StaggeringSolver
    )
    assert solver.supports_boxed_lcp() is supports_boxed
    assert solver.supports_friction_index() is supports_findex
    assert solver.supports_problem(problem) is (
        solver_type is not dart.StaggeringSolver
    )
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

    is_staggering = solver_type is dart.StaggeringSolver

    assert solver.supports_problem(standard) is (not is_staggering)
    assert solver.supports_problem(boxed) is (supports_boxed and not is_staggering)
    assert solver.supports_problem(findex) is supports_findex


def test_staggering_solver_reports_only_friction_blocks_as_native() -> None:
    solver = dart.StaggeringSolver()
    standard = dart.LcpProblem(np.eye(2), np.array([1.0, 2.0]))
    boxed = dart.LcpProblem(
        np.eye(2), np.array([1.0, 2.0]), np.array([-1.0, 0.0]), np.ones(2)
    )
    findex = dart.LcpProblem(
        np.eye(2),
        np.array([1.0, 2.0]),
        np.array([0.0, -1.0]),
        np.array([np.inf, 1.0]),
        np.array([-1, 0], dtype=np.int32),
    )

    assert not solver.supports_problem(standard)
    assert not solver.supports_problem(boxed)
    assert solver.supports_problem(findex)
    assert not solver.supports_standard_lcp()
    assert not solver.supports_boxed_lcp()
    assert solver.supports_friction_index()

    result, solution = solver.solve(standard)
    assert result.status == dart.LcpSolverStatus.SUCCESS
    np.testing.assert_allclose(solution, [1.0, 2.0], atol=1e-6)


def _assert_parameter_value(actual: object, expected: object) -> None:
    if isinstance(expected, float):
        assert actual == pytest.approx(expected)
    else:
        assert actual == expected


def test_parameterized_lcp_solvers_expose_dartpy_parameters() -> None:
    cases = (
        (
            dart.PgsSolver,
            dart.PgsSolverParameters,
            {"epsilon_for_division": 1e-9, "randomize_constraint_order": False},
            {"epsilon_for_division": 2e-9, "randomize_constraint_order": True},
        ),
        (
            dart.SymmetricPsorSolver,
            dart.SymmetricPsorSolverParameters,
            {"epsilon_for_division": 1e-9},
            {"epsilon_for_division": 2e-9},
        ),
        (
            dart.JacobiSolver,
            dart.JacobiSolverParameters,
            {"epsilon_for_division": 1e-9, "worker_threads": 1},
            {"epsilon_for_division": 2e-9, "worker_threads": 2},
        ),
        (
            dart.RedBlackGaussSeidelSolver,
            dart.RedBlackGaussSeidelSolverParameters,
            {"epsilon_for_division": 1e-9, "worker_threads": 1},
            {"epsilon_for_division": 2e-9, "worker_threads": 2},
        ),
        (
            dart.BlockedJacobiSolver,
            dart.BlockedJacobiSolverParameters,
            {"block_sizes": [], "worker_threads": 1},
            {"block_sizes": [1, 1], "worker_threads": 2},
        ),
        (
            dart.BgsSolver,
            dart.BgsSolverParameters,
            {"block_sizes": []},
            {"block_sizes": [1, 1]},
        ),
        (
            dart.NncgSolver,
            dart.NncgSolverParameters,
            {"pgs_iterations": 1, "restart_interval": 10, "restart_threshold": 1.0},
            {"pgs_iterations": 2, "restart_interval": 4, "restart_threshold": 0.75},
        ),
        (
            dart.SubspaceMinimizationSolver,
            dart.SubspaceMinimizationSolverParameters,
            {"pgs_iterations": 5, "active_set_tolerance": 0.0},
            {"pgs_iterations": 3, "active_set_tolerance": 1e-8},
        ),
        (
            dart.ApgdSolver,
            dart.ApgdSolverParameters,
            {
                "epsilon_for_division": 1e-9,
                "adaptive_restart": True,
                "restart_check_interval": 5,
            },
            {
                "epsilon_for_division": 2e-9,
                "adaptive_restart": False,
                "restart_check_interval": 3,
            },
        ),
        (
            dart.TgsSolver,
            dart.TgsSolverParameters,
            {"epsilon_for_division": 1e-9},
            {"epsilon_for_division": 2e-9},
        ),
        (
            dart.MinimumMapNewtonSolver,
            dart.MinimumMapNewtonSolverParameters,
            {
                "max_line_search_steps": 8,
                "step_reduction": 0.5,
                "sufficient_decrease": 1e-4,
                "min_step": 1e-6,
                "max_gradient_descent_warm_start_steps": 0,
                "max_gradient_descent_line_search_steps": 8,
                "gradient_descent_step_reduction": 0.5,
                "gradient_descent_sufficient_decrease": 1e-4,
                "gradient_descent_min_step": 1e-8,
                "max_pgs_warm_start_iterations": 0,
                "pgs_warm_start_relaxation": 1.0,
            },
            {
                "max_line_search_steps": 9,
                "step_reduction": 0.4,
                "sufficient_decrease": 2e-4,
                "min_step": 1e-7,
                "max_gradient_descent_warm_start_steps": 2,
                "max_gradient_descent_line_search_steps": 6,
                "gradient_descent_step_reduction": 0.4,
                "gradient_descent_sufficient_decrease": 2e-4,
                "gradient_descent_min_step": 2e-8,
                "max_pgs_warm_start_iterations": 3,
                "pgs_warm_start_relaxation": 0.9,
            },
        ),
        (
            dart.FischerBurmeisterNewtonSolver,
            dart.FischerBurmeisterNewtonSolverParameters,
            {
                "smoothing_epsilon": 1e-8,
                "max_line_search_steps": 8,
                "step_reduction": 0.5,
                "sufficient_decrease": 1e-4,
                "min_step": 1e-6,
                "max_gradient_descent_warm_start_steps": 0,
                "max_gradient_descent_line_search_steps": 8,
                "gradient_descent_step_reduction": 0.5,
                "gradient_descent_sufficient_decrease": 1e-4,
                "gradient_descent_min_step": 1e-8,
                "max_pgs_warm_start_iterations": 0,
                "pgs_warm_start_relaxation": 1.0,
            },
            {
                "smoothing_epsilon": 2e-8,
                "max_line_search_steps": 9,
                "step_reduction": 0.4,
                "sufficient_decrease": 2e-4,
                "min_step": 1e-7,
                "max_gradient_descent_warm_start_steps": 2,
                "max_gradient_descent_line_search_steps": 6,
                "gradient_descent_step_reduction": 0.4,
                "gradient_descent_sufficient_decrease": 2e-4,
                "gradient_descent_min_step": 2e-8,
                "max_pgs_warm_start_iterations": 3,
                "pgs_warm_start_relaxation": 0.9,
            },
        ),
        (
            dart.PenalizedFischerBurmeisterNewtonSolver,
            dart.PenalizedFischerBurmeisterNewtonSolverParameters,
            {
                "smoothing_epsilon": 1e-8,
                "lambda_": 0.5,
                "max_line_search_steps": 8,
                "step_reduction": 0.5,
                "sufficient_decrease": 1e-4,
                "min_step": 1e-6,
                "max_gradient_descent_warm_start_steps": 0,
                "max_gradient_descent_line_search_steps": 8,
                "gradient_descent_step_reduction": 0.5,
                "gradient_descent_sufficient_decrease": 1e-4,
                "gradient_descent_min_step": 1e-8,
                "max_pgs_warm_start_iterations": 0,
                "pgs_warm_start_relaxation": 1.0,
            },
            {
                "smoothing_epsilon": 2e-8,
                "lambda_": 0.4,
                "max_line_search_steps": 9,
                "step_reduction": 0.4,
                "sufficient_decrease": 2e-4,
                "min_step": 1e-7,
                "max_gradient_descent_warm_start_steps": 2,
                "max_gradient_descent_line_search_steps": 6,
                "gradient_descent_step_reduction": 0.4,
                "gradient_descent_sufficient_decrease": 2e-4,
                "gradient_descent_min_step": 2e-8,
                "max_pgs_warm_start_iterations": 3,
                "pgs_warm_start_relaxation": 0.9,
            },
        ),
        (
            dart.InteriorPointSolver,
            dart.InteriorPointSolverParameters,
            {"sigma": 0.1, "step_scale": 0.99},
            {"sigma": 0.2, "step_scale": 0.9},
        ),
        (
            dart.MprgpSolver,
            dart.MprgpSolverParameters,
            {
                "symmetry_tolerance": 1e-9,
                "epsilon_for_division": 1e-12,
                "check_positive_definite": True,
            },
            {
                "symmetry_tolerance": 1e-8,
                "epsilon_for_division": 2e-12,
                "check_positive_definite": False,
            },
        ),
        (
            dart.ShockPropagationSolver,
            dart.ShockPropagationSolverParameters,
            {"block_sizes": [], "layers": []},
            {"block_sizes": [1, 1], "layers": [[0], [1]]},
        ),
    )

    for solver_type, params_type, defaults, updated in cases:
        solver = solver_type()
        params = params_type()
        assert isinstance(solver.parameters, params_type)
        for name, expected in defaults.items():
            _assert_parameter_value(getattr(params, name), expected)

        for name, value in updated.items():
            setattr(params, name, value)
        solver.parameters = params
        round_trip = solver.parameters

        for name, expected in updated.items():
            _assert_parameter_value(getattr(round_trip, name), expected)


def test_advanced_boxed_solver_parameters_round_trip_from_dartpy_math() -> None:
    admm = dart.AdmmSolver()
    admm_params = dart.AdmmSolverParameters()
    assert admm_params.rho_init == pytest.approx(4.0)
    assert admm_params.mu_prox == pytest.approx(1e-9)
    assert admm_params.adaptive_rho_tolerance == pytest.approx(5.0)
    assert admm_params.adaptive_rho

    admm_params.rho_init = 0.35
    admm_params.mu_prox = 1e-8
    admm_params.adaptive_rho_tolerance = 2.5
    admm_params.adaptive_rho = False
    admm.parameters = admm_params
    assert admm.parameters.rho_init == pytest.approx(0.35)
    assert admm.parameters.mu_prox == pytest.approx(1e-8)
    assert admm.parameters.adaptive_rho_tolerance == pytest.approx(2.5)
    assert not admm.parameters.adaptive_rho

    sap = dart.SapSolver()
    sap_params = dart.SapSolverParameters()
    assert sap_params.regularization == pytest.approx(1e-4)
    assert sap_params.armijos_parameter == pytest.approx(1e-4)
    assert sap_params.backtracking_factor == pytest.approx(0.5)
    assert sap_params.max_line_search_iterations == 20

    sap_params.regularization = 1e-3
    sap_params.armijos_parameter = 5e-4
    sap_params.backtracking_factor = 0.25
    sap_params.max_line_search_iterations = 8
    sap.parameters = sap_params
    assert sap.parameters.regularization == pytest.approx(1e-3)
    assert sap.parameters.armijos_parameter == pytest.approx(5e-4)
    assert sap.parameters.backtracking_factor == pytest.approx(0.25)
    assert sap.parameters.max_line_search_iterations == 8

    newton = dart.BoxedSemiSmoothNewtonSolver()
    newton_params = dart.BoxedSemiSmoothNewtonSolverParameters()
    assert newton_params.max_line_search_steps == 10
    assert newton_params.step_reduction == pytest.approx(0.5)
    assert newton_params.sufficient_decrease == pytest.approx(1e-4)
    assert newton_params.min_step == pytest.approx(1e-8)
    assert newton_params.jacobian_regularization == pytest.approx(1e-10)
    assert newton_params.max_pgs_warm_start_iterations == 0
    assert newton_params.pgs_warm_start_relaxation == pytest.approx(1.0)
    assert newton_params.max_friction_index_exact_solve_dimension == 48

    newton_params.max_line_search_steps = 12
    newton_params.step_reduction = 0.35
    newton_params.sufficient_decrease = 2e-4
    newton_params.min_step = 1e-10
    newton_params.jacobian_regularization = 1e-8
    newton_params.max_pgs_warm_start_iterations = 5
    newton_params.pgs_warm_start_relaxation = 0.9
    newton_params.max_friction_index_exact_solve_dimension = 192
    newton.parameters = newton_params
    assert newton.parameters.max_line_search_steps == 12
    assert newton.parameters.step_reduction == pytest.approx(0.35)
    assert newton.parameters.sufficient_decrease == pytest.approx(2e-4)
    assert newton.parameters.min_step == pytest.approx(1e-10)
    assert newton.parameters.jacobian_regularization == pytest.approx(1e-8)
    assert newton.parameters.max_pgs_warm_start_iterations == 5
    assert newton.parameters.pgs_warm_start_relaxation == pytest.approx(0.9)
    assert newton.parameters.max_friction_index_exact_solve_dimension == 192


@pytest.mark.parametrize(
    ("solver", "params"),
    (
        (dart.AdmmSolver(), dart.AdmmSolverParameters()),
        (dart.SapSolver(), dart.SapSolverParameters()),
        (
            dart.BoxedSemiSmoothNewtonSolver(),
            dart.BoxedSemiSmoothNewtonSolverParameters(),
        ),
    ),
)
def test_customized_advanced_boxed_solvers_solve_boxed_problem(
    solver: dart.LcpSolver, params: object
) -> None:
    if isinstance(params, dart.AdmmSolverParameters):
        params.rho_init = 0.45
        params.mu_prox = 1e-8
        params.adaptive_rho_tolerance = 2.0
    elif isinstance(params, dart.SapSolverParameters):
        params.regularization = 1e-3
        params.max_line_search_iterations = 12
    elif isinstance(params, dart.BoxedSemiSmoothNewtonSolverParameters):
        params.max_line_search_steps = 16
        params.jacobian_regularization = 1e-8
        params.max_pgs_warm_start_iterations = 5
        params.max_friction_index_exact_solve_dimension = 192
    solver.parameters = params

    problem = dart.LcpProblem(
        np.eye(2), np.array([0.25, 0.75]), np.zeros(2), np.ones(2)
    )
    options = dart.LcpOptions.high_accuracy()
    options.max_iterations = 120
    options.validate_solution = False

    result, solution = solver.solve(problem, options=options)

    assert solver.supports_problem(problem)
    assert result.status in (
        dart.LcpSolverStatus.SUCCESS,
        dart.LcpSolverStatus.MAX_ITERATIONS,
    )
    assert solution.shape == (2,)
    assert np.all(solution >= problem.lo - 1e-8)
    assert np.all(solution <= problem.hi + 1e-8)


@pytest.mark.parametrize(
    ("solver", "params", "expected_message"),
    (
        (dart.AdmmSolver(), dart.AdmmSolverParameters(), "rho_init"),
        (dart.SapSolver(), dart.SapSolverParameters(), "regularization"),
        (dart.PgsSolver(), dart.PgsSolverParameters(), "epsilon_for_division"),
        (dart.ApgdSolver(), dart.ApgdSolverParameters(), "restart_check_interval"),
        (dart.TgsSolver(), dart.TgsSolverParameters(), "epsilon_for_division"),
        (
            dart.MinimumMapNewtonSolver(),
            dart.MinimumMapNewtonSolverParameters(),
            "max_line_search_steps",
        ),
        (
            dart.FischerBurmeisterNewtonSolver(),
            dart.FischerBurmeisterNewtonSolverParameters(),
            "smoothing_epsilon",
        ),
        (
            dart.PenalizedFischerBurmeisterNewtonSolver(),
            dart.PenalizedFischerBurmeisterNewtonSolverParameters(),
            "lambda",
        ),
        (
            dart.InteriorPointSolver(),
            dart.InteriorPointSolverParameters(),
            "step_scale",
        ),
        (
            dart.BoxedSemiSmoothNewtonSolver(),
            dart.BoxedSemiSmoothNewtonSolverParameters(),
            "max_line_search_steps",
        ),
    ),
)
def test_advanced_boxed_solvers_reject_invalid_parameters_from_dartpy_math(
    solver: dart.LcpSolver, params: object, expected_message: str
) -> None:
    if isinstance(params, dart.AdmmSolverParameters):
        params.rho_init = 0.0
    elif isinstance(params, dart.SapSolverParameters):
        params.regularization = 0.0
    elif isinstance(params, dart.PgsSolverParameters):
        params.epsilon_for_division = 0.0
    elif isinstance(params, dart.ApgdSolverParameters):
        params.restart_check_interval = -1
    elif isinstance(params, dart.TgsSolverParameters):
        params.epsilon_for_division = 0.0
    elif isinstance(params, dart.MinimumMapNewtonSolverParameters):
        params.max_line_search_steps = 0
    elif isinstance(params, dart.FischerBurmeisterNewtonSolverParameters):
        params.smoothing_epsilon = 0.0
    elif isinstance(params, dart.PenalizedFischerBurmeisterNewtonSolverParameters):
        params.lambda_ = 0.0
    elif isinstance(params, dart.InteriorPointSolverParameters):
        params.step_scale = 0.0
    elif isinstance(params, dart.BoxedSemiSmoothNewtonSolverParameters):
        params.max_line_search_steps = 0
    solver.parameters = params

    problem = dart.LcpProblem(np.eye(2), np.array([0.25, 0.75]))
    result, _ = solver.solve(problem)

    assert result.status == dart.LcpSolverStatus.INVALID_PROBLEM
    assert expected_message in result.message


def test_lcp_solver_supports_problem_uses_default_standard_tolerance() -> None:
    A = np.eye(2)
    near_standard = dart.LcpProblem(
        A,
        np.array([1.0, 2.0]),
        np.array([1e-9, -1e-9]),
        np.array([np.inf, np.inf]),
    )

    assert near_standard.get_type() == dart.LcpProblemType.BOXED
    assert near_standard.get_type(1e-8) == dart.LcpProblemType.STANDARD

    solver = dart.LemkeSolver()
    assert not solver.supports_boxed_lcp()
    assert solver.supports_problem(near_standard)
    assert not solver.supports_problem(near_standard, standard_tolerance=0.0)
    assert solver.supports_problem(near_standard, standard_tolerance=1e-8)


def test_direct_solver_reports_only_tiny_standard_problems_as_native() -> None:
    small = dart.LcpProblem(np.eye(3), np.array([1.0, 2.0, 3.0]))
    large = dart.LcpProblem(np.eye(4), np.array([1.0, 2.0, 3.0, 4.0]))

    solver = dart.DirectSolver()
    assert solver.supports_standard_lcp()
    assert solver.supports_problem(small)
    assert not solver.supports_problem(large)

    result, solution = solver.solve(large)
    assert result.status == dart.LcpSolverStatus.SUCCESS
    np.testing.assert_allclose(solution, np.array([1.0, 2.0, 3.0, 4.0]))


def test_mprgp_solver_reports_only_spd_standard_problems_as_native() -> None:
    spd = dart.LcpProblem(np.eye(2), np.array([1.0, 2.0]))
    nonsymmetric = dart.LcpProblem(
        np.array([[2.0, 1.0], [0.0, 2.0]]),
        np.array([1.0, 2.0]),
    )
    indefinite = dart.LcpProblem(
        np.array([[1.0, 0.0], [0.0, -1.0]]),
        np.array([1.0, -1.0]),
    )
    boxed = dart.LcpProblem(
        np.eye(2), np.array([1.0, 2.0]), np.array([-1.0, 0.0]), np.ones(2)
    )

    solver = dart.MprgpSolver()
    assert solver.supports_standard_lcp()
    assert solver.supports_problem(spd)
    assert not solver.supports_problem(nonsymmetric)
    assert not solver.supports_problem(indefinite)
    assert not solver.supports_problem(boxed)


def test_baraff_solver_reports_only_psd_standard_problems_as_native() -> None:
    spd = dart.LcpProblem(np.eye(2), np.array([1.0, 2.0]))
    psd = dart.LcpProblem(np.array([[1.0, 0.0], [0.0, 0.0]]), np.array([1.0, 0.0]))
    nonsymmetric = dart.LcpProblem(
        np.array([[2.0, 1.0], [0.0, 2.0]]),
        np.array([1.0, 2.0]),
    )
    indefinite = dart.LcpProblem(
        np.array([[1.0, 0.0], [0.0, -1.0]]),
        np.array([1.0, -1.0]),
    )
    boxed = dart.LcpProblem(
        np.eye(2), np.array([1.0, 2.0]), np.array([-1.0, 0.0]), np.ones(2)
    )

    solver = dart.BaraffSolver()
    assert solver.supports_standard_lcp()
    assert solver.supports_problem(spd)
    assert solver.supports_problem(psd)
    assert not solver.supports_problem(nonsymmetric)
    assert not solver.supports_problem(indefinite)
    assert not solver.supports_problem(boxed)

    result, solution = solver.solve(indefinite)
    assert result.status == dart.LcpSolverStatus.SUCCESS
    w = indefinite.A @ solution - indefinite.b
    assert np.all(solution >= -1e-8)
    assert np.all(w >= -1e-8)
    assert np.linalg.norm(solution * w, ord=np.inf) <= 1e-8


def test_lcp_solver_rejects_wrong_initial_guess_size() -> None:
    problem = dart.LcpProblem(np.eye(2), np.array([1.0, 2.0]))
    solver = dart.DantzigSolver()

    with pytest.raises(ValueError, match="initial_guess"):
        solver.solve(problem, initial_guess=np.zeros(3))


def test_lcp_solver_rejects_non_finite_problem_data() -> None:
    A = np.eye(2)
    A[0, 1] = np.nan
    problem = dart.LcpProblem(A, np.array([1.0, 2.0]))

    assert problem.get_type() == dart.LcpProblemType.INVALID
    assert not problem.is_valid()
    assert "non-finite" in problem.get_validation_message()
    assert not problem.is_standard_lcp()
    assert not problem.is_boxed_lcp()
    assert not problem.has_friction_index()
    assert not dart.DantzigSolver().supports_problem(problem)

    result, _ = dart.DantzigSolver().solve(problem)

    assert result.status == dart.LcpSolverStatus.INVALID_PROBLEM
    assert "non-finite" in result.message
