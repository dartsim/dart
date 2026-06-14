"""Side-by-side DART 7 contact comparison for boxed-LCP physics.

The scene runs two matched ``World`` instances over the same representative
contact packets: sliding friction, an inclined static-friction hold, a billiard
collision, a high-mass-ratio stack, and a thin card pile. One world uses the
default sequential impulse contact path; the other uses the boxed-LCP contact
path.
"""

from __future__ import annotations

import csv
import math
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 0.002
_RAMP_TILT_RAD = math.radians(12.0)
_HISTORY = 180
_BENCHMARK_SMOKE_FILTER = "BM_LCP_COMPARE_SMOKE"
_BENCHMARK_COMMAND = (
    f"pixi run bm lcp_compare -- --benchmark_filter={_BENCHMARK_SMOKE_FILTER}"
)
_STANDALONE_LCP_SOLVERS_EXPOSED_IN_DARTPY = True
_STANDALONE_SMOKE_EXPECTED = np.array([1.0, 0.5, 2.0, 1.5])
_STANDALONE_SUCCESS_STATUSES = {"Success", "MaxIterations"}
_STANDALONE_PROBLEM_SUITE_LABEL = "Representative solver suite"

_LIVE_PACKET_ROWS: tuple[dict[str, str], ...] = (
    {
        "packet": "Sliding friction",
        "metric": "sliding speed, contacts",
        "benchmark": "world contact friction-index packets",
    },
    {
        "packet": "Static-friction ramp",
        "metric": "ramp-parallel slide",
        "benchmark": "active-set transition packets",
    },
    {
        "packet": "Billiard collision",
        "metric": "symmetry, momentum, kinetic-energy error",
        "benchmark": "contact solver invariant packets",
    },
    {
        "packet": "High-mass-ratio stack",
        "metric": "stack drift, step time",
        "benchmark": "world stack scale packets",
    },
    {
        "packet": "Thin card pile",
        "metric": "card spread, height loss",
        "benchmark": "world card-pile packets",
    },
)

_REPRESENTATIVE_REQUIREMENT_ROWS: tuple[dict[str, str], ...] = (
    {
        "requirement": "Billiard symmetry, energy, momentum",
        "live_packet": "Billiard collision",
        "benchmark_packet": "world_billiards",
        "metrics": "symmetry error, momentum error, kinetic-energy error",
        "evidence": "live plots and BM_LcpWorldBilliardsStep_BoxedLcp",
    },
    {
        "requirement": "High mass-ratio stack",
        "live_packet": "High-mass-ratio stack",
        "benchmark_packet": "world_stack, mass_ratio_boxed",
        "metrics": "stack drift, step time, active-bound residuals",
        "evidence": "world stack packets plus standalone mass-ratio boxed row",
    },
    {
        "requirement": "Thin card pile",
        "live_packet": "Thin card pile",
        "benchmark_packet": "world_card_pile",
        "metrics": "card spread, height loss, step time",
        "evidence": "live card metrics and BM_LcpWorldCardPileStep_BoxedLcp",
    },
    {
        "requirement": "Scalability smoke",
        "live_packet": _STANDALONE_PROBLEM_SUITE_LABEL,
        "benchmark_packet": "batch_scale, moderate_scale_standard",
        "metrics": "lcp_dimension, elapsed time, residual, complementarity",
        "evidence": "standalone moderate-scale row plus batch benchmark packets",
    },
    {
        "requirement": "Friction coupling and active tangential bounds",
        "live_packet": "Sliding friction, Static-friction ramp",
        "benchmark_packet": "active_friction_index_contact",
        "metrics": "slide speed, ramp slide, bound violation, contact count",
        "evidence": "FrictionIndex evidence rows and active tangential-bound case",
    },
)

_BENCHMARK_PACKET_ROWS: tuple[dict[str, str], ...] = (
    {
        "packet": "standard_spd_smoke",
        "surface": "standard LCP",
        "benchmark_filter": "BM_LcpCompare/Standard",
        "coverage": "small SPD contract smoke",
    },
    {
        "packet": "active_set_transition",
        "surface": "boxed/findex",
        "benchmark_filter": (
            "BM_LcpActiveSetTransition|"
            "BM_LcpNewtonWarmStart|"
            "BM_LcpNewtonWarmStartBatchSerial|"
            "BM_LcpNewtonWarmStartBatchParallel"
        ),
        "coverage": "bound changes, warm-start behavior, and warm-start batches",
    },
    {
        "packet": "active_set_scale",
        "surface": "standard/boxed/findex",
        "benchmark_filter": (
            "BM_LcpLargerActiveSetTransition|"
            "BM_LcpStressActiveSetTransition|"
            "BM_LcpExtremeActiveSetTransition|"
            "BM_LcpProductionActiveSetTransition|"
            "BM_LcpProductionActiveSetTransitionBatchSerial|"
            "BM_LcpProductionActiveSetTransitionBatchParallel"
        ),
        "coverage": "active-set scaling and production batches",
    },
    {
        "packet": "active_friction_index_contact",
        "surface": "findex contact",
        "benchmark_filter": "BM_LcpActiveFrictionIndexContact",
        "coverage": "coupled two-contact active tangent bounds",
    },
    {
        "packet": "contact_solver_comparison_sweep",
        "surface": "findex contact",
        "benchmark_filter": (
            "BM_LcpContactSolverComparisonSweep|"
            "BM_LcpStaggeringContactPipelineSweep"
        ),
        "coverage": "DART 7 contact-pipeline solver families",
    },
    {
        "packet": "contact_normal_standard_sweep",
        "surface": "standard contact",
        "benchmark_filter": "BM_LcpContactNormalStandardSweep",
        "coverage": "normal-only contact subproblems",
    },
    {
        "packet": "validation_friction_index",
        "surface": "findex validation",
        "benchmark_filter": (
            "BM_LcpValidation_Serial_FrictionIndex|"
            "BM_LcpValidation_Threaded_FrictionIndex"
        ),
        "coverage": "serial and threaded residual/complementarity validation",
    },
    {
        "packet": "singular_degenerate",
        "surface": "standard/boxed/findex",
        "benchmark_filter": "BM_LcpSingularDegenerate",
        "coverage": "rank-deficient and degenerate complementarity",
    },
    {
        "packet": "singular_degenerate_scale",
        "surface": "standard/boxed/findex",
        "benchmark_filter": (
            "BM_LcpLargerSingularDegenerate|"
            "BM_LcpStressSingularDegenerate|"
            "BM_LcpExtremeSingularDegenerate|"
            "BM_LcpSingularDegenerateFrictionIndexBatchSerial|"
            "BM_LcpSingularDegenerateFrictionIndexBatchParallel|"
            "BM_LcpSingularDegenerateStandardBoxedBatchSerial|"
            "BM_LcpSingularDegenerateStandardBoxedBatchParallel"
        ),
        "coverage": "rank-deficient scaling and batch degeneracy",
    },
    {
        "packet": "near_singular",
        "surface": "standard/boxed",
        "benchmark_filter": (
            "BM_LcpNearSingular|"
            "BM_LcpNearSingularBatchSerial|"
            "BM_LcpNearSingularBatchParallel"
        ),
        "coverage": "conditioning, regularization, and batch pressure",
    },
    {
        "packet": "mild_ill_conditioned",
        "surface": "standard/boxed/findex",
        "benchmark_filter": (
            "BM_LcpMildIllConditioned|"
            "BM_LcpMildIllConditionedBatchSerial|"
            "BM_LcpMildIllConditionedBatchParallel"
        ),
        "coverage": "mild conditioning, coupling, and batch conditioning pressure",
    },
    {
        "packet": "batch_scale",
        "surface": "standard/boxed/findex",
        "benchmark_filter": "BM_LcpBatch|BM_LcpGroupedBatch",
        "coverage": "parallel scalability and grouped batches",
    },
    {
        "packet": "cuda_batch_scale",
        "surface": "cuda standard/boxed/findex",
        "benchmark_filter": (
            "BM_LcpCudaJacobiBatch|"
            "BM_LcpCudaPgsBatch|"
            "BM_LcpCudaJacobiGroupedBatch|"
            "BM_LcpCudaPgsGroupedBatch"
        ),
        "coverage": "CUDA Jacobi/PGS batch and grouped-batch solves",
    },
    {
        "packet": "solver_parameter_sweeps",
        "surface": "standard/boxed/findex",
        "benchmark_filter": (
            "BM_LcpPgsRelaxationSweep|"
            "BM_LcpSymmetricPsorRelaxationSweep|"
            "BM_LcpJacobiSolverThreading|"
            "BM_LcpRedBlackGaussSeidelRelaxationSweep|"
            "BM_LcpRedBlackGaussSeidelSolverThreadingBanded|"
            "BM_LcpBlockedJacobiSolverThreadingBanded|"
            "BM_LcpBoxedSemiSmoothNewtonLineSearchSweep|"
            "BM_LcpPivotingScaleSweep|"
            "BM_LcpBlockPartitionSweep|"
            "BM_LcpApgdRestartSweep|"
            "BM_LcpTgsIterationBudgetSweep|"
            "BM_LcpNncgPgsIterationsSweep|"
            "BM_LcpSubspaceMinimizationPgsIterationsSweep|"
            "BM_LcpShockPropagationLayerSweep|"
            "BM_LcpNewtonWarmStart|"
            "BM_LcpMprgpSpdCheckSweep|"
            "BM_LcpInteriorPointPathSweep|"
            "BM_LcpAdmmRhoSweep|"
            "BM_LcpSapRegularizationSweep"
        ),
        "coverage": "solver-specific tuning and robustness sweeps",
    },
    {
        "packet": "world_contact",
        "surface": "findex contact",
        "benchmark_filter": "BM_LcpWorldContact|BM_LcpWorldBoxContact",
        "coverage": "assembled contact packets from simulation worlds",
    },
    {
        "packet": "world_contact_step",
        "surface": "boxed contact",
        "benchmark_filter": (
            "BM_LcpWorldSeparatedStep_BoxedLcp|BM_LcpWorldBoxStep_BoxedLcp"
        ),
        "coverage": "separated-contact and dense-box world step invariants",
    },
    {
        "packet": "world_billiards",
        "surface": "boxed contact",
        "benchmark_filter": "BM_LcpWorldBilliardsStep_BoxedLcp",
        "coverage": "billiards symmetry, momentum, and energy invariants",
    },
    {
        "packet": "world_stack",
        "surface": "findex/boxed contact",
        "benchmark_filter": (
            "BM_LcpWorldStackContact/|BM_LcpWorldStackStep_BoxedLcp"
        ),
        "coverage": "mass-ratio stack contact scaling",
    },
    {
        "packet": "world_card_pile",
        "surface": "boxed contact",
        "benchmark_filter": "BM_LcpWorldCardPileStep_BoxedLcp",
        "coverage": "thin high-aspect-ratio stacked contact scaling",
    },
    {
        "packet": "articulated_contact",
        "surface": "findex contact",
        "benchmark_filter": "BM_LcpWorldArticulated|BM_LcpArticulatedUnifiedContact",
        "coverage": "robot-link contact assembly and solve cost",
    },
    {
        "packet": "cuda_contact_batch",
        "surface": "cuda findex contact",
        "benchmark_filter": (
            "BM_LcpCudaJacobiWorldContactBatch|"
            "BM_LcpCudaPgsWorldContactBatch|"
            "BM_LcpCudaJacobiWorldContactGroupedBatch|"
            "BM_LcpCudaPgsWorldContactGroupedBatch|"
            "BM_LcpCudaJacobiWorldBoxContactBatch|"
            "BM_LcpCudaPgsWorldBoxContactBatch|"
            "BM_LcpCudaJacobiWorldBoxContactGroupedBatch|"
            "BM_LcpCudaPgsWorldBoxContactGroupedBatch|"
            "BM_LcpCudaJacobiWorldStackContactBatch|"
            "BM_LcpCudaPgsWorldStackContactBatch|"
            "BM_LcpCudaJacobiWorldStackContactGroupedBatch|"
            "BM_LcpCudaPgsWorldStackContactGroupedBatch|"
            "BM_LcpCudaJacobiArticulatedUnifiedContactGroupedBatch|"
            "BM_LcpCudaPgsArticulatedUnifiedContactGroupedBatch|"
            "BM_LcpCudaJacobiMixedContactGroupedBatch|"
            "BM_LcpCudaPgsMixedContactGroupedBatch"
        ),
        "coverage": "CUDA world/contact batches across representative packets",
    },
)


def _benchmark_filter_union(rows: tuple[dict[str, str], ...]) -> str:
    tokens: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for token in row["benchmark_filter"].split("|"):
            if token and token not in seen:
                seen.add(token)
                tokens.append(token)
    return "|".join(tokens)


_REPRESENTATIVE_BENCHMARK_FILTER = _benchmark_filter_union(_BENCHMARK_PACKET_ROWS)
_REPRESENTATIVE_BENCHMARK_COMMAND = (
    "pixi run bm lcp_compare -- --benchmark_filter="
    f"'{_REPRESENTATIVE_BENCHMARK_FILTER}'"
)
_PERFORMANCE_PROFILE_REFRESH_COMMAND = (
    "pixi run python scripts/lcp_performance_profile.py --run "
    "--cache build/lcp_profile_full.json "
    "--output docs/background/lcp/figures "
    "--benchmark-timeout 900"
)
_PERFORMANCE_PROFILE_SMOKE_COMMAND = (
    "pixi run python scripts/lcp_performance_profile.py --run "
    "--allow-partial "
    "--benchmark-filter BM_LcpCompare/Standard/Dantzig/12 "
    "--benchmark-min-time 0.01 "
    "--cache build/lcp_profile_smoke.json "
    "--output build/lcp_profile_smoke "
    "--benchmark-timeout 120"
)
_PERFORMANCE_PROFILE_EVIDENCE_ARTIFACT = (
    "docs/background/lcp/figures/performance_profile_evidence.csv"
)
_SOURCE_ROOT = Path(__file__).resolve().parents[4]
_PERFORMANCE_PROFILE_EVIDENCE_PATH = (
    _SOURCE_ROOT / _PERFORMANCE_PROFILE_EVIDENCE_ARTIFACT
)
_PERFORMANCE_PROFILE_EVIDENCE_SCHEMA_ROWS: tuple[dict[str, str], ...] = (
    {
        "fields": "category, solver, problem_size",
        "meaning": "Profile surface, solver name, and benchmark problem size.",
    },
    {
        "fields": "lcp_dimension, contact_count",
        "meaning": "Linear system dimension plus FrictionIndex contact count.",
    },
    {
        "fields": "solver_identity_schema_version, solver_manifest_index",
        "meaning": "Roster-stable solver identity emitted by the C++ benchmark.",
    },
    {
        "fields": (
            "solver_family_pivoting, solver_family_projection, "
            "solver_family_newton, solver_family_other"
        ),
        "meaning": "One-hot solver family identity for apples-to-apples grouping.",
    },
    {
        "fields": "time_ns",
        "meaning": "Measured benchmark time used for profile ratios.",
    },
    {
        "fields": "contract_ok, iterations",
        "meaning": "Per-row solver contract result and iteration count.",
    },
    {
        "fields": "residual, complementarity, bound_violation",
        "meaning": "Numerical feasibility metrics required by the roster guard.",
    },
    {
        "fields": (
            "solver_supports_standard, solver_supports_boxed, "
            "solver_supports_friction_index, solver_supports_problem"
        ),
        "meaning": "Concrete native support flags for each solver and problem row.",
    },
    {
        "fields": (
            "problem_type_standard, problem_type_boxed, "
            "problem_type_friction_index, problem_type_invalid"
        ),
        "meaning": "Problem-form counters checked against the benchmark category.",
    },
)
_PERFORMANCE_PROFILE_ROWS: tuple[dict[str, str], ...] = (
    {
        "surface": "Standard",
        "artifact": "docs/background/lcp/figures/performance_profile_standard.csv",
        "evidence_artifact": _PERFORMANCE_PROFILE_EVIDENCE_ARTIFACT,
        "problem_sizes": "2, 3, 12, 24, 48, 96",
        "current_leaders": (
            "Direct/Dantzig/InteriorPoint/BGS/Jacobi/BlockedJacobi/Apgd/"
            "Sap/SubspaceMinimization/SymmetricPsor/Admm/Pgs/Tgs and "
            "strict-interior FischerBurmeisterNewton/MinimumMapNewton/"
            "PenalizedFischerBurmeisterNewton rows"
        ),
        "current_laggards": (
            "No Standard solver average is above 1.6x; MPRGP and "
            "RedBlackGaussSeidel are the largest rows; "
            "BoxedSemiSmoothNewton, NNCG, Baraff, and ShockPropagation are "
            "the next largest rows"
        ),
        "takeaway": (
            "Strict-interior linear solves remove the old pivot, barrier, "
            "Dantzig, Newton-family including boxed semi-smooth, projection, "
            "MPRGP, ADMM, APGD/Jacobi, ShockPropagation, and small/medium "
            "block hot rows. The LLT-first Newton/interior-point exact helper "
            "plus Dantzig, Lemke, Baraff, Symmetric PSOR, "
            "SubspaceMinimization, BGS, NNCG, BlockedJacobi, and RedBlack "
            "Gauss-Seidel standard exact paths, plus APGD, SAP, and a raised "
            "Jacobi exact gate, keep every Standard average below 1.6x."
        ),
    },
    {
        "surface": "Boxed",
        "artifact": "docs/background/lcp/figures/performance_profile_boxed.csv",
        "evidence_artifact": _PERFORMANCE_PROFILE_EVIDENCE_ARTIFACT,
        "problem_sizes": "12, 24, 48",
        "current_leaders": (
            "Pgs/Tgs/Jacobi/Dantzig/RedBlackGaussSeidel/Apgd/SymmetricPsor "
            "are the current leading group"
        ),
        "current_laggards": (
            "Sap is the only Boxed solver average above 1.6x; "
            "ShockPropagation, Admm, BGS, BlockedJacobi, "
            "BoxedSemiSmoothNewton, SubspaceMinimization, and NNCG are the "
            "next largest rows"
        ),
        "takeaway": (
            "Projection methods and validated exact paths now lead or closely "
            "trail active-bound rows; delayed ShockPropagation reset and the "
            "LLT-first boxed exact helper reduced several boxed rows, while "
            "the refreshed Boxed RedBlackGaussSeidel and Symmetric PSOR rows "
            "moved out of the largest group. SAP, ShockPropagation, ADMM, "
            "and block/subspace rows remain the next profile targets."
        ),
    },
    {
        "surface": "FrictionIndex",
        "artifact": (
            "docs/background/lcp/figures/performance_profile_frictionindex.csv"
        ),
        "evidence_artifact": _PERFORMANCE_PROFILE_EVIDENCE_ARTIFACT,
        "problem_sizes": "4, 16, 64",
        "current_leaders": (
            "Tgs/Pgs/Staggering/Jacobi/Dantzig/RedBlackGaussSeidel/"
            "Apgd/NNCG are the current leading group"
        ),
        "current_laggards": (
            "Sap and ShockPropagation are the current FrictionIndex averages "
            "above 1.6x; SubspaceMinimization, BlockedJacobi, "
            "SymmetricPsor, BoxedSemiSmoothNewton, Admm, and BGS are the next "
            "largest rows"
        ),
        "takeaway": (
            "Validated interior friction-index fast paths removed most block, "
            "staggering, and subspace hot rows, and the boxed semi-smooth "
            "line-search shortcut plus configurable exact shortcut trim Newton "
            "iterations; the refreshed FrictionIndex profile now points at "
            "SAP and ShockPropagation as the remaining above-1.6x rows, with "
            "subspace, block, Symmetric PSOR, boxed Newton, ADMM, and BGS "
            "forming the next group."
        ),
    },
)
_PROFILE_CATEGORY_SUPPORT_FIELDS = {
    "Standard": "solver_supports_standard",
    "Boxed": "solver_supports_boxed",
    "FrictionIndex": "solver_supports_friction_index",
}
_PROFILE_EVIDENCE_REQUIRED_SURFACES = tuple(_PROFILE_CATEGORY_SUPPORT_FIELDS)
_PROFILE_SOLVER_SUPPORT_FIELDS = {
    "solver_supports_standard": "standard",
    "solver_supports_boxed": "boxed",
    "solver_supports_friction_index": "findex",
}
_PROFILE_CATEGORY_PROBLEM_TYPE_FIELDS = {
    "Standard": "problem_type_standard",
    "Boxed": "problem_type_boxed",
    "FrictionIndex": "problem_type_friction_index",
}
_PROFILE_PROBLEM_TYPE_FIELDS = tuple(_PROFILE_CATEGORY_PROBLEM_TYPE_FIELDS.values()) + (
    "problem_type_invalid",
)
_SOLVER_IDENTITY_SCHEMA_VERSION = 1
_PROFILE_SOLVER_FAMILY_COUNTER_BY_FAMILY = {
    "Pivoting": "solver_family_pivoting",
    "Projection": "solver_family_projection",
    "Newton": "solver_family_newton",
    "Other": "solver_family_other",
}
_PROFILE_SOLVER_FAMILY_COUNTER_FIELDS = tuple(
    _PROFILE_SOLVER_FAMILY_COUNTER_BY_FAMILY.values()
)
_PERFORMANCE_PROFILE_EVIDENCE_REQUIRED_COLUMNS = (
    "category",
    "solver",
    "problem_size",
    "lcp_dimension",
    "contact_count",
    "solver_identity_schema_version",
    "solver_manifest_index",
    *_PROFILE_SOLVER_FAMILY_COUNTER_FIELDS,
    "time_ns",
    "contract_ok",
    "iterations",
    "residual",
    "complementarity",
    "bound_violation",
    "solver_supports_standard",
    "solver_supports_boxed",
    "solver_supports_friction_index",
    "solver_supports_problem",
    *tuple(_PROFILE_CATEGORY_PROBLEM_TYPE_FIELDS.values()),
    "problem_type_invalid",
)


def _format_evidence_int_values(values: set[int]) -> str:
    if not values:
        return "-"
    return ", ".join(str(value) for value in sorted(values))


def _as_int_counter(row: dict[str, str], name: str) -> int | None:
    value = row.get(name, "")
    if not value:
        return None
    try:
        numeric = float(value)
    except ValueError:
        return None
    if not math.isfinite(numeric):
        return None
    rounded = int(round(numeric))
    if abs(numeric - rounded) > 1e-9:
        return None
    return rounded


def _as_finite_float_counter(row: dict[str, str], name: str) -> float | None:
    value = row.get(name, "")
    if not value:
        return None
    try:
        numeric = float(value)
    except ValueError:
        return None
    if not math.isfinite(numeric):
        return None
    return numeric


def _as_float_counter(row: dict[str, str], name: str) -> float:
    value = _as_finite_float_counter(row, name)
    if value is None:
        return 0.0
    return value


def _validate_positive_evidence_int(
    row: dict[str, str],
    surface: str,
    solver: str,
    field: str,
) -> int:
    value = _as_int_counter(row, field)
    if value is None or value <= 0:
        raise RuntimeError(
            "invalid LCP performance profile evidence row: "
            f"{surface}/{solver} has {field}={row.get(field, '')!r}"
        )
    return value


def _validate_finite_nonnegative_evidence_float(
    row: dict[str, str],
    surface: str,
    solver: str,
    field: str,
) -> float:
    value = _as_finite_float_counter(row, field)
    if value is None or value < 0.0:
        raise RuntimeError(
            "invalid LCP performance profile evidence row: "
            f"{surface}/{solver} has {field}={row.get(field, '')!r}"
        )
    return value


def _validate_performance_profile_evidence_row(
    row: dict[str, str],
) -> tuple[str, str, int]:
    surface = row.get("category", "")
    solver = row.get("solver") or "<unknown>"
    support_field = _PROFILE_CATEGORY_SUPPORT_FIELDS.get(surface)
    problem_type_field = _PROFILE_CATEGORY_PROBLEM_TYPE_FIELDS.get(surface)
    if support_field is None or problem_type_field is None:
        raise RuntimeError(
            "unknown LCP performance profile evidence category: "
            f"{surface!r}"
        )
    solver_support_rows_by_name = {
        support_row["name"]: support_row for support_row in _SOLVER_SUPPORT_ROWS
    }
    solver_support_row = solver_support_rows_by_name.get(solver)
    if solver_support_row is None:
        raise RuntimeError(
            "unknown LCP performance profile evidence solver: "
            f"{solver!r}"
        )
    solver_manifest_index_by_name = {
        support_row["name"]: index
        for index, support_row in enumerate(_SOLVER_SUPPORT_ROWS, start=1)
    }
    identity_version = _as_int_counter(row, "solver_identity_schema_version")
    if identity_version != _SOLVER_IDENTITY_SCHEMA_VERSION:
        raise RuntimeError(
            "mismatched LCP performance profile evidence row: "
            f"{surface}/{solver} has solver_identity_schema_version="
            f"{identity_version}; expected {_SOLVER_IDENTITY_SCHEMA_VERSION}"
        )
    manifest_index = _as_int_counter(row, "solver_manifest_index")
    expected_manifest_index = solver_manifest_index_by_name[solver]
    if manifest_index != expected_manifest_index:
        raise RuntimeError(
            "mismatched LCP performance profile evidence row: "
            f"{surface}/{solver} has solver_manifest_index={manifest_index}; "
            f"expected {expected_manifest_index}"
        )
    solver_family = solver_support_row["family"]
    expected_family_counter = _PROFILE_SOLVER_FAMILY_COUNTER_BY_FAMILY.get(
        solver_family
    )
    if expected_family_counter is None:
        raise RuntimeError(
            "unknown LCP solver family in demo manifest: "
            f"{solver}/{solver_family}"
        )
    family_counter_sum = 0
    for field in _PROFILE_SOLVER_FAMILY_COUNTER_FIELDS:
        value = _as_int_counter(row, field)
        expected = 1 if field == expected_family_counter else 0
        if value == 1:
            family_counter_sum += 1
        if value != expected:
            raise RuntimeError(
                "mismatched LCP performance profile evidence row: "
                f"{surface}/{solver} has {field}={value}; expected "
                f"{expected} for {solver_family}"
            )
    if family_counter_sum != 1:
        raise RuntimeError(
            "mismatched LCP performance profile evidence row: "
            f"{surface}/{solver} solver family counters are not one-hot"
        )
    for field, solver_support_key in _PROFILE_SOLVER_SUPPORT_FIELDS.items():
        value = _as_int_counter(row, field)
        expected = 1 if solver_support_row[solver_support_key] else 0
        if value != expected:
            raise RuntimeError(
                "mismatched LCP performance profile evidence row: "
                f"{surface}/{solver} has {field}={value}; expected {expected}"
            )
    if _as_int_counter(row, support_field) != 1:
        raise RuntimeError(
            "non-native LCP performance profile evidence row: "
            f"{surface}/{solver} has {support_field}=0"
        )
    if _as_int_counter(row, "solver_supports_problem") != 1:
        raise RuntimeError(
            "unsupported LCP performance profile evidence row: "
            f"{surface}/{solver} has solver_supports_problem=0"
        )
    problem_type_sum = 0
    for field in _PROFILE_PROBLEM_TYPE_FIELDS:
        value = _as_int_counter(row, field)
        expected = 1 if field == problem_type_field else 0
        if value == 1:
            problem_type_sum += 1
        if value != expected:
            raise RuntimeError(
                "mismatched LCP performance profile evidence row: "
                f"{surface}/{solver} has {field}={value}"
            )
    if problem_type_sum != 1:
        raise RuntimeError(
            "mismatched LCP performance profile evidence row: "
            f"{surface}/{solver} problem type counters are not one-hot"
        )
    problem_size = _validate_positive_evidence_int(
        row, surface, solver, "problem_size"
    )
    lcp_dimension = _validate_positive_evidence_int(
        row, surface, solver, "lcp_dimension"
    )
    if surface == "FrictionIndex":
        expected_dimension = 3 * problem_size
        if lcp_dimension != expected_dimension:
            raise RuntimeError(
                "mismatched LCP performance profile evidence row: "
                f"{surface}/{solver} has lcp_dimension={lcp_dimension}; "
                f"expected {expected_dimension}"
            )
    elif lcp_dimension != problem_size:
        raise RuntimeError(
            "mismatched LCP performance profile evidence row: "
            f"{surface}/{solver} has lcp_dimension={lcp_dimension}; "
            f"expected {problem_size}"
        )
    contact_count = _as_int_counter(row, "contact_count")
    if surface == "FrictionIndex":
        if contact_count != problem_size:
            raise RuntimeError(
                "mismatched LCP performance profile evidence row: "
                f"{surface}/{solver} has contact_count={contact_count}; "
                f"expected {problem_size}"
            )
    elif row.get("contact_count", "") and (
        contact_count is None or contact_count < 0
    ):
        raise RuntimeError(
            "invalid LCP performance profile evidence row: "
            f"{surface}/{solver} has contact_count="
            f"{row.get('contact_count', '')!r}"
        )
    time_ns = _as_finite_float_counter(row, "time_ns")
    if time_ns is None or time_ns <= 0.0:
        raise RuntimeError(
            "invalid LCP performance profile evidence row: "
            f"{surface}/{solver} has time_ns={row.get('time_ns', '')!r}"
        )
    contract_ok = _as_int_counter(row, "contract_ok")
    if contract_ok != 1:
        raise RuntimeError(
            "invalid LCP performance profile evidence row: "
            f"{surface}/{solver} has contract_ok={row.get('contract_ok', '')!r}"
        )
    iterations = _as_int_counter(row, "iterations")
    if iterations is None or iterations < 0:
        raise RuntimeError(
            "invalid LCP performance profile evidence row: "
            f"{surface}/{solver} has iterations={row.get('iterations', '')!r}"
        )
    for metric in ("residual", "complementarity", "bound_violation"):
        _validate_finite_nonnegative_evidence_float(row, surface, solver, metric)
    return surface, solver, problem_size


def _duplicate_values(values: list[str]) -> list[str]:
    duplicates: list[str] = []
    seen: set[str] = set()
    seen_duplicates: set[str] = set()
    for value in values:
        if value in seen and value not in seen_duplicates:
            duplicates.append(value)
            seen_duplicates.add(value)
        seen.add(value)
    return duplicates


def _performance_profile_evidence_summary_rows() -> tuple[dict[str, str], ...]:
    if not _PERFORMANCE_PROFILE_EVIDENCE_PATH.is_file():
        return (
            {
                "surface": "unavailable",
                "rows": "0",
                "solvers": "0",
                "dimensions": "-",
                "contacts": "-",
                "contract_ok": "0/0",
                "max_iterations": "0",
                "max_residual": "0.00e+00",
                "max_complementarity": "0.00e+00",
                "max_bound_violation": "0.00e+00",
            },
        )

    summaries: dict[str, dict[str, Any]] = {}
    observed_evidence_keys: set[tuple[str, str, int]] = set()
    with _PERFORMANCE_PROFILE_EVIDENCE_PATH.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        header = reader.fieldnames or []
        duplicate_columns = _duplicate_values(header)
        if duplicate_columns:
            raise RuntimeError(
                "LCP performance profile evidence contains duplicate "
                f"columns: {duplicate_columns}"
            )
        missing_columns = [
            column
            for column in _PERFORMANCE_PROFILE_EVIDENCE_REQUIRED_COLUMNS
            if column not in header
        ]
        if missing_columns:
            raise RuntimeError(
                "LCP performance profile evidence is missing required "
                f"columns: {missing_columns}"
            )
        row_count = 0
        for row in reader:
            row_count += 1
            evidence_key = _validate_performance_profile_evidence_row(row)
            if evidence_key in observed_evidence_keys:
                surface, solver, problem_size = evidence_key
                raise RuntimeError(
                    "duplicate LCP performance profile evidence row: "
                    f"{surface}/{solver}/{problem_size}"
                )
            observed_evidence_keys.add(evidence_key)
            surface = evidence_key[0]
            summary = summaries.setdefault(
                surface,
                {
                    "rows": 0,
                    "solvers": set(),
                    "dimensions": set(),
                    "contacts": set(),
                    "contract_ok": 0,
                    "max_iterations": 0,
                    "max_residual": 0.0,
                    "max_complementarity": 0.0,
                    "max_bound_violation": 0.0,
                },
            )
            summary["rows"] += 1
            summary["solvers"].add(row["solver"])
            lcp_dimension = _as_int_counter(row, "lcp_dimension")
            if lcp_dimension is not None:
                summary["dimensions"].add(lcp_dimension)
            contact_count = _as_int_counter(row, "contact_count")
            if contact_count is not None:
                summary["contacts"].add(contact_count)
            if row.get("contract_ok") == "1":
                summary["contract_ok"] += 1
            summary["max_iterations"] = max(
                summary["max_iterations"],
                _as_int_counter(row, "iterations") or 0,
            )
            summary["max_residual"] = max(
                summary["max_residual"],
                _as_float_counter(row, "residual"),
            )
            summary["max_complementarity"] = max(
                summary["max_complementarity"],
                _as_float_counter(row, "complementarity"),
            )
            summary["max_bound_violation"] = max(
                summary["max_bound_violation"],
                _as_float_counter(row, "bound_violation"),
            )
        if row_count == 0:
            raise RuntimeError("LCP performance profile evidence has no rows")
        missing_surfaces = [
            surface
            for surface in _PROFILE_EVIDENCE_REQUIRED_SURFACES
            if surface not in summaries
        ]
        if missing_surfaces:
            raise RuntimeError(
                "LCP performance profile evidence is missing surfaces: "
                f"{missing_surfaces}"
            )

    rows: list[dict[str, str]] = []
    for surface in ("Standard", "Boxed", "FrictionIndex"):
        summary = summaries.get(surface)
        if summary is None:
            continue
        row_count = summary["rows"]
        rows.append(
            {
                "surface": surface,
                "rows": str(row_count),
                "solvers": str(len(summary["solvers"])),
                "dimensions": _format_evidence_int_values(summary["dimensions"]),
                "contacts": _format_evidence_int_values(summary["contacts"]),
                "contract_ok": f"{summary['contract_ok']}/{row_count}",
                "max_iterations": str(summary["max_iterations"]),
                "max_residual": f"{summary['max_residual']:.2e}",
                "max_complementarity": f"{summary['max_complementarity']:.2e}",
                "max_bound_violation": f"{summary['max_bound_violation']:.2e}",
            }
        )
    return tuple(rows)

_SOLVER_SUPPORT_ROWS: tuple[dict[str, Any], ...] = (
    {
        "name": "Dantzig",
        "family": "Pivoting",
        "standard": True,
        "boxed": True,
        "findex": True,
        "role": "exact boxed/contact reference",
    },
    {
        "name": "Lemke",
        "family": "Pivoting",
        "standard": True,
        "boxed": False,
        "findex": False,
        "role": "standard pivot baseline",
    },
    {
        "name": "Baraff",
        "family": "Pivoting",
        "standard": True,
        "boxed": False,
        "findex": False,
        "role": "standard pivot baseline",
    },
    {
        "name": "Direct",
        "family": "Pivoting",
        "standard": True,
        "boxed": False,
        "findex": False,
        "role": "dense standard solve",
    },
    {
        "name": "Pgs",
        "family": "Projection",
        "standard": True,
        "boxed": True,
        "findex": True,
        "role": "iterative contact baseline",
    },
    {
        "name": "SymmetricPsor",
        "family": "Projection",
        "standard": True,
        "boxed": True,
        "findex": True,
        "role": "relaxed projected iteration",
    },
    {
        "name": "Jacobi",
        "family": "Projection",
        "standard": True,
        "boxed": True,
        "findex": True,
        "role": "parallel projection baseline",
    },
    {
        "name": "RedBlackGaussSeidel",
        "family": "Projection",
        "standard": True,
        "boxed": True,
        "findex": True,
        "role": "color-partitioned iteration",
    },
    {
        "name": "BlockedJacobi",
        "family": "Projection",
        "standard": True,
        "boxed": True,
        "findex": True,
        "role": "block-parallel projection",
    },
    {
        "name": "BGS",
        "family": "Projection",
        "standard": True,
        "boxed": True,
        "findex": True,
        "role": "blocked Gauss-Seidel",
    },
    {
        "name": "NNCG",
        "family": "Projection",
        "standard": True,
        "boxed": True,
        "findex": True,
        "role": "nonlinear conjugate-gradient projection",
    },
    {
        "name": "SubspaceMinimization",
        "family": "Projection",
        "standard": True,
        "boxed": True,
        "findex": True,
        "role": "active-subspace projection",
    },
    {
        "name": "Apgd",
        "family": "Projection",
        "standard": True,
        "boxed": True,
        "findex": True,
        "role": "accelerated projected gradient",
    },
    {
        "name": "Tgs",
        "family": "Projection",
        "standard": True,
        "boxed": True,
        "findex": True,
        "role": "temporal Gauss-Seidel",
    },
    {
        "name": "MinimumMapNewton",
        "family": "Newton",
        "standard": True,
        "boxed": False,
        "findex": False,
        "role": "standard smooth Newton map",
    },
    {
        "name": "FischerBurmeisterNewton",
        "family": "Newton",
        "standard": True,
        "boxed": False,
        "findex": False,
        "role": "standard FB Newton map",
    },
    {
        "name": "PenalizedFischerBurmeisterNewton",
        "family": "Newton",
        "standard": True,
        "boxed": False,
        "findex": False,
        "role": "standard penalized FB Newton map",
    },
    {
        "name": "InteriorPoint",
        "family": "Other",
        "standard": True,
        "boxed": False,
        "findex": False,
        "role": "standard barrier path",
    },
    {
        "name": "MPRGP",
        "family": "Other",
        "standard": True,
        "boxed": False,
        "findex": False,
        "role": "standard projected-gradient comparison",
    },
    {
        "name": "ShockPropagation",
        "family": "Other",
        "standard": True,
        "boxed": True,
        "findex": True,
        "role": "layered contact ordering",
    },
    {
        "name": "Staggering",
        "family": "Other",
        "standard": False,
        "boxed": False,
        "findex": True,
        "role": "contact pipeline staggering",
    },
    {
        "name": "Admm",
        "family": "Other",
        "standard": True,
        "boxed": True,
        "findex": True,
        "role": "operator-splitting baseline",
    },
    {
        "name": "Sap",
        "family": "Other",
        "standard": True,
        "boxed": True,
        "findex": True,
        "role": "regularized contact baseline",
    },
    {
        "name": "BoxedSemiSmoothNewton",
        "family": "Newton",
        "standard": True,
        "boxed": True,
        "findex": True,
        "role": "boxed/contact Newton comparison",
    },
)

_SOLVER_GUIDANCE_ROWS: tuple[dict[str, str], ...] = (
    {
        "family": "Pivoting and direct",
        "solvers": "Direct, Dantzig, Lemke, Baraff",
        "best_fit": "small, reference, ill-conditioned, or exact rows",
        "strength": "robust exact or near-exact baselines for validating others",
        "tradeoff": "cubic-style scaling makes large dense rows expensive",
        "evidence": "Standard and Boxed profile leaders include Dantzig/direct rows",
    },
    {
        "family": "Projection iterations",
        "solvers": "Pgs, Tgs, SymmetricPsor, Jacobi, RedBlackGaussSeidel",
        "best_fit": "real-time approximate solves and parallel-friendly sweeps",
        "strength": "cheap iterations, warm-start friendly, predictable budgets",
        "tradeoff": "needs tuning and can leave larger residuals on hard rows",
        "evidence": "Projection rows anchor the Boxed and FrictionIndex leading group",
    },
    {
        "family": "Block/contact structure",
        "solvers": "BGS, BlockedJacobi, Staggering, ShockPropagation",
        "best_fit": "contact piles, friction blocks, stacks, and layered scenes",
        "strength": "uses per-contact or layer structure instead of flat scalar rows",
        "tradeoff": "structure assumptions can be weaker on generic dense LCP rows",
        "evidence": "Contact packet and FrictionIndex rows expose block/layer behavior",
    },
    {
        "family": "Newton, interior, and QP",
        "solvers": (
            "MinimumMapNewton, FischerBurmeisterNewton, "
            "PenalizedFischerBurmeisterNewton, BoxedSemiSmoothNewton, "
            "InteriorPoint, MPRGP"
        ),
        "best_fit": "high-accuracy standard rows, strict-interior rows, SPD cases",
        "strength": "fast local convergence when assumptions and linear solves are good",
        "tradeoff": "more setup, linear algebra cost, and form-specific support",
        "evidence": "Profile evidence separates standard-only and boxed/findex support",
    },
    {
        "family": "Accelerated and splitting",
        "solvers": "Apgd, NNCG, SubspaceMinimization, Admm, Sap",
        "best_fit": "larger bounded rows, regularized rows, and tunable approximations",
        "strength": "balances scalability, smoothness, and robust fallback behavior",
        "tradeoff": "sensitive to restart, rho, regularization, and warm-start tuning",
        "evidence": "Parameter sweeps and profile laggards show where tuning matters",
    },
)

_SOLVER_CLASS_NAMES: dict[str, str] = {
    "Dantzig": "DantzigSolver",
    "Lemke": "LemkeSolver",
    "Baraff": "BaraffSolver",
    "Direct": "DirectSolver",
    "Pgs": "PgsSolver",
    "SymmetricPsor": "SymmetricPsorSolver",
    "Jacobi": "JacobiSolver",
    "RedBlackGaussSeidel": "RedBlackGaussSeidelSolver",
    "BlockedJacobi": "BlockedJacobiSolver",
    "BGS": "BgsSolver",
    "NNCG": "NncgSolver",
    "SubspaceMinimization": "SubspaceMinimizationSolver",
    "Apgd": "ApgdSolver",
    "Tgs": "TgsSolver",
    "MinimumMapNewton": "MinimumMapNewtonSolver",
    "FischerBurmeisterNewton": "FischerBurmeisterNewtonSolver",
    "PenalizedFischerBurmeisterNewton": "PenalizedFischerBurmeisterNewtonSolver",
    "InteriorPoint": "InteriorPointSolver",
    "MPRGP": "MprgpSolver",
    "ShockPropagation": "ShockPropagationSolver",
    "Staggering": "StaggeringSolver",
    "Admm": "AdmmSolver",
    "Sap": "SapSolver",
    "BoxedSemiSmoothNewton": "BoxedSemiSmoothNewtonSolver",
}

_CARD_KEYS: tuple[str, ...] = tuple(f"card_{index}" for index in range(7))


@dataclass
class _ComparisonCase:
    label: str
    method_name: str
    world: sx.World
    bridge: WorldRenderBridge
    bodies: dict[str, Any]
    initial_positions: dict[str, np.ndarray]
    ramp_tangent: np.ndarray
    initial_billiard_momentum_x: float
    initial_billiard_energy: float
    step_ms_history: deque[float] = field(
        default_factory=lambda: deque(maxlen=_HISTORY)
    )
    contact_history: deque[float] = field(
        default_factory=lambda: deque(maxlen=_HISTORY)
    )
    slide_speed_history: deque[float] = field(
        default_factory=lambda: deque(maxlen=_HISTORY)
    )
    ramp_slide_history: deque[float] = field(
        default_factory=lambda: deque(maxlen=_HISTORY)
    )
    momentum_error_history: deque[float] = field(
        default_factory=lambda: deque(maxlen=_HISTORY)
    )
    energy_error_history: deque[float] = field(
        default_factory=lambda: deque(maxlen=_HISTORY)
    )
    billiard_symmetry_history: deque[float] = field(
        default_factory=lambda: deque(maxlen=_HISTORY)
    )
    stack_drift_history: deque[float] = field(
        default_factory=lambda: deque(maxlen=_HISTORY)
    )
    card_spread_history: deque[float] = field(
        default_factory=lambda: deque(maxlen=_HISTORY)
    )
    card_height_loss_history: deque[float] = field(
        default_factory=lambda: deque(maxlen=_HISTORY)
    )


@dataclass(frozen=True)
class _StandaloneProblemCase:
    name: str
    label: str
    surface: str
    support_key: str
    challenge: str
    make_problem: Callable[[], tuple[dart.LcpProblem, np.ndarray]]
    tolerance: float = 1e-4


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * np.asarray(half_extents, dtype=float)


def _tilt_quaternion(
    angle: float = _RAMP_TILT_RAD,
) -> tuple[float, float, float, float]:
    return (math.cos(0.5 * angle), 0.0, math.sin(0.5 * angle), 0.0)


def _yaw_quaternion(angle: float) -> tuple[float, float, float, float]:
    return (math.cos(0.5 * angle), 0.0, 0.0, math.sin(0.5 * angle))


def _rotate_y(angle: float, vector: np.ndarray) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    x, y, z = np.asarray(vector, dtype=float)
    return np.array([c * x + s * z, y, -s * x + c * z])


def _translation(body: Any) -> np.ndarray:
    return np.asarray(body.translation, dtype=float).reshape(3)


def _velocity(body: Any) -> np.ndarray:
    return np.asarray(body.linear_velocity, dtype=float).reshape(3)


def _mass(body: Any) -> float:
    return float(getattr(body, "mass", 0.0))


def _linear_kinetic_energy(body: Any) -> float:
    velocity = _velocity(body)
    return 0.5 * _mass(body) * float(np.dot(velocity, velocity))


def _add_box(
    world: sx.World,
    bridge: WorldRenderBridge,
    name: str,
    half_extents: np.ndarray,
    position: np.ndarray,
    color: tuple[float, float, float],
    *,
    is_static: bool = False,
    mass: float = 1.0,
    friction: float = 0.6,
    restitution: float = 0.0,
    orientation: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    linear_velocity: np.ndarray | None = None,
) -> sx.RigidBody:
    options = sx.RigidBodyOptions()
    options.position = np.asarray(position, dtype=float)
    options.orientation = orientation
    options.is_static = is_static
    if linear_velocity is not None:
        options.linear_velocity = np.asarray(linear_velocity, dtype=float)

    body = world.add_rigid_body(name, options)
    body.set_collision_shape(
        sx.CollisionShape.box(np.asarray(half_extents, dtype=float))
    )
    body.friction = float(friction)
    body.restitution = float(restitution)
    if not is_static:
        body.mass = float(mass)

    bridge.add_rigid_body_visual(
        body,
        dart.BoxShape(_full(half_extents)),
        color,
        name=f"{name}_visual",
    )
    return body


def _add_sphere(
    world: sx.World,
    bridge: WorldRenderBridge,
    name: str,
    radius: float,
    position: np.ndarray,
    color: tuple[float, float, float],
    *,
    mass: float = 1.0,
    friction: float = 0.05,
    restitution: float = 0.9,
    linear_velocity: np.ndarray | None = None,
) -> sx.RigidBody:
    options = sx.RigidBodyOptions()
    options.position = np.asarray(position, dtype=float)
    if linear_velocity is not None:
        options.linear_velocity = np.asarray(linear_velocity, dtype=float)

    body = world.add_rigid_body(name, options)
    body.set_collision_shape(sx.CollisionShape.sphere(radius))
    body.mass = float(mass)
    body.friction = float(friction)
    body.restitution = float(restitution)

    bridge.add_rigid_body_visual(
        body,
        dart.SphereShape(radius),
        color,
        name=f"{name}_visual",
    )
    return body


def _build_comparison_case(
    method: sx.ContactSolverMethod,
    method_name: str,
    x_offset: float,
    colors: dict[str, tuple[float, float, float]],
) -> _ComparisonCase:
    world = sx.World(contact_solver_method=method)
    world.time_step = _TIME_STEP
    bridge = WorldRenderBridge(world, name=f"lcp_physics_{method_name}_render")
    bodies: dict[str, Any] = {}

    # Sliding friction packet.
    slide_y = -3.0
    slide_ground = _add_box(
        world,
        bridge,
        f"{method_name}_slide_ground",
        np.array([1.15, 0.42, 0.05]),
        np.array([x_offset, slide_y, -0.05]),
        colors["ground"],
        is_static=True,
        friction=0.65,
    )
    bodies["slide_ground"] = slide_ground
    bodies["slide_box"] = _add_box(
        world,
        bridge,
        f"{method_name}_slide_box",
        np.array([0.18, 0.18, 0.18]),
        np.array([x_offset - 0.48, slide_y, 0.18]),
        colors["slide"],
        mass=1.0,
        friction=0.65,
        linear_velocity=np.array([1.25, 0.0, 0.0]),
    )

    # Inclined static-friction packet.
    ramp_y = -1.15
    ramp_half = np.array([1.10, 0.42, 0.06])
    ramp_center = np.array([x_offset, ramp_y, 0.18])
    ramp_tangent = _rotate_y(_RAMP_TILT_RAD, np.array([1.0, 0.0, 0.0]))
    ramp_normal = _rotate_y(_RAMP_TILT_RAD, np.array([0.0, 0.0, 1.0]))
    ramp_orientation = _tilt_quaternion()
    bodies["ramp"] = _add_box(
        world,
        bridge,
        f"{method_name}_ramp",
        ramp_half,
        ramp_center,
        colors["ground"],
        is_static=True,
        friction=0.55,
        orientation=ramp_orientation,
    )
    ramp_box_pos = (
        ramp_center - 0.42 * ramp_tangent + (ramp_half[2] + 0.16) * ramp_normal
    )
    bodies["ramp_box"] = _add_box(
        world,
        bridge,
        f"{method_name}_ramp_box",
        np.array([0.16, 0.16, 0.16]),
        ramp_box_pos,
        colors["ramp"],
        mass=1.0,
        friction=0.55,
        orientation=ramp_orientation,
    )

    # Billiard collision packet.
    billiard_y = 0.75
    bodies["billiard_ground"] = _add_box(
        world,
        bridge,
        f"{method_name}_billiard_ground",
        np.array([1.05, 0.42, 0.035]),
        np.array([x_offset, billiard_y, -0.035]),
        colors["ground"],
        is_static=True,
        friction=0.03,
    )
    radius = 0.14
    bodies["cue_ball"] = _add_sphere(
        world,
        bridge,
        f"{method_name}_cue_ball",
        radius,
        np.array([x_offset - 0.48, billiard_y, radius]),
        colors["cue"],
        mass=1.0,
        friction=0.03,
        restitution=0.9,
        linear_velocity=np.array([1.45, 0.0, 0.0]),
    )
    bodies["target_ball"] = _add_sphere(
        world,
        bridge,
        f"{method_name}_target_ball",
        radius,
        np.array([x_offset + 0.02, billiard_y, radius]),
        colors["target"],
        mass=1.0,
        friction=0.03,
        restitution=0.9,
    )

    # High-mass-ratio stack packet.
    stack_y = 2.65
    bodies["stack_ground"] = _add_box(
        world,
        bridge,
        f"{method_name}_stack_ground",
        np.array([0.55, 0.42, 0.05]),
        np.array([x_offset, stack_y, -0.05]),
        colors["ground"],
        is_static=True,
        friction=0.85,
    )
    stack_half = np.array([0.18, 0.18, 0.10])
    stack_specs = [
        ("stack_light", 0.25, np.array([0.000, 0.000, 0.10])),
        ("stack_mid", 5.0, np.array([0.020, 0.000, 0.305])),
        ("stack_heavy", 50.0, np.array([-0.015, 0.000, 0.510])),
    ]
    for index, (key, mass, local_pos) in enumerate(stack_specs):
        bodies[key] = _add_box(
            world,
            bridge,
            f"{method_name}_{key}",
            stack_half,
            np.array([x_offset, stack_y, 0.0]) + local_pos,
            colors[f"stack_{index}"],
            mass=mass,
            friction=0.85,
        )

    # Thin card pile packet.
    card_y = 4.25
    bodies["card_pile_ground"] = _add_box(
        world,
        bridge,
        f"{method_name}_card_pile_ground",
        np.array([0.72, 0.46, 0.04]),
        np.array([x_offset, card_y, -0.04]),
        colors["ground"],
        is_static=True,
        friction=0.90,
    )
    card_half = np.array([0.30, 0.18, 0.012])
    card_offsets = [
        np.array([-0.020, -0.012, 0.0]),
        np.array([0.014, 0.010, 0.0]),
        np.array([-0.006, 0.018, 0.0]),
        np.array([0.022, -0.006, 0.0]),
        np.array([-0.016, 0.004, 0.0]),
        np.array([0.008, -0.018, 0.0]),
        np.array([0.026, 0.014, 0.0]),
    ]
    card_yaws = [-0.08, 0.04, 0.10, -0.05, 0.07, -0.11, 0.03]
    for index, key in enumerate(_CARD_KEYS):
        local_pos = card_offsets[index].copy()
        local_pos[2] = card_half[2] + index * (2.15 * card_half[2])
        bodies[key] = _add_box(
            world,
            bridge,
            f"{method_name}_{key}",
            card_half,
            np.array([x_offset, card_y, 0.0]) + local_pos,
            colors[f"card_{index % 3}"],
            mass=0.08,
            friction=0.90,
            orientation=_yaw_quaternion(card_yaws[index]),
            linear_velocity=(
                np.array([0.16, -0.03, 0.0])
                if index == len(_CARD_KEYS) - 1
                else None
            ),
        )

    world.enter_simulation_mode()
    bridge.sync()

    initial_positions = {key: _translation(body) for key, body in bodies.items()}
    initial_billiard_momentum_x = sum(
        _mass(bodies[key]) * float(_velocity(bodies[key])[0])
        for key in ("cue_ball", "target_ball")
    )
    initial_billiard_energy = sum(
        _linear_kinetic_energy(bodies[key]) for key in ("cue_ball", "target_ball")
    )

    return _ComparisonCase(
        label=(
            "Sequential impulse"
            if method == sx.ContactSolverMethod.SEQUENTIAL_IMPULSE
            else "Boxed LCP"
        ),
        method_name=method_name,
        world=world,
        bridge=bridge,
        bodies=bodies,
        initial_positions=initial_positions,
        ramp_tangent=ramp_tangent,
        initial_billiard_momentum_x=initial_billiard_momentum_x,
        initial_billiard_energy=initial_billiard_energy,
    )


def _make_cases() -> list[_ComparisonCase]:
    sequential_colors = {
        "ground": (0.43, 0.45, 0.47),
        "slide": (0.20, 0.46, 0.86),
        "ramp": (0.18, 0.55, 0.90),
        "cue": (0.16, 0.62, 0.82),
        "target": (0.52, 0.76, 0.92),
        "stack_0": (0.16, 0.38, 0.70),
        "stack_1": (0.22, 0.50, 0.82),
        "stack_2": (0.34, 0.64, 0.92),
        "card_0": (0.17, 0.45, 0.72),
        "card_1": (0.26, 0.60, 0.84),
        "card_2": (0.42, 0.72, 0.90),
    }
    lcp_colors = {
        "ground": (0.43, 0.45, 0.47),
        "slide": (0.92, 0.50, 0.17),
        "ramp": (0.94, 0.62, 0.22),
        "cue": (0.90, 0.44, 0.20),
        "target": (0.96, 0.72, 0.32),
        "stack_0": (0.70, 0.30, 0.12),
        "stack_1": (0.86, 0.44, 0.16),
        "stack_2": (0.97, 0.61, 0.25),
        "card_0": (0.82, 0.32, 0.13),
        "card_1": (0.94, 0.50, 0.19),
        "card_2": (0.98, 0.67, 0.26),
    }
    return [
        _build_comparison_case(
            sx.ContactSolverMethod.SEQUENTIAL_IMPULSE,
            "sequential_impulse",
            -1.55,
            sequential_colors,
        ),
        _build_comparison_case(
            sx.ContactSolverMethod.BOXED_LCP,
            "boxed_lcp",
            1.55,
            lcp_colors,
        ),
    ]


def _update_metrics(case: _ComparisonCase, step_ms: float) -> None:
    bodies = case.bodies
    case.step_ms_history.append(float(step_ms))
    case.contact_history.append(float(len(case.world.collide())))
    case.slide_speed_history.append(
        float(np.linalg.norm(_velocity(bodies["slide_box"])))
    )

    ramp_delta = _translation(bodies["ramp_box"]) - case.initial_positions["ramp_box"]
    case.ramp_slide_history.append(float(np.dot(ramp_delta, case.ramp_tangent)))

    momentum_x = sum(
        _mass(bodies[key]) * float(_velocity(bodies[key])[0])
        for key in ("cue_ball", "target_ball")
    )
    case.momentum_error_history.append(
        abs(momentum_x - case.initial_billiard_momentum_x)
    )
    energy = sum(
        _linear_kinetic_energy(bodies[key]) for key in ("cue_ball", "target_ball")
    )
    case.energy_error_history.append(
        abs(energy - case.initial_billiard_energy)
        / max(abs(case.initial_billiard_energy), 1.0e-12)
    )
    case.billiard_symmetry_history.append(
        max(
            abs(_translation(bodies[key])[1] - case.initial_positions[key][1])
            for key in ("cue_ball", "target_ball")
        )
    )

    stack_drift = 0.0
    for key in ("stack_light", "stack_mid", "stack_heavy"):
        drift_xy = _translation(bodies[key])[:2] - case.initial_positions[key][:2]
        stack_drift = max(stack_drift, float(np.linalg.norm(drift_xy)))
    case.stack_drift_history.append(stack_drift)

    card_spread = 0.0
    for key in _CARD_KEYS:
        drift_xy = _translation(bodies[key])[:2] - case.initial_positions[key][:2]
        card_spread = max(card_spread, float(np.linalg.norm(drift_xy)))
    top_initial = max(case.initial_positions[key][2] for key in _CARD_KEYS)
    top_current = max(float(_translation(bodies[key])[2]) for key in _CARD_KEYS)
    case.card_spread_history.append(card_spread)
    case.card_height_loss_history.append(max(0.0, top_initial - top_current))


def _last(history: deque[float]) -> float:
    return history[-1] if history else 0.0


def _metrics_snapshot(
    cases: list[_ComparisonCase],
) -> dict[str, dict[str, float | int]]:
    return {
        case.label: {
            "step_samples": len(case.step_ms_history),
            "contact_samples": len(case.contact_history),
            "step_ms": _last(case.step_ms_history),
            "contacts": _last(case.contact_history),
            "sliding_speed": _last(case.slide_speed_history),
            "ramp_slide": _last(case.ramp_slide_history),
            "billiard_momentum_error": _last(case.momentum_error_history),
            "billiard_energy_error": _last(case.energy_error_history),
            "billiard_symmetry_error": _last(case.billiard_symmetry_history),
            "stack_lateral_drift": _last(case.stack_drift_history),
            "card_spread": _last(case.card_spread_history),
            "card_height_loss": _last(case.card_height_loss_history),
        }
        for case in cases
    }


def _write_table_cell(builder: object, value: str) -> None:
    if builder.table_next_column():
        builder.text(value)


def _write_metric_row(
    builder: object,
    label: str,
    sequential: str,
    boxed_lcp: str,
) -> None:
    builder.table_next_row()
    _write_table_cell(builder, label)
    _write_table_cell(builder, sequential)
    _write_table_cell(builder, boxed_lcp)


def _coverage_label(row: dict[str, Any]) -> str:
    surfaces = (("standard", "std"), ("boxed", "boxed"), ("findex", "findex"))
    supported = [label for key, label in surfaces if row[key]]
    delegated = [label for key, label in surfaces if not row[key]]
    supported_text = " + ".join(supported)
    if len(supported) == 1:
        supported_text = f"{supported_text} only"
    if delegated:
        return f"{supported_text}; {'/'.join(delegated)} delegate"
    return supported_text


def _smoke_coverage_label(row: dict[str, Any]) -> str:
    return _coverage_label(
        {
            "standard": row["native_standard"],
            "boxed": row["native_boxed"],
            "findex": row["native_findex"],
        }
    )


def _solver_manifest_summary() -> dict[str, int]:
    return {
        "solver_count": len(_SOLVER_SUPPORT_ROWS),
        "standard_count": sum(1 for row in _SOLVER_SUPPORT_ROWS if row["standard"]),
        "boxed_count": sum(1 for row in _SOLVER_SUPPORT_ROWS if row["boxed"]),
        "findex_count": sum(1 for row in _SOLVER_SUPPORT_ROWS if row["findex"]),
    }


def _copy_rows(rows: tuple[dict[str, Any], ...]) -> list[dict[str, Any]]:
    return [dict(row) for row in rows]


def _make_standard_spd_case() -> tuple[dart.LcpProblem, np.ndarray]:
    A = np.array(
        [
            [4.0, 0.35, 0.10],
            [0.35, 3.0, 0.20],
            [0.10, 0.20, 2.5],
        ],
        dtype=float,
    )
    expected = np.array([0.35, 0.20, 0.45], dtype=float)
    return dart.LcpProblem(A, A @ expected), expected


def _make_ill_conditioned_standard_case() -> tuple[dart.LcpProblem, np.ndarray]:
    A = np.diag([0.015, 1.0, 65.0]).astype(float)
    A[0, 1] = A[1, 0] = 0.002
    A[1, 2] = A[2, 1] = 0.15
    expected = np.array([0.55, 0.35, 0.18], dtype=float)
    return dart.LcpProblem(A, A @ expected), expected


def _make_near_singular_standard_case() -> tuple[dart.LcpProblem, np.ndarray]:
    A = np.array(
        [
            [0.0010, 0.0001, 0.0000, 0.0000],
            [0.0001, 1.0000, 0.0200, 0.0000],
            [0.0000, 0.0200, 2.0000, 0.0200],
            [0.0000, 0.0000, 0.0200, 4.0000],
        ],
        dtype=float,
    )
    expected = np.array([0.25, 0.20, 0.30, 0.15], dtype=float)
    return dart.LcpProblem(A, A @ expected), expected


def _make_boxed_active_bounds_case() -> tuple[dart.LcpProblem, np.ndarray]:
    A = np.array(
        [
            [3.0, 0.20, 0.10],
            [0.20, 2.5, 0.05],
            [0.10, 0.05, 2.2],
        ],
        dtype=float,
    )
    expected = np.array([-0.30, 0.20, 0.50], dtype=float)
    lo = np.array([-0.30, 0.0, -0.20], dtype=float)
    hi = np.array([0.80, 0.35, 0.50], dtype=float)
    w = np.array([0.35, 0.0, -0.25], dtype=float)
    return dart.LcpProblem(A, A @ expected - w, lo, hi), expected


def _make_mass_ratio_boxed_case() -> tuple[dart.LcpProblem, np.ndarray]:
    scales = np.array([0.02, 0.05, 0.30, 1.0, 5.0, 25.0, 80.0, 160.0])
    A = np.diag(scales).astype(float)
    for index in range(scales.size - 1):
        coupling = 0.01 * math.sqrt(scales[index] * scales[index + 1])
        A[index, index + 1] = coupling
        A[index + 1, index] = coupling

    lo = np.array([-0.40, -0.25, 0.00, -0.10, 0.10, 0.20, 0.50, 0.80])
    hi = np.array([0.20, 0.30, 0.60, 0.20, 0.80, 0.90, 1.10, 1.40])
    expected = np.array([-0.40, -0.05, 0.25, 0.20, 0.45, 0.90, 0.75, 1.40])
    w = np.array([0.03, 0.0, 0.0, -0.04, 0.0, -0.20, 0.0, -0.50])

    return dart.LcpProblem(A, A @ expected - w, lo, hi), expected


def _make_singular_degenerate_boxed_case() -> tuple[dart.LcpProblem, np.ndarray]:
    A = np.array(
        [
            [2.0, 2.0, 0.0, 0.0],
            [2.0, 2.0, 0.0, 0.0],
            [0.0, 0.0, 1.2, 0.15],
            [0.0, 0.0, 0.15, 1.0],
        ],
        dtype=float,
    )
    lo = np.array([0.0, -0.20, -0.50, -0.40])
    hi = np.array([0.0, 1.00, 0.80, 0.60])
    expected = np.array([0.0, 1.0, 0.25, -0.10])
    w = np.array([0.20, -0.15, 0.0, 0.0])

    return dart.LcpProblem(A, A @ expected - w, lo, hi), expected


def _make_friction_index_contact_case() -> tuple[dart.LcpProblem, np.ndarray]:
    A = np.array(
        [
            [3.0, 0.18, 0.05],
            [0.18, 2.4, 0.08],
            [0.05, 0.08, 2.1],
        ],
        dtype=float,
    )
    expected = np.array([0.80, 0.20, -0.15], dtype=float)
    lo = np.array([0.0, -np.inf, -np.inf], dtype=float)
    hi = np.array([np.inf, 0.50, 0.50], dtype=float)
    findex = np.array([-1, 0, 0], dtype=np.int32)
    return dart.LcpProblem(A, A @ expected, lo, hi, findex), expected


def _make_active_friction_index_contact_case() -> tuple[dart.LcpProblem, np.ndarray]:
    A = np.array(
        [
            [3.0, 0.12, 0.04, 0.08, 0.02, 0.0],
            [0.12, 2.4, 0.08, 0.01, 0.04, 0.0],
            [0.04, 0.08, 2.1, 0.0, 0.0, 0.03],
            [0.08, 0.01, 0.0, 2.8, 0.10, 0.05],
            [0.02, 0.04, 0.0, 0.10, 2.2, 0.07],
            [0.0, 0.0, 0.03, 0.05, 0.07, 2.0],
        ],
        dtype=float,
    )
    expected = np.array([0.60, 0.30, -0.24, 0.35, -0.175, 0.05], dtype=float)
    lo = np.array([0.0, -np.inf, -np.inf, 0.0, -np.inf, -np.inf], dtype=float)
    hi = np.array([np.inf, 0.50, 0.50, np.inf, 0.50, 0.50], dtype=float)
    findex = np.array([-1, 0, 0, -1, 3, 3], dtype=np.int32)
    w = np.array([0.0, -0.04, 0.0, 0.0, 0.03, 0.0], dtype=float)
    return dart.LcpProblem(A, A @ expected - w, lo, hi, findex), expected


def _make_moderate_scale_standard_case() -> tuple[dart.LcpProblem, np.ndarray]:
    n = 12
    A = np.eye(n, dtype=float) * 2.0
    for index in range(n - 1):
        A[index, index + 1] = 0.08
        A[index + 1, index] = 0.08
    for index in range(n - 2):
        A[index, index + 2] = 0.025
        A[index + 2, index] = 0.025
    expected = np.linspace(0.15, 0.50, n, dtype=float)
    return dart.LcpProblem(A, A @ expected), expected


_STANDALONE_PROBLEM_CASES: tuple[_StandaloneProblemCase, ...] = (
    _StandaloneProblemCase(
        name="standard_spd",
        label="Standard SPD",
        surface="standard",
        support_key="standard",
        challenge="well-conditioned exact baseline",
        make_problem=_make_standard_spd_case,
    ),
    _StandaloneProblemCase(
        name="ill_conditioned_standard",
        label="Ill-conditioned standard",
        surface="standard",
        support_key="standard",
        challenge="large mass-ratio style conditioning",
        make_problem=_make_ill_conditioned_standard_case,
        tolerance=2e-4,
    ),
    _StandaloneProblemCase(
        name="near_singular_standard",
        label="Near-singular standard",
        surface="standard",
        support_key="standard",
        challenge="weakly constrained mode with regularization pressure",
        make_problem=_make_near_singular_standard_case,
        tolerance=2e-4,
    ),
    _StandaloneProblemCase(
        name="boxed_active_bounds",
        label="Boxed active bounds",
        surface="boxed",
        support_key="boxed",
        challenge="mixed lower, interior, and upper active sets",
        make_problem=_make_boxed_active_bounds_case,
        tolerance=2e-4,
    ),
    _StandaloneProblemCase(
        name="mass_ratio_boxed",
        label="Mass-ratio boxed",
        surface="boxed",
        support_key="boxed",
        challenge="large mass-ratio conditioning with active bounds",
        make_problem=_make_mass_ratio_boxed_case,
        tolerance=5e-4,
    ),
    _StandaloneProblemCase(
        name="singular_degenerate_boxed",
        label="Singular degenerate boxed",
        surface="boxed",
        support_key="boxed",
        challenge="rank-deficient complementarity with opposing active bounds",
        make_problem=_make_singular_degenerate_boxed_case,
        tolerance=2e-4,
    ),
    _StandaloneProblemCase(
        name="friction_index_contact",
        label="Friction-index contact",
        surface="findex",
        support_key="findex",
        challenge="normal-scaled tangential friction bounds",
        make_problem=_make_friction_index_contact_case,
    ),
    _StandaloneProblemCase(
        name="active_friction_index_contact",
        label="Active friction-index contact",
        surface="findex",
        support_key="findex",
        challenge="two-contact active tangential bounds with coupled normals",
        make_problem=_make_active_friction_index_contact_case,
        tolerance=2e-4,
    ),
    _StandaloneProblemCase(
        name="moderate_scale_standard",
        label="Moderate-scale standard",
        surface="standard",
        support_key="standard",
        challenge="scalability smoke with banded coupling",
        make_problem=_make_moderate_scale_standard_case,
        tolerance=2e-4,
    ),
)


def _run_standalone_solver_smoke() -> list[dict[str, Any]]:
    problem = dart.LcpProblem(
        np.eye(_STANDALONE_SMOKE_EXPECTED.size), _STANDALONE_SMOKE_EXPECTED
    )
    options = dart.LcpOptions.high_accuracy()
    options.max_iterations = 1000

    rows: list[dict[str, Any]] = []
    for support_row in _SOLVER_SUPPORT_ROWS:
        solver_type = getattr(dart, _SOLVER_CLASS_NAMES[support_row["name"]])
        solver = solver_type()
        result, solution = solver.solve(problem, options=options)
        native_standard = bool(solver.supports_standard_lcp())
        native_problem = bool(solver.supports_problem(problem))
        rows.append(
            {
                "name": support_row["name"],
                "family": support_row["family"],
                "native_standard": native_standard,
                "native_boxed": bool(solver.supports_boxed_lcp()),
                "native_findex": bool(solver.supports_friction_index()),
                "native_problem": native_problem,
                "solve_route": "native" if native_problem else "delegated",
                "status": dart.lcp_solver_status_to_string(result.status),
                "iterations": int(result.iterations),
                "residual": float(result.residual),
                "complementarity": float(result.complementarity),
                "solution_error": float(
                    np.linalg.norm(solution - _STANDALONE_SMOKE_EXPECTED, ord=np.inf)
                ),
            }
        )
    return rows


def _run_standalone_problem_suite() -> list[dict[str, Any]]:
    options = dart.LcpOptions.high_accuracy()
    options.max_iterations = 1000
    options.validate_solution = False

    rows: list[dict[str, Any]] = []
    for problem_case in _STANDALONE_PROBLEM_CASES:
        problem, expected = problem_case.make_problem()
        problem_type = dart.lcp_problem_type_to_string(problem.get_type())
        lcp_dimension = int(problem.size())
        findex_row_count = int(problem.get_friction_index_row_count())
        findex_contact_count = int(problem.get_friction_index_contact_count())
        for support_row in _SOLVER_SUPPORT_ROWS:
            solver_type = getattr(dart, _SOLVER_CLASS_NAMES[support_row["name"]])
            solver = solver_type()
            native_supported = bool(solver.supports_problem(problem))
            started_ns = time.perf_counter_ns()
            result, solution = solver.solve(problem, options=options)
            elapsed_us = float(time.perf_counter_ns() - started_ns) / 1000.0
            status = dart.lcp_solver_status_to_string(result.status)
            solution_error = float(np.linalg.norm(solution - expected, ord=np.inf))
            rows.append(
                {
                    "case": problem_case.name,
                    "label": problem_case.label,
                    "surface": problem_case.surface,
                    "problem_type": problem_type,
                    "lcp_dimension": lcp_dimension,
                    "findex_row_count": findex_row_count,
                    "findex_contact_count": findex_contact_count,
                    "challenge": problem_case.challenge,
                    "solver": support_row["name"],
                    "family": support_row["family"],
                    "native_supported": native_supported,
                    "solve_route": "native" if native_supported else "delegated",
                    "status": status,
                    "iterations": int(result.iterations),
                    "residual": float(result.residual),
                    "complementarity": float(result.complementarity),
                    "solution_error": solution_error,
                    "elapsed_us": elapsed_us,
                    "contract_ok": (
                        status in _STANDALONE_SUCCESS_STATUSES
                        and solution_error <= problem_case.tolerance
                    ),
                }
            )
    return rows


def _summarize_standalone_problem_suite(
    rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    summaries: list[dict[str, Any]] = []
    for problem_case in _STANDALONE_PROBLEM_CASES:
        case_rows = [row for row in rows if row["case"] == problem_case.name]
        native_rows = [row for row in case_rows if row["native_supported"]]
        delegated_rows = [row for row in case_rows if not row["native_supported"]]
        max_error = max((row["solution_error"] for row in case_rows), default=0.0)
        max_residual = max((row["residual"] for row in case_rows), default=0.0)
        max_complementarity = max(
            (row["complementarity"] for row in case_rows), default=0.0
        )
        max_elapsed_us = max((row["elapsed_us"] for row in case_rows), default=0.0)
        fastest_row = min(case_rows, key=lambda row: row["elapsed_us"], default=None)
        fastest_native_row = min(
            native_rows, key=lambda row: row["elapsed_us"], default=None
        )
        slowest_row = max(case_rows, key=lambda row: row["elapsed_us"], default=None)
        summaries.append(
            {
                "case": problem_case.name,
                "label": problem_case.label,
                "surface": problem_case.surface,
                "problem_type": case_rows[0]["problem_type"] if case_rows else "",
                "lcp_dimension": (
                    int(case_rows[0]["lcp_dimension"]) if case_rows else 0
                ),
                "findex_row_count": (
                    int(case_rows[0]["findex_row_count"]) if case_rows else 0
                ),
                "findex_contact_count": (
                    int(case_rows[0]["findex_contact_count"]) if case_rows else 0
                ),
                "challenge": problem_case.challenge,
                "solver_count": len(case_rows),
                "native_solver_count": len(native_rows),
                "delegated_solver_count": len(delegated_rows),
                "contract_ok_count": sum(1 for row in case_rows if row["contract_ok"]),
                "native_contract_ok_count": sum(
                    1 for row in native_rows if row["contract_ok"]
                ),
                "delegated_contract_ok_count": sum(
                    1 for row in delegated_rows if row["contract_ok"]
                ),
                "max_solution_error": max_error,
                "max_residual": max_residual,
                "max_complementarity": max_complementarity,
                "max_elapsed_us": max_elapsed_us,
                "fastest_solver": fastest_row["solver"] if fastest_row else "",
                "fastest_elapsed_us": (
                    fastest_row["elapsed_us"] if fastest_row else 0.0
                ),
                "fastest_native_solver": (
                    fastest_native_row["solver"] if fastest_native_row else ""
                ),
                "fastest_native_elapsed_us": (
                    fastest_native_row["elapsed_us"] if fastest_native_row else 0.0
                ),
                "slowest_solver": slowest_row["solver"] if slowest_row else "",
                "slowest_elapsed_us": (
                    slowest_row["elapsed_us"] if slowest_row else max_elapsed_us
                ),
            }
        )
    return summaries


def _native_case_surface_summary(rows: list[dict[str, Any]]) -> str:
    entries: list[str] = []
    for surface in ("standard", "boxed", "findex"):
        surface_rows = [row for row in rows if row["surface"] == surface]
        native_count = sum(1 for row in surface_rows if row["native_supported"])
        if native_count > 0:
            entries.append(f"{surface} {native_count}/{len(surface_rows)}")
    return ", ".join(entries) if entries else "none"


def _summarize_standalone_solver_profiles(
    rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    profiles: list[dict[str, Any]] = []
    for support_row in _SOLVER_SUPPORT_ROWS:
        solver_rows = [row for row in rows if row["solver"] == support_row["name"]]
        native_rows = [row for row in solver_rows if row["native_supported"]]
        delegated_rows = [row for row in solver_rows if not row["native_supported"]]
        slowest_row = max(solver_rows, key=lambda row: row["elapsed_us"], default=None)
        profiles.append(
            {
                "solver": support_row["name"],
                "family": support_row["family"],
                "native_surfaces": _native_case_surface_summary(solver_rows),
                "problem_count": len(solver_rows),
                "native_case_count": len(native_rows),
                "delegated_case_count": len(delegated_rows),
                "contract_ok_count": sum(
                    1 for row in solver_rows if row["contract_ok"]
                ),
                "native_contract_ok_count": sum(
                    1 for row in native_rows if row["contract_ok"]
                ),
                "delegated_contract_ok_count": sum(
                    1 for row in delegated_rows if row["contract_ok"]
                ),
                "max_solution_error": max(
                    (row["solution_error"] for row in solver_rows), default=0.0
                ),
                "max_residual": max(
                    (row["residual"] for row in solver_rows), default=0.0
                ),
                "max_complementarity": max(
                    (row["complementarity"] for row in solver_rows), default=0.0
                ),
                "total_elapsed_us": sum(row["elapsed_us"] for row in solver_rows),
                "slowest_case": slowest_row["case"] if slowest_row else "",
                "slowest_elapsed_us": slowest_row["elapsed_us"] if slowest_row else 0.0,
            }
        )
    return profiles


def _advanced_solver_parameter_rows() -> list[dict[str, str]]:
    def format_value(value: object) -> str:
        if isinstance(value, bool):
            return str(value).lower()
        if isinstance(value, float):
            return f"{value:.0e}" if 0.0 < abs(value) < 1e-3 else f"{value:g}"
        if isinstance(value, list):
            return "[" + ", ".join(format_value(item) for item in value) + "]"
        return str(value)

    def make_row(
        solver: str,
        surface: str,
        params: object,
        parameter_names: list[str],
        benchmark_filter: str,
    ) -> dict[str, str]:
        return {
            "solver": solver,
            "surface": surface,
            "parameters": ", ".join(parameter_names),
            "defaults": ", ".join(
                f"{name}={format_value(getattr(params, name))}"
                for name in parameter_names
            ),
            "benchmark_filter": benchmark_filter,
        }

    newton_parameter_names = [
        "max_line_search_steps",
        "step_reduction",
        "sufficient_decrease",
        "min_step",
        "max_gradient_descent_warm_start_steps",
        "max_gradient_descent_line_search_steps",
        "gradient_descent_step_reduction",
        "gradient_descent_sufficient_decrease",
        "gradient_descent_min_step",
        "max_pgs_warm_start_iterations",
        "pgs_warm_start_relaxation",
    ]

    return [
        make_row(
            "Pgs",
            "boxed/findex",
            dart.PgsSolverParameters(),
            ["epsilon_for_division", "randomize_constraint_order"],
            "BM_LcpPgsRelaxationSweep",
        ),
        make_row(
            "SymmetricPsor",
            "standard",
            dart.SymmetricPsorSolverParameters(),
            ["epsilon_for_division"],
            "BM_LcpSymmetricPsorRelaxationSweep",
        ),
        make_row(
            "Jacobi",
            "boxed/findex",
            dart.JacobiSolverParameters(),
            ["epsilon_for_division", "worker_threads"],
            "BM_LcpJacobiSolverThreading",
        ),
        make_row(
            "RedBlackGaussSeidel",
            "boxed/findex",
            dart.RedBlackGaussSeidelSolverParameters(),
            ["epsilon_for_division", "worker_threads"],
            (
                "BM_LcpRedBlackGaussSeidelRelaxationSweep|"
                "BM_LcpRedBlackGaussSeidelSolverThreadingBanded"
            ),
        ),
        make_row(
            "BlockedJacobi",
            "boxed/findex",
            dart.BlockedJacobiSolverParameters(),
            ["block_sizes", "worker_threads"],
            "BM_LcpBlockPartitionSweep|BM_LcpBlockedJacobiSolverThreadingBanded",
        ),
        make_row(
            "BGS",
            "boxed/findex",
            dart.BgsSolverParameters(),
            ["block_sizes"],
            "BM_LcpBlockPartitionSweep",
        ),
        make_row(
            "NNCG",
            "boxed/findex",
            dart.NncgSolverParameters(),
            ["pgs_iterations", "restart_interval", "restart_threshold"],
            "BM_LcpNncgPgsIterationsSweep",
        ),
        make_row(
            "SubspaceMinimization",
            "boxed/findex",
            dart.SubspaceMinimizationSolverParameters(),
            ["pgs_iterations", "active_set_tolerance"],
            "BM_LcpSubspaceMinimizationPgsIterationsSweep",
        ),
        make_row(
            "Apgd",
            "boxed/findex",
            dart.ApgdSolverParameters(),
            [
                "epsilon_for_division",
                "adaptive_restart",
                "restart_check_interval",
            ],
            "BM_LcpApgdRestartSweep",
        ),
        make_row(
            "Tgs",
            "boxed/findex",
            dart.TgsSolverParameters(),
            ["epsilon_for_division"],
            "BM_LcpTgsIterationBudgetSweep",
        ),
        make_row(
            "MinimumMapNewton",
            "standard",
            dart.MinimumMapNewtonSolverParameters(),
            newton_parameter_names,
            "BM_LcpNewtonWarmStart",
        ),
        make_row(
            "FischerBurmeisterNewton",
            "standard",
            dart.FischerBurmeisterNewtonSolverParameters(),
            ["smoothing_epsilon", *newton_parameter_names],
            "BM_LcpNewtonWarmStart",
        ),
        make_row(
            "PenalizedFischerBurmeisterNewton",
            "standard",
            dart.PenalizedFischerBurmeisterNewtonSolverParameters(),
            ["smoothing_epsilon", "lambda_", *newton_parameter_names],
            "BM_LcpNewtonWarmStart",
        ),
        make_row(
            "BoxedSemiSmoothNewton",
            "boxed/findex",
            dart.BoxedSemiSmoothNewtonSolverParameters(),
            [
                "max_line_search_steps",
                "step_reduction",
                "sufficient_decrease",
                "min_step",
                "jacobian_regularization",
                "max_pgs_warm_start_iterations",
                "pgs_warm_start_relaxation",
                "max_friction_index_exact_solve_dimension",
            ],
            "BM_LcpBoxedSemiSmoothNewtonLineSearchSweep",
        ),
        make_row(
            "InteriorPoint",
            "standard",
            dart.InteriorPointSolverParameters(),
            ["sigma", "step_scale"],
            "BM_LcpInteriorPointPathSweep",
        ),
        make_row(
            "MPRGP",
            "standard",
            dart.MprgpSolverParameters(),
            [
                "symmetry_tolerance",
                "epsilon_for_division",
                "check_positive_definite",
            ],
            "BM_LcpMprgpSpdCheckSweep",
        ),
        make_row(
            "ShockPropagation",
            "boxed/findex",
            dart.ShockPropagationSolverParameters(),
            ["block_sizes", "layers"],
            "BM_LcpShockPropagationLayerSweep",
        ),
        make_row(
            "Admm",
            "boxed/findex",
            dart.AdmmSolverParameters(),
            ["rho_init", "mu_prox", "adaptive_rho_tolerance", "adaptive_rho"],
            "BM_LcpAdmmRhoSweep",
        ),
        make_row(
            "Sap",
            "boxed/findex",
            dart.SapSolverParameters(),
            [
                "regularization",
                "armijos_parameter",
                "backtracking_factor",
                "max_line_search_iterations",
            ],
            "BM_LcpSapRegularizationSweep",
        ),
    ]


def build() -> SceneSetup:
    state: dict[str, list[_ComparisonCase]] = {"cases": _make_cases()}
    standalone_solver_rows = _run_standalone_solver_smoke()
    standalone_problem_rows = _run_standalone_problem_suite()
    standalone_problem_summary_rows = _summarize_standalone_problem_suite(
        standalone_problem_rows
    )
    standalone_solver_profile_rows = _summarize_standalone_solver_profiles(
        standalone_problem_rows
    )
    advanced_solver_parameter_rows = _advanced_solver_parameter_rows()
    performance_profile_evidence_summary_rows = (
        _performance_profile_evidence_summary_rows()
    )

    def reset() -> None:
        state["cases"] = _make_cases()

    def pre_step() -> None:
        for case in state["cases"]:
            started = time.perf_counter_ns()
            case.bridge.pre_step()
            elapsed_ms = float(time.perf_counter_ns() - started) / 1.0e6
            _update_metrics(case, elapsed_ms)

    def live_metrics_snapshot() -> dict[str, dict[str, float | int]]:
        return _metrics_snapshot(state["cases"])

    def renderable_provider() -> list[Any]:
        renderables: list[Any] = []
        for case in state["cases"]:
            renderables.extend(case.bridge.renderable_provider())
        return renderables

    def force_drag(event: dict[str, Any]) -> None:
        for case in state["cases"]:
            case.bridge.force_drag(event)

    def build_panel(builder: object, context: object) -> None:
        cases = state["cases"]
        sequential = cases[0]
        boxed_lcp = cases[1]

        builder.text(
            "packets: sliding friction, ramp hold, billiard, mass-ratio stack, card pile"
        )
        builder.text(f"time step: {_TIME_STEP:.4f} s")
        builder.text(f"world time: {boxed_lcp.world.time:.3f} s")
        if builder.button("Reset rollout"):
            reset()
            return

        if builder.begin_table(
            "lcp_physics_metrics", ["Metric", "Sequential", "Boxed LCP"]
        ):
            _write_metric_row(
                builder,
                "step",
                f"{_last(sequential.step_ms_history):.3f} ms",
                f"{_last(boxed_lcp.step_ms_history):.3f} ms",
            )
            _write_metric_row(
                builder,
                "contacts",
                f"{_last(sequential.contact_history):.0f}",
                f"{_last(boxed_lcp.contact_history):.0f}",
            )
            _write_metric_row(
                builder,
                "sliding speed",
                f"{_last(sequential.slide_speed_history):.3f} m/s",
                f"{_last(boxed_lcp.slide_speed_history):.3f} m/s",
            )
            _write_metric_row(
                builder,
                "ramp slide",
                f"{_last(sequential.ramp_slide_history):.4f} m",
                f"{_last(boxed_lcp.ramp_slide_history):.4f} m",
            )
            _write_metric_row(
                builder,
                "billiard momentum error",
                f"{_last(sequential.momentum_error_history):.3e}",
                f"{_last(boxed_lcp.momentum_error_history):.3e}",
            )
            _write_metric_row(
                builder,
                "billiard energy error",
                f"{_last(sequential.energy_error_history):.3e}",
                f"{_last(boxed_lcp.energy_error_history):.3e}",
            )
            _write_metric_row(
                builder,
                "billiard symmetry error",
                f"{_last(sequential.billiard_symmetry_history):.4f} m",
                f"{_last(boxed_lcp.billiard_symmetry_history):.4f} m",
            )
            _write_metric_row(
                builder,
                "stack lateral drift",
                f"{_last(sequential.stack_drift_history):.4f} m",
                f"{_last(boxed_lcp.stack_drift_history):.4f} m",
            )
            _write_metric_row(
                builder,
                "card spread",
                f"{_last(sequential.card_spread_history):.4f} m",
                f"{_last(boxed_lcp.card_spread_history):.4f} m",
            )
            _write_metric_row(
                builder,
                "card height loss",
                f"{_last(sequential.card_height_loss_history):.4f} m",
                f"{_last(boxed_lcp.card_height_loss_history):.4f} m",
            )
            builder.end_table()

        builder.plot_lines("Sequential step ms", list(sequential.step_ms_history))
        builder.plot_lines("Boxed LCP step ms", list(boxed_lcp.step_ms_history))
        builder.plot_lines("Sequential ramp slide", list(sequential.ramp_slide_history))
        builder.plot_lines("Boxed LCP ramp slide", list(boxed_lcp.ramp_slide_history))
        builder.plot_lines(
            "Sequential billiard momentum error",
            list(sequential.momentum_error_history),
        )
        builder.plot_lines(
            "Boxed LCP billiard momentum error",
            list(boxed_lcp.momentum_error_history),
        )
        builder.plot_lines(
            "Sequential billiard energy error",
            list(sequential.energy_error_history),
        )
        builder.plot_lines(
            "Boxed LCP billiard energy error",
            list(boxed_lcp.energy_error_history),
        )
        builder.plot_lines(
            "Sequential billiard symmetry error",
            list(sequential.billiard_symmetry_history),
        )
        builder.plot_lines(
            "Boxed LCP billiard symmetry error",
            list(boxed_lcp.billiard_symmetry_history),
        )
        builder.plot_lines(
            "Sequential stack drift", list(sequential.stack_drift_history)
        )
        builder.plot_lines("Boxed LCP stack drift", list(boxed_lcp.stack_drift_history))
        builder.plot_lines(
            "Sequential card spread", list(sequential.card_spread_history)
        )
        builder.plot_lines(
            "Boxed LCP card spread", list(boxed_lcp.card_spread_history)
        )
        builder.separator()
        if builder.collapsing_header("Live packet cards", default_open=False):
            if builder.begin_table(
                "lcp_live_packet_cards", ["Packet", "Metric", "Benchmark analog"]
            ):
                for row in _LIVE_PACKET_ROWS:
                    builder.table_next_row()
                    _write_table_cell(builder, row["packet"])
                    _write_table_cell(builder, row["metric"])
                    _write_table_cell(builder, row["benchmark"])
                builder.end_table()

        if builder.collapsing_header(
            "Representative requirement coverage", default_open=False
        ):
            if builder.begin_table(
                "lcp_representative_requirement_coverage",
                [
                    "Requirement",
                    "Live packet",
                    "Benchmark packet",
                    "Metrics",
                    "Evidence",
                ],
            ):
                for row in _REPRESENTATIVE_REQUIREMENT_ROWS:
                    builder.table_next_row()
                    _write_table_cell(builder, row["requirement"])
                    _write_table_cell(builder, row["live_packet"])
                    _write_table_cell(builder, row["benchmark_packet"])
                    _write_table_cell(builder, row["metrics"])
                    _write_table_cell(builder, row["evidence"])
                builder.end_table()

        if builder.collapsing_header("Solver manifest", default_open=False):
            summary = _solver_manifest_summary()
            builder.text(
                "solver manifest: "
                f"{summary['solver_count']} solvers, "
                f"{summary['standard_count']} standard, "
                f"{summary['boxed_count']} boxed, "
                f"{summary['findex_count']} friction-index"
            )
            builder.text(f"benchmark smoke: {_BENCHMARK_COMMAND}")
            if builder.begin_table(
                "lcp_solver_manifest", ["Solver", "Family", "Coverage"]
            ):
                for row in _SOLVER_SUPPORT_ROWS:
                    builder.table_next_row()
                    _write_table_cell(builder, row["name"])
                    _write_table_cell(builder, row["family"])
                    _write_table_cell(builder, _coverage_label(row))
                builder.end_table()

        if builder.collapsing_header("Benchmark packets", default_open=False):
            if builder.begin_table(
                "lcp_benchmark_packets",
                ["Packet", "Surface", "Benchmark filter", "Coverage"],
            ):
                for row in _BENCHMARK_PACKET_ROWS:
                    builder.table_next_row()
                    _write_table_cell(builder, row["packet"])
                    _write_table_cell(builder, row["surface"])
                    _write_table_cell(builder, row["benchmark_filter"])
                    _write_table_cell(builder, row["coverage"])
                builder.end_table()

        if builder.collapsing_header("Performance profiles", default_open=False):
            builder.text(f"profile refresh: {_PERFORMANCE_PROFILE_REFRESH_COMMAND}")
            builder.text(f"profile smoke: {_PERFORMANCE_PROFILE_SMOKE_COMMAND}")
            if builder.begin_table(
                "lcp_performance_profiles",
                [
                    "Surface",
                    "Profile CSV",
                    "Evidence CSV",
                    "Problem sizes",
                    "Current leaders",
                    "Current laggards",
                    "Takeaway",
                ],
            ):
                for row in _PERFORMANCE_PROFILE_ROWS:
                    builder.table_next_row()
                    _write_table_cell(builder, row["surface"])
                    _write_table_cell(builder, row["artifact"])
                    _write_table_cell(builder, row["evidence_artifact"])
                    _write_table_cell(builder, row["problem_sizes"])
                    _write_table_cell(builder, row["current_leaders"])
                    _write_table_cell(builder, row["current_laggards"])
                    _write_table_cell(builder, row["takeaway"])
                builder.end_table()
            if builder.begin_table(
                "lcp_performance_profile_evidence_schema",
                ["Field(s)", "Meaning"],
            ):
                for row in _PERFORMANCE_PROFILE_EVIDENCE_SCHEMA_ROWS:
                    builder.table_next_row()
                    _write_table_cell(builder, row["fields"])
                    _write_table_cell(builder, row["meaning"])
                builder.end_table()
            if builder.begin_table(
                "lcp_performance_profile_evidence_summary",
                [
                    "Surface",
                    "Rows",
                    "Solvers",
                    "LCP dimensions",
                    "Contacts",
                    "OK",
                    "Max it",
                    "Max residual",
                    "Max comp",
                    "Max bound",
                ],
            ):
                for row in performance_profile_evidence_summary_rows:
                    builder.table_next_row()
                    _write_table_cell(builder, row["surface"])
                    _write_table_cell(builder, row["rows"])
                    _write_table_cell(builder, row["solvers"])
                    _write_table_cell(builder, row["dimensions"])
                    _write_table_cell(builder, row["contacts"])
                    _write_table_cell(builder, row["contract_ok"])
                    _write_table_cell(builder, row["max_iterations"])
                    _write_table_cell(builder, row["max_residual"])
                    _write_table_cell(builder, row["max_complementarity"])
                    _write_table_cell(builder, row["max_bound_violation"])
                builder.end_table()

        if builder.collapsing_header("Standalone solver smoke", default_open=False):
            if builder.begin_table(
                "lcp_standalone_solver_smoke",
                [
                    "Solver",
                    "Family",
                    "Native coverage",
                    "Route",
                    "Status",
                    "Error",
                    "Residual",
                    "Complementarity",
                ],
            ):
                for row in standalone_solver_rows:
                    builder.table_next_row()
                    _write_table_cell(builder, row["name"])
                    _write_table_cell(builder, row["family"])
                    _write_table_cell(builder, _smoke_coverage_label(row))
                    _write_table_cell(builder, row["solve_route"])
                    _write_table_cell(
                        builder, f"{row['status']} ({row['iterations']} it)"
                    )
                    _write_table_cell(builder, f"{row['solution_error']:.2e}")
                    _write_table_cell(builder, f"{row['residual']:.2e}")
                    _write_table_cell(builder, f"{row['complementarity']:.2e}")
                builder.end_table()

        if builder.collapsing_header(
            _STANDALONE_PROBLEM_SUITE_LABEL, default_open=False
        ):
            if builder.begin_table(
                "lcp_representative_solver_suite",
                [
                    "Problem",
                    "Type",
                    "Rows",
                    "FI contacts",
                    "Challenge",
                    "Native",
                    "Delegated",
                    "Max error",
                    "Max comp",
                    "Max residual",
                    "Fastest",
                    "Fastest native",
                    "Slowest",
                ],
            ):
                for row in standalone_problem_summary_rows:
                    builder.table_next_row()
                    _write_table_cell(builder, row["label"])
                    _write_table_cell(builder, row["problem_type"])
                    _write_table_cell(builder, str(row["lcp_dimension"]))
                    _write_table_cell(builder, str(row["findex_contact_count"]))
                    _write_table_cell(builder, row["challenge"])
                    _write_table_cell(
                        builder,
                        f"{row['native_contract_ok_count']}/{row['native_solver_count']}",
                    )
                    _write_table_cell(
                        builder,
                        (
                            f"{row['delegated_contract_ok_count']}/"
                            f"{row['delegated_solver_count']}"
                        ),
                    )
                    _write_table_cell(builder, f"{row['max_solution_error']:.2e}")
                    _write_table_cell(builder, f"{row['max_complementarity']:.2e}")
                    _write_table_cell(builder, f"{row['max_residual']:.2e}")
                    _write_table_cell(
                        builder,
                        (
                            f"{row['fastest_solver']} "
                            f"({row['fastest_elapsed_us']:.1f} us)"
                        ),
                    )
                    _write_table_cell(
                        builder,
                        (
                            f"{row['fastest_native_solver']} "
                            f"({row['fastest_native_elapsed_us']:.1f} us)"
                        ),
                    )
                    _write_table_cell(
                        builder,
                        (
                            f"{row['slowest_solver']} "
                            f"({row['slowest_elapsed_us']:.1f} us)"
                        ),
                    )
                builder.end_table()

        if builder.collapsing_header("Representative solver details", default_open=False):
            if builder.begin_table(
                "lcp_representative_solver_details",
                [
                    "Problem",
                    "Solver",
                    "Route",
                    "Status",
                    "Contract",
                    "Iterations",
                    "Error",
                    "Residual",
                    "Complementarity",
                    "us",
                ],
            ):
                for row in standalone_problem_rows:
                    builder.table_next_row()
                    _write_table_cell(builder, row["label"])
                    _write_table_cell(builder, row["solver"])
                    _write_table_cell(builder, row["solve_route"])
                    _write_table_cell(builder, row["status"])
                    _write_table_cell(builder, "OK" if row["contract_ok"] else "Fail")
                    _write_table_cell(builder, str(row["iterations"]))
                    _write_table_cell(builder, f"{row['solution_error']:.2e}")
                    _write_table_cell(builder, f"{row['residual']:.2e}")
                    _write_table_cell(builder, f"{row['complementarity']:.2e}")
                    _write_table_cell(builder, f"{row['elapsed_us']:.1f}")
                builder.end_table()

        if builder.collapsing_header("Solver selection guide", default_open=False):
            if builder.begin_table(
                "lcp_solver_selection_guide",
                [
                    "Family",
                    "Solvers",
                    "Best fit",
                    "Strength",
                    "Tradeoff",
                    "Evidence cue",
                ],
            ):
                for row in _SOLVER_GUIDANCE_ROWS:
                    builder.table_next_row()
                    _write_table_cell(builder, row["family"])
                    _write_table_cell(builder, row["solvers"])
                    _write_table_cell(builder, row["best_fit"])
                    _write_table_cell(builder, row["strength"])
                    _write_table_cell(builder, row["tradeoff"])
                    _write_table_cell(builder, row["evidence"])
                builder.end_table()

        if builder.collapsing_header("Solver comparison profile", default_open=False):
            if builder.begin_table(
                "lcp_solver_profile",
                [
                    "Solver",
                    "Native cases",
                    "OK",
                    "Native OK",
                    "Delegated OK",
                    "Total us",
                    "Worst error",
                    "Worst residual",
                    "Worst comp",
                    "Slowest case",
                    "Slowest us",
                ],
            ):
                for row in standalone_solver_profile_rows:
                    builder.table_next_row()
                    _write_table_cell(builder, row["solver"])
                    _write_table_cell(builder, row["native_surfaces"])
                    _write_table_cell(
                        builder,
                        f"{row['contract_ok_count']}/{row['problem_count']}",
                    )
                    _write_table_cell(
                        builder,
                        (
                            f"{row['native_contract_ok_count']}/"
                            f"{row['native_case_count']}"
                        ),
                    )
                    _write_table_cell(
                        builder,
                        (
                            f"{row['delegated_contract_ok_count']}/"
                            f"{row['delegated_case_count']}"
                        ),
                    )
                    _write_table_cell(builder, f"{row['total_elapsed_us']:.1f}")
                    _write_table_cell(builder, f"{row['max_solution_error']:.2e}")
                    _write_table_cell(builder, f"{row['max_residual']:.2e}")
                    _write_table_cell(builder, f"{row['max_complementarity']:.2e}")
                    _write_table_cell(builder, row["slowest_case"])
                    _write_table_cell(builder, f"{row['slowest_elapsed_us']:.1f}")
                builder.end_table()

            if builder.begin_table(
                "lcp_advanced_solver_parameters",
                ["Solver", "Surface", "Parameters", "Defaults", "Benchmark"],
            ):
                for row in advanced_solver_parameter_rows:
                    builder.table_next_row()
                    _write_table_cell(builder, row["solver"])
                    _write_table_cell(builder, row["surface"])
                    _write_table_cell(builder, row["parameters"])
                    _write_table_cell(builder, row["defaults"])
                    _write_table_cell(builder, row["benchmark_filter"])
                builder.end_table()

        builder.separator()
        boxed_lcp.bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=state["cases"][0].bridge.render_world,
        pre_step=pre_step,
        force_drag=force_drag,
        panels=[ScenePanel("LCP Physics", build_panel, initial_size=(440.0, 520.0))],
        renderable_provider=renderable_provider,
        info={
            "comparison_worlds": [case.world for case in state["cases"]],
            "contact_solver_methods": [case.label for case in state["cases"]],
            "packets": [
                "sliding_friction",
                "static_friction_ramp",
                "billiard_collision",
                "high_mass_ratio_stack",
                "thin_card_pile",
            ],
            "live_packet_rows": _copy_rows(_LIVE_PACKET_ROWS),
            "representative_requirement_rows": _copy_rows(
                _REPRESENTATIVE_REQUIREMENT_ROWS
            ),
            "live_metrics_snapshot": live_metrics_snapshot,
            "benchmark_packet_rows": _copy_rows(_BENCHMARK_PACKET_ROWS),
            "performance_profile_rows": _copy_rows(_PERFORMANCE_PROFILE_ROWS),
            "performance_profile_evidence_schema_rows": _copy_rows(
                _PERFORMANCE_PROFILE_EVIDENCE_SCHEMA_ROWS
            ),
            "performance_profile_evidence_summary_rows": [
                dict(row) for row in performance_profile_evidence_summary_rows
            ],
            "solver_rows": _copy_rows(_SOLVER_SUPPORT_ROWS),
            "solver_guidance_rows": _copy_rows(_SOLVER_GUIDANCE_ROWS),
            "standalone_solver_rows": [dict(row) for row in standalone_solver_rows],
            "standalone_problem_rows": [
                dict(row) for row in standalone_problem_rows
            ],
            "standalone_problem_summary_rows": [
                dict(row) for row in standalone_problem_summary_rows
            ],
            "standalone_solver_profile_rows": [
                dict(row) for row in standalone_solver_profile_rows
            ],
            "advanced_solver_parameter_rows": [
                dict(row) for row in advanced_solver_parameter_rows
            ],
            "solver_manifest_summary": _solver_manifest_summary(),
            "benchmark_command": _BENCHMARK_COMMAND,
            "benchmark_smoke_filter": _BENCHMARK_SMOKE_FILTER,
            "performance_profile_refresh_command": (
                _PERFORMANCE_PROFILE_REFRESH_COMMAND
            ),
            "performance_profile_smoke_command": _PERFORMANCE_PROFILE_SMOKE_COMMAND,
            "representative_benchmark_filter": _REPRESENTATIVE_BENCHMARK_FILTER,
            "representative_benchmark_command": _REPRESENTATIVE_BENCHMARK_COMMAND,
            "standalone_lcp_solvers_exposed_in_dartpy": (
                _STANDALONE_LCP_SOLVERS_EXPOSED_IN_DARTPY
            ),
        },
    )


SCENE = PythonDemoScene(
    id="lcp_physics",
    title="LCP Physics",
    category="LCP Physics",
    summary="Matched DART 7 contact packets comparing sequential impulse and boxed LCP.",
    build=build,
)
