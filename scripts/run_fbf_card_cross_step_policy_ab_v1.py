#!/usr/bin/env python3
"""Run the frozen 90-step card-house cross-step-policy A/B protocol v1."""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import os
import platform
import statistics
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence

from run_fbf_literal_arch101_v1 import (
    _executable_identity,
    _sha256_file,
    _tool_identity,
)

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BINARY = (
    ROOT / "build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace"
)
PROTOCOL = (
    ROOT / "docs/dev_tasks/fbf_exact_coulomb_friction/"
    "CARD_HOUSE_CROSS_STEP_POLICY_AB_V1.md"
)
TRACE_SOURCE = ROOT / "tests/benchmark/integration/fbf_paper_trace.cpp"
IDENTITY_HELPER_SOURCE = ROOT / "scripts/run_fbf_literal_arch101_v1.py"

SCHEMA_VERSION = "dart.fbf_card_cross_step_policy_ab_v1/v1"
EXPECTED_PROTOCOL_CONTRACT_SHA256 = (
    "a2babf6d28f802fce85d49abf4231c5ac94f3db62747f73a093cbba5ee3db4b5"
)
EXPECTED_HEADER_SHA256 = (
    "b8590420ebcbf62c522fb88a5cad06f0c5ebd917400cf578c4c63f2d76dc1a36"
)
EXPECTED_STEPS = 90
EXPECTED_CPU = 8
POLICIES = ("dart_current", "author_policy_inspired_b3f3c5c")
AUTHOR_POLICY = "author_policy_inspired_b3f3c5c"
POLICY_COLUMNS = (
    "cross_step_policy_contract",
    "requested_cross_step_policy",
    "actual_cross_step_policy",
    "requested_native_contact_manifold_mode",
    "actual_native_contact_manifold_mode",
    "collision_max_contacts",
    "collision_max_contacts_per_pair",
    "step_exact_attempts",
    "step_exact_max_iterations_accepted",
    "step_warm_start_gamma_caps",
    "step_unconverged_warm_start_cache_skips",
    "worst_exact_residual_to_date",
    "last_exact_diagnostics_contract",
    "last_exact_initial_natural_map_residual",
    "last_exact_final_natural_map_residual",
    "last_exact_uncapped_initial_gamma",
    "last_exact_warm_start_gamma_cap_applied",
    "warm_start_match_mode",
    "warm_start_match_distance",
    "warm_start_normal_cosine",
    "strict_warm_start_match_distance",
    "warm_start_max_age",
    "persistent_gamma_safe_bound_scale",
    "minimum_adaptive_gamma",
    "maximum_adaptive_gamma",
    "warm_start_gamma_natural_residual_threshold",
    "warm_start_gamma_cap",
    "persist_uncapped_gamma_after_warm_cap",
    "require_residual_improvement_for_unconverged_cache_save",
    "coupling_variation_tolerance",
    "shrink_factor",
    "max_step_shrink_iterations",
)


class EvidenceError(RuntimeError):
    """Raised when output cannot be accepted as frozen protocol evidence."""


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _write_json(path: Path, value: object) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _protocol_contract_sha256() -> str:
    text = PROTOCOL.read_text(encoding="utf-8")
    marker = "## Frozen v1 result"
    if marker not in text:
        raise EvidenceError("protocol result marker is missing")
    frozen = (text.split(marker, 1)[0].rstrip() + "\n").encode()
    digest = _sha256_bytes(frozen)
    if digest != EXPECTED_PROTOCOL_CONTRACT_SHA256:
        raise EvidenceError("frozen protocol contract hash drifted")
    return digest


def _execution_identity(binary: Path) -> dict[str, Any]:
    return {
        "protocol_contract_sha256": _protocol_contract_sha256(),
        "runner_source_sha256": _sha256_file(Path(__file__).resolve()),
        "trace_source_sha256": _sha256_file(TRACE_SOURCE),
        "identity_helper_source_sha256": _sha256_file(IDENTITY_HELPER_SOURCE),
        "trace_executable": _executable_identity(binary),
        "taskset_tool": _tool_identity("taskset"),
    }


def _command(binary: Path, policy: str) -> list[str]:
    if policy not in POLICIES:
        raise EvidenceError(f"unsupported cross-step policy: {policy}")
    return [
        "taskset",
        "--cpu-list",
        str(EXPECTED_CPU),
        str(binary),
        "card_house_26_settle_projectile_full",
        "exact_fbf",
        "1",
        str(EXPECTED_STEPS),
        "nan",
        "performance",
        "default",
        "default",
        "1",
        "paper_cpu",
        "native",
        "default",
        "0",
        "0",
        "default",
        policy,
    ]


def _captured_text(value: str | bytes | None) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return value


def _run_captured(command: Sequence[str], timeout: float) -> tuple[str, str, int]:
    try:
        completed = subprocess.run(
            list(command),
            cwd=ROOT,
            text=True,
            capture_output=True,
            check=False,
            timeout=timeout,
        )
        return completed.stdout, completed.stderr, completed.returncode
    except subprocess.TimeoutExpired as error:
        return _captured_text(error.stdout), _captured_text(error.stderr), 124


def _parse_trace(text: str) -> tuple[list[str], list[dict[str, str]]]:
    lines = text.splitlines()
    if not lines:
        raise EvidenceError("trace emitted no CSV")
    if _sha256_bytes((lines[0] + "\n").encode()) != EXPECTED_HEADER_SHA256:
        raise EvidenceError("trace header hash mismatch")
    reader = csv.DictReader(io.StringIO(text))
    header = list(reader.fieldnames or ())
    if len(header) != 115 or len(set(header)) != 115:
        raise EvidenceError("cross-step policy trace must have 115 unique columns")
    if tuple(header[-len(POLICY_COLUMNS) :]) != POLICY_COLUMNS:
        raise EvidenceError("cross-step policy trace suffix schema drifted")
    rows = list(reader)
    if any(None in row or any(value is None for value in row.values()) for row in rows):
        raise EvidenceError("trace row has missing or extra columns")
    return header, rows


def _int(row: dict[str, str], field: str) -> int:
    try:
        return int(row[field])
    except (KeyError, ValueError) as error:
        raise EvidenceError(f"invalid integer field {field!r}") from error


def _float(row: dict[str, str], field: str) -> float:
    try:
        value = float(row[field])
    except (KeyError, ValueError) as error:
        raise EvidenceError(f"invalid floating-point field {field!r}") from error
    if not math.isfinite(value):
        raise EvidenceError(f"non-finite required field {field!r}")
    return value


def _optional_finite_float(row: dict[str, str], field: str) -> float | None:
    try:
        value = float(row[field])
    except (KeyError, ValueError) as error:
        raise EvidenceError(f"invalid floating-point field {field!r}") from error
    if math.isnan(value):
        return None
    if not math.isfinite(value):
        raise EvidenceError(f"infinite field {field!r}")
    return value


def _require_close(row: dict[str, str], field: str, expected: float) -> None:
    if not math.isclose(_float(row, field), expected, rel_tol=1e-12, abs_tol=1e-15):
        raise EvidenceError(f"frozen floating-point field {field!r} drifted")


def _require_nan(row: dict[str, str], field: str) -> None:
    try:
        value = float(row[field])
    except (KeyError, ValueError) as error:
        raise EvidenceError(f"invalid floating-point field {field!r}") from error
    if not math.isnan(value):
        raise EvidenceError(f"frozen disabled field {field!r} is not NaN")


def _validate_policy_readbacks(row: dict[str, str], policy: str) -> None:
    if policy == "dart_current":
        strings = {"warm_start_match_mode": "either_body_local_feature"}
        integers = {
            "strict_warm_start_match_distance": 0,
            "warm_start_max_age": -1,
            "persist_uncapped_gamma_after_warm_cap": 0,
            "require_residual_improvement_for_unconverged_cache_save": 0,
            "max_step_shrink_iterations": 20,
        }
        floating = {
            "step_size_recovery_growth_factor": 1.05,
            "step_size_scale": 1.0,
            "warm_start_match_distance": 0.025,
            "warm_start_normal_cosine": 0.9,
            "persistent_gamma_safe_bound_scale": 1.0,
            "coupling_variation_tolerance": 0.9,
            "shrink_factor": 0.7,
        }
        for field in (
            "minimum_adaptive_gamma",
            "maximum_adaptive_gamma",
            "warm_start_gamma_natural_residual_threshold",
            "warm_start_gamma_cap",
            "last_exact_initial_natural_map_residual",
            "last_exact_final_natural_map_residual",
        ):
            _require_nan(row, field)
    elif policy == AUTHOR_POLICY:
        strings = {"warm_start_match_mode": "ordered_body_b_local_feature"}
        integers = {
            "strict_warm_start_match_distance": 1,
            "warm_start_max_age": 3,
            "persist_uncapped_gamma_after_warm_cap": 1,
            "require_residual_improvement_for_unconverged_cache_save": 1,
            "max_step_shrink_iterations": 8,
        }
        floating = {
            "step_size_recovery_growth_factor": 1.0 / 0.7,
            "step_size_scale": 10.0,
            "warm_start_match_distance": 0.02,
            "warm_start_normal_cosine": 0.9,
            "persistent_gamma_safe_bound_scale": 10.0,
            "minimum_adaptive_gamma": 1e-6,
            "maximum_adaptive_gamma": 1e6,
            "warm_start_gamma_natural_residual_threshold": 1e-4,
            "warm_start_gamma_cap": 1e4,
            "coupling_variation_tolerance": 0.9,
            "shrink_factor": 0.7,
        }
        for field in (
            "last_exact_initial_natural_map_residual",
            "last_exact_final_natural_map_residual",
        ):
            if _float(row, field) < 0.0:
                raise EvidenceError(f"negative natural-map residual {field!r}")
    else:
        raise EvidenceError(f"unsupported cross-step policy: {policy}")

    for field, expected in strings.items():
        if row.get(field) != expected:
            raise EvidenceError(f"frozen string field {field!r} drifted")
    for field, expected in integers.items():
        if _int(row, field) != expected:
            raise EvidenceError(f"frozen integer field {field!r} drifted")
    for field, expected in floating.items():
        _require_close(row, field, expected)


def _validate_common_row(row: dict[str, str], policy: str, step: int) -> None:
    strings = {
        "scenario": "card_house_26_settle_projectile_full",
        "solver": "exact_fbf",
        "solver_contract": "paper_cpu",
        "precision_contract": "float64_vs_paper_float32",
        "scene_contract": (
            "reconstructed_cards_density_1000_from_newton_default_"
            "cube_drop_author_overrides_unavailable"
        ),
        "baumgarte_contract": "split_impulse_no_velocity_baumgarte_vs_paper",
        "collision_frontend": "native",
        "inner_local_solver": "exact_metric",
        "row_operator_request": "contact_row_no_dense_snapshot",
        "row_operator_mode": "contact_row_no_dense_snapshot",
        "initial_gamma_contract": "automatic_safe_bound",
        "cross_step_policy_contract": "card_house_cross_step_policy_ab_v1",
        "requested_cross_step_policy": policy,
        "actual_cross_step_policy": policy,
        "requested_native_contact_manifold_mode": "compact",
        "actual_native_contact_manifold_mode": "compact",
    }
    for field, expected in strings.items():
        if row.get(field) != expected:
            raise EvidenceError(f"frozen string field {field!r} drifted")

    integers = {
        "step": step,
        "inner_sweeps_requested": 10,
        "fixed_inner_sweeps_requested": 1,
        "step_size_persistence_enabled": 1,
        "requested_threads": 1,
        "actual_threads": 1,
        "finite_state": 1,
        "card_count": 26,
        "projectile_count": 0,
        "max_outer_iterations": 200,
        "accept_outer_max_iterations": 1,
        "inner_local_iterations": 1,
        "adaptive_step_size_enabled": 1,
        "warm_start_enabled": 1,
        "projected_gradient_retry_enabled": 0,
        "dense_residual_polish_enabled": 0,
        "fallback_to_boxed_lcp_enabled": 0,
        "diagonal_seed_enabled": 0,
        "matrix_free_seed_enabled": 0,
        "split_impulse_enabled": 1,
        "step_parallel_contact_row_delassus_products": 0,
        "max_contact_row_participants_to_date": 1,
        "projectile_card_contacts": 0,
        "collision_max_contacts": 512,
        "collision_max_contacts_per_pair": 4,
    }
    for field, expected in integers.items():
        if _int(row, field) != expected:
            raise EvidenceError(f"frozen integer field {field!r} drifted")

    if row.get("phase") != "settle":
        raise EvidenceError("card-house phase drifted")
    if row.get("exact_diagnostics_contract") != (
        "last_exact_group_public_getters_contact_row_no_dense_snapshot_"
        "warm_fraction_over_step_contacts"
    ):
        raise EvidenceError("exact diagnostics contract drifted")
    if row.get("exact_contact_row_logical_cpus_to_date") != "none":
        raise EvidenceError("single-thread exact CPU readback drifted")
    if row.get("max_phase_contact_row_logical_cpus_to_date") != "none":
        raise EvidenceError("single-thread phase CPU readback drifted")
    if not row.get("tracked_body") or row["tracked_body"] == "none":
        raise EvidenceError("tracked card identity is unavailable")

    _require_close(row, "time", step / 60.0)
    _require_close(row, "tolerance", 1e-6)
    _require_close(row, "outer_relaxation", 1.0)
    for field in (
        "wall_ms",
        "penetration_depth_min",
        "penetration_depth_median",
        "penetration_depth_p95",
        "penetration_depth_max",
        "residual",
        "residual_primal_feasibility",
        "residual_dual_feasibility",
        "residual_complementarity",
        "accepted_gamma",
        "safe_gamma",
        "coupling_variation_ratio",
        "warm_start_matched_fraction",
        "min_card_axis_up",
        "min_center_height",
        "max_card_horizontal_travel",
        "max_projectile_speed",
        "x",
        "y",
        "z",
        "vx",
        "vy",
        "vz",
        "up_z",
        "max_card_center_displacement_from_initial",
        "min_card_orientation_alignment_from_initial",
        "worst_exact_residual_to_date",
        "last_exact_uncapped_initial_gamma",
    ):
        _float(row, field)
    _optional_finite_float(row, "step_size_persistence_request")

    if _float(row, "wall_ms") < 0.0:
        raise EvidenceError("negative wall time")
    if _float(row, "residual") < 0.0:
        raise EvidenceError("negative scaled residual")
    if _float(row, "worst_exact_residual_to_date") < 0.0:
        raise EvidenceError("negative cumulative worst scaled residual")
    if not 0.0 <= _float(row, "warm_start_matched_fraction") <= 1.0:
        raise EvidenceError("warm-start matched fraction is outside [0, 1]")

    contacts = _int(row, "contacts")
    attempts = _int(row, "step_exact_attempts")
    solves = _int(row, "step_exact_solves")
    failures = _int(row, "step_exact_failures")
    if contacts <= 0 or contacts > 512:
        raise EvidenceError("card-house row must contain 1..512 contacts")
    if attempts <= 0:
        raise EvidenceError("card-house row has no exact attempt")
    if (
        min(
            solves,
            failures,
            _int(row, "step_fallbacks"),
            _int(row, "step_warm_starts"),
            _int(row, "unique_colliding_body_pairs"),
            _int(row, "warm_start_matched_contacts"),
        )
        < 0
    ):
        raise EvidenceError("negative exact counter")
    if _int(row, "unique_colliding_body_pairs") > contacts:
        raise EvidenceError("unique contact-pair count exceeds contacts")
    if attempts != solves + failures:
        raise EvidenceError("exact attempts do not equal solves plus failures")
    accepted_caps = _int(row, "step_exact_max_iterations_accepted")
    if accepted_caps < 0 or accepted_caps > solves:
        raise EvidenceError("accepted-cap count exceeds exact solves")
    for field in (
        "step_warm_start_gamma_caps",
        "step_unconverged_warm_start_cache_skips",
        "step_contact_row_delassus_products",
        "step_fbf_iterations",
        "shrink_iterations",
    ):
        if _int(row, field) < 0:
            raise EvidenceError(f"negative counter field {field!r}")
    if _int(row, "step_contact_row_delassus_products") == 0:
        raise EvidenceError("exact attempt used no contact-row products")
    for field in (
        "step_size_persistence_used",
        "last_exact_warm_start_gamma_cap_applied",
    ):
        if _int(row, field) not in (0, 1):
            raise EvidenceError(f"non-boolean field {field!r}")
    matched = _int(row, "warm_start_matched_contacts")
    if matched < 0 or matched > contacts:
        raise EvidenceError("warm-start match count is outside the contact set")
    if row.get("status") not in {
        "success",
        "max_iterations_accepted",
        "fbf_failed",
    }:
        raise EvidenceError("unknown exact wrapper status")
    if row.get("last_exact_diagnostics_contract") not in {
        "last_exact_group_only_single_group",
        "last_exact_group_only_multi_group_noncomparable",
    }:
        raise EvidenceError("last-group diagnostics contract drifted")

    _validate_policy_readbacks(row, policy)


def _mean(values: Sequence[float]) -> float:
    return statistics.fmean(values) if values else 0.0


def _distribution(values: Sequence[float]) -> dict[str, float]:
    return {
        "minimum": min(values),
        "maximum": max(values),
        "mean": _mean(values),
        "median": statistics.median(values),
    }


def _arm_summary(
    policy: str, rows: Sequence[dict[str, str]], returncode: int
) -> dict[str, Any]:
    if returncode not in (0, 1):
        raise EvidenceError(f"{policy} returned {returncode}, expected zero or one")
    if not rows or len(rows) > EXPECTED_STEPS:
        raise EvidenceError(f"{policy} row count is outside 1..90")
    for step, row in enumerate(rows, 1):
        _validate_common_row(row, policy, step)

    failure_rows = [
        index for index, row in enumerate(rows, 1) if _int(row, "step_exact_failures")
    ]
    if returncode == 0:
        if len(rows) != EXPECTED_STEPS:
            raise EvidenceError(f"{policy} zero return is not a complete trace")
        process_exit_class = "complete_exit_zero"
    elif failure_rows:
        if failure_rows != [len(rows)]:
            raise EvidenceError(
                f"{policy} failed prefix does not stop at its sole failing row"
            )
        process_exit_class = "exact_failure_prefix"
    elif (
        len(rows) == EXPECTED_STEPS
        and sum(_int(row, "step_exact_failures") for row in rows) == 0
        and _float(rows[-1], "residual") > 1e-6
    ):
        process_exit_class = "complete_terminal_convergence_gate_failure"
    else:
        raise EvidenceError(f"{policy} return one has no frozen exit class")

    attempts = [_int(row, "step_exact_attempts") for row in rows]
    solves = [_int(row, "step_exact_solves") for row in rows]
    failures = [_int(row, "step_exact_failures") for row in rows]
    fallbacks = [_int(row, "step_fallbacks") for row in rows]
    accepted_caps = [_int(row, "step_exact_max_iterations_accepted") for row in rows]
    worst_residuals = [_float(row, "worst_exact_residual_to_date") for row in rows]
    strict_rows = [
        attempts[index] == solves[index]
        and failures[index] == 0
        and fallbacks[index] == 0
        and accepted_caps[index] == 0
        and worst_residuals[index] <= 1e-6
        for index in range(len(rows))
    ]
    strict = returncode == 0 and len(rows) == EXPECTED_STEPS and all(strict_rows)
    promote = policy == AUTHOR_POLICY and strict

    residuals = [_float(row, "residual") for row in rows]
    initial_natural = [
        value
        for row in rows
        if (
            value := _optional_finite_float(
                row, "last_exact_initial_natural_map_residual"
            )
        )
        is not None
    ]
    final_natural = [
        value
        for row in rows
        if (
            value := _optional_finite_float(
                row, "last_exact_final_natural_map_residual"
            )
        )
        is not None
    ]
    uncapped_gamma = [_float(row, "last_exact_uncapped_initial_gamma") for row in rows]
    persistence_requests = [
        value
        for row in rows
        if (value := _optional_finite_float(row, "step_size_persistence_request"))
        is not None
    ]
    contacts = [_int(row, "contacts") for row in rows]
    warm_fraction = [_float(row, "warm_start_matched_fraction") for row in rows]
    wall = [_float(row, "wall_ms") for row in rows]
    terminal = rows[-1]
    return {
        "policy": policy,
        "requested_steps": EXPECTED_STEPS,
        "emitted_steps": len(rows),
        "child_returncode": returncode,
        "process_exit_class": process_exit_class,
        "first_failed_step": failure_rows[0] if failure_rows else None,
        "strict_gate": strict,
        "promotion_to_separately_preregistered_600": promote,
        "counters": {
            "exact_attempts": sum(attempts),
            "exact_solves": sum(solves),
            "exact_failures": sum(failures),
            "boxed_lcp_fallbacks": sum(fallbacks),
            "accepted_outer_iteration_caps": sum(accepted_caps),
            "warm_start_gamma_caps": sum(
                _int(row, "step_warm_start_gamma_caps") for row in rows
            ),
            "unconverged_warm_start_cache_skips": sum(
                _int(row, "step_unconverged_warm_start_cache_skips") for row in rows
            ),
        },
        "strict_gate_failed_rows": sum(not value for value in strict_rows),
        "scaled_residual": _distribution(residuals),
        "whole_run_worst_scaled_residual": max(worst_residuals),
        "natural_map_last_group_diagnostic": {
            "comparable_to_whole_step": False,
            "finite_row_count": len(final_natural),
            "initial": _distribution(initial_natural) if initial_natural else None,
            "final": _distribution(final_natural) if final_natural else None,
            "improved_rows": sum(
                final < initial
                for initial, final in zip(initial_natural, final_natural)
            ),
        },
        "uncapped_initial_gamma": _distribution(uncapped_gamma),
        "gamma_persistence": {
            "used_rows": sum(_int(row, "step_size_persistence_used") for row in rows),
            "finite_request_rows": len(persistence_requests),
            "request": (
                _distribution(persistence_requests) if persistence_requests else None
            ),
        },
        "warm_start_matched_fraction": _distribution(warm_fraction),
        "contacts": {
            "minimum": min(contacts),
            "maximum": max(contacts),
            "mean": _mean(contacts),
        },
        "card_pose": {
            "maximum_displacement": max(
                _float(row, "max_card_center_displacement_from_initial") for row in rows
            ),
            "minimum_orientation_alignment": min(
                _float(row, "min_card_orientation_alignment_from_initial")
                for row in rows
            ),
            "minimum_axis_up": min(_float(row, "min_card_axis_up") for row in rows),
            "minimum_center_height": min(
                _float(row, "min_center_height") for row in rows
            ),
        },
        "terminal": {
            "step": _int(terminal, "step"),
            "status": terminal["status"],
            "scaled_residual": _float(terminal, "residual"),
            "worst_scaled_residual_to_date": _float(
                terminal, "worst_exact_residual_to_date"
            ),
            "exact_attempts": _int(terminal, "step_exact_attempts"),
            "exact_solves": _int(terminal, "step_exact_solves"),
            "exact_failures": _int(terminal, "step_exact_failures"),
        },
        "wall_time_diagnostic_only": {
            "mean_ms": _mean(wall),
            "median_ms": statistics.median(wall),
            "maximum_ms": max(wall),
        },
    }


def _delta(
    author: float | int | None, current: float | int | None
) -> float | int | None:
    if author is None or current is None:
        return None
    return author - current


def _comparison(summaries: dict[str, dict[str, Any]]) -> dict[str, Any]:
    current = summaries["dart_current"]
    author = summaries[AUTHOR_POLICY]
    current_natural = current["natural_map_last_group_diagnostic"]
    author_natural = author["natural_map_last_group_diagnostic"]
    return {
        "direction": "author_policy_inspired_b3f3c5c_minus_dart_current",
        "compound_cross_step_policy_is_only_intended_factor": True,
        "comparison_artifact_integrity_valid": True,
        "paper_parity": False,
        "causal_component_attribution": False,
        "timing_used_in_comparison_or_verdict": False,
        "strict_gate": {
            policy: summaries[policy]["strict_gate"] for policy in POLICIES
        },
        "process_exit_class": {
            policy: summaries[policy]["process_exit_class"] for policy in POLICIES
        },
        "promotion_to_separately_preregistered_600": author[
            "promotion_to_separately_preregistered_600"
        ],
        "auto_launched_600_step_child": False,
        "deterministic_deltas": {
            "emitted_steps": _delta(author["emitted_steps"], current["emitted_steps"]),
            "first_failed_step": _delta(
                author["first_failed_step"], current["first_failed_step"]
            ),
            "exact_attempts": _delta(
                author["counters"]["exact_attempts"],
                current["counters"]["exact_attempts"],
            ),
            "exact_solves": _delta(
                author["counters"]["exact_solves"],
                current["counters"]["exact_solves"],
            ),
            "exact_failures": _delta(
                author["counters"]["exact_failures"],
                current["counters"]["exact_failures"],
            ),
            "boxed_lcp_fallbacks": _delta(
                author["counters"]["boxed_lcp_fallbacks"],
                current["counters"]["boxed_lcp_fallbacks"],
            ),
            "accepted_outer_iteration_caps": _delta(
                author["counters"]["accepted_outer_iteration_caps"],
                current["counters"]["accepted_outer_iteration_caps"],
            ),
            "warm_start_gamma_caps": _delta(
                author["counters"]["warm_start_gamma_caps"],
                current["counters"]["warm_start_gamma_caps"],
            ),
            "unconverged_warm_start_cache_skips": _delta(
                author["counters"]["unconverged_warm_start_cache_skips"],
                current["counters"]["unconverged_warm_start_cache_skips"],
            ),
            "mean_scaled_residual": _delta(
                author["scaled_residual"]["mean"],
                current["scaled_residual"]["mean"],
            ),
            "whole_run_worst_scaled_residual": _delta(
                author["whole_run_worst_scaled_residual"],
                current["whole_run_worst_scaled_residual"],
            ),
            "mean_last_group_natural_residual": _delta(
                (
                    author_natural["final"]["mean"]
                    if author_natural["final"] is not None
                    else None
                ),
                (
                    current_natural["final"]["mean"]
                    if current_natural["final"] is not None
                    else None
                ),
            ),
            "mean_warm_start_matched_fraction": _delta(
                author["warm_start_matched_fraction"]["mean"],
                current["warm_start_matched_fraction"]["mean"],
            ),
            "gamma_persistence_used_rows": _delta(
                author["gamma_persistence"]["used_rows"],
                current["gamma_persistence"]["used_rows"],
            ),
            "mean_contacts": _delta(
                author["contacts"]["mean"], current["contacts"]["mean"]
            ),
            "maximum_card_displacement": _delta(
                author["card_pose"]["maximum_displacement"],
                current["card_pose"]["maximum_displacement"],
            ),
            "minimum_card_orientation_alignment": _delta(
                author["card_pose"]["minimum_orientation_alignment"],
                current["card_pose"]["minimum_orientation_alignment"],
            ),
            "terminal_scaled_residual": _delta(
                author["terminal"]["scaled_residual"],
                current["terminal"]["scaled_residual"],
            ),
        },
        "claim_boundary": (
            "Source-informed compound DART cross-step-policy comparison on the "
            "reconstructed 26-card scene only; not paper parity, source "
            "equivalence, component causality, real time, or a timing verdict."
        ),
    }


def _report(summaries: dict[str, dict[str, Any]], comparison: dict[str, Any]) -> str:
    lines = [
        "# Card-House Cross-Step Policy A/B v1",
        "",
        "Comparison artifact integrity valid: **yes**.",
        "Paper parity: **no**. Timing verdict: **none**.",
        "",
    ]
    for policy in POLICIES:
        summary = summaries[policy]
        lines.extend(
            [
                f"## {policy}",
                "",
                f"- process exit class: {summary['process_exit_class']}",
                f"- emitted steps: {summary['emitted_steps']}/90",
                f"- first failed step: {summary['first_failed_step']}",
                f"- strict gate: {summary['strict_gate']}",
                f"- attempts / solves / failures: "
                f"{summary['counters']['exact_attempts']} / "
                f"{summary['counters']['exact_solves']} / "
                f"{summary['counters']['exact_failures']}",
                f"- fallbacks / accepted caps: "
                f"{summary['counters']['boxed_lcp_fallbacks']} / "
                f"{summary['counters']['accepted_outer_iteration_caps']}",
                f"- whole-run worst scaled residual: "
                f"{summary['whole_run_worst_scaled_residual']:.17g}",
                f"- promotion to separately preregistered 600: "
                f"{summary['promotion_to_separately_preregistered_600']}",
                "",
            ]
        )
    lines.extend(
        [
            "The two arms differ by a compound source-informed policy; the run cannot",
            "attribute an outcome to any individual policy component. Wall time is",
            "retained only as a diagnostic and is excluded from comparison and verdicts.",
            "This runner never launches a 600-step child.",
            "",
            f"Deterministic delta object: "
            f"`{json.dumps(comparison['deterministic_deltas'], sort_keys=True)}`",
            "",
        ]
    )
    return "\n".join(lines)


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=DEFAULT_BINARY)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=3600.0)
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        parser.error("--timeout must be positive and finite")
    return args


def _prepare_output(path: Path) -> None:
    if path.exists():
        raise EvidenceError(f"output directory must be fresh: {path}")
    path.mkdir(parents=True)
    for policy in POLICIES:
        (path / policy).mkdir()


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    binary = args.binary.resolve()
    output = args.output_dir.resolve()
    if not binary.is_file():
        raise SystemExit(f"trace binary not found: {binary}")
    try:
        _prepare_output(output)
        identity = _execution_identity(binary)
    except (EvidenceError, OSError, ValueError) as error:
        raise SystemExit(str(error)) from error

    invocation: dict[str, Any] = {
        "commands": {policy: _command(binary, policy) for policy in POLICIES},
        "run_order": list(POLICIES),
        "cwd": str(ROOT),
        "timeout_seconds_per_child": args.timeout,
        "identity_rechecks": [],
        "children": {},
        "auto_launch_600": False,
    }
    summaries: dict[str, dict[str, Any]] = {}
    errors: dict[str, str] = {}
    for policy in POLICIES:
        stdout, stderr, returncode = _run_captured(
            invocation["commands"][policy], args.timeout
        )
        raw_path = output / policy / "raw.csv"
        stderr_path = output / policy / "stderr.txt"
        raw_path.write_text(stdout, encoding="utf-8")
        stderr_path.write_text(stderr, encoding="utf-8")
        invocation["children"][policy] = {
            "returncode": returncode,
            "stdout_bytes": len(stdout.encode()),
            "stderr_bytes": len(stderr.encode()),
        }
        try:
            rechecked_identity = _execution_identity(binary)
            identity_matches = rechecked_identity == identity
            invocation["identity_rechecks"].append(
                {"after": policy, "matches_initial": identity_matches}
            )
            if not identity_matches:
                raise EvidenceError(f"execution identity drifted after {policy}")
            _, rows = _parse_trace(stdout)
            summaries[policy] = _arm_summary(policy, rows, returncode)
        except (EvidenceError, OSError, ValueError) as error:
            errors[policy] = str(error)

    integrity_valid = not errors and set(summaries) == set(POLICIES)
    promotion = (
        summaries.get(AUTHOR_POLICY, {}).get(
            "promotion_to_separately_preregistered_600", False
        )
        if integrity_valid
        else False
    )
    if integrity_valid:
        comparison = _comparison(summaries)
    else:
        comparison = {
            "comparison_artifact_integrity_valid": False,
            "paper_parity": False,
            "timing_used_in_comparison_or_verdict": False,
            "promotion_to_separately_preregistered_600": False,
            "auto_launched_600_step_child": False,
            "errors": errors,
        }
    summary_document = {
        "schema_version": SCHEMA_VERSION,
        "comparison_artifact_integrity_valid": integrity_valid,
        "promotion_to_separately_preregistered_600": promotion,
        "arms": summaries,
        "errors": errors,
    }

    invocation_path = output / "invocation.json"
    summary_path = output / "summary.json"
    comparison_path = output / "comparison.json"
    report_path = output / "REPORT.md"
    _write_json(invocation_path, invocation)
    _write_json(summary_path, summary_document)
    _write_json(comparison_path, comparison)
    report_path.write_text(
        (
            _report(summaries, comparison)
            if integrity_valid
            else "# Invalid card-house cross-step-policy A/B artifact\n\n"
            f"{json.dumps(errors, sort_keys=True)}\n"
        ),
        encoding="utf-8",
    )

    payload_paths = [
        output / policy / name
        for policy in POLICIES
        for name in ("raw.csv", "stderr.txt")
        if (output / policy / name).is_file()
    ] + [invocation_path, summary_path, comparison_path, report_path]
    metadata = {
        "schema_version": SCHEMA_VERSION,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "comparison_artifact_integrity_valid": integrity_valid,
        "promotion_to_separately_preregistered_600": promotion,
        "auto_launched_600_step_child": False,
        "paper_parity": False,
        "timing_verdict": None,
        "binary": str(binary),
        "protocol": str(PROTOCOL.relative_to(ROOT)),
        "runner": str(Path(__file__).resolve().relative_to(ROOT)),
        "trace_source": str(TRACE_SOURCE.relative_to(ROOT)),
        "source_identity": identity,
        "affinity": {
            "source": "explicit_taskset",
            "logical_cpus": [EXPECTED_CPU],
        },
        "platform": platform.platform(),
        "python": sys.version,
        "environment": {
            key: os.environ.get(key)
            for key in ("OMP_NUM_THREADS", "DART_DISABLE_COMPILER_CACHE")
        },
        "artifact_sha256": {
            str(path.relative_to(output)): _sha256_file(path) for path in payload_paths
        },
    }
    metadata_path = output / "metadata.json"
    _write_json(metadata_path, metadata)
    index_paths = [*payload_paths, metadata_path]
    artifact_index = {
        "schema_version": SCHEMA_VERSION,
        "files": {
            str(path.relative_to(output)): {
                "size_bytes": path.stat().st_size,
                "sha256": _sha256_file(path),
            }
            for path in sorted(index_paths)
        },
    }
    _write_json(output / "artifact-index.json", artifact_index)
    return 0 if integrity_valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
