#!/usr/bin/env python3
"""Run the frozen Native card-house manifold-sensitivity protocol v1."""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import os
import platform
import shutil
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
    "CARD_HOUSE_MANIFOLD_SENSITIVITY_V1.md"
)
TRACE_SOURCE = ROOT / "tests/benchmark/integration/fbf_paper_trace.cpp"
IDENTITY_HELPER_SOURCE = ROOT / "scripts/run_fbf_literal_arch101_v1.py"

SCHEMA_VERSION = "dart.fbf_card_manifold_sensitivity_v1/v1"
EXPECTED_PROTOCOL_CONTRACT_SHA256 = (
    "1bb8b3ea68a0929a8c30c147acddf677d630948e67d650f19331e21d998c7789"
)
EXPECTED_HEADER_SHA256 = (
    "007311fb28062377dd6a0d26cad1ab4f7e2c99f359afd33554651f3cc0929ef5"
)
EXPECTED_STEPS = 600
EXPECTED_CPU = 8
MODES = ("compact", "four_point_planar")
SENSITIVITY_COLUMNS = (
    "manifold_sensitivity_contract",
    "requested_native_contact_manifold_mode",
    "actual_native_contact_manifold_mode",
    "collision_max_contacts",
    "collision_max_contacts_per_pair",
    "step_exact_max_iterations_accepted",
    "step_internal_fbf_status",
    "step_internal_fbf_best_iteration",
    "step_internal_fbf_best_residual",
    "colliding_body_pair_labels",
    "contact_multiplicity_by_body_pair",
)


class EvidenceError(RuntimeError):
    pass


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


def _command(binary: Path, mode: str) -> list[str]:
    if mode not in MODES:
        raise EvidenceError(f"unsupported manifold mode: {mode}")
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
        mode,
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
    if len(header) != 94 or len(set(header)) != 94:
        raise EvidenceError("sensitivity trace must have 94 unique columns")
    if tuple(header[-len(SENSITIVITY_COLUMNS) :]) != SENSITIVITY_COLUMNS:
        raise EvidenceError("sensitivity trace suffix schema drifted")
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


def _require_close(row: dict[str, str], field: str, expected: float) -> None:
    if not math.isclose(_float(row, field), expected, rel_tol=1e-12, abs_tol=1e-15):
        raise EvidenceError(f"frozen floating-point field {field!r} drifted")


def _validate_pairs(row: dict[str, str]) -> tuple[tuple[str, int], ...]:
    contacts = _int(row, "contacts")
    expected_pairs = _int(row, "unique_colliding_body_pairs")
    labels_text = row["colliding_body_pair_labels"]
    counts_text = row["contact_multiplicity_by_body_pair"]
    if (
        contacts <= 0
        or contacts > 512
        or labels_text == "none"
        or counts_text == "none"
    ):
        raise EvidenceError("card sensitivity row has no contact-pair graph")
    labels = tuple(labels_text.split(";"))
    if labels != tuple(sorted(set(labels))) or len(labels) != expected_pairs:
        raise EvidenceError("pair labels are not sorted, unique, and count-matched")
    for label in labels:
        parts = label.split("|")
        if len(parts) != 2 or not all(parts) or "null" in parts:
            raise EvidenceError("pair label is malformed or contains a null body")
    parsed: dict[str, int] = {}
    for item in counts_text.split(";"):
        label, separator, count_text = item.rpartition("=")
        if not separator or label in parsed:
            raise EvidenceError("pair multiplicity map is malformed")
        try:
            count = int(count_text)
        except ValueError as error:
            raise EvidenceError("pair multiplicity is not an integer") from error
        if count <= 0 or count > 4:
            raise EvidenceError("pair multiplicity is outside the frozen 1..4 cap")
        parsed[label] = count
    if tuple(parsed) != labels or sum(parsed.values()) != contacts:
        raise EvidenceError("pair multiplicities do not match labels or contacts")
    return tuple(parsed.items())


def _validate_common_row(row: dict[str, str], mode: str, step: int) -> tuple[str, ...]:
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
        "initial_gamma_contract": "automatic_safe_bound",
        "manifold_sensitivity_contract": "card_house_native_manifold_sensitivity_v1",
        "requested_native_contact_manifold_mode": mode,
        "actual_native_contact_manifold_mode": mode,
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
        "collision_max_contacts": 512,
        "collision_max_contacts_per_pair": 4,
    }
    for field, expected in integers.items():
        if _int(row, field) != expected:
            raise EvidenceError(f"frozen integer field {field!r} drifted")
    expected_phase = "settle" if step <= 402 else "projectile"
    expected_projectiles = 0 if step <= 402 else 4
    if row.get("phase") != expected_phase:
        raise EvidenceError("card-house phase drifted")
    if _int(row, "card_count") != 26:
        raise EvidenceError("card count drifted")
    if _int(row, "projectile_count") != expected_projectiles:
        raise EvidenceError("projectile phase count drifted")
    if row.get("row_operator_mode") != "contact_row_no_dense_snapshot":
        raise EvidenceError("contact-row execution mode drifted")
    _require_close(row, "time", step / 60.0)
    _require_close(row, "tolerance", 1e-6)
    _require_close(row, "step_size_scale", 1.0)
    _require_close(row, "outer_relaxation", 1.0)
    _require_close(row, "step_size_recovery_growth_factor", 1.05)
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
        "step_internal_fbf_best_residual",
    ):
        _float(row, field)
    solves = _int(row, "step_exact_solves")
    failures = _int(row, "step_exact_failures")
    if solves + failures < 1 or _int(row, "step_fallbacks") != 0:
        raise EvidenceError("contact step has no exact attempt or used fallback")
    accepted_caps = _int(row, "step_exact_max_iterations_accepted")
    if accepted_caps < 0 or accepted_caps > solves:
        raise EvidenceError("aggregate accepted-cap count exceeds exact solves")
    persistence_used = _int(row, "step_size_persistence_used")
    if persistence_used not in (0, 1):
        raise EvidenceError("step-size persistence use is not boolean")
    if _int(row, "shrink_iterations") < 0:
        raise EvidenceError("negative shrink-iteration count")
    matched_contacts = _int(row, "warm_start_matched_contacts")
    if matched_contacts < 0 or matched_contacts > _int(row, "contacts"):
        raise EvidenceError("warm-start match count is outside the contact set")
    wrapper = row["status"]
    internal = row["step_internal_fbf_status"]
    allowed = {
        "success": {"success"},
        "max_iterations_accepted": {"max_iterations"},
        "fbf_failed": {
            "max_iterations",
            "invalid_input",
            "inner_solver_failed",
            "step_size_underflow",
        },
    }
    if wrapper not in allowed or internal not in allowed[wrapper]:
        raise EvidenceError("wrapper and internal FBF statuses are inconsistent")
    best_iteration = _int(row, "step_internal_fbf_best_iteration")
    if best_iteration < 0 or best_iteration > _int(row, "step_fbf_iterations"):
        raise EvidenceError("internal best iteration is outside the exact attempt")
    return _validate_pairs(row)


def _mean(values: Sequence[float]) -> float:
    return statistics.fmean(values) if values else 0.0


def _distribution(values: Sequence[float]) -> dict[str, float]:
    return {
        "minimum": min(values),
        "maximum": max(values),
        "mean": _mean(values),
        "median": statistics.median(values),
    }


def _mode_summary(
    mode: str, rows: Sequence[dict[str, str]], returncode: int
) -> dict[str, Any]:
    if returncode not in (0, 1):
        raise EvidenceError(f"{mode} returned {returncode}, expected zero or one")
    if not rows or len(rows) > EXPECTED_STEPS:
        raise EvidenceError(f"{mode} row count is outside 1..600")
    pair_maps: list[tuple[tuple[str, int], ...]] = []
    for step, row in enumerate(rows, 1):
        pair_maps.append(_validate_common_row(row, mode, step))
    failures = [
        index for index, row in enumerate(rows, 1) if _int(row, "step_exact_failures")
    ]
    total_failures = sum(_int(row, "step_exact_failures") for row in rows)
    if returncode == 0:
        if len(rows) != EXPECTED_STEPS or failures or total_failures:
            raise EvidenceError(
                f"{mode} zero return is not a complete failure-free trace"
            )
    else:
        if (
            failures != [len(rows)]
            or total_failures != 1
            or _int(rows[-1], "step_exact_failures") != 1
        ):
            raise EvidenceError(
                f"{mode} failed prefix does not stop at its sole failure"
            )
    accepted_cap_counts = [
        _int(row, "step_exact_max_iterations_accepted") for row in rows
    ]
    strict_successes = sum(
        _int(row, "step_exact_failures") == 0
        and _int(row, "step_exact_max_iterations_accepted") == 0
        and _float(row, "residual") <= 1e-6
        for row in rows
    )
    accepted_cap_rows = sum(count > 0 for count in accepted_cap_counts)
    accepted_cap_groups = sum(accepted_cap_counts)
    strict_trajectory = (
        returncode == 0
        and len(rows) == EXPECTED_STEPS
        and strict_successes == EXPECTED_STEPS
        and accepted_cap_groups == 0
    )
    contacts = [_int(row, "contacts") for row in rows]
    pairs = [_int(row, "unique_colliding_body_pairs") for row in rows]
    residuals = [_float(row, "residual") for row in rows]
    primal = [_float(row, "residual_primal_feasibility") for row in rows]
    dual = [_float(row, "residual_dual_feasibility") for row in rows]
    complementarity = [_float(row, "residual_complementarity") for row in rows]
    warm_fraction = [_float(row, "warm_start_matched_fraction") for row in rows]
    warm_contacts = [_int(row, "warm_start_matched_contacts") for row in rows]
    best_iterations = [_int(row, "step_internal_fbf_best_iteration") for row in rows]
    wall = [_float(row, "wall_ms") for row in rows]
    pair_identity_union = sorted({label for pairs in pair_maps for label, _ in pairs})
    multiplicities = [count for pairs in pair_maps for _, count in pairs]
    terminal = rows[-1]
    return {
        "mode": mode,
        "requested_steps": EXPECTED_STEPS,
        "emitted_steps": len(rows),
        "child_returncode": returncode,
        "first_failed_step": failures[0] if failures else None,
        "strict_success_rows": strict_successes,
        "accepted_cap_rows": accepted_cap_rows,
        "accepted_cap_exact_groups": accepted_cap_groups,
        "exact_failure_rows": len(failures),
        "exact_failures": total_failures,
        "exact_attempts": sum(
            _int(row, "step_exact_solves") + _int(row, "step_exact_failures")
            for row in rows
        ),
        "boxed_lcp_fallbacks": 0,
        "strict_trajectory_valid": strict_trajectory,
        "physical_outcome_verdict": None,
        "timing_verdict": None,
        "terminal": {
            "wrapper_status": terminal["status"],
            "internal_fbf_status": terminal["step_internal_fbf_status"],
            "iterations": _int(terminal, "step_fbf_iterations"),
            "best_iteration": _int(terminal, "step_internal_fbf_best_iteration"),
            "residual": _float(terminal, "residual"),
            "best_residual": _float(terminal, "step_internal_fbf_best_residual"),
        },
        "contacts": {
            "minimum": min(contacts),
            "maximum": max(contacts),
            "mean": _mean(contacts),
        },
        "unique_pairs": {
            "minimum": min(pairs),
            "maximum": max(pairs),
            "mean": _mean(pairs),
        },
        "pair_graph": {
            "identity_union": pair_identity_union,
            "identity_union_count": len(pair_identity_union),
            "identity_transition_rows": sum(
                tuple(label for label, _ in first)
                != tuple(label for label, _ in second)
                for first, second in zip(pair_maps, pair_maps[1:])
            ),
            "multiplicity_transition_rows": sum(
                first != second for first, second in zip(pair_maps, pair_maps[1:])
            ),
            "multiplicity": _distribution(multiplicities),
        },
        "residual": _distribution(residuals),
        "residual_components": {
            "primal": _distribution(primal),
            "dual": _distribution(dual),
            "complementarity": _distribution(complementarity),
        },
        "best_iteration": {
            "minimum": min(best_iterations),
            "maximum": max(best_iterations),
            "mean": _mean(best_iterations),
            "median": statistics.median(best_iterations),
        },
        "warm_start": {
            "matched_contacts_minimum": min(warm_contacts),
            "matched_contacts_maximum": max(warm_contacts),
            "matched_contacts_mean": _mean(warm_contacts),
            "matched_fraction": _distribution(warm_fraction),
        },
        "step_size_persistence_used_rows": sum(
            _int(row, "step_size_persistence_used") for row in rows
        ),
        "card_pose": {
            "maximum_displacement": max(
                _float(row, "max_card_center_displacement_from_initial") for row in rows
            ),
            "minimum_orientation_alignment": min(
                _float(row, "min_card_orientation_alignment_from_initial")
                for row in rows
            ),
        },
        "wall_time_diagnostic_only": {
            "mean_ms": _mean(wall),
            "median_ms": statistics.median(wall),
            "maximum_ms": max(wall),
        },
    }


def _delta(four: float | int | None, compact: float | int | None) -> float | int | None:
    if four is None or compact is None:
        return None
    return four - compact


def _comparison(summaries: dict[str, dict[str, Any]]) -> dict[str, Any]:
    compact = summaries["compact"]
    four = summaries["four_point_planar"]
    compact_identities = set(compact["pair_graph"]["identity_union"])
    four_identities = set(four["pair_graph"]["identity_union"])
    return {
        "direction": "four_point_planar_minus_compact",
        "manifold_is_only_intended_factor": True,
        "comparison_artifact_integrity_valid": True,
        "paper_parity": False,
        "timing_used_in_verdict": False,
        "strict_trajectory_valid": {
            mode: summaries[mode]["strict_trajectory_valid"] for mode in MODES
        },
        "deltas": {
            "emitted_steps": _delta(four["emitted_steps"], compact["emitted_steps"]),
            "first_failed_step": _delta(
                four["first_failed_step"], compact["first_failed_step"]
            ),
            "strict_success_rows": _delta(
                four["strict_success_rows"], compact["strict_success_rows"]
            ),
            "accepted_cap_rows": _delta(
                four["accepted_cap_rows"], compact["accepted_cap_rows"]
            ),
            "accepted_cap_exact_groups": _delta(
                four["accepted_cap_exact_groups"],
                compact["accepted_cap_exact_groups"],
            ),
            "exact_failure_rows": _delta(
                four["exact_failure_rows"], compact["exact_failure_rows"]
            ),
            "exact_failures": _delta(four["exact_failures"], compact["exact_failures"]),
            "exact_attempts": _delta(four["exact_attempts"], compact["exact_attempts"]),
            "terminal_residual": _delta(
                four["terminal"]["residual"], compact["terminal"]["residual"]
            ),
            "terminal_iterations": _delta(
                four["terminal"]["iterations"], compact["terminal"]["iterations"]
            ),
            "terminal_best_iteration": _delta(
                four["terminal"]["best_iteration"],
                compact["terminal"]["best_iteration"],
            ),
            "terminal_best_residual": _delta(
                four["terminal"]["best_residual"],
                compact["terminal"]["best_residual"],
            ),
            "mean_contacts": _delta(
                four["contacts"]["mean"], compact["contacts"]["mean"]
            ),
            "mean_unique_pairs": _delta(
                four["unique_pairs"]["mean"], compact["unique_pairs"]["mean"]
            ),
            "pair_identity_union_count": _delta(
                four["pair_graph"]["identity_union_count"],
                compact["pair_graph"]["identity_union_count"],
            ),
            "pair_identity_transition_rows": _delta(
                four["pair_graph"]["identity_transition_rows"],
                compact["pair_graph"]["identity_transition_rows"],
            ),
            "pair_multiplicity_transition_rows": _delta(
                four["pair_graph"]["multiplicity_transition_rows"],
                compact["pair_graph"]["multiplicity_transition_rows"],
            ),
            "mean_pair_multiplicity": _delta(
                four["pair_graph"]["multiplicity"]["mean"],
                compact["pair_graph"]["multiplicity"]["mean"],
            ),
            "mean_warm_start_matched_contacts": _delta(
                four["warm_start"]["matched_contacts_mean"],
                compact["warm_start"]["matched_contacts_mean"],
            ),
            "mean_warm_start_matched_fraction": _delta(
                four["warm_start"]["matched_fraction"]["mean"],
                compact["warm_start"]["matched_fraction"]["mean"],
            ),
            "step_size_persistence_used_rows": _delta(
                four["step_size_persistence_used_rows"],
                compact["step_size_persistence_used_rows"],
            ),
            "mean_primal_residual": _delta(
                four["residual_components"]["primal"]["mean"],
                compact["residual_components"]["primal"]["mean"],
            ),
            "mean_dual_residual": _delta(
                four["residual_components"]["dual"]["mean"],
                compact["residual_components"]["dual"]["mean"],
            ),
            "mean_complementarity_residual": _delta(
                four["residual_components"]["complementarity"]["mean"],
                compact["residual_components"]["complementarity"]["mean"],
            ),
            "maximum_card_displacement": _delta(
                four["card_pose"]["maximum_displacement"],
                compact["card_pose"]["maximum_displacement"],
            ),
            "minimum_card_orientation_alignment": _delta(
                four["card_pose"]["minimum_orientation_alignment"],
                compact["card_pose"]["minimum_orientation_alignment"],
            ),
        },
        "pair_identity_difference": {
            "only_compact": sorted(compact_identities - four_identities),
            "only_four_point_planar": sorted(four_identities - compact_identities),
            "shared": sorted(compact_identities & four_identities),
        },
        "claim_boundary": (
            "Reconstructed-scene Native manifold sensitivity only. Wall time is "
            "diagnostic and excluded from causal, real-time, paper, and performance verdicts."
        ),
    }


def _report(summaries: dict[str, dict[str, Any]], comparison: dict[str, Any]) -> str:
    lines = [
        "# Card-House Native Manifold Sensitivity v1",
        "",
        "Comparison artifact integrity valid: **yes**.",
        "Paper parity: **no**. Timing verdict: **none**.",
        "",
    ]
    for mode in MODES:
        summary = summaries[mode]
        lines.extend(
            [
                f"## {mode}",
                "",
                f"- emitted steps: {summary['emitted_steps']}/600",
                f"- first failed step: {summary['first_failed_step']}",
                f"- strict successes / accepted caps / failures: "
                f"{summary['strict_success_rows']} / {summary['accepted_cap_rows']} / "
                f"{summary['exact_failure_rows']}",
                f"- terminal wrapper/internal status: "
                f"{summary['terminal']['wrapper_status']} / "
                f"{summary['terminal']['internal_fbf_status']}",
                f"- terminal residual: {summary['terminal']['residual']:.17g}",
                f"- contact range: {summary['contacts']['minimum']}.."
                f"{summary['contacts']['maximum']}",
                f"- unique-pair range: {summary['unique_pairs']['minimum']}.."
                f"{summary['unique_pairs']['maximum']}",
                "",
            ]
        )
    lines.extend(
        [
            "The only intended factor is Native contact-manifold mode. Raw wall time is",
            "retained for transparency but is excluded from every scientific verdict.",
            "",
            f"Comparison delta object: `{json.dumps(comparison['deltas'], sort_keys=True)}`",
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
    for mode in MODES:
        (path / mode).mkdir()


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
        "commands": {mode: _command(binary, mode) for mode in MODES},
        "run_order": list(MODES),
        "cwd": str(ROOT),
        "timeout_seconds_per_child": args.timeout,
        "identity_rechecks": [],
        "children": {},
    }
    summaries: dict[str, dict[str, Any]] = {}
    artifact_error: str | None = None
    for mode in MODES:
        stdout, stderr, returncode = _run_captured(
            invocation["commands"][mode], args.timeout
        )
        raw_path = output / mode / "raw.csv"
        stderr_path = output / mode / "stderr.txt"
        raw_path.write_text(stdout, encoding="utf-8")
        stderr_path.write_text(stderr, encoding="utf-8")
        invocation["children"][mode] = {
            "returncode": returncode,
            "stdout_bytes": len(stdout.encode()),
            "stderr_bytes": len(stderr.encode()),
        }
        try:
            if _execution_identity(binary) != identity:
                raise EvidenceError(f"execution identity drifted after {mode}")
            invocation["identity_rechecks"].append(f"after_{mode}")
            _, rows = _parse_trace(stdout)
            summaries[mode] = _mode_summary(mode, rows, returncode)
        except (EvidenceError, OSError, ValueError) as error:
            artifact_error = str(error)
            break

    integrity_valid = artifact_error is None and set(summaries) == set(MODES)
    if integrity_valid:
        comparison = _comparison(summaries)
    else:
        comparison = {
            "comparison_artifact_integrity_valid": False,
            "paper_parity": False,
            "timing_used_in_verdict": False,
            "error": artifact_error or "both modes were not completed",
        }
    summary_document = {
        "schema_version": SCHEMA_VERSION,
        "comparison_artifact_integrity_valid": integrity_valid,
        "modes": summaries,
        "error": artifact_error,
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
            else "# Invalid card-manifold comparison artifact\n\n"
            f"{comparison['error']}\n"
        ),
        encoding="utf-8",
    )

    payload_paths = [
        output / mode / name
        for mode in MODES
        for name in ("raw.csv", "stderr.txt")
        if (output / mode / name).is_file()
    ] + [invocation_path, summary_path, comparison_path, report_path]
    metadata = {
        "schema_version": SCHEMA_VERSION,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "comparison_artifact_integrity_valid": integrity_valid,
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
