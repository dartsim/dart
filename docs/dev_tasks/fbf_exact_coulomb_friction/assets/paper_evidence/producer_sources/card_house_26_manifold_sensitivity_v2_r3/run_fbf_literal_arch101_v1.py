#!/usr/bin/env python3
"""Run the frozen literal 101-stone standing protocol v1.

The runner treats a complete 600-step pass and an expected fail-fast solver
prefix differently. A failed prefix is preserved as a scientific negative, but
it is explicitly marked ``artifact_valid=false`` and is never timing evidence.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import os
import platform
import re
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BINARY = (
    ROOT / "build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace"
)
DEFAULT_COLLISION_PROBE = (
    ROOT / "build/default/cpp/Release/tests/benchmark/integration/"
    "fbf_paper_arch_wedge_collision_probe"
)
PROTOCOL = ROOT / "docs/dev_tasks/fbf_exact_coulomb_friction/LITERAL_ARCH_101_V1.md"
TRACE_SOURCE = ROOT / "tests/benchmark/integration/fbf_paper_trace.cpp"
COLLISION_PROBE_SOURCE = (
    ROOT / "tests/benchmark/integration/fbf_paper_arch_wedge_collision_probe.cpp"
)

SCHEMA_VERSION = "dart.fbf_literal_arch101_v1/v1"
SCENARIO = "masonry_arch_101_literal_wedge"
SCENE_CONTRACT = "reconstructed_literal_wedge_arch_nonpaper_native_collision_frontend"
SOLVER_CONTRACT = "dart_best_nonpaper_colored_inner_bgs_threaded_world"
SCHEDULE_CONTRACT = "dart_deterministic_manifold_colored_bgs_nonpaper"
EXPECTED_STEPS = 600
EXPECTED_CPUS = (8, 10, 12, 14)
EXPECTED_PROTOCOL_CONTRACT_SHA256 = (
    "031140a359fcb1c59f8b7377ce95e31720ec00368f175b70c7daed5fe761ffc3"
)
EXPECTED_HEADER_SHA256 = (
    "424195336cb42753179130c9f6fcba3a8ddea9bf669cfa70c0f77fcb4c6335a5"
)
EXPECTED_HEADER = tuple(
    "step,time,scenario,solver,solver_contract,precision_contract,scene_contract,"
    "baumgarte_contract,collision_frontend,inner_local_solver,"
    "inner_sweeps_requested,fixed_inner_sweeps_requested,"
    "step_size_persistence_enabled,step_size_recovery_growth_factor,"
    "step_size_persistence_used,step_size_persistence_request,"
    "row_operator_request,row_operator_mode,wall_ms,requested_threads,"
    "actual_threads,contacts,unique_colliding_body_pairs,penetration_depth_min,"
    "penetration_depth_median,penetration_depth_p95,penetration_depth_max,"
    "exact_diagnostics_contract,step_exact_solves,step_warm_starts,"
    "step_exact_failures,step_fallbacks,step_fbf_iterations,residual,status,"
    "residual_primal_feasibility,residual_dual_feasibility,"
    "residual_complementarity,accepted_gamma,safe_gamma,shrink_iterations,"
    "coupling_variation_ratio,warm_start_matched_contacts,"
    "warm_start_matched_fraction,phase,card_count,projectile_count,finite_state,"
    "min_card_axis_up,min_center_height,max_card_horizontal_travel,"
    "max_projectile_speed,tracked_body,x,y,z,vx,vy,vz,up_z,"
    "max_outer_iterations,tolerance,accept_outer_max_iterations,"
    "inner_local_iterations,adaptive_step_size_enabled,warm_start_enabled,"
    "projected_gradient_retry_enabled,dense_residual_polish_enabled,"
    "fallback_to_boxed_lcp_enabled,diagonal_seed_enabled,"
    "matrix_free_seed_enabled,step_size_scale,outer_relaxation,"
    "initial_gamma_contract,split_impulse_enabled,"
    "step_contact_row_delassus_products,"
    "step_parallel_contact_row_delassus_products,"
    "max_contact_row_participants_to_date,"
    "exact_contact_row_logical_cpus_to_date,"
    "max_phase_contact_row_logical_cpus_to_date,"
    "max_card_center_displacement_from_initial,"
    "min_card_orientation_alignment_from_initial,projectile_card_contacts,"
    "inner_bgs_schedule_contract,last_exact_colored_bgs_used,"
    "last_exact_colored_bgs_solves,last_exact_colored_bgs_dispatches,"
    "last_exact_colored_bgs_max_participants,last_exact_colored_bgs_manifolds,"
    "last_exact_colored_bgs_colors,"
    "last_exact_colored_bgs_max_manifolds_per_color,"
    "exact_colored_bgs_logical_cpus,max_phase_exact_colored_bgs_logical_cpus,"
    "max_arch_body_displacement_from_initial,"
    "min_arch_body_orientation_alignment_from_initial".split(",")
)

FINITE_FLOAT_FIELDS = (
    "time",
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
    "max_card_horizontal_travel",
    "max_projectile_speed",
    "x",
    "y",
    "z",
    "vx",
    "vy",
    "vz",
    "up_z",
    "tolerance",
    "step_size_recovery_growth_factor",
    "step_size_scale",
    "outer_relaxation",
    "max_arch_body_displacement_from_initial",
    "min_arch_body_orientation_alignment_from_initial",
)

INTEGER_FIELDS = (
    "step",
    "inner_sweeps_requested",
    "fixed_inner_sweeps_requested",
    "step_size_persistence_enabled",
    "step_size_persistence_used",
    "requested_threads",
    "actual_threads",
    "contacts",
    "unique_colliding_body_pairs",
    "step_exact_solves",
    "step_warm_starts",
    "step_exact_failures",
    "step_fallbacks",
    "step_fbf_iterations",
    "shrink_iterations",
    "warm_start_matched_contacts",
    "card_count",
    "projectile_count",
    "finite_state",
    "max_outer_iterations",
    "accept_outer_max_iterations",
    "inner_local_iterations",
    "adaptive_step_size_enabled",
    "warm_start_enabled",
    "projected_gradient_retry_enabled",
    "dense_residual_polish_enabled",
    "fallback_to_boxed_lcp_enabled",
    "diagonal_seed_enabled",
    "matrix_free_seed_enabled",
    "split_impulse_enabled",
    "step_contact_row_delassus_products",
    "step_parallel_contact_row_delassus_products",
    "max_contact_row_participants_to_date",
    "projectile_card_contacts",
    "last_exact_colored_bgs_used",
    "last_exact_colored_bgs_solves",
    "last_exact_colored_bgs_dispatches",
    "last_exact_colored_bgs_max_participants",
    "last_exact_colored_bgs_manifolds",
    "last_exact_colored_bgs_colors",
    "last_exact_colored_bgs_max_manifolds_per_color",
)

FINGERPRINT_FIELDS = (
    "scenario",
    "solver",
    "solver_contract",
    "precision_contract",
    "scene_contract",
    "baumgarte_contract",
    "collision_frontend",
    "inner_local_solver",
    "inner_sweeps_requested",
    "fixed_inner_sweeps_requested",
    "step_size_persistence_enabled",
    "step_size_recovery_growth_factor",
    "row_operator_request",
    "row_operator_mode",
    "requested_threads",
    "actual_threads",
    "contacts",
    "unique_colliding_body_pairs",
    "phase",
    "card_count",
    "projectile_count",
    "tracked_body",
    "max_outer_iterations",
    "tolerance",
    "accept_outer_max_iterations",
    "inner_local_iterations",
    "adaptive_step_size_enabled",
    "warm_start_enabled",
    "projected_gradient_retry_enabled",
    "dense_residual_polish_enabled",
    "fallback_to_boxed_lcp_enabled",
    "diagonal_seed_enabled",
    "matrix_free_seed_enabled",
    "step_size_scale",
    "outer_relaxation",
    "initial_gamma_contract",
    "split_impulse_enabled",
    "inner_bgs_schedule_contract",
    "last_exact_colored_bgs_max_participants",
    "last_exact_colored_bgs_manifolds",
    "last_exact_colored_bgs_colors",
    "last_exact_colored_bgs_max_manifolds_per_color",
    "exact_colored_bgs_logical_cpus",
    "max_phase_exact_colored_bgs_logical_cpus",
)


class EvidenceError(RuntimeError):
    """Raised when output cannot be accepted as frozen protocol evidence."""


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_json(path: Path, payload: Any) -> None:
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
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


def _build_command(binary: Path) -> list[str]:
    return [
        "taskset",
        "--cpu-list",
        ",".join(str(cpu) for cpu in EXPECTED_CPUS),
        str(binary),
        SCENARIO,
        "exact_fbf",
        "1",
        str(EXPECTED_STEPS),
        "nan",
        "performance",
        "default",
        "default",
        "4",
        "dart_best_colored_bgs",
        "native",
        "default",
        "0",
        "0",
    ]


def _build_collision_probe_command(probe: Path) -> list[str]:
    return [str(probe), "2"]


def _parse_probe_record(line: str) -> tuple[str, dict[str, str]]:
    parts = line.split(",")
    if not parts or parts[0] not in {"metadata", "sample", "pair", "verdict"}:
        raise EvidenceError("collision probe emitted an unknown record type")
    values: dict[str, str] = {}
    for item in parts[1:]:
        key, separator, value = item.partition("=")
        if not separator or not key or key in values:
            raise EvidenceError("collision probe emitted a malformed record")
        values[key] = value
    return parts[0], values


def _probe_int(values: dict[str, str], field: str) -> int:
    try:
        return int(values[field])
    except (KeyError, ValueError) as error:
        raise EvidenceError(f"invalid collision probe integer {field!r}") from error


def _probe_float(values: dict[str, str], field: str) -> float:
    try:
        value = float(values[field])
    except (KeyError, ValueError) as error:
        raise EvidenceError(f"invalid collision probe float {field!r}") from error
    if not math.isfinite(value):
        raise EvidenceError(f"non-finite collision probe field {field!r}")
    return value


def _validate_collision_probe(stdout: str, returncode: int) -> dict[str, Any]:
    if returncode != 0:
        raise EvidenceError(f"collision probe returned {returncode}, expected zero")
    target: dict[str, list[dict[str, str]]] = {
        "metadata": [],
        "sample": [],
        "pair": [],
        "verdict": [],
    }
    for line in stdout.splitlines():
        kind, values = _parse_probe_record(line)
        if (
            values.get("stone_count") == "101"
            and values.get("backend") == "native"
            and values.get("policy") == "closure_1um"
            and values.get("gap_policy") == "omitted_offsets"
        ):
            target[kind].append(values)

    if len(target["metadata"]) != 1 or len(target["sample"]) != 2:
        raise EvidenceError("collision probe target metadata/sample is not unique")
    if len(target["verdict"]) != 1:
        raise EvidenceError("collision probe target verdict is not unique")

    identity_fields = {"stone_count", "backend", "policy", "gap_policy"}
    metadata_fields = identity_fields | {
        "end_face_expansion_m",
        "downward_shift_m",
        "friction",
        "pinned_springers",
        "pinned_springers_valid",
        "mobile_skeletons",
        "collision_only",
        "dynamic_claim",
        "exact_polyhedral_inertia",
        "source_density_kg_m3",
        "exact_volume_m3",
        "nominal_mass_from_exact_volume_kg",
        "convex_shape_aabb_volume_m3",
    }
    sample_fields = identity_fields | {
        "repeat",
        "contacts",
        "unique_pairs",
        "adjacent_stone_pairs",
        "nonadjacent_stone_pairs",
        "springer_ground_pairs",
        "unexpected_ground_pairs",
        "min_penetration_m",
        "max_penetration_m",
        "mean_penetration_m",
        "nonfinite_contacts",
    }
    pair_fields = identity_fields | {"first", "second", "kind", "contacts"}
    verdict_fields = identity_fields | {
        "repeated_collision_stable",
        "numerical_100_contact_target_observed",
        "genuine_contact_graph",
        "dynamic_path_candidate",
        "paper_contract_proven",
    }
    metadata = target["metadata"][0]
    samples = sorted(target["sample"], key=lambda values: _probe_int(values, "repeat"))
    verdict = target["verdict"][0]
    if set(metadata) != metadata_fields or any(
        set(sample) != sample_fields for sample in samples
    ):
        raise EvidenceError("collision probe target metadata/sample schema drifted")
    if set(verdict) != verdict_fields:
        raise EvidenceError("collision probe target verdict schema drifted")
    if any(set(pair) != pair_fields for pair in target["pair"]):
        raise EvidenceError("collision probe target pair schema drifted")

    expected_metadata = {
        "pinned_springers": "0:100",
        "pinned_springers_valid": "true",
        "collision_only": "true",
        "dynamic_claim": "false",
        "exact_polyhedral_inertia": "true",
    }
    for field, expected in expected_metadata.items():
        if metadata[field] != expected:
            raise EvidenceError(f"collision probe metadata {field!r} drifted")
    if _probe_int(metadata, "mobile_skeletons") != 99:
        raise EvidenceError("collision probe mobile skeleton count drifted")
    for field, expected in (
        ("end_face_expansion_m", 1e-6),
        ("downward_shift_m", 0.001001),
        ("friction", 0.8),
        ("source_density_kg_m3", 1000.0),
    ):
        if not math.isclose(
            _probe_float(metadata, field), expected, rel_tol=1e-12, abs_tol=1e-15
        ):
            raise EvidenceError(f"collision probe metadata {field!r} drifted")
    for field in (
        "exact_volume_m3",
        "nominal_mass_from_exact_volume_kg",
        "convex_shape_aabb_volume_m3",
    ):
        if _probe_float(metadata, field) <= 0.0:
            raise EvidenceError(f"collision probe metadata {field!r} is not positive")

    expected_sample = {
        "contacts": 102,
        "unique_pairs": 102,
        "adjacent_stone_pairs": 100,
        "nonadjacent_stone_pairs": 0,
        "springer_ground_pairs": 2,
        "unexpected_ground_pairs": 0,
        "nonfinite_contacts": 0,
    }
    penetration_summaries: list[tuple[float, float, float]] = []
    for repeat, sample in enumerate(samples):
        if _probe_int(sample, "repeat") != repeat:
            raise EvidenceError("collision probe repeats are not exactly 0 and 1")
        for field, expected in expected_sample.items():
            if _probe_int(sample, field) != expected:
                raise EvidenceError(f"collision probe sample {field!r} drifted")
        minimum = _probe_float(sample, "min_penetration_m")
        maximum = _probe_float(sample, "max_penetration_m")
        mean = _probe_float(sample, "mean_penetration_m")
        if minimum < 0.0 or not minimum <= mean <= maximum:
            raise EvidenceError("collision probe penetration summary is inconsistent")
        penetration_summaries.append((minimum, maximum, mean))
    if any(
        not math.isclose(first, second, rel_tol=0.0, abs_tol=1e-12)
        for first, second in zip(
            penetration_summaries[0], penetration_summaries[1], strict=True
        )
    ):
        raise EvidenceError("collision probe repeated penetration summary drifted")

    adjacent_edges: set[tuple[int, int]] = set()
    ground_indices: set[int] = set()
    stone_pattern = re.compile(r"^masonry_arch_wedge_([0-9]+)_shape$")
    ground_name = "masonry_arch_ground_shape"
    if len(target["pair"]) != 102:
        raise EvidenceError("collision probe target pair record count drifted")
    for pair in target["pair"]:
        if _probe_int(pair, "contacts") != 1:
            raise EvidenceError("collision probe compact pair contact count drifted")
        names = (pair["first"], pair["second"])
        matches = [stone_pattern.fullmatch(name) for name in names]
        if all(matches):
            indices = sorted(int(match.group(1)) for match in matches if match)
            if pair["kind"] != "adjacent_stones" or indices[1] - indices[0] != 1:
                raise EvidenceError("collision probe emitted a non-adjacent stone pair")
            adjacent_edges.add((indices[0], indices[1]))
        elif ground_name in names and sum(match is not None for match in matches) == 1:
            match = next(match for match in matches if match is not None)
            index = int(match.group(1))
            if pair["kind"] != "springer_ground" or index not in {0, 100}:
                raise EvidenceError("collision probe emitted an unexpected ground pair")
            ground_indices.add(index)
        else:
            raise EvidenceError("collision probe emitted an unknown target pair")
    if adjacent_edges != {(index, index + 1) for index in range(100)}:
        raise EvidenceError("collision probe adjacent-pair identity set drifted")
    if ground_indices != {0, 100}:
        raise EvidenceError("collision probe springer-ground identity set drifted")

    expected_verdict = {
        "repeated_collision_stable": "true",
        "numerical_100_contact_target_observed": "not_applicable",
        "genuine_contact_graph": "true",
        "dynamic_path_candidate": "not_applicable",
        "paper_contract_proven": "false",
    }
    for field, expected in expected_verdict.items():
        if verdict[field] != expected:
            raise EvidenceError(f"collision probe verdict {field!r} drifted")

    return {
        "returncode": returncode,
        "repeat_count": 2,
        "constructed_initial_scene_passed": True,
        "contacts": 102,
        "unique_pairs": 102,
        "adjacent_stone_pairs": 100,
        "springer_ground_pairs": 2,
        "nonadjacent_stone_pairs": 0,
        "unexpected_ground_pairs": 0,
        "nonfinite_contacts": 0,
        "repeated_collision_stable": True,
        "genuine_contact_graph": True,
        "scope": "constructed_initial_scene_collision_only_compact_manifold",
        "dynamic_pair_identity_evidence": False,
        "promotion_boundary": (
            "The probe proves pair identities only for the constructed initial "
            "scene. Positive long-run promotion still requires dynamic "
            "pair-identity evidence for the FourPointPlanar trace."
        ),
    }


def _parse_trace(stdout: str) -> tuple[list[str], list[dict[str, str]]]:
    lines = stdout.splitlines()
    if not lines:
        raise EvidenceError("trace emitted no CSV")
    if _sha256_bytes((lines[0] + "\n").encode()) != EXPECTED_HEADER_SHA256:
        raise EvidenceError("trace header hash mismatch")
    reader = csv.DictReader(io.StringIO(stdout))
    header = list(reader.fieldnames or ())
    if tuple(header) != EXPECTED_HEADER or len(set(header)) != 95:
        raise EvidenceError("trace must use the frozen 95-column schema")
    rows = list(reader)
    if any(None in row for row in rows):
        raise EvidenceError("trace row has extra columns")
    if any(any(value is None for value in row.values()) for row in rows):
        raise EvidenceError("trace row has missing columns")
    return header, rows


def _int(row: dict[str, str], field: str) -> int:
    try:
        return int(row[field])
    except (KeyError, ValueError) as error:
        raise EvidenceError(f"invalid integer field {field!r}") from error


def _float(row: dict[str, str], field: str) -> float:
    try:
        return float(row[field])
    except (KeyError, ValueError) as error:
        raise EvidenceError(f"invalid floating-point field {field!r}") from error


def _require_close(row: dict[str, str], field: str, expected: float) -> None:
    value = _float(row, field)
    if not math.isclose(value, expected, rel_tol=1e-12, abs_tol=0.0):
        raise EvidenceError(f"frozen field {field!r} drifted")


def _cpu_set(value: str, field: str) -> set[int]:
    parts = value.split(";")
    try:
        cpus = {int(part) for part in parts}
    except ValueError as error:
        raise EvidenceError(f"invalid CPU residency field {field!r}") from error
    if len(parts) != len(cpus):
        raise EvidenceError(f"duplicate CPU in residency field {field!r}")
    return cpus


def _validate_common_row(row: dict[str, str], expected_step: int) -> None:
    for field in INTEGER_FIELDS:
        _int(row, field)
    for field in FINITE_FLOAT_FIELDS:
        if not math.isfinite(_float(row, field)):
            raise EvidenceError(f"non-finite required field {field!r}")

    if _int(row, "step") != expected_step:
        raise EvidenceError("trace steps are not ordered and contiguous")
    if not math.isclose(
        _float(row, "time"), expected_step / 60.0, rel_tol=1e-12, abs_tol=1e-12
    ):
        raise EvidenceError("trace time does not match 60 Hz stepping")

    expected_strings = {
        "scenario": SCENARIO,
        "solver": "exact_fbf",
        "solver_contract": SOLVER_CONTRACT,
        "precision_contract": "float64",
        "scene_contract": SCENE_CONTRACT,
        "baumgarte_contract": ("split_impulse_no_velocity_baumgarte_erp_zero_vs_paper"),
        "collision_frontend": "native",
        "inner_local_solver": "exact_metric",
        "row_operator_request": "contact_row_no_dense_snapshot",
        "row_operator_mode": "contact_row_no_dense_snapshot",
        "exact_diagnostics_contract": (
            "last_exact_group_public_getters_contact_row_no_dense_snapshot_"
            "warm_fraction_over_step_contacts"
        ),
        "phase": "single_phase",
        "tracked_body": "masonry_arch_stone_50_body",
        "initial_gamma_contract": "automatic_safe_bound",
        "inner_bgs_schedule_contract": SCHEDULE_CONTRACT,
        "exact_contact_row_logical_cpus_to_date": "none",
        "max_phase_contact_row_logical_cpus_to_date": "none",
    }
    for field, expected in expected_strings.items():
        if row.get(field) != expected:
            raise EvidenceError(f"frozen field {field!r} drifted")

    expected_integers = {
        "inner_sweeps_requested": 30,
        "fixed_inner_sweeps_requested": 1,
        "step_size_persistence_enabled": 0,
        "step_size_persistence_used": 0,
        "requested_threads": 4,
        "actual_threads": 4,
        "contacts": 400,
        "unique_colliding_body_pairs": 100,
        "card_count": 0,
        "projectile_count": 0,
        "finite_state": 1,
        "max_outer_iterations": 5000,
        "accept_outer_max_iterations": 0,
        "inner_local_iterations": 1,
        "adaptive_step_size_enabled": 1,
        "warm_start_enabled": 1,
        "projected_gradient_retry_enabled": 0,
        "dense_residual_polish_enabled": 0,
        "fallback_to_boxed_lcp_enabled": 0,
        "diagonal_seed_enabled": 0,
        "matrix_free_seed_enabled": 0,
        "split_impulse_enabled": 1,
        "projectile_card_contacts": 0,
        "last_exact_colored_bgs_used": 1,
        "last_exact_colored_bgs_dispatches": 1,
        "last_exact_colored_bgs_max_participants": 4,
        "last_exact_colored_bgs_manifolds": 100,
        "last_exact_colored_bgs_colors": 3,
        "last_exact_colored_bgs_max_manifolds_per_color": 34,
    }
    for field, expected in expected_integers.items():
        if _int(row, field) != expected:
            raise EvidenceError(f"frozen field {field!r} drifted")

    _require_close(row, "step_size_recovery_growth_factor", 1.05)
    _require_close(row, "tolerance", 1e-6)
    _require_close(row, "step_size_scale", 35.0)
    _require_close(row, "outer_relaxation", 1.1)
    if row.get("step_size_persistence_request") != "nan":
        raise EvidenceError("disabled gamma persistence emitted a request")
    for field in (
        "min_card_axis_up",
        "min_center_height",
        "max_card_center_displacement_from_initial",
        "min_card_orientation_alignment_from_initial",
    ):
        if row.get(field) != "nan":
            raise EvidenceError(f"non-card sentinel field {field!r} drifted")

    expected_cpus = set(EXPECTED_CPUS)
    for field in (
        "exact_colored_bgs_logical_cpus",
        "max_phase_exact_colored_bgs_logical_cpus",
    ):
        if _cpu_set(row[field], field) != expected_cpus:
            raise EvidenceError(f"colored CPU residency field {field!r} drifted")
    if _int(row, "last_exact_colored_bgs_solves") <= 0:
        raise EvidenceError("colored BGS reported no inner solve work")
    if _int(row, "step_contact_row_delassus_products") <= 0:
        raise EvidenceError("contact-row operator reported no work")
    if _int(row, "step_parallel_contact_row_delassus_products") != 0:
        raise EvidenceError("colored solve unexpectedly used parallel row products")
    if _int(row, "max_contact_row_participants_to_date") != 1:
        raise EvidenceError("colored solve contact-row participant count drifted")


def _work_fingerprint(rows: Sequence[dict[str, str]]) -> str:
    if not rows:
        raise EvidenceError("trace contains no rows")
    expected = tuple(rows[0][field] for field in FINGERPRINT_FIELDS)
    for row in rows[1:]:
        if tuple(row[field] for field in FINGERPRINT_FIELDS) != expected:
            raise EvidenceError("normalized scene/work fingerprint drifted")
    canonical = json.dumps(
        list(zip(FINGERPRINT_FIELDS, expected, strict=True)),
        separators=(",", ":"),
        ensure_ascii=True,
    )
    return _sha256_bytes(canonical.encode())


def _row_gate_failures(row: dict[str, str]) -> list[str]:
    failures: list[str] = []
    solves = _int(row, "step_exact_solves")
    exact_failures = _int(row, "step_exact_failures")
    if solves + exact_failures != 1:
        failures.append("one_exact_attempt")
    if solves != 1:
        failures.append("one_exact_solve")
    if row.get("status") != "success":
        failures.append("success_status")
    if _float(row, "residual") > 1e-6:
        failures.append("residual_at_most_1e-6")
    if exact_failures != 0:
        failures.append("zero_exact_failures")
    if _int(row, "step_fallbacks") != 0:
        failures.append("zero_boxed_fallbacks")
    if _float(row, "max_arch_body_displacement_from_initial") > 0.05:
        failures.append("maximum_displacement_at_most_0.05")
    if _float(row, "min_arch_body_orientation_alignment_from_initial") < 0.80:
        failures.append("minimum_orientation_at_least_0.80")
    return failures


def _validate_trace(
    *, header: Sequence[str], rows: Sequence[dict[str, str]], returncode: int
) -> dict[str, Any]:
    if tuple(header) != EXPECTED_HEADER:
        raise EvidenceError("trace header is not the frozen schema")
    if returncode not in (0, 1):
        raise EvidenceError(f"unexpected child return code {returncode}")
    if not rows or len(rows) > EXPECTED_STEPS:
        raise EvidenceError("trace row count is outside 1..600")
    for expected_step, row in enumerate(rows, start=1):
        _validate_common_row(row, expected_step)
    fingerprint = _work_fingerprint(rows)
    gate_failures = [_row_gate_failures(row) for row in rows]

    base = {
        "schema_version": SCHEMA_VERSION,
        "scenario": SCENARIO,
        "requested_steps": EXPECTED_STEPS,
        "emitted_steps": len(rows),
        "child_returncode": returncode,
        "trace_columns": len(header),
        "trace_header_sha256": EXPECTED_HEADER_SHA256,
        "protocol_contract_sha256": EXPECTED_PROTOCOL_CONTRACT_SHA256,
        "normalized_scene_work_fingerprint_sha256": fingerprint,
        "timing_evidence_eligible": False,
        "timing_statistics": None,
    }
    if returncode == 0:
        if len(rows) != EXPECTED_STEPS:
            raise EvidenceError("zero-return trace did not emit all 600 rows")
        if any(gate_failures):
            raise EvidenceError("zero-return trace falsifies a protocol gate")
        return {
            **base,
            "classification": "valid_positive_standing_evidence",
            "artifact_valid": True,
            "standing_claim_passed": True,
            "first_failed_step": None,
            "failed_gates": [],
            "solver": {
                "maximum_residual": max(_float(row, "residual") for row in rows),
                "exact_failures": 0,
                "boxed_fallbacks": 0,
            },
            "preservation": {
                "maximum_displacement": max(
                    _float(row, "max_arch_body_displacement_from_initial")
                    for row in rows
                ),
                "minimum_orientation_alignment": min(
                    _float(row, "min_arch_body_orientation_alignment_from_initial")
                    for row in rows
                ),
            },
        }

    failing_steps = [
        index for index, failures in enumerate(gate_failures, 1) if failures
    ]
    if not failing_steps:
        raise EvidenceError("fail-fast return has no independently failed gate")
    if failing_steps != [len(rows)]:
        raise EvidenceError("fail-fast trace continued past its first failed step")
    terminal = rows[-1]
    if (
        terminal.get("status") != "fbf_failed"
        or _int(terminal, "step_exact_failures") != 1
        or _int(terminal, "step_exact_solves") != 0
    ):
        raise EvidenceError("return 1 is not the expected exact-FBF fail-fast path")
    return {
        **base,
        "classification": "failed_prefix_scientific_negative",
        "artifact_valid": False,
        "standing_claim_passed": False,
        "first_failed_step": len(rows),
        "failed_gates": gate_failures[-1],
        "terminal_status": terminal["status"],
        "terminal_residual": _float(terminal, "residual"),
        "claim_boundary": (
            "Failed prefix retained as a scientific negative. It is not a valid "
            "standing artifact and must never be used as timing evidence."
        ),
    }


def _read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8").strip()


def _affinity_metadata() -> dict[str, Any]:
    topology: dict[str, dict[str, Any]] = {}
    physical_keys: set[tuple[str, str]] = set()
    all_max_frequencies = [
        int(_read_text(path))
        for path in Path("/sys/devices/system/cpu").glob(
            "cpu[0-9]*/cpufreq/cpuinfo_max_freq"
        )
    ]
    if not all_max_frequencies:
        raise EvidenceError("CPU maximum-frequency topology is unavailable")
    minimum_class_frequency = min(all_max_frequencies)
    for cpu in EXPECTED_CPUS:
        base = Path(f"/sys/devices/system/cpu/cpu{cpu}")
        package = _read_text(base / "topology/physical_package_id")
        core = _read_text(base / "topology/core_id")
        siblings = _read_text(base / "topology/thread_siblings_list")
        max_frequency = int(_read_text(base / "cpufreq/cpuinfo_max_freq"))
        governor = _read_text(base / "cpufreq/scaling_governor")
        physical_keys.add((package, core))
        topology[str(cpu)] = {
            "physical_package_id": package,
            "core_id": core,
            "thread_siblings_list": siblings,
            "cpuinfo_max_freq_khz": max_frequency,
            "scaling_governor": governor,
            "declared_p_core": max_frequency > minimum_class_frequency,
        }
    if len(physical_keys) != len(EXPECTED_CPUS):
        raise EvidenceError("selected CPUs do not map one-to-one to physical cores")
    if not all(record["declared_p_core"] for record in topology.values()):
        raise EvidenceError("selected CPUs are not in the higher-frequency core class")
    return {
        "source": "explicit_taskset",
        "logical_cpus": list(EXPECTED_CPUS),
        "one_logical_per_physical_core": True,
        "physical_core_keys": [list(key) for key in sorted(physical_keys)],
        "topology": topology,
    }


def _tool_identity(name: str) -> dict[str, Any]:
    path_text = shutil.which(name)
    if path_text is None:
        raise EvidenceError(f"required tool is unavailable: {name}")
    path = Path(path_text)
    resolved = path.resolve()
    if not resolved.is_file():
        raise EvidenceError(f"resolved {name} tool is not a regular file")
    return {
        "name": name,
        "path": str(path),
        "resolved_path": str(resolved),
        "size_bytes": resolved.stat().st_size,
        "sha256": _sha256_file(resolved),
    }


def _executable_identity(binary: Path) -> dict[str, Any]:
    binary = binary.resolve()
    if not binary.is_file():
        raise EvidenceError(f"executable not found: {binary}")
    ldd_tool = _tool_identity("ldd")
    result = subprocess.run(
        [ldd_tool["path"], str(binary)],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        raise EvidenceError(f"ldd failed for {binary}: {result.stderr.strip()}")

    libraries: list[dict[str, Any]] = []
    for raw_line in result.stdout.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if "=> not found" in line:
            raise EvidenceError(f"unresolved shared library: {line}")
        if "=>" in line:
            soname, remainder = (part.strip() for part in line.split("=>", 1))
            path_text = remainder.rsplit(" (", 1)[0].strip()
        elif line.startswith("/"):
            path_text = line.rsplit(" (", 1)[0].strip()
            soname = Path(path_text).name
        else:
            # linux-vdso is a kernel-provided virtual object, not a resolved
            # regular shared-library file that can be hashed.
            continue
        reported_path = Path(path_text)
        if not reported_path.is_absolute():
            raise EvidenceError(f"ldd emitted a non-absolute library path: {line}")
        try:
            resolved_path = reported_path.resolve(strict=True)
        except OSError as error:
            raise EvidenceError(
                f"ldd library path cannot be resolved: {line}"
            ) from error
        if not resolved_path.is_file():
            raise EvidenceError(f"ldd library is not a regular file: {resolved_path}")
        libraries.append(
            {
                "soname": soname,
                "reported_path": str(reported_path),
                "resolved_path": str(resolved_path),
                "size_bytes": resolved_path.stat().st_size,
                "sha256": _sha256_file(resolved_path),
            }
        )
    libraries.sort(key=lambda item: (item["soname"], item["resolved_path"]))
    if not libraries:
        raise EvidenceError(f"ldd resolved no regular shared libraries for {binary}")

    build_root = (ROOT / "build").resolve()
    build_libdart = [
        library
        for library in libraries
        if library["soname"].startswith("libdart.so")
        and Path(library["resolved_path"]).is_relative_to(build_root)
    ]
    if len(build_libdart) != 1:
        raise EvidenceError("ldd did not resolve exactly one build-tree libdart")
    normalized_resolution = (
        "\n".join(
            f"{library['soname']} => {library['reported_path']} => "
            f"{library['resolved_path']}"
            for library in libraries
        )
        + "\n"
    )
    return {
        "path": str(binary),
        "size_bytes": binary.stat().st_size,
        "sha256": _sha256_file(binary),
        "ldd_tool": ldd_tool,
        "ldd_resolution_output_normalized": normalized_resolution,
        "ldd_resolution_output_normalized_sha256": _sha256_bytes(
            normalized_resolution.encode()
        ),
        "ldd_stderr_sha256": _sha256_bytes(result.stderr.encode()),
        "resolved_regular_shared_libraries": libraries,
        "resolved_regular_shared_library_count": len(libraries),
        "resolved_build_libdart": build_libdart[0],
    }


def _execution_identity(binary: Path, collision_probe: Path) -> dict[str, Any]:
    return {
        "protocol_contract_sha256": _protocol_contract_sha256(),
        "runner_source_sha256": _sha256_file(Path(__file__).resolve()),
        "trace_source_sha256": _sha256_file(TRACE_SOURCE),
        "trace_executable": _executable_identity(binary),
        "collision_probe_source_sha256": _sha256_file(COLLISION_PROBE_SOURCE),
        "collision_probe_executable": _executable_identity(collision_probe),
        "taskset_tool": _tool_identity("taskset"),
    }


def _report(summary: dict[str, Any]) -> str:
    if summary["classification"] == "valid_positive_standing_evidence":
        return f"""# Literal 101-Stone Standing Protocol v1

Classification: **valid positive standing evidence**.

All {summary['emitted_steps']} requested steps passed the independently checked
solver, graph, colored-execution, displacement, orientation, and finite-state
gates. The normalized scene/work fingerprint is
`{summary['normalized_scene_work_fingerprint_sha256']}`.

The collision-only Compact probe proves the constructed time-zero graph:
102 pairs comprise 100 adjacent stone pairs plus two pinned-springer ground
pairs. The dynamic FourPointPlanar trace exposes the 400-contact, 100-constraint
pair, three-color, width-34 aggregate graph beginning at step 1, but not pair
identities. Positive long-run promotion still requires dynamic pair-identity
evidence.

This is local evidence for the frozen DART reconstruction only. Timings are
diagnostic and are not paper-comparison evidence.
"""
    if summary["classification"] == "failed_prefix_scientific_negative":
        return f"""# Literal 101-Stone Standing Protocol v1 Scientific Negative

Classification: **failed-prefix scientific negative**. Artifact valid: **no**.
Standing claim passed: **no**.

The exact-FBF child failed closed at step {summary['first_failed_step']} with
status `{summary['terminal_status']}` and residual
`{summary['terminal_residual']:.17g}`. Failed gates:
{', '.join(summary['failed_gates'])}.

The raw prefix and stderr are retained for diagnosis. This prefix is not a
valid standing artifact and must never be used as timing evidence.

The collision-only Compact probe covers only constructed time zero. The dynamic
FourPointPlanar trace reports aggregate counts and coloring beginning at step 1,
not pair identities.
"""
    return f"""# Literal 101-Stone Standing Protocol v1 Invalid Artifact

Classification: **invalid artifact**. Artifact valid: **no**. Standing claim
passed: **no**.

Validation error: {summary['error']}

The retained bytes must not be used for standing, timing, or paper claims.
"""


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=DEFAULT_BINARY)
    parser.add_argument("--collision-probe", type=Path, default=DEFAULT_COLLISION_PROBE)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=3600.0)
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        parser.error("--timeout must be positive and finite")
    return args


def _prepare_output(output: Path) -> None:
    if output.exists():
        raise EvidenceError(f"output directory must be fresh: {output}")
    output.mkdir(parents=True)


def _captured_text(value: str | bytes | None) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return value


def _run_captured(command: Sequence[str], timeout: float) -> tuple[str, str, int]:
    try:
        result = subprocess.run(
            list(command),
            cwd=ROOT,
            text=True,
            capture_output=True,
            check=False,
            timeout=timeout,
        )
        return result.stdout, result.stderr, result.returncode
    except subprocess.TimeoutExpired as error:
        return _captured_text(error.stdout), _captured_text(error.stderr), 124


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    binary = args.binary.resolve()
    collision_probe = args.collision_probe.resolve()
    output = args.output_dir.resolve()
    if not binary.is_file():
        raise SystemExit(f"trace binary not found: {binary}")
    if not collision_probe.is_file():
        raise SystemExit(f"collision probe binary not found: {collision_probe}")
    try:
        _prepare_output(output)
        affinity = _affinity_metadata()
        identity = _execution_identity(binary, collision_probe)
    except (EvidenceError, OSError, ValueError) as error:
        raise SystemExit(str(error)) from error

    command = _build_command(binary)
    collision_probe_command = _build_collision_probe_command(collision_probe)
    invocation: dict[str, Any] = {
        "command": command,
        "collision_probe_command": collision_probe_command,
        "cwd": str(ROOT),
        "affinity": affinity,
        "timeout_seconds": args.timeout,
        "identity_rechecks": [],
    }
    stdout = ""
    stderr = ""
    returncode: int | None = None
    probe_stdout = ""
    probe_stderr = ""
    probe_returncode: int | None = None
    probe_validation: dict[str, Any] | None = None
    try:
        probe_stdout, probe_stderr, probe_returncode = _run_captured(
            collision_probe_command, args.timeout
        )
        if _execution_identity(binary, collision_probe) != identity:
            raise EvidenceError(
                "full executable, source, library, protocol, or runner identity "
                "drifted after collision probe"
            )
        invocation["identity_rechecks"].append("after_collision_probe")
        probe_validation = _validate_collision_probe(probe_stdout, probe_returncode)
        stdout, stderr, returncode = _run_captured(command, args.timeout)
        if _execution_identity(binary, collision_probe) != identity:
            raise EvidenceError(
                "full executable, source, library, protocol, or runner identity "
                "drifted after trace"
            )
        invocation["identity_rechecks"].append("after_trace")
        header, rows = _parse_trace(stdout)
        summary = _validate_trace(header=header, rows=rows, returncode=returncode)
        summary["collision_probe"] = probe_validation
        summary["dynamic_pair_identity_evidence"] = False
        summary["positive_long_run_promotion_eligible"] = False
    except (EvidenceError, OSError, ValueError) as error:
        summary = {
            "schema_version": SCHEMA_VERSION,
            "classification": "invalid_artifact",
            "artifact_valid": False,
            "standing_claim_passed": False,
            "timing_evidence_eligible": False,
            "timing_statistics": None,
            "child_returncode": returncode,
            "collision_probe_returncode": probe_returncode,
            "collision_probe": probe_validation,
            "dynamic_pair_identity_evidence": False,
            "positive_long_run_promotion_eligible": False,
            "error": str(error),
        }

    raw_path = output / "raw.csv"
    stderr_path = output / "stderr.txt"
    probe_stdout_path = output / "collision_probe_stdout.txt"
    probe_stderr_path = output / "collision_probe_stderr.txt"
    raw_path.write_text(stdout, encoding="utf-8")
    stderr_path.write_text(stderr, encoding="utf-8")
    probe_stdout_path.write_text(probe_stdout, encoding="utf-8")
    probe_stderr_path.write_text(probe_stderr, encoding="utf-8")
    invocation.update(
        {
            "child_returncode": returncode,
            "collision_probe_returncode": probe_returncode,
            "raw_stdout_bytes": len(stdout.encode()),
            "stderr_bytes": len(stderr.encode()),
            "collision_probe_stdout_bytes": len(probe_stdout.encode()),
            "collision_probe_stderr_bytes": len(probe_stderr.encode()),
        }
    )

    invocation_path = output / "invocation.json"
    summary_path = output / "summary.json"
    report_path = output / "REPORT.md"
    _write_json(invocation_path, invocation)
    _write_json(summary_path, summary)
    report_path.write_text(_report(summary), encoding="utf-8")

    taskset = shutil.which("taskset")
    metadata = {
        "schema_version": SCHEMA_VERSION,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "classification": summary["classification"],
        "artifact_valid": summary["artifact_valid"],
        "standing_claim_passed": summary["standing_claim_passed"],
        "timing_evidence_eligible": False,
        "command": command,
        "collision_probe_command": collision_probe_command,
        "cwd": str(ROOT),
        "child_returncode": returncode,
        "collision_probe_returncode": probe_returncode,
        "affinity": affinity,
        "binary": str(binary),
        "collision_probe_binary": str(collision_probe),
        "trace_source": str(TRACE_SOURCE.relative_to(ROOT)),
        "collision_probe_source": str(COLLISION_PROBE_SOURCE.relative_to(ROOT)),
        "runner_source": str(Path(__file__).resolve().relative_to(ROOT)),
        "protocol": str(PROTOCOL.relative_to(ROOT)),
        "source_identity": identity,
        "collision_probe": probe_validation,
        "dynamic_pair_identity_evidence": False,
        "positive_long_run_promotion_eligible": False,
        "scene_graph_evidence_scope": {
            "collision_probe": (
                "Compact collision-only constructed time-zero graph: 102 pairs "
                "are 100 adjacent stone pairs plus two springer-ground pairs"
            ),
            "trace": (
                "FourPointPlanar aggregate 400-contact, 100-constraint-pair, "
                "three-color, width-34 fields beginning at step 1; dynamic pair "
                "identities are not present"
            ),
        },
        "runtime_provenance_scope": (
            "Recorded executable, ldd, taskset, and resolved regular "
            "shared-library file identities; this is not a claim over all host "
            "runtime state"
        ),
        "normalized_scene_work_fingerprint_sha256": summary.get(
            "normalized_scene_work_fingerprint_sha256"
        ),
        "taskset": {
            "path": taskset,
            "sha256": _sha256_file(Path(taskset)) if taskset else None,
        },
        "platform": platform.platform(),
        "python": sys.version,
        "environment": {
            key: os.environ.get(key)
            for key in ("OMP_NUM_THREADS", "DART_DISABLE_COMPILER_CACHE")
        },
        "artifact_sha256": {
            "raw.csv": _sha256_file(raw_path),
            "stderr.txt": _sha256_file(stderr_path),
            "collision_probe_stdout.txt": _sha256_file(probe_stdout_path),
            "collision_probe_stderr.txt": _sha256_file(probe_stderr_path),
            "invocation.json": _sha256_file(invocation_path),
            "summary.json": _sha256_file(summary_path),
            "REPORT.md": _sha256_file(report_path),
        },
    }
    _write_json(output / "metadata.json", metadata)
    return 0 if summary["classification"] != "invalid_artifact" else 1


if __name__ == "__main__":
    raise SystemExit(main())
