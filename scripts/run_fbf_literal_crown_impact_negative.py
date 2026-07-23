#!/usr/bin/env python3
"""Capture the frozen literal-wedge crown-impact v1 scientific negative.

This runner deliberately distinguishes a valid evidence artifact from a
passing impact claim. The child trace must complete 720 rows, return the
preregistered fail-closed status, and expose the known failed gates. A zero
child exit or a passing final impact gate invalidates the bundle.
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
DEFAULT_OUTPUT = (
    ROOT / "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence/"
    "fig07_arch25_literal_impact_v1_negative"
)
PREREGISTRATION = (
    ROOT / "docs/dev_tasks/fbf_exact_coulomb_friction/LITERAL_CROWN_IMPACT_V1.md"
)
SCENARIO = "masonry_arch_25_literal_wedge_crown_impact_v1"
REFERENCE_SCENARIO = "masonry_arch_25_literal_wedge"
SCENE_CONTRACT = (
    "reconstructed_literal_wedge_crown_impact_v1_nonpaper_native_" "collision_frontend"
)
SCHEMA_VERSION = "dart.fbf_literal_crown_impact_negative/v1"
EXPECTED_STEPS = 720
EXPECTED_COLUMNS = 136
EXPECTED_REFERENCE_COLUMNS = 95
EXPECTED_HEADER_SHA256 = (
    "47ccfd25d7ef942287a4f1fb54ecbf3b19dabc93cd9bd791057e77fc2339f1e8"
)
EXPECTED_REFERENCE_HEADER_SHA256 = (
    "424195336cb42753179130c9f6fcba3a8ddea9bf669cfa70c0f77fcb4c6335a5"
)
EXPECTED_NORMALIZED_FINGERPRINT = (
    "86b37ec37d28259514453c9ecc9e7d5f12afe08118a0b04abd40e72acd147384"
)
EXPECTED_PREREGISTRATION_CONTRACT_SHA256 = (
    "4cdea674f366fc2d18eadf11ef4333d491786d5d85e3fc16fa611ea7dede3f37"
)
EXPECTED_STDERR = (
    "literal-wedge crown-impact v1 failed a preregistered acceptance gate; "
    "preserve the trace as a scientific negative"
)
EXPECTED_FINAL_GATES = {
    "preimpact_standing_gate": "1",
    "projectile_contact_order_gate": "1",
    "impact_exact_gate": "0",
    "impact_residual_gate": "0",
    "impact_finite_gate": "1",
    "crown_response_gate": "1",
    "final_all_body_displacement_gate": "0",
    "final_orientation_gate": "1",
    "final_far_field_displacement_gate": "0",
    "final_springer_gate": "1",
    "final_far_field_adjacency_gate": "1",
    "final_impact_acceptance_gate": "0",
}
NORMALIZED_FINGERPRINT_EXCLUSIONS = {
    "wall_ms",
    "exact_contact_row_logical_cpus_to_date",
    "max_phase_contact_row_logical_cpus_to_date",
    "exact_colored_bgs_logical_cpus",
    "max_phase_exact_colored_bgs_logical_cpus",
}
STANDING_PREFIX_COMPARISON_EXCLUSIONS = {
    "scenario",
    "scene_contract",
    "wall_ms",
    "exact_contact_row_logical_cpus_to_date",
    "max_phase_contact_row_logical_cpus_to_date",
    "exact_colored_bgs_logical_cpus",
    "max_phase_exact_colored_bgs_logical_cpus",
}
OUTPUT_FILES = (
    "raw.csv",
    "stderr.txt",
    "standing-reference.csv",
    "standing-reference.stderr.txt",
    "metadata.json",
    "summary.json",
    "REPORT.md",
)


class EvidenceError(RuntimeError):
    """Raised when the trace is not the frozen scientific negative."""


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _tool_identity(name: str) -> dict[str, Any]:
    path_text = shutil.which(name)
    if path_text is None:
        raise EvidenceError(f"required tool is unavailable: {name}")
    path = Path(path_text)
    try:
        resolved = path.resolve(strict=True)
    except (OSError, RuntimeError) as error:
        raise EvidenceError(f"{name} tool cannot be resolved: {path}") from error
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
            # linux-vdso is a kernel-provided virtual object, not a regular
            # shared-library file that can be resolved and hashed.
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


def _preregistration_contract_sha256() -> str:
    text = PREREGISTRATION.read_text(encoding="utf-8")
    marker = "## Frozen v1 result"
    if marker not in text:
        raise EvidenceError("preregistration result marker is missing")
    frozen_contract = (text.split(marker, 1)[0].rstrip() + "\n").encode()
    digest = _sha256_bytes(frozen_contract)
    if digest != EXPECTED_PREREGISTRATION_CONTRACT_SHA256:
        raise EvidenceError("frozen preregistration contract hash drifted")
    return digest


def _write_json(path: Path, payload: Any) -> None:
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


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


def _taskset_prefix(
    cpu: int | None, taskset_identity: dict[str, Any] | None
) -> list[str]:
    if cpu is None:
        if taskset_identity is not None:
            raise EvidenceError("taskset identity is invalid without a selected CPU")
        return []
    if taskset_identity is None:
        raise EvidenceError("a pinned taskset identity is required with --cpu")
    resolved_path = taskset_identity.get("resolved_path")
    if not isinstance(resolved_path, str) or not resolved_path:
        raise EvidenceError("pinned taskset identity has no resolved executable path")
    return [resolved_path, "-c", str(cpu)]


def _build_command(
    binary: Path,
    cpu: int | None,
    taskset_identity: dict[str, Any] | None = None,
) -> list[str]:
    command = [
        str(binary),
        SCENARIO,
        "exact_fbf",
        "1",
        str(EXPECTED_STEPS),
        "nan",
        "performance",
        "default",
        "default",
        "1",
        "dart_best_colored_bgs",
        "native",
        "default",
        "0",
        "0",
    ]
    return [*_taskset_prefix(cpu, taskset_identity), *command]


def _build_reference_command(
    binary: Path,
    cpu: int | None,
    taskset_identity: dict[str, Any] | None = None,
) -> list[str]:
    command = [
        str(binary),
        REFERENCE_SCENARIO,
        "exact_fbf",
        "1",
        "600",
        "nan",
        "performance",
        "default",
        "default",
        "1",
        "dart_best_colored_bgs",
        "native",
        "default",
        "0",
        "0",
    ]
    return [*_taskset_prefix(cpu, taskset_identity), *command]


def _require_execution_closure_unchanged(
    *,
    binary: Path,
    runtime_identity: dict[str, Any],
    taskset_identity: dict[str, Any] | None,
    stage: str,
) -> None:
    if _executable_identity(binary) != runtime_identity:
        raise EvidenceError(f"runtime closure drifted after {stage}")
    if taskset_identity is None:
        return
    try:
        current_taskset_identity = _tool_identity("taskset")
    except (EvidenceError, OSError) as error:
        raise EvidenceError(
            f"taskset executable identity cannot be revalidated after {stage}: {error}"
        ) from error
    if current_taskset_identity != taskset_identity:
        raise EvidenceError(f"taskset executable identity drifted after {stage}")


def _parse_csv_trace(
    stdout: str,
    *,
    expected_header_sha256: str,
    expected_columns: int,
    label: str,
) -> tuple[list[str], list[dict[str, str]]]:
    lines = stdout.splitlines()
    if not lines:
        raise EvidenceError(f"{label} emitted no CSV")
    header_payload = (lines[0] + "\n").encode()
    if _sha256_bytes(header_payload) != expected_header_sha256:
        raise EvidenceError(f"{label} header hash mismatch")

    reader = csv.DictReader(io.StringIO(stdout))
    header = list(reader.fieldnames or ())
    if len(header) != expected_columns or len(set(header)) != expected_columns:
        raise EvidenceError(f"{label} must have {expected_columns} unique columns")
    rows = list(reader)
    if any(None in row for row in rows):
        raise EvidenceError("impact trace row has extra columns")
    if any(any(value is None for value in row.values()) for row in rows):
        raise EvidenceError("impact trace row has missing columns")
    return header, rows


def _parse_trace(stdout: str) -> tuple[list[str], list[dict[str, str]]]:
    return _parse_csv_trace(
        stdout,
        expected_header_sha256=EXPECTED_HEADER_SHA256,
        expected_columns=EXPECTED_COLUMNS,
        label="impact trace",
    )


def _parse_reference_trace(
    stdout: str,
) -> tuple[list[str], list[dict[str, str]]]:
    return _parse_csv_trace(
        stdout,
        expected_header_sha256=EXPECTED_REFERENCE_HEADER_SHA256,
        expected_columns=EXPECTED_REFERENCE_COLUMNS,
        label="standing reference trace",
    )


def _compare_standing_prefix(
    *,
    impact_header: Sequence[str],
    impact_rows: Sequence[dict[str, str]],
    reference_header: Sequence[str],
    reference_rows: Sequence[dict[str, str]],
) -> dict[str, Any]:
    if len(impact_rows) < 600 or len(reference_rows) != 600:
        raise EvidenceError("standing-prefix comparison requires 600 rows per lane")
    expected_reference_fields = list(impact_header[:EXPECTED_REFERENCE_COLUMNS])
    if list(reference_header) != expected_reference_fields:
        raise EvidenceError("standing reference schema is not the impact base schema")
    fields = [
        field
        for field in reference_header
        if field not in STANDING_PREFIX_COMPARISON_EXCLUSIONS
    ]
    if len(fields) != 88:
        raise EvidenceError(
            f"standing-prefix comparison expected 88 fields, got {len(fields)}"
        )
    mismatches: list[str] = []
    for index, (impact, reference) in enumerate(
        zip(impact_rows[:600], reference_rows, strict=True), start=1
    ):
        for field in fields:
            if impact[field] != reference[field]:
                mismatches.append(
                    f"step {index} {field}: impact={impact[field]!r} "
                    f"reference={reference[field]!r}"
                )
                if len(mismatches) >= 20:
                    break
        if len(mismatches) >= 20:
            break
    if mismatches:
        raise EvidenceError(
            "standing-prefix reference mismatch: " + "; ".join(mismatches)
        )
    return {
        "reference_scenario": REFERENCE_SCENARIO,
        "steps_compared": 600,
        "fields_compared": len(fields),
        "excluded_fields": sorted(STANDING_PREFIX_COMPARISON_EXCLUSIONS),
        "mismatches": 0,
        "pass": True,
    }


def _normalized_fingerprint(
    header: Sequence[str], rows: Sequence[dict[str, str]]
) -> str:
    fields = [
        field for field in header if field not in NORMALIZED_FINGERPRINT_EXCLUSIONS
    ]
    digest = hashlib.sha256()
    for row in rows:
        for field in fields:
            digest.update(field.encode())
            digest.update(b"=")
            digest.update(row[field].encode())
            digest.update(b"\0")
        digest.update(b"\n")
    return digest.hexdigest()


def _require_frozen_fingerprint(summary: dict[str, Any]) -> None:
    if summary.get("normalized_trace_fingerprint") != EXPECTED_NORMALIZED_FINGERPRINT:
        raise EvidenceError("normalized frozen-negative fingerprint drifted")


def _validate_negative(
    *,
    header: Sequence[str],
    rows: Sequence[dict[str, str]],
    returncode: int,
    stderr: str,
) -> dict[str, Any]:
    if returncode != 1:
        raise EvidenceError(
            f"trace return code must be the expected fail-closed 1, got {returncode}"
        )
    if stderr.strip() != EXPECTED_STDERR:
        raise EvidenceError("trace stderr does not identify the frozen negative")
    if len(rows) != EXPECTED_STEPS:
        raise EvidenceError(f"expected 720 trace rows, got {len(rows)}")
    if [_int(row, "step") for row in rows] != list(range(1, EXPECTED_STEPS + 1)):
        raise EvidenceError("trace steps are not the complete 1..720 sequence")
    if any(row.get("scenario") != SCENARIO for row in rows):
        raise EvidenceError("trace contains an unexpected scenario")
    if any(row.get("scene_contract") != SCENE_CONTRACT for row in rows):
        raise EvidenceError("trace scene contract is not the frozen non-paper label")

    prefix = rows[:600]
    impact = rows[600:]
    if any(
        row.get("impact_phase") != "standing_prefix"
        or row.get("standing_prefix_comparable") != "1"
        or row.get("impact_projectile_count") != "0"
        or row.get("step_projectile_arch_contacts") != "0"
        or row.get("step_projectile_ground_contacts") != "0"
        for row in prefix
    ):
        raise EvidenceError("standing prefix contains impact activity")
    if prefix[-1].get("preimpact_standing_gate") != "1":
        raise EvidenceError("pre-impact standing gate did not pass at step 600")
    if any(
        row.get("impact_phase") != "crown_impact"
        or row.get("standing_prefix_comparable") != "0"
        or row.get("impact_projectile_count") != "3"
        for row in impact
    ):
        raise EvidenceError("impact phase does not contain the frozen three cubes")

    final = rows[-1]
    prefix_final = rows[599]
    if final.get("final_gates_authoritative") != "1":
        raise EvidenceError("final impact gates are not authoritative")
    if final.get("preimpact_snapshot_captured") != "1":
        raise EvidenceError("pre-impact pose snapshot is missing")
    for field, expected in EXPECTED_FINAL_GATES.items():
        if final.get(field) != expected:
            raise EvidenceError(
                f"unexpected frozen final gate {field}: {final.get(field)!r}"
            )

    if _int(final, "first_projectile_arch_contact_step") != 607:
        raise EvidenceError("first projectile-arch contact step drifted")
    if _int(final, "first_projectile_ground_contact_step") != 616:
        raise EvidenceError("first projectile-ground contact step drifted")
    if _int(final, "exact_failures_to_date") != 0:
        raise EvidenceError("scientific negative contains an exact failure")
    if _int(final, "boxed_fallbacks_to_date") != 0:
        raise EvidenceError("scientific negative contains a boxed fallback")
    if _int(final, "max_iterations_accepted_to_date") != 5:
        raise EvidenceError("accepted-cap count drifted from the frozen result")
    if _int(final, "exact_solves_to_date") <= _int(
        prefix_final, "exact_solves_to_date"
    ):
        raise EvidenceError("impact phase contains no exact solve progress")

    residual = _float(final, "worst_exact_residual_to_date")
    max_crown = _float(final, "max_crown_displacement_to_date")
    max_arch = _float(final, "max_arch_body_displacement_from_preimpact")
    min_orientation = _float(final, "min_arch_orientation_alignment_from_preimpact")
    max_far_field = _float(final, "max_far_field_displacement_from_preimpact")
    max_springer = _float(final, "max_springer_displacement_from_preimpact")
    min_springer_alignment = _float(
        final, "min_springer_orientation_alignment_from_preimpact"
    )
    finite_metrics = (
        residual,
        max_crown,
        max_arch,
        min_orientation,
        max_far_field,
        max_springer,
        min_springer_alignment,
    )
    if not all(math.isfinite(value) for value in finite_metrics):
        raise EvidenceError("scientific negative contains a non-finite gate metric")
    if residual <= 1e-6:
        raise EvidenceError("frozen residual-failure reason is absent")
    if max_arch <= 0.07:
        raise EvidenceError("frozen all-body displacement failure is absent")
    if max_far_field <= 0.007:
        raise EvidenceError("frozen far-field displacement failure is absent")
    if _int(final, "far_field_adjacent_pairs") != 16:
        raise EvidenceError("far-field adjacency preservation drifted")

    recomputed_gates = {
        "preimpact_standing_gate": _int(final, "preimpact_standing_gate") == 1,
        "projectile_contact_order_gate": (
            _int(final, "first_projectile_arch_contact_step") > 0
            and _int(final, "first_projectile_arch_contact_step")
            < _int(final, "first_projectile_ground_contact_step")
        ),
        "impact_exact_gate": (
            _int(final, "exact_solves_to_date")
            > _int(prefix_final, "exact_solves_to_date")
            and _int(final, "exact_failures_to_date") == 0
            and _int(final, "boxed_fallbacks_to_date") == 0
            and _int(final, "max_iterations_accepted_to_date") == 0
        ),
        "impact_residual_gate": residual <= 1e-6,
        "impact_finite_gate": _int(final, "finite_state_to_date") == 1,
        "crown_response_gate": max_crown >= 0.0001,
        "final_all_body_displacement_gate": max_arch <= 0.07,
        "final_orientation_gate": min_orientation >= 0.8660254037844386,
        "final_far_field_displacement_gate": max_far_field <= 0.007,
        "final_springer_gate": (
            max_springer <= 1e-12 and min_springer_alignment >= 1.0 - 1e-12
        ),
        "final_far_field_adjacency_gate": (
            _int(final, "far_field_adjacent_pairs") == 16
        ),
    }
    recomputed_gates["final_impact_acceptance_gate"] = all(recomputed_gates.values())
    for field, recomputed in recomputed_gates.items():
        emitted = bool(_int(final, field))
        if emitted is not recomputed:
            raise EvidenceError(
                f"emitted gate {field}={emitted} disagrees with recomputed "
                f"value {recomputed}"
            )

    failed_gates = [
        field for field, expected in EXPECTED_FINAL_GATES.items() if expected == "0"
    ]
    return {
        "schema_version": SCHEMA_VERSION,
        "classification": "valid_scientific_negative",
        "artifact_valid": True,
        "impact_claim_passed": False,
        "expected_fail_closed_process_observed": True,
        "scenario": SCENARIO,
        "scene_contract": SCENE_CONTRACT,
        "trace_rows": len(rows),
        "trace_columns": len(header),
        "trace_header_sha256": EXPECTED_HEADER_SHA256,
        "normalized_trace_fingerprint": _normalized_fingerprint(header, rows),
        "standing_prefix": {
            "steps": 600,
            "projectile_free": True,
            "preimpact_standing_gate": True,
        },
        "contact_order": {
            "first_projectile_arch_contact_step": _int(
                final, "first_projectile_arch_contact_step"
            ),
            "first_projectile_arch_contact_time": _float(
                final, "first_projectile_arch_contact_time"
            ),
            "first_projectile_ground_contact_step": _int(
                final, "first_projectile_ground_contact_step"
            ),
            "first_projectile_ground_contact_time": _float(
                final, "first_projectile_ground_contact_time"
            ),
            "gate_passed": True,
        },
        "solver": {
            "exact_solves": _int(final, "exact_solves_to_date"),
            "exact_failures": 0,
            "boxed_fallbacks": 0,
            "accepted_iteration_caps": _int(final, "max_iterations_accepted_to_date"),
            "worst_residual": _float(final, "worst_exact_residual_to_date"),
            "residual_limit": 1e-6,
        },
        "preservation": {
            "maximum_crown_displacement": _float(
                final, "max_crown_displacement_to_date"
            ),
            "minimum_crown_response": 0.0001,
            "final_maximum_arch_displacement": _float(
                final, "max_arch_body_displacement_from_preimpact"
            ),
            "maximum_arch_displacement_limit": 0.07,
            "final_minimum_orientation_alignment": _float(
                final, "min_arch_orientation_alignment_from_preimpact"
            ),
            "minimum_orientation_alignment": 0.8660254037844386,
            "final_maximum_far_field_displacement": _float(
                final, "max_far_field_displacement_from_preimpact"
            ),
            "maximum_far_field_displacement_limit": 0.007,
            "final_maximum_springer_displacement": _float(
                final, "max_springer_displacement_from_preimpact"
            ),
            "final_minimum_springer_alignment": _float(
                final, "min_springer_orientation_alignment_from_preimpact"
            ),
            "final_far_field_adjacent_pairs": _int(final, "far_field_adjacent_pairs"),
        },
        "final_gate_values": recomputed_gates,
        "failed_gates": failed_gates,
        "claim_boundary": (
            "This is a valid fail-closed artifact for a reconstructed, non-paper "
            "scientific negative. It is not a passing impact or paper-parity claim."
        ),
    }


def _metadata(
    *,
    binary: Path,
    command: Sequence[str],
    returncode: int,
    raw_path: Path,
    stderr_path: Path,
    reference_command: Sequence[str],
    reference_returncode: int,
    reference_path: Path,
    reference_stderr_path: Path,
    preregistration_contract_sha256: str,
    runtime_identity: dict[str, Any],
    taskset_identity: dict[str, Any] | None,
    identity_rechecks: Sequence[str],
    selected_cpu: int | None,
) -> dict[str, Any]:
    source = ROOT / "tests/benchmark/integration/fbf_paper_trace.cpp"
    runner = Path(__file__).resolve()
    return {
        "schema_version": SCHEMA_VERSION,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "classification": "valid_scientific_negative",
        "impact_claim_passed": False,
        "command": list(command),
        "cwd": str(ROOT),
        "child_returncode": returncode,
        "binary": str(binary.resolve()),
        "binary_size_bytes": runtime_identity["size_bytes"],
        "binary_sha256": runtime_identity["sha256"],
        "runtime_identity": runtime_identity,
        "executed_tool_closure": (
            {"taskset": taskset_identity} if taskset_identity is not None else {}
        ),
        "identity_rechecks": list(identity_rechecks),
        "runtime_provenance_scope": (
            "Recorded the executed trace binary, ldd tool, every resolved regular "
            "shared-library file, and exactly one build-tree libdart with resolved "
            "paths, sizes, and SHA-256 digests. When CPU affinity was requested, "
            "the exact resolved taskset executable was used and recorded with its "
            "size and SHA-256 digest. The full closure was unchanged after both "
            "child traces."
        ),
        "trace_source": str(source.relative_to(ROOT)),
        "trace_source_sha256": _sha256_file(source),
        "runner_source": str(runner.relative_to(ROOT)),
        "runner_source_sha256": _sha256_file(runner),
        "preregistration": str(PREREGISTRATION.relative_to(ROOT)),
        "preregistration_contract_sha256": preregistration_contract_sha256,
        "raw_csv_sha256": _sha256_file(raw_path),
        "stderr_sha256": _sha256_file(stderr_path),
        "standing_reference_command": list(reference_command),
        "standing_reference_returncode": reference_returncode,
        "standing_reference_csv_sha256": _sha256_file(reference_path),
        "standing_reference_stderr_sha256": _sha256_file(reference_stderr_path),
        "platform": platform.platform(),
        "python": sys.version,
        "selected_cpu": selected_cpu,
        "environment": {
            key: os.environ.get(key)
            for key in ("OMP_NUM_THREADS", "DART_DISABLE_COMPILER_CACHE")
        },
    }


def _report(summary: dict[str, Any], metadata: dict[str, Any]) -> str:
    solver = summary["solver"]
    preservation = summary["preservation"]
    contact = summary["contact_order"]
    return f"""# Literal-Wedge Crown-Impact v1 Scientific Negative

Classification: **valid scientific negative**. Impact claim passed: **no**.

The frozen reconstructed/non-paper trace completed {summary['trace_rows']} steps,
returned the expected fail-closed child status `{metadata['child_returncode']}`,
and preserved a projectile-free standing prefix through step 600.

The independently rerun standing scenario matches that prefix across
{summary['standing_prefix_comparison']['fields_compared']} eligible fields for
all {summary['standing_prefix_comparison']['steps_compared']} steps with zero
mismatches.

- First projectile-arch contact: step {contact['first_projectile_arch_contact_step']}
- First projectile-ground contact: step {contact['first_projectile_ground_contact_step']}
- Accepted iteration caps: {solver['accepted_iteration_caps']}
- Worst residual: {solver['worst_residual']:.17g} (limit `1e-6`)
- Final maximum arch displacement: {preservation['final_maximum_arch_displacement']:.17g} m (limit `0.07 m`)
- Final maximum far-field displacement: {preservation['final_maximum_far_field_displacement']:.17g} m (limit `0.007 m`)
- Failed gates: {', '.join(summary['failed_gates'])}

This bundle validates the expected negative artifact only. It must never be
presented as a passing impact example or as paper parity.
"""


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=DEFAULT_BINARY)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--cpu", type=int)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    binary = args.binary.resolve()
    output = args.output_dir.resolve()
    if not binary.is_file():
        raise SystemExit(f"trace binary not found: {binary}")
    if output.exists() and any(output.iterdir()):
        raise SystemExit(f"refusing to overwrite non-empty evidence dir: {output}")
    output.mkdir(parents=True, exist_ok=True)
    try:
        preregistration_contract_sha256 = _preregistration_contract_sha256()
        runtime_identity = _executable_identity(binary)
        taskset_identity = _tool_identity("taskset") if args.cpu is not None else None
    except (EvidenceError, OSError) as error:
        raise SystemExit(str(error)) from error

    command = _build_command(binary, args.cpu, taskset_identity)
    result = subprocess.run(
        command,
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    raw_path = output / "raw.csv"
    stderr_path = output / "stderr.txt"
    raw_path.write_text(result.stdout, encoding="utf-8")
    stderr_path.write_text(result.stderr, encoding="utf-8")

    identity_rechecks: list[str] = []
    try:
        _require_execution_closure_unchanged(
            binary=binary,
            runtime_identity=runtime_identity,
            taskset_identity=taskset_identity,
            stage="impact trace",
        )
        identity_rechecks.append("after_impact_trace")
        header, rows = _parse_trace(result.stdout)
        summary = _validate_negative(
            header=header,
            rows=rows,
            returncode=result.returncode,
            stderr=result.stderr,
        )
        _require_frozen_fingerprint(summary)
        reference_command = _build_reference_command(binary, args.cpu, taskset_identity)
        reference_result = subprocess.run(
            reference_command,
            cwd=ROOT,
            text=True,
            capture_output=True,
            check=False,
        )
        reference_path = output / "standing-reference.csv"
        reference_stderr_path = output / "standing-reference.stderr.txt"
        reference_path.write_text(reference_result.stdout, encoding="utf-8")
        reference_stderr_path.write_text(reference_result.stderr, encoding="utf-8")
        _require_execution_closure_unchanged(
            binary=binary,
            runtime_identity=runtime_identity,
            taskset_identity=taskset_identity,
            stage="standing reference trace",
        )
        identity_rechecks.append("after_standing_reference_trace")
        if reference_result.returncode != 0:
            raise EvidenceError(
                "standing reference trace did not complete successfully"
            )
        reference_header, reference_rows = _parse_reference_trace(
            reference_result.stdout
        )
        summary["standing_prefix_comparison"] = _compare_standing_prefix(
            impact_header=header,
            impact_rows=rows,
            reference_header=reference_header,
            reference_rows=reference_rows,
        )
        summary["standing_prefix_comparison"]["reference_csv_sha256"] = _sha256_file(
            reference_path
        )
    except (EvidenceError, OSError) as error:
        failure = {
            "schema_version": SCHEMA_VERSION,
            "classification": "invalid_artifact",
            "artifact_valid": False,
            "impact_claim_passed": False,
            "error": str(error),
        }
        _write_json(output / "summary.json", failure)
        raise SystemExit(str(error)) from error

    metadata = _metadata(
        binary=binary,
        command=command,
        returncode=result.returncode,
        raw_path=raw_path,
        stderr_path=stderr_path,
        reference_command=reference_command,
        reference_returncode=reference_result.returncode,
        reference_path=reference_path,
        reference_stderr_path=reference_stderr_path,
        preregistration_contract_sha256=preregistration_contract_sha256,
        runtime_identity=runtime_identity,
        taskset_identity=taskset_identity,
        identity_rechecks=identity_rechecks,
        selected_cpu=args.cpu,
    )
    _write_json(output / "summary.json", summary)
    _write_json(output / "metadata.json", metadata)
    (output / "REPORT.md").write_text(_report(summary, metadata), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
