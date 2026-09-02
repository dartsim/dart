#!/usr/bin/env python3
"""Shared schema contract for AVBD evidence packets (PLAN-091 WP-091.1).

This module owns the AVBD packet-writer family's shared packet schema
version and the resolved-solver-identity field contract. From
``AVBD_PACKET_SCHEMA_VERSION`` onward a packet is valid only when it
machine-records the solver configuration that actually ran; committed
packets at older versions stay readable, and their contact rows are
relabeled in prose instead (see the standing rule in
``docs/design/dart7_architecture_assessment.md`` and item 10 of
``docs/plans/solver-family-intake.md``).

Writer scripts adopt this contract the next time their packet is
regenerated; ``scripts/check_avbd_packets.py`` enforces it for
committed packets. Other packet families adopt the same field contract
in follow-up PLAN-091 packets.
"""

from __future__ import annotations

import hashlib
import math
import re
import struct
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

# Current AVBD packet schema version. Packets written at this version or
# newer must carry RESOLVED_SOLVER_IDENTITY_KEY and record how rigid contact
# selected the reported solver family. Schema version 5 additionally reserves
# ``plan104_claims`` for typed, row-bound PLAN-104 closure evidence. Schema
# version 6 binds paper benchmark rows to a separately fingerprinted solver
# configuration and the numeric AVBD Table 2 profile. The claim field stays
# optional because partial packets must remain valid evidence without claiming
# completion.
AVBD_PACKET_SCHEMA_VERSION = 6
PLAN104_CLAIMS_MIN_SCHEMA_VERSION = 5

# Google Benchmark aggregates repetition counters in double precision. A
# 32-bit fingerprint word stored as a counter is ~1e9, so its squared terms
# (~1e18) exceed the exactly representable integer range and the stddev of five
# identical values can come out as tens instead of zero. A genuinely different
# fingerprint in one repetition moves the stddev by a large fraction of the
# word itself, so a relative bound separates the two by many orders.
# The squared term of a word `v` carries an absolute rounding error of about
# ulp(v^2) = v^2 * 2^-52, so the reported spread of identical words is bounded
# by roughly v * 2^-26 (about 19 for v ~ 1.26e9). Twice that bound still sits
# six orders below any real fingerprint change.
FINGERPRINT_WORD_STDDEV_RELATIVE_BOUND = 2.0 * 2.0**-26
FINGERPRINT_WORD_COUNTER_SUFFIXES = ("_fingerprint_hi", "_fingerprint_lo")


def stable_counter_stddev_is_noise(key: str, stddev: object, reference: object) -> bool:
    """Whether a repetition-invariant counter's stddev proves no drift.

    Every counter except the 32-bit fingerprint words must report an exact
    zero spread; the fingerprint words may carry the double-precision
    aggregation noise documented above, scaled by their median value.
    """
    if not isinstance(stddev, (int, float)) or isinstance(stddev, bool):
        return False
    if not math.isfinite(stddev):
        return False
    if stddev == 0.0:
        return True
    if not key.endswith(FINGERPRINT_WORD_COUNTER_SUFFIXES):
        return False
    if not isinstance(reference, (int, float)) or isinstance(reference, bool):
        return False
    if not math.isfinite(reference):
        return False
    return abs(float(stddev)) <= FINGERPRINT_WORD_STDDEV_RELATIVE_BOUND * abs(
        float(reference)
    )


# Flag variables the project extends in scope after the caller's cache value,
# and the only tokens it appends. `run_figure13_benchmark.py` pins the same
# table (it is a sealed capture root, so the runner keeps its own copy) and
# `tests/test_avbd_packet_schema.py` keeps the two in step.
EVIDENCE_PROJECT_APPENDED_FLAGS: dict[str, frozenset[str]] = {
    "CMAKE_SHARED_LINKER_FLAGS": frozenset({"-Wl,--no-undefined"}),
}


def evidence_definition_matches(name: str, expected: str, compiled: object) -> bool:
    """Whether a compiled build-configuration value satisfies an evidence pin.

    The cache proves the caller injected no flags; the project itself appends
    link options in scope (`-Wl,--no-undefined` for shared libraries), which
    the compiled record keeps verbatim because its digest binds the exact
    flags the benchmark was built with. Only that project-appended suffix is
    accepted on top of the pinned value.
    """
    if compiled == expected:
        return True
    tokens = EVIDENCE_PROJECT_APPENDED_FLAGS.get(name)
    if tokens is None or not isinstance(compiled, str):
        return False
    if not compiled.startswith(expected):
        return False
    suffix = compiled[len(expected) :].split()
    return bool(suffix) and set(suffix) <= tokens


# Packet-level source seal: an ordered list of repository files whose bytes the
# packet was produced from. `check_avbd_packets.py` recomputes every digest, so
# any drift in a bound file invalidates the packet until it is regenerated.
SOURCE_PROVENANCE_ALGORITHM = "sha256-length-prefixed-path-and-content-v1"


def make_source_provenance(
    repo_root: Path, relative_paths: Sequence[str]
) -> dict[str, Any]:
    """Seal `relative_paths` (repository-relative, in order) under `repo_root`."""
    if not relative_paths:
        raise ValueError("source provenance needs at least one bound file")
    combined = hashlib.sha256()
    files: list[dict[str, str]] = []
    for relative_text in relative_paths:
        relative = Path(relative_text)
        try:
            payload = (repo_root / relative).read_bytes()
        except FileNotFoundError as exc:
            raise ValueError(f"{relative_text}: source file not found") from exc
        encoded_path = relative.as_posix().encode("utf-8")
        combined.update(struct.pack("<Q", len(encoded_path)))
        combined.update(encoded_path)
        combined.update(struct.pack("<Q", len(payload)))
        combined.update(payload)
        files.append(
            {
                "path": relative.as_posix(),
                "sha256": hashlib.sha256(payload).hexdigest(),
            }
        )
    return {
        "algorithm": SOURCE_PROVENANCE_ALGORITHM,
        "digest": combined.hexdigest(),
        "files": files,
    }


# Legacy packets pinned below the claims contract carry their own claim
# boundary so downstream readers cannot promote a bare historical skeleton into
# current AVBD evidence. `check_avbd_packets.py` compares these fields exactly.
LEGACY_EVIDENCE_BOUNDARY_STATUS = "legacy_unbound"


def legacy_evidence_boundary(supported_scope: str, *, reason: str) -> dict[str, Any]:
    """Return the claim boundary of a legacy packet with unavailable inputs."""
    return {
        "artifact_status": LEGACY_EVIDENCE_BOUNDARY_STATUS,
        "avbd_performance_claim_supported": False,
        "avbd_solver_evidence": False,
        "capture_artifacts_accessible": False,
        "current_build_bound": False,
        "historical_identifiers_retained": True,
        "historical_measurements_preserved": True,
        "measurement_runtime_identity_recorded": False,
        "plan104_avbd_row_closure_supported": False,
        "semantic_visual_review_recorded": False,
        "reason": reason,
        "supported_scope": supported_scope,
    }


MULTIBODY_IDENTITY_MIN_SCHEMA_VERSION = 6
SOLVER_CONFIGURATION_MIN_SCHEMA_VERSION = 6

# Exact ordered source-provenance contracts for the current Figure 13 packet
# chain. Writers and the committed-packet checker consume this one mapping so
# deleting, adding, or reordering a bound source cannot be hidden by
# recomputing the packet digest.
PAPER_PACKET_SOURCE_PATHS: dict[str, tuple[str, ...]] = {
    "avbd-paper-breakable-wall-packet.json": (
        "dart/gui/view_quality.cpp",
        "dart/gui/view_quality.hpp",
        "dart/simulation/world.cpp",
        "dart/simulation/world.hpp",
        "dart/simulation/world_options.hpp",
        "dart/simulation/comps/joint.hpp",
        "dart/simulation/compute/detail/world_step_stages.hpp",
        "dart/simulation/compute/rigid_body_contact_stage.cpp",
        "dart/simulation/detail/rigid_pair_constraint.hpp",
        "dart/simulation/detail/rigid_avbd/rigid_block_kernel.hpp",
        "dart/simulation/detail/rigid_avbd/rigid_world_contact.hpp",
        "dart/simulation/detail/world_step_schedule.hpp",
        "python/dartpy/_view_quality.py",
        "python/examples/demos/scenes/avbd_paper_breakable_wall.py",
        "python/tests/integration/test_demos_cycle.py",
        "tests/benchmark/simulation/bm_avbd_rigid_fixed_joint.cpp",
        "tests/unit/simulation/world/test_world.cpp",
        "scripts/_image_tools.py",
        "scripts/avbd_packet_schema.py",
        "scripts/capture_py_demo.py",
        "scripts/capture_source_provenance.py",
        "scripts/image_verdict.py",
        "scripts/write_avbd_paper_breakable_wall_packet.py",
    ),
    "avbd-paper-vbd-comparison-packet.json": (
        "dart/simulation/world.cpp",
        "dart/simulation/world.hpp",
        "dart/simulation/world_options.hpp",
        "dart/simulation/comps/joint.hpp",
        "dart/simulation/compute/detail/world_step_stages.hpp",
        "dart/simulation/compute/rigid_body_contact_stage.cpp",
        "dart/simulation/detail/rigid_pair_constraint.hpp",
        "dart/simulation/detail/rigid_avbd/rigid_block_kernel.hpp",
        "dart/simulation/detail/rigid_avbd/rigid_world_contact.hpp",
        "dart/simulation/detail/world_step_schedule.hpp",
        "python/examples/demos/scenes/avbd_paper_breakable_wall.py",
        "python/examples/demos/scenes/vbd_paper_breakable_wall.py",
        "python/tests/integration/test_demos_cycle.py",
        "tests/benchmark/simulation/bm_avbd_rigid_fixed_joint.cpp",
        "tests/unit/simulation/world/test_world.cpp",
        "scripts/_image_tools.py",
        "scripts/avbd_packet_schema.py",
        "scripts/capture_py_demo.py",
        "scripts/capture_source_provenance.py",
        "scripts/image_verdict.py",
        "scripts/write_avbd_paper_breakable_wall_packet.py",
        "scripts/write_avbd_paper_vbd_comparison_packet.py",
    ),
    "avbd-paper-sequential-impulse-comparison-packet.json": (
        "dart/simulation/world.cpp",
        "dart/simulation/world.hpp",
        "dart/simulation/world_options.hpp",
        "dart/simulation/comps/joint.hpp",
        "dart/simulation/compute/detail/world_step_stages.hpp",
        "dart/simulation/compute/rigid_body_contact_stage.cpp",
        "dart/simulation/detail/rigid_pair_constraint.hpp",
        "dart/simulation/detail/world_step_schedule.hpp",
        "python/examples/demos/scenes/avbd_paper_breakable_wall.py",
        "python/examples/demos/scenes/vbd_paper_breakable_wall.py",
        "python/examples/demos/scenes/sequential_impulse_paper_breakable_wall.py",
        "python/tests/integration/test_demos_cycle.py",
        "tests/benchmark/simulation/bm_avbd_rigid_fixed_joint.cpp",
        "tests/unit/simulation/contact/test_boxed_lcp_contact.cpp",
        "tests/unit/simulation/world/test_world.cpp",
        "tests/unit/simulation/world/test_world_resolved_configuration.cpp",
        "tests/test_avbd_packet_schema.py",
        "scripts/_image_tools.py",
        "scripts/avbd_packet_schema.py",
        "scripts/capture_py_demo.py",
        "scripts/capture_source_provenance.py",
        "scripts/image_verdict.py",
        "scripts/write_avbd_paper_breakable_wall_packet.py",
        "scripts/write_avbd_paper_vbd_comparison_packet.py",
        "scripts/write_avbd_paper_sequential_impulse_comparison_packet.py",
    ),
}

# Schema version 4 narrowed 'contact_solver_method' point-joint identities to
# 'none' or 'sequential_impulse'. Older packets legitimately recorded 'avbd'
# there because public pair constraints then ran the private AVBD projection.
CONTACT_METHOD_POINT_JOINT_MIN_SCHEMA_VERSION = 4

# First schema version that requires the resolved-solver-identity field.
SOLVER_IDENTITY_MIN_SCHEMA_VERSION = 2

# First schema version that requires the rigid-contact selection source.
RIGID_CONTACT_SELECTION_MIN_SCHEMA_VERSION = 3

RESOLVED_SOLVER_IDENTITY_KEY = "resolved_solver_identity"
RIGID_CONTACT_SELECTION_KEY = "rigid_contact_selection"
MULTIBODY_INTEGRATION_FAMILY_KEY = "multibody_integration_family"
IDENTITY_SCOPE_KEY = "identity_scope"
PER_BENCHMARK_ROW_IDENTITY_SCOPE = "per_benchmark_row"
PLAN104_CLAIMS_KEY = "plan104_claims"

PLAN104_PREDICATES = (
    "artifact_valid",
    "solver_contract_valid",
    "physical_outcome_valid",
    "manual_inspected",
    "media_decoded",
    "performance_comparable",
    "claim_valid",
)
PLAN104_BACKENDS = ("cpu", "cuda")
PLAN104_CLAIM_ID_PATTERN = re.compile(r"^(?:avbd|vbd)(?:\.[a-z0-9][a-z0-9_]*){2,}$")

# The contact path that actually resolved rigid-rigid contacts in the timed
# scene. "vbd" and "avbd" may be selected by their public world solver
# families; "avbd" may also be selected by the compatibility-only internal
# body opt-in. Contact-free scenes record "none".
ALLOWED_RIGID_CONTACT_SOLVERS = (
    "avbd",
    "boxed_lcp",
    "none",
    "sequential_impulse",
    "vbd",
)

# The solver family that resolved rigid-body point-joint/motor/distance-spring
# rows; joint-free scenes record "none".
ALLOWED_RIGID_POINT_JOINT_SOLVERS = (
    "avbd",
    "none",
    "sequential_impulse",
    "vbd",
)

ALLOWED_RIGID_CONTACT_SELECTIONS = (
    "body_opt_in",
    "contact_solver_method",
    "not_applicable",
    "world_solver_family",
)

ALLOWED_MULTIBODY_INTEGRATION_FAMILIES = (
    "none",
    "semi_implicit",
    "variational",
)

_REQUIRED_ENUM_FIELDS: tuple[tuple[str, tuple[str, ...]], ...] = (
    ("rigid_contact_solver", ALLOWED_RIGID_CONTACT_SOLVERS),
    ("rigid_point_joint_solver", ALLOWED_RIGID_POINT_JOINT_SOLVERS),
)


def packet_schema_version_errors(
    packet: Mapping[str, Any], packet_name: str
) -> list[str]:
    """Validate the packet's schema_version field."""
    version = packet.get("schema_version")
    if not isinstance(version, int) or isinstance(version, bool):
        return [f"{packet_name}: schema_version must be an integer"]
    if version < 1:
        return [f"{packet_name}: schema_version must be >= 1"]
    if version > AVBD_PACKET_SCHEMA_VERSION:
        return [
            f"{packet_name}: schema_version {version} is newer than the "
            f"supported version {AVBD_PACKET_SCHEMA_VERSION}"
        ]
    return []


def _ordered_boolean_map_errors(
    value: Any,
    *,
    label: str,
    allowed_keys: tuple[str, ...],
    require_exact_keys: bool,
) -> list[str]:
    if not isinstance(value, Mapping):
        return [f"{label} must be an object"]

    errors: list[str] = []
    keys = tuple(value)
    unknown = [key for key in keys if key not in allowed_keys]
    if unknown:
        rendered = ", ".join(sorted(repr(key) for key in unknown))
        errors.append(f"{label} has unknown keys: {rendered}")
    expected_order = tuple(key for key in allowed_keys if key in value)
    if keys != expected_order:
        errors.append(f"{label} keys must follow this order: {', '.join(allowed_keys)}")
    if require_exact_keys and keys != allowed_keys:
        errors.append(
            f"{label} must contain exactly, in order: {', '.join(allowed_keys)}"
        )
    if not require_exact_keys and not keys:
        errors.append(f"{label} must contain at least one result")
    for key, result in value.items():
        if not isinstance(result, bool):
            errors.append(f"{label}.{key} must be boolean")
    return errors


def plan104_claims_errors(packet: Mapping[str, Any], packet_name: str) -> list[str]:
    """Validate optional typed PLAN-104 row claims in a current packet.

    A packet may remain useful partial evidence without ``plan104_claims``.
    When the field is present, each row claim is deliberately narrow: it owns
    only the ordered predicate and backend result maps that the coverage
    contract compares byte-for-byte before accepting closure.
    """
    errors = packet_schema_version_errors(packet, packet_name)
    if errors:
        return errors

    if PLAN104_CLAIMS_KEY not in packet:
        return []
    claims = packet[PLAN104_CLAIMS_KEY]
    version = packet["schema_version"]
    if version < PLAN104_CLAIMS_MIN_SCHEMA_VERSION:
        return [
            f"{packet_name}: {PLAN104_CLAIMS_KEY} requires schema_version "
            f"{PLAN104_CLAIMS_MIN_SCHEMA_VERSION}"
        ]
    if not isinstance(claims, Mapping) or not claims:
        return [f"{packet_name}: {PLAN104_CLAIMS_KEY} must be a non-empty object"]

    target = packet.get("target")
    contract_rows: set[str] = set()
    if not isinstance(target, Mapping):
        errors.append(
            f"{packet_name}: {PLAN104_CLAIMS_KEY} requires target to be an object"
        )
    else:
        raw_contract_rows = target.get("contract_rows")
        if not isinstance(raw_contract_rows, list) or not raw_contract_rows:
            errors.append(
                f"{packet_name}: {PLAN104_CLAIMS_KEY} requires "
                "target.contract_rows to be a non-empty list"
            )
        else:
            for index, row_id in enumerate(raw_contract_rows):
                if (
                    not isinstance(row_id, str)
                    or PLAN104_CLAIM_ID_PATTERN.fullmatch(row_id) is None
                ):
                    errors.append(
                        f"{packet_name}: target.contract_rows[{index}] must be a "
                        "canonical VBD or AVBD row ID"
                    )
                    continue
                if row_id in contract_rows:
                    errors.append(
                        f"{packet_name}: target.contract_rows must not duplicate "
                        f"{row_id!r}"
                    )
                contract_rows.add(row_id)

    for claim_id, claim in claims.items():
        label = f"{packet_name}: {PLAN104_CLAIMS_KEY}.{claim_id}"
        if (
            not isinstance(claim_id, str)
            or PLAN104_CLAIM_ID_PATTERN.fullmatch(claim_id) is None
        ):
            errors.append(
                f"{packet_name}: {PLAN104_CLAIMS_KEY} keys must be canonical "
                "VBD or AVBD row IDs"
            )
            continue
        if claim_id not in contract_rows:
            errors.append(f"{label} is not authorized by target.contract_rows")
        if not isinstance(claim, Mapping):
            errors.append(f"{label} must be an object")
            continue
        if tuple(claim) != ("status", "predicate_results", "backend_results"):
            errors.append(
                f"{label} must contain exactly, in order: "
                "status, predicate_results, backend_results"
            )
        if claim.get("status") != "complete":
            errors.append(f"{label}.status must be 'complete'")
        errors.extend(
            _ordered_boolean_map_errors(
                claim.get("predicate_results"),
                label=f"{label}.predicate_results",
                allowed_keys=PLAN104_PREDICATES,
                require_exact_keys=False,
            )
        )
        predicate_results = claim.get("predicate_results")
        if isinstance(predicate_results, Mapping):
            false_predicates = [
                predicate
                for predicate in predicate_results
                if predicate_results.get(predicate) is not True
            ]
            if false_predicates:
                rendered = ", ".join(
                    predicate if isinstance(predicate, str) else repr(predicate)
                    for predicate in false_predicates
                )
                errors.append(
                    f"{label}.predicate_results must be true for: " f"{rendered}"
                )
        errors.extend(
            _ordered_boolean_map_errors(
                claim.get("backend_results"),
                label=f"{label}.backend_results",
                allowed_keys=PLAN104_BACKENDS,
                require_exact_keys=True,
            )
        )
        backend_results = claim.get("backend_results")
        if isinstance(backend_results, Mapping):
            false_backends = [
                backend
                for backend in PLAN104_BACKENDS
                if backend_results.get(backend) is not True
            ]
            if false_backends:
                errors.append(
                    f"{label}.backend_results must be true for: "
                    f"{', '.join(false_backends)}"
                )
    return errors


def _per_benchmark_row_solver_identity_errors(
    packet: Mapping[str, Any], packet_name: str, identity: Mapping[str, Any]
) -> list[str]:
    """Validate a deliberately heterogeneous benchmark packet identity.

    A heterogeneous packet cannot truthfully summarize its runtime solver with
    one flat identity. Schema-v6 packets may therefore put a narrow scope
    marker at the packet root and attach a complete flat identity to every raw
    and derived benchmark row. Requiring at least two distinct row identities
    prevents homogeneous packets from using this escape hatch.
    """
    version = packet["schema_version"]
    label = f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}"
    if version < MULTIBODY_IDENTITY_MIN_SCHEMA_VERSION:
        return [
            f"{label}.{IDENTITY_SCOPE_KEY} requires schema_version "
            f"{MULTIBODY_IDENTITY_MIN_SCHEMA_VERSION} or newer"
        ]

    errors: list[str] = []
    expected_keys = {IDENTITY_SCOPE_KEY, "recorded_from"}
    if set(identity) != expected_keys:
        errors.append(
            f"{label} with {IDENTITY_SCOPE_KEY} must contain exactly "
            f"{sorted(expected_keys)!r}"
        )
    if identity.get(IDENTITY_SCOPE_KEY) != PER_BENCHMARK_ROW_IDENTITY_SCOPE:
        errors.append(
            f"{label}.{IDENTITY_SCOPE_KEY} must be "
            f"{PER_BENCHMARK_ROW_IDENTITY_SCOPE!r}"
        )
    recorded_from = identity.get("recorded_from")
    if not isinstance(recorded_from, str) or not recorded_from:
        errors.append(f"{label}.recorded_from must be a non-empty string")

    benchmark = packet.get("benchmark")
    if not isinstance(benchmark, Mapping):
        errors.append(f"{packet_name}: benchmark must be an object")
        return errors

    collections_found = 0
    signatures: set[tuple[object, ...]] = set()
    identity_by_benchmark: dict[str, tuple[object, ...]] = {}
    for collection_name in ("rows", "scale_data"):
        if collection_name not in benchmark:
            continue
        collections_found += 1
        rows = benchmark[collection_name]
        collection_label = f"{packet_name}: benchmark.{collection_name}"
        if not isinstance(rows, list) or not rows:
            errors.append(f"{collection_label} must be a non-empty list")
            continue
        for index, row in enumerate(rows):
            row_label = f"{collection_label}[{index}]"
            if not isinstance(row, Mapping):
                errors.append(f"{row_label} must be an object")
                continue
            row_identity = row.get(RESOLVED_SOLVER_IDENTITY_KEY)
            if not isinstance(row_identity, Mapping):
                errors.append(
                    f"{row_label}.{RESOLVED_SOLVER_IDENTITY_KEY} must be an object"
                )
                continue
            if IDENTITY_SCOPE_KEY in row_identity:
                errors.append(
                    f"{row_label}.{RESOLVED_SOLVER_IDENTITY_KEY} must be a flat "
                    "runtime identity, not another scope marker"
                )
                continue
            nested_errors = resolved_solver_identity_errors(
                {
                    "schema_version": version,
                    RESOLVED_SOLVER_IDENTITY_KEY: row_identity,
                },
                row_label,
            )
            errors.extend(nested_errors)
            if not nested_errors:
                signature = tuple(
                    row_identity.get(key)
                    for key in (
                        "rigid_contact_solver",
                        "rigid_point_joint_solver",
                        RIGID_CONTACT_SELECTION_KEY,
                        MULTIBODY_INTEGRATION_FAMILY_KEY,
                    )
                )
                signatures.add(signature)
                benchmark_id = row.get("benchmark") or row.get("run_name")
                if not isinstance(benchmark_id, str) or not benchmark_id:
                    errors.append(
                        f"{row_label} must identify its benchmark with "
                        "benchmark or run_name"
                    )
                    continue
                previous = identity_by_benchmark.setdefault(benchmark_id, signature)
                if previous != signature:
                    errors.append(
                        f"{row_label}.{RESOLVED_SOLVER_IDENTITY_KEY} conflicts "
                        f"with another row for benchmark {benchmark_id!r}"
                    )
    if collections_found == 0:
        errors.append(
            f"{packet_name}: a {PER_BENCHMARK_ROW_IDENTITY_SCOPE!r} identity "
            "requires benchmark.rows or benchmark.scale_data"
        )
    if collections_found > 0 and len(signatures) < 2 and not errors:
        errors.append(
            f"{packet_name}: {PER_BENCHMARK_ROW_IDENTITY_SCOPE!r} requires at "
            "least two distinct validated row solver identities"
        )
    return errors


def resolved_solver_identity_errors(
    packet: Mapping[str, Any], packet_name: str
) -> list[str]:
    """Validate the resolved-solver-identity contract for one packet.

    Packets at SOLVER_IDENTITY_MIN_SCHEMA_VERSION or newer must carry the
    identity object; older packets may omit it, but a present identity is
    validated at any version.
    """
    errors = packet_schema_version_errors(packet, packet_name)
    if errors:
        return errors
    version = packet["schema_version"]

    identity = packet.get(RESOLVED_SOLVER_IDENTITY_KEY)
    if identity is None:
        if version >= SOLVER_IDENTITY_MIN_SCHEMA_VERSION:
            return [
                f"{packet_name}: schema_version {version} requires "
                f"{RESOLVED_SOLVER_IDENTITY_KEY}"
            ]
        return []
    if not isinstance(identity, Mapping):
        return [f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY} must be an object"]

    if IDENTITY_SCOPE_KEY in identity:
        return _per_benchmark_row_solver_identity_errors(packet, packet_name, identity)

    for field, allowed in _REQUIRED_ENUM_FIELDS:
        value = identity.get(field)
        if not isinstance(value, str) or not value:
            errors.append(
                f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}.{field} "
                "must be a non-empty string"
            )
        elif value not in allowed:
            errors.append(
                f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}.{field} "
                f"must be one of {sorted(allowed)}, got {value!r}"
            )

    multibody_family = identity.get(MULTIBODY_INTEGRATION_FAMILY_KEY)
    if multibody_family is None:
        if version >= MULTIBODY_IDENTITY_MIN_SCHEMA_VERSION:
            errors.append(
                f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
                f"{MULTIBODY_INTEGRATION_FAMILY_KEY} is required at "
                f"schema_version {version}"
            )
    elif (
        not isinstance(multibody_family, str)
        or multibody_family not in ALLOWED_MULTIBODY_INTEGRATION_FAMILIES
    ):
        errors.append(
            f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
            f"{MULTIBODY_INTEGRATION_FAMILY_KEY} must be one of "
            f"{sorted(ALLOWED_MULTIBODY_INTEGRATION_FAMILIES)}, got "
            f"{multibody_family!r}"
        )

    emplaced = identity.get("avbd_rigid_contact_config_emplaced")
    if not isinstance(emplaced, bool):
        errors.append(
            f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
            "avbd_rigid_contact_config_emplaced must be a boolean"
        )

    selection = identity.get(RIGID_CONTACT_SELECTION_KEY)
    selection_required = version >= RIGID_CONTACT_SELECTION_MIN_SCHEMA_VERSION
    if selection is None:
        if selection_required:
            errors.append(
                f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
                f"{RIGID_CONTACT_SELECTION_KEY} is required at schema_version "
                f"{version}"
            )
    elif not isinstance(selection, str) or selection not in (
        ALLOWED_RIGID_CONTACT_SELECTIONS
    ):
        errors.append(
            f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
            f"{RIGID_CONTACT_SELECTION_KEY} must be one of "
            f"{sorted(ALLOWED_RIGID_CONTACT_SELECTIONS)}, got {selection!r}"
        )

    recorded_from = identity.get("recorded_from")
    if not isinstance(recorded_from, str) or not recorded_from:
        errors.append(
            f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}.recorded_from "
            "must be a non-empty string naming how the identity was captured"
        )

    rigid_contact_solver = identity.get("rigid_contact_solver")
    rigid_point_joint_solver = identity.get("rigid_point_joint_solver")
    if version < RIGID_CONTACT_SELECTION_MIN_SCHEMA_VERSION and (
        rigid_contact_solver == "vbd" or rigid_point_joint_solver == "vbd"
    ):
        errors.append(
            f"{packet_name}: VBD solver identities require schema_version "
            f"{RIGID_CONTACT_SELECTION_MIN_SCHEMA_VERSION} or newer"
        )
    if (
        version < RIGID_CONTACT_SELECTION_MIN_SCHEMA_VERSION
        and rigid_point_joint_solver == "sequential_impulse"
    ):
        errors.append(
            f"{packet_name}: Sequential Impulse point-joint solver identities "
            "require schema_version "
            f"{RIGID_CONTACT_SELECTION_MIN_SCHEMA_VERSION} or newer"
        )
    if selection is None:
        if (
            isinstance(emplaced, bool)
            and not emplaced
            and rigid_contact_solver == "avbd"
        ):
            errors.append(
                f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
                "rigid_contact_solver cannot be 'avbd' when "
                "avbd_rigid_contact_config_emplaced is false"
            )
    elif selection == "body_opt_in":
        if rigid_contact_solver != "avbd" or emplaced is not True:
            errors.append(
                f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
                "rigid_contact_selection 'body_opt_in' requires "
                "rigid_contact_solver 'avbd' and an emplaced private config"
            )
        if identity.get("rigid_point_joint_solver") == "vbd":
            errors.append(
                f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
                "rigid_point_joint_solver 'vbd' requires "
                "rigid_contact_selection 'world_solver_family'"
            )
    elif selection == "world_solver_family":
        if rigid_contact_solver not in ("avbd", "vbd"):
            errors.append(
                f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
                "rigid_contact_selection 'world_solver_family' requires "
                "rigid_contact_solver 'avbd' or 'vbd'"
            )
        if rigid_point_joint_solver not in ("none", rigid_contact_solver):
            errors.append(
                f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
                "rigid_contact_selection 'world_solver_family' requires "
                "rigid_point_joint_solver to be 'none' or match "
                "rigid_contact_solver"
            )
    elif selection == "contact_solver_method":
        if (
            rigid_contact_solver not in ("boxed_lcp", "sequential_impulse")
            or emplaced is not False
        ):
            errors.append(
                f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
                "rigid_contact_selection 'contact_solver_method' requires "
                "a boxed_lcp or sequential_impulse contact solver without "
                "a private AVBD body config"
            )
        if version >= CONTACT_METHOD_POINT_JOINT_MIN_SCHEMA_VERSION:
            if rigid_point_joint_solver not in ("none", "sequential_impulse"):
                errors.append(
                    f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
                    "rigid_contact_selection 'contact_solver_method' requires "
                    "rigid_point_joint_solver 'none' or 'sequential_impulse' "
                    "from schema_version "
                    f"{CONTACT_METHOD_POINT_JOINT_MIN_SCHEMA_VERSION} onward"
                )
        elif rigid_point_joint_solver == "vbd":
            errors.append(
                f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
                "rigid_contact_selection 'contact_solver_method' cannot carry "
                "rigid_point_joint_solver 'vbd'"
            )
    elif selection == "not_applicable" and rigid_contact_solver != "none":
        errors.append(
            f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
            "rigid_contact_selection 'not_applicable' requires "
            "rigid_contact_solver 'none'"
        )

    return errors


# --- Deriving the packet identity from the resolved-configuration report -----
#
# PLAN-091 WP-091.11 slice 4: the C++ resolved-configuration report
# (``dart::simulation::World::recordResolvedConfiguration``, slices 1-2) is the
# engine's own account of which solver family resolved each domain. These
# mappings translate the report's rigid-contact family strings onto the
# packet's ``rigid_contact_solver`` enum so the packet identity is *derived
# from the report's contract* through one place, rather than each writer
# hand-typing the enum. The report-family strings below mirror the C++ report;
# ``tests/test_world_resolved_configuration.cpp`` pins them on the engine side.

# Report rigid-contact ``resolved`` family strings → packet enum values.
_REPORT_RIGID_CONTACT_FAMILY_TO_PACKET = {
    "avbd": "avbd",
    "sequential-impulse": "sequential_impulse",
    "boxed-lcp": "boxed_lcp",
    "vbd": "vbd",
}

# A non-AVBD public family may still report this exact suffix when a body
# carries the compatibility-only internal AVBD rigid-contact opt-in.
_REPORT_AVBD_CONTACT_OPT_IN_SUFFIX = " + avbd (opt-in)"

_RUNTIME_IDENTITY_COUNTERS = (
    "runtime_identity_recorded",
    "runtime_identity_applicable",
    "runtime_identity_not_applicable",
    "runtime_identity_public_avbd_rigid",
    "runtime_identity_variational_multibody",
    "runtime_identity_contract_passed",
)


def _runtime_counter_errors(
    row: Mapping[str, Any], expected: Mapping[str, float], label: str
) -> list[str]:
    errors: list[str] = []
    for key, expected_value in expected.items():
        value = row.get(key)
        if (
            not isinstance(value, (int, float))
            or isinstance(value, bool)
            or not math.isfinite(value)
        ):
            errors.append(f"{label}: benchmark counter {key} must be finite")
        elif float(value) != expected_value:
            errors.append(
                f"{label}: benchmark counter {key} must be "
                f"{expected_value:g}, got {value!r}"
            )
    return errors


def make_resolved_solver_identity_from_benchmark_row(
    row: Mapping[str, Any], *, recorded_from: str
) -> dict[str, Any]:
    """Derive a flat solver identity from one benchmark runtime-counter row.

    The benchmark fixture records mutually exclusive public-rigid-AVBD and
    Variational-multibody contracts after ``World::enterSimulationMode``.
    Writers call this on a representative measured/median row instead of
    inferring identity from a historical ``BM_Avbd*`` symbol.
    """
    if not isinstance(row, Mapping):
        raise ValueError("benchmark runtime identity row must be an object")
    label = str(row.get("run_name") or row.get("name") or "benchmark row")
    errors = _runtime_counter_errors(
        row,
        {
            "runtime_identity_recorded": 1.0,
            "runtime_identity_applicable": 1.0,
            "runtime_identity_not_applicable": 0.0,
            "runtime_identity_contract_passed": 1.0,
        },
        label,
    )
    category_values: dict[str, float] = {}
    for key in _RUNTIME_IDENTITY_COUNTERS[-3:-1]:
        value = row.get(key)
        if (
            not isinstance(value, (int, float))
            or isinstance(value, bool)
            or not math.isfinite(value)
            or float(value) not in (0.0, 1.0)
        ):
            errors.append(f"{label}: benchmark counter {key} must be 0 or 1")
        else:
            category_values[key] = float(value)
    if sum(category_values.values()) != 1.0:
        errors.append(
            f"{label}: exactly one runtime solver identity category must be active"
        )
    if errors:
        raise ValueError("; ".join(errors))

    if category_values["runtime_identity_public_avbd_rigid"] == 1.0:
        errors = _runtime_counter_errors(
            row,
            {
                "public_avbd_family": 1.0,
                "resolved_rigid_body_avbd": 1.0,
                "resolved_rigid_contact_avbd": 1.0,
                "resolved_multibody_variational": 0.0,
            },
            label,
        )
        pair_avbd = row.get("resolved_rigid_pair_constraint_avbd")
        pair_inactive = row.get("resolved_rigid_pair_constraint_not_applicable")
        pair_errors = _runtime_counter_errors(
            row,
            {
                "resolved_rigid_pair_constraint_avbd": (
                    1.0 if pair_avbd == 1.0 else 0.0
                ),
                "resolved_rigid_pair_constraint_not_applicable": (
                    1.0 if pair_inactive == 1.0 else 0.0
                ),
            },
            label,
        )
        errors.extend(pair_errors)
        if pair_avbd not in (0, 0.0, 1, 1.0) or pair_inactive not in (
            0,
            0.0,
            1,
            1.0,
        ):
            pass
        elif float(pair_avbd) + float(pair_inactive) != 1.0:
            errors.append(
                f"{label}: exactly one AVBD pair-row applicability counter "
                "must be active"
            )
        if errors:
            raise ValueError("; ".join(errors))
        return make_resolved_solver_identity(
            resolved_rigid_contact_family="avbd",
            rigid_point_joint_solver="avbd" if pair_avbd == 1.0 else "none",
            avbd_rigid_contact_config_emplaced=False,
            recorded_from=recorded_from,
            rigid_contact_selection="world_solver_family",
            multibody_integration_family="none",
        )

    errors = _runtime_counter_errors(
        row,
        {
            "public_avbd_family": 0.0,
            "public_sequential_impulse_family": 1.0,
            "resolved_rigid_body_avbd": 0.0,
            "resolved_rigid_contact_avbd": 0.0,
            "resolved_rigid_body_sequential_impulse": 1.0,
            "resolved_rigid_contact_sequential_impulse": 1.0,
            "configured_multibody_variational": 1.0,
            "resolved_multibody_variational": 1.0,
        },
        label,
    )
    pair_si = row.get("resolved_rigid_pair_constraint_sequential_impulse")
    pair_inactive = row.get("resolved_rigid_pair_constraint_not_applicable")
    errors.extend(
        _runtime_counter_errors(
            row,
            {
                "resolved_rigid_pair_constraint_sequential_impulse": (
                    1.0 if pair_si == 1.0 else 0.0
                ),
                "resolved_rigid_pair_constraint_not_applicable": (
                    1.0 if pair_inactive == 1.0 else 0.0
                ),
            },
            label,
        )
    )
    if pair_si not in (0, 0.0, 1, 1.0) or pair_inactive not in (
        0,
        0.0,
        1,
        1.0,
    ):
        pass
    elif float(pair_si) + float(pair_inactive) != 1.0:
        errors.append(
            f"{label}: exactly one Sequential Impulse pair-row applicability "
            "counter must be active"
        )
    if errors:
        raise ValueError("; ".join(errors))
    return make_resolved_solver_identity(
        resolved_rigid_contact_family="sequential-impulse",
        rigid_point_joint_solver=("sequential_impulse" if pair_si == 1.0 else "none"),
        avbd_rigid_contact_config_emplaced=False,
        recorded_from=recorded_from,
        multibody_integration_family="variational",
    )


def make_per_benchmark_row_solver_identity(*, recorded_from: str) -> dict[str, str]:
    """Build the root marker for a heterogeneous benchmark packet."""
    if not isinstance(recorded_from, str) or not recorded_from:
        raise ValueError("per-benchmark-row identity source must be non-empty")
    return {
        IDENTITY_SCOPE_KEY: PER_BENCHMARK_ROW_IDENTITY_SCOPE,
        "recorded_from": recorded_from,
    }


def resolved_rigid_contact_solver_from_report(resolved_family: str) -> str:
    """Map a report rigid-contact ``resolved`` family string to the packet enum.

    ``resolved_family`` is the string the C++ report records for the
    ``rigid-contact`` domain (e.g. ``"avbd"``, ``"sequential-impulse"``,
    ``"boxed-lcp"``, or a ``"... + avbd (opt-in)"`` substitution). Raises
    ``ValueError`` for an empty or unrecognized family so a report-vocabulary
    change cannot silently produce an out-of-contract packet enum.
    """
    text = resolved_family.strip().lower()
    if not text:
        raise ValueError("resolved rigid-contact family must be non-empty")
    if text.endswith(_REPORT_AVBD_CONTACT_OPT_IN_SUFFIX):
        base_family = text[: -len(_REPORT_AVBD_CONTACT_OPT_IN_SUFFIX)]
        if base_family not in ("boxed-lcp", "sequential-impulse"):
            raise ValueError(
                "unrecognized AVBD opt-in base rigid-contact family "
                f"{base_family!r}; expected 'boxed-lcp' or "
                "'sequential-impulse'"
            )
        return "avbd"
    try:
        return _REPORT_RIGID_CONTACT_FAMILY_TO_PACKET[text]
    except KeyError:
        raise ValueError(
            f"unrecognized resolved rigid-contact family {resolved_family!r}; "
            f"known: {sorted(_REPORT_RIGID_CONTACT_FAMILY_TO_PACKET)} "
            f"(plus an '{_REPORT_AVBD_CONTACT_OPT_IN_SUFFIX}' suffix)"
        ) from None


def make_resolved_solver_identity(
    *,
    resolved_rigid_contact_family: str | None,
    rigid_point_joint_solver: str,
    avbd_rigid_contact_config_emplaced: bool,
    recorded_from: str,
    rigid_contact_selection: str | None = None,
    multibody_integration_family: str = "none",
) -> dict[str, Any]:
    """Build a validated ``resolved_solver_identity`` object from the report.

    ``resolved_rigid_contact_family`` is the report's rigid-contact family
    string for a scene that resolves rigid-rigid contacts, or ``None`` for a
    contact-free scene (recorded as ``"none"``). The returned object is
    validated against the schema contract before it is handed back, so a writer
    cannot emit an identity the checker would reject.
    """
    if resolved_rigid_contact_family is None:
        rigid_contact_solver = "none"
    else:
        rigid_contact_solver = resolved_rigid_contact_solver_from_report(
            resolved_rigid_contact_family
        )

    if rigid_contact_selection is None:
        if rigid_contact_solver == "none":
            rigid_contact_selection = "not_applicable"
        elif rigid_contact_solver in ("boxed_lcp", "sequential_impulse"):
            rigid_contact_selection = "contact_solver_method"
        elif avbd_rigid_contact_config_emplaced:
            rigid_contact_selection = "body_opt_in"

    identity: dict[str, Any] = {
        "avbd_rigid_contact_config_emplaced": bool(avbd_rigid_contact_config_emplaced),
        MULTIBODY_INTEGRATION_FAMILY_KEY: multibody_integration_family,
        "recorded_from": recorded_from,
        RIGID_CONTACT_SELECTION_KEY: rigid_contact_selection,
        "rigid_contact_solver": rigid_contact_solver,
        "rigid_point_joint_solver": rigid_point_joint_solver,
    }

    errors = resolved_solver_identity_errors(
        {
            "schema_version": AVBD_PACKET_SCHEMA_VERSION,
            RESOLVED_SOLVER_IDENTITY_KEY: identity,
        },
        "make_resolved_solver_identity",
    )
    if errors:
        raise ValueError("; ".join(errors))
    return identity
