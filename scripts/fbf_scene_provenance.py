#!/usr/bin/env python3
"""Canonical semantic-physics provenance for FBF demo contracts.

The FBF demos currently bind every scene contract to the same monolithic
``FbfPaperFrictionScene.cpp`` source hash.  That broad binding is useful for
auditing an exact binary, but unrelated presentation edits make it unsuitable
as the identity of a scene's physics contract.  This module separates those
two identities without weakening the contract: the semantic projection is a
deep copy of the complete contract with exactly that one broad implementation
hash removed.

Only the current, explicitly listed contract schemas are accepted.  Adding or
moving an implementation identity is intentionally fail-closed so a schema
change cannot silently widen what the semantic digest omits.
"""

from __future__ import annotations

import copy
import dataclasses
import hashlib
import json
import math
import re
from typing import Any

SEMANTIC_PROVENANCE_SCHEMA_VERSION = "dart.fbf_semantic_physics_provenance/v1"

_SHA256_PATTERN = re.compile(r"[0-9a-f]{64}\Z")
_IMPLEMENTATION_IDENTITY_KEYS = frozenset(
    {
        "implementation_sha256",
        "demo_implementation_sha256",
        "implementation_source_sha256",
    }
)


class SemanticProvenanceError(ValueError):
    """Raised when an FBF contract cannot be projected unambiguously."""


@dataclasses.dataclass(frozen=True)
class SemanticPhysicsProvenance:
    """Separated semantic and broad-source identities for one FBF contract."""

    schema_version: str
    family: str
    projection: dict[str, Any]
    canonical_json: str
    semantic_sha256: str
    broad_implementation_sha256: str


@dataclasses.dataclass(frozen=True)
class _SchemaPolicy:
    family: str
    kind: str
    identity_container: str
    identity_key: str
    identity_container_keys: frozenset[str]


_LITERAL_BINDING_KEYS = frozenset(
    {
        "spec_sha256",
        "implementation_sha256",
        "geometry_sha256",
        "solver_options_sha256",
    }
)
_CARD_HOUSE_BINDING_KEYS = frozenset(
    {
        "repository",
        "commit",
        "tree",
        "card_house_run_blob",
        "card_house_run_py_sha256",
        "fbf_config_py_sha256",
        "solver_fbf_py_sha256",
        "configuration_spec_sha256",
        "demo_implementation_sha256",
    }
)
_BACKSPIN_BINDING_KEYS = frozenset(
    {
        "repository",
        "commit",
        "tree",
        "runner_path",
        "runner_blob",
        "runner_sha256",
        "source_run_id",
        "sweep_results_sha256",
        "metadata_json_sha256",
        "result_json_sha256",
        "trajectory_npz_sha256",
        "history_json_sha256",
        "configuration_spec_sha256",
        "exact_solver_options_sha256",
        "demo_implementation_sha256",
    }
)
_INCLINE_BINDING_KEYS = frozenset(
    {
        "repository",
        "commit",
        "tree",
        "runner_path",
        "runner_blob",
        "runner_sha256",
        "configuration_spec_sha256",
        "exact_solver_options_sha256",
        "demo_implementation_sha256",
    }
)
_PAINLEVE_BINDING_KEYS = frozenset(
    {
        "repository",
        "commit",
        "tree",
        "painleve_run_blob",
        "painleve_run_py_sha256",
        "configuration_spec_sha256",
        "exact_solver_options_sha256",
        "demo_implementation_sha256",
    }
)
_AUTHOR_MASONRY_BINDING_KEYS = frozenset(
    {
        "repository",
        "commit",
        "tree",
        "run_py_blob",
        "run_py_sha256",
        "mesh_tree",
        "mesh_tree_sha256",
        "mesh_directory",
        "configuration_spec_sha256",
        "dart_adapter_sha256",
        "demo_implementation_sha256",
    }
)
_TURNTABLE_BINARY_BINDING_KEYS = frozenset({"role", "implementation_source_sha256"})


def _source_binding_policy(
    family: str,
    keys: frozenset[str],
    *,
    kind: str = "source_configuration_dynamics_adapter",
) -> _SchemaPolicy:
    return _SchemaPolicy(
        family=family,
        kind=kind,
        identity_container="source_binding",
        identity_key="demo_implementation_sha256",
        identity_container_keys=keys,
    )


_SCHEMA_POLICIES = {
    "dart.fbf_literal_masonry_arch_physics_contract/v1": _SchemaPolicy(
        family="literal_masonry_arch",
        kind="physics_control",
        identity_container="source_binding",
        identity_key="implementation_sha256",
        identity_container_keys=_LITERAL_BINDING_KEYS,
    ),
    "dart.fbf_author_card_house_dynamics_adapter/v1": _source_binding_policy(
        "author_card_house", _CARD_HOUSE_BINDING_KEYS
    ),
    "dart.fbf_author_backspin_dynamics_adapter/v1": _source_binding_policy(
        "author_backspin",
        _BACKSPIN_BINDING_KEYS,
        kind="current_source_configuration_dynamics_adapter",
    ),
    "dart.fbf_author_incline_sweep_dynamics_adapter/v1": (
        _source_binding_policy(
            "author_incline",
            _INCLINE_BINDING_KEYS,
            kind="current_source_configuration_dynamics_adapter",
        )
    ),
    "dart.fbf_author_painleve_dynamics_adapter/v1": _source_binding_policy(
        "author_painleve", _PAINLEVE_BINDING_KEYS
    ),
    "dart.fbf_author_turntable_physics_contract/v1": _SchemaPolicy(
        family="author_turntable",
        kind="physics_control",
        identity_container="binary_binding",
        identity_key="implementation_source_sha256",
        identity_container_keys=_TURNTABLE_BINARY_BINDING_KEYS,
    ),
    "dart.fbf_author_card_house_configuration_contract/v1": _SchemaPolicy(
        family="author_card_house",
        kind="configuration_only",
        identity_container="binary_binding",
        identity_key="implementation_source_sha256",
        identity_container_keys=_TURNTABLE_BINARY_BINDING_KEYS,
    ),
    "dart.fbf_author_masonry_arch_crown_impact_dart_adapter/v1": (
        _source_binding_policy("author_masonry_arch", _AUTHOR_MASONRY_BINDING_KEYS)
    ),
    (
        "dart.fbf_author_masonry_arch_crown_impact_source_continuation_"
        "dart_adapter/v1"
    ): _source_binding_policy("author_masonry_arch", _AUTHOR_MASONRY_BINDING_KEYS),
    "dart.fbf_author_masonry_arch_standing_dart_adapter/v1": (
        _source_binding_policy("author_masonry_arch", _AUTHOR_MASONRY_BINDING_KEYS)
    ),
}


def _path_label(path: tuple[str | int, ...]) -> str:
    label = "$"
    for part in path:
        if isinstance(part, int):
            label += f"[{part}]"
        else:
            label += f".{part}"
    return label


def _validate_json_value(value: Any, path: tuple[str | int, ...] = ()) -> None:
    if value is None or isinstance(value, (bool, str, int)):
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise SemanticProvenanceError(
                f"{_path_label(path)} must be a finite JSON number"
            )
        return
    if isinstance(value, list):
        for index, item in enumerate(value):
            _validate_json_value(item, (*path, index))
        return
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise SemanticProvenanceError(
                    f"{_path_label(path)} has a non-string object key"
                )
            if key.endswith("sha256") and not (
                isinstance(item, str) and _SHA256_PATTERN.fullmatch(item)
            ):
                raise SemanticProvenanceError(
                    f"{_path_label((*path, key))} must be a lowercase SHA-256"
                )
            _validate_json_value(item, (*path, key))
        return
    raise SemanticProvenanceError(
        f"{_path_label(path)} has unsupported JSON value type "
        f"{type(value).__name__}"
    )


def _implementation_identity_paths(
    value: Any, path: tuple[str | int, ...] = ()
) -> list[tuple[str | int, ...]]:
    paths: list[tuple[str | int, ...]] = []
    if isinstance(value, dict):
        for key, item in value.items():
            child_path = (*path, key)
            if key in _IMPLEMENTATION_IDENTITY_KEYS:
                paths.append(child_path)
            paths.extend(_implementation_identity_paths(item, child_path))
    elif isinstance(value, list):
        for index, item in enumerate(value):
            paths.extend(_implementation_identity_paths(item, (*path, index)))
    return paths


def build_semantic_physics_provenance(
    contract: dict[str, Any],
) -> SemanticPhysicsProvenance:
    """Return a fail-closed semantic projection and its SHA-256 digest.

    The returned projection retains every contract member except the one
    schema-declared monolithic implementation-source hash.  ``contract`` is
    never mutated.
    """

    if not isinstance(contract, dict):
        raise SemanticProvenanceError("FBF physics contract must be a JSON object")

    _validate_json_value(contract)
    schema_version = contract.get("schema_version")
    if not isinstance(schema_version, str):
        raise SemanticProvenanceError("$.schema_version must be a string")
    policy = _SCHEMA_POLICIES.get(schema_version)
    if policy is None:
        raise SemanticProvenanceError(
            f"unsupported FBF physics contract schema {schema_version!r}"
        )
    if contract.get("kind") != policy.kind:
        raise SemanticProvenanceError(
            f"{schema_version}: expected contract kind {policy.kind!r}"
        )

    container = contract.get(policy.identity_container)
    if not isinstance(container, dict):
        raise SemanticProvenanceError(
            f"$.{policy.identity_container} must be a JSON object"
        )
    actual_container_keys = frozenset(container)
    if actual_container_keys != policy.identity_container_keys:
        missing = sorted(policy.identity_container_keys - actual_container_keys)
        unexpected = sorted(actual_container_keys - policy.identity_container_keys)
        raise SemanticProvenanceError(
            f"$.{policy.identity_container} has an unexpected shape; "
            f"missing={missing}, unexpected={unexpected}"
        )
    if any(not isinstance(value, str) or not value for value in container.values()):
        raise SemanticProvenanceError(
            f"$.{policy.identity_container} values must be nonempty strings"
        )

    expected_identity_path = (
        policy.identity_container,
        policy.identity_key,
    )
    observed_identity_paths = _implementation_identity_paths(contract)
    if observed_identity_paths != [expected_identity_path]:
        observed = [_path_label(path) for path in observed_identity_paths]
        raise SemanticProvenanceError(
            "contract implementation identity is ambiguous; "
            f"expected={_path_label(expected_identity_path)}, observed={observed}"
        )

    broad_implementation_sha256 = container[policy.identity_key]
    projection = copy.deepcopy(contract)
    del projection[policy.identity_container][policy.identity_key]
    canonical_json = json.dumps(
        projection,
        allow_nan=False,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    )
    semantic_sha256 = hashlib.sha256(canonical_json.encode("utf-8")).hexdigest()
    return SemanticPhysicsProvenance(
        schema_version=schema_version,
        family=policy.family,
        projection=projection,
        canonical_json=canonical_json,
        semantic_sha256=semantic_sha256,
        broad_implementation_sha256=broad_implementation_sha256,
    )


def semantic_physics_projection(contract: dict[str, Any]) -> dict[str, Any]:
    """Return the complete semantic projection for ``contract``."""

    return build_semantic_physics_provenance(contract).projection


def semantic_physics_sha256(contract: dict[str, Any]) -> str:
    """Return the canonical semantic-physics SHA-256 for ``contract``."""

    return build_semantic_physics_provenance(contract).semantic_sha256
