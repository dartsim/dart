import copy
import hashlib
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "scripts"))

from fbf_scene_provenance import (  # noqa: E402
    SemanticProvenanceError,
    build_semantic_physics_provenance,
    semantic_physics_projection,
    semantic_physics_sha256,
)

SCHEMA_CASES = {
    "dart.fbf_literal_masonry_arch_physics_contract/v1": (
        "physics_control",
        "source_binding",
        "implementation_sha256",
        {
            "spec_sha256",
            "implementation_sha256",
            "geometry_sha256",
            "solver_options_sha256",
        },
    ),
    "dart.fbf_author_card_house_dynamics_adapter/v1": (
        "source_configuration_dynamics_adapter",
        "source_binding",
        "demo_implementation_sha256",
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
        },
    ),
    "dart.fbf_author_backspin_dynamics_adapter/v1": (
        "current_source_configuration_dynamics_adapter",
        "source_binding",
        "demo_implementation_sha256",
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
        },
    ),
    "dart.fbf_author_incline_sweep_dynamics_adapter/v1": (
        "current_source_configuration_dynamics_adapter",
        "source_binding",
        "demo_implementation_sha256",
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
        },
    ),
    "dart.fbf_author_painleve_dynamics_adapter/v1": (
        "source_configuration_dynamics_adapter",
        "source_binding",
        "demo_implementation_sha256",
        {
            "repository",
            "commit",
            "tree",
            "painleve_run_blob",
            "painleve_run_py_sha256",
            "configuration_spec_sha256",
            "exact_solver_options_sha256",
            "demo_implementation_sha256",
        },
    ),
    "dart.fbf_author_turntable_physics_contract/v1": (
        "physics_control",
        "binary_binding",
        "implementation_source_sha256",
        {"role", "implementation_source_sha256"},
    ),
    "dart.fbf_author_card_house_configuration_contract/v1": (
        "configuration_only",
        "binary_binding",
        "implementation_source_sha256",
        {"role", "implementation_source_sha256"},
    ),
    "dart.fbf_author_masonry_arch_crown_impact_dart_adapter/v1": (
        "source_configuration_dynamics_adapter",
        "source_binding",
        "demo_implementation_sha256",
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
        },
    ),
    (
        "dart.fbf_author_masonry_arch_crown_impact_source_continuation_"
        "dart_adapter/v1"
    ): (
        "source_configuration_dynamics_adapter",
        "source_binding",
        "demo_implementation_sha256",
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
        },
    ),
    "dart.fbf_author_masonry_arch_standing_dart_adapter/v1": (
        "source_configuration_dynamics_adapter",
        "source_binding",
        "demo_implementation_sha256",
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
        },
    ),
}


def _value_for_binding_key(key):
    if key.endswith("sha256"):
        return hashlib.sha256(key.encode("utf-8")).hexdigest()
    return f"pinned:{key}"


def _contract(schema_version):
    kind, container_name, _, container_keys = SCHEMA_CASES[schema_version]
    contract = {
        "schema_version": schema_version,
        "kind": kind,
        container_name: {key: _value_for_binding_key(key) for key in container_keys},
        "scenario": {
            "id": f"scenario:{schema_version}",
            "friction": 0.8,
            "scheduled_actions": [1600, "p"],
        },
        "solver": {
            "lane": "exact_fbf",
            "tolerance": 1e-6,
            "fallback_to_boxed_lcp_enabled": False,
        },
        "claim_boundary": {
            "trajectory_equivalent": False,
            "paper_parity": False,
        },
    }
    if container_name == "binary_binding":
        contract["author_source"] = {"commit": "a" * 40}
        contract["physics_spec_source_sha256"] = "b" * 64
    return contract


def _reverse_object_order(value):
    if isinstance(value, dict):
        return {
            key: _reverse_object_order(item)
            for key, item in reversed(tuple(value.items()))
        }
    if isinstance(value, list):
        return [_reverse_object_order(item) for item in value]
    return value


@pytest.mark.parametrize("schema_version", SCHEMA_CASES)
def test_all_current_contract_schemas_have_one_broad_identity(schema_version):
    contract = _contract(schema_version)
    _, container_name, identity_key, _ = SCHEMA_CASES[schema_version]

    result = build_semantic_physics_provenance(contract)

    assert result.schema_version == schema_version
    assert result.broad_implementation_sha256 == contract[container_name][identity_key]
    assert identity_key not in result.projection[container_name]
    assert result.projection["scenario"] == contract["scenario"]
    assert result.projection["solver"] == contract["solver"]
    assert result.projection["claim_boundary"] == contract["claim_boundary"]
    for key, value in contract[container_name].items():
        if key != identity_key:
            assert result.projection[container_name][key] == value


def test_semantic_digest_is_independent_of_object_key_order():
    contract = _contract("dart.fbf_author_backspin_dynamics_adapter/v1")

    reordered = _reverse_object_order(contract)

    assert semantic_physics_sha256(reordered) == semantic_physics_sha256(contract)
    assert build_semantic_physics_provenance(reordered).canonical_json == (
        build_semantic_physics_provenance(contract).canonical_json
    )


def test_family_and_solver_lane_remain_part_of_semantic_identity():
    literal = _contract("dart.fbf_literal_masonry_arch_physics_contract/v1")
    card_house = _contract("dart.fbf_author_card_house_dynamics_adapter/v1")
    boxed_card_house = copy.deepcopy(card_house)
    boxed_card_house["solver"]["lane"] = "boxed_lcp"

    literal_result = build_semantic_physics_provenance(literal)
    card_result = build_semantic_physics_provenance(card_house)

    assert literal_result.family == "literal_masonry_arch"
    assert card_result.family == "author_card_house"
    assert literal_result.semantic_sha256 != card_result.semantic_sha256
    assert semantic_physics_sha256(boxed_card_house) != card_result.semantic_sha256


def test_physics_mutation_changes_semantic_digest():
    contract = _contract("dart.fbf_author_incline_sweep_dynamics_adapter/v1")
    changed = copy.deepcopy(contract)
    changed["scenario"]["friction"] = 0.81

    assert semantic_physics_sha256(changed) != semantic_physics_sha256(contract)


def test_only_broad_implementation_hash_is_removed():
    contract = _contract("dart.fbf_author_painleve_dynamics_adapter/v1")
    changed = copy.deepcopy(contract)
    changed["source_binding"]["demo_implementation_sha256"] = "f" * 64

    original = build_semantic_physics_provenance(contract)
    rebound = build_semantic_physics_provenance(changed)

    assert rebound.projection == original.projection
    assert rebound.semantic_sha256 == original.semantic_sha256
    assert rebound.broad_implementation_sha256 == "f" * 64
    assert rebound.broad_implementation_sha256 != original.broad_implementation_sha256
    assert semantic_physics_projection(contract) == original.projection


def test_input_contract_is_not_mutated():
    contract = _contract("dart.fbf_author_card_house_dynamics_adapter/v1")
    original = copy.deepcopy(contract)

    result = build_semantic_physics_provenance(contract)
    result.projection["solver"]["lane"] = "mutated_result_only"

    assert contract == original


def test_unknown_schema_is_rejected():
    contract = _contract("dart.fbf_author_card_house_dynamics_adapter/v1")
    contract["schema_version"] = "dart.fbf_future_contract/v1"

    with pytest.raises(SemanticProvenanceError, match="unsupported.*schema"):
        build_semantic_physics_provenance(contract)


@pytest.mark.parametrize("malformed", [None, [], "source_binding"])
def test_malformed_source_binding_is_rejected(malformed):
    contract = _contract("dart.fbf_author_backspin_dynamics_adapter/v1")
    contract["source_binding"] = malformed

    with pytest.raises(SemanticProvenanceError, match="source_binding"):
        build_semantic_physics_provenance(contract)


def test_changed_source_binding_shape_is_rejected():
    contract = _contract("dart.fbf_author_card_house_dynamics_adapter/v1")
    del contract["source_binding"]["solver_fbf_py_sha256"]
    contract["source_binding"]["unreviewed_hash"] = "a" * 64

    with pytest.raises(SemanticProvenanceError, match="unexpected shape"):
        build_semantic_physics_provenance(contract)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_non_finite_numbers_are_rejected(value):
    contract = _contract("dart.fbf_author_incline_sweep_dynamics_adapter/v1")
    contract["solver"]["tolerance"] = value

    with pytest.raises(SemanticProvenanceError, match="finite JSON number"):
        build_semantic_physics_provenance(contract)


@pytest.mark.parametrize("value", [True, "not-a-hash", "A" * 64])
def test_malformed_broad_implementation_hash_is_rejected(value):
    contract = _contract("dart.fbf_author_painleve_dynamics_adapter/v1")
    contract["source_binding"]["demo_implementation_sha256"] = value

    with pytest.raises(SemanticProvenanceError, match="lowercase SHA-256"):
        build_semantic_physics_provenance(contract)


def test_unexpected_implementation_identity_path_is_rejected():
    contract = _contract("dart.fbf_author_painleve_dynamics_adapter/v1")
    contract["solver"]["implementation_sha256"] = "e" * 64

    with pytest.raises(SemanticProvenanceError, match="identity is ambiguous"):
        build_semantic_physics_provenance(contract)


def test_non_json_contract_value_is_rejected():
    contract = _contract("dart.fbf_author_backspin_dynamics_adapter/v1")
    contract["scenario"]["unsupported_tuple"] = (1, 2)

    with pytest.raises(SemanticProvenanceError, match="unsupported JSON value type"):
        build_semantic_physics_provenance(contract)
