"""Tests for scripts/avbd_packet_schema.py (PLAN-091 WP-091.11 slice 4).

These cover the report-derived ``resolved_solver_identity`` builder: it must
translate the C++
resolved-configuration report's rigid-contact family vocabulary onto the packet
enum through the single mapping in the schema module.
"""

import importlib.util
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "avbd_packet_schema.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("avbd_packet_schema", SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


schema = _load_module()


# Representative writer identities exercise the shared builder contract.
_WRITER_CASES = {
    "breakable_joint": (
        {
            "resolved_rigid_contact_family": None,
            "rigid_point_joint_solver": "avbd",
            "avbd_rigid_contact_config_emplaced": False,
            "recorded_from": "breakable joint scale benchmark row family",
        },
        {
            "avbd_rigid_contact_config_emplaced": False,
            "multibody_integration_family": "none",
            "recorded_from": "breakable joint scale benchmark row family",
            "rigid_contact_selection": "not_applicable",
            "rigid_contact_solver": "none",
            "rigid_point_joint_solver": "avbd",
        },
    ),
    "breakable_motor": (
        {
            "resolved_rigid_contact_family": None,
            "rigid_point_joint_solver": "avbd",
            "avbd_rigid_contact_config_emplaced": False,
            "recorded_from": "breakable motor scale benchmark row family",
        },
        {
            "avbd_rigid_contact_config_emplaced": False,
            "multibody_integration_family": "none",
            "recorded_from": "breakable motor scale benchmark row family",
            "rigid_contact_selection": "not_applicable",
            "rigid_contact_solver": "none",
            "rigid_point_joint_solver": "avbd",
        },
    ),
    "friction_sweep": (
        {
            "resolved_rigid_contact_family": "sequential-impulse",
            "rigid_point_joint_solver": "none",
            "avbd_rigid_contact_config_emplaced": False,
            "recorded_from": "friction coefficient sweep benchmark scene counters",
        },
        {
            "avbd_rigid_contact_config_emplaced": False,
            "multibody_integration_family": "none",
            "recorded_from": "friction coefficient sweep benchmark scene counters",
            "rigid_contact_selection": "contact_solver_method",
            "rigid_contact_solver": "sequential_impulse",
            "rigid_point_joint_solver": "none",
        },
    ),
    "paper_scale": (
        {
            "resolved_rigid_contact_family": None,
            "rigid_point_joint_solver": "avbd",
            "avbd_rigid_contact_config_emplaced": False,
            "recorded_from": "paper-scale high-ratio iteration benchmark row family",
        },
        {
            "avbd_rigid_contact_config_emplaced": False,
            "multibody_integration_family": "none",
            "recorded_from": "paper-scale high-ratio iteration benchmark row family",
            "rigid_contact_selection": "not_applicable",
            "rigid_contact_solver": "none",
            "rigid_point_joint_solver": "avbd",
        },
    ),
}


@pytest.mark.parametrize("name", sorted(_WRITER_CASES))
def test_builder_reproduces_committed_writer_identities(name):
    kwargs, expected = _WRITER_CASES[name]
    assert schema.make_resolved_solver_identity(**kwargs) == expected


def test_builder_output_passes_schema_contract():
    for kwargs, _ in _WRITER_CASES.values():
        identity = schema.make_resolved_solver_identity(**kwargs)
        packet = {
            "schema_version": schema.AVBD_PACKET_SCHEMA_VERSION,
            schema.RESOLVED_SOLVER_IDENTITY_KEY: identity,
        }
        assert schema.resolved_solver_identity_errors(packet, "case") == []


def test_current_flat_identity_requires_explicit_multibody_family():
    identity = schema.make_resolved_solver_identity(
        resolved_rigid_contact_family="avbd",
        rigid_point_joint_solver="none",
        avbd_rigid_contact_config_emplaced=False,
        recorded_from="runtime report",
        rigid_contact_selection="world_solver_family",
    )
    del identity["multibody_integration_family"]

    errors = schema.resolved_solver_identity_errors(
        {
            "schema_version": schema.AVBD_PACKET_SCHEMA_VERSION,
            schema.RESOLVED_SOLVER_IDENTITY_KEY: identity,
        },
        "case",
    )

    assert any("multibody_integration_family is required" in error for error in errors)


def test_pre_v5_identity_without_multibody_family_stays_readable():
    packet = {
        "schema_version": schema.MULTIBODY_IDENTITY_MIN_SCHEMA_VERSION - 1,
        schema.RESOLVED_SOLVER_IDENTITY_KEY: {
            "avbd_rigid_contact_config_emplaced": False,
            "recorded_from": "legacy runtime report",
            "rigid_contact_selection": "world_solver_family",
            "rigid_contact_solver": "avbd",
            "rigid_point_joint_solver": "avbd",
        },
    }

    assert schema.resolved_solver_identity_errors(packet, "case") == []


def test_unknown_multibody_family_is_rejected():
    identity = schema.make_resolved_solver_identity(
        resolved_rigid_contact_family=None,
        rigid_point_joint_solver="none",
        avbd_rigid_contact_config_emplaced=False,
        recorded_from="runtime report",
    )
    identity["multibody_integration_family"] = "avbd"

    errors = schema.resolved_solver_identity_errors(
        {
            "schema_version": schema.AVBD_PACKET_SCHEMA_VERSION,
            schema.RESOLVED_SOLVER_IDENTITY_KEY: identity,
        },
        "case",
    )

    assert any("must be one of" in error for error in errors), errors


def _runtime_identity_row(runtime_family, *, pair_rows):
    common = {
        "name": "BM_RuntimeIdentity/1_median",
        "runtime_identity_recorded": 1.0,
        "runtime_identity_applicable": 1.0,
        "runtime_identity_not_applicable": 0.0,
        "runtime_identity_contract_passed": 1.0,
    }
    if runtime_family == "public_avbd_rigid":
        return {
            **common,
            "runtime_identity_public_avbd_rigid": 1.0,
            "runtime_identity_variational_multibody": 0.0,
            "public_avbd_family": 1.0,
            "resolved_rigid_body_avbd": 1.0,
            "resolved_rigid_contact_avbd": 1.0,
            "resolved_rigid_pair_constraint_avbd": 1.0 if pair_rows else 0.0,
            "resolved_rigid_pair_constraint_not_applicable": (
                0.0 if pair_rows else 1.0
            ),
            "resolved_multibody_variational": 0.0,
        }
    return {
        **common,
        "runtime_identity_public_avbd_rigid": 0.0,
        "runtime_identity_variational_multibody": 1.0,
        "public_avbd_family": 0.0,
        "public_sequential_impulse_family": 1.0,
        "resolved_rigid_body_avbd": 0.0,
        "resolved_rigid_contact_avbd": 0.0,
        "resolved_rigid_body_sequential_impulse": 1.0,
        "resolved_rigid_contact_sequential_impulse": 1.0,
        "resolved_rigid_pair_constraint_sequential_impulse": (
            1.0 if pair_rows else 0.0
        ),
        "resolved_rigid_pair_constraint_not_applicable": (0.0 if pair_rows else 1.0),
        "configured_multibody_variational": 1.0,
        "resolved_multibody_variational": 1.0,
    }


def test_runtime_counters_derive_public_avbd_rigid_identity():
    identity = schema.make_resolved_solver_identity_from_benchmark_row(
        _runtime_identity_row("public_avbd_rigid", pair_rows=True),
        recorded_from="benchmark runtime counters",
    )

    assert identity["rigid_contact_solver"] == "avbd"
    assert identity["rigid_point_joint_solver"] == "avbd"
    assert identity["multibody_integration_family"] == "none"


def test_runtime_counters_derive_variational_multibody_identity():
    identity = schema.make_resolved_solver_identity_from_benchmark_row(
        _runtime_identity_row("variational_multibody", pair_rows=False),
        recorded_from="benchmark runtime counters",
    )

    assert identity["rigid_contact_solver"] == "sequential_impulse"
    assert identity["rigid_point_joint_solver"] == "none"
    assert identity["multibody_integration_family"] == "variational"


@pytest.mark.parametrize(
    ("field", "replacement"),
    (
        ("runtime_identity_recorded", 0.0),
        ("runtime_identity_contract_passed", 0.0),
        ("runtime_identity_variational_multibody", 1.0),
        ("resolved_rigid_contact_avbd", 0.0),
        ("resolved_rigid_pair_constraint_avbd", 0.0),
        ("runtime_identity_recorded", float("nan")),
    ),
)
def test_runtime_identity_counter_mutations_fail_closed(field, replacement):
    row = _runtime_identity_row("public_avbd_rigid", pair_rows=True)
    row[field] = replacement

    with pytest.raises(ValueError):
        schema.make_resolved_solver_identity_from_benchmark_row(
            row,
            recorded_from="benchmark runtime counters",
        )


def _heterogeneous_identity_packet(*, duplicate_identity=False):
    avbd = schema.make_resolved_solver_identity_from_benchmark_row(
        _runtime_identity_row("public_avbd_rigid", pair_rows=True),
        recorded_from="benchmark runtime counters",
    )
    variational = schema.make_resolved_solver_identity_from_benchmark_row(
        _runtime_identity_row("variational_multibody", pair_rows=False),
        recorded_from="benchmark runtime counters",
    )
    if duplicate_identity:
        variational = avbd
    return {
        "schema_version": schema.AVBD_PACKET_SCHEMA_VERSION,
        "resolved_solver_identity": schema.make_per_benchmark_row_solver_identity(
            recorded_from="benchmark runtime counters"
        ),
        "benchmark": {
            "rows": [
                {"benchmark": "BM_Rigid/1", "resolved_solver_identity": avbd},
                {
                    "benchmark": "BM_Articulated/1",
                    "resolved_solver_identity": variational,
                },
            ],
            "scale_data": [
                {"benchmark": "BM_Rigid/1", "resolved_solver_identity": avbd},
                {
                    "benchmark": "BM_Articulated/1",
                    "resolved_solver_identity": variational,
                },
            ],
        },
    }


def test_heterogeneous_packet_accepts_complete_per_row_identities():
    packet = _heterogeneous_identity_packet()
    assert schema.resolved_solver_identity_errors(packet, "case") == []


def test_heterogeneous_packet_rejects_missing_row_identity():
    packet = _heterogeneous_identity_packet()
    del packet["benchmark"]["rows"][1]["resolved_solver_identity"]

    errors = schema.resolved_solver_identity_errors(packet, "case")

    assert any(
        "benchmark.rows[1].resolved_solver_identity" in error for error in errors
    )


def test_homogeneous_packet_cannot_hide_behind_per_row_scope():
    packet = _heterogeneous_identity_packet(duplicate_identity=True)

    errors = schema.resolved_solver_identity_errors(packet, "case")

    assert any("at least two distinct" in error for error in errors), errors


def test_heterogeneous_packet_rejects_conflicting_identity_for_same_benchmark():
    packet = _heterogeneous_identity_packet()
    packet["benchmark"]["scale_data"][1]["benchmark"] = "BM_Rigid/1"

    errors = schema.resolved_solver_identity_errors(packet, "case")

    assert any("conflicts with another row" in error for error in errors), errors


def test_contact_solver_method_rejects_private_avbd_body_config():
    identity = schema.make_resolved_solver_identity(
        resolved_rigid_contact_family="sequential-impulse",
        rigid_point_joint_solver="none",
        avbd_rigid_contact_config_emplaced=False,
        recorded_from="resolved report",
    )
    identity["avbd_rigid_contact_config_emplaced"] = True
    packet = {
        "schema_version": schema.AVBD_PACKET_SCHEMA_VERSION,
        schema.RESOLVED_SOLVER_IDENTITY_KEY: identity,
    }

    errors = schema.resolved_solver_identity_errors(packet, "case")

    assert any("without a private AVBD body config" in error for error in errors)


@pytest.mark.parametrize(
    ("report_family", "expected"),
    [
        ("sequential-impulse", "sequential_impulse"),
        ("boxed-lcp", "boxed_lcp"),
        ("avbd", "avbd"),
        ("sequential-impulse + avbd (opt-in)", "avbd"),
        ("SEQUENTIAL-IMPULSE", "sequential_impulse"),
    ],
)
def test_report_family_maps_to_packet_enum(report_family, expected):
    assert schema.resolved_rigid_contact_solver_from_report(report_family) == expected


def test_contact_free_scene_records_none():
    identity = schema.make_resolved_solver_identity(
        resolved_rigid_contact_family=None,
        rigid_point_joint_solver="none",
        avbd_rigid_contact_config_emplaced=False,
        recorded_from="contact-free scene",
    )
    assert identity["rigid_contact_solver"] == "none"


def test_avbd_substitution_requires_emplaced_config():
    # The report's AVBD substitution marker maps to "avbd"; with the opt-in
    # emplaced it is a valid identity.
    identity = schema.make_resolved_solver_identity(
        resolved_rigid_contact_family="sequential-impulse + avbd (opt-in)",
        rigid_point_joint_solver="none",
        avbd_rigid_contact_config_emplaced=True,
        recorded_from="avbd-opt-in scene",
    )
    assert identity["rigid_contact_solver"] == "avbd"

    # Claiming the avbd contact path without the opt-in is rejected by the
    # schema contract, so the builder refuses to produce it.
    with pytest.raises(ValueError):
        schema.make_resolved_solver_identity(
            resolved_rigid_contact_family="sequential-impulse + avbd (opt-in)",
            rigid_point_joint_solver="none",
            avbd_rigid_contact_config_emplaced=False,
            recorded_from="avbd-opt-in scene",
        )


def test_public_avbd_family_records_selection_without_private_config():
    identity = schema.make_resolved_solver_identity(
        resolved_rigid_contact_family="avbd",
        rigid_point_joint_solver="avbd",
        avbd_rigid_contact_config_emplaced=False,
        recorded_from="World resolved-configuration report",
        rigid_contact_selection="world_solver_family",
    )
    assert identity == {
        "avbd_rigid_contact_config_emplaced": False,
        "multibody_integration_family": "none",
        "recorded_from": "World resolved-configuration report",
        "rigid_contact_selection": "world_solver_family",
        "rigid_contact_solver": "avbd",
        "rigid_point_joint_solver": "avbd",
    }


def test_public_avbd_family_requires_explicit_selection_source():
    with pytest.raises(ValueError):
        schema.make_resolved_solver_identity(
            resolved_rigid_contact_family="avbd",
            rigid_point_joint_solver="avbd",
            avbd_rigid_contact_config_emplaced=False,
            recorded_from="World resolved-configuration report",
        )


def test_public_vbd_family_records_matching_world_owned_rows():
    identity = schema.make_resolved_solver_identity(
        resolved_rigid_contact_family="vbd",
        rigid_point_joint_solver="vbd",
        avbd_rigid_contact_config_emplaced=False,
        recorded_from="World resolved-configuration report",
        rigid_contact_selection="world_solver_family",
    )
    assert identity["rigid_contact_solver"] == "vbd"
    assert identity["rigid_point_joint_solver"] == "vbd"


@pytest.mark.parametrize("solver", ["avbd", "vbd"])
def test_public_world_family_accepts_body_config_refinement(solver):
    identity = schema.make_resolved_solver_identity(
        resolved_rigid_contact_family=solver,
        rigid_point_joint_solver=solver,
        avbd_rigid_contact_config_emplaced=True,
        recorded_from="World resolved-configuration report",
        rigid_contact_selection="world_solver_family",
    )

    assert identity["rigid_contact_selection"] == "world_solver_family"
    assert identity["rigid_contact_solver"] == solver
    assert identity["avbd_rigid_contact_config_emplaced"] is True


def test_public_sequential_impulse_family_records_matching_world_owned_rows():
    identity = schema.make_resolved_solver_identity(
        resolved_rigid_contact_family="sequential-impulse",
        rigid_point_joint_solver="sequential_impulse",
        avbd_rigid_contact_config_emplaced=False,
        recorded_from="World resolved-configuration report",
    )
    assert identity["rigid_contact_solver"] == "sequential_impulse"
    assert identity["rigid_point_joint_solver"] == "sequential_impulse"
    assert identity["rigid_contact_selection"] == "contact_solver_method"


def test_public_world_family_rejects_contradictory_point_joint_family():
    packet = {
        "schema_version": schema.AVBD_PACKET_SCHEMA_VERSION,
        schema.RESOLVED_SOLVER_IDENTITY_KEY: {
            "avbd_rigid_contact_config_emplaced": False,
            "recorded_from": "contradictory test identity",
            "rigid_contact_selection": "world_solver_family",
            "rigid_contact_solver": "vbd",
            "rigid_point_joint_solver": "avbd",
        },
    }

    errors = schema.resolved_solver_identity_errors(packet, "case")

    assert any(
        "rigid_point_joint_solver to be 'none' or match" in error for error in errors
    )


def test_vbd_point_rows_require_public_vbd_world_family():
    packet = {
        "schema_version": schema.AVBD_PACKET_SCHEMA_VERSION,
        schema.RESOLVED_SOLVER_IDENTITY_KEY: {
            "avbd_rigid_contact_config_emplaced": False,
            "recorded_from": "impossible VBD point-row identity",
            "rigid_contact_selection": "contact_solver_method",
            "rigid_contact_solver": "sequential_impulse",
            "rigid_point_joint_solver": "vbd",
        },
    }

    errors = schema.resolved_solver_identity_errors(packet, "case")

    assert any(
        "requires rigid_point_joint_solver 'none' or 'sequential_impulse'" in error
        for error in errors
    )


@pytest.mark.parametrize("contact_solver", ["boxed_lcp", "sequential_impulse"])
def test_contact_method_can_select_contacts_with_sequential_impulse_point_rows(
    contact_solver,
):
    identity = schema.make_resolved_solver_identity(
        resolved_rigid_contact_family=contact_solver.replace("_", "-"),
        rigid_point_joint_solver="sequential_impulse",
        avbd_rigid_contact_config_emplaced=False,
        recorded_from="World resolved-configuration report",
    )
    assert identity["rigid_contact_selection"] == "contact_solver_method"
    assert identity["rigid_contact_solver"] == contact_solver
    assert identity["rigid_point_joint_solver"] == "sequential_impulse"


def test_schema_v2_cannot_claim_vbd_without_selection_source_contract():
    packet = {
        "schema_version": 2,
        schema.RESOLVED_SOLVER_IDENTITY_KEY: {
            "avbd_rigid_contact_config_emplaced": False,
            "recorded_from": "pre-selection-source schema",
            "rigid_contact_solver": "vbd",
            "rigid_point_joint_solver": "vbd",
        },
    }

    errors = schema.resolved_solver_identity_errors(packet, "case")

    assert any(
        "VBD solver identities require schema_version 3" in error for error in errors
    )


def test_schema_v2_cannot_claim_sequential_impulse_point_rows():
    packet = {
        "schema_version": 2,
        schema.RESOLVED_SOLVER_IDENTITY_KEY: {
            "avbd_rigid_contact_config_emplaced": False,
            "recorded_from": "pre-selection-source schema",
            "rigid_contact_solver": "sequential_impulse",
            "rigid_point_joint_solver": "sequential_impulse",
        },
    }

    errors = schema.resolved_solver_identity_errors(packet, "case")

    assert any(
        "Sequential Impulse point-joint solver identities require "
        "schema_version 3" in error
        for error in errors
    )


def test_unrecognized_report_family_raises():
    with pytest.raises(ValueError):
        schema.resolved_rigid_contact_solver_from_report("magic-solver")
    with pytest.raises(ValueError):
        schema.resolved_rigid_contact_solver_from_report("magic-avbd-solver")
    with pytest.raises(ValueError):
        schema.resolved_rigid_contact_solver_from_report("magic + avbd (opt-in)")
    with pytest.raises(ValueError):
        schema.resolved_rigid_contact_solver_from_report("")


def test_invalid_point_joint_solver_rejected():
    with pytest.raises(ValueError):
        schema.make_resolved_solver_identity(
            resolved_rigid_contact_family=None,
            rigid_point_joint_solver="boxed_lcp",  # not a point-joint family
            avbd_rigid_contact_config_emplaced=False,
            recorded_from="bad point-joint solver",
        )


def _contact_method_identity_packet(version, point_joint_solver):
    return {
        "schema_version": version,
        schema.RESOLVED_SOLVER_IDENTITY_KEY: {
            "avbd_rigid_contact_config_emplaced": False,
            "recorded_from": "resolved configuration report",
            "rigid_contact_selection": "contact_solver_method",
            "rigid_contact_solver": "sequential_impulse",
            "rigid_point_joint_solver": point_joint_solver,
        },
    }


def test_contact_method_v3_accepts_legacy_avbd_point_rows():
    # Pre-slice packets legitimately recorded 'avbd' point-joint identities
    # under 'contact_solver_method' because public pair constraints then ran
    # the private AVBD projection; version 3 packets must stay valid.
    packet = _contact_method_identity_packet(3, "avbd")
    assert schema.resolved_solver_identity_errors(packet, "case") == []


def test_contact_method_current_version_rejects_avbd_point_rows():
    packet = _contact_method_identity_packet(schema.AVBD_PACKET_SCHEMA_VERSION, "avbd")
    errors = schema.resolved_solver_identity_errors(packet, "case")
    assert any(
        "rigid_point_joint_solver 'none' or 'sequential_impulse'" in error
        for error in errors
    )


def test_contact_method_v3_still_rejects_vbd_point_rows():
    packet = _contact_method_identity_packet(3, "vbd")
    errors = schema.resolved_solver_identity_errors(packet, "case")
    assert any(
        "cannot carry rigid_point_joint_solver 'vbd'" in error for error in errors
    )


def test_future_schema_version_is_rejected_fail_closed():
    packet = {"schema_version": schema.AVBD_PACKET_SCHEMA_VERSION + 1}

    errors = schema.packet_schema_version_errors(packet, "case")

    assert any("newer than the supported version" in error for error in errors)


def _plan104_claim(**overrides):
    claim = {
        "status": "complete",
        "predicate_results": {
            "artifact_valid": True,
            "solver_contract_valid": True,
            "physical_outcome_valid": True,
            "performance_comparable": True,
            "claim_valid": True,
        },
        "backend_results": {"cpu": True, "cuda": True},
    }
    claim.update(overrides)
    return claim


def _plan104_packet(claims, *, contract_rows=None):
    if contract_rows is None:
        contract_rows = ["avbd.method.cpu_solver"]
    return {
        "schema_version": schema.AVBD_PACKET_SCHEMA_VERSION,
        "target": {"contract_rows": contract_rows},
        "plan104_claims": claims,
    }


def test_current_packet_may_omit_plan104_claims_while_partial():
    packet = {"schema_version": schema.AVBD_PACKET_SCHEMA_VERSION}
    assert schema.plan104_claims_errors(packet, "case") == []


def test_current_packet_accepts_typed_complete_plan104_claim():
    packet = _plan104_packet({"avbd.method.cpu_solver": _plan104_claim()})
    assert schema.plan104_claims_errors(packet, "case") == []


def test_plan104_claims_require_target_contract_rows():
    packet = _plan104_packet({"avbd.method.cpu_solver": _plan104_claim()})
    del packet["target"]

    errors = schema.plan104_claims_errors(packet, "case")

    assert any("requires target to be an object" in error for error in errors)


@pytest.mark.parametrize(
    "claim_id",
    (
        "method.cpu_solver",
        "AVBD.method.cpu_solver",
        "avbd.method.cpu-solver",
        "avbd..cpu_solver",
        "avbd.method",
    ),
)
def test_plan104_claim_keys_must_be_canonical_row_ids(claim_id):
    packet = _plan104_packet({claim_id: _plan104_claim()}, contract_rows=[claim_id])

    errors = schema.plan104_claims_errors(packet, "case")

    assert any("canonical VBD or AVBD row ID" in error for error in errors)


def test_plan104_claim_cannot_escape_target_contract_rows():
    packet = _plan104_packet(
        {"avbd.method.cuda_solver": _plan104_claim()},
        contract_rows=["avbd.method.cpu_solver"],
    )

    errors = schema.plan104_claims_errors(packet, "case")

    assert any("is not authorized by target.contract_rows" in error for error in errors)


def test_plan104_target_contract_rows_must_be_unique_canonical_ids():
    packet = _plan104_packet(
        {"avbd.method.cpu_solver": _plan104_claim()},
        contract_rows=[
            "avbd.method.cpu_solver",
            "avbd.method.cpu_solver",
            "not-canonical",
        ],
    )

    errors = schema.plan104_claims_errors(packet, "case")

    assert any("must not duplicate" in error for error in errors)
    assert any("target.contract_rows[2]" in error for error in errors)


@pytest.mark.parametrize(
    ("claims", "message"),
    [
        (None, "must be a non-empty object"),
        (
            {"avbd.method.cpu_solver": _plan104_claim(status="partial")},
            "status must be 'complete'",
        ),
        (
            {
                "avbd.method.cpu_solver": _plan104_claim(
                    backend_results={"cpu": True, "cuda": False}
                )
            },
            "backend_results must be true for: cuda",
        ),
        (
            {
                "avbd.method.cpu_solver": _plan104_claim(
                    predicate_results={"unknown": True}
                )
            },
            "predicate_results has unknown keys",
        ),
        (
            {
                "avbd.method.cpu_solver": {
                    **_plan104_claim(),
                    "unvalidated_note": "not part of the closure contract",
                }
            },
            "must contain exactly, in order",
        ),
        (
            {
                "avbd.method.cpu_solver": _plan104_claim(
                    predicate_results={"artifact_valid": "yes"}
                )
            },
            "artifact_valid must be boolean",
        ),
        (
            {
                "avbd.method.cpu_solver": _plan104_claim(
                    predicate_results={"artifact_valid": False}
                )
            },
            "predicate_results must be true for: artifact_valid",
        ),
        (
            {"avbd.method.cpu_solver": _plan104_claim(backend_results={"cpu": True})},
            "must contain exactly, in order: cpu, cuda",
        ),
    ],
)
def test_plan104_claim_mutations_are_rejected(claims, message):
    packet = _plan104_packet(claims)
    errors = schema.plan104_claims_errors(packet, "case")
    assert any(message in error for error in errors), errors


@pytest.mark.parametrize("map_name", ("predicate_results", "backend_results"))
def test_plan104_claim_non_string_result_key_fails_closed(map_name):
    claim = _plan104_claim()
    claim[map_name] = {0: True, **claim[map_name]}
    packet = _plan104_packet({"avbd.method.cpu_solver": claim})

    errors = schema.plan104_claims_errors(packet, "case")

    assert any(f"{map_name} has unknown keys" in error for error in errors), errors


def test_plan104_claim_non_string_false_predicate_key_fails_closed():
    claim = _plan104_claim(predicate_results={0: False})
    packet = _plan104_packet({"avbd.method.cpu_solver": claim})

    errors = schema.plan104_claims_errors(packet, "case")

    assert any("predicate_results must be true for: 0" in error for error in errors)


def test_legacy_packet_cannot_add_plan104_claims():
    packet = _plan104_packet({"avbd.method.cpu_solver": _plan104_claim()})
    packet["schema_version"] = schema.PLAN104_CLAIMS_MIN_SCHEMA_VERSION - 1
    errors = schema.plan104_claims_errors(packet, "case")
    assert any("requires schema_version 5" in error for error in errors), errors


def test_plan104_claim_minimum_schema_version_is_fixed_at_five():
    assert schema.PLAN104_CLAIMS_MIN_SCHEMA_VERSION == 5


@pytest.mark.parametrize(
    "mutation",
    [
        lambda claim: {
            "predicate_results": claim["predicate_results"],
            "status": claim["status"],
            "backend_results": claim["backend_results"],
        },
        lambda claim: {
            **claim,
            "predicate_results": dict(
                reversed(tuple(claim["predicate_results"].items()))
            ),
        },
        lambda claim: {
            **claim,
            "backend_results": dict(reversed(tuple(claim["backend_results"].items()))),
        },
    ],
)
def test_plan104_claim_same_content_order_mutations_are_rejected(mutation):
    claim = _plan104_claim()
    packet = _plan104_packet({"avbd.method.cpu_solver": mutation(claim)})

    errors = schema.plan104_claims_errors(packet, "case")

    assert errors, packet
    assert any("order" in error for error in errors), errors


def test_evidence_project_appended_flags_match_the_benchmark_runner():
    schema = _load_module()
    runner_spec = importlib.util.spec_from_file_location(
        "run_figure13_benchmark", SCRIPT.parent / "run_figure13_benchmark.py"
    )
    assert runner_spec is not None and runner_spec.loader is not None
    runner = importlib.util.module_from_spec(runner_spec)
    # The runner imports its sibling scripts by module name.
    if str(SCRIPT.parent) not in sys.path:
        sys.path.insert(0, str(SCRIPT.parent))
    runner_spec.loader.exec_module(runner)
    assert set(schema.EVIDENCE_PROJECT_APPENDED_FLAGS) == set(
        runner.PROJECT_APPENDED_FLAG_KEYS
    )
    for name, tokens in schema.EVIDENCE_PROJECT_APPENDED_FLAGS.items():
        assert tokens == runner.PROJECT_APPENDED_FLAG_TOKENS
    assert schema.evidence_definition_matches(
        "CMAKE_SHARED_LINKER_FLAGS", "", " -Wl,--no-undefined"
    )
    assert not schema.evidence_definition_matches(
        "CMAKE_SHARED_LINKER_FLAGS", "", "-Wl,-O2"
    )
    assert not schema.evidence_definition_matches("CMAKE_CXX_FLAGS", "", " -O0")


def test_stable_counter_stddev_noise_bound_separates_float_artifacts_from_drift():
    schema = _load_module()
    word = "solver_configuration_fingerprint_hi"
    # Five identical 32-bit fingerprint words aggregated in double precision.
    assert schema.stable_counter_stddev_is_noise(word, 17.88854381999832, 1260079489.0)
    assert schema.stable_counter_stddev_is_noise(word, 0.0, 4160164591.0)
    # Anything beyond the rounding bound is a real fingerprint change.
    assert not schema.stable_counter_stddev_is_noise(word, 1.0e3, 1260079489.0)
    assert not schema.stable_counter_stddev_is_noise(word, 1.0e6, 1260079489.0)
    # Every other invariant counter must not drift at all.
    assert not schema.stable_counter_stddev_is_noise("public_avbd_family", 0.5, 1.0)
    assert not schema.stable_counter_stddev_is_noise("rigid_avbd_beta", 1.0, 10.0)
    assert schema.stable_counter_stddev_is_noise("rigid_avbd_beta", 0.0, 10.0)
    assert not schema.stable_counter_stddev_is_noise(word, None, 1.0)


def test_fingerprint_word_noise_bound_is_exact_at_its_boundary():
    schema = _load_module()
    reference = 1260079489.0
    bound = schema.FINGERPRINT_WORD_STDDEV_RELATIVE_BOUND * reference
    assert schema.FINGERPRINT_WORD_STDDEV_RELATIVE_BOUND == 4.0 * 2.0**-26
    assert schema.stable_counter_stddev_is_noise(
        "scene_spec_fingerprint_lo", bound, reference
    )
    assert not schema.stable_counter_stddev_is_noise(
        "scene_spec_fingerprint_lo", math.nextafter(bound, math.inf), reference
    )
    # Five repetitions of a 2^32 word: sqrt(5) * v * 2^-26 must stay inside.
    largest_word = 4294967295.0
    assert schema.stable_counter_stddev_is_noise(
        "scene_spec_fingerprint_hi",
        math.sqrt(5.0) * largest_word * 2.0**-26,
        largest_word,
    )


def test_evidence_definition_requires_a_token_boundary_before_appended_flags():
    schema = _load_module()
    name = "CMAKE_SHARED_LINKER_FLAGS"
    assert schema.evidence_definition_matches(
        name, "-fuse-ld=lld", "-fuse-ld=lld -Wl,--no-undefined"
    )
    assert not schema.evidence_definition_matches(
        name, "-fuse-ld=lld", "-fuse-ld=lld-Wl,--no-undefined"
    )
    assert schema.evidence_definition_matches(name, "", "-Wl,--no-undefined")
    assert not schema.evidence_definition_matches(name, "", "-Wl,--no-undefined -O3")
