"""Tests for scripts/avbd_packet_schema.py (PLAN-091 WP-091.11 slice 4).

These cover the report-derived ``resolved_solver_identity`` builder: it must
reproduce, byte-for-byte, the identities the AVBD packet writers previously
hand-typed (so committed packets are unchanged), and it must translate the C++
resolved-configuration report's rigid-contact family vocabulary onto the packet
enum through the single mapping in the schema module.
"""

import importlib.util
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


# The identities the four AVBD packet writers committed before slice 4. The
# builder must reproduce each exactly so regeneration leaves the packets byte
# identical.
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


@pytest.mark.parametrize(
    ("selection", "contact_solver", "emplaced"),
    [
        ("body_opt_in", "avbd", True),
        ("contact_solver_method", "sequential_impulse", False),
    ],
)
def test_vbd_point_rows_require_public_vbd_world_family(
    selection,
    contact_solver,
    emplaced,
):
    packet = {
        "schema_version": schema.AVBD_PACKET_SCHEMA_VERSION,
        schema.RESOLVED_SOLVER_IDENTITY_KEY: {
            "avbd_rigid_contact_config_emplaced": emplaced,
            "recorded_from": "impossible VBD point-row identity",
            "rigid_contact_selection": selection,
            "rigid_contact_solver": contact_solver,
            "rigid_point_joint_solver": "vbd",
        },
    }

    errors = schema.resolved_solver_identity_errors(packet, "case")

    assert any("rigid_point_joint_solver 'vbd' requires" in error for error in errors)


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
