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


@pytest.mark.parametrize(
    ("report_family", "expected"),
    [
        ("sequential-impulse", "sequential_impulse"),
        ("boxed-lcp", "boxed_lcp"),
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


def test_unrecognized_report_family_raises():
    with pytest.raises(ValueError):
        schema.resolved_rigid_contact_solver_from_report("magic-solver")
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
