"""Tests for scripts/check_avbd_packets.py (PLAN-091 WP-091.1)."""

import importlib.util
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "check_avbd_packets.py"
PACKET_DIR = ROOT / "docs" / "plans" / "104-vertex-block-descent-solver"


def _load_module():
    spec = importlib.util.spec_from_file_location("check_avbd_packets", SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _identity(**overrides):
    identity = {
        "rigid_contact_solver": "sequential_impulse",
        "rigid_point_joint_solver": "avbd",
        "avbd_rigid_contact_config_emplaced": False,
        "recorded_from": "scene-construction audit (WP-091.1)",
    }
    identity.update(overrides)
    return identity


def _write_packet(tmp_path, name, packet):
    path = tmp_path / name
    path.write_text(json.dumps(packet, indent=2, sort_keys=True) + "\n")
    return path


def test_committed_packet_corpus_passes():
    module = _load_module()
    assert module.main([]) == 0


def test_new_version_packet_without_identity_is_rejected(tmp_path):
    module = _load_module()
    path = _write_packet(
        tmp_path,
        "avbd-new-scene-packet.json",
        {"schema_version": module.AVBD_PACKET_SCHEMA_VERSION},
    )
    errors = module.packet_errors(path)
    assert any("requires resolved_solver_identity" in error for error in errors)


def test_new_version_packet_with_identity_passes(tmp_path):
    module = _load_module()
    path = _write_packet(
        tmp_path,
        "avbd-new-scene-packet.json",
        {
            "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
            "resolved_solver_identity": _identity(),
        },
    )
    assert module.packet_errors(path) == []
    assert module.main(["--packet", str(path)]) == 0


def test_avbd_contact_claim_without_emplaced_config_is_rejected(tmp_path):
    module = _load_module()
    path = _write_packet(
        tmp_path,
        "avbd-new-scene-packet.json",
        {
            "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
            "resolved_solver_identity": _identity(
                rigid_contact_solver="avbd",
                avbd_rigid_contact_config_emplaced=False,
            ),
        },
    )
    errors = module.packet_errors(path)
    assert any("cannot be 'avbd' when" in error for error in errors), errors


def test_unknown_solver_name_is_rejected(tmp_path):
    module = _load_module()
    path = _write_packet(
        tmp_path,
        "avbd-new-scene-packet.json",
        {
            "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
            "resolved_solver_identity": _identity(
                rigid_contact_solver="warm_started_magic"
            ),
        },
    )
    errors = module.packet_errors(path)
    assert any("rigid_contact_solver" in error for error in errors)


def test_new_packet_file_below_current_version_is_rejected(tmp_path):
    module = _load_module()
    path = _write_packet(
        tmp_path,
        "avbd-new-scene-packet.json",
        {"schema_version": 1},
    )
    errors = module.packet_errors(path)
    assert any("must be written at schema_version" in error for error in errors)


def test_legacy_allowlisted_packet_stays_readable(tmp_path):
    module = _load_module()
    name = "avbd-demo2d-pyramid-packet.json"
    assert name in module.LEGACY_IDENTITY_EXEMPT_PACKETS
    path = _write_packet(tmp_path, name, {"schema_version": 1})
    assert module.packet_errors(path) == []


def test_non_integer_schema_version_is_rejected(tmp_path):
    module = _load_module()
    path = _write_packet(
        tmp_path,
        "avbd-new-scene-packet.json",
        {"schema_version": "two"},
    )
    errors = module.packet_errors(path)
    assert any("schema_version must be an integer" in error for error in errors)


def test_legacy_allowlist_matches_committed_corpus():
    module = _load_module()
    committed = {path.name for path in PACKET_DIR.glob("avbd-*-packet.json")}
    assert committed <= module.LEGACY_IDENTITY_EXEMPT_PACKETS
