from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_ROOT / "scripts"))

import bound_legacy_avbd_packet as tool  # noqa: E402
import check_avbd_packets as checker  # noqa: E402

PINNED_NAME = "avbd-articulated-compliant-joints-packet.json"


def _legacy_packet() -> dict[str, object]:
    return {
        "schema_version": 3,
        "zeta": 1,
        "visual_capture": {
            "screenshot": {"sha256": "a" * 64},
            "image_verdict": {"sha256": "b" * 64, "pass": True},
        },
        "alpha": 2,
    }


def test_bound_packet_appends_the_allowlisted_boundary_once() -> None:
    packet = _legacy_packet()
    assert tool.bound_packet(packet, PINNED_NAME, tool.DEFAULT_REASON) is True
    boundary = packet["evidence_boundary"]
    assert boundary["artifact_status"] == "legacy_unbound"
    assert (
        boundary["supported_scope"]
        == checker.LEGACY_NON_EVIDENCE_BOUNDARY_SCOPES[PINNED_NAME]
    )
    assert boundary["reason"] == tool.DEFAULT_REASON
    assert checker._legacy_non_evidence_boundary_errors(packet, PINNED_NAME) == []
    # Idempotent: a second run changes nothing.
    assert tool.bound_packet(packet, PINNED_NAME, tool.DEFAULT_REASON) is False


def test_bound_packet_rejects_packets_outside_the_allowlist() -> None:
    with pytest.raises(tool.LegacyPacketBoundaryError, match="not an allowlisted"):
        tool.bound_packet(_legacy_packet(), "avbd-demo3d-stack-packet-x.json", "r")


def test_bound_packet_rejects_the_wrong_pinned_version() -> None:
    packet = _legacy_packet()
    packet["schema_version"] = 5
    with pytest.raises(tool.LegacyPacketBoundaryError, match="pinned legacy version 3"):
        tool.bound_packet(packet, PINNED_NAME, "r")


def test_main_preserves_pinned_key_order_and_only_appends(tmp_path: Path) -> None:
    path = tmp_path / PINNED_NAME
    path.write_text(json.dumps(_legacy_packet(), indent=2) + "\n")
    assert tool.main(["--packet", str(path)]) == 0
    written = json.loads(path.read_text())
    assert list(written) == ["schema_version", "zeta", "visual_capture", "alpha", "evidence_boundary"]
    assert path.read_text().endswith("}\n")


def test_checker_skips_image_binding_only_for_bounded_legacy_packets() -> None:
    packet = _legacy_packet()
    unbounded = checker._image_verdict_binding_errors(packet, PINNED_NAME)
    assert any("image_sha256" in error for error in unbounded)
    tool.bound_packet(packet, PINNED_NAME, tool.DEFAULT_REASON)
    assert checker._image_verdict_binding_errors(packet, PINNED_NAME) == []
    # A current-schema packet with the same shape keeps the binding requirement.
    current = _legacy_packet()
    current["schema_version"] = checker.AVBD_PACKET_SCHEMA_VERSION
    tool_boundary = tool.legacy_evidence_boundary("x", reason="r")
    current["evidence_boundary"] = tool_boundary
    assert any(
        "image_sha256" in error
        for error in checker._image_verdict_binding_errors(current, PINNED_NAME)
    )
