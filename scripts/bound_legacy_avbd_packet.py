#!/usr/bin/env python3
"""Stamp the allowlisted claim boundary into a pinned legacy AVBD packet.

Legacy packets whose writer inputs no longer exist stay pinned at their
committed schema version. `check_avbd_packets.py` requires each of them to
carry the `evidence_boundary` that declares the packet a historical, unbound
artifact. This tool writes exactly that boundary (and nothing else) so no
packet JSON is edited by hand.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from avbd_packet_schema import (  # noqa: E402
    PLAN104_CLAIMS_MIN_SCHEMA_VERSION,
    legacy_evidence_boundary,
)
from check_avbd_packets import (  # noqa: E402
    LEGACY_NON_EVIDENCE_BOUNDARY_SCOPES,
    LEGACY_PACKET_SCHEMA_VERSIONS,
)

DEFAULT_REASON = (
    "This legacy packet preserves historical hashes and timing metadata whose "
    "source artifacts, writer inputs and runtime identity counters are "
    "unavailable. It stays pinned at its committed schema version and supports "
    "no current AVBD claim."
)


class LegacyPacketBoundaryError(RuntimeError):
    pass


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--packet", type=Path, action="append", required=True)
    parser.add_argument("--reason", default=DEFAULT_REASON)
    return parser.parse_args(argv)


def bound_packet(packet: dict[str, object], packet_name: str, reason: str) -> bool:
    """Insert the boundary; return True when the packet changed."""
    scope = LEGACY_NON_EVIDENCE_BOUNDARY_SCOPES.get(packet_name)
    if scope is None:
        raise LegacyPacketBoundaryError(
            f"{packet_name} is not an allowlisted legacy packet"
        )
    version = packet.get("schema_version")
    pinned = LEGACY_PACKET_SCHEMA_VERSIONS.get(packet_name)
    if not isinstance(version, int) or version != pinned:
        raise LegacyPacketBoundaryError(
            f"{packet_name}: schema_version must be the pinned legacy version "
            f"{pinned}, got {version!r}"
        )
    if version >= PLAN104_CLAIMS_MIN_SCHEMA_VERSION:
        raise LegacyPacketBoundaryError(
            f"{packet_name}: schema_version {version} already carries the "
            "claims contract and needs no legacy boundary"
        )
    boundary = legacy_evidence_boundary(scope, reason=reason)
    if packet.get("evidence_boundary") == boundary:
        return False
    packet["evidence_boundary"] = boundary
    return True


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    for path in args.packet:
        packet = json.loads(path.read_text())
        if not isinstance(packet, dict):
            raise LegacyPacketBoundaryError(f"{path}: packet must be a JSON object")
        changed = bound_packet(packet, path.name, args.reason)
        if changed:
            # Keep the pinned key order: only the boundary is appended.
            path.write_text(json.dumps(packet, indent=2) + "\n")
        print(f"{path}: {'bounded' if changed else 'already bounded'}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except LegacyPacketBoundaryError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
