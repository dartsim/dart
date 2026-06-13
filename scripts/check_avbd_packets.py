#!/usr/bin/env python3
"""Validate committed AVBD evidence packets against the shared schema.

Enforces PLAN-091 WP-091.1: AVBD packets at the current schema version
must machine-record the resolved solver configuration that actually ran
(``resolved_solver_identity``). Packets committed before the identity
contract stay readable through a legacy allowlist, but new packet files
must be written at the current schema version. The field contract lives
in ``scripts/avbd_packet_schema.py``.
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
    AVBD_PACKET_SCHEMA_VERSION,
    packet_schema_version_errors,
    resolved_solver_identity_errors,
)

REPO_ROOT = SCRIPT_DIR.parent
DEFAULT_PACKET_DIR = REPO_ROOT / "docs" / "plans" / "104-vertex-block-descent-solver"
PACKET_GLOB = "avbd-*-packet.json"

# Packets committed before the resolved-solver-identity contract
# (WP-091.1). They remain readable at schema_version 1; their
# sequential-impulse contact rows are relabeled in prose instead of
# being rewritten. Do not add new packets here: new packet files must
# be written at AVBD_PACKET_SCHEMA_VERSION with a recorded identity.
LEGACY_IDENTITY_EXEMPT_PACKETS = frozenset(
    {
        "avbd-articulated-breakable-joint-packet.json",
        "avbd-articulated-breakable-motor-packet.json",
        "avbd-articulated-fixed-pair-breakable-joint-packet.json",
        "avbd-articulated-high-ratio-chain-packet.json",
        "avbd-articulated-prismatic-motor-packet.json",
        "avbd-articulated-prismatic-pair-breakable-motor-packet.json",
        "avbd-articulated-revolute-motor-packet.json",
        "avbd-articulated-spherical-breakable-joint-packet.json",
        "avbd-articulated-spherical-pair-breakable-joint-packet.json",
        "avbd-articulated-world-prismatic-breakable-motor-packet.json",
        "avbd-articulated-world-revolute-breakable-motor-packet.json",
        "avbd-demo2d-cards-packet.json",
        "avbd-demo2d-dynamic-friction-packet.json",
        "avbd-demo2d-fracture-packet.json",
        "avbd-demo2d-ground-packet.json",
        "avbd-demo2d-hanging-rope-packet.json",
        "avbd-demo2d-heavy-rope-packet.json",
        "avbd-demo2d-joint-grid-packet.json",
        "avbd-demo2d-motor-packet.json",
        "avbd-demo2d-net-packet.json",
        "avbd-demo2d-pyramid-packet.json",
        "avbd-demo2d-rod-packet.json",
        "avbd-demo2d-rope-packet.json",
        "avbd-demo2d-soft-body-packet.json",
        "avbd-demo2d-spring-packet.json",
        "avbd-demo2d-spring-ratio-packet.json",
        "avbd-demo2d-stack-packet.json",
        "avbd-demo2d-stack-ratio-packet.json",
        "avbd-demo2d-static-friction-packet.json",
        "avbd-demo3d-breakable-packet.json",
        "avbd-demo3d-bridge-packet.json",
        "avbd-demo3d-dynamic-friction-packet.json",
        "avbd-demo3d-ground-packet.json",
        "avbd-demo3d-heavy-rope-packet.json",
        "avbd-demo3d-pyramid-packet.json",
        "avbd-demo3d-rope-packet.json",
        "avbd-demo3d-soft-body-packet.json",
        "avbd-demo3d-spring-packet.json",
        "avbd-demo3d-spring-ratio-packet.json",
        "avbd-demo3d-stack-packet.json",
        "avbd-demo3d-stack-ratio-packet.json",
        "avbd-demo3d-static-friction-packet.json",
        "avbd-empty-baseline-packet.json",
        "avbd-paper-scale-high-ratio-chain-packet.json",
        "avbd-rigid-breakable-joint-packet.json",
        "avbd-rigid-prismatic-motor-packet.json",
        "avbd-rigid-revolute-motor-packet.json",
        "avbd-rigid-spherical-breakable-joint-packet.json",
    }
)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--packet",
        action="append",
        type=Path,
        default=None,
        help="Explicit packet file to validate (repeatable); defaults to "
        "every avbd-*-packet.json under --packet-dir.",
    )
    parser.add_argument(
        "--packet-dir",
        type=Path,
        default=DEFAULT_PACKET_DIR,
        help="Directory scanned for avbd-*-packet.json files.",
    )
    return parser.parse_args(argv)


def packet_errors(path: Path) -> list[str]:
    name = path.name
    if not path.is_file():
        return [f"{name}: packet file not found at {path}"]
    try:
        packet = json.loads(path.read_text())
    except json.JSONDecodeError as exc:
        return [f"{name}: invalid JSON ({exc})"]
    if not isinstance(packet, dict):
        return [f"{name}: packet must be a JSON object"]

    errors = packet_schema_version_errors(packet, name)
    if errors:
        return errors

    version = packet["schema_version"]
    if (
        version < AVBD_PACKET_SCHEMA_VERSION
        and name not in LEGACY_IDENTITY_EXEMPT_PACKETS
    ):
        errors.append(
            f"{name}: new AVBD packets must be written at schema_version "
            f"{AVBD_PACKET_SCHEMA_VERSION} with a recorded "
            "resolved_solver_identity (legacy allowlist covers only packets "
            "committed before WP-091.1)"
        )
    errors.extend(resolved_solver_identity_errors(packet, name))
    return errors


def collect_packets(args: argparse.Namespace) -> list[Path]:
    if args.packet:
        return list(args.packet)
    return sorted(args.packet_dir.glob(PACKET_GLOB))


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    packets = collect_packets(args)
    if not packets:
        print(f"No {PACKET_GLOB} packets found under {args.packet_dir}")
        return 1

    all_errors: list[str] = []
    for path in packets:
        all_errors.extend(packet_errors(path))

    if all_errors:
        for error in all_errors:
            print(f"ERROR: {error}")
        return 1

    print(f"Validated {len(packets)} AVBD packet(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
