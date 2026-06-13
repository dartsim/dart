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

from collections.abc import Mapping
from typing import Any

# Current AVBD packet schema version. Packets written at this version or
# newer must carry RESOLVED_SOLVER_IDENTITY_KEY.
AVBD_PACKET_SCHEMA_VERSION = 2

# First schema version that requires the resolved-solver-identity field.
SOLVER_IDENTITY_MIN_SCHEMA_VERSION = 2

RESOLVED_SOLVER_IDENTITY_KEY = "resolved_solver_identity"

# The contact path that actually resolved rigid-rigid contacts in the timed
# scene. "avbd" is valid only when the scene emplaces the internal AVBD
# rigid-contact opt-in config so that every active contact has a configured
# body; contact-free scenes record "none".
ALLOWED_RIGID_CONTACT_SOLVERS = (
    "avbd",
    "boxed_lcp",
    "none",
    "sequential_impulse",
)

# The solver family that resolved rigid-body point-joint/motor/distance-spring
# rows; joint-free scenes record "none".
ALLOWED_RIGID_POINT_JOINT_SOLVERS = ("avbd", "none")

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
    return []


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

    emplaced = identity.get("avbd_rigid_contact_config_emplaced")
    if not isinstance(emplaced, bool):
        errors.append(
            f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
            "avbd_rigid_contact_config_emplaced must be a boolean"
        )

    recorded_from = identity.get("recorded_from")
    if not isinstance(recorded_from, str) or not recorded_from:
        errors.append(
            f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}.recorded_from "
            "must be a non-empty string naming how the identity was captured"
        )

    if (
        isinstance(emplaced, bool)
        and not emplaced
        and identity.get("rigid_contact_solver") == "avbd"
    ):
        errors.append(
            f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
            "rigid_contact_solver cannot be 'avbd' when "
            "avbd_rigid_contact_config_emplaced is false"
        )

    return errors
