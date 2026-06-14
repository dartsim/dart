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


# --- Deriving the packet identity from the resolved-configuration report -----
#
# PLAN-091 WP-091.11 slice 4: the C++ resolved-configuration report
# (``dart::simulation::World::recordResolvedConfiguration``, slices 1-2) is the
# engine's own account of which solver family resolved each domain. These
# mappings translate the report's rigid-contact family strings onto the
# packet's ``rigid_contact_solver`` enum so the packet identity is *derived
# from the report's contract* through one place, rather than each writer
# hand-typing the enum. The report-family strings below mirror the C++ report;
# ``tests/test_world_resolved_configuration.cpp`` pins them on the engine side.

# Report rigid-contact ``resolved`` family strings → packet enum values.
_REPORT_RIGID_CONTACT_FAMILY_TO_PACKET = {
    "sequential-impulse": "sequential_impulse",
    "boxed-lcp": "boxed_lcp",
}

# The report appends this marker to the resolved rigid-contact family when a
# body carries the internal AVBD rigid-contact opt-in (the WP-091.1 silent
# substitution, made explicit in slice 2).
_REPORT_AVBD_CONTACT_MARKER = "avbd"


def resolved_rigid_contact_solver_from_report(resolved_family: str) -> str:
    """Map a report rigid-contact ``resolved`` family string to the packet enum.

    ``resolved_family`` is the string the C++ report records for the
    ``rigid-contact`` domain (e.g. ``"sequential-impulse"``, ``"boxed-lcp"``,
    or a ``"... + avbd (opt-in)"`` substitution). Raises ``ValueError`` for an
    empty or unrecognized family so a report-vocabulary change cannot silently
    produce an out-of-contract packet enum.
    """
    text = resolved_family.strip().lower()
    if not text:
        raise ValueError("resolved rigid-contact family must be non-empty")
    if _REPORT_AVBD_CONTACT_MARKER in text:
        return "avbd"
    try:
        return _REPORT_RIGID_CONTACT_FAMILY_TO_PACKET[text]
    except KeyError:
        raise ValueError(
            f"unrecognized resolved rigid-contact family {resolved_family!r}; "
            f"known: {sorted(_REPORT_RIGID_CONTACT_FAMILY_TO_PACKET)} "
            f"(plus an '{_REPORT_AVBD_CONTACT_MARKER}' substitution marker)"
        ) from None


def make_resolved_solver_identity(
    *,
    resolved_rigid_contact_family: str | None,
    rigid_point_joint_solver: str,
    avbd_rigid_contact_config_emplaced: bool,
    recorded_from: str,
) -> dict[str, Any]:
    """Build a validated ``resolved_solver_identity`` object from the report.

    ``resolved_rigid_contact_family`` is the report's rigid-contact family
    string for a scene that resolves rigid-rigid contacts, or ``None`` for a
    contact-free scene (recorded as ``"none"``). The returned object is
    validated against the schema contract before it is handed back, so a writer
    cannot emit an identity the checker would reject.
    """
    if resolved_rigid_contact_family is None:
        rigid_contact_solver = "none"
    else:
        rigid_contact_solver = resolved_rigid_contact_solver_from_report(
            resolved_rigid_contact_family
        )

    identity: dict[str, Any] = {
        "avbd_rigid_contact_config_emplaced": bool(avbd_rigid_contact_config_emplaced),
        "recorded_from": recorded_from,
        "rigid_contact_solver": rigid_contact_solver,
        "rigid_point_joint_solver": rigid_point_joint_solver,
    }

    errors = resolved_solver_identity_errors(
        {
            "schema_version": AVBD_PACKET_SCHEMA_VERSION,
            RESOLVED_SOLVER_IDENTITY_KEY: identity,
        },
        "make_resolved_solver_identity",
    )
    if errors:
        raise ValueError("; ".join(errors))
    return identity
