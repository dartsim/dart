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
# newer must carry RESOLVED_SOLVER_IDENTITY_KEY and record how rigid contact
# selected the reported solver family.
AVBD_PACKET_SCHEMA_VERSION = 4

# Exact ordered source-provenance contracts for the current Figure 13 packet
# chain. Writers and the committed-packet checker consume this one mapping so
# deleting, adding, or reordering a bound source cannot be hidden by
# recomputing the packet digest.
PAPER_PACKET_SOURCE_PATHS: dict[str, tuple[str, ...]] = {
    "avbd-paper-breakable-wall-packet.json": (
        "dart/gui/view_quality.cpp",
        "dart/gui/view_quality.hpp",
        "dart/simulation/world.cpp",
        "dart/simulation/world.hpp",
        "dart/simulation/world_options.hpp",
        "dart/simulation/comps/joint.hpp",
        "dart/simulation/compute/detail/world_step_stages.hpp",
        "dart/simulation/compute/rigid_body_contact_stage.cpp",
        "dart/simulation/detail/rigid_pair_constraint.hpp",
        "dart/simulation/detail/rigid_avbd/rigid_block_kernel.hpp",
        "dart/simulation/detail/rigid_avbd/rigid_world_contact.hpp",
        "dart/simulation/detail/world_step_schedule.hpp",
        "python/dartpy/_view_quality.py",
        "python/examples/demos/scenes/avbd_paper_breakable_wall.py",
        "python/tests/integration/test_demos_cycle.py",
        "tests/benchmark/simulation/bm_avbd_rigid_fixed_joint.cpp",
        "tests/unit/simulation/world/test_world.cpp",
        "scripts/avbd_packet_schema.py",
        "scripts/capture_py_demo.py",
        "scripts/capture_source_provenance.py",
        "scripts/write_avbd_paper_breakable_wall_packet.py",
    ),
    "avbd-paper-vbd-comparison-packet.json": (
        "dart/simulation/world.cpp",
        "dart/simulation/world.hpp",
        "dart/simulation/world_options.hpp",
        "dart/simulation/comps/joint.hpp",
        "dart/simulation/compute/detail/world_step_stages.hpp",
        "dart/simulation/compute/rigid_body_contact_stage.cpp",
        "dart/simulation/detail/rigid_pair_constraint.hpp",
        "dart/simulation/detail/rigid_avbd/rigid_block_kernel.hpp",
        "dart/simulation/detail/rigid_avbd/rigid_world_contact.hpp",
        "dart/simulation/detail/world_step_schedule.hpp",
        "python/examples/demos/scenes/avbd_paper_breakable_wall.py",
        "python/examples/demos/scenes/vbd_paper_breakable_wall.py",
        "python/tests/integration/test_demos_cycle.py",
        "tests/benchmark/simulation/bm_avbd_rigid_fixed_joint.cpp",
        "tests/unit/simulation/world/test_world.cpp",
        "scripts/avbd_packet_schema.py",
        "scripts/capture_py_demo.py",
        "scripts/capture_source_provenance.py",
        "scripts/write_avbd_paper_vbd_comparison_packet.py",
    ),
    "avbd-paper-sequential-impulse-comparison-packet.json": (
        "dart/simulation/world.cpp",
        "dart/simulation/world.hpp",
        "dart/simulation/world_options.hpp",
        "dart/simulation/comps/joint.hpp",
        "dart/simulation/compute/detail/world_step_stages.hpp",
        "dart/simulation/compute/rigid_body_contact_stage.cpp",
        "dart/simulation/detail/rigid_pair_constraint.hpp",
        "dart/simulation/detail/world_step_schedule.hpp",
        "python/examples/demos/scenes/avbd_paper_breakable_wall.py",
        "python/examples/demos/scenes/vbd_paper_breakable_wall.py",
        "python/examples/demos/scenes/sequential_impulse_paper_breakable_wall.py",
        "python/tests/integration/test_demos_cycle.py",
        "tests/benchmark/simulation/bm_avbd_rigid_fixed_joint.cpp",
        "tests/unit/simulation/contact/test_boxed_lcp_contact.cpp",
        "tests/unit/simulation/world/test_world.cpp",
        "tests/unit/simulation/world/test_world_resolved_configuration.cpp",
        "tests/test_avbd_packet_schema.py",
        "scripts/avbd_packet_schema.py",
        "scripts/capture_py_demo.py",
        "scripts/capture_source_provenance.py",
        "scripts/write_avbd_paper_breakable_wall_packet.py",
        "scripts/write_avbd_paper_vbd_comparison_packet.py",
        "scripts/write_avbd_paper_sequential_impulse_comparison_packet.py",
    ),
}

# Schema version 4 narrowed 'contact_solver_method' point-joint identities to
# 'none' or 'sequential_impulse'. Older packets legitimately recorded 'avbd'
# there because public pair constraints then ran the private AVBD projection.
CONTACT_METHOD_POINT_JOINT_MIN_SCHEMA_VERSION = 4

# First schema version that requires the resolved-solver-identity field.
SOLVER_IDENTITY_MIN_SCHEMA_VERSION = 2

# First schema version that requires the rigid-contact selection source.
RIGID_CONTACT_SELECTION_MIN_SCHEMA_VERSION = 3

RESOLVED_SOLVER_IDENTITY_KEY = "resolved_solver_identity"
RIGID_CONTACT_SELECTION_KEY = "rigid_contact_selection"

# The contact path that actually resolved rigid-rigid contacts in the timed
# scene. "vbd" and "avbd" may be selected by their public world solver
# families; "avbd" may also be selected by the compatibility-only internal
# body opt-in. Contact-free scenes record "none".
ALLOWED_RIGID_CONTACT_SOLVERS = (
    "avbd",
    "boxed_lcp",
    "none",
    "sequential_impulse",
    "vbd",
)

# The solver family that resolved rigid-body point-joint/motor/distance-spring
# rows; joint-free scenes record "none".
ALLOWED_RIGID_POINT_JOINT_SOLVERS = (
    "avbd",
    "none",
    "sequential_impulse",
    "vbd",
)

ALLOWED_RIGID_CONTACT_SELECTIONS = (
    "body_opt_in",
    "contact_solver_method",
    "not_applicable",
    "world_solver_family",
)

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
    if version > AVBD_PACKET_SCHEMA_VERSION:
        return [
            f"{packet_name}: schema_version {version} is newer than the "
            f"supported version {AVBD_PACKET_SCHEMA_VERSION}"
        ]
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

    selection = identity.get(RIGID_CONTACT_SELECTION_KEY)
    selection_required = version >= RIGID_CONTACT_SELECTION_MIN_SCHEMA_VERSION
    if selection is None:
        if selection_required:
            errors.append(
                f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
                f"{RIGID_CONTACT_SELECTION_KEY} is required at schema_version "
                f"{version}"
            )
    elif not isinstance(selection, str) or selection not in (
        ALLOWED_RIGID_CONTACT_SELECTIONS
    ):
        errors.append(
            f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
            f"{RIGID_CONTACT_SELECTION_KEY} must be one of "
            f"{sorted(ALLOWED_RIGID_CONTACT_SELECTIONS)}, got {selection!r}"
        )

    recorded_from = identity.get("recorded_from")
    if not isinstance(recorded_from, str) or not recorded_from:
        errors.append(
            f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}.recorded_from "
            "must be a non-empty string naming how the identity was captured"
        )

    rigid_contact_solver = identity.get("rigid_contact_solver")
    rigid_point_joint_solver = identity.get("rigid_point_joint_solver")
    if version < RIGID_CONTACT_SELECTION_MIN_SCHEMA_VERSION and (
        rigid_contact_solver == "vbd" or rigid_point_joint_solver == "vbd"
    ):
        errors.append(
            f"{packet_name}: VBD solver identities require schema_version "
            f"{RIGID_CONTACT_SELECTION_MIN_SCHEMA_VERSION} or newer"
        )
    if (
        version < RIGID_CONTACT_SELECTION_MIN_SCHEMA_VERSION
        and rigid_point_joint_solver == "sequential_impulse"
    ):
        errors.append(
            f"{packet_name}: Sequential Impulse point-joint solver identities "
            "require schema_version "
            f"{RIGID_CONTACT_SELECTION_MIN_SCHEMA_VERSION} or newer"
        )
    if selection is None:
        if (
            isinstance(emplaced, bool)
            and not emplaced
            and rigid_contact_solver == "avbd"
        ):
            errors.append(
                f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
                "rigid_contact_solver cannot be 'avbd' when "
                "avbd_rigid_contact_config_emplaced is false"
            )
    elif selection == "body_opt_in":
        if rigid_contact_solver != "avbd" or emplaced is not True:
            errors.append(
                f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
                "rigid_contact_selection 'body_opt_in' requires "
                "rigid_contact_solver 'avbd' and an emplaced private config"
            )
        if identity.get("rigid_point_joint_solver") == "vbd":
            errors.append(
                f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
                "rigid_point_joint_solver 'vbd' requires "
                "rigid_contact_selection 'world_solver_family'"
            )
    elif selection == "world_solver_family":
        if rigid_contact_solver not in ("avbd", "vbd") or emplaced is not False:
            errors.append(
                f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
                "rigid_contact_selection 'world_solver_family' requires "
                "rigid_contact_solver 'avbd' or 'vbd' "
                "without a private body config"
            )
        if rigid_point_joint_solver not in ("none", rigid_contact_solver):
            errors.append(
                f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
                "rigid_contact_selection 'world_solver_family' requires "
                "rigid_point_joint_solver to be 'none' or match "
                "rigid_contact_solver"
            )
    elif selection == "contact_solver_method":
        if (
            rigid_contact_solver not in ("boxed_lcp", "sequential_impulse")
            or emplaced is not False
        ):
            errors.append(
                f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
                "rigid_contact_selection 'contact_solver_method' requires "
                "a boxed_lcp or sequential_impulse contact solver without "
                "a private AVBD body config"
            )
        if version >= CONTACT_METHOD_POINT_JOINT_MIN_SCHEMA_VERSION:
            if rigid_point_joint_solver not in ("none", "sequential_impulse"):
                errors.append(
                    f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
                    "rigid_contact_selection 'contact_solver_method' requires "
                    "rigid_point_joint_solver 'none' or 'sequential_impulse' "
                    "from schema_version "
                    f"{CONTACT_METHOD_POINT_JOINT_MIN_SCHEMA_VERSION} onward"
                )
        elif rigid_point_joint_solver == "vbd":
            errors.append(
                f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
                "rigid_contact_selection 'contact_solver_method' cannot carry "
                "rigid_point_joint_solver 'vbd'"
            )
    elif selection == "not_applicable" and rigid_contact_solver != "none":
        errors.append(
            f"{packet_name}: {RESOLVED_SOLVER_IDENTITY_KEY}."
            "rigid_contact_selection 'not_applicable' requires "
            "rigid_contact_solver 'none'"
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
    "avbd": "avbd",
    "sequential-impulse": "sequential_impulse",
    "boxed-lcp": "boxed_lcp",
    "vbd": "vbd",
}

# A non-AVBD public family may still report this exact suffix when a body
# carries the compatibility-only internal AVBD rigid-contact opt-in.
_REPORT_AVBD_CONTACT_OPT_IN_SUFFIX = " + avbd (opt-in)"


def resolved_rigid_contact_solver_from_report(resolved_family: str) -> str:
    """Map a report rigid-contact ``resolved`` family string to the packet enum.

    ``resolved_family`` is the string the C++ report records for the
    ``rigid-contact`` domain (e.g. ``"avbd"``, ``"sequential-impulse"``,
    ``"boxed-lcp"``, or a ``"... + avbd (opt-in)"`` substitution). Raises
    ``ValueError`` for an empty or unrecognized family so a report-vocabulary
    change cannot silently produce an out-of-contract packet enum.
    """
    text = resolved_family.strip().lower()
    if not text:
        raise ValueError("resolved rigid-contact family must be non-empty")
    if text.endswith(_REPORT_AVBD_CONTACT_OPT_IN_SUFFIX):
        base_family = text[: -len(_REPORT_AVBD_CONTACT_OPT_IN_SUFFIX)]
        if base_family not in ("boxed-lcp", "sequential-impulse"):
            raise ValueError(
                "unrecognized AVBD opt-in base rigid-contact family "
                f"{base_family!r}; expected 'boxed-lcp' or "
                "'sequential-impulse'"
            )
        return "avbd"
    try:
        return _REPORT_RIGID_CONTACT_FAMILY_TO_PACKET[text]
    except KeyError:
        raise ValueError(
            f"unrecognized resolved rigid-contact family {resolved_family!r}; "
            f"known: {sorted(_REPORT_RIGID_CONTACT_FAMILY_TO_PACKET)} "
            f"(plus an '{_REPORT_AVBD_CONTACT_OPT_IN_SUFFIX}' suffix)"
        ) from None


def make_resolved_solver_identity(
    *,
    resolved_rigid_contact_family: str | None,
    rigid_point_joint_solver: str,
    avbd_rigid_contact_config_emplaced: bool,
    recorded_from: str,
    rigid_contact_selection: str | None = None,
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

    if rigid_contact_selection is None:
        if rigid_contact_solver == "none":
            rigid_contact_selection = "not_applicable"
        elif rigid_contact_solver in ("boxed_lcp", "sequential_impulse"):
            rigid_contact_selection = "contact_solver_method"
        elif avbd_rigid_contact_config_emplaced:
            rigid_contact_selection = "body_opt_in"

    identity: dict[str, Any] = {
        "avbd_rigid_contact_config_emplaced": bool(avbd_rigid_contact_config_emplaced),
        "recorded_from": recorded_from,
        RIGID_CONTACT_SELECTION_KEY: rigid_contact_selection,
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
