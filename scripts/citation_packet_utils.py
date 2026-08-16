"""Shared helpers for PLAN-123 citation evidence packets.

Centralizes the typed-unsupported markers for quantities DART 7 `main` does
not currently report, so no packet can quietly publish a sentinel zero as if
it were a measurement. Each marker names the exact code path that makes the
quantity unavailable, so a reader can check the claim and a later slice can
delete the marker once WS4 exposes the value.
"""

from __future__ import annotations

from typing import Any

# `recordSolverDiagnostics(World&, std::size_t iterations, double residual =
# 0.0)` in dart/simulation/compute/rigid_body_contact_stage.cpp is called from
# every rigid contact path without a residual argument, so
# StepMetrics.last_step_residual is structurally 0.0 on this path -- "not
# computed", never "converged to zero".
UNSUPPORTED_SOLVER_RESIDUAL: dict[str, str] = {
    "status": "unsupported",
    "reason": (
        "StepMetrics.last_step_residual is structurally zero for rigid "
        "contact on this branch: recordSolverDiagnostics() in "
        "dart/simulation/compute/rigid_body_contact_stage.cpp takes "
        "residual = 0.0 by default and no rigid contact call site passes "
        "one, so no residual is computed for either contact solver. "
        "Exposing a comparable residual is PLAN-123 WS4 work."
    ),
}

# The BoxedLcp branch (rigid_body_contact_stage.cpp, `if
# (world.getContactSolverMethod() == ContactSolverMethod::BoxedLcp)`) returns
# before any recordSolverDiagnostics() call, so a 0 iteration count means
# "not recorded", not "solved in zero iterations".
UNSUPPORTED_BOXED_LCP_ITERATIONS: dict[str, str] = {
    "status": "unsupported",
    "reason": (
        "The BoxedLcp branch in "
        "dart/simulation/compute/rigid_body_contact_stage.cpp returns before "
        "recordSolverDiagnostics() runs, so this scene recorded no iteration "
        "count for the method: a 0 here means 'not reported', not 'zero "
        "iterations'. (An opt-in AVBD stage earlier in the same step can "
        "record a count before that branch is reached; this scene enables "
        "none, and a nonzero count is published with its provenance instead "
        "of this marker.)"
    ),
}

# The sequential-impulse path records the *configured* iteration count
# (`recordSolverDiagnostics(world, m_iterations)`), and its Gauss-Seidel loop
# runs a fixed number of sweeps with no convergence exit, so the number is a
# setting echoed back rather than an observed convergence measurement.
SEQUENTIAL_IMPULSE_ITERATIONS_NOTE = (
    "Sequential impulse records the configured iteration count "
    "(recordSolverDiagnostics(world, m_iterations)); its Gauss-Seidel loop "
    "has no convergence exit, so this is the configured sweep count, not an "
    "observed iteration-to-convergence."
)

# StepMetrics.max_penetration_depth is accumulated as
# `std::max(metrics.maxPenetrationDepth, std::max(0.0, contact.depth))`, so it
# is one-sided: 0.0 means "no positive penetration observed" and cannot be
# distinguished from "contacts reported non-positive depth".
PENETRATION_CLAMP_NOTE = (
    "StepMetrics.max_penetration_depth clamps each contact depth with "
    "std::max(0.0, depth), so a reported 0.0 means no positive penetration "
    "was observed and does not distinguish resting-exactly-tangent from "
    "separated contacts."
)


# A ratio against a peak of exactly zero is undefined, not zero: with no
# signal there is nothing for the antisymmetry residual to be relative to.
UNSUPPORTED_ANTISYMMETRY_RATIO: dict[str, str] = {
    "status": "unsupported",
    "reason": (
        "Peak lateral drift is exactly zero for this method, so the "
        "antisymmetry-to-peak ratio is undefined rather than zero."
    ),
}


def solver_iterations_by_method(
    iterations_by_method: dict[str, int],
) -> dict[str, Any]:
    """Type per-method iteration counts, marking unrecorded ones.

    A zero is the runtime's "nothing was recorded" sentinel: the BoxedLcp
    branch returns before `recordSolverDiagnostics()`, and the
    sequential-impulse call is skipped when a step assembles no constraints.
    Either way an absent count is emitted as a typed-unsupported marker.
    A genuinely recorded count is published as measured, whichever method
    produced it -- keying off the method name instead would launder a real
    AVBD-sourced count into "unsupported".
    """
    typed: dict[str, Any] = {}
    for method, value in sorted(iterations_by_method.items()):
        if isinstance(value, int) and value > 0:
            typed[method] = value
        elif method == "BOXED_LCP":
            typed[method] = dict(UNSUPPORTED_BOXED_LCP_ITERATIONS)
        else:
            typed[method] = {
                "status": "unsupported",
                "reason": (
                    "No iteration count was recorded for this method in this "
                    "scene: StepMetrics.last_step_iterations stayed at its "
                    "per-step reset value, which means 'not reported' rather "
                    "than 'zero iterations'."
                ),
            }
    return typed


# PLAN-123's evidence lands through this pull request; its head ref survives
# branch deletion and squash-merge, which is what makes recorded target
# commits reproducible from a clean checkout.
CITATION_PR_NUMBER = 3445


def target_fetch_hint() -> str:
    """How a clean checkout reaches a packet's target.commit forever."""
    return (
        f"git fetch origin pull/{CITATION_PR_NUMBER}/head && "
        "git checkout <target.commit>"
    )


def packet_content_digest(packet: dict[str, Any]) -> str:
    """Digest of a packet minus its review block (the validator's algorithm).

    A review pass binds to this digest, so regenerating a packet with
    different content invalidates prior passes instead of carrying them onto
    evidence they never reviewed.
    """
    import hashlib as _hashlib
    import json as _json

    content = {key: value for key, value in packet.items() if key != "review"}
    return (
        "sha256:"
        + _hashlib.sha256(
            _json.dumps(content, sort_keys=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest()
    )


def preserve_review(
    output_path: "Any", new_packet: dict[str, Any] | None = None
) -> dict[str, Any]:
    """Carry forward review passes that still review THIS packet.

    Packets are generated, but review passes are recorded by people and other
    agents after the fact. A pass is kept only when its `content_digest`
    matches the regenerated packet's content: reviews of an earlier packet
    (different target commit, scene, metrics, or conclusion) are dropped so
    the two-review floor cannot be satisfied by evidence nobody re-reviewed.
    Passing `new_packet=None` (legacy call) keeps nothing, which fails closed.
    """
    try:
        import json as _json
        from pathlib import Path as _Path

        existing = _json.loads(_Path(output_path).read_text(encoding="utf-8"))
    except OSError, ValueError:
        return {"passes": []}
    review = existing.get("review")
    if not (isinstance(review, dict) and isinstance(review.get("passes"), list)):
        return {"passes": []}
    if new_packet is None:
        return {"passes": []}
    expected = packet_content_digest(new_packet)
    kept = [
        entry
        for entry in review["passes"]
        if isinstance(entry, dict) and entry.get("content_digest") == expected
    ]
    return {"passes": kept}


def record_review_pass(packet_path: "Any", reviewer: str, summary: str) -> None:
    """Append a review pass bound to the packet's current content digest."""
    import json as _json
    from pathlib import Path as _Path

    path = _Path(packet_path)
    packet = _json.loads(path.read_text(encoding="utf-8"))
    review = packet.setdefault("review", {})
    passes = review.setdefault("passes", [])
    passes.append(
        {
            "reviewer": reviewer,
            "summary": summary,
            "verdict": "pass",
            "content_digest": packet_content_digest(packet),
        }
    )
    path.write_text(
        _json.dumps(packet, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def world_resolved_configuration(world: Any) -> dict[str, Any]:
    """Record the World's own bake-time solver resolution.

    This is the authoritative answer to "which method actually ran": the
    World reports, per domain, what was requested, what it resolved to, and
    why. Echoing back the requested option cannot distinguish a method that
    ran from one that was silently substituted.
    """
    resolved = world.resolved_configuration
    return {
        "source": "World.resolved_configuration (bake-time resolution)",
        "has_substitution": bool(resolved.has_substitution()),
        "by_domain": {
            note.domain: {
                "requested": note.requested,
                "resolved": note.resolved,
                "reason": note.reason,
                "is_substitution": bool(note.is_substitution),
            }
            for note in resolved.notes
        },
    }
