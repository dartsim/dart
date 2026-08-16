"""Shared helpers for release-6.20 citation evidence packets.

Centralizes the typed-unsupported markers for quantities DART 6.20 does not
report, so no packet can quietly publish a sentinel zero as a measurement.
Each marker names why the quantity is unavailable on this branch.

Additive tooling only: no library, API, ABI, or default change.
"""

from __future__ import annotations

from typing import Any

# BoxedLcpConstraintSolver exposes neither an iteration count nor a residual
# per solve on this branch, and the Dantzig-primary/PGS-secondary fallback
# raises no observable event.
UNSUPPORTED_SOLVER_ITERATIONS: dict[str, str] = {
    "status": "unsupported",
    "reason": (
        "release-6.20 exposes no per-solve LCP iteration count: "
        "BoxedLcpConstraintSolver has no getNumIterations()-style accessor, "
        "so an iteration number cannot be recorded without changing public "
        "API, which the LTS compatibility contract forbids."
    ),
}

UNSUPPORTED_SOLVER_RESIDUAL: dict[str, str] = {
    "status": "unsupported",
    "reason": (
        "release-6.20 exposes no per-solve LCP residual: the boxed-LCP "
        "solvers report no convergence measure through public API, so a "
        "residual cannot be recorded without an API change the LTS "
        "compatibility contract forbids."
    ),
}

UNSUPPORTED_FALLBACK_EVENTS: dict[str, str] = {
    "status": "unsupported",
    "reason": (
        "The Dantzig-primary/PGS-secondary fallback inside "
        "BoxedLcpConstraintSolver emits no observable event on this branch, "
        "so per-solve fallback occurrences cannot be counted."
    ),
}

# A ratio against a peak of exactly zero is undefined, not zero.
UNSUPPORTED_ANTISYMMETRY_RATIO: dict[str, str] = {
    "status": "unsupported",
    "reason": (
        "Peak lateral drift is exactly zero for this detector, so the "
        "antisymmetry-to-peak ratio is undefined rather than zero."
    ),
}


# PLAN-623's evidence lands through this pull request; its head ref survives
# branch deletion and squash-merge, which is what makes recorded target
# commits reproducible from a clean checkout.
CITATION_PR_NUMBER = 3444


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
    output_path: Any, new_packet: dict[str, Any] | None = None
) -> dict[str, Any]:
    """Carry forward review passes that still review THIS packet.

    Packets are generated, but review passes are recorded by people and other
    agents afterwards. A pass is kept only when its `content_digest` matches
    the regenerated packet's content: reviews of an earlier packet (different
    target commit, scene, metrics, or conclusion) are dropped so the
    two-review floor cannot be satisfied by evidence nobody re-reviewed.
    Passing `new_packet=None` (legacy call) keeps nothing, which fails closed.
    """
    try:
        import json as _json
        from pathlib import Path as _Path

        existing = _json.loads(_Path(output_path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
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


def record_review_pass(packet_path: Any, reviewer: str, summary: str) -> None:
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
            "content_digest": packet_content_digest(packet),
        }
    )
    path.write_text(
        _json.dumps(packet, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
