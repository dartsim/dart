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


def preserve_review(output_path: Any) -> dict[str, Any]:
    """Carry forward review passes already recorded in a packet.

    Packets are generated, but review passes are recorded by people and other
    agents afterwards. Without this, every regeneration erases them.
    """
    try:
        import json as _json
        from pathlib import Path as _Path

        existing = _json.loads(_Path(output_path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {"passes": []}
    review = existing.get("review")
    if isinstance(review, dict) and isinstance(review.get("passes"), list):
        return {"passes": review["passes"]}
    return {"passes": []}
