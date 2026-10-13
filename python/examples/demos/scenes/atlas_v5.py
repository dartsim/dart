"""Fixed-base inspection of the verified Atlas v5 bundle."""

import dartpy as dart

from .modern_humanoids import build_atlas


def scenes():
    return [
        dart.viewer.Scene(
            "atlas_v5",
            title="Atlas v5 (No Head)",
            category="Robots",
            summary="Inspect a verified cached humanoid and pose its joints.",
            factory=build_atlas,
        )
    ]
