"""Fixed-base inspection of the verified Unitree G1 bundle."""

import dartpy as dart

from .modern_humanoids import build_g1


def scenes():
    return [
        dart.viewer.Scene(
            "unitree_g1",
            title="Unitree G1 (29 DOF)",
            category="Robots",
            summary="Inspect a verified cached humanoid and pose its joints.",
            factory=build_g1,
        )
    ]
