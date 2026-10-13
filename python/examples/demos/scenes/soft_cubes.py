"""Soft cube stack with native recorded-state playback."""

import dartpy as dart

from .soft_bodies import _build


def build() -> dart.viewer.SceneSetup:
    return _build(
        "dart://sample/skel/soft_cubes.skel", [2.5, -2.8, 2.0], [0.0, 0.0, 0.35]
    )


def scenes():
    return [
        dart.viewer.Scene(
            "soft_cubes",
            title="Soft Cubes",
            category="Soft Bodies",
            summary="Soft cube stack with recorded-state playback.",
            factory=build,
        )
    ]
