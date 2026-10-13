"""Open chain of soft links with native recorded-state playback."""

import dartpy as dart

from .soft_bodies import _build


def build() -> dart.viewer.SceneSetup:
    return _build(
        "dart://sample/skel/soft_open_chain.skel", [2.7, -2.7, 1.9], [0.0, 0.0, 0.25]
    )


def scenes():
    return [
        dart.viewer.Scene(
            "soft_open_chain",
            title="Soft Open Chain",
            category="Soft Bodies",
            summary="Open chain of soft links with recorded-state playback.",
            factory=build,
        )
    ]
