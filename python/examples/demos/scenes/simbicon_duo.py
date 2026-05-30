"""SIMBICON Duo: Atlas and Unitree G1 balancing side by side in one world.

Both humanoids are driven by the *same* robot-agnostic SIMBICON controller
(`_simbicon.SimbiconController`), each with its own per-robot config, in a
single shared world -- demonstrating that the control law generalizes across
very different humanoids.
"""

from __future__ import annotations

from ..runner import PythonDemoScene, SceneSetup
from ._simbicon_robots import (
    build_simbicon_setup,
    load_atlas_skeleton,
    load_g1_skeleton,
    make_atlas_config,
    make_g1_config,
)


def build() -> SceneSetup:
    world, _controllers, pre_step = build_simbicon_setup(
        [
            (load_atlas_skeleton, make_atlas_config(), -0.8),
            (load_g1_skeleton, make_g1_config(), 0.8),
        ]
    )
    setup = SceneSetup(world=world, info={"robots": ["atlas", "g1"]})
    setup.pre_step = pre_step
    return setup


SCENE = PythonDemoScene(
    id="simbicon_duo",
    title="SIMBICON Duo (Atlas + G1)",
    category="Control & IK",
    summary="Atlas and Unitree G1 balancing together under one SIMBICON law.",
    build=build,
)
