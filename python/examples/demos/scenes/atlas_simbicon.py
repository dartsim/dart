"""Atlas SIMBICON: the Atlas v5 humanoid balancing/stepping with SIMBICON.

Uses the robot-agnostic SIMBICON controller (`_simbicon.SimbiconController`)
with the Atlas role/gain config. See `_simbicon.py` for the algorithm.
"""

from __future__ import annotations

from ..runner import PythonDemoScene, SceneSetup
from ._simbicon_robots import (
    build_simbicon_setup,
    load_atlas_skeleton,
    make_atlas_config,
)


def build() -> SceneSetup:
    world, _controllers, pre_step = build_simbicon_setup(
        [(load_atlas_skeleton, make_atlas_config(), 0.0)]
    )
    setup = SceneSetup(world=world, info={"robot": "atlas"})
    setup.pre_step = pre_step
    return setup


SCENE = PythonDemoScene(
    id="atlas_simbicon",
    title="Atlas SIMBICON",
    category="Control & IK",
    summary="Atlas humanoid balancing in place under a SIMBICON controller.",
    build=build,
)
