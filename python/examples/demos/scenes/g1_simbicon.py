"""G1 SIMBICON: the Unitree G1 humanoid balancing/stepping with SIMBICON.

Demonstrates that the robot-agnostic SIMBICON controller
(`_simbicon.SimbiconController`) drives a second, very different humanoid (the
G1 is far lighter and smaller than Atlas) from the same control law with only a
per-robot config. The G1 model is fetched from its public ROS package and
cached locally on first run.
"""

from __future__ import annotations

from ..runner import PythonDemoScene, SceneSetup
from ._simbicon_robots import (
    build_simbicon_setup,
    load_g1_skeleton,
    make_g1_config,
    make_simbicon_panel,
)


def build() -> SceneSetup:
    world, controllers, pre_step = build_simbicon_setup(
        [(load_g1_skeleton, make_g1_config(), 0.0)]
    )
    setup = SceneSetup(
        world=world,
        panels=[make_simbicon_panel("G1 SIMBICON", controllers)],
        info={"robot": "g1"},
    )
    setup.pre_step = pre_step
    return setup


SCENE = PythonDemoScene(
    id="g1_simbicon",
    title="G1 SIMBICON",
    category="Control & IK",
    summary="Unitree G1 humanoid balancing in place under a SIMBICON controller.",
    build=build,
)
