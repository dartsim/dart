"""Mixed chain scene: a 10-body mixed rigid/soft articulated chain.

Mirrors examples/demos/scenes/mixed_chain.cpp. The C++ scene seeds the
first 3 DOFs from a uniform RNG; the Python mirror uses a deterministic
small sine pattern so the headless run is reproducible.
"""

from __future__ import annotations

import math

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup

_URI = "dart://sample/skel/test/test_articulated_bodies_10bodies.skel"
_CHAIN_NAME = "mixed_chain"


def build() -> SceneSetup:
    world = dart.io.SkelParser.read_world(_URI)
    if world is None:
        raise RuntimeError(f"Failed to load {_URI}")
    world.set_gravity([0.0, -9.81, 0.0])

    chain = world.get_skeleton(1)
    if chain is None:
        raise RuntimeError("Mixed chain world did not contain a chain")
    chain.set_name(_CHAIN_NAME)

    num_dofs = chain.get_num_dofs()
    pose = [0.0] * num_dofs
    for i in range(min(3, num_dofs)):
        pose[i] = 0.3 * math.sin(0.5 * i)
    chain.set_positions(pose)

    return SceneSetup(world=world, info={"golden_skeletons": [_CHAIN_NAME]})


SCENE = PythonDemoScene(
    id="mixed_chain",
    title="Mixed Chain",
    category="Soft Bodies",
    summary="A 10-body mixed rigid/soft articulated chain.",
    build=build,
)
