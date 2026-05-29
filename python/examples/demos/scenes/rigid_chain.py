"""Rigid-chain scene: a damped articulated chain loaded from chain.skel.

Mirrors the C++ `examples/demos/scenes/rigid_chain.cpp` scene so the
golden-set parity smoke (PLAN-103 Phase 2) can assert both languages
produce the same simulated state. The initial pose is a deterministic
damped sine across the chain's DOFs, matching the C++ counterpart.
"""

from __future__ import annotations

import math

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup

_CHAIN_URI = "dart://sample/skel/chain.skel"
_CHAIN_NAME = "rigid_chain"


def build() -> SceneSetup:
    world = dart.io.SkelParser.read_world(_CHAIN_URI)
    if world is None:
        raise RuntimeError(f"Failed to load {_CHAIN_URI}")

    world.set_gravity([0.0, -9.81, 0.0])
    world.set_time_step(1.0 / 2000.0)

    chain = world.get_skeleton(0)
    if chain is None:
        raise RuntimeError("Rigid chain world did not contain a skeleton")
    chain.set_name(_CHAIN_NAME)

    num_dofs = chain.get_num_dofs()
    initial_pose = [0.4 * math.sin(0.7 * i) for i in range(num_dofs)]
    chain.set_positions(initial_pose)

    return SceneSetup(world=world, info={"golden_skeletons": [_CHAIN_NAME]})


SCENE = PythonDemoScene(
    id="rigid_chain",
    title="Rigid Chain",
    category="Rigid Body",
    summary="A damped articulated chain loaded from chain.skel.",
    build=build,
)
