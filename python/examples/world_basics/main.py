#!/usr/bin/env python3
"""Minimal dartpy_nb example showing World management."""

from __future__ import annotations

import dartpy_nb as dart


def make_single_body() -> dart.dynamics.Skeleton:
    """Create a skeleton with a single free joint/body pair."""

    skeleton = dart.dynamics.Skeleton()
    skeleton.setName("demo_skeleton")

    joint_props = dart.dynamics.FreeJointProperties()
    joint_props.mName = "root_joint"
    _, body = skeleton.createFreeJointAndBodyNodePair(None, joint_props)
    body.setName("base")
    return skeleton


def main() -> None:
    world = dart.simulation.World("nanobind_world")
    world.setGravity(0.0, 0.0, -9.81)
    world.setTimeStep(0.001)
    world.addSkeleton(make_single_body())

    print("Initial gravity:", world.getGravity())
    print("World contains", world.getNumSkeletons(), "skeleton(s)")

    for _ in range(5):
        world.step()
        print(f"time={world.getTime():.4f}",
              f"frames={world.getSimFrames()}")

    world.reset()
    print("World reset -> time:", world.getTime())


if __name__ == "__main__":
    main()
