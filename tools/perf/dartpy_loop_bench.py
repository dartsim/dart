#!/usr/bin/env python3
"""Measure a DART 6 style Python simulation loop with pyperf."""

import math
from time import perf_counter

import dartpy as dart
import pyperf

SCENE = "dart://sample/skel/cubes.skel"
WARMUP_STEPS = 1000
STEPS = 1000


def make_world():
    world = dart.utils.SkelParser.readWorld(SCENE)
    if world is None:
        raise RuntimeError(f"Cannot load {SCENE}")
    world.getConstraintSolver().setCollisionDetector(
        dart.collision.DARTCollisionDetector()
    )
    world.setNumSimulationThreads(1)
    options = world.getDeactivationOptions()
    options.mEnabled = False
    world.setDeactivationOptions(options)
    for _ in range(WARMUP_STEPS):
        world.step()
    assert world.getNumSkeletons() == 4
    assert world.getLastCollisionResult().getNumContacts() > 0
    return world


def step_and_read(world):
    world.step()
    checksum = 0.0
    for s in range(world.getNumSkeletons()):
        skeleton = world.getSkeleton(s)
        for b in range(skeleton.getNumBodyNodes()):
            checksum += skeleton.getBodyNode(b).getWorldTransform().matrix().sum()
    for contact in world.getLastCollisionResult().getContacts():
        checksum += (
            contact.point.sum()
            + contact.normal.sum()
            + contact.force.sum()
            + contact.penetrationDepth
        )
    return checksum


def bench_loop(loops):
    elapsed = 0.0
    for _ in range(loops):
        world = make_world()
        checksum = 0.0
        start = perf_counter()
        for _ in range(STEPS):
            checksum += step_and_read(world)
        elapsed += perf_counter() - start
        assert math.isfinite(checksum)
        assert world.getLastCollisionResult().getNumContacts() > 0
        assert world.getSimFrames() == WARMUP_STEPS + STEPS
    return elapsed


def main():
    runner = pyperf.Runner()
    runner.bench_time_func(
        "dartpy_step_read_cubes",
        bench_loop,
        inner_loops=STEPS,
        metadata={
            "dartpy_version": dart.__version__ or "unknown",
            "scene": SCENE,
            "detector": "dart",
            "simulation_threads": 1,
            "time_step": 0.001,
            "warmup_steps": WARMUP_STEPS,
            "measurement_steps": STEPS,
            "deactivation": "disabled",
        },
    )


if __name__ == "__main__":
    main()
