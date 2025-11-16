#!/usr/bin/env python3
"""Demonstrate dartpy_nb screw joint configuration."""

from __future__ import annotations

import math

import dartpy_nb as dart
import numpy as np


def build_screw_joint_skeleton() -> dart.dynamics.Skeleton:
    skel = dart.dynamics.Skeleton("screw_demo")
    props = dart.dynamics.ScrewJointProperties()
    props.mAxis = np.array([0.0, 0.0, 1.0])
    joint, body = skel.createScrewJointAndBodyNodePair(None, props)
    joint.setPitch(0.25)
    body.setName("link")
    return skel


def main() -> None:
    skeleton = build_screw_joint_skeleton()
    joint = skeleton.getJoint(0)
    dof = skeleton.getDof(0)

    joint.setAxis(np.array([1.0, 0.0, 0.0]))
    joint.setPitch(math.pi / 8.0)

    dof.setPosition(0.1)
    dof.setVelocity(0.2)

    print("Screw joint axis:", joint.getAxis())
    print("Pitch:", joint.getPitch())
    print(
        "Pose (position, velocity):",
        dof.getPosition(),
        dof.getVelocity(),
    )


if __name__ == "__main__":
    main()
