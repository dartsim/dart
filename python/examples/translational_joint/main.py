#!/usr/bin/env python3
"""Simple translational joint demo using dartpy_nb."""

from __future__ import annotations

import dartpy_nb as dart
import numpy as np


def build_system() -> dart.dynamics.Skeleton:
    skeleton = dart.dynamics.Skeleton("translational_demo")
    props = dart.dynamics.TranslationalJointProperties()
    joint, body = skeleton.createTranslationalJointAndBodyNodePair(None, props)
    joint.setPositions(np.array([0.0, 0.0, 0.0]))
    body.setName("slider")
    return skeleton


def main() -> None:
    skeleton = build_system()
    joint = skeleton.getJoint(0)

    # Move along each axis
    offsets = np.array([0.2, -0.1, 0.3])
    joint.setPositions(offsets)

    print("Joint type:", joint.getType())
    print("Current offsets:", joint.getPositions())
    print("Jacobian at pose:\n", joint.getRelativeJacobianStatic(offsets))


if __name__ == "__main__":
    main()
