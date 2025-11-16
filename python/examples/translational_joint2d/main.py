#!/usr/bin/env python3
"""Demonstrate dartpy_nb TranslationalJoint2D planes."""

from __future__ import annotations

import dartpy_nb as dart
import numpy as np


def main() -> None:
    skeleton = dart.dynamics.Skeleton("trans2d_demo")
    joint, body = skeleton.createTranslationalJoint2DAndBodyNodePair()
    body.setName("plate")

    joint.setXYPlane(renameDofs=True)
    joint.setPositions(np.array([0.05, -0.05]))

    print("Initial offsets:", joint.getPositions())
    joint.setArbitraryPlane(np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0]), True)
    joint.setPositions(np.array([0.02, 0.04]))
    print("Arbitrary plane offsets:", joint.getPositions())


if __name__ == "__main__":
    main()
