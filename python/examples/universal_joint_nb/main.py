#!/usr/bin/env python3
"""Universal joint demo using dartpy_nb."""

from __future__ import annotations

import dartpy_nb as dart
import numpy as np


def main() -> None:
    skeleton = dart.dynamics.Skeleton("universal_demo")
    joint, body = skeleton.createUniversalJointAndBodyNodePair()
    body.setName("link")

    joint.setAxis1(np.array([1.0, 0.0, 0.0]))
    joint.setAxis2(np.array([0.0, 1.0, 0.0]))
    joint.setPositions(np.array([0.2, 0.1]))

    print("Axis1:", joint.getAxis1())
    print("Axis2:", joint.getAxis2())
    print("Positions:", joint.getPositions())
    print("Jacobian:\n", joint.getRelativeJacobianStatic(np.array([0.2, 0.1])))


if __name__ == "__main__":
    main()
