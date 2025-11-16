#!/usr/bin/env python3
"""Quick demo of dartpy_nb math geometry helpers."""

from __future__ import annotations

import numpy as np

import dartpy_nb


def main() -> None:
    angles = np.array([0.2, 0.4, -0.1])
    rot = dartpy_nb.math.eulerZYXToMatrix(angles)
    print("Rotation matrix:\n", rot)
    recovered = dartpy_nb.math.matrixToEulerZYX(rot)
    print("Recovered angles:", recovered)
    twist = np.zeros(6)
    pose = dartpy_nb.math.expMap(twist)
    print("expMap(0) =\n", pose.matrix())


if __name__ == "__main__":
    main()
