#!/usr/bin/env python3
"""Inspect and tweak a DegreeOfFreedom via dartpy_nb."""

from __future__ import annotations

import math

import dartpy_nb as dart
import numpy as np


def build_skeleton() -> dart.dynamics.Skeleton:
    skeleton = dart.dynamics.Skeleton("demo_dof")

    joint_props = dart.dynamics.RevoluteJointProperties()
    joint_props.mName = "hinge"
    joint_props.mAxis = np.array([0.0, 0.0, 1.0])
    skeleton.createRevoluteJointAndBodyNodePair(None, joint_props)
    return skeleton


def main() -> None:
    skeleton = build_skeleton()
    dof = skeleton.getDof(0)
    dof.setName("yaw")
    dof.setPositionLimits(-math.pi / 2.0, math.pi / 2.0)
    dof.setVelocityLimits(-2.0, 2.0)

    dof.setPosition(0.25)
    dof.setVelocity(0.5)
    dof.setForce(3.0)

    print(f"Skeleton: {skeleton.getName()}")
    print(f"DOF name: {dof.getName()}")
    print("Position limits:", dof.getPositionLimits())
    print("Velocity limits:", dof.getVelocityLimits())
    print(f"Current state pos={dof.getPosition():.3f} vel={dof.getVelocity():.3f}")
    print("Applied torque:", dof.getForce())


if __name__ == "__main__":
    main()
