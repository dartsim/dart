#!/usr/bin/env python3
"""Minimal example that exercises the nanobind-powered math constants."""

from __future__ import annotations

import math

import dartpy_nb


def main() -> None:
    math_mod = dartpy_nb.math
    print("dartpy_nb.math.pi:", math_mod.pi)
    demo_angle = 45.0
    radians = math_mod.deg2rad(demo_angle)
    print(f"deg2rad({demo_angle}) = {radians}")
    print("Matches math.radians?", math.isclose(radians, math.radians(demo_angle)))


if __name__ == "__main__":
    main()
