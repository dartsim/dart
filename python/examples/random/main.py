#!/usr/bin/env python3
"""Showcase Random bindings from dartpy_nb."""

from __future__ import annotations

import dartpy_nb


def main() -> None:
    rng = dartpy_nb.math.Random()
    dartpy_nb.math.Random.setSeed(2025)
    values = [dartpy_nb.math.Random.uniform(-5.0, 5.0) for _ in range(3)]
    print("Seed:", dartpy_nb.math.Random.getSeed())
    print("Sampled values:", values)


if __name__ == "__main__":
    main()
