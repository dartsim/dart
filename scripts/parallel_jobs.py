#!/usr/bin/env python3
"""Compute build parallelism for pixi tasks."""
from __future__ import annotations

import os


def compute_parallel_jobs() -> int:
    cpus = os.cpu_count() or 1
    if cpus <= 8:
        return cpus
    return max(1, (cpus * 3) // 4)


def main() -> int:
    print(compute_parallel_jobs())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
