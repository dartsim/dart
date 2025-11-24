#!/usr/bin/env python3
"""Compute build parallelism for pixi tasks."""
from __future__ import annotations

import os


def compute_parallel_jobs() -> int:
    cpus = os.cpu_count() or 1

    # CI runners advertise very high CPU counts, but spawning that many build
    # jobs frequently exhausts process slots (posix_spawn failures). Cap
    # aggressively when running under GitHub Actions to keep builds stable.
    if os.getenv("GITHUB_ACTIONS"):
        return min(16, cpus)

    if cpus <= 8:
        jobs = cpus
    else:
        jobs = max(1, (cpus * 3) // 4)
    # Avoid exhausting process limits on beefy self-hosted runners.
    return min(jobs, 32)


def main() -> int:
    print(compute_parallel_jobs())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
