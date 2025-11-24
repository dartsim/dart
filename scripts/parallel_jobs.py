#!/usr/bin/env python3
"""Compute build parallelism for pixi tasks."""
from __future__ import annotations

import os


def compute_parallel_jobs() -> int:
    cpus = os.cpu_count() or 1

    # CI runners advertise many CPUs but have tight process limits. Use a single
    # job on GitHub-hosted runners to avoid posix_spawn failures, but keep local
    # parallelism intact.
    if os.getenv("GITHUB_ACTIONS"):
        return 1

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
