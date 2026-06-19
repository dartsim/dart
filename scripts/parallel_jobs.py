#!/usr/bin/env python3
"""Single source of truth for DART build/test parallelism.

Two knobs are exported:

* ``compute_parallel_jobs()``: how many compile/link jobs to launch.
* ``compute_load_limit()``: the OS load ceiling for load-aware scheduling.

Both honor environment overrides and an optional host-local
``~/.dart-dev/parallelism.env`` file (``KEY=VALUE`` lines) so every clone on a
workstation can share one policy. Environment variables always win over the
file. See ``docs/ai/verification.md`` for the release-branch gate commands that
call this helper.
"""

from __future__ import annotations

import os
from pathlib import Path

_DART_DEV_ENV_LOADED = False


def _running_under_ci() -> bool:
    return bool(os.getenv("GITHUB_ACTIONS") or os.getenv("CI"))


def load_dart_dev_env() -> None:
    """Merge ``~/.dart-dev/parallelism.env`` into the environment once.

    Existing environment variables are never overwritten, so an explicit
    ``DART_PARALLEL_JOBS=...`` on the command line still wins. CI never reads
    the file, keeping automated runs hermetic.
    """
    global _DART_DEV_ENV_LOADED
    if _DART_DEV_ENV_LOADED or _running_under_ci():
        return
    _DART_DEV_ENV_LOADED = True

    env_file = Path.home() / ".dart-dev" / "parallelism.env"
    try:
        lines = env_file.read_text(encoding="utf-8").splitlines()
    except (OSError, ValueError):
        return

    for raw in lines:
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key:
            os.environ.setdefault(key, value)


def _env_int(name: str) -> int | None:
    raw = os.environ.get(name)
    if not raw:
        return None
    try:
        return int(raw.strip())
    except ValueError:
        return None


def compute_parallel_jobs() -> int:
    """Return the compile/link job count for build and test commands."""
    load_dart_dev_env()

    override = _env_int("DART_PARALLEL_JOBS")
    if override and override > 0:
        return override

    cpus = os.cpu_count() or 1

    # CI runners advertise high CPU counts, but spawning that many build jobs
    # frequently exhausts process slots. Cap automated runs aggressively.
    if os.getenv("GITHUB_ACTIONS"):
        return min(16, cpus)

    if cpus <= 8:
        jobs = cpus
    else:
        jobs = max(1, (cpus * 3) // 4)

    # Avoid exhausting process limits on large self-hosted machines.
    return min(jobs, 32)


def compute_load_limit() -> int | None:
    """Return the OS load ceiling for ninja ``-l`` and ctest ``--test-load``.

    Returns ``None`` when load-aware scheduling should be disabled, so callers
    add no flag and preserve historical CI behavior.
    """
    load_dart_dev_env()

    raw = os.environ.get("DART_BUILD_LOAD_LIMIT")
    if raw is not None:
        token = raw.strip().lower()
        if token in {"0", "off", "none", "false", ""}:
            return None
        try:
            value = int(token)
            return value if value > 0 else None
        except ValueError:
            return None

    if _running_under_ci():
        return None

    return (os.cpu_count() or 1) + 2


def ninja_load_args() -> list[str]:
    """Return native-tool passthrough args for ninja's load limit."""
    limit = compute_load_limit()
    return ["-l", str(limit)] if limit else []


def main() -> int:
    print(compute_parallel_jobs())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
