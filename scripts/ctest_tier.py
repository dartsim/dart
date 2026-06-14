#!/usr/bin/env python3
"""Run a ctest selection with DART's shared parallelism, load limit, and timeout.

Centralizes test-running ergonomics for the tiered ``verify-*`` / ``test-*``
tasks (see ``docs/design/local_verification_pipeline.md``):

* parallel execution with a per-test ``--timeout`` (so a hung test cannot stall
  a whole tier),
* load-aware ``--test-load`` so concurrent clones share the CPU,
* ``--rerun-failed`` for the inner loop.

Local runs default to the shared parallelism policy. CI runs stay **sequential**
(matching historical behavior) unless ``CTEST_PARALLEL_LEVEL`` or ``--jobs`` is
set, which keeps heavy integration tests from oversubscribing constrained
runners.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

try:
    from parallel_jobs import compute_load_limit, compute_parallel_jobs
except ImportError:  # pragma: no cover - defensive fallback

    def compute_parallel_jobs() -> int:
        return max(os.cpu_count() or 1, 1)

    def compute_load_limit() -> int | None:
        return os.cpu_count() or 1


# Experimental tests are fast on average but have a long tail; the Tier-1 quick
# subset skips these so it stays inside its budget. They run at the full tier.
# These mirror the heavy tests the build itself special-cases under DART_CODECOV
# (tests/unit/simulation/experimental/CMakeLists.txt) — test_rigid_ipc_paper_
# experiments alone runs ~350s and otherwise bounds the whole parallel suite.
EXPERIMENTAL_LONG_POLES = (
    "^test_world$",
    "^test_variational_integration$",
    "^test_rigid_ipc_fixture$",
    "^test_rigid_ipc_paper_experiments$",
    "^test_deformable_body$",
)

DEFAULT_TIMEOUT = 600


def _running_ci() -> bool:
    return bool(os.getenv("GITHUB_ACTIONS") or os.getenv("CI"))


def resolve_build_dir(build_type: str) -> tuple[Path, str | None]:
    """Return ``(test_dir, build_config)`` for the configured generator layout.

    Single-config generators (Ninja/Make, used on Linux/macOS) place the cache
    in ``build/<env>/cpp/<BuildType>``. Multi-config generators (Visual Studio,
    Xcode) place it in ``build/<env>/cpp`` and choose the configuration at test
    time via ``--build-config``. Detect which is configured and mirror the
    per-platform ``test`` / ``test-simulation`` tasks, so the tiered runners
    work under both layouts -- otherwise a multi-config tree (e.g. win-64), which
    has no ``/<BuildType>`` subdirectory, looks unbuilt to this script.
    """
    env_name = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    base = Path("build") / env_name / "cpp"
    single_config = base / build_type
    if (single_config / "CMakeCache.txt").is_file():
        return single_config, None
    if (base / "CMakeCache.txt").is_file():
        return base, build_type
    # Nothing configured yet: keep the single-config layout for a clear error.
    return single_config, None


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-type", default="Release")
    parser.add_argument("-L", "--label", action="append", default=[])
    parser.add_argument("-LE", "--label-exclude", action="append", default=[])
    parser.add_argument("-R", "--tests-regex")
    parser.add_argument("-E", "--exclude-regex")
    parser.add_argument(
        "--exclude-long-poles",
        action="store_true",
        help="Skip the DART 7 simulation long-pole tests (Tier-1 quick subset)",
    )
    parser.add_argument("--timeout", type=int, default=DEFAULT_TIMEOUT)
    parser.add_argument("--jobs", "-j", type=int, default=None)
    parser.add_argument("--rerun-failed", action="store_true")
    return parser.parse_args(argv)


def resolve_jobs(explicit: int | None) -> int | None:
    if explicit and explicit > 0:
        return explicit
    if os.environ.get("CTEST_PARALLEL_LEVEL"):
        return None  # defer to the environment variable ctest already honors
    return 1 if _running_ci() else compute_parallel_jobs()


def main(argv: list[str]) -> int:
    args = parse_args(argv)

    build_dir, build_config = resolve_build_dir(args.build_type)
    if not build_dir.exists():
        raise SystemExit(
            f"Build directory {build_dir} does not exist. Build the tests first."
        )

    cmd = ["ctest", "--test-dir", str(build_dir), "--output-on-failure"]
    if build_config:
        cmd += ["--build-config", build_config]

    jobs = resolve_jobs(args.jobs)
    if jobs:
        cmd += ["--parallel", str(jobs)]
    load_limit = compute_load_limit()
    if load_limit:
        cmd += ["--test-load", str(load_limit)]
    if args.timeout and args.timeout > 0:
        cmd += ["--timeout", str(args.timeout)]

    # --rerun-failed selects from the previous run; selection flags conflict.
    # Fall back to a normal selection when there is no prior failure log.
    rerun = args.rerun_failed
    if (
        rerun
        and not (build_dir / "Testing" / "Temporary" / "LastTestsFailed.log").is_file()
    ):
        print(
            "warning: --rerun-failed found no prior failures; "
            "running the full selection instead.",
            file=sys.stderr,
        )
        rerun = False

    if rerun:
        cmd.append("--rerun-failed")
    else:
        for label in args.label:
            cmd += ["-L", label]
        for label in args.label_exclude:
            cmd += ["-LE", label]
        if args.tests_regex:
            cmd += ["-R", args.tests_regex]
        excludes = [args.exclude_regex] if args.exclude_regex else []
        if args.exclude_long_poles:
            excludes.extend(EXPERIMENTAL_LONG_POLES)
        if excludes:
            cmd += ["-E", "|".join(f"({e})" for e in excludes)]

    return subprocess.run(cmd).returncode


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
