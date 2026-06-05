#!/usr/bin/env python3
"""Helper to invoke cmake --build with consistent options."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

try:
    from parallel_jobs import compute_parallel_jobs, ninja_load_args
except ImportError:  # pragma: no cover

    def compute_parallel_jobs() -> int:
        cpus = os.cpu_count() or 1
        if cpus <= 8:
            jobs = cpus
        else:
            jobs = max(1, (cpus * 3) // 4)
        # Avoid exhausting process limits on beefy self-hosted runners.
        return min(jobs, 32)

    def ninja_load_args() -> list[str]:
        return []


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--build-dir",
        help=(
            "Path to the CMake build tree. When omitted, falls back to "
            "CMAKE_BUILD_DIR or build/$PIXI_ENVIRONMENT_NAME/cpp/$BUILD_TYPE."
        ),
    )
    parser.add_argument(
        "--config",
        help="Multi-config generator configuration (e.g., Release)",
    )
    parser.add_argument(
        "--parallel",
        type=int,
        help="Override the parallel job count",
    )
    parser.add_argument(
        "--target",
        action="append",
        default=[],
        help="Target(s) to build. Can be provided multiple times.",
    )
    return parser.parse_args()


def resolve_parallel(explicit: int | None) -> int:
    if explicit:
        return explicit
    env_value = os.environ.get("DART_PARALLEL_JOBS")
    if env_value:
        try:
            return int(env_value)
        except ValueError:
            pass
    return compute_parallel_jobs()


def resolve_build_dir(value: str | None) -> Path:
    """Determine the build directory from CLI args or environment."""
    if value:
        return Path(value)
    env_default = os.environ.get("CMAKE_BUILD_DIR")
    if env_default:
        return Path(env_default)
    pixi_env = os.environ.get("PIXI_ENVIRONMENT_NAME")
    build_type = os.environ.get("BUILD_TYPE")
    if pixi_env and build_type:
        return Path("build") / pixi_env / "cpp" / build_type
    raise SystemExit(
        "--build-dir is required. Provide it via CLI, set CMAKE_BUILD_DIR, "
        "or ensure PIXI_ENVIRONMENT_NAME and BUILD_TYPE are defined."
    )


def refresh_stale_nanobind_static_archive(build_dir: Path) -> None:
    """Force stale nanobind static archives to rebuild after package updates.

    Some package installs preserve nanobind source mtimes from the wheel. If the
    pixi environment updates nanobind in place, Ninja can keep a newer static
    archive that was compiled from older sources while the updated headers now
    reference newer module lifecycle hooks.
    """

    nm = shutil.which("nm")
    if not nm:
        return

    try:
        import nanobind  # type: ignore
    except Exception:
        return

    nb_root = Path(nanobind.__file__).resolve().parent
    nb_internals_cpp = nb_root / "src" / "nb_internals.cpp"
    nb_internals_h = nb_root / "src" / "nb_internals.h"
    if not nb_internals_cpp.is_file():
        return

    try:
        internals_text = nb_internals_cpp.read_text(encoding="utf-8")
    except OSError:
        return

    required_symbols = ("nb_module_traverse", "nb_module_clear")
    if not all(symbol in internals_text for symbol in required_symbols):
        return

    archives = sorted(build_dir.rglob("libnanobind-static*.a"))
    for archive in archives:
        result = subprocess.run(
            [nm, "-C", str(archive)],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        if result.returncode != 0:
            continue

        defined_symbols = {
            symbol
            for line in result.stdout.splitlines()
            if " T " in line or " t " in line
            for symbol in required_symbols
            if symbol in line
        }
        if all(symbol in defined_symbols for symbol in required_symbols):
            continue

        nb_internals_cpp.touch()
        if nb_internals_h.is_file():
            nb_internals_h.touch()
        print(
            "Detected stale nanobind static archive; touched "
            f"{nb_internals_cpp} so CMake rebuilds {archive}.",
            file=sys.stderr,
        )
        return


def run_build(
    build_dir: Path, config: str | None, parallel: int, target: str | None
) -> None:
    def _pipe_external_logs() -> None:
        """Print useful external project logs when available for debugging."""
        stamp_dir = (
            build_dir / "test" / "FAKE_INSTALL-prefix" / "src" / "FAKE_INSTALL-stamp"
        )
        # Only print the logs we know are small enough to surface in CI.
        if not stamp_dir.exists():
            return

        for log_path in sorted(stamp_dir.glob("FAKE_INSTALL-*-*.log")):
            try:
                lines = log_path.read_text(errors="replace").splitlines()
            except OSError as exc:  # pragma: no cover - best-effort logging
                print(f"Note: failed to read {log_path}: {exc}", file=sys.stderr)
                continue

            tail = "\n".join(lines[-400:])
            print(f"\n----- Begin log: {log_path} -----\n{tail}\n----- End log -----\n")

    cmd = ["cmake", "--build", str(build_dir)]
    if config:
        cmd += ["--config", config]
    cmd += ["--parallel", str(parallel)]
    if target:
        cmd += ["--target", target]
    # Load-aware scheduling: pass ninja's `-l` so concurrent clones share the
    # CPU. Guard on a Ninja build tree since `--parallel`'s passthrough goes to
    # the native tool and `-l` is meaningless to MSBuild/Xcode.
    if (build_dir / "build.ninja").is_file():
        load_args = ninja_load_args()
        if load_args:
            cmd += ["--", *load_args]
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError:
        _pipe_external_logs()
        raise


def main() -> int:
    args = parse_args()
    build_dir = resolve_build_dir(args.build_dir)
    refresh_stale_nanobind_static_archive(build_dir)
    parallel = resolve_parallel(args.parallel)
    targets = args.target or [None]
    for target in targets:
        run_build(build_dir, args.config, parallel, target)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
