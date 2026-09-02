#!/usr/bin/env python3
"""Build, quiet-gate, watchdog, and run the three Figure 13 benchmarks."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import socket
import stat
import subprocess
import sys
import tempfile
import time
import uuid
from collections.abc import Mapping
from datetime import datetime, timezone
from pathlib import Path

from capture_source_provenance import (
    DART_LIBRARY_BUILD_IDENTITY_ALGORITHM,
    CaptureSourceProvenanceError,
    compute_capture_source_provenance,
    dart_library_build_identity,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
TARGET = "bm_avbd_rigid_fixed_joint"
FILTER = "^BM_(Avbd|Vbd|SequentialImpulse)PaperBreakableWallStep/iterations:120$"
SCHEMA_VERSION = "dart.figure13_benchmark_run/v3"
BUILD_IDENTITY_ALGORITHM = "sha256-canonical-compiled-benchmark-identity-v3"
BUILD_CONFIGURATION_ALGORITHM = "sha256-cmake-build-configuration-record-v2"
LOADER_POLICY_ALGORITHM = "empty-loader-control-environment-v1"
RUNTIME_IMAGE_INVENTORY_ALGORITHM = "sha256-complete-mapped-elf-inventory-v1"
DEFAULT_QUIET_SECONDS = 120.0
DEFAULT_SAMPLE_INTERVAL_SECONDS = 1.0
DEFAULT_MAX_NORMALIZED_LOAD = 0.25
MIN_WARMUP_SECONDS = 1.0
EVIDENCE_BUILD_MARKER = "DART_FIGURE13_EVIDENCE_BUILD:BOOL=ON"
# Flag variables the project extends in scope after the caller's cache value;
# the compiled record may carry those project-appended tokens.
PROJECT_APPENDED_FLAG_KEYS = frozenset(
    {
        "CMAKE_CXX_FLAGS",
        "CMAKE_EXE_LINKER_FLAGS",
        "CMAKE_MODULE_LINKER_FLAGS",
        "CMAKE_SHARED_LINKER_FLAGS",
    }
)
EVIDENCE_CMAKE_DEFINITIONS = (
    "BUILD_SHARED_LIBS=ON",
    "BUILD_TESTING=ON",
    "CMAKE_BUILD_TYPE=Release",
    "CMAKE_CXX_FLAGS=",
    "CMAKE_CXX_FLAGS_RELEASE=-O3 -DNDEBUG",
    "CMAKE_EXE_LINKER_FLAGS=",
    "CMAKE_EXE_LINKER_FLAGS_RELEASE=",
    "CMAKE_EXPORT_COMPILE_COMMANDS=ON",
    "CMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF",
    "CMAKE_MODULE_LINKER_FLAGS=",
    "CMAKE_MODULE_LINKER_FLAGS_RELEASE=",
    "CMAKE_SHARED_LINKER_FLAGS=",
    "CMAKE_SHARED_LINKER_FLAGS_RELEASE=",
    "DARTPY_DEBUG_SYMBOLS=OFF",
    "DART_BUILD_COLLISION_REFERENCE_BENCHMARKS=OFF",
    "DART_BUILD_COLLISION_REFERENCE_TESTS=OFF",
    "DART_BUILD_DARTPY=OFF",
    "DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS=OFF",
    "DART_BUILD_DIFF=OFF",
    "DART_BUILD_EXAMPLES=OFF",
    "DART_BUILD_GUI=OFF",
    "DART_BUILD_IO_USD=OFF",
    "DART_BUILD_MEMORY_DIAGNOSTICS=OFF",
    "DART_BUILD_PROFILE=OFF",
    "DART_BUILD_TESTS=ON",
    "DART_BUILD_TUTORIALS=OFF",
    "DART_CODECOV=OFF",
    "DART_COLLISION_DEPRECATE_LEGACY_NAMES=ON",
    "DART_ENABLE_ASAN=OFF",
    "DART_ENABLE_EXPERIMENTAL_CUDA=OFF",
    "DART_ENABLE_GUI_FILAMENT_SMOKE_TESTS=OFF",
    "DART_ENABLE_SDFORMAT=OFF",
    "DART_ENABLE_SIMD=OFF",
    "DART_FAST_DEBUG=OFF",
    "DART_FIGURE13_EVIDENCE_BUILD=ON",
    "DART_FORCE_COLORED_OUTPUT=OFF",
    "DART_PROFILE_BUILTIN=OFF",
    "DART_PROFILE_TRACY=OFF",
    "DART_SIMD_FORCE_SCALAR=OFF",
    "DART_SIMULATION_VERBOSE=OFF",
    "DART_STRICT_SYMBOL_VISIBILITY=OFF",
    "DART_USE_MOLD=OFF",
    "DART_USE_SYSTEM_BULLET=ON",
    "DART_USE_SYSTEM_FILAMENT=ON",
    "DART_USE_SYSTEM_FMT=ON",
    "DART_USE_SYSTEM_GOOGLEBENCHMARK=ON",
    "DART_USE_SYSTEM_GOOGLETEST=ON",
    "DART_USE_SYSTEM_IMGUI=OFF",
    "DART_USE_SYSTEM_NANOBIND=OFF",
    "DART_USE_SYSTEM_ODE=ON",
    "DART_USE_SYSTEM_TRACY=OFF",
    "DART_VERBOSE=OFF",
)
BUILD_CONFIGURATION_KEYS = (
    "CMAKE_GENERATOR",
    "CMAKE_GENERATOR_PLATFORM",
    "CMAKE_GENERATOR_TOOLSET",
    "CMAKE_CXX_COMPILER",
    "CMAKE_CXX_COMPILER_ID",
    "CMAKE_CXX_COMPILER_VERSION",
    "CMAKE_CXX_COMPILER_TARGET",
    "CMAKE_CXX_COMPILER_EXTERNAL_TOOLCHAIN",
    "CMAKE_CXX_COMPILER_FRONTEND_VARIANT",
    "CMAKE_CXX_COMPILER_LAUNCHER",
    "CMAKE_SYSTEM_NAME",
    "CMAKE_SYSTEM_VERSION",
    "CMAKE_SYSTEM_PROCESSOR",
    "CMAKE_BUILD_TYPE",
    "CMAKE_CONFIGURATION_TYPES",
    "CMAKE_TOOLCHAIN_FILE",
    "CMAKE_SYSROOT",
    "CMAKE_CXX_FLAGS",
    "CMAKE_CXX_FLAGS_RELEASE",
    "CMAKE_EXE_LINKER_FLAGS",
    "CMAKE_EXE_LINKER_FLAGS_RELEASE",
    "CMAKE_SHARED_LINKER_FLAGS",
    "CMAKE_SHARED_LINKER_FLAGS_RELEASE",
    "CMAKE_MODULE_LINKER_FLAGS",
    "CMAKE_MODULE_LINKER_FLAGS_RELEASE",
    "CMAKE_LINKER_TYPE",
    "CMAKE_INTERPROCEDURAL_OPTIMIZATION",
    "BUILD_SHARED_LIBS",
    "BUILD_TESTING",
    "DARTPY_DEBUG_SYMBOLS",
    "DART_BUILD_COLLISION_REFERENCE_BENCHMARKS",
    "DART_BUILD_COLLISION_REFERENCE_TESTS",
    "DART_BUILD_DARTPY",
    "DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS",
    "DART_BUILD_DIFF",
    "DART_BUILD_EXAMPLES",
    "DART_BUILD_GUI",
    "DART_BUILD_IO_USD",
    "DART_BUILD_MEMORY_DIAGNOSTICS",
    "DART_BUILD_PROFILE",
    "DART_BUILD_TESTS",
    "DART_BUILD_TUTORIALS",
    "DART_CODECOV",
    "DART_COLLISION_DEPRECATE_LEGACY_NAMES",
    "DART_COMPILER_CACHE",
    "DART_ENABLE_ASAN",
    "DART_ENABLE_EXPERIMENTAL_CUDA",
    "DART_ENABLE_GUI_FILAMENT_SMOKE_TESTS",
    "DART_ENABLE_SDFORMAT",
    "DART_ENABLE_SIMD",
    "DART_FAST_DEBUG",
    "DART_FIGURE13_EVIDENCE_BUILD",
    "DART_FORCE_COLORED_OUTPUT",
    "DART_NORMALIZE_BUILD_PATHS",
    "DART_PROFILE_BUILTIN",
    "DART_PROFILE_TRACY",
    "DART_SIMD_FORCE_SCALAR",
    "DART_SIMULATION_VERBOSE",
    "DART_STRICT_SYMBOL_VISIBILITY",
    "DART_USE_MOLD",
    "DART_USE_SYSTEM_BULLET",
    "DART_USE_SYSTEM_FILAMENT",
    "DART_USE_SYSTEM_FMT",
    "DART_USE_SYSTEM_GOOGLEBENCHMARK",
    "DART_USE_SYSTEM_GOOGLETEST",
    "DART_USE_SYSTEM_IMGUI",
    "DART_USE_SYSTEM_NANOBIND",
    "DART_USE_SYSTEM_ODE",
    "DART_USE_SYSTEM_TRACY",
    "DART_VERBOSE",
)
BUILD_PROCESS_NAMES = {
    "c++",
    "cc",
    "clang",
    "clang++",
    "cmake",
    "gcc",
    "g++",
    "ld",
    "ld.lld",
    "ninja",
}
BENCHMARK_ENVIRONMENT_PREFIXES = ("BENCHMARK_",)
LOADER_ENVIRONMENT_PREFIXES = ("DYLD_", "LD_")
UNRECORDED_BUILD_ENVIRONMENT_VARIABLES = (
    "CCACHE_PREFIX",
    "CMAKE_CXX_COMPILER_LAUNCHER",
    "CMAKE_LINKER_TYPE",
    "DART_COMPILER_CACHE",
    "DART_NORMALIZE_BUILD_PATHS",
    "GLIBC_TUNABLES",
)
REQUIRED_RUNTIME_IMAGE_ROLES = (
    "dynamic_loader",
    "google_benchmark",
    "libc",
    "libm",
    "libstdcxx",
)


class Figure13BenchmarkError(RuntimeError):
    """Raised when benchmark evidence cannot pass its run gates."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_digest(value: object) -> str:
    return hashlib.sha256(
        json.dumps(
            value, ensure_ascii=False, separators=(",", ":"), sort_keys=True
        ).encode("utf-8")
    ).hexdigest()


def _reject_duplicate_json_keys(
    pairs: list[tuple[str, object]],
) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise Figure13BenchmarkError(f"benchmark JSON repeats object key {key!r}")
        result[key] = value
    return result


def _strict_json_loads(payload: str) -> object:
    return json.loads(payload, object_pairs_hook=_reject_duplicate_json_keys)


def _utc_now() -> str:
    return (
        datetime.now(timezone.utc)
        .isoformat(timespec="microseconds")
        .replace("+00:00", "Z")
    )


def _loader_environment_variables(
    environment: Mapping[str, str] | None = None,
) -> list[str]:
    values = os.environ if environment is None else environment
    return sorted(
        name
        for name, value in values.items()
        if value and name.startswith(LOADER_ENVIRONMENT_PREFIXES)
    )


def _reject_loader_injection_environment() -> dict[str, object]:
    present = _loader_environment_variables()
    if present:
        raise Figure13BenchmarkError(
            "Figure 13 evidence forbids loader-control environment variables: "
            f"{present!r}"
        )
    return {
        "algorithm": LOADER_POLICY_ALGORITHM,
        "forbidden_environment_prefixes": list(LOADER_ENVIRONMENT_PREFIXES),
        "passed": True,
        "present_environment_variables": [],
    }


def _unrecorded_build_environment_variables(
    environment: Mapping[str, str] | None = None,
) -> list[str]:
    values = os.environ if environment is None else environment
    return sorted(
        name
        for name, value in values.items()
        if value
        and (
            name in UNRECORDED_BUILD_ENVIRONMENT_VARIABLES
            or name.startswith(BENCHMARK_ENVIRONMENT_PREFIXES)
        )
    )


def _reject_unrecorded_build_environment() -> None:
    present = _unrecorded_build_environment_variables()
    if present:
        raise Figure13BenchmarkError(
            "Figure 13 evidence forbids environment variables the recorded "
            f"build configuration does not describe: {present!r}"
        )


def _sanitized_subprocess_environment() -> dict[str, str]:
    environment = os.environ.copy()
    prefixes = BENCHMARK_ENVIRONMENT_PREFIXES + LOADER_ENVIRONMENT_PREFIXES
    for name in list(environment):
        if name.startswith(prefixes) or name in UNRECORDED_BUILD_ENVIRONMENT_VARIABLES:
            environment.pop(name)
    return environment


def _build_dir(build_type: str, run_token: str) -> Path:
    environment = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    return (
        REPO_ROOT / "build" / environment / "figure13-evidence" / run_token / build_type
    )


def _configuration_record(values: dict[str, str]) -> str:
    if set(values) != set(BUILD_CONFIGURATION_KEYS):
        raise Figure13BenchmarkError(
            "build configuration record has an unexpected key set"
        )
    return "".join(
        [f"algorithm={BUILD_CONFIGURATION_ALGORITHM}\n"]
        + [f"{key}={values[key]}\n" for key in BUILD_CONFIGURATION_KEYS]
    )


def _parse_cmake_cache(path: Path) -> dict[str, str]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as exc:
        raise Figure13BenchmarkError(
            f"cannot read evidence CMake cache: {exc}"
        ) from exc
    values: dict[str, str] = {}
    for line in lines:
        if not line or line.startswith(("#", "//")) or ":" not in line:
            continue
        name_and_type, separator, value = line.partition("=")
        if not separator:
            continue
        name, type_separator, _cache_type = name_and_type.partition(":")
        if not type_separator or not name:
            continue
        if name in values:
            raise Figure13BenchmarkError(
                f"evidence CMake cache repeats variable {name!r}"
            )
        values[name] = value
    return values


def _load_build_configuration(build_dir: Path) -> dict[str, object]:
    cache = _parse_cmake_cache(build_dir / "CMakeCache.txt")
    for definition in EVIDENCE_CMAKE_DEFINITIONS:
        name, expected = definition.split("=", maxsplit=1)
        if cache.get(name) != expected:
            raise Figure13BenchmarkError(
                f"evidence CMake cache {name} must be {expected!r}"
            )
    if cache.get("CMAKE_GENERATOR") != "Ninja":
        raise Figure13BenchmarkError("evidence CMake generator must be Ninja")

    manifest = build_dir / "generated" / "dart" / "capture_build_configuration.txt"
    try:
        record = manifest.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as exc:
        raise Figure13BenchmarkError(
            f"cannot read compiled build configuration record: {exc}"
        ) from exc
    entries: dict[str, str] = {}
    for line in record.splitlines():
        name, separator, value = line.partition("=")
        if not separator or not name or name in entries:
            raise Figure13BenchmarkError(
                "compiled build configuration record is malformed"
            )
        entries[name] = value
    if entries.pop("algorithm", None) != BUILD_CONFIGURATION_ALGORITHM:
        raise Figure13BenchmarkError(
            "compiled build configuration algorithm does not match"
        )
    if set(entries) != set(BUILD_CONFIGURATION_KEYS):
        raise Figure13BenchmarkError(
            "compiled build configuration record has an unexpected key set"
        )
    if record != _configuration_record(entries):
        raise Figure13BenchmarkError(
            "compiled build configuration record is not canonical"
        )
    for definition in EVIDENCE_CMAKE_DEFINITIONS:
        name, expected = definition.split("=", maxsplit=1)
        if name not in BUILD_CONFIGURATION_KEYS:
            continue
        compiled = entries.get(name)
        if compiled == expected:
            continue
        # The cache proves the caller injected no flags; the project itself
        # appends link options in scope (for example `-Wl,--no-undefined` for
        # shared libraries), which the compiled record must keep verbatim
        # because its digest binds the exact flags the benchmark was built
        # with. Accept only such project-appended suffixes.
        if (
            name in PROJECT_APPENDED_FLAG_KEYS
            and isinstance(compiled, str)
            and compiled.startswith(expected)
            and compiled[len(expected) :].strip()
        ):
            continue
        raise Figure13BenchmarkError(
            f"compiled build configuration {name} must be {expected!r}"
        )
    if entries.get("CMAKE_GENERATOR") != "Ninja":
        raise Figure13BenchmarkError(
            "compiled build configuration generator must be Ninja"
        )
    for name in (
        "CMAKE_CXX_COMPILER",
        "CMAKE_CXX_COMPILER_ID",
        "CMAKE_CXX_COMPILER_VERSION",
        "CMAKE_SYSTEM_NAME",
        "CMAKE_SYSTEM_PROCESSOR",
    ):
        if entries.get(name) in (None, "", "<UNDEFINED>"):
            raise Figure13BenchmarkError(
                f"compiled build configuration {name} must be resolved"
            )
    return {
        "algorithm": BUILD_CONFIGURATION_ALGORITHM,
        "digest": hashlib.sha256(record.encode("utf-8")).hexdigest(),
        "values": entries,
    }


def _configure_evidence_build(build_dir: Path) -> dict[str, object]:
    if build_dir.exists():
        raise Figure13BenchmarkError(
            f"fresh Figure 13 evidence build directory already exists: {build_dir}"
        )
    prefix = os.environ.get("CONDA_PREFIX")
    if not prefix:
        raise Figure13BenchmarkError(
            "Figure 13 evidence configure requires the Pixi CONDA_PREFIX"
        )
    build_dir.parent.mkdir(parents=True, exist_ok=True)
    environment = _sanitized_subprocess_environment()
    for name in (
        "CFLAGS",
        "CPPFLAGS",
        "CXXFLAGS",
        "LDFLAGS",
        "CMAKE_ARGS",
        "CMAKE_TOOLCHAIN_FILE",
    ):
        environment.pop(name, None)
    command = [
        "cmake",
        "-G",
        "Ninja",
        "-S",
        str(REPO_ROOT),
        "-B",
        str(build_dir),
        f"-DCMAKE_INSTALL_PREFIX={prefix}",
        f"-DCMAKE_PREFIX_PATH={prefix}",
        f"-DPython3_EXECUTABLE={sys.executable}",
        *(f"-D{definition}" for definition in EVIDENCE_CMAKE_DEFINITIONS),
    ]
    subprocess.run(command, check=True, cwd=REPO_ROOT, env=environment)
    cache = build_dir / "CMakeCache.txt"
    if not cache.is_file() or EVIDENCE_BUILD_MARKER not in cache.read_text(
        encoding="utf-8"
    ):
        raise Figure13BenchmarkError(
            "fresh Figure 13 configure did not record its evidence-build marker"
        )
    return _load_build_configuration(build_dir)


def _find_binary(build_dir: Path) -> Path:
    candidates = (
        build_dir / "bin" / TARGET,
        build_dir / "tests" / "benchmark" / TARGET,
        build_dir / "tests" / "benchmark" / "simulation" / TARGET,
    )
    for candidate in candidates:
        if candidate.is_file() and not candidate.is_symlink():
            return candidate.resolve()
    raise Figure13BenchmarkError(
        f"built benchmark executable {TARGET!r} was not found below {build_dir}"
    )


def _build_target(build_dir: Path) -> Path:
    if not (build_dir / "CMakeCache.txt").is_file():
        raise Figure13BenchmarkError(
            "Figure 13 evidence tree was not configured by this runner"
        )
    environment = _sanitized_subprocess_environment()
    environment["CMAKE_BUILD_DIR"] = str(build_dir)
    subprocess.run(
        [
            sys.executable,
            str(REPO_ROOT / "scripts" / "cmake_build.py"),
            "--build-dir",
            str(build_dir),
            "--target",
            TARGET,
        ],
        check=True,
        cwd=REPO_ROOT,
        env=environment,
    )
    return _find_binary(build_dir)


def _foreign_build_processes() -> list[dict[str, object]]:
    if not sys.platform.startswith("linux"):
        return []
    own_pid = os.getpid()
    processes: list[dict[str, object]] = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == own_pid:
            continue
        try:
            command = (entry / "cmdline").read_bytes().split(b"\0")
        except FileNotFoundError, PermissionError, ProcessLookupError:
            continue
        if not command or not command[0]:
            continue
        executable = Path(os.fsdecode(command[0])).name.lower()
        if executable not in BUILD_PROCESS_NAMES:
            continue
        processes.append(
            {
                "command": " ".join(
                    os.fsdecode(argument) for argument in command[:4] if argument
                ),
                "pid": int(entry.name),
            }
        )
    return processes


def _load_snapshot() -> dict[str, object]:
    cpu_count = os.cpu_count() or 1
    try:
        load_one = os.getloadavg()[0]
    except AttributeError, OSError:
        load_one = 0.0
    return {
        "foreign_build_processes": _foreign_build_processes(),
        "load_one": load_one,
        "normalized_load_one": load_one / cpu_count,
    }


def _decode_proc_maps_path(value: str) -> str:
    return (
        value.replace(r"\134", "\\")
        .replace(r"\040", " ")
        .replace(r"\011", "\t")
        .replace(r"\012", "\n")
    )


def _mapped_elf_image_paths(pid: int) -> dict[Path, tuple[int, int, int]]:
    if not sys.platform.startswith("linux"):
        raise Figure13BenchmarkError(
            "Figure 13 runtime-image inventory currently requires Linux /proc"
        )
    try:
        lines = Path(f"/proc/{pid}/maps").read_text(encoding="utf-8").splitlines()
    except (FileNotFoundError, PermissionError, ProcessLookupError) as exc:
        raise Figure13BenchmarkError(
            f"cannot inventory benchmark runtime images: {exc}"
        ) from exc
    paths: dict[Path, tuple[int, int, int]] = {}
    for line in lines:
        fields = line.split(maxsplit=5)
        if len(fields) != 6 or not fields[5].startswith("/"):
            continue
        if fields[5].endswith(" (deleted)"):
            raise Figure13BenchmarkError(
                "benchmark mapped an image that was deleted during its run"
            )
        raw_path = Path(_decode_proc_maps_path(fields[5]))
        try:
            path = raw_path.resolve(strict=True)
            with path.open("rb") as file:
                is_elf = file.read(4) == b"\x7fELF"
        except (OSError, RuntimeError, ValueError) as exc:
            raise Figure13BenchmarkError(
                f"cannot inspect mapped benchmark image {raw_path}: {exc}"
            ) from exc
        if not is_elf:
            continue
        try:
            major_text, minor_text = fields[3].split(":", maxsplit=1)
            identity = (
                int(major_text, 16),
                int(minor_text, 16),
                int(fields[4]),
            )
        except ValueError as exc:
            raise Figure13BenchmarkError(
                f"cannot decode mapped benchmark image identity: {line!r}"
            ) from exc
        previous = paths.setdefault(path, identity)
        if previous != identity:
            raise Figure13BenchmarkError(
                f"benchmark mapped multiple binary images at {path}"
            )
    return paths


def _mapped_binary_image_record(
    path: Path, expected_identity: tuple[int, int, int]
) -> dict[str, object]:
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        descriptor = os.open(path, flags)
    except OSError as exc:
        raise Figure13BenchmarkError(
            f"cannot open mapped benchmark image without following links: {path}: {exc}"
        ) from exc
    try:
        before = os.fstat(descriptor)
        actual_identity = (
            os.major(before.st_dev),
            os.minor(before.st_dev),
            before.st_ino,
        )
        if not stat.S_ISREG(before.st_mode) or actual_identity != expected_identity:
            raise Figure13BenchmarkError(
                "mapped benchmark image path no longer identifies the running "
                f"binary image: {path}"
            )
        if not 1 <= before.st_size <= 0xFFFFFFFFFFFFFFFF:
            raise Figure13BenchmarkError(
                f"mapped benchmark image size is invalid: {path}"
            )
        digest = hashlib.sha256()
        first_chunk = True
        while chunk := os.read(descriptor, 1024 * 1024):
            if first_chunk and not chunk.startswith(b"\x7fELF"):
                raise Figure13BenchmarkError(
                    f"mapped benchmark image is not an ELF file: {path}"
                )
            first_chunk = False
            digest.update(chunk)
        after = os.fstat(descriptor)
        if (
            after.st_dev != before.st_dev
            or after.st_ino != before.st_ino
            or after.st_size != before.st_size
            or after.st_mtime_ns != before.st_mtime_ns
            or after.st_ctime_ns != before.st_ctime_ns
        ):
            raise Figure13BenchmarkError(
                f"mapped benchmark image changed while hashing: {path}"
            )
    finally:
        os.close(descriptor)
    try:
        path_stat = path.lstat()
    except OSError as exc:
        raise Figure13BenchmarkError(
            f"cannot revalidate mapped benchmark image path {path}: {exc}"
        ) from exc
    path_identity = (
        os.major(path_stat.st_dev),
        os.minor(path_stat.st_dev),
        path_stat.st_ino,
    )
    if not stat.S_ISREG(path_stat.st_mode) or path_identity != expected_identity:
        raise Figure13BenchmarkError(
            "mapped benchmark image path changed after hashing: " f"{path}"
        )
    return {
        "file": path.name,
        "path": str(path),
        "sha256": digest.hexdigest(),
        "size_bytes": before.st_size,
    }


def _runtime_binary_image_records(
    paths: dict[Path, tuple[int, int, int]],
) -> list[dict[str, object]]:
    if not paths:
        raise Figure13BenchmarkError(
            "benchmark process did not expose any mapped ELF images"
        )
    return [
        _mapped_binary_image_record(path, paths[path])
        for path in sorted(paths, key=str)
    ]


def _runtime_image_roles(images: list[dict[str, object]]) -> dict[str, str]:
    role_matches: dict[str, list[str]] = {
        role: [] for role in REQUIRED_RUNTIME_IMAGE_ROLES
    }
    for image in images:
        name = str(image.get("file", "")).lower()
        path = str(image.get("path", ""))
        if name.startswith(("ld-linux", "ld-musl")) or name == "ld.so":
            role_matches["dynamic_loader"].append(path)
        if name == "libbenchmark.so" or name.startswith("libbenchmark.so."):
            role_matches["google_benchmark"].append(path)
        if name == "libc.so" or name.startswith("libc.so."):
            role_matches["libc"].append(path)
        if name == "libm.so" or name.startswith("libm.so."):
            role_matches["libm"].append(path)
        if name == "libstdc++.so" or name.startswith("libstdc++.so."):
            role_matches["libstdcxx"].append(path)
    missing = [role for role, matches in role_matches.items() if not matches]
    if missing:
        raise Figure13BenchmarkError(
            "benchmark mapped ELF inventory is missing required runtime roles: "
            f"{missing!r}"
        )
    ambiguous = {
        role: matches for role, matches in role_matches.items() if len(matches) != 1
    }
    if ambiguous:
        raise Figure13BenchmarkError(
            "benchmark mapped ELF inventory has ambiguous required runtime roles: "
            f"{ambiguous!r}"
        )
    return {role: role_matches[role][0] for role in REQUIRED_RUNTIME_IMAGE_ROLES}


def _runtime_image_inventory(
    paths: dict[Path, tuple[int, int, int]], *, binary: Path
) -> dict[str, object]:
    images = _runtime_binary_image_records(paths)
    binary = binary.resolve()
    if [image["path"] for image in images].count(str(binary)) != 1:
        raise Figure13BenchmarkError(
            "benchmark mapped ELF inventory must contain the exact executable"
        )
    payload = {
        "images": images,
        "required_roles": _runtime_image_roles(images),
    }
    return {
        "algorithm": RUNTIME_IMAGE_INVENTORY_ALGORITHM,
        **payload,
        "digest": _canonical_digest(payload),
    }


def _validate_runtime_image_inventory(
    value: object, *, binary: Path
) -> dict[str, object]:
    if not isinstance(value, dict) or set(value) != {
        "algorithm",
        "digest",
        "images",
        "required_roles",
    }:
        raise Figure13BenchmarkError(
            "benchmark runtime-image inventory has an unexpected field set"
        )
    if value.get("algorithm") != RUNTIME_IMAGE_INVENTORY_ALGORITHM:
        raise Figure13BenchmarkError(
            "benchmark runtime-image inventory algorithm does not match"
        )
    images = value.get("images")
    if not isinstance(images, list) or not images:
        raise Figure13BenchmarkError(
            "benchmark runtime-image inventory must contain mapped ELF images"
        )
    normalized_images: list[dict[str, object]] = []
    for index, image in enumerate(images):
        if not isinstance(image, dict) or set(image) != {
            "file",
            "path",
            "sha256",
            "size_bytes",
        }:
            raise Figure13BenchmarkError(
                f"benchmark runtime image {index} has an unexpected field set"
            )
        path_value = image.get("path")
        if not isinstance(path_value, str) or not Path(path_value).is_absolute():
            raise Figure13BenchmarkError(
                f"benchmark runtime image {index} path must be absolute"
            )
        candidate = Path(path_value)
        try:
            resolved = candidate.resolve(strict=True)
        except (OSError, RuntimeError, ValueError) as exc:
            raise Figure13BenchmarkError(
                f"cannot resolve benchmark runtime image {index}: {exc}"
            ) from exc
        if candidate.is_symlink() or resolved != candidate or not candidate.is_file():
            raise Figure13BenchmarkError(
                f"benchmark runtime image {index} must be canonical and non-symlink"
            )
        try:
            with candidate.open("rb") as file:
                if file.read(4) != b"\x7fELF":
                    raise Figure13BenchmarkError(
                        f"benchmark runtime image {index} is not an ELF file"
                    )
        except OSError as exc:
            raise Figure13BenchmarkError(
                f"cannot read benchmark runtime image {index}: {exc}"
            ) from exc
        expected = {
            "file": candidate.name,
            "path": str(candidate),
            "sha256": _sha256(candidate),
            "size_bytes": candidate.stat().st_size,
        }
        if image != expected:
            raise Figure13BenchmarkError(
                f"benchmark runtime image {index} bytes or identity changed"
            )
        normalized_images.append(expected)
    paths = [str(image["path"]) for image in normalized_images]
    if paths != sorted(set(paths)):
        raise Figure13BenchmarkError(
            "benchmark runtime images must be unique and path-sorted"
        )
    roles = _runtime_image_roles(normalized_images)
    if value.get("required_roles") != roles:
        raise Figure13BenchmarkError(
            "benchmark runtime-image required-role binding does not match"
        )
    if paths.count(str(binary.resolve())) != 1:
        raise Figure13BenchmarkError(
            "benchmark runtime-image inventory must contain the exact executable"
        )
    payload = {"images": normalized_images, "required_roles": roles}
    if value.get("digest") != _canonical_digest(payload):
        raise Figure13BenchmarkError(
            "benchmark runtime-image inventory digest does not match"
        )
    return {
        "algorithm": RUNTIME_IMAGE_INVENTORY_ALGORITHM,
        **payload,
        "digest": _canonical_digest(payload),
    }


def _dart_library_inventory(
    images: list[dict[str, object]], *, build_dir: Path
) -> list[dict[str, object]]:
    inventory = []
    build_dir = build_dir.resolve()
    for image in images:
        path = Path(str(image["path"]))
        if not path.name.lower().startswith("libdart"):
            continue
        if not path.is_relative_to(build_dir):
            raise Figure13BenchmarkError(
                "benchmark loaded a DART library outside the selected build tree: "
                f"{path}"
            )
        try:
            build_identity = dart_library_build_identity(path)
        except CaptureSourceProvenanceError as exc:
            raise Figure13BenchmarkError(str(exc)) from exc
        inventory.append(
            {
                "build_identity": build_identity,
                **image,
            }
        )
    if not inventory:
        raise Figure13BenchmarkError(
            "benchmark process did not expose any loaded DART shared libraries"
        )
    return inventory


def _require_quiet_snapshot(
    snapshot: dict[str, object], *, max_normalized_load: float, phase: str
) -> None:
    processes = snapshot["foreign_build_processes"]
    normalized = float(snapshot["normalized_load_one"])
    if processes:
        raise Figure13BenchmarkError(
            f"{phase} detected foreign build processes: {processes!r}"
        )
    if normalized > max_normalized_load:
        raise Figure13BenchmarkError(
            f"{phase} normalized one-minute load {normalized:.6f} exceeds "
            f"{max_normalized_load:.6f}"
        )


def _quiet_gate(
    duration_seconds: float,
    interval_seconds: float,
    max_normalized_load: float,
) -> dict[str, object]:
    started_at = _utc_now()
    started_monotonic = time.monotonic()
    deadline = started_monotonic + duration_seconds
    samples: list[dict[str, object]] = []
    while True:
        snapshot = _load_snapshot()
        _require_quiet_snapshot(
            snapshot,
            max_normalized_load=max_normalized_load,
            phase="quiet-host gate",
        )
        samples.append(snapshot)
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break
        time.sleep(min(interval_seconds, remaining))
    return {
        "duration_seconds": duration_seconds,
        "elapsed_seconds": time.monotonic() - started_monotonic,
        "finished_at": _utc_now(),
        "max_normalized_load": max(
            float(sample["normalized_load_one"]) for sample in samples
        ),
        "normalized_load_limit": max_normalized_load,
        "passed": True,
        "sample_count": len(samples),
        "sample_interval_seconds": interval_seconds,
        "started_at": started_at,
    }


def _host_identity() -> dict[str, object]:
    cpu_model = platform.processor().strip()
    if sys.platform.startswith("linux"):
        try:
            for line in Path("/proc/cpuinfo").read_text(encoding="utf-8").splitlines():
                key, separator, value = line.partition(":")
                if separator and key.strip() in {"model name", "Hardware"}:
                    cpu_model = value.strip()
                    break
        except OSError:
            pass
    identity = {
        "cpu_count": os.cpu_count() or 1,
        "cpu_model": cpu_model or "unknown",
        "hostname": socket.gethostname(),
        "machine": platform.machine(),
        "platform": platform.platform(),
        "system": platform.system(),
    }
    return {**identity, "host_token": _canonical_digest(identity)}


def _validate_passthrough(arguments: list[str]) -> None:
    required = {
        f"--benchmark_filter={FILTER}",
        "--benchmark_repetitions=5",
        "--benchmark_report_aggregates_only=true",
        "--benchmark_out_format=json",
        f"--benchmark_min_warmup_time={MIN_WARMUP_SECONDS}",
    }
    if set(arguments) != required:
        raise Figure13BenchmarkError(
            "Figure 13 benchmark arguments must be exactly the three-method "
            "five-repeat aggregate contract"
        )
    if any(argument.startswith("--benchmark_context") for argument in arguments):
        raise Figure13BenchmarkError("caller-injected benchmark context is forbidden")


def _run_with_watchdog(
    binary: Path,
    output: Path,
    arguments: list[str],
    *,
    build_dir: Path,
    interval_seconds: float,
    max_normalized_load: float,
) -> tuple[dict[str, object], dict[str, object], list[dict[str, object]]]:
    started_at = _utc_now()
    started_monotonic = time.monotonic()
    command = [str(binary), *arguments, f"--benchmark_out={output}"]
    process = subprocess.Popen(
        command,
        cwd=REPO_ROOT,
        env=_sanitized_subprocess_environment(),
    )
    samples: list[dict[str, object]] = []
    mapped_image_paths: dict[Path, tuple[int, int, int]] = {}
    first_seen_images: dict[str, dict[str, object]] = {}
    violation: Figure13BenchmarkError | None = None
    while process.poll() is None:
        snapshot = _load_snapshot()
        samples.append(snapshot)
        current_mappings = _mapped_elf_image_paths(process.pid)
        newly_mapped: dict[Path, tuple[int, int, int]] = {}
        for path, identity in current_mappings.items():
            previous = mapped_image_paths.setdefault(path, identity)
            if previous != identity:
                violation = Figure13BenchmarkError(
                    f"benchmark changed its mapped ELF image at {path}"
                )
                process.terminate()
                break
            if str(path) not in first_seen_images:
                newly_mapped[path] = identity
        if violation is None and newly_mapped:
            for image in _runtime_binary_image_records(newly_mapped):
                first_seen_images[str(image["path"])] = image
        try:
            _require_quiet_snapshot(
                snapshot,
                max_normalized_load=max_normalized_load,
                phase="in-run watchdog",
            )
        except Figure13BenchmarkError as exc:
            violation = exc
            process.terminate()
            break
        time.sleep(interval_seconds)
    return_code = process.wait()
    if violation is not None:
        raise violation
    if return_code != 0:
        raise Figure13BenchmarkError(
            f"Figure 13 benchmark exited with status {return_code}"
        )
    final_snapshot = _load_snapshot()
    samples.append(final_snapshot)
    _require_quiet_snapshot(
        final_snapshot,
        max_normalized_load=max_normalized_load,
        phase="in-run watchdog final sample",
    )
    gate = {
        "elapsed_seconds": time.monotonic() - started_monotonic,
        "finished_at": _utc_now(),
        "max_normalized_load": max(
            float(sample["normalized_load_one"]) for sample in samples
        ),
        "normalized_load_limit": max_normalized_load,
        "passed": True,
        "sample_count": len(samples),
        "sample_interval_seconds": interval_seconds,
        "started_at": started_at,
    }
    final_inventory = _runtime_image_inventory(mapped_image_paths, binary=binary)
    first_seen = [first_seen_images[path] for path in sorted(first_seen_images)]
    if not first_seen or final_inventory["images"] != first_seen:
        raise Figure13BenchmarkError(
            "mapped ELF image bytes or identities changed during the run"
        )
    final_libraries = _dart_library_inventory(
        final_inventory["images"], build_dir=build_dir
    )
    return gate, final_inventory, final_libraries


def _write_evidence(
    raw_output: Path,
    final_output: Path,
    *,
    binary: Path,
    binary_sha256: str,
    host_identity: dict[str, object],
    quiet_host: dict[str, object],
    watchdog: dict[str, object],
    loader_environment: dict[str, object],
    runtime_image_inventory: dict[str, object],
    loaded_dart_libraries: list[dict[str, object]],
    build_configuration: dict[str, object],
    run_token: str,
) -> None:
    data = _strict_json_loads(raw_output.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not isinstance(data.get("context"), dict):
        raise Figure13BenchmarkError("benchmark output is missing its context object")
    context = data["context"]
    try:
        context_executable = Path(context["dart_benchmark_executable_path"]).resolve(
            strict=True
        )
    except (KeyError, OSError, RuntimeError, TypeError, ValueError) as exc:
        raise Figure13BenchmarkError(
            f"benchmark did not emit a valid executable identity: {exc}"
        ) from exc
    if context_executable != binary or _sha256(context_executable) != binary_sha256:
        raise Figure13BenchmarkError(
            "running benchmark executable bytes changed or were misreported"
        )
    context_build_configuration_digest = context.get("dart_build_configuration_digest")
    if context_build_configuration_digest != build_configuration.get("digest"):
        raise Figure13BenchmarkError(
            "running benchmark build configuration does not match its fresh "
            "canonical configure record"
        )
    runtime_image_inventory = _validate_runtime_image_inventory(
        runtime_image_inventory, binary=binary
    )
    runtime_images = runtime_image_inventory["images"]
    image_by_path = {
        image.get("path"): image for image in runtime_images if isinstance(image, dict)
    }
    if len(image_by_path) != len(runtime_images):
        raise Figure13BenchmarkError(
            "benchmark runtime-image inventory paths must be unique"
        )
    runtime_dart_paths = {
        image["path"]
        for image in runtime_images
        if str(image["file"]).lower().startswith("libdart")
    }
    declared_dart_paths = {
        library.get("path")
        for library in loaded_dart_libraries
        if isinstance(library, dict)
    }
    if runtime_dart_paths != declared_dart_paths:
        raise Figure13BenchmarkError(
            "loaded DART library inventory must exactly cover every mapped "
            "libdart image"
        )
    for library in loaded_dart_libraries:
        if not isinstance(library, dict):
            raise Figure13BenchmarkError(
                "loaded DART library inventory entries must be objects"
            )
        runtime_image = image_by_path.get(library.get("path"))
        if (
            not isinstance(runtime_image, dict)
            or {
                key: library.get(key)
                for key in ("file", "path", "sha256", "size_bytes")
            }
            != runtime_image
        ):
            raise Figure13BenchmarkError(
                "loaded DART library identity is not bound to the complete "
                f"runtime-image inventory: {library.get('path')}"
            )
        identity = library.get("build_identity")
        if not isinstance(identity, dict):
            raise Figure13BenchmarkError(
                "loaded DART library is missing its compiled build identity"
            )
        expected = {
            "cmake_build_type": context.get("dart_cmake_build_type"),
            "compiler_id": context.get("dart_compiler_id"),
            "compiler_version": context.get("dart_compiler_version"),
            "ndebug": context.get("dart_ndebug") == "1",
            "optimization_enabled": (context.get("dart_optimization_enabled") == "1"),
            "source_git_head": context.get("dart_capture_source_git_head"),
            "source_provenance_digest": context.get(
                "dart_capture_source_provenance_digest"
            ),
            "build_configuration_digest": context_build_configuration_digest,
        }
        for key, value in expected.items():
            if identity.get(key) != value:
                raise Figure13BenchmarkError(
                    "loaded DART library build identity does not match the "
                    f"running benchmark for {library.get('path')}: {key}"
                )
    build_payload = {
        "benchmark_source_sha256": context.get("dart_benchmark_source_sha256"),
        "build_configuration": build_configuration,
        "capture_source_git_head": context.get("dart_capture_source_git_head"),
        "capture_source_provenance_digest": context.get(
            "dart_capture_source_provenance_digest"
        ),
        "cmake_build_type": context.get("dart_cmake_build_type"),
        "compiler_id": context.get("dart_compiler_id"),
        "compiler_version": context.get("dart_compiler_version"),
        "executable_file": binary.name,
        "executable_path": str(binary),
        "executable_sha256": binary_sha256,
        "executable_size_bytes": binary.stat().st_size,
        "loaded_dart_libraries": loaded_dart_libraries,
        "runtime_image_inventory": runtime_image_inventory,
        "ndebug": context.get("dart_ndebug"),
        "optimization_enabled": context.get("dart_optimization_enabled"),
    }
    build_identity = {
        "algorithm": BUILD_IDENTITY_ALGORITHM,
        **build_payload,
        "digest": _canonical_digest(build_payload),
    }
    capture_source = compute_capture_source_provenance(REPO_ROOT)
    run_payload = {
        "benchmark_policy": {
            "filter": FILTER,
            "min_warmup_time_seconds": MIN_WARMUP_SECONDS,
            "repetitions": 5,
            "report_aggregates_only": True,
        },
        "benchmark_context_date": context.get("date"),
        "build_identity": build_identity,
        # The recorded Git HEAD only describes the evidence if nothing in the
        # attested scopes was modified, and only if no ignored file inside a
        # capture root escaped the digest.
        "capture_ignored_paths": capture_source["ignored_paths"],
        "capture_working_tree_clean": capture_source["working_tree_clean"],
        "host_identity": host_identity,
        "host_token": host_identity["host_token"],
        "loader_environment": loader_environment,
        "quiet_host": quiet_host,
        "run_token": run_token,
        "watchdog": watchdog,
    }
    evidence = {
        "schema_version": SCHEMA_VERSION,
        **run_payload,
        "digest": _canonical_digest(run_payload),
    }
    data["dart_evidence_run"] = evidence
    final_output.parent.mkdir(parents=True, exist_ok=True)
    temporary = final_output.with_name(f".{final_output.name}.tmp")
    temporary.write_text(
        json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(final_output)


def parse_args(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
    before, separator, after = argv, [], []
    if "--" in argv:
        index = argv.index("--")
        before, separator, after = argv[:index], ["--"], argv[index + 1 :]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--build-type", default="Release")
    parser.add_argument("--quiet-seconds", type=float, default=DEFAULT_QUIET_SECONDS)
    parser.add_argument(
        "--sample-interval-seconds",
        type=float,
        default=DEFAULT_SAMPLE_INTERVAL_SECONDS,
    )
    parser.add_argument(
        "--max-normalized-load",
        type=float,
        default=DEFAULT_MAX_NORMALIZED_LOAD,
    )
    args = parser.parse_args(before)
    if not separator:
        raise Figure13BenchmarkError("benchmark arguments must follow '--'")
    return args, after


def main(argv: list[str] | None = None) -> int:
    args, benchmark_args = parse_args(sys.argv[1:] if argv is None else argv)
    loader_environment = _reject_loader_injection_environment()
    _reject_unrecorded_build_environment()
    if args.build_type != "Release":
        raise Figure13BenchmarkError("Figure 13 evidence requires --build-type=Release")
    if args.quiet_seconds < DEFAULT_QUIET_SECONDS:
        raise Figure13BenchmarkError(
            f"--quiet-seconds must be at least {DEFAULT_QUIET_SECONDS:g}"
        )
    if not 0.0 < args.sample_interval_seconds <= DEFAULT_SAMPLE_INTERVAL_SECONDS:
        raise Figure13BenchmarkError("--sample-interval-seconds must be in (0, 1]")
    if not 0.0 < args.max_normalized_load <= DEFAULT_MAX_NORMALIZED_LOAD:
        raise Figure13BenchmarkError(
            f"--max-normalized-load must be in (0, {DEFAULT_MAX_NORMALIZED_LOAD:g}]"
        )
    _validate_passthrough(benchmark_args)

    run_token = str(uuid.uuid4())
    build_dir = _build_dir(args.build_type, run_token)
    build_configuration = _configure_evidence_build(build_dir)
    binary = _build_target(build_dir)
    binary_sha256 = _sha256(binary)
    host_identity = _host_identity()
    quiet_host = _quiet_gate(
        args.quiet_seconds,
        args.sample_interval_seconds,
        args.max_normalized_load,
    )
    if _sha256(binary) != binary_sha256:
        raise Figure13BenchmarkError(
            "benchmark executable changed after the quiet-host gate"
        )
    with tempfile.TemporaryDirectory(prefix="dart-figure13-benchmark-") as temp_dir:
        raw_output = Path(temp_dir) / "benchmark.json"
        watchdog, runtime_image_inventory, loaded_dart_libraries = _run_with_watchdog(
            binary,
            raw_output,
            benchmark_args,
            build_dir=build_dir,
            interval_seconds=args.sample_interval_seconds,
            max_normalized_load=args.max_normalized_load,
        )
        if _sha256(binary) != binary_sha256:
            raise Figure13BenchmarkError(
                "benchmark executable changed during the measured run"
            )
        _write_evidence(
            raw_output,
            args.output,
            binary=binary,
            binary_sha256=binary_sha256,
            host_identity=host_identity,
            quiet_host=quiet_host,
            watchdog=watchdog,
            loader_environment=loader_environment,
            runtime_image_inventory=runtime_image_inventory,
            loaded_dart_libraries=loaded_dart_libraries,
            build_configuration=build_configuration,
            run_token=run_token,
        )
    print(f"Figure 13 benchmark evidence: {args.output}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Figure13BenchmarkError as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
