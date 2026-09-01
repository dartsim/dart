#!/usr/bin/env python3
"""Configure CMake build directories for the pixi ``config*`` tasks.

Single owner of the configure logic that used to be duplicated across six
bash blocks in pixi.toml (``config``, ``config-debug``, ``config-py``,
``config-coverage``, ``config-install``, ``config-asan``). Each variant
reproduces its original block byte-for-byte at the CMake-cache level;
divergences BETWEEN variants (launcher policy, flag sets, build dirs) are
intentional and preserved, not normalized. The win-64 config tasks and the
gazebo feature's config-gz keep their own definitions in pixi.toml.

Run as ``python scripts/cmake_config.py <variant> [options]`` from the
workspace root (pixi tasks do this). Environment-variable overrides honored
by the original blocks (DART_*_OVERRIDE, DART_DISABLE_COMPILER_CACHE,
SCCACHE_GHA_ENABLED, CMAKE_*_COMPILER_LAUNCHER, BUILD_TYPE, ...) keep the
same semantics.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

from cmake_host_linker_flags import cmake_host_linker_flags

ON_OFF = ("ON", "OFF")


def _read_text_lines_impl(path: str) -> list[str] | None:
    try:
        with open(path, encoding="utf-8", errors="replace") as handle:
            return handle.read().splitlines()
    except OSError:
        return None


def _nanobind_cmake_dir_impl() -> str:
    result = subprocess.run(
        [sys.executable, "-m", "nanobind", "--cmake_dir"],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def _detect_cuda_archs_impl() -> str | None:
    """Port of the nvidia-smi compute_cap detection pipeline.

    Returns a ';'-joined, lexicographically sorted, deduplicated list of
    compute capabilities with non-digits stripped (e.g. "8.9" -> "89"),
    or None when nvidia-smi is unavailable or yields nothing.
    """
    if shutil.which("nvidia-smi") is None:
        return None
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=compute_cap", "--format=csv,noheader"],
            check=True,
            capture_output=True,
            text=True,
        )
    except OSError, subprocess.CalledProcessError:
        return None
    return parse_cuda_archs(result.stdout)


@dataclass
class Tools:
    """Injectable process/system externals (faked in tests)."""

    which: Callable[[str], str | None] = shutil.which
    detect_cuda_archs: Callable[[], str | None] = _detect_cuda_archs_impl
    nanobind_cmake_dir: Callable[[], str] = _nanobind_cmake_dir_impl
    host_linker_defs: Callable[[], list[str]] = cmake_host_linker_flags
    read_text_lines: Callable[[str], list[str] | None] = _read_text_lines_impl


@dataclass
class ConfigPlan:
    """Everything main() needs to execute one configure."""

    build_dir: str
    cmake_args: list[str]
    # Paths to delete before configuring (coverage wipes its build dir; the
    # config-py CUDA compiler-change guard drops stale cache metadata).
    remove_paths: list[str] = field(default_factory=list)
    messages: list[str] = field(default_factory=list)


def parse_cuda_archs(raw: str) -> str | None:
    """tr -cd '0-9\\n' | sed '/^$/d' | sort -u | paste -sd';' equivalent."""
    values = []
    for line in raw.splitlines():
        digits = "".join(ch for ch in line if ch.isdigit())
        if digits:
            values.append(digits)
    if not values:
        return None
    return ";".join(sorted(set(values)))


def apply_sccache_gha_disabled(
    environ: dict, *, unset_cuda_launcher: bool
) -> str | None:
    """SCCACHE_GHA_ENABLED=false forces the compiler cache off.

    Returns the forced DART_DISABLE_COMPILER_CACHE value ("ON") when the
    kill switch is set, else None. Mutates ``environ`` exactly like the bash
    ``unset`` lines did (the CUDA launcher is only unset by the variants
    that did so).
    """
    if environ.get("SCCACHE_GHA_ENABLED") != "false":
        return None
    environ.pop("DART_COMPILER_CACHE", None)
    environ.pop("CMAKE_C_COMPILER_LAUNCHER", None)
    environ.pop("CMAKE_CXX_COMPILER_LAUNCHER", None)
    if unset_cuda_launcher:
        environ.pop("CMAKE_CUDA_COMPILER_LAUNCHER", None)
    return "ON"


def pick_launchers(environ: dict, tools: Tools, *, ccache_fallback: bool) -> None:
    """Fill CMAKE_C/CXX_COMPILER_LAUNCHER when the C launcher is unset.

    sccache is preferred (skipped when SCCACHE_GHA_ENABLED=false); ccache is
    the fallback only for the variants that had one (config-asan did not).
    """
    if environ.get("CMAKE_C_COMPILER_LAUNCHER"):
        return
    if environ.get("SCCACHE_GHA_ENABLED") != "false" and tools.which("sccache"):
        environ["CMAKE_C_COMPILER_LAUNCHER"] = "sccache"
        environ["CMAKE_CXX_COMPILER_LAUNCHER"] = "sccache"
    elif ccache_fallback and tools.which("ccache"):
        environ["CMAKE_C_COMPILER_LAUNCHER"] = "ccache"
        environ["CMAKE_CXX_COMPILER_LAUNCHER"] = "ccache"


def launcher_defs(environ: dict) -> list[str]:
    defs = []
    c_launcher = environ.get("CMAKE_C_COMPILER_LAUNCHER")
    cxx_launcher = environ.get("CMAKE_CXX_COMPILER_LAUNCHER")
    if c_launcher:
        defs.append(f"-DCMAKE_C_COMPILER_LAUNCHER={c_launcher}")
    if cxx_launcher:
        defs.append(f"-DCMAKE_CXX_COMPILER_LAUNCHER={cxx_launcher}")
    return defs


def cuda_launcher_def(environ: dict, *, typed: bool) -> str:
    """CUDA builds never inherit the C++ launcher implicitly: sccache can
    leave nvcc/fatbinary without its generated PTX input, so the launcher is
    passed through only when explicitly set and pinned empty otherwise.
    config-py wrote the typed :STRING form; config/config-debug did not."""
    suffix = ":STRING" if typed else ""
    value = environ.get("CMAKE_CUDA_COMPILER_LAUNCHER", "")
    return f"-DCMAKE_CUDA_COMPILER_LAUNCHER{suffix}={value}"


def resolve_cuda(environ: dict, tools: Tools) -> tuple[str, str]:
    """DART_ENABLE_EXPERIMENTAL_CUDA_OVERRIDE + arch auto-detection."""
    enable = environ.get("DART_ENABLE_EXPERIMENTAL_CUDA_OVERRIDE") or "OFF"
    archs = environ.get("DART_CUDA_ARCHITECTURES", "")
    if enable == "ON" and not archs:
        detected = tools.detect_cuda_archs()
        if detected:
            archs = detected
    return enable, archs


def _override(environ: dict, name: str, default: str) -> str:
    """bash ${NAME:-default}: unset OR empty falls back."""
    return environ.get(name) or default


def _require_env(environ: dict, name: str) -> str:
    value = environ.get(name)
    if value is None:
        raise SystemExit(f"{name} is not set (run through 'pixi run ...')")
    return value


def plan_config(opts: argparse.Namespace, environ: dict, tools: Tools) -> ConfigPlan:
    """The default Release-family configure (pixi task: config)."""
    build_type = opts.build_type
    asan = _override(environ, "DART_ENABLE_ASAN", "OFF")
    verbose = _override(environ, "DART_VERBOSE", "OFF")
    dartpy = _override(environ, "DART_BUILD_DARTPY_OVERRIDE", opts.dartpy)
    cuda, cuda_archs = resolve_cuda(environ, tools)
    diff = _override(environ, "DART_BUILD_DIFF_OVERRIDE", "OFF")
    collision_tests = _override(
        environ, "DART_BUILD_COLLISION_REFERENCE_TESTS_OVERRIDE", "OFF"
    )
    collision_benchmarks = _override(
        environ, "DART_BUILD_COLLISION_REFERENCE_BENCHMARKS_OVERRIDE", "OFF"
    )
    gui = _override(environ, "DART_BUILD_GUI_OVERRIDE", "ON")
    examples = _override(environ, "DART_BUILD_EXAMPLES_OVERRIDE", "ON")
    demos_memory = _override(
        environ, "DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS_OVERRIDE", "OFF"
    )
    # The demos panel needs the library instrumentation, so it defaults to the
    # demos value; override it alone to instrument the library without the panel.
    memory = _override(environ, "DART_BUILD_MEMORY_DIAGNOSTICS_OVERRIDE", demos_memory)
    tutorials = _override(environ, "DART_BUILD_TUTORIALS_OVERRIDE", "ON")
    tests = _override(environ, "DART_BUILD_TESTS_OVERRIDE", "ON")
    mold = _override(environ, "DART_USE_MOLD_OVERRIDE", "OFF")
    system_imgui = _override(
        environ, "DART_USE_SYSTEM_IMGUI_OVERRIDE", opts.use_system_imgui
    )
    gui_smoke = _override(
        environ, "DART_ENABLE_GUI_FILAMENT_SMOKE_TESTS_OVERRIDE", "OFF"
    )
    disable_cache = _override(environ, "DART_DISABLE_COMPILER_CACHE", "OFF")
    # Local CUDA builds default the compiler cache off unless explicitly set.
    if cuda == "ON" and not environ.get("DART_DISABLE_COMPILER_CACHE"):
        disable_cache = "ON"
    forced = apply_sccache_gha_disabled(environ, unset_cuda_launcher=True)
    if forced:
        disable_cache = forced
    pick_launchers(environ, tools, ccache_fallback=True)
    defs = launcher_defs(environ)
    host_defs = tools.host_linker_defs()
    if cuda == "ON":
        defs.append(cuda_launcher_def(environ, typed=False))
    nanobind_dir = tools.nanobind_cmake_dir()
    conda_prefix = _require_env(environ, "CONDA_PREFIX")
    pixi_env = _require_env(environ, "PIXI_ENVIRONMENT_NAME")
    build_dir = f"build/{pixi_env}/cpp/{build_type}"
    cmake_args = [
        "-G",
        "Ninja",
        "-S",
        ".",
        "-B",
        build_dir,
        f"-DCMAKE_INSTALL_PREFIX={conda_prefix}",
        f"-DCMAKE_BUILD_TYPE={build_type}",
        f"-DCMAKE_PREFIX_PATH={conda_prefix}",
        f"-DDART_BUILD_DARTPY={dartpy}",
        f"-DDART_ENABLE_EXPERIMENTAL_CUDA={cuda}",
        f"-DDART_BUILD_DIFF={diff}",
        f"-DDART_CUDA_ARCHITECTURES={cuda_archs}",
        f"-DDART_BUILD_COLLISION_REFERENCE_TESTS={collision_tests}",
        f"-DDART_BUILD_COLLISION_REFERENCE_BENCHMARKS={collision_benchmarks}",
        f"-DDART_BUILD_GUI={gui}",
        f"-DDART_BUILD_EXAMPLES={examples}",
        f"-DDART_BUILD_DEMOS_MEMORY_DIAGNOSTICS={demos_memory}",
        f"-DDART_BUILD_MEMORY_DIAGNOSTICS={memory}",
        f"-DDART_BUILD_TUTORIALS={tutorials}",
        f"-DDART_BUILD_TESTS={tests}",
        f"-DDART_USE_MOLD={mold}",
        f"-DDART_BUILD_PROFILE={opts.build_profile}",
        f"-DDART_PROFILE_BUILTIN={opts.profile_builtin}",
        f"-DDART_PROFILE_TRACY={opts.profile_tracy}",
        f"-DDART_ENABLE_ASAN={asan}",
        f"-DDART_DISABLE_COMPILER_CACHE={disable_cache}",
        "-DDART_USE_SYSTEM_GOOGLEBENCHMARK=ON",
        "-DDART_USE_SYSTEM_GOOGLETEST=ON",
        f"-DDART_USE_SYSTEM_IMGUI={system_imgui}",
        f"-DDART_ENABLE_GUI_FILAMENT_SMOKE_TESTS={gui_smoke}",
        "-DDART_USE_SYSTEM_NANOBIND=OFF",
        "-DDART_USE_SYSTEM_TRACY=OFF",
        f"-DDART_VERBOSE={verbose}",
        *host_defs,
        f"-Dnanobind_DIR={nanobind_dir}",
        *defs,
    ]
    return ConfigPlan(build_dir=build_dir, cmake_args=cmake_args)


def plan_config_debug(
    opts: argparse.Namespace, environ: dict, tools: Tools
) -> ConfigPlan:
    """Debug configure (pixi task: config-debug).

    Differs from ``config`` by design: BUILD_TYPE comes from the environment
    (default Debug), the profile toggles are environment-driven, and the
    DART_USE_MOLD flag is not passed.
    """
    build_type = _override(environ, "BUILD_TYPE", "Debug")
    profile_builtin = _override(environ, "DART_PROFILE_BUILTIN", "ON")
    profile_tracy = _override(environ, "DART_PROFILE_TRACY", "OFF")
    asan = _override(environ, "DART_ENABLE_ASAN", "OFF")
    verbose = _override(environ, "DART_VERBOSE", "OFF")
    dartpy = _override(environ, "DART_BUILD_DARTPY_OVERRIDE", opts.dartpy)
    cuda, cuda_archs = resolve_cuda(environ, tools)
    diff = _override(environ, "DART_BUILD_DIFF_OVERRIDE", "OFF")
    collision_tests = _override(
        environ, "DART_BUILD_COLLISION_REFERENCE_TESTS_OVERRIDE", "OFF"
    )
    collision_benchmarks = _override(
        environ, "DART_BUILD_COLLISION_REFERENCE_BENCHMARKS_OVERRIDE", "OFF"
    )
    gui = _override(environ, "DART_BUILD_GUI_OVERRIDE", "ON")
    examples = _override(environ, "DART_BUILD_EXAMPLES_OVERRIDE", "ON")
    demos_memory = _override(
        environ, "DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS_OVERRIDE", "OFF"
    )
    memory = _override(environ, "DART_BUILD_MEMORY_DIAGNOSTICS_OVERRIDE", demos_memory)
    tutorials = _override(environ, "DART_BUILD_TUTORIALS_OVERRIDE", "ON")
    tests = _override(environ, "DART_BUILD_TESTS_OVERRIDE", "ON")
    system_imgui = _override(environ, "DART_USE_SYSTEM_IMGUI_OVERRIDE", "ON")
    gui_smoke = _override(
        environ, "DART_ENABLE_GUI_FILAMENT_SMOKE_TESTS_OVERRIDE", "OFF"
    )
    disable_cache = _override(environ, "DART_DISABLE_COMPILER_CACHE", "OFF")
    if cuda == "ON" and not environ.get("DART_DISABLE_COMPILER_CACHE"):
        disable_cache = "ON"
    forced = apply_sccache_gha_disabled(environ, unset_cuda_launcher=True)
    if forced:
        disable_cache = forced
    pick_launchers(environ, tools, ccache_fallback=True)
    defs = launcher_defs(environ)
    host_defs = tools.host_linker_defs()
    if cuda == "ON":
        defs.append(cuda_launcher_def(environ, typed=False))
    nanobind_dir = tools.nanobind_cmake_dir()
    conda_prefix = _require_env(environ, "CONDA_PREFIX")
    pixi_env = _require_env(environ, "PIXI_ENVIRONMENT_NAME")
    build_dir = f"build/{pixi_env}/cpp/{build_type}"
    cmake_args = [
        "-G",
        "Ninja",
        "-S",
        ".",
        "-B",
        build_dir,
        f"-DCMAKE_INSTALL_PREFIX={conda_prefix}",
        f"-DCMAKE_BUILD_TYPE={build_type}",
        f"-DCMAKE_PREFIX_PATH={conda_prefix}",
        f"-DDART_BUILD_DARTPY={dartpy}",
        f"-DDART_ENABLE_EXPERIMENTAL_CUDA={cuda}",
        f"-DDART_BUILD_DIFF={diff}",
        f"-DDART_CUDA_ARCHITECTURES={cuda_archs}",
        f"-DDART_BUILD_COLLISION_REFERENCE_TESTS={collision_tests}",
        f"-DDART_BUILD_COLLISION_REFERENCE_BENCHMARKS={collision_benchmarks}",
        f"-DDART_BUILD_GUI={gui}",
        f"-DDART_BUILD_EXAMPLES={examples}",
        f"-DDART_BUILD_DEMOS_MEMORY_DIAGNOSTICS={demos_memory}",
        f"-DDART_BUILD_MEMORY_DIAGNOSTICS={memory}",
        f"-DDART_BUILD_TUTORIALS={tutorials}",
        f"-DDART_BUILD_TESTS={tests}",
        f"-DDART_BUILD_PROFILE={opts.build_profile}",
        f"-DDART_PROFILE_BUILTIN={profile_builtin}",
        f"-DDART_PROFILE_TRACY={profile_tracy}",
        f"-DDART_ENABLE_ASAN={asan}",
        f"-DDART_DISABLE_COMPILER_CACHE={disable_cache}",
        "-DDART_USE_SYSTEM_GOOGLEBENCHMARK=ON",
        "-DDART_USE_SYSTEM_GOOGLETEST=ON",
        f"-DDART_USE_SYSTEM_IMGUI={system_imgui}",
        f"-DDART_ENABLE_GUI_FILAMENT_SMOKE_TESTS={gui_smoke}",
        "-DDART_USE_SYSTEM_NANOBIND=OFF",
        "-DDART_USE_SYSTEM_TRACY=OFF",
        f"-DDART_VERBOSE={verbose}",
        *host_defs,
        f"-Dnanobind_DIR={nanobind_dir}",
        *defs,
    ]
    return ConfigPlan(build_dir=build_dir, cmake_args=cmake_args)


_CUDA_CACHE_KEYS = {
    "CMAKE_C_COMPILER": "c_compiler",
    "CMAKE_CXX_COMPILER": "cxx_compiler",
    "CMAKE_CUDA_COMPILER": "cuda_compiler",
    "CMAKE_CUDA_COMPILER_LAUNCHER": "cuda_compiler_launcher",
}


def _cuda_cache_reset_needed(
    cache_lines: list[str],
    environ: dict,
    conda_prefix: str,
) -> bool:
    """CUDA compiler-change guard: reset build dirs whose cached compilers no
    longer match the current toolchain (nvcc pinned to the conda toolkit; the
    C/C++ compilers compared against the active CC/CXX when set, which also
    catches dirs configured by the retired system-host-compiler workaround),
    or whose cached CUDA launcher is set while the environment's is not."""
    cached = {v: "" for v in _CUDA_CACHE_KEYS.values()}
    for line in cache_lines:
        key, sep, value = line.partition("=")
        if not sep:
            continue
        name = key.split(":", 1)[0]
        slot = _CUDA_CACHE_KEYS.get(name)
        if slot is not None:
            cached[slot] = value
    if cached["cuda_compiler"] != f"{conda_prefix}/bin/nvcc":
        return True
    if environ.get("CC") and cached["c_compiler"] != environ["CC"]:
        return True
    if environ.get("CXX") and cached["cxx_compiler"] != environ["CXX"]:
        return True
    if (
        not environ.get("CMAKE_CUDA_COMPILER_LAUNCHER")
        and cached["cuda_compiler_launcher"]
    ):
        return True
    return False


def plan_config_py(opts: argparse.Namespace, environ: dict, tools: Tools) -> ConfigPlan:
    """dartpy-focused configure (pixi task: config-py).

    Exists to build dartpy: honors an explicit dartpy=ON request over the
    cuda environment's DART_BUILD_DARTPY_OVERRIDE=OFF, forwards the
    experimental CUDA opt-in, and — for CUDA builds — switches the C/C++
    toolchain to the host system compiler (the prebuilt Filament libraries
    need a newer glibc than the conda portability sysroot exports) while
    CUDA keeps the conda nvcc. See the original block's rationale, preserved
    in the inline comments below.
    """
    build_type = opts.build_type
    build_dir_name = opts.build_dir or build_type
    verbose = _override(environ, "DART_VERBOSE", "OFF")
    dartpy = _override(environ, "DART_BUILD_DARTPY_OVERRIDE", opts.dartpy)
    # Honor an explicit dartpy=ON request over an environment-level OFF
    # override (the cuda environment disables dartpy for its lean compute
    # path; 'pixi run -e cuda py-demos' still needs a CUDA-enabled dartpy).
    if dartpy == "OFF" and opts.dartpy == "ON":
        dartpy = "ON"
    cuda, cuda_archs = resolve_cuda(environ, tools)
    collision_tests = _override(
        environ, "DART_BUILD_COLLISION_REFERENCE_TESTS_OVERRIDE", "OFF"
    )
    collision_benchmarks = _override(
        environ, "DART_BUILD_COLLISION_REFERENCE_BENCHMARKS_OVERRIDE", "OFF"
    )
    system_imgui = _override(
        environ, "DART_USE_SYSTEM_IMGUI_OVERRIDE", opts.use_system_imgui
    )
    conda_prefix = _require_env(environ, "CONDA_PREFIX")
    pixi_env = _require_env(environ, "PIXI_ENVIRONMENT_NAME")
    build_dir = f"build/{pixi_env}/cpp/{build_dir_name}"

    compiler_defs: list[str] = []
    remove_paths: list[str] = []
    messages: list[str] = []
    if cuda == "ON":
        # CUDA toolchain for the CUDA-enabled dartpy + GUI build. Pin nvcc to
        # the conda toolkit so CMake does not pick up an older system nvcc;
        # the conda compiler (activated by the cuda feature) builds the C/C++
        # sources and the final link, and nvcc's host pass follows it via the
        # env's NVCC_PREPEND_FLAGS. The packaged conda-forge Filament is built
        # against the conda sysroot, so the system-host-compiler/glibc
        # workaround that used to live here is unnecessary. The cache guard
        # resets build dirs whose cached compilers no longer match the current
        # toolchain (including dirs configured by the retired workaround) and
        # clears a stale CUDA launcher (sccache can leave nvcc/fatbinary
        # without its generated PTX input). No effect on non-CUDA builds.
        compiler_defs = [f"-DCMAKE_CUDA_COMPILER={conda_prefix}/bin/nvcc"]
        cache_path = f"{build_dir}/CMakeCache.txt"
        cache_lines = tools.read_text_lines(cache_path)
        if cache_lines is not None and _cuda_cache_reset_needed(
            cache_lines, environ, conda_prefix
        ):
            messages.append(
                f"CMake compiler cache changed for {build_dir}; "
                "resetting cache metadata."
            )
            remove_paths = [cache_path, f"{build_dir}/CMakeFiles"]

    disable_cache = _override(environ, "DART_DISABLE_COMPILER_CACHE", "OFF")
    # CUDA configs still leave CMAKE_CUDA_COMPILER_LAUNCHER empty below
    # (sccache can leave nvcc/fatbinary without generated PTX) but keep the
    # C/CXX launchers so CUDA-enabled dartpy rebuilds cache host C++ TUs —
    # hence no CUDA launcher unset here, unlike config/config-debug.
    forced = apply_sccache_gha_disabled(environ, unset_cuda_launcher=False)
    if forced:
        disable_cache = forced
    pick_launchers(environ, tools, ccache_fallback=True)
    defs = launcher_defs(environ)
    if cuda == "ON":
        defs.append(cuda_launcher_def(environ, typed=True))
    nanobind_dir = tools.nanobind_cmake_dir()
    host_defs = tools.host_linker_defs()
    cmake_args = [
        "-G",
        "Ninja",
        "-S",
        ".",
        "-B",
        build_dir,
        f"-DCMAKE_INSTALL_PREFIX={conda_prefix}",
        f"-DCMAKE_BUILD_TYPE={build_type}",
        f"-DCMAKE_PREFIX_PATH={conda_prefix}",
        f"-DDART_BUILD_DARTPY={dartpy}",
        f"-DDART_ENABLE_EXPERIMENTAL_CUDA={cuda}",
        f"-DDART_CUDA_ARCHITECTURES={cuda_archs}",
        f"-DDART_BUILD_COLLISION_REFERENCE_TESTS={collision_tests}",
        f"-DDART_BUILD_COLLISION_REFERENCE_BENCHMARKS={collision_benchmarks}",
        f"-DDART_DISABLE_COMPILER_CACHE={disable_cache}",
        f"-DDART_BUILD_PROFILE={_override(environ, 'DART_BUILD_PROFILE', 'ON')}",
        f"-DDART_PROFILE_BUILTIN={_override(environ, 'DART_PROFILE_BUILTIN', 'ON')}",
        f"-DDART_PROFILE_TRACY={_override(environ, 'DART_PROFILE_TRACY', 'OFF')}",
        f"-DDART_ENABLE_ASAN={_override(environ, 'DART_ENABLE_ASAN', 'OFF')}",
        "-DDART_USE_SYSTEM_GOOGLEBENCHMARK=ON",
        "-DDART_USE_SYSTEM_GOOGLETEST=ON",
        f"-DDART_USE_SYSTEM_IMGUI={system_imgui}",
        "-DDART_USE_SYSTEM_NANOBIND=OFF",
        "-DDART_USE_SYSTEM_TRACY=OFF",
        f"-DDART_VERBOSE={verbose}",
        f"-Dnanobind_DIR={nanobind_dir}",
        *compiler_defs,
        *host_defs,
        *defs,
    ]
    return ConfigPlan(
        build_dir=build_dir,
        cmake_args=cmake_args,
        remove_paths=remove_paths,
        messages=messages,
    )


def plan_config_coverage(
    opts: argparse.Namespace, environ: dict, tools: Tools
) -> ConfigPlan:
    """Coverage configure (pixi task: config-coverage).

    Always starts from a wiped build directory so stale non-coverage objects
    cannot pollute the gcda/gcno set. Profiling is hard-off and DART_CODECOV
    hard-on by design.
    """
    pixi_env = environ.get("PIXI_ENVIRONMENT_NAME") or "default"
    build_type = _require_env(environ, "BUILD_TYPE")
    verbose = _require_env(environ, "DART_VERBOSE")
    build_dir = f"build/{pixi_env}/cpp/{build_type}"
    dartpy = _override(environ, "DART_BUILD_DARTPY_OVERRIDE", "OFF")
    collision_tests = _override(
        environ, "DART_BUILD_COLLISION_REFERENCE_TESTS_OVERRIDE", "OFF"
    )
    collision_benchmarks = _override(
        environ, "DART_BUILD_COLLISION_REFERENCE_BENCHMARKS_OVERRIDE", "OFF"
    )
    disable_cache = _override(environ, "DART_DISABLE_COMPILER_CACHE", "OFF")
    gui = _override(environ, "DART_BUILD_GUI_OVERRIDE", "ON")
    forced = apply_sccache_gha_disabled(environ, unset_cuda_launcher=False)
    if forced:
        disable_cache = forced
    pick_launchers(environ, tools, ccache_fallback=True)
    defs = launcher_defs(environ)
    host_defs = tools.host_linker_defs()
    conda_prefix = _require_env(environ, "CONDA_PREFIX")
    cmake_args = [
        "-G",
        "Ninja",
        "-S",
        ".",
        "-B",
        build_dir,
        f"-DCMAKE_INSTALL_PREFIX={conda_prefix}",
        f"-DCMAKE_BUILD_TYPE={build_type}",
        f"-DCMAKE_PREFIX_PATH={conda_prefix}",
        "-DDART_BUILD_PROFILE=OFF",
        f"-DDART_BUILD_DARTPY={dartpy}",
        f"-DDART_BUILD_GUI={gui}",
        f"-DDART_DISABLE_COMPILER_CACHE={disable_cache}",
        "-DDART_CODECOV=ON",
        f"-DDART_BUILD_COLLISION_REFERENCE_TESTS={collision_tests}",
        f"-DDART_BUILD_COLLISION_REFERENCE_BENCHMARKS={collision_benchmarks}",
        "-DDART_USE_SYSTEM_GOOGLEBENCHMARK=ON",
        "-DDART_USE_SYSTEM_GOOGLETEST=ON",
        "-DDART_USE_SYSTEM_IMGUI=ON",
        "-DDART_USE_SYSTEM_TRACY=OFF",
        f"-DDART_VERBOSE={verbose}",
        *host_defs,
        *defs,
    ]
    return ConfigPlan(
        build_dir=build_dir,
        cmake_args=cmake_args,
        remove_paths=[build_dir],
        messages=[f"Configuring coverage build directory: {build_dir}"],
    )


def plan_config_install(
    opts: argparse.Namespace, environ: dict, tools: Tools
) -> ConfigPlan:
    """Install-smoke configure (pixi task: config-install).

    Configures a dedicated build tree directly under build/<env>/cpp (no
    build-type subdirectory) used by 'pixi run install'; dartpy comes from
    the task argument only (no environment override).
    """
    verbose = _require_env(environ, "DART_VERBOSE")
    build_type = _require_env(environ, "BUILD_TYPE")
    gui = _override(environ, "DART_BUILD_GUI_OVERRIDE", "ON")
    collision_tests = _override(
        environ, "DART_BUILD_COLLISION_REFERENCE_TESTS_OVERRIDE", "OFF"
    )
    collision_benchmarks = _override(
        environ, "DART_BUILD_COLLISION_REFERENCE_BENCHMARKS_OVERRIDE", "OFF"
    )
    disable_cache = _override(environ, "DART_DISABLE_COMPILER_CACHE", "OFF")
    forced = apply_sccache_gha_disabled(environ, unset_cuda_launcher=False)
    if forced:
        disable_cache = forced
    pick_launchers(environ, tools, ccache_fallback=True)
    defs = launcher_defs(environ)
    host_defs = tools.host_linker_defs()
    conda_prefix = _require_env(environ, "CONDA_PREFIX")
    pixi_env = _require_env(environ, "PIXI_ENVIRONMENT_NAME")
    build_dir = f"build/{pixi_env}/cpp"
    cmake_args = [
        "-G",
        "Ninja",
        "-S",
        ".",
        "-B",
        build_dir,
        f"-DCMAKE_INSTALL_PREFIX={conda_prefix}",
        f"-DCMAKE_BUILD_TYPE={build_type}",
        f"-DCMAKE_PREFIX_PATH={conda_prefix}",
        f"-DDART_BUILD_DARTPY={opts.dartpy}",
        f"-DDART_BUILD_GUI={gui}",
        f"-DDART_BUILD_COLLISION_REFERENCE_TESTS={collision_tests}",
        f"-DDART_BUILD_COLLISION_REFERENCE_BENCHMARKS={collision_benchmarks}",
        f"-DDART_DISABLE_COMPILER_CACHE={disable_cache}",
        "-DDART_BUILD_PROFILE=OFF",
        "-DDART_USE_SYSTEM_GOOGLEBENCHMARK=ON",
        "-DDART_USE_SYSTEM_GOOGLETEST=ON",
        "-DDART_USE_SYSTEM_IMGUI=ON",
        "-DDART_USE_SYSTEM_NANOBIND=OFF",
        "-DDART_USE_SYSTEM_TRACY=OFF",
        f"-DDART_VERBOSE={verbose}",
        *host_defs,
        *defs,
    ]
    return ConfigPlan(build_dir=build_dir, cmake_args=cmake_args)


def plan_config_asan(
    opts: argparse.Namespace, environ: dict, tools: Tools
) -> ConfigPlan:
    """AddressSanitizer configure (pixi task: config-asan).

    The build DIRECTORY name (BUILD_TYPE, default 'asan') is decoupled from
    the CMake build type (CMAKE_BUILD_TYPE, default RelWithDebInfo). Only
    sccache is used as a launcher — no ccache fallback — matching the
    original block.
    """
    build_type = _override(environ, "BUILD_TYPE", "asan")
    cmake_build_type = _override(environ, "CMAKE_BUILD_TYPE", "RelWithDebInfo")
    build_profile = _override(environ, "DART_BUILD_PROFILE", "OFF")
    verbose = _override(environ, "DART_VERBOSE", "OFF")
    dartpy = _override(environ, "DART_BUILD_DARTPY_OVERRIDE", "ON")
    collision_tests = _override(
        environ, "DART_BUILD_COLLISION_REFERENCE_TESTS_OVERRIDE", "OFF"
    )
    collision_benchmarks = _override(
        environ, "DART_BUILD_COLLISION_REFERENCE_BENCHMARKS_OVERRIDE", "OFF"
    )
    disable_cache = _override(environ, "DART_DISABLE_COMPILER_CACHE", "OFF")
    forced = apply_sccache_gha_disabled(environ, unset_cuda_launcher=False)
    if forced:
        disable_cache = forced
    pick_launchers(environ, tools, ccache_fallback=False)
    defs = launcher_defs(environ)
    host_defs = tools.host_linker_defs()
    conda_prefix = _require_env(environ, "CONDA_PREFIX")
    pixi_env = _require_env(environ, "PIXI_ENVIRONMENT_NAME")
    build_dir = f"build/{pixi_env}/cpp/{build_type}"
    cmake_args = [
        "-G",
        "Ninja",
        "-S",
        ".",
        "-B",
        build_dir,
        f"-DCMAKE_INSTALL_PREFIX={conda_prefix}",
        f"-DCMAKE_BUILD_TYPE={cmake_build_type}",
        f"-DCMAKE_PREFIX_PATH={conda_prefix}",
        f"-DDART_BUILD_DARTPY={dartpy}",
        f"-DDART_BUILD_COLLISION_REFERENCE_TESTS={collision_tests}",
        f"-DDART_BUILD_COLLISION_REFERENCE_BENCHMARKS={collision_benchmarks}",
        f"-DDART_DISABLE_COMPILER_CACHE={disable_cache}",
        f"-DDART_BUILD_PROFILE={build_profile}",
        "-DDART_ENABLE_ASAN=ON",
        "-DDART_USE_SYSTEM_GOOGLEBENCHMARK=ON",
        "-DDART_USE_SYSTEM_GOOGLETEST=ON",
        "-DDART_USE_SYSTEM_IMGUI=ON",
        "-DDART_USE_SYSTEM_NANOBIND=OFF",
        "-DDART_USE_SYSTEM_TRACY=OFF",
        f"-DDART_VERBOSE={verbose}",
        *host_defs,
        *defs,
    ]
    return ConfigPlan(build_dir=build_dir, cmake_args=cmake_args)


PLANNERS = {
    "config": plan_config,
    "config-debug": plan_config_debug,
    "config-py": plan_config_py,
    "config-coverage": plan_config_coverage,
    "config-install": plan_config_install,
    "config-asan": plan_config_asan,
}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("variant", choices=sorted(PLANNERS))
    parser.add_argument("--dartpy", default="ON", choices=ON_OFF)
    parser.add_argument("--build-type", default="Release")
    parser.add_argument("--build-profile", default="ON", choices=ON_OFF)
    parser.add_argument("--profile-builtin", default="ON", choices=ON_OFF)
    parser.add_argument("--profile-tracy", default="OFF", choices=ON_OFF)
    parser.add_argument("--use-system-imgui", default="ON", choices=ON_OFF)
    parser.add_argument("--build-dir", default="")
    return parser


def execute(plan: ConfigPlan) -> int:
    for message in plan.messages:
        print(message)
    for path in plan.remove_paths:
        target = Path(path)
        if target.is_dir():
            shutil.rmtree(target)
        elif target.exists():
            target.unlink()
    result = subprocess.run(["cmake", *plan.cmake_args], check=False)
    return result.returncode


def main(argv: list[str] | None = None) -> int:
    opts = build_parser().parse_args(argv)
    plan = PLANNERS[opts.variant](opts, os.environ, Tools())
    return execute(plan)


if __name__ == "__main__":
    sys.exit(main())
