"""Tests for scripts/cmake_config.py (the pixi config* task backend)."""

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = ROOT / "scripts"
SCRIPT = SCRIPTS_DIR / "cmake_config.py"


def _load_module():
    if str(SCRIPTS_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("cmake_config", SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


MODULE = _load_module()


def _tools(
    *,
    which=lambda name: None,
    detect_cuda_archs=lambda: None,
    nanobind_cmake_dir=lambda: "/nb/cmake",
    host_linker_defs=lambda: [],
    read_text_lines=lambda path: None,
):
    return MODULE.Tools(
        which=which,
        detect_cuda_archs=detect_cuda_archs,
        nanobind_cmake_dir=nanobind_cmake_dir,
        host_linker_defs=host_linker_defs,
        read_text_lines=read_text_lines,
    )


def _env(**extra):
    base = {"CONDA_PREFIX": "/prefix", "PIXI_ENVIRONMENT_NAME": "default"}
    base.update(extra)
    return base


def _opts(*argv):
    return MODULE.build_parser().parse_args(list(argv))


def _flags(plan):
    out = {}
    for arg in plan.cmake_args:
        if arg.startswith("-D"):
            key, _, value = arg[2:].partition("=")
            out[key.split(":", 1)[0]] = value
    return out


def test_config_default_flags():
    plan = MODULE.plan_config(_opts("config"), _env(), _tools())
    flags = _flags(plan)
    assert plan.build_dir == "build/default/cpp/Release"
    assert flags["CMAKE_BUILD_TYPE"] == "Release"
    assert flags["DART_BUILD_DARTPY"] == "ON"
    assert flags["DART_BUILD_GUI"] == "ON"  # defaults ON on every platform
    assert flags["DART_BUILD_TESTS"] == "ON"
    assert flags["DART_USE_MOLD"] == "OFF"
    assert flags["DART_ENABLE_EXPERIMENTAL_CUDA"] == "OFF"
    assert flags["DART_DISABLE_COMPILER_CACHE"] == "OFF"
    assert flags["nanobind_DIR"] == "/nb/cmake"
    assert "CMAKE_C_COMPILER_LAUNCHER" not in flags
    assert not plan.remove_paths


def test_config_has_no_filament_provider_flags():
    # conda-forge filament is a workspace dependency; the retired
    # DART_USE_SYSTEM_FILAMENT/DART_FETCH_FILAMENT flags are gone.
    flags = _flags(MODULE.plan_config(_opts("config"), _env(), _tools()))
    assert "DART_USE_SYSTEM_FILAMENT" not in flags
    assert "DART_FETCH_FILAMENT" not in flags


def test_config_env_overrides_beat_arg_defaults():
    env = _env(DART_BUILD_DARTPY_OVERRIDE="OFF", DART_BUILD_TESTS_OVERRIDE="OFF")
    flags = _flags(MODULE.plan_config(_opts("config"), env, _tools()))
    assert flags["DART_BUILD_DARTPY"] == "OFF"
    assert flags["DART_BUILD_TESTS"] == "OFF"


def test_config_launcher_prefers_sccache_then_ccache():
    have_both = _tools(
        which=lambda n: f"/bin/{n}" if n in ("sccache", "ccache") else None
    )
    flags = _flags(MODULE.plan_config(_opts("config"), _env(), have_both))
    assert flags["CMAKE_C_COMPILER_LAUNCHER"] == "sccache"
    assert flags["CMAKE_CXX_COMPILER_LAUNCHER"] == "sccache"

    only_ccache = _tools(which=lambda n: "/bin/ccache" if n == "ccache" else None)
    flags = _flags(MODULE.plan_config(_opts("config"), _env(), only_ccache))
    assert flags["CMAKE_C_COMPILER_LAUNCHER"] == "ccache"


def test_config_asan_has_no_ccache_fallback():
    only_ccache = _tools(which=lambda n: "/bin/ccache" if n == "ccache" else None)
    env = _env(BUILD_TYPE="asan", CMAKE_BUILD_TYPE="RelWithDebInfo")
    flags = _flags(MODULE.plan_config_asan(_opts("config-asan"), env, only_ccache))
    assert "CMAKE_C_COMPILER_LAUNCHER" not in flags


def test_sccache_gha_disabled_unsets_and_forces_cache_off():
    env = _env(
        SCCACHE_GHA_ENABLED="false",
        CMAKE_C_COMPILER_LAUNCHER="sccache",
        CMAKE_CXX_COMPILER_LAUNCHER="sccache",
        CMAKE_CUDA_COMPILER_LAUNCHER="sccache",
        DART_COMPILER_CACHE="sccache",
    )
    have_sccache = _tools(which=lambda n: "/bin/sccache" if n == "sccache" else None)
    flags = _flags(MODULE.plan_config(_opts("config"), env, have_sccache))
    assert flags["DART_DISABLE_COMPILER_CACHE"] == "ON"
    # Unset by the kill switch, and sccache pickup is skipped as well.
    assert "CMAKE_C_COMPILER_LAUNCHER" not in flags
    assert "CMAKE_C_COMPILER_LAUNCHER" not in env
    # config also unsets the CUDA launcher (config-py deliberately does not).
    assert "CMAKE_CUDA_COMPILER_LAUNCHER" not in env


def test_config_py_cuda_keeps_compiler_cache_on():
    # Unlike config/config-debug, config-py deliberately keeps the compiler
    # cache under CUDA so dartpy rebuilds cache host C++ translation units.
    env = _env(DART_ENABLE_EXPERIMENTAL_CUDA_OVERRIDE="ON")
    flags = _flags(
        MODULE.plan_config_py(
            _opts("config-py"), env, _tools(detect_cuda_archs=lambda: "89")
        )
    )
    assert flags["DART_DISABLE_COMPILER_CACHE"] == "OFF"


def test_config_py_keeps_cuda_launcher_on_gha_kill_switch():
    env = _env(SCCACHE_GHA_ENABLED="false", CMAKE_CUDA_COMPILER_LAUNCHER="sccache")
    MODULE.plan_config_py(_opts("config-py"), env, _tools())
    assert env["CMAKE_CUDA_COMPILER_LAUNCHER"] == "sccache"


def test_parse_cuda_archs_matches_shell_pipeline():
    # tr -cd '0-9\n' | sed '/^$/d' | sort -u | paste -sd';' — the sort is
    # lexicographic, and duplicates collapse.
    assert MODULE.parse_cuda_archs("8.9\n12.0\n8.9\n") == "120;89"
    assert MODULE.parse_cuda_archs("\n\n") is None


def test_config_cuda_enables_arch_detection_and_cache_disable():
    env = _env(DART_ENABLE_EXPERIMENTAL_CUDA_OVERRIDE="ON")
    tools = _tools(detect_cuda_archs=lambda: "89")
    flags = _flags(MODULE.plan_config(_opts("config"), env, tools))
    assert flags["DART_ENABLE_EXPERIMENTAL_CUDA"] == "ON"
    assert flags["DART_CUDA_ARCHITECTURES"] == "89"
    # Local CUDA builds default the compiler cache off...
    assert flags["DART_DISABLE_COMPILER_CACHE"] == "ON"
    # ...and pin the CUDA launcher empty (untyped form for config).
    plan = MODULE.plan_config(
        _opts("config"), _env(DART_ENABLE_EXPERIMENTAL_CUDA_OVERRIDE="ON"), tools
    )
    assert "-DCMAKE_CUDA_COMPILER_LAUNCHER=" in plan.cmake_args


def test_config_cuda_explicit_archs_respected():
    env = _env(
        DART_ENABLE_EXPERIMENTAL_CUDA_OVERRIDE="ON", DART_CUDA_ARCHITECTURES="75"
    )
    called = []

    def detect():
        called.append(True)
        return "89"

    flags = _flags(
        MODULE.plan_config(_opts("config"), env, _tools(detect_cuda_archs=detect))
    )
    assert flags["DART_CUDA_ARCHITECTURES"] == "75"
    assert not called


def test_config_py_dartpy_on_request_beats_env_off_override():
    env = _env(DART_BUILD_DARTPY_OVERRIDE="OFF")
    flags = _flags(MODULE.plan_config_py(_opts("config-py"), env, _tools()))
    assert flags["DART_BUILD_DARTPY"] == "ON"

    flags = _flags(
        MODULE.plan_config_py(_opts("config-py", "--dartpy=OFF"), _env(), _tools())
    )
    assert flags["DART_BUILD_DARTPY"] == "OFF"


def test_config_py_build_dir_falls_back_to_build_type():
    plan = MODULE.plan_config_py(_opts("config-py"), _env(), _tools())
    assert plan.build_dir == "build/default/cpp/Release"
    plan = MODULE.plan_config_py(
        _opts("config-py", "--build-dir=Release-docking"), _env(), _tools()
    )
    assert plan.build_dir == "build/default/cpp/Release-docking"


def test_config_py_cuda_pins_conda_nvcc_and_typed_launcher():
    # The conda toolchain builds C/C++ (activation sets CC/CXX); the script
    # only pins nvcc and leaves the environment untouched.
    env = _env(DART_ENABLE_EXPERIMENTAL_CUDA_OVERRIDE="ON", LDFLAGS="-Lorig")
    plan = MODULE.plan_config_py(
        _opts("config-py"), env, _tools(detect_cuda_archs=lambda: "89")
    )
    assert "CC" not in env
    assert env["LDFLAGS"] == "-Lorig"
    assert "-DCMAKE_CUDA_COMPILER=/prefix/bin/nvcc" in plan.cmake_args
    assert "-DCMAKE_CUDA_HOST_COMPILER" not in " ".join(plan.cmake_args)
    assert "-DCMAKE_CUDA_COMPILER_LAUNCHER:STRING=" in plan.cmake_args


def test_config_py_cuda_cache_reset_on_compiler_change():
    # A stale nvcc (e.g. system nvcc) forces the reset.
    stale_nvcc = [
        "CMAKE_CUDA_COMPILER:FILEPATH=/usr/bin/nvcc",
    ]
    env = _env(DART_ENABLE_EXPERIMENTAL_CUDA_OVERRIDE="ON")
    plan = MODULE.plan_config_py(
        _opts("config-py"), env, _tools(read_text_lines=lambda path: stale_nvcc)
    )
    assert plan.remove_paths == [
        "build/default/cpp/Release/CMakeCache.txt",
        "build/default/cpp/Release/CMakeFiles",
    ]

    # C/C++ compilers are compared only when CC/CXX are set (conda activation
    # sets them in the cuda env); a dir configured by the retired host-compiler
    # workaround then no longer matches and is reset.
    workaround_dir = [
        "CMAKE_C_COMPILER:FILEPATH=/usr/bin/cc",
        "CMAKE_CXX_COMPILER:FILEPATH=/usr/bin/c++",
        "CMAKE_CUDA_COMPILER:FILEPATH=/prefix/bin/nvcc",
    ]
    env = _env(
        DART_ENABLE_EXPERIMENTAL_CUDA_OVERRIDE="ON",
        CC="/prefix/bin/cc",
        CXX="/prefix/bin/c++",
    )
    plan = MODULE.plan_config_py(
        _opts("config-py"), env, _tools(read_text_lines=lambda path: workaround_dir)
    )
    assert plan.remove_paths != []

    # Without CC/CXX in the environment the C/C++ comparison is skipped.
    env = _env(DART_ENABLE_EXPERIMENTAL_CUDA_OVERRIDE="ON")
    plan = MODULE.plan_config_py(
        _opts("config-py"), env, _tools(read_text_lines=lambda path: workaround_dir)
    )
    assert plan.remove_paths == []


def test_config_py_cuda_cache_reset_when_launcher_now_unset():
    cache = [
        "CMAKE_C_COMPILER:FILEPATH=/usr/bin/cc",
        "CMAKE_CXX_COMPILER:FILEPATH=/usr/bin/c++",
        "CMAKE_CUDA_COMPILER:FILEPATH=/prefix/bin/nvcc",
        "CMAKE_CUDA_COMPILER_LAUNCHER:STRING=sccache",
    ]
    env = _env(DART_ENABLE_EXPERIMENTAL_CUDA_OVERRIDE="ON")
    plan = MODULE.plan_config_py(
        _opts("config-py"), env, _tools(read_text_lines=lambda path: cache)
    )
    assert plan.remove_paths != []


def test_config_coverage_wipes_build_dir_and_hard_flags():
    env = _env(BUILD_TYPE="Debug", DART_VERBOSE="OFF")
    plan = MODULE.plan_config_coverage(_opts("config-coverage"), env, _tools())
    assert plan.remove_paths == ["build/default/cpp/Debug"]
    flags = _flags(plan)
    assert flags["DART_CODECOV"] == "ON"
    assert flags["DART_BUILD_PROFILE"] == "OFF"
    assert flags["DART_BUILD_DARTPY"] == "OFF"
    assert flags["DART_USE_SYSTEM_IMGUI"] == "ON"
    assert "DART_ENABLE_ASAN" not in flags


def test_config_coverage_defaults_pixi_env_name():
    env = {"CONDA_PREFIX": "/prefix", "BUILD_TYPE": "Debug", "DART_VERBOSE": "OFF"}
    plan = MODULE.plan_config_coverage(_opts("config-coverage"), env, _tools())
    assert plan.build_dir == "build/default/cpp/Debug"


def test_config_install_dir_has_no_build_type_suffix():
    env = _env(BUILD_TYPE="Release", DART_VERBOSE="OFF")
    plan = MODULE.plan_config_install(
        _opts("config-install", "--dartpy=OFF"), env, _tools()
    )
    assert plan.build_dir == "build/default/cpp"
    flags = _flags(plan)
    # dartpy comes from the task argument only — no env override.
    assert flags["DART_BUILD_DARTPY"] == "OFF"
    assert flags["DART_BUILD_PROFILE"] == "OFF"
    assert "nanobind_DIR" not in flags


def test_config_install_ignores_dartpy_env_override():
    env = _env(
        BUILD_TYPE="Release", DART_VERBOSE="OFF", DART_BUILD_DARTPY_OVERRIDE="OFF"
    )
    flags = _flags(MODULE.plan_config_install(_opts("config-install"), env, _tools()))
    assert flags["DART_BUILD_DARTPY"] == "ON"


def test_config_asan_decouples_dir_name_from_cmake_build_type():
    env = _env(BUILD_TYPE="asan", CMAKE_BUILD_TYPE="RelWithDebInfo")
    plan = MODULE.plan_config_asan(_opts("config-asan"), env, _tools())
    assert plan.build_dir == "build/default/cpp/asan"
    flags = _flags(plan)
    assert flags["CMAKE_BUILD_TYPE"] == "RelWithDebInfo"
    assert flags["DART_ENABLE_ASAN"] == "ON"
    assert "DART_BUILD_GUI" not in flags
    assert "nanobind_DIR" not in flags


def test_config_debug_env_driven_and_no_mold_flag():
    env = _env(BUILD_TYPE="FastDebug", DART_PROFILE_BUILTIN="OFF")
    plan = MODULE.plan_config_debug(_opts("config-debug"), env, _tools())
    assert plan.build_dir == "build/default/cpp/FastDebug"
    flags = _flags(plan)
    assert flags["CMAKE_BUILD_TYPE"] == "FastDebug"
    assert flags["DART_PROFILE_BUILTIN"] == "OFF"
    assert "DART_USE_MOLD" not in flags
