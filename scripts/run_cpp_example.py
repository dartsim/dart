#!/usr/bin/env python3
"""Build and run a C++ example with optional runtime arguments."""

from __future__ import annotations

import argparse
import os
import platform
import re
import shutil
import subprocess
import sys
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

FILAMENT_ALL_SCENES = (
    "mvp",
    "hello-world",
    "boxes",
    "hardcoded-design",
    "rigid-chain",
    "rigid-loop",
    "mixed-chain",
    "coupler-constraint",
    "add-delete-skels",
    "vehicle",
    "hybrid-dynamics",
    "joint-constraints",
    "free-joint-cases",
    "human-joint-limits",
    "lcp-physics",
    "mimic-pendulums",
    "atlas-puppet",
    "hubo-puppet",
    "atlas-simbicon",
    "operational-space-control",
    "wam-ikfast",
    "fetch",
    "tinkertoy",
    "drag-and-drop",
    "simple-frames",
    "soft-bodies",
    "point-cloud",
    "capsule-ground-contact",
    "simulation-event-handler",
    "polyhedron",
    "heightmap",
)
FILAMENT_EXTRA_KNOWN_SCENES = ("g1",)
FILAMENT_KNOWN_SCENES = (*FILAMENT_ALL_SCENES, *FILAMENT_EXTRA_KNOWN_SCENES)


def _filament_smoke_test_name(scene: str) -> str:
    if scene == "mvp":
        return "EXAMPLE_dartsim_headless_smoke"
    suffix = scene.replace("-", "_")
    return f"EXAMPLE_dartsim_{suffix}_headless_smoke"


FILAMENT_SMOKE_PATTERN = "|".join(
    _filament_smoke_test_name(scene) for scene in FILAMENT_ALL_SCENES
)


@dataclass(frozen=True)
class ExampleSpec:
    build_target: str
    binary_name: str
    requirements: tuple[str, ...] = ()
    default_args: tuple[str, ...] = ()


EXAMPLE_SPECS = {
    "dartsim": ExampleSpec("dartsim", "dartsim", ("filament", "simulation")),
    # The consolidated World demos app needs the current World implementation
    # component built as well.
    "demos": ExampleSpec("dart-demos", "dart-demos", ("filament", "simulation")),
}

# World-scene demos hosted by `dart-demos`.
_DEMOS_SCENE_IDS = (
    "rigid_body",
    "planned_inverse_kinematics",
    "planned_simbicon_walking",
    "planned_operational_space_control",
    "planned_robot_puppets",
    "planned_mobile_manipulation",
    "deformable_body",
    "vbd_deformable",
)

_DART6_DEMO_PLACEHOLDER_SCENE_IDS = {
    "atlas_puppet": "planned_robot_puppets",
    "atlas_simbicon": "planned_simbicon_walking",
    "biped_stand": "planned_simbicon_walking",
    "fetch": "planned_mobile_manipulation",
    "g1_puppet": "planned_robot_puppets",
    "hubo_puppet": "planned_robot_puppets",
    "operational_space_control": "planned_operational_space_control",
    "vehicle": "planned_mobile_manipulation",
    "wam_ikfast": "planned_inverse_kinematics",
}

_COLLISION_DEBUG_PY_DEMO_SCENE_IDS = (
    "rigid_contact_inspector",
    "rigid_collision_query_options",
    "rigid_collision_casts",
)

_DART6_DEMO_PY_DEMO_REPLACEMENT_SCENE_IDS = {
    "collision_sandbox": _COLLISION_DEBUG_PY_DEMO_SCENE_IDS,
    "point_cloud": _COLLISION_DEBUG_PY_DEMO_SCENE_IDS,
    "polyhedron_visual": _COLLISION_DEBUG_PY_DEMO_SCENE_IDS,
}

_REMOVED_DART6_DEMOS_SCENE_IDS = (
    "add_delete_skels",
    "atlas_puppet",
    "atlas_simbicon",
    "biped_stand",
    "box_stacking",
    "boxes",
    "capsule_ground_contact",
    "collision_sandbox",
    "coupler_constraint",
    "drag_and_drop",
    "empty",
    "experimental_deformable",
    "experimental_rigid_body",
    "experimental_vbd",
    "fetch",
    "free_joint_cases",
    "g1_puppet",
    "hardcoded_design",
    "heightmap",
    "hubo_puppet",
    "human_joint_limits",
    "hybrid_dynamics",
    "imgui",
    "joint_constraints",
    "lcp_physics",
    "mimic_pendulums",
    "mixed_chain",
    "operational_space_control",
    "point_cloud",
    "polyhedron_visual",
    "rigid_chain",
    "rigid_cubes",
    "rigid_loop",
    "rigid_shapes",
    "shapes",
    "simple_frames",
    "simulation_event_handler",
    "soft_bodies",
    "tinkertoy",
    "vehicle",
    "wam_ikfast",
)

REMOVED_EXAMPLES = {
    "raylib": "The Raylib GUI example has been removed. Use `pixi run ex dartsim` "
    "or one of the DART GUI example names instead.",
    "dart_raylib": "The Raylib GUI example has been removed. Use `pixi run ex dartsim` "
    "or one of the DART GUI example names instead.",
    "raylib_gui": "The Raylib GUI example has been removed. Use `pixi run ex dartsim` "
    "or one of the DART GUI example names instead.",
    "filament_gui": "The backend-named GUI example has been renamed. Use "
    "`pixi run ex dartsim` or one of the DART GUI example names instead.",
    "dart_filament_gui": "The backend-named GUI example has been renamed. Use "
    "`pixi run ex dartsim` or one of the DART GUI example names instead.",
    "hello_world": "The classic C++ hello_world standalone example was retired "
    "from main during the DART 7 World promotion. Run the DART 7 demo app with "
    "`pixi run demos -- --scene rigid_body`, or use a release-6.* branch for "
    "the DART 6 example source.",
    "gui_scene_diagnostics": "The classic gui_scene_diagnostics standalone "
    "example was retired from main during the DART 7 World promotion. Use "
    "`pixi run demos -- --list` for maintained DART 7 GUI scenes, or use a "
    "release-6.* branch for the DART 6 diagnostic source.",
    "csv_logger": "The csv_logger standalone example was retired from main "
    "during the DART 7 World promotion because it depended on DART 6 "
    "whole-World loading. Use a release-6.* branch for the retired source.",
    "headless_simulation": "The headless_simulation standalone example was "
    "retired from main during the DART 7 World promotion because it depended "
    "on DART 6 whole-World loading. Use a release-6.* branch for the retired "
    "source.",
    "unified_loading": "The unified_loading standalone example was retired "
    "from main during the DART 7 World promotion because it depended on the "
    "removed public whole-World IO API. Use dart::io::readSkeleton for "
    "DART 7 skeleton loading, or use a release-6.* branch for DART 6 "
    "whole-World loading parity.",
}

# The Python demos viewer (`pixi run py-demos` / `python -m examples.demos`) is
# not a C++ example, so it has no CMake/ninja build target. Catch the common
# attempts to launch it through the C++ `ex` runner and point at its own task.
_PYTHON_DEMO_ALIASES = ("py-demos", "py_demos", "pydemos")


def parse_args(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description=__doc__,
        add_help=False,  # Allow --help to pass through to the example.
    )
    parser.add_argument(
        "target",
        nargs="?",
        default=None,
        help="CMake target / example binary name (e.g., atlas_simbicon)",
    )
    parser.add_argument(
        "--build-type",
        default=os.environ.get("BUILD_TYPE", "Release"),
        help="CMake build type to use (default: BUILD_TYPE or Release)",
    )
    parser.add_argument(
        "--pixi-smoke",
        action="store_true",
        help="Run this example's registered smoke tests instead of the binary.",
    )
    parser.add_argument(
        "--pixi-help",
        action="store_true",
        help="Show this help for the pixi wrapper.",
    )
    known, unknown = parser.parse_known_args(argv)

    if known.pixi_help:
        parser.print_help()
        sys.exit(0)
    if known.target is None:
        parser.print_usage(sys.stderr)
        parser.exit(
            2,
            f"{parser.prog}: error: the following arguments are required: target\n",
        )

    return known, unknown


def _normalize_target(target: str) -> str:
    if target in REMOVED_EXAMPLES:
        raise SystemExit(REMOVED_EXAMPLES[target])
    if target in _PYTHON_DEMO_ALIASES:
        raise SystemExit(
            "'py-demos' is the Python demos viewer, not a C++ example target, "
            "so it has no ninja build target. Run it with its own task:\n"
            "  pixi run py-demos                 # launch the first scene\n"
            "  pixi run py-demos -- --list       # list the scene catalog\n"
            "  pixi run py-demos -- --scene <id> # launch a specific scene\n"
            "The C++ demos viewer is a separate app: pixi run ex demos"
        )
    # DART 6 demo ids were historically spelled with hyphens (the spelling
    # FILAMENT_ALL_SCENES still uses), so canonicalize to the underscore scene-id
    # form before the demo-scene lookups below. Without this, e.g.
    # `pixi run ex rigid-cubes` would skip the removal guidance and fall through
    # to building a nonexistent CMake target.
    canonical = target.replace("-", "_")
    placeholder_scene = _DART6_DEMO_PLACEHOLDER_SCENE_IDS.get(canonical)
    if placeholder_scene is not None:
        raise SystemExit(
            f"The '{target}' DART 6 demo scene has been removed from "
            "dart-demos. A planned World-native placeholder is available:\n"
            f"  pixi run demos -- --scene {placeholder_scene}\n"
            "Run `pixi run demos -- --list` for the current World demo catalog."
        )
    py_demo_scenes = _DART6_DEMO_PY_DEMO_REPLACEMENT_SCENE_IDS.get(canonical)
    if py_demo_scenes is not None:
        scene_commands = "\n".join(
            f"  pixi run py-demos -- --scene {scene_id}" for scene_id in py_demo_scenes
        )
        raise SystemExit(
            f"The '{target}' DART 6 demo scene has been removed from "
            "dart-demos. Use maintained Python GUI rows for the concrete "
            f"collision-debugging workflows:\n{scene_commands}\n"
            "Run `pixi run py-demos -- --list` for the current Python demo "
            "catalog."
        )
    if canonical in _REMOVED_DART6_DEMOS_SCENE_IDS:
        raise SystemExit(
            f"The '{target}' DART 6 demo scene has been removed from "
            "dart-demos. Run `pixi run demos -- --list` for the current "
            "World demo catalog."
        )
    if canonical in _DEMOS_SCENE_IDS:
        raise SystemExit(
            f"The '{target}' GUI example is now a scene in the dart-demos app. "
            f"Run it with: pixi run demos -- --scene {canonical}"
        )
    return target


def _resolve_example(target: str) -> ExampleSpec:
    return EXAMPLE_SPECS.get(target, ExampleSpec(target, target))


def _build_dir(build_type: str) -> Path:
    env_name = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    return Path("build") / env_name / "cpp" / build_type


def _cmake_cache_bool(build_dir: Path, option: str) -> bool | None:
    cache_path = build_dir / "CMakeCache.txt"
    if not cache_path.is_file():
        return None

    needle = f"{option}:BOOL="
    with cache_path.open("r", encoding="utf-8", errors="ignore") as cache:
        for line in cache:
            if not line.startswith(needle):
                continue
            value = line.strip().split("=", maxsplit=1)[-1].upper()
            if value in {"ON", "TRUE", "1"}:
                return True
            if value in {"OFF", "FALSE", "0"}:
                return False
            return None
    return None


def _cache_bool_matches(build_dir: Path, option: str, desired: str) -> bool:
    value = _cmake_cache_bool(build_dir, option)
    if value is None:
        return False
    return value == (desired.upper() in {"ON", "TRUE", "1"})


def _env_option(env: dict[str, str], name: str, default: str) -> str:
    return env.get(name, default)


def _is_linux_x86_64() -> bool:
    machine = platform.machine().lower()
    return sys.platform.startswith("linux") and machine in {"x86_64", "amd64"}


def _has_filament_root(env: dict[str, str]) -> bool:
    return bool(env.get("Filament_ROOT") or env.get("FILAMENT_ROOT"))


def _default_fetch_filament(env: dict[str, str]) -> str:
    if _has_filament_root(env):
        return "OFF"
    return "ON" if _is_linux_x86_64() else "OFF"


def _default_use_system_filament(fetch_filament: str) -> str:
    return "OFF" if fetch_filament.upper() == "ON" else "ON"


def _prepend_env_path(env: dict[str, str], name: str, value: Path) -> None:
    current = env.get(name)
    env[name] = f"{value}{os.pathsep}{current}" if current else str(value)


def _apply_libcxx_prefix(env: dict[str, str]) -> None:
    prefix = env.get("LIBCXX_PREFIX")
    if not prefix:
        conda_prefix = env.get("CONDA_PREFIX")
        if conda_prefix:
            lib_dir = Path(conda_prefix) / "lib"
            if any(lib_dir.glob("libc++.*")) and any(lib_dir.glob("libc++abi.*")):
                prefix = conda_prefix
    if not prefix:
        return

    lib_dir = Path(prefix) / "lib"
    _prepend_env_path(env, "CMAKE_LIBRARY_PATH", lib_dir)
    if sys.platform.startswith("linux"):
        _prepend_env_path(env, "LD_LIBRARY_PATH", lib_dir)
    elif sys.platform == "darwin":
        _prepend_env_path(env, "DYLD_LIBRARY_PATH", lib_dir)
    elif os.name == "nt":
        _prepend_env_path(env, "PATH", Path(prefix) / "bin")


def _configure(
    build_dir: Path, definitions: dict[str, str], env: dict[str, str]
) -> None:
    cmd = ["cmake", "-S", ".", "-B", str(build_dir)]
    cmd.extend(f"-D{name}={value}" for name, value in definitions.items())
    subprocess.run(cmd, check=True, env=env)


def _option_override(
    definitions: dict[str, str], env: dict[str, str], option: str, env_name: str
) -> None:
    if env_name in env:
        definitions[option] = env[env_name]


def _ensure_filament(build_dir: Path, env: dict[str, str], smoke: bool) -> None:
    fetch_filament = _env_option(
        env, "DART_FETCH_FILAMENT_OVERRIDE", _default_fetch_filament(env)
    )
    use_system_filament = _env_option(
        env,
        "DART_USE_SYSTEM_FILAMENT_OVERRIDE",
        _default_use_system_filament(fetch_filament),
    )

    desired = {
        "DART_BUILD_GUI": "ON",
        "DART_BUILD_EXAMPLES": "ON",
        "DART_BUILD_TUTORIALS": "OFF",
        "DART_USE_SYSTEM_FILAMENT": use_system_filament,
        "DART_FETCH_FILAMENT": fetch_filament,
    }
    if smoke:
        desired["DART_BUILD_TESTS"] = "ON"
        desired["DART_ENABLE_GUI_FILAMENT_SMOKE_TESTS"] = "ON"

    _option_override(desired, env, "DART_BUILD_DARTPY", "DART_BUILD_DARTPY_OVERRIDE")
    _option_override(desired, env, "DART_BUILD_TESTS", "DART_BUILD_TESTS_OVERRIDE")
    # The standalone dartsim editor needs the ImGui docking API, which only the
    # fetched docking branch provides (DART_USE_SYSTEM_IMGUI=OFF).
    _option_override(
        desired, env, "DART_USE_SYSTEM_IMGUI", "DART_USE_SYSTEM_IMGUI_OVERRIDE"
    )

    definitions = {
        option: value
        for option, value in desired.items()
        if not _cache_bool_matches(build_dir, option, value)
    }
    if not definitions:
        return

    print(
        "Configuring DART GUI example requirements "
        f"({', '.join(f'{k}={v}' for k, v in definitions.items())})...",
        file=sys.stderr,
    )
    _configure(build_dir, definitions, env)


def _ensure_target_requirements(
    build_dir: Path, spec: ExampleSpec, env: dict[str, str], smoke: bool
) -> None:
    if "filament" in spec.requirements:
        _ensure_filament(build_dir, env, smoke)


def ensure_build_exists(build_dir: Path, build_type: str) -> None:
    if build_dir.exists():
        return

    msg = (
        f"Build directory {build_dir} does not exist.\n"
        f"Run `pixi run config --build_type {build_type}` first, "
        "then re-run this command."
    )
    raise SystemExit(msg)


def _build_example(build_dir: Path, build_target: str, env: dict[str, str]) -> None:
    subprocess.run(
        [
            sys.executable,
            "scripts/cmake_build.py",
            "--build-dir",
            str(build_dir),
            "--target",
            build_target,
        ],
        check=True,
        env=env,
    )


def _binary_path(build_dir: Path, binary_name: str) -> Path:
    suffix = ".exe" if os.name == "nt" else ""
    return build_dir / "bin" / f"{binary_name}{suffix}"


def _has_arg(run_args: list[str], *names: str) -> bool:
    return any(arg in names for arg in run_args)


def _run_args_with_defaults(spec: ExampleSpec, run_args: list[str]) -> list[str]:
    if not spec.default_args:
        return list(run_args)

    default_args = list(spec.default_args)
    if _has_arg(run_args, "--scene") and _has_arg(default_args, "--scene"):
        for index, arg in enumerate(default_args):
            if arg == "--scene":
                del default_args[index : index + 2]
                break
    return [*default_args, *run_args]


def _filament_screenshot_path(scene: str) -> Path:
    env_name = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    scene_suffix = scene.replace("-", "_")
    return Path("build") / env_name / f"dartsim_{scene_suffix}.ppm"


def _all_scene_ids_for_spec(spec: ExampleSpec) -> tuple[str, ...]:
    if spec.binary_name == "dart-demos":
        return _DEMOS_SCENE_IDS
    return FILAMENT_ALL_SCENES


def _split_filament_scenes(
    run_args: list[str], all_scene_ids: tuple[str, ...] = FILAMENT_ALL_SCENES
) -> tuple[list[str], list[str], bool]:
    """Return scenes, remaining args, and whether --scene was user-supplied."""
    args = list(run_args)
    for index, arg in enumerate(args):
        if arg != "--scene" or index + 1 >= len(args):
            continue
        scene = args[index + 1]
        del args[index : index + 2]
        if scene == "all":
            return list(all_scene_ids), args, True
        return [scene], args, True
    return ["mvp"], args, False


def _path_with_scene(path: Path, scene: str) -> Path:
    scene_suffix = scene.replace("-", "_")
    return path.with_name(f"{path.stem}_{scene_suffix}{path.suffix}")


def _has_linux_display(env: Mapping[str, str]) -> bool:
    return bool(env.get("DISPLAY") or env.get("WAYLAND_DISPLAY"))


def _append_filament_dimension_arg(
    args: list[str],
    flag: str,
    env_name: str,
    headless_default: str,
    headless: bool,
) -> None:
    if _has_arg(args, flag):
        return

    value = os.environ.get(env_name)
    if not value and headless:
        value = headless_default
    if value:
        args.extend([flag, value])


def _prepare_filament_run_args(
    run_args: list[str],
    scene: str,
    scene_option_explicit: bool,
    multiple_scenes: bool,
) -> list[str]:
    args = list(run_args)
    if _has_arg(args, "--help", "-h"):
        return args

    headless = _has_arg(args, "--headless") or (
        sys.platform.startswith("linux") and not _has_linux_display(os.environ)
    )
    # No --scene means the standalone dartsim editor; explicit --scene mvp means
    # the legacy MVP fixture and must stay explicit after scene expansion.
    if (scene != "mvp" or scene_option_explicit) and not _has_arg(args, "--scene"):
        args.extend(["--scene", scene])
    if headless and not _has_arg(args, "--headless"):
        args.append("--headless")
    if headless and not _has_arg(args, "--frames"):
        args.extend(["--frames", os.environ.get("DART_GUI_FILAMENT_FRAMES", "10")])
    _append_filament_dimension_arg(
        args, "--width", "DART_GUI_FILAMENT_WIDTH", "640", headless
    )
    _append_filament_dimension_arg(
        args, "--height", "DART_GUI_FILAMENT_HEIGHT", "480", headless
    )
    if headless:
        screenshot = os.environ.get("DART_GUI_FILAMENT_SCREENSHOT")
        if _has_arg(args, "--screenshot"):
            if multiple_scenes:
                for index, arg in enumerate(args):
                    if arg == "--screenshot" and index + 1 < len(args):
                        path = _path_with_scene(Path(args[index + 1]), scene)
                        args[index + 1] = str(path)
                        path.parent.mkdir(parents=True, exist_ok=True)
                        break
            return args

        path = Path(screenshot) if screenshot else _filament_screenshot_path(scene)
        if multiple_scenes and screenshot:
            path = _path_with_scene(path, scene)
        path.parent.mkdir(parents=True, exist_ok=True)
        args.extend(["--screenshot", str(path)])
    return args


def _prepend_runtime_library_path(env: dict[str, str], build_dir: Path) -> None:
    lib_dir = build_dir / "lib"
    if sys.platform.startswith("linux"):
        _prepend_env_path(env, "LD_LIBRARY_PATH", lib_dir)
    elif sys.platform == "darwin":
        _prepend_env_path(env, "DYLD_LIBRARY_PATH", lib_dir)
    elif os.name == "nt":
        _prepend_env_path(env, "PATH", lib_dir)


def _runtime_env(
    env: dict[str, str], build_dir: Path, software_gl: bool
) -> dict[str, str]:
    runtime_env = env.copy()
    _apply_libcxx_prefix(runtime_env)
    _prepend_runtime_library_path(runtime_env, build_dir)
    if software_gl:
        runtime_env.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
        runtime_env.setdefault("MESA_LOADER_DRIVER_OVERRIDE", "llvmpipe")
        egl_vendor = Path("/usr/share/glvnd/egl_vendor.d/50_mesa.json")
        if egl_vendor.is_file():
            runtime_env.setdefault("__EGL_VENDOR_LIBRARY_FILENAMES", str(egl_vendor))
    return runtime_env


def _uses_headless_filament(run_args: list[str]) -> bool:
    return _has_arg(run_args, "--headless") or (
        sys.platform.startswith("linux") and not _has_linux_display(os.environ)
    )


def _run_with_optional_xvfb(
    command: list[str], env: dict[str, str], use_xvfb: bool
) -> None:
    final_command = command
    if use_xvfb:
        xvfb_run = shutil.which("xvfb-run")
        if not xvfb_run:
            raise SystemExit(
                "DART GUI examples need DISPLAY or xvfb-run for the "
                "Filament OpenGL backend on Linux."
            )
        final_command = [
            xvfb_run,
            "-a",
            "-s",
            "-screen 0 1280x720x24",
            *command,
        ]

    subprocess.run(final_command, check=True, env=env)


def _parse_cmake_version(output: str) -> tuple[int, int, int] | None:
    match = re.search(r"\bversion\s+(\d+)\.(\d+)(?:\.(\d+))?", output)
    if not match:
        return None
    patch = match.group(3)
    return (int(match.group(1)), int(match.group(2)), int(patch or 0))


def _ctest_supports_no_tests_error() -> bool:
    result = subprocess.run(
        ["ctest", "--version"],
        check=False,
        capture_output=True,
        text=True,
    )
    version = _parse_cmake_version(result.stdout + result.stderr)
    return version is not None and version >= (3, 26, 0)


def _validate_filament_smoke_tests_discovered(
    build_dir: Path, env: dict[str, str]
) -> None:
    result = subprocess.run(
        [
            "ctest",
            "--test-dir",
            str(build_dir),
            "-N",
            "-R",
            FILAMENT_SMOKE_PATTERN,
        ],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )
    output = result.stdout + result.stderr
    match = re.search(r"Total Tests:\s*(\d+)", output)
    if match is None or int(match.group(1)) == 0:
        raise SystemExit("No DART GUI smoke tests were discovered.")


def _run_filament_smoke(build_dir: Path, env: dict[str, str]) -> None:
    runtime_env = _runtime_env(env, build_dir, software_gl=True)
    command = [
        "ctest",
        "--test-dir",
        str(build_dir),
        "-R",
        FILAMENT_SMOKE_PATTERN,
        "--output-on-failure",
    ]
    if _ctest_supports_no_tests_error():
        command.insert(-1, "--no-tests=error")
    else:
        _validate_filament_smoke_tests_discovered(build_dir, runtime_env)
    use_xvfb = sys.platform.startswith("linux") and not _has_linux_display(os.environ)
    _run_with_optional_xvfb(command, runtime_env, use_xvfb)


def _run_example_binary(
    build_dir: Path,
    spec: ExampleSpec,
    run_args: list[str],
    env: dict[str, str],
) -> None:
    run_args = _run_args_with_defaults(spec, run_args)
    binary = _binary_path(build_dir, spec.binary_name)
    if not binary.exists():
        raise SystemExit(f"Binary not found: {binary}")

    if "filament" not in spec.requirements:
        runtime_env = _runtime_env(env, build_dir, software_gl=False)
        subprocess.run([str(binary), *run_args], check=True, env=runtime_env)
        return

    scenes, base_args, scene_option_explicit = _split_filament_scenes(
        run_args, _all_scene_ids_for_spec(spec)
    )
    multiple_scenes = len(scenes) > 1
    for scene in scenes:
        prepared_args = _prepare_filament_run_args(
            base_args, scene, scene_option_explicit, multiple_scenes
        )
        headless = _uses_headless_filament(prepared_args)
        runtime_env = _runtime_env(env, build_dir, software_gl=headless)
        command = [str(binary), *prepared_args]
        print("Running:", " ".join(command))
        use_xvfb = (
            headless
            and sys.platform.startswith("linux")
            and not _has_linux_display(os.environ)
        )
        _run_with_optional_xvfb(command, runtime_env, use_xvfb)


def _imgui_override(target: str, run_args: list[str], smoke: bool) -> str | None:
    """ImGui variant to pin for a GUI run, or None to leave the cache as-is.

    The standalone dartsim editor needs the ImGui docking branch that only the
    fetched ImGui provides (DART_USE_SYSTEM_IMGUI=OFF); every other GUI path
    (scene fixtures, other GUI examples) must use system ImGui (ON). Pinning the
    value per run keeps a launch from inheriting a stale cache state left by a
    previous run (e.g. OFF after an editor launch). Smoke runs keep their
    externally configured cache.
    """
    if smoke:
        return None
    launching_docked_editor = target == "dartsim" and not _has_arg(run_args, "--scene")
    return "OFF" if launching_docked_editor else "ON"


def _apply_imgui_override(
    env: dict[str, str],
    target: str,
    run_args: list[str],
    smoke: bool,
    requirements: tuple[str, ...],
) -> None:
    """Pin DART_USE_SYSTEM_IMGUI_OVERRIDE for a GUI run.

    Assigns directly rather than via setdefault: the `pixi run dartsim` task
    pre-sets this variable to OFF, so per-run pinning must overwrite it or a
    scene-fixture launch (`pixi run dartsim -- --scene ...`) would stay on the
    editor's docking build instead of system ImGui.
    """
    if "filament" not in requirements:
        return
    override = _imgui_override(target, run_args, smoke)
    if override is not None:
        env["DART_USE_SYSTEM_IMGUI_OVERRIDE"] = override


def run(
    target: str,
    build_type: str,
    run_args: list[str],
    smoke: bool,
) -> int:
    build_dir = _build_dir(build_type)
    ensure_build_exists(build_dir, build_type)

    target = _normalize_target(target)
    spec = _resolve_example(target)

    env = os.environ.copy()
    env["BUILD_TYPE"] = build_type
    env["CMAKE_BUILD_DIR"] = str(build_dir)
    _apply_libcxx_prefix(env)

    # Pin the ImGui variant for GUI runs so a launch never inherits a stale
    # build-cache state from a previous run: the editor needs the docking branch
    # (OFF), while scene fixtures and other GUI examples need system ImGui (ON).
    _apply_imgui_override(env, target, run_args, smoke, spec.requirements)

    _ensure_target_requirements(build_dir, spec, env, smoke)
    _build_example(build_dir, spec.build_target, env)

    if smoke:
        if "filament" not in spec.requirements:
            raise SystemExit(f"No smoke mode is registered for example {target!r}")
        _run_filament_smoke(build_dir, env)
    else:
        _run_example_binary(build_dir, spec, run_args, env)
    return 0


def main(argv: list[str]) -> int:
    args, run_args = parse_args(argv)
    return run(args.target, args.build_type, run_args, args.pixi_smoke)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
