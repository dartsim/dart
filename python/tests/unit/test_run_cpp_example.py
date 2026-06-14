import importlib.util
import re
import sys
from pathlib import Path

import pytest

# Standalone filament-routed binaries that still resolve via EXAMPLE_SPECS.
# Former scene-style examples now live as dart-demos scenes and are exercised
# by ``test_normalize_target_redirects_demos_scenes`` instead.
FILAMENT_ROUTED_EXAMPLES = {
    "dartsim": (),
}


@pytest.fixture(scope="module")
def run_cpp_example():
    repo_root = Path(__file__).resolve().parents[3]
    script_path = repo_root / "scripts" / "run_cpp_example.py"

    spec = importlib.util.spec_from_file_location("run_cpp_example", script_path)
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("target", ["raylib", "dart_raylib", "raylib_gui"])
def test_normalize_target_rejects_removed_raylib(run_cpp_example, target):
    with pytest.raises(SystemExit) as exc:
        run_cpp_example._normalize_target(target)

    assert "Raylib GUI example has been removed" in str(exc.value)


@pytest.mark.parametrize("target", ["filament_gui", "dart_filament_gui"])
def test_normalize_target_rejects_backend_named_gui(run_cpp_example, target):
    with pytest.raises(SystemExit) as exc:
        run_cpp_example._normalize_target(target)

    assert "backend-named GUI example has been renamed" in str(exc.value)


def test_normalize_target_passthrough(run_cpp_example, capsys):
    assert run_cpp_example._normalize_target("demos") == "demos"
    assert capsys.readouterr().err == ""


@pytest.mark.parametrize(
    "target",
    [
        "hello_world",
        "gui_scene_diagnostics",
        "csv_logger",
        "headless_simulation",
        "unified_loading",
    ],
)
def test_normalize_target_rejects_retired_classic_standalone_examples(
    run_cpp_example, target
):
    with pytest.raises(SystemExit) as exc:
        run_cpp_example._normalize_target(target)

    message = str(exc.value)
    assert "retired from main" in message
    assert "release-6.*" in message


@pytest.mark.parametrize(
    "target",
    [
        "rigid_body",
        "planned_inverse_kinematics",
        "planned_simbicon_walking",
        "planned_operational_space_control",
        "planned_robot_puppets",
        "planned_mobile_manipulation",
        "deformable_body",
        "vbd_deformable",
    ],
)
def test_normalize_target_redirects_demos_scenes(run_cpp_example, target):
    with pytest.raises(SystemExit) as exc:
        run_cpp_example._normalize_target(target)

    message = str(exc.value)
    assert "dart-demos" in message
    assert f"--scene {target}" in message


@pytest.mark.parametrize("target", ["boxes"])
def test_normalize_target_rejects_removed_dart6_demos(run_cpp_example, target):
    with pytest.raises(SystemExit) as exc:
        run_cpp_example._normalize_target(target)

    message = str(exc.value)
    assert "DART 6 demo scene has been removed" in message
    assert "pixi run demos -- --list" in message


@pytest.mark.parametrize(
    ("target", "replacement"),
    [
        ("atlas_simbicon", "planned_simbicon_walking"),
        ("operational_space_control", "planned_operational_space_control"),
        ("wam_ikfast", "planned_inverse_kinematics"),
    ],
)
def test_normalize_target_mentions_planned_dart6_replacements(
    run_cpp_example, target, replacement
):
    with pytest.raises(SystemExit) as exc:
        run_cpp_example._normalize_target(target)

    message = str(exc.value)
    assert "DART 6 demo scene has been removed" in message
    assert f"pixi run demos -- --scene {replacement}" in message
    assert "pixi run demos -- --list" in message


@pytest.mark.parametrize(
    "target",
    [
        "collision_sandbox",
        "point_cloud",
        "polyhedron_visual",
        "point-cloud",
        "polyhedron-visual",
    ],
)
def test_normalize_target_routes_retired_collision_demos_to_py_demos(
    run_cpp_example, target
):
    with pytest.raises(SystemExit) as exc:
        run_cpp_example._normalize_target(target)

    message = str(exc.value)
    assert "DART 6 demo scene has been removed" in message
    assert "pixi run py-demos -- --scene rigid_contact_inspector" in message
    assert "pixi run py-demos -- --scene rigid_collision_query_options" in message
    assert "pixi run py-demos -- --scene rigid_collision_casts" in message
    assert "pixi run py-demos -- --list" in message
    assert "planned_collision_sandbox" not in message


@pytest.mark.parametrize(
    ("target", "expected"),
    [
        # Hyphen-spelled removed demos (the FILAMENT_ALL_SCENES spelling) must
        # still reach the removal/placeholder guidance instead of falling
        # through to a nonexistent CMake target.
        ("rigid-cubes", "DART 6 demo scene has been removed"),
        ("atlas-simbicon", "pixi run demos -- --scene planned_simbicon_walking"),
        ("point-cloud", "pixi run py-demos -- --scene rigid_contact_inspector"),
        ("rigid-body", "pixi run demos -- --scene rigid_body"),
    ],
)
def test_normalize_target_canonicalizes_hyphen_demo_ids(
    run_cpp_example, target, expected
):
    with pytest.raises(SystemExit) as exc:
        run_cpp_example._normalize_target(target)

    assert expected in str(exc.value)


@pytest.mark.parametrize("target", ["py-demos", "py_demos", "pydemos"])
def test_normalize_target_redirects_py_demos(run_cpp_example, target):
    # `py-demos` is the Python demos viewer (its own `pixi run py-demos` task),
    # not a C++ example target, so the C++ `ex` runner must redirect rather than
    # fail with an unknown-ninja-target error.
    with pytest.raises(SystemExit) as exc:
        run_cpp_example._normalize_target(target)

    message = str(exc.value)
    assert "pixi run py-demos" in message
    assert "ex demos" in message


def test_parse_args_requires_target(run_cpp_example, capsys):
    with pytest.raises(SystemExit) as exc:
        run_cpp_example.parse_args([])

    assert exc.value.code == 2
    captured = capsys.readouterr()
    assert "usage:" in captured.err
    assert "required: target" in captured.err


def test_parse_args_allows_pixi_help_without_target(run_cpp_example, capsys):
    with pytest.raises(SystemExit) as exc:
        run_cpp_example.parse_args(["--pixi-help"])

    assert exc.value.code == 0
    captured = capsys.readouterr()
    assert "CMake target / example binary name" in captured.out
    assert captured.err == ""


@pytest.mark.parametrize(
    ("target", "build_target", "binary_name", "requirements", "default_args"),
    [
        (
            "dartsim",
            "dartsim",
            "dartsim",
            ("filament", "simulation"),
            (),
        ),
        (
            "demos",
            "dart-demos",
            "dart-demos",
            ("filament", "simulation"),
            (),
        ),
    ],
)
def test_resolve_example(
    target,
    build_target,
    binary_name,
    requirements,
    default_args,
    run_cpp_example,
):
    spec = run_cpp_example._resolve_example(target)
    assert spec.build_target == build_target
    assert spec.binary_name == binary_name
    assert spec.requirements == requirements
    assert spec.default_args == default_args


@pytest.mark.parametrize(
    ("target", "default_args"), sorted(FILAMENT_ROUTED_EXAMPLES.items())
)
def test_filament_routed_examples_resolve_to_filament(
    target, default_args, run_cpp_example
):
    spec = run_cpp_example._resolve_example(target)
    assert spec.build_target == target
    assert spec.binary_name == target
    assert "filament" in spec.requirements
    assert spec.default_args == default_args


def test_dartsim_requires_experimental_world(run_cpp_example):
    spec = run_cpp_example._resolve_example("dartsim")

    assert "simulation" in spec.requirements


def _scene_from_default_args(default_args):
    args = list(default_args)
    if "--scene" not in args:
        return "mvp"

    index = args.index("--scene")
    assert index + 1 < len(args)
    return args[index + 1]


def test_filament_routed_scene_defaults_are_known(run_cpp_example):
    routed_scenes = {
        _scene_from_default_args(default_args)
        for default_args in FILAMENT_ROUTED_EXAMPLES.values()
    }

    assert routed_scenes <= set(run_cpp_example.FILAMENT_KNOWN_SCENES)


def test_filament_routed_examples_do_not_inject_scene_defaults():
    for example, default_args in FILAMENT_ROUTED_EXAMPLES.items():
        assert "--scene" not in default_args, example


def test_split_filament_scene_all_uses_smoke_scene_list(run_cpp_example):
    scenes, args, scene_option_explicit = run_cpp_example._split_filament_scenes(
        ["--frames", "1", "--scene", "all", "--width", "320"]
    )

    assert tuple(scenes) == run_cpp_example.FILAMENT_ALL_SCENES
    assert args == ["--frames", "1", "--width", "320"]
    assert scene_option_explicit is True
    assert "g1" not in scenes


def test_split_demos_scene_all_uses_world_catalog(run_cpp_example):
    spec = run_cpp_example._resolve_example("demos")
    all_scene_ids = run_cpp_example._all_scene_ids_for_spec(spec)
    scenes, args, scene_option_explicit = run_cpp_example._split_filament_scenes(
        ["--frames", "1", "--scene", "all", "--width", "320"],
        all_scene_ids,
    )

    assert all_scene_ids == run_cpp_example._DEMOS_SCENE_IDS
    assert tuple(scenes) == run_cpp_example._DEMOS_SCENE_IDS
    assert args == ["--frames", "1", "--width", "320"]
    assert scene_option_explicit is True
    assert "mvp" not in scenes
    assert "hello-world" not in scenes


def test_split_filament_scenes_marks_implicit_editor_default(run_cpp_example):
    scenes, args, scene_option_explicit = run_cpp_example._split_filament_scenes([])

    assert scenes == ["mvp"]
    assert args == []
    assert scene_option_explicit is False


def test_prepare_filament_run_args_keeps_no_scene_launch_for_editor(
    run_cpp_example, monkeypatch
):
    monkeypatch.setenv("DISPLAY", ":99")

    args = run_cpp_example._prepare_filament_run_args(
        [], "mvp", scene_option_explicit=False, multiple_scenes=False
    )

    assert "--scene" not in args


def test_prepare_filament_run_args_preserves_explicit_mvp_scene(
    run_cpp_example, monkeypatch
):
    monkeypatch.setenv("DISPLAY", ":99")

    args = run_cpp_example._prepare_filament_run_args(
        [], "mvp", scene_option_explicit=True, multiple_scenes=False
    )

    assert args[:2] == ["--scene", "mvp"]


def test_prepare_filament_run_args_preserves_mvp_scene_all(
    run_cpp_example, monkeypatch
):
    monkeypatch.setenv("DISPLAY", ":99")
    scenes, base_args, scene_option_explicit = run_cpp_example._split_filament_scenes(
        ["--scene", "all", "--frames", "1"]
    )

    args = run_cpp_example._prepare_filament_run_args(
        base_args,
        scenes[0],
        scene_option_explicit=scene_option_explicit,
        multiple_scenes=len(scenes) > 1,
    )

    assert args[:4] == ["--frames", "1", "--scene", "mvp"]


def test_filament_smoke_pattern_uses_scene_all_list(run_cpp_example):
    assert run_cpp_example.FILAMENT_SMOKE_PATTERN == "|".join(
        run_cpp_example._filament_smoke_test_name(scene)
        for scene in run_cpp_example.FILAMENT_ALL_SCENES
    )


def test_imgui_override_pins_variant_for_non_smoke_gui_runs(run_cpp_example):
    # The docked editor (dartsim without --scene) needs the fetched docking ImGui.
    assert run_cpp_example._imgui_override("dartsim", [], smoke=False) == "OFF"
    # Scene fixtures and other GUI examples must use system ImGui, even right
    # after an editor launch left the cache at OFF.
    assert (
        run_cpp_example._imgui_override("dartsim", ["--scene", "boxes"], smoke=False)
        == "ON"
    )
    assert run_cpp_example._imgui_override("rigid_cubes", [], smoke=False) == "ON"
    # Smoke runs keep their externally configured cache (no override pinned).
    assert run_cpp_example._imgui_override("dartsim", [], smoke=True) is None


def test_apply_imgui_override_overwrites_preset(run_cpp_example):
    # A scene-fixture launch must use system ImGui even when the editor preset
    # (DART_USE_SYSTEM_IMGUI_OVERRIDE=OFF, set by the `pixi run dartsim` task) is
    # already in the environment.
    env = {"DART_USE_SYSTEM_IMGUI_OVERRIDE": "OFF"}
    run_cpp_example._apply_imgui_override(
        env, "dartsim", ["--scene", "boxes"], smoke=False, requirements=("filament",)
    )
    assert env["DART_USE_SYSTEM_IMGUI_OVERRIDE"] == "ON"

    # The editor launch pins OFF.
    env = {}
    run_cpp_example._apply_imgui_override(
        env, "dartsim", [], smoke=False, requirements=("filament",)
    )
    assert env["DART_USE_SYSTEM_IMGUI_OVERRIDE"] == "OFF"

    # Non-GUI targets leave the variable untouched.
    env = {}
    run_cpp_example._apply_imgui_override(
        env, "some_tool", [], smoke=False, requirements=()
    )
    assert "DART_USE_SYSTEM_IMGUI_OVERRIDE" not in env


def _cmake_filament_smoke_scene_pairs(cmake_text):
    match = re.search(
        r"set\(\s*DART_GUI_FILAMENT_SMOKE_SCENE_PAIRS\s+(.*?)\n\s*\)",
        cmake_text,
        re.DOTALL,
    )
    assert match is not None

    tokens = re.findall(r"[a-z0-9_-]+", match.group(1))
    assert len(tokens) % 2 == 0
    return tuple(zip(tokens[0::2], tokens[1::2]))


def _filament_sources_cmake_text(run_cpp_example):
    repo_root = Path(run_cpp_example.__file__).resolve().parents[1]
    cmake_path = repo_root / "dart" / "gui" / "detail" / "backend_sources.cmake"
    return cmake_path.read_text(encoding="utf-8")


def _cmake_headless_smoke_test_call(cmake_text, first_argument):
    match = re.search(
        r"_dart_gui_filament_add_headless_smoke_test\(\s+"
        + re.escape(first_argument)
        + r"\s+(.*?)\n\s+\)",
        cmake_text,
        re.DOTALL,
    )
    assert match is not None
    return match.group(1)


def _cmake_tokens(cmake_text):
    return re.findall(r"\$\{[^}]+\}|[A-Za-z0-9_-]+", cmake_text)


def _cmake_option_value(cmake_text, option):
    tokens = _cmake_tokens(cmake_text)
    assert option in tokens
    index = tokens.index(option)
    assert index + 1 < len(tokens)
    return tokens[index + 1]


def test_filament_scene_all_matches_registered_smoke_tests(run_cpp_example):
    cmake_text = _filament_sources_cmake_text(run_cpp_example)

    scene_pairs = _cmake_filament_smoke_scene_pairs(cmake_text)
    registered_scenes = tuple(scene for _suffix, scene in scene_pairs)
    registered_tests = (
        "EXAMPLE_dartsim_headless_smoke",
        *(f"EXAMPLE_dartsim_{suffix}_headless_smoke" for suffix, _scene in scene_pairs),
    )

    assert registered_scenes == run_cpp_example.FILAMENT_ALL_SCENES[1:]
    assert registered_tests == tuple(run_cpp_example.FILAMENT_SMOKE_PATTERN.split("|"))


def test_filament_smoke_cmake_registers_analysis_modes(run_cpp_example):
    cmake_text = _filament_sources_cmake_text(run_cpp_example)
    default_call = _cmake_headless_smoke_test_call(
        cmake_text, "EXAMPLE_dartsim_headless_smoke"
    )
    scene_call = _cmake_headless_smoke_test_call(cmake_text, "${_test_name}")

    # The no-scene launch is the DART 7 World editor, whose minimal default
    # scene uses the non-blank (basic) check; the legacy --scene fixtures also
    # use the basic check.
    assert "ANALYZE" in _cmake_tokens(default_call)
    assert _cmake_option_value(default_call, "ANALYSIS_MODE") == "basic"
    assert "ANALYZE" in _cmake_tokens(scene_call)
    assert _cmake_option_value(scene_call, "ANALYSIS_MODE") == "basic"


def test_run_args_with_defaults_does_not_inject_g1_scene(run_cpp_example):
    spec = run_cpp_example._resolve_example("g1_puppet")

    assert run_cpp_example._run_args_with_defaults(spec, ["--frames", "1"]) == [
        "--frames",
        "1",
    ]


def test_run_args_with_defaults_preserves_explicit_scene(run_cpp_example):
    spec = run_cpp_example._resolve_example("g1_puppet")

    assert run_cpp_example._run_args_with_defaults(
        spec, ["--scene", "mvp", "--frames", "1"]
    ) == ["--scene", "mvp", "--frames", "1"]


def test_cmake_cache_bool(run_cpp_example, tmp_path):
    cache_path = tmp_path / "CMakeCache.txt"
    cache_path.write_text("DART_BUILD_GUI:BOOL=ON\n", encoding="utf-8")
    assert run_cpp_example._cmake_cache_bool(tmp_path, "DART_BUILD_GUI") is True

    cache_path.write_text("DART_BUILD_GUI:BOOL=OFF\n", encoding="utf-8")
    assert run_cpp_example._cmake_cache_bool(tmp_path, "DART_BUILD_GUI") is False

    cache_path.write_text("DART_BUILD_GUI:BOOL=maybe\n", encoding="utf-8")
    assert run_cpp_example._cmake_cache_bool(tmp_path, "DART_BUILD_GUI") is None

    cache_path.write_text("UNRELATED:BOOL=ON\n", encoding="utf-8")
    assert run_cpp_example._cmake_cache_bool(tmp_path, "DART_BUILD_GUI") is None


def test_run_filament_smoke_uses_no_tests_error_when_supported(
    run_cpp_example, tmp_path, monkeypatch
):
    calls = []

    def fake_run_with_optional_xvfb(command, env, use_xvfb):
        calls.append((command, env, use_xvfb))

    monkeypatch.setenv("DISPLAY", ":99")
    monkeypatch.setattr(
        run_cpp_example, "_run_with_optional_xvfb", fake_run_with_optional_xvfb
    )
    monkeypatch.setattr(run_cpp_example, "_ctest_supports_no_tests_error", lambda: True)

    env = {"EXAMPLE": "1"}
    run_cpp_example._run_filament_smoke(tmp_path, env)

    assert len(calls) == 1
    command, runtime_env, use_xvfb = calls[0]
    assert command[:3] == ["ctest", "--test-dir", str(tmp_path)]
    assert "--no-tests=error" in command
    assert runtime_env["EXAMPLE"] == "1"
    assert use_xvfb is False


def test_run_filament_smoke_probes_tests_for_old_ctest(
    run_cpp_example, tmp_path, monkeypatch
):
    calls = []
    probes = []

    def fake_run_with_optional_xvfb(command, env, use_xvfb):
        calls.append((command, env, use_xvfb))

    def fake_validate(build_dir, env):
        probes.append((build_dir, env))

    monkeypatch.setenv("DISPLAY", ":99")
    monkeypatch.setattr(
        run_cpp_example, "_ctest_supports_no_tests_error", lambda: False
    )
    monkeypatch.setattr(
        run_cpp_example, "_validate_filament_smoke_tests_discovered", fake_validate
    )
    monkeypatch.setattr(
        run_cpp_example, "_run_with_optional_xvfb", fake_run_with_optional_xvfb
    )

    env = {"EXAMPLE": "1"}
    run_cpp_example._run_filament_smoke(tmp_path, env)

    assert len(probes) == 1
    assert probes[0][0] == tmp_path
    assert probes[0][1]["EXAMPLE"] == "1"
    assert len(calls) == 1
    command, runtime_env, use_xvfb = calls[0]
    assert command[:3] == ["ctest", "--test-dir", str(tmp_path)]
    assert "--no-tests=error" not in command
    assert runtime_env["EXAMPLE"] == "1"
    assert use_xvfb is False


@pytest.mark.parametrize(
    ("output", "expected"),
    [
        ("ctest version 3.26.4", (3, 26, 4)),
        ("cmake version 4.2", (4, 2, 0)),
        ("not a version", None),
    ],
)
def test_parse_cmake_version(run_cpp_example, output, expected):
    assert run_cpp_example._parse_cmake_version(output) == expected


@pytest.mark.parametrize(
    ("env", "expected"),
    [
        ({}, False),
        ({"DISPLAY": ":99"}, True),
        ({"WAYLAND_DISPLAY": "wayland-0"}, True),
        ({"DISPLAY": "", "WAYLAND_DISPLAY": ""}, False),
    ],
)
def test_has_linux_display(run_cpp_example, env, expected):
    assert run_cpp_example._has_linux_display(env) is expected
