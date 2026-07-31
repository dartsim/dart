"""Tests for DART 6.20's deterministic AI infrastructure checks."""

import copy
import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


infra = _load(
    "check_ai_infrastructure", ROOT / "scripts" / "check_ai_infrastructure.py"
)
hook = _load("check_agent_hook", ROOT / "scripts" / "check_agent_hook.py")
bridge = _load("pretool_guard_bridge", ROOT / "scripts" / "pretool_guard_bridge.py")
setup = _load("setup_ai", ROOT / "scripts" / "setup_ai.py")


def test_repository_ai_infrastructure_is_valid():
    assert infra.run_checks(ROOT) == []


def test_inactive_cpp_test_policies_name_tracked_sources_and_owners():
    for relative, policy in infra.APPROVED_INACTIVE_CPP_TESTS.items():
        assert (ROOT / relative).is_file()
        assert (ROOT / policy["owner"]).is_file()


def test_static_ikfast_test_has_an_explicit_inactive_policy():
    assert infra.APPROVED_INACTIVE_CPP_TESTS["tests/integration/test_IkFast.cpp"] == {
        "owner": "tests/integration/CMakeLists.txt",
        "command": "dart_add_test",
        "scopes": (
            "if:TARGET dart-utils-urdf",
            "if:BUILD_SHARED_LIBS",
        ),
        "cache": {"BUILD_SHARED_LIBS": "OFF"},
    }


def test_repository_check_works_without_python_utf8_mode():
    env = {
        **os.environ,
        "LC_ALL": "C",
        "PYTHONCOERCECLOCALE": "0",
        "PYTHONUTF8": "0",
    }
    result = subprocess.run(
        [sys.executable, "scripts/check_ai_infrastructure.py", "--check"],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stdout + result.stderr


def test_repository_sync_check_reads_utf8_without_python_utf8_mode():
    source = ROOT / ".claude" / "skills" / "dart-contribute" / "SKILL.md"
    assert "❌" in source.read_text(encoding="utf-8")
    env = {
        **os.environ,
        "LC_ALL": "C",
        "PYTHONCOERCECLOCALE": "0",
        "PYTHONUTF8": "0",
    }
    result = subprocess.run(
        [sys.executable, "scripts/sync_ai_commands.py", "--check"],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
    )
    output = result.stdout + result.stderr

    assert "UnicodeDecodeError" not in output
    assert result.returncode == 0, output


def test_release_scenarios_are_exercisable():
    assert infra.exercise_scenarios(ROOT) == []


def test_release_scenarios_reject_structural_only_ai_completion_gate():
    data = copy.deepcopy(_scenario_data())
    orientation = next(
        scenario for scenario in data["scenarios"] if scenario["id"] == "orientation"
    )
    orientation["full_gates"] = [
        "pixi run python scripts/check_ai_infrastructure.py --check"
    ]

    errors = infra.exercise_scenarios(ROOT, data, emit=False)

    assert any(
        "structural-only AI checker is not completion evidence" in error
        for error in errors
    )


def test_simulation_scenario_routes_text_first_and_visual_policy():
    scenarios = {scenario["id"]: scenario for scenario in _scenario_data()["scenarios"]}
    scenario = scenarios["simulation-verification"]

    assert len(scenarios) == 8
    assert scenario["expected_route"]["name"] == "dart-verify-sim"
    assert scenario["focused_gates"] == ["pixi run test"]
    assert scenario["full_gates"] == ["pixi run test-py", "pixi run test-all"]
    assert {
        "dart/simulation",
        "dart/dynamics",
        "dart/collision",
        "dart/constraint",
        "dart/gui",
        "dart/utils",
        "python",
        "examples",
        "tutorials",
        "matching tests",
        "temporary claim-tied evidence",
    } <= set(scenario["permitted_scopes"])
    assert scenario["evidence_policy"] == (
        "text-first-with-claim-tied-visual-or-documented-exception"
    )
    assert scenario["semantic_review_policy"] == (
        "native-image-inspection-or-image-capable-handoff-with-explicit-limitation"
    )


def test_model_upgrade_scenario_routes_release_audit_and_durable_context():
    scenarios = {scenario["id"]: scenario for scenario in _scenario_data()["scenarios"]}
    scenario = scenarios["model-upgrade"]

    assert scenario["expected_route"]["name"] == "dart-model-upgrade"
    assert scenario["specialist_agent"] == "dart_release_auditor"
    assert "docs/plans/dashboard.md" in scenario["owner_docs"]
    assert "docs/dev_tasks/README.md" in scenario["owner_docs"]
    assert scenario["focused_gates"] == ["pixi run check-ai-infra"]
    assert scenario["full_gates"] == ["pixi run test-ai-infra", "pixi run lint"]


def test_release_maintenance_scenario_runs_release_tests():
    scenarios = {scenario["id"]: scenario for scenario in _scenario_data()["scenarios"]}

    assert scenarios["release-maintenance"]["focused_gates"] == ["pixi run test"]


def _scenario_data():
    return json.loads((ROOT / "docs" / "ai" / "agent-scenarios.json").read_text())


def test_branch_profile_and_scenario_keys_match_shared_schema():
    profile = json.loads((ROOT / "docs" / "ai" / "branch-profile.json").read_text())
    scenarios = _scenario_data()

    assert set(profile) == infra.BRANCH_PROFILE_KEYS
    assert set(scenarios) == infra.SCENARIO_TOP_LEVEL_KEYS
    for scenario in scenarios["scenarios"]:
        assert infra.SCENARIO_KEYS.issubset(scenario)
        assert set(scenario) - infra.SCENARIO_KEYS <= infra.SCENARIO_OPTIONAL_KEYS
        assert set(scenario["expected_route"]) == infra.ROUTE_KEYS


@pytest.mark.parametrize(
    ("mutation", "expected"),
    [
        ("policy", "wrong evidence policy"),
        ("semantic_policy", "wrong semantic review policy"),
        ("prompt", "wrong claim-dependent prompt"),
        ("route", "must route to dart-verify-sim"),
        ("scope", "must cover simulation"),
        ("focused_gate", "focused correctness gate"),
        ("full_gate", "full correctness gates"),
    ],
)
def test_simulation_scenario_contract_mutations_are_rejected(mutation, expected):
    data = copy.deepcopy(_scenario_data())
    scenario = next(
        item for item in data["scenarios"] if item["id"] == "simulation-verification"
    )
    if mutation == "policy":
        scenario["evidence_policy"] = "screenshot-only"
    elif mutation == "semantic_policy":
        scenario["semantic_review_policy"] = "machine-check-only"
    elif mutation == "prompt":
        scenario["prompt_class"] = "take a screenshot"
    elif mutation == "route":
        scenario["expected_route"] = {
            "kind": "domain_skill",
            "name": "dart-test",
            "path": ".agents/skills/dart-test/SKILL.md",
        }
    elif mutation == "scope":
        scenario["permitted_scopes"].remove("dart/gui")
    elif mutation == "focused_gate":
        scenario["focused_gates"] = ["pixi run test-py"]
    else:
        scenario["full_gates"] = ["pixi run test-all"]

    errors = infra.exercise_scenarios(ROOT, data, emit=False)

    assert any(expected in error for error in errors)


@pytest.mark.parametrize(
    "marker",
    [
        "text correctness oracle",
        "agent-capture",
        "image-verdict",
        "sole correctness oracle",
        "bm-boxes-headless",
        "Xvfb",
        "--factory module:callable",
        "model/scene loading",
        "collision/contact/constraints",
        "simulation stepping",
        "OSG rendering",
        "visual example",
        "OSG capture is unavailable",
        "replacement evidence",
        "settled-contact",
        "text/geometry oracle",
        "test-agent-debug-overlay",
        "/tmp/dart-visual-evidence/capture_auto0.png",
        "semantic inspection",
        "native image viewer",
        "original detail",
        "do not average",
        "verification-bundle",
        "limiting horizontal or",
        "no-bounded-renderable",
    ],
)
def test_simulation_skill_contract_markers_are_required(monkeypatch, marker):
    source = ROOT / ".claude/skills/dart-verify-sim/SKILL.md"
    original = source.read_text()
    real_read_text = Path.read_text

    def read_text(path, *args, **kwargs):
        if path == source:
            return original.replace(marker, "missing-marker")
        return real_read_text(path, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", read_text)

    errors = infra.exercise_scenarios(ROOT, emit=False)

    assert any("missing contract marker" in error for error in errors)


@pytest.mark.parametrize(
    ("relative", "marker"),
    [
        ("AGENTS.md", "docs/ai/verification.md"),
        (".codex/agents/dart_scout.toml", "text correctness oracle"),
        (".codex/agents/dart_reviewer.toml", "text-first evidence"),
        (".claude/commands/dart-new-task.md", "route through `dart-verify-sim`"),
        (
            ".claude/commands/dart-new-task.md",
            "record why it is unavailable or not applicable",
        ),
        (".claude/commands/dart-ultrawork.md", "routes through `dart-verify-sim`"),
        (".claude/commands/dart-resume.md", "route through `dart-verify-sim`"),
        (".claude/commands/dart-pr.md", "use `dart-verify-sim`"),
        (".claude/commands/dart-manage-pr.md", "Visual verification"),
        (
            ".claude/commands/dart-review-pr.md",
            "require the `dart-verify-sim` text oracle",
        ),
        (".claude/commands/dart-review-pr.md", "justified replacement"),
        (".claude/skills/dart-build/SKILL.md", "dart-verify-sim"),
        (".claude/skills/dart-test/SKILL.md", "load `dart-verify-sim`"),
        (
            ".claude/skills/dart-test/SKILL.md",
            "capture is unavailable or not applicable",
        ),
        (".claude/skills/dart-io/SKILL.md", "also load `dart-verify-sim`"),
        (
            ".claude/skills/dart-io/SKILL.md",
            "claim-tied OSG visual corroboration",
        ),
        (".claude/skills/dart-python/SKILL.md", "load `dart-verify-sim`"),
        (
            ".claude/skills/dart-python/SKILL.md",
            "focused Python text/behavior oracle",
        ),
        (
            ".claude/skills/dart-python/SKILL.md",
            "collision/contact/constraints",
        ),
        (".claude/skills/dart-python/SKILL.md", "GUI/OSG output"),
        (".claude/skills/dart-python/SKILL.md", "visual exception"),
        (".claude/skills/dart-ci/SKILL.md", "also load `dart-verify-sim`"),
        (".claude/skills/dart-ci/SKILL.md", "visual exception"),
        (
            ".claude/commands/dart-downstream-fix.md",
            "route through `dart-verify-sim`",
        ),
        (".claude/commands/dart-downstream-fix.md", "visual exception"),
        (
            ".claude/commands/dart-backport-pr.md",
            "release branch's `dart-verify-sim`",
        ),
        (".claude/commands/dart-backport-pr.md", "visual exception"),
        (".claude/commands/dart-release-ci-fix.md", "use `dart-verify-sim`"),
        (".claude/commands/dart-release-ci-fix.md", "visual exception"),
        (".claude/commands/dart-fix-ci.md", "use `dart-verify-sim`"),
        (
            ".claude/commands/dart-fix-ci.md",
            "collision/contact/constraints",
        ),
        (".claude/commands/dart-fix-ci.md", "visual exception"),
        ("docs/ai/verification.md", "required renderer"),
        ("docs/ai/verification.md", "Name the replacement"),
    ],
)
def test_simulation_consumer_routes_are_required(monkeypatch, relative, marker):
    path = ROOT / relative
    original = path.read_text()
    real_read_text = Path.read_text

    def read_text(candidate, *args, **kwargs):
        if candidate == path:
            return original.replace(marker, "missing-marker", 1)
        return real_read_text(candidate, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", read_text)

    errors = infra.exercise_scenarios(ROOT, emit=False)

    assert any(
        relative in error and "simulation route marker" in error for error in errors
    )


def test_boolean_schema_versions_are_rejected():
    profile = json.loads((ROOT / "docs" / "ai" / "branch-profile.json").read_text())
    profile["schema_version"] = True
    profile_errors = []
    infra.check_branch_profile(ROOT, profile_errors, profile)
    scenarios = copy.deepcopy(_scenario_data())
    scenarios["schema_version"] = True

    scenario_errors = infra.exercise_scenarios(ROOT, scenarios, emit=False)

    assert any("schema_version" in error for error in profile_errors)
    assert scenario_errors == ["docs/ai/agent-scenarios.json: invalid schema"]


def test_branch_profile_marker_mutations_are_rejected():
    profile = json.loads((ROOT / "docs" / "ai" / "branch-profile.json").read_text())
    missing = copy.deepcopy(profile)
    missing["required_markers"].append("definitely missing release marker")
    forbidden = copy.deepcopy(profile)
    forbidden["forbidden_markers"].append("# Agent Guidelines for DART 6.20")
    missing_errors = []
    forbidden_errors = []

    infra.check_branch_profile(ROOT, missing_errors, missing)
    infra.check_branch_profile(ROOT, forbidden_errors, forbidden)

    assert any("required marker not visible" in error for error in missing_errors)
    assert any("forbidden marker is visible" in error for error in forbidden_errors)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("required_markers", 1),
        ("required_paths", [1]),
        ("forbidden_markers", [""]),
        ("forbidden_paths", [None]),
        ("downstream_gates", {"gate": "pixi run test"}),
    ],
)
def test_branch_profile_malformed_lists_return_errors(field, value):
    profile = json.loads((ROOT / "docs" / "ai" / "branch-profile.json").read_text())
    profile[field] = value
    errors = []

    infra.check_branch_profile(ROOT, errors, profile)

    assert any(
        f"`{field}` must be a non-empty string list" in error for error in errors
    )


@pytest.mark.parametrize("unsafe", ["/etc/passwd", "../outside", "docs/../AGENTS.md"])
def test_branch_profile_paths_cannot_escape_or_be_non_normalized(unsafe):
    profile = json.loads((ROOT / "docs" / "ai" / "branch-profile.json").read_text())
    profile["required_paths"] = [unsafe]
    errors = []

    infra.check_branch_profile(ROOT, errors, profile)

    assert any("invalid repository-relative required path" in error for error in errors)


def test_scoped_agent_instruction_files_are_required(tmp_path):
    for path in infra.required_paths(tmp_path):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("placeholder\n")
    scoped = tmp_path / ".agents" / "AGENTS.md"
    scoped.unlink()
    errors = []

    infra.check_required_files(tmp_path, errors)

    assert errors == ["missing required file: .agents/AGENTS.md"]


@pytest.mark.parametrize(
    ("field", "value", "expected"),
    [
        ("start_dir", "missing/start", "missing start_dir"),
        ("instruction_chain", [], "instruction_chain"),
        ("permitted_scopes", [], "permitted_scopes"),
        ("focused_gates", [], "focused_gates"),
        ("full_gates", [], "full_gates"),
        ("recovery", "docs/missing.md", "missing recovery"),
        ("forbidden_paths", ["AGENTS.md"], "forbidden_paths"),
    ],
)
def test_scenario_contract_mutations_are_rejected(field, value, expected):
    data = copy.deepcopy(_scenario_data())
    data["scenarios"][0][field] = value

    errors = infra.exercise_scenarios(ROOT, data, emit=False)

    assert any(expected in error for error in errors)


@pytest.mark.parametrize(
    ("field", "value", "expected"),
    [
        ("instruction_chain", [1], "instruction_chain"),
        ("owner_docs", [1], "owner_docs"),
        ("permitted_scopes", [1], "permitted_scopes"),
        ("focused_gates", [1], "focused_gates"),
        ("full_gates", [None], "full_gates"),
        ("forbidden_paths", [1], "forbidden_paths"),
    ],
)
def test_scenario_malformed_lists_return_errors(field, value, expected):
    data = copy.deepcopy(_scenario_data())
    data["scenarios"][0][field] = value

    errors = infra.exercise_scenarios(ROOT, data, emit=False)

    assert any(expected in error for error in errors)


@pytest.mark.parametrize(
    ("field", "unsafe", "expected"),
    [
        ("start_dir", "/", "start_dir"),
        ("instruction_chain", ["../AGENTS.md"], "instruction_chain"),
        ("owner_docs", ["/etc/passwd"], "owner doc"),
        ("recovery", "../outside.md", "recovery pointer"),
        ("forbidden_paths", ["docs/../main-only"], "forbidden path"),
    ],
)
def test_scenario_paths_cannot_escape_or_be_non_normalized(field, unsafe, expected):
    data = copy.deepcopy(_scenario_data())
    data["scenarios"][0][field] = unsafe

    errors = infra.exercise_scenarios(ROOT, data, emit=False)

    assert any(expected in error for error in errors)


def test_scenario_route_path_cannot_escape_repository():
    data = copy.deepcopy(_scenario_data())
    data["scenarios"][0]["expected_route"]["path"] = "/etc/passwd"

    errors = infra.exercise_scenarios(ROOT, data, emit=False)

    assert any("repository-relative route path" in error for error in errors)


def test_unknown_scenario_route_is_rejected():
    data = copy.deepcopy(_scenario_data())
    data["scenarios"][1]["expected_route"]["name"] = "dart-main-only"

    errors = infra.exercise_scenarios(ROOT, data, emit=False)

    assert any("unknown workflow route" in error for error in errors)


def test_scenario_extra_or_nested_agent_keys_are_rejected():
    extra = copy.deepcopy(_scenario_data())
    extra["scenarios"][0]["extra"] = True
    nested = copy.deepcopy(_scenario_data())
    nested["scenarios"][2]["expected_route"]["agent"] = "dart_scout"

    extra_errors = infra.exercise_scenarios(ROOT, extra, emit=False)
    nested_errors = infra.exercise_scenarios(ROOT, nested, emit=False)

    assert any("keys must contain" in error for error in extra_errors)
    assert any("expected_route keys must be" in error for error in nested_errors)


@pytest.mark.parametrize(
    ("command", "expected"),
    [
        ("pixi run lint && definitely-not-a-valid-gate", "invalid gate command"),
        (
            "pixi run python scripts/definitely_missing.py --check",
            "references missing file",
        ),
        (
            "pixi run python scripts/../AGENTS.md",
            "path escapes repository",
        ),
        (
            "pixi run python -m pytest /etc/passwd -q",
            "path escapes repository",
        ),
        ("pixi run lint unexpected-argument", "unexpected arguments"),
    ],
)
def test_scenario_gate_trailing_or_missing_commands_are_rejected(command, expected):
    data = copy.deepcopy(_scenario_data())
    data["scenarios"][0]["focused_gates"] = [command]

    errors = infra.exercise_scenarios(ROOT, data, emit=False)

    assert any(expected in error for error in errors)


def test_task_collection_includes_default_and_feature_tasks():
    data = {
        "tasks": {"default-task": {}},
        "feature": {"gazebo": {"tasks": {"feature-task": {}}}},
    }
    assert infra.collect_task_names(data) == {"default-task", "feature-task"}


def test_task_command_collection_includes_platform_variants():
    data = {
        "tasks": {"test": {"cmd": "ctest unix"}},
        "target": {"win-64": {"tasks": {"test": {"cmd": ["ctest", "windows"]}}}},
    }
    assert infra.collect_task_commands(data, "test") == [
        "ctest unix",
        "ctest windows",
    ]


def test_task_definition_collection_includes_platform_variants():
    data = {
        "tasks": {"test-all": {"cmd": "build unix", "env": {"BUILD_TYPE": "Release"}}},
        "target": {
            "win-64": {
                "tasks": {
                    "test-all": {
                        "cmd": "build windows",
                        "env": {"BUILD_TYPE": "Release"},
                    }
                }
            }
        },
    }

    assert [
        task["cmd"] for task in infra.collect_task_definitions(data, "test-all")
    ] == [
        "build unix",
        "build windows",
    ]


@pytest.mark.parametrize(
    ("command", "expected"),
    [
        ("cmake --build build/default --target ALL", True),
        (
            "cmake --build build/default --target test && "
            "cmake --build build/default --target ALL",
            False,
        ),
        ("cmake --build build/default --target test --target ALL", False),
        ("cmake --build build/default --target clean --target ALL", False),
        ('cmake --build "$PIXI_PROJECT_ROOT/build/default" --target ALL', True),
        ("cmake --build $PIXI_PROJECT_ROOT/build/default --target ALL", False),
        (
            "cmake --build build/default\nruntime-wrapper --target ALL",
            False,
        ),
        (
            'cmake --build "$(runtime-wrapper)" --target ALL',
            False,
        ),
        (
            "cmake --build `runtime-wrapper` --target ALL",
            False,
        ),
    ],
)
def test_single_cmake_all_build_contract(command, expected):
    assert infra.is_single_cmake_all_build(command) is expected


@pytest.mark.parametrize(
    ("command", "target", "expected"),
    [
        (
            'cmake --build "build/default" -j --target tests_and_run',
            "tests_and_run",
            True,
        ),
        (
            'cmake --build "build/default" --config "$BUILD_TYPE" '
            "-j --target tests_and_run",
            "tests_and_run",
            True,
        ),
        (
            'cmake --build "build/default" --target tests_and_run && ctest',
            "tests_and_run",
            False,
        ),
        (
            'cmake --build "build/default" --target tests_and_run ALL',
            "tests_and_run",
            False,
        ),
    ],
)
def test_single_cmake_target_build_contract(command, target, expected):
    assert infra.is_single_cmake_target_build(command, target) is expected


@pytest.mark.parametrize(
    ("command", "expected"),
    [
        ('cmake -S . -B "$PIXI_PROJECT_ROOT/build/default"', True),
        ("cmake -S . -B $PIXI_PROJECT_ROOT/build/default", False),
        (
            "cmake -G Ninja -S . -B build/default -DCMAKE_BUILD_TYPE=Release",
            True,
        ),
        ("cmake -S . -B build/default\nninja test", False),
        ('cmake -S "$(python -m unittest)" -B build/default', False),
        (
            'cmake -E env sh -c "ninja -C build/default test" ' "-S . -B build/default",
            False,
        ),
        (
            "cmake -S . -B build/default --install build/default",
            False,
        ),
        (
            "cmake -S . -B build/default -C runtime-cache.cmake",
            False,
        ),
        ("cmake -S . -B build/default -N", False),
        (
            "cmake -S . -B build/default "
            "-DCMAKE_PROJECT_TOP_LEVEL_INCLUDES=runtime.cmake",
            False,
        ),
        (
            'cmake -S . -B build/default -DCMAKE_PREFIX_PATH="x${IFS}-C"',
            False,
        ),
        (
            'cmake -S . -B build/default -DCMAKE_PREFIX_PATH="$UNKNOWN_PATH"',
            False,
        ),
        ("cmake -S runtime-project -B build/default", False),
    ],
)
def test_configuration_only_command_contract(command, expected):
    assert infra.is_configuration_only_command(command) is expected


def test_pixi_reference_check_includes_durable_resume_surfaces(tmp_path):
    (tmp_path / "pixi.toml").write_text(
        '[tasks]\nlint = { cmd = "true" }\n', encoding="utf-8"
    )
    resume = tmp_path / "docs/dev_tasks/example/RESUME.md"
    resume.parent.mkdir(parents=True)
    resume.write_text("Run `pixi run test-unit` before resuming.\n", encoding="utf-8")
    errors = []

    infra.check_pixi_references(tmp_path, errors)

    assert errors == [
        "docs/dev_tasks/example/RESUME.md:1: unknown Pixi task `test-unit`"
    ]


def test_pixi_reference_check_includes_numbered_task_context(tmp_path):
    (tmp_path / "pixi.toml").write_text(
        '[tasks]\nlint = { cmd = "true" }\n', encoding="utf-8"
    )
    packet = tmp_path / "docs/dev_tasks/example/07-work-packet.md"
    packet.parent.mkdir(parents=True)
    packet.write_text("Run `pixi run test-unit` before resuming.\n", encoding="utf-8")
    errors = []

    infra.check_pixi_references(tmp_path, errors)

    assert errors == [
        "docs/dev_tasks/example/07-work-packet.md:1: unknown Pixi task `test-unit`"
    ]


def _copy_test_gate_contract(root: Path) -> None:
    for relative in (
        "pixi.toml",
        "AGENTS.md",
        "CMakeLists.txt",
        "docs/onboarding/testing.md",
        "docs/onboarding/release-management.md",
        "tests/CMakeLists.txt",
        "python/CMakeLists.txt",
        "python/tests/CMakeLists.txt",
        "cmake/DARTRunCTest.cmake",
    ):
        target = root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(
            (ROOT / relative).read_text(encoding="utf-8"), encoding="utf-8"
        )


def _replace_required_pytest_command(cmake: Path, replacement: str) -> None:
    marker = (
        "    COMMAND\n"
        '      "${Python3_EXECUTABLE}" -I '
        '"${PROJECT_SOURCE_DIR}/scripts/run_pytest.py"\n'
        '      --pythonpath "${DART_PYTHONPATH}" ${dartpy_test_files} -v\n'
    )
    text = cmake.read_text(encoding="utf-8")
    assert marker in text
    cmake.write_text(text.replace(marker, replacement, 1), encoding="utf-8")


@pytest.mark.parametrize(
    ("relative", "marker"),
    [
        ("AGENTS.md", "pixi run test         # Build and run C++ tests"),
        (
            "docs/onboarding/testing.md",
            "CMake's File API",
        ),
        (
            "docs/onboarding/release-management.md",
            "`test-all` does not format or check lint",
        ),
    ],
)
def test_test_gate_contract_rejects_operational_doc_drift(tmp_path, relative, marker):
    _copy_test_gate_contract(tmp_path)
    path = tmp_path / relative
    path.write_text(
        path.read_text(encoding="utf-8").replace(marker, "stale gate claim", 1),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(relative in error and marker in error for error in errors)


@pytest.mark.parametrize(
    "relative",
    (".codex/AGENTS.md", "docs/ai/README.md", "docs/ai/verification.md"),
)
def test_test_gate_contract_rejects_structural_only_completion_gate(tmp_path, relative):
    _copy_test_gate_contract(tmp_path)
    path = tmp_path / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        (ROOT / relative)
        .read_text(encoding="utf-8")
        .replace(
            "pixi run check-ai-infra",
            "pixi run python scripts/check_ai_infrastructure.py --check",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        f"{relative}: structural-only AI checker is presented as a completion "
        "gate" in error
        for error in errors
    )


def test_test_gate_contract_rejects_task_semantic_drift(tmp_path):
    _copy_test_gate_contract(tmp_path)
    pixi = tmp_path / "pixi.toml"
    pixi.write_text(
        pixi.read_text(encoding="utf-8").replace("--target ALL", "--target build"),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert errors == [
        "pixi.toml: `test-all` task must retain command marker `--target ALL`"
    ]


@pytest.mark.parametrize(
    ("old", "new", "expected"),
    [
        (
            "--target ALL",
            "--target ALL && ctest",
            "`test-all` runtime coverage must remain owned by the CMake `ALL` graph",
        ),
        (
            "--target ALL",
            "--target ALL test",
            "every `test-all` command must be one CMake build",
        ),
        (
            'depends-on = ["config"]',
            'depends-on = ["config", "test"]',
            "every `test-all` variant must depend only on",
        ),
        (
            'env = { BUILD_TYPE = "Release" }',
            'env = { BUILD_TYPE = "Debug" }',
            'every `test-all` variant must set only `BUILD_TYPE = "Release"`',
        ),
    ],
)
def test_test_gate_contract_rejects_test_all_runtime_semantics(
    tmp_path, old, new, expected
):
    _copy_test_gate_contract(tmp_path)
    pixi = tmp_path / "pixi.toml"
    text = pixi.read_text(encoding="utf-8")
    test_all_offset = text.index("test-all =")
    text = text[:test_all_offset] + text[test_all_offset:].replace(old, new, 1)
    pixi.write_text(text, encoding="utf-8")
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(expected in error for error in errors)


@pytest.mark.parametrize(
    ("old", "new", "expected"),
    [
        (
            "--target tests_and_run",
            "--target tests",
            "`test` task must retain command marker `--target tests_and_run`",
        ),
        (
            "--target tests_and_run",
            "--target tests_and_run && ctest",
            "every `test` command must be one CMake build",
        ),
        (
            'depends-on = ["config"]',
            'depends-on = ["build-tests"]',
            "every `test` variant must depend only on",
        ),
        (
            'env = { BUILD_TYPE = "Release" }',
            'env = { BUILD_TYPE = "Debug" }',
            'every `test` variant must set only `BUILD_TYPE = "Release"`',
        ),
    ],
)
def test_test_gate_contract_rejects_focused_test_runtime_semantics(
    tmp_path, old, new, expected
):
    _copy_test_gate_contract(tmp_path)
    pixi = tmp_path / "pixi.toml"
    text = pixi.read_text(encoding="utf-8")
    test_offset = text.index("\ntest = {")
    text = text[:test_offset] + text[test_offset:].replace(old, new, 1)
    pixi.write_text(text, encoding="utf-8")
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(expected in error for error in errors)


@pytest.mark.parametrize("task", ("test-ai-infra", "test-agent-debug-overlay"))
def test_test_gate_contract_rejects_raw_canonical_pytest_tasks(tmp_path, task):
    _copy_test_gate_contract(tmp_path)
    pixi = tmp_path / "pixi.toml"
    text = pixi.read_text(encoding="utf-8")
    task_offset = text.index(f"{task} =")
    text = text[:task_offset] + text[task_offset:].replace(
        '"scripts/run_pytest.py"',
        '"-m", "pytest"',
        1,
    )
    pixi.write_text(text, encoding="utf-8")
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        f"pixi.toml: `{task}` must use the guarded repository pytest runner" in error
        for error in errors
    )


def test_test_gate_contract_rejects_dependency_indirection(tmp_path):
    _copy_test_gate_contract(tmp_path)
    pixi = tmp_path / "pixi.toml"
    text = pixi.read_text(encoding="utf-8")
    test_all_offset = text.index("test-all =")
    text = text[:test_all_offset] + text[test_all_offset:].replace(
        'depends-on = ["config"]',
        'depends-on = ["runtime-wrapper"]',
        1,
    )
    text += '\nruntime-wrapper = { cmd = "ctest", depends-on = ["test"] }\n'
    pixi.write_text(text, encoding="utf-8")
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        "every `test-all` variant must depend only on" in error for error in errors
    )


@pytest.mark.parametrize(
    ("old", "new", "expected"),
    [
        (
            "    cmake \\\n        -G Ninja",
            "    ctest && cmake \\\n        -G Ninja",
            "every `config` command used by `test-all` must only configure CMake",
        ),
        (
            '""", env = { DART_VERBOSE = "OFF", BUILD_TYPE = "Release" } }',
            '""", depends-on = ["runtime-wrapper"], '
            'env = { DART_VERBOSE = "OFF", BUILD_TYPE = "Release" } }',
            "`config` must not depend on other tasks",
        ),
        (
            'env = { DART_VERBOSE = "OFF", BUILD_TYPE = "Release" }',
            'env = { DART_VERBOSE = "OFF -C runtime.cmake", '
            'BUILD_TYPE = "Release" }',
            "every `config` variant must use the exact branch-owned environment",
        ),
    ],
)
def test_test_gate_contract_rejects_runtime_config_path(tmp_path, old, new, expected):
    _copy_test_gate_contract(tmp_path)
    pixi = tmp_path / "pixi.toml"
    text = pixi.read_text(encoding="utf-8")
    assert old in text
    pixi.write_text(text.replace(old, new, 1), encoding="utf-8")
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(expected in error for error in errors)


@pytest.mark.parametrize(
    ("old", "new", "expected"),
    [
        (
            "--check --semantic-cmake",
            "--check",
            "must run the semantic CMake infrastructure check exactly",
        ),
        (
            '"config",\n], env = { BUILD_TYPE = "Release" } }\n'
            "exercise-agent-scenarios",
            '"config", "test",\n], env = { BUILD_TYPE = "Release" } }\n'
            "exercise-agent-scenarios",
            "must depend only on `config`",
        ),
        (
            '], env = { BUILD_TYPE = "Release" } }\nexercise-agent-scenarios',
            '], env = { BUILD_TYPE = "Debug" } }\nexercise-agent-scenarios',
            "must select only the Release configuration",
        ),
    ],
)
def test_test_gate_contract_rejects_ai_check_semantic_drift(
    tmp_path, old, new, expected
):
    _copy_test_gate_contract(tmp_path)
    pixi = tmp_path / "pixi.toml"
    text = pixi.read_text(encoding="utf-8")
    check_offset = text.index("check-ai-infra =")
    prefix = text[:check_offset]
    check_and_after = text[check_offset:]
    assert old in check_and_after
    pixi.write_text(prefix + check_and_after.replace(old, new, 1), encoding="utf-8")
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(expected in error for error in errors)


@pytest.mark.parametrize(
    "flag",
    (
        "-DBUILD_TESTING=ON",
        "-DDART_BUILD_DARTPY=ON",
        "-DDART_USE_SYSTEM_PYBIND11=ON",
    ),
)
def test_test_gate_contract_requires_runtime_config_flags(tmp_path, flag):
    _copy_test_gate_contract(tmp_path)
    pixi = tmp_path / "pixi.toml"
    pixi.write_text(
        pixi.read_text(encoding="utf-8").replace(
            f"        {flag} \\\n",
            "",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        f"must pin `{flag.removeprefix('-D')}` exactly once" in error
        for error in errors
    )


@pytest.mark.parametrize(
    "flag",
    (
        "-DBUILD_TESTING=ON",
        "-DDART_BUILD_DARTPY=ON",
        "-DDART_USE_SYSTEM_PYBIND11=ON",
    ),
)
def test_test_gate_contract_rejects_conflicting_runtime_config_flags(tmp_path, flag):
    _copy_test_gate_contract(tmp_path)
    variable = flag.removeprefix("-D").split("=", maxsplit=1)[0]
    pixi = tmp_path / "pixi.toml"
    pixi.write_text(
        pixi.read_text(encoding="utf-8").replace(
            f"        {flag} \\\n",
            f"        {flag} \\\n        -D{variable}=OFF \\\n",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        f"must pin `{flag.removeprefix('-D')}` exactly once" in error
        for error in errors
    )


def test_test_gate_contract_requires_runtime_graph(tmp_path):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / "CMakeLists.txt"
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "list(APPEND all_target_candidates tests_and_run pytest)",
            "list(APPEND all_target_candidates tests)",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        "CMakeLists.txt: missing `test-all` graph marker" in error for error in errors
    )


def test_test_gate_contract_rejects_commented_runtime_graph(tmp_path):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / "CMakeLists.txt"
    marker = "list(APPEND all_target_candidates tests_and_run pytest)"
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(marker, f"# {marker}", 1),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        f"CMakeLists.txt: missing `test-all` graph marker `{marker}`" in error
        for error in errors
    )


@pytest.mark.parametrize(
    "replacement",
    (
        'message("list(APPEND all_target_candidates tests_and_run pytest)")',
        (
            "message(\n"
            "  list(APPEND all_target_candidates tests_and_run pytest)\n"
            ")"
        ),
        (
            "if(FALSE)\n"
            "  list(APPEND all_target_candidates tests_and_run pytest)\n"
            "endif()"
        ),
        (
            "if(\n"
            "  FALSE\n"
            ")\n"
            "  list(APPEND all_target_candidates tests_and_run pytest)\n"
            "endif()"
        ),
    ),
)
def test_test_gate_contract_rejects_inactive_runtime_graph(tmp_path, replacement):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / "CMakeLists.txt"
    marker = "list(APPEND all_target_candidates tests_and_run pytest)"
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(marker, replacement, 1),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        f"CMakeLists.txt: missing `test-all` graph marker `{marker}`" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("relative", "condition", "expected_marker"),
    (
        (
            "python/tests/CMakeLists.txt",
            "DARTPY_PYTEST_FOUND",
            "scripts/run_pytest.py",
        ),
    ),
)
def test_test_gate_contract_rejects_inactive_runtime_targets(
    tmp_path, relative, condition, expected_marker
):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / relative
    text = cmake.read_text(encoding="utf-8")
    condition_offset = text.index(f"if({condition})")
    text = text[:condition_offset] + text[condition_offset:].replace(
        f"if({condition})",
        f"if({condition})\n  if(FALSE)",
        1,
    )
    else_offset = text.index("else()", condition_offset)
    text = text[:else_offset] + "  endif()\n" + text[else_offset:]
    cmake.write_text(text, encoding="utf-8")
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        f"{relative}: missing `test-all` graph marker `{expected_marker}`" in error
        for error in errors
    )


def test_test_gate_contract_rejects_ctest_runner_drift(tmp_path):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / "tests/CMakeLists.txt"
    marker = "${PROJECT_SOURCE_DIR}/cmake/DARTRunCTest.cmake"
    text = cmake.read_text(encoding="utf-8")
    assert marker in text
    cmake.write_text(
        text.replace(marker, "${PROJECT_SOURCE_DIR}/cmake/Noop.cmake", 1),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        "tests/CMakeLists.txt: missing `test-all` graph marker "
        "`cmake/DARTRunCTest.cmake`" in error
        for error in errors
    )


@pytest.mark.parametrize(
    "injection",
    (
        "set(_dart_gtest_unsets)\n",
        "set(_dart_ctest_arguments --show-only)\n",
        "cmake_language(EXIT 0)\n",
    ),
)
def test_test_gate_contract_rejects_ctest_runner_dataflow_bypass(tmp_path, injection):
    _copy_test_gate_contract(tmp_path)
    runner = tmp_path / "cmake" / "DARTRunCTest.cmake"
    text = runner.read_text(encoding="utf-8")
    final_call = text.rfind("execute_process(")
    assert final_call > 0
    runner.write_text(
        text[:final_call] + injection + text[final_call:],
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        "cmake/DARTRunCTest.cmake: commands and lexical control flow must "
        "exactly match" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("opener", "closer"),
    (
        ("if(FALSE)\n", "endif()\n"),
        ("function(unused_ctest_runner)\n", "endfunction()\n"),
    ),
)
def test_test_gate_contract_rejects_unreachable_ctest_execution(
    tmp_path, opener, closer
):
    _copy_test_gate_contract(tmp_path)
    runner = tmp_path / "cmake" / "DARTRunCTest.cmake"
    text = runner.read_text(encoding="utf-8")
    final_call = text.rfind("execute_process(")
    assert final_call > 0
    runner.write_text(
        text[:final_call] + opener + text[final_call:] + closer,
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        "cmake/DARTRunCTest.cmake: commands and lexical control flow must "
        "exactly match" in error
        for error in errors
    )


def test_test_gate_contract_rejects_ctest_command_shadowing(tmp_path):
    _copy_test_gate_contract(tmp_path)
    runner = tmp_path / "cmake" / "DARTRunCTest.cmake"
    runner.write_text(
        "macro(execute_process)\n"
        "  set(_dart_environment_result 0)\n"
        "endmacro()\n" + runner.read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        "cmake/DARTRunCTest.cmake: commands and lexical control flow must "
        "exactly match" in error
        for error in errors
    )


def test_test_gate_contract_rejects_quoted_pytest_target_spoof(tmp_path):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / "python/tests/CMakeLists.txt"
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "add_custom_target(\n    pytest",
            'add_custom_target(\n    fake\n    "pytest COMMAND"',
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        "python/tests/CMakeLists.txt: missing `test-all` graph marker "
        "`scripts/run_pytest.py`" in error
        for error in errors
    )


def test_test_gate_contract_rejects_quoted_pytest_command_spoof(tmp_path):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / "python/tests/CMakeLists.txt"
    _replace_required_pytest_command(
        cmake,
        (
            "    COMMAND ${CMAKE_COMMAND} -E echo\n"
            '      "COMMAND ${CMAKE_COMMAND} -E env '
            "PYTHONPATH=${DART_PYTHONPATH} "
            "${Python3_EXECUTABLE} "
            "${PROJECT_SOURCE_DIR}/scripts/run_pytest.py "
            '${dartpy_test_files} -v"\n'
        ),
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        "python/tests/CMakeLists.txt: missing `test-all` graph marker "
        "`scripts/run_pytest.py`" in error
        for error in errors
    )


def test_test_gate_contract_rejects_bracket_pytest_command_spoof(tmp_path):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / "python/tests/CMakeLists.txt"
    _replace_required_pytest_command(
        cmake,
        (
            "    [=[COMMAND ${CMAKE_COMMAND} -E env "
            "PYTHONPATH=${DART_PYTHONPATH} "
            "${Python3_EXECUTABLE} "
            "${PROJECT_SOURCE_DIR}/scripts/run_pytest.py "
            "${dartpy_test_files} -v]=]\n"
        ),
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        "python/tests/CMakeLists.txt: missing `test-all` graph marker "
        "`scripts/run_pytest.py`" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("relative", "needle", "occurrence"),
    (
        (
            "CMakeLists.txt",
            "list(APPEND all_target_candidates tests_and_run pytest)",
            1,
        ),
        (
            "CMakeLists.txt",
            "add_custom_target(ALL DEPENDS ${all_targets})",
            1,
        ),
        (
            "tests/CMakeLists.txt",
            "add_custom_target(\n  tests_and_run",
            1,
        ),
        (
            "python/tests/CMakeLists.txt",
            "add_custom_target(\n    pytest",
            1,
        ),
        (
            "python/tests/CMakeLists.txt",
            "add_custom_target(\n    pytest",
            2,
        ),
    ),
)
def test_test_gate_contract_rejects_unreachable_runtime_graph(
    tmp_path, relative, needle, occurrence
):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / relative
    text = cmake.read_text(encoding="utf-8")
    offset = -1
    for _ in range(occurrence):
        offset = text.index(needle, offset + 1)
    cmake.write_text(
        text[:offset] + "return()\n  " + text[offset:],
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        f"{relative}: `return()` may bypass the required `test-all` graph" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("value", "separator", "expected"),
    (
        (
            "$<TARGET_FILE_DIR:dartpy>:/tmp/build/python",
            ":",
            ["$<TARGET_FILE_DIR:dartpy>", "/tmp/build/python"],
        ),
        (
            "$<TARGET_FILE_DIR:dartpy>;C:/build/python",
            ";",
            ["$<TARGET_FILE_DIR:dartpy>", "C:/build/python"],
        ),
        (
            r"$<TARGET_FILE_DIR:dartpy>\;C:/build/python",
            ";",
            ["$<TARGET_FILE_DIR:dartpy>", "C:/build/python"],
        ),
        (
            "$<IF:$<BOOL:1>,/tmp/a,/tmp/b>:/tmp/build/python",
            ":",
            ["$<IF:$<BOOL:1>,/tmp/a,/tmp/b>", "/tmp/build/python"],
        ),
        ("$<TARGET_FILE_DIR:dartpy", ":", []),
    ),
)
def test_split_cmake_path_list_preserves_generator_expressions(
    value, separator, expected
):
    assert infra.split_cmake_path_list(value, separator) == expected


def _configure_semantic_graph_fixture(
    root: Path,
    generator: str | None = None,
    force_windows: bool = False,
) -> Path:
    sources = {
        "CMakeLists.txt": """
cmake_minimum_required(VERSION 3.22)
project(test_graph C CXX)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
enable_testing()
set(BUILD_TESTING ON CACHE BOOL "")
set(DART_BUILD_DARTPY ON CACHE BOOL "")
set(DART_USE_SYSTEM_PYBIND11 ON CACHE BOOL "")
add_library(dartpy MODULE python/dartpy.c)
if(WIN32)
  set(
    DART_PYTHONPATH
    "$<TARGET_FILE_DIR:dartpy>\\;${CMAKE_BINARY_DIR}/python"
  )
else()
  set(DART_PYTHONPATH "$<TARGET_FILE_DIR:dartpy>:${CMAKE_BINARY_DIR}/python")
endif()
add_subdirectory(tests)
add_subdirectory(python/tests)
set(all_target_candidates dartpy)
if(BUILD_TESTING)
  list(APPEND all_target_candidates tests_and_run pytest)
endif()
foreach(target_candidate ${all_target_candidates})
  if(TARGET ${target_candidate})
    list(APPEND all_targets ${target_candidate})
  endif()
endforeach()
add_custom_target(ALL DEPENDS ${all_targets})
""".lstrip(),
        "tests/CMakeLists.txt": """
add_subdirectory(integration)
add_subdirectory(regression)
add_subdirectory(unit)
add_custom_target(
  tests_and_run
  COMMAND
    ${CMAKE_COMMAND}
    "-DDART_CTEST_COMMAND=${CMAKE_CTEST_COMMAND}"
    "-DDART_CTEST_CONFIGURATION=$<CONFIG>"
    -P "${PROJECT_SOURCE_DIR}/cmake/DARTRunCTest.cmake"
  DEPENDS ${integration_tests} ${regression_tests} ${unit_tests}
)
""".lstrip(),
        "tests/integration/CMakeLists.txt": """
add_executable(INTEGRATION_semantic test_semantic.cpp)
add_test(NAME INTEGRATION_semantic COMMAND INTEGRATION_semantic)
set(integration_tests INTEGRATION_semantic PARENT_SCOPE)
""".lstrip(),
        "tests/regression/CMakeLists.txt": """
add_executable(test_semantic_regression test_semantic.cpp)
add_test(NAME test_semantic_regression COMMAND test_semantic_regression)
set(regression_tests test_semantic_regression PARENT_SCOPE)
""".lstrip(),
        "tests/unit/CMakeLists.txt": """
add_executable(UNIT_semantic test_semantic.cpp)
add_test(NAME UNIT_semantic COMMAND UNIT_semantic)
set(unit_tests UNIT_semantic PARENT_SCOPE)
""".lstrip(),
        "python/dartpy.c": "int dartpy_semantic_fixture(void) { return 0; }\n",
        "python/tests/CMakeLists.txt": """
set(DARTPY_PYTEST_FOUND TRUE)
set(dartpy_test_files test_semantic.py)
set(dartpy_test_utils)
if(DARTPY_PYTEST_FOUND)
  add_custom_target(
    pytest
    COMMAND
      ${CMAKE_COMMAND} -E echo
      "Running pytest by: ${Python3_EXECUTABLE} -I ${PROJECT_SOURCE_DIR}/scripts/run_pytest.py --pythonpath ${DART_PYTHONPATH} [sources]"
    COMMAND
      "${Python3_EXECUTABLE}" -I "${PROJECT_SOURCE_DIR}/scripts/run_pytest.py"
      --pythonpath "${DART_PYTHONPATH}" ${dartpy_test_files} -v
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    SOURCES
      ${dartpy_test_files}
      ${dartpy_test_utils}
      "${PROJECT_SOURCE_DIR}/scripts/run_pytest.py"
    DEPENDS dartpy
  )
else()
  add_custom_target(
    pytest
    COMMAND
      ${CMAKE_COMMAND} -E echo
      "Warning: Failed to run pytest because pytest is not found!"
  )
endif()
""".lstrip(),
        "python/tests/test_semantic.py": "def test_semantic():\n    pass\n",
        "cmake/DARTRunCTest.cmake": (ROOT / "cmake/DARTRunCTest.cmake").read_text(
            encoding="utf-8"
        ),
        "scripts/run_pytest.py": (ROOT / "scripts/run_pytest.py").read_text(
            encoding="utf-8"
        ),
        "pyproject.toml": """
[tool.pytest.ini_options]
minversion = "6.0"
addopts = ["-ra", "--showlocals", "--strict-markers", "--strict-config"]
xfail_strict = true
filterwarnings = ["error"]
testpaths = ["tests"]
""".lstrip(),
    }
    for relative, text in sources.items():
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
    for relative in (
        "tests/integration/test_semantic.cpp",
        "tests/regression/test_semantic.cpp",
        "tests/unit/test_semantic.cpp",
    ):
        path = root / relative
        path.write_text("int main(void) { return 0; }\n", encoding="utf-8")

    build = root / "build"
    command = [
        "cmake",
        "-S",
        str(root),
        "-B",
        str(build),
        "-DCMAKE_BUILD_TYPE=Release",
    ]
    if generator is not None:
        command[1:1] = ["-G", generator]
    if force_windows:
        command.append("-DWIN32=TRUE")
    result = subprocess.run(
        command,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return build


def test_cmake_semantic_graph_probe_accepts_effective_runtime_graph(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors == []


def test_cmake_semantic_graph_probe_accepts_release_multi_config_graph(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path, "Ninja Multi-Config")
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors == []


def test_cmake_semantic_graph_probe_accepts_escaped_windows_path(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path, force_windows=True)
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors == []


def test_cmake_semantic_graph_probe_accepts_legacy_multi_config_tests(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path, "Ninja Multi-Config")
    replacements = {
        "tests/integration/CMakeLists.txt": (
            "add_test(NAME INTEGRATION_semantic COMMAND INTEGRATION_semantic)",
            "add_test(INTEGRATION_semantic INTEGRATION_semantic)",
        ),
        "tests/regression/CMakeLists.txt": (
            "add_test(NAME test_semantic_regression COMMAND test_semantic_regression)",
            "add_test(test_semantic_regression test_semantic_regression)",
        ),
        "tests/unit/CMakeLists.txt": (
            "add_test(NAME UNIT_semantic COMMAND UNIT_semantic)",
            "add_test(UNIT_semantic UNIT_semantic)",
        ),
    }
    for relative, (old, new) in replacements.items():
        path = tmp_path / relative
        path.write_text(
            path.read_text(encoding="utf-8").replace(old, new, 1),
            encoding="utf-8",
        )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors == []


def test_cmake_semantic_graph_probe_rejects_unowned_legacy_registration(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path, "Ninja Multi-Config")
    cmake = tmp_path / "tests/regression/CMakeLists.txt"
    cmake.write_text(
        cmake.read_text(encoding="utf-8") + "\nadd_test(ghost_test ghost_test)\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "CTest commands do not map to configured C++ test targets" in error
        and "ghost_test" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_rejects_external_build_directory(tmp_path):
    root = tmp_path / "repo"
    root.mkdir()
    errors = []

    infra.check_cmake_test_graph(root, tmp_path / "outside", errors)

    assert errors == [
        "CMake semantic test graph: build directory must be inside "
        f"`{root / 'build'}`"
    ]


def test_cmake_build_discovery_accepts_cmake_slash_spelling(tmp_path, monkeypatch):
    build = tmp_path / "build" / "default" / "cpp" / "Release"
    build.mkdir(parents=True)
    cache_home = str(tmp_path.resolve()).replace("/", "\\")
    (build / "CMakeCache.txt").write_text(
        f"CMAKE_HOME_DIRECTORY:INTERNAL={cache_home}\n",
        encoding="utf-8",
    )
    monkeypatch.setenv("PIXI_ENVIRONMENT_NAME", "default")
    monkeypatch.setenv("BUILD_TYPE", "Release")
    monkeypatch.setattr(infra.sys, "platform", "linux")

    assert infra.discover_cmake_build_dir(tmp_path) == build


def test_cmake_build_discovery_uses_windows_multi_config_owner(tmp_path, monkeypatch):
    base = tmp_path / "build" / "default" / "cpp"
    nested = base / "Release"
    nested.mkdir(parents=True)
    cache = f"CMAKE_HOME_DIRECTORY:INTERNAL={tmp_path.resolve()}\n"
    (base / "CMakeCache.txt").write_text(cache, encoding="utf-8")
    (nested / "CMakeCache.txt").write_text(cache, encoding="utf-8")
    monkeypatch.setenv("PIXI_ENVIRONMENT_NAME", "default")
    monkeypatch.setenv("BUILD_TYPE", "Release")
    monkeypatch.setattr(infra.sys, "platform", "win32")

    assert infra.discover_cmake_build_dir(tmp_path) == base


@pytest.mark.parametrize(
    ("compiler_id", "simulate_id", "expected"),
    (("MSVC", "", True), ("Clang", "MSVC", True), ("GNU", "", False)),
)
def test_cmake_msvc_semantics_follow_configured_compiler_state(
    tmp_path, compiler_id, simulate_id, expected
):
    compiler = tmp_path / "CMakeFiles" / "4.3.3" / "CMakeCXXCompiler.cmake"
    compiler.parent.mkdir(parents=True)
    compiler.write_text(
        f'set(CMAKE_CXX_COMPILER_ID "{compiler_id}")\n'
        f'set(CMAKE_CXX_SIMULATE_ID "{simulate_id}")\n',
        encoding="utf-8",
    )
    cache = {
        "CMAKE_CACHE_MAJOR_VERSION": "4",
        "CMAKE_CACHE_MINOR_VERSION": "3",
        "CMAKE_CACHE_PATCH_VERSION": "3",
    }
    errors = []

    actual = infra.cmake_compiler_uses_msvc_abi(tmp_path, cache, errors)

    assert errors == []
    assert actual is expected


def test_cmake_semantic_graph_probe_requires_fresh_file_api_reply(
    tmp_path, monkeypatch
):
    build = _configure_semantic_graph_fixture(tmp_path)
    errors = []
    infra.check_cmake_test_graph(tmp_path, build, errors)
    assert errors == []

    monkeypatch.setattr(
        infra.subprocess,
        "run",
        lambda *args, **kwargs: subprocess.CompletedProcess(args[0], 0, "", ""),
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors == [
        "CMake semantic test graph: CMake File API did not emit a fresh index"
    ]


def test_cmake_semantic_graph_probe_checks_post_configure_cache(tmp_path):
    cmake = tmp_path / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8")
        + '\nset(DART_USE_SYSTEM_PYBIND11 OFF CACHE BOOL "" FORCE)\n',
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "configured `DART_USE_SYSTEM_PYBIND11` must be `ON`" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_requires_release_configuration(tmp_path):
    cmake = tmp_path / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8")
        + '\nset(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)\n',
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any("must use `CMAKE_BUILD_TYPE=Release`" in error for error in errors)


def test_cmake_semantic_graph_probe_rejects_omitted_test_subtree(tmp_path):
    cmake = tmp_path / "tests" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "add_subdirectory(regression)\n",
            "",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "configured test graph omits" in error and "tests/regression" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_rejects_unregistered_ctest_target(tmp_path):
    cmake = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "add_test(NAME test_semantic_regression COMMAND "
            "test_semantic_regression)\n",
            "",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any("not registered with CTest" in error for error in errors)


def test_cmake_semantic_graph_probe_rejects_disabled_ctest_target(tmp_path):
    cmake = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8")
        + "\nset_tests_properties(test_semantic_regression "
        "PROPERTIES DISABLED TRUE)\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "CTest test `test_semantic_regression` is disabled" in error for error in errors
    )


def test_cmake_semantic_graph_probe_rejects_list_only_ctest_target(tmp_path):
    cmake = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "COMMAND test_semantic_regression)",
            "COMMAND test_semantic_regression --gtest_list_tests)",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "CTest test `test_semantic_regression` uses non-executing option "
        "`--gtest_list_tests`" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_rejects_unapproved_gtest_environment(
    tmp_path,
):
    cmake = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8")
        + "\nset_tests_properties(test_semantic_regression "
        'PROPERTIES ENVIRONMENT "GTEST_FILTER=-*")\n',
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "CTest test `test_semantic_regression` has unapproved GTest "
        "environment ['GTEST_FILTER=-*']" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_rejects_gtest_environment_modification(
    tmp_path,
):
    cmake = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8")
        + "\nset_tests_properties(test_semantic_regression PROPERTIES "
        'ENVIRONMENT_MODIFICATION "GTEST_FILTER=set:-*")\n',
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "CTest test `test_semantic_regression` has unapproved GTest "
        "environment modifications" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("property_name", "value"),
    (
        ("WILL_FAIL", "TRUE"),
        ("PASS_REGULAR_EXPRESSION", '".*"'),
        ("SKIP_REGULAR_EXPRESSION", '".*"'),
        ("SKIP_RETURN_CODE", "1"),
    ),
)
def test_cmake_semantic_graph_probe_rejects_result_neutralizing_properties(
    tmp_path, property_name, value
):
    cmake = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8")
        + "\nset_tests_properties(test_semantic_regression PROPERTIES "
        f"{property_name} {value})\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        f"CTest test `test_semantic_regression` uses result-neutralizing "
        f"property `{property_name}`" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_requires_zero_test_guard_for_selection(
    tmp_path, monkeypatch
):
    cmake = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "COMMAND test_semantic_regression)",
            "COMMAND test_semantic_regression --gtest_filter=Semantic.*)",
            1,
        ),
        encoding="utf-8",
    )
    monkeypatch.setitem(
        infra.APPROVED_CTEST_SELECTIONS,
        "test_semantic_regression",
        {
            "arguments": ("--gtest_filter=Semantic.*",),
            "environment": (),
        },
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "selected CTest test `test_semantic_regression` does not fail on zero "
        "tests" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_rejects_ambiguous_zero_test_guard(
    tmp_path, monkeypatch
):
    cmake = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "add_test(NAME test_semantic_regression COMMAND "
            "test_semantic_regression)\n",
            "add_test(NAME test_semantic_regression COMMAND "
            "test_semantic_regression --gtest_filter=Semantic.*)\n"
            "set_tests_properties(test_semantic_regression PROPERTIES "
            'FAIL_REGULAR_EXPRESSION "0 tests")\n',
            1,
        ),
        encoding="utf-8",
    )
    monkeypatch.setitem(
        infra.APPROVED_CTEST_SELECTIONS,
        "test_semantic_regression",
        {
            "arguments": ("--gtest_filter=Semantic.*",),
            "environment": (),
        },
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "does not fail on zero tests with exact pattern" in error for error in errors
    )


def test_cmake_test_runner_clears_future_ambient_gtest_controls(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    source = tmp_path / "tests" / "regression" / "test_semantic.cpp"
    source.write_text(
        """
#include <cstdlib>
#include <fstream>

int main()
{
  if (std::getenv("GTEST_FUTURE_SELECTOR") != nullptr)
    return 0;
  std::ofstream("semantic-body-ran") << "ran";
  return 0;
}
""".lstrip(),
        encoding="utf-8",
    )
    environment = os.environ.copy()
    environment["GTEST_FUTURE_SELECTOR"] = "skip-everything"

    result = subprocess.run(
        [
            "cmake",
            "--build",
            str(build),
            "--target",
            "tests_and_run",
            "--config",
            "Release",
        ],
        capture_output=True,
        text=True,
        env=environment,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert (build / "tests" / "regression" / "semantic-body-ran").is_file()


def _copy_test_runner_probe_contract(root: Path) -> None:
    for relative in (
        "pyproject.toml",
        "cmake/DARTRunCTest.cmake",
        "scripts/run_pytest.py",
    ):
        target = root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(
            (ROOT / relative).read_text(encoding="utf-8"),
            encoding="utf-8",
        )


def test_test_runner_semantic_probe_accepts_repository_contract(tmp_path):
    _copy_test_runner_probe_contract(tmp_path)
    errors = []

    infra.check_test_runner_semantics(tmp_path, errors)

    assert errors == []


def test_test_runner_semantic_probe_stays_on_repository_volume(tmp_path, monkeypatch):
    _copy_test_runner_probe_contract(tmp_path)
    original = infra.tempfile.TemporaryDirectory
    parents = []

    def tracked_temporary_directory(*args, **kwargs):
        parents.append(Path(kwargs["dir"]).resolve())
        return original(*args, **kwargs)

    monkeypatch.setattr(
        infra.tempfile,
        "TemporaryDirectory",
        tracked_temporary_directory,
    )
    errors = []

    infra.check_test_runner_semantics(tmp_path, errors)

    assert errors == []
    assert parents == [(tmp_path / "build").resolve()]


@pytest.mark.parametrize(
    ("relative", "old", "new", "expected"),
    (
        (
            "cmake/DARTRunCTest.cmake",
            "execute_process(\n  COMMAND\n",
            "set(_dart_ctest_arguments --show-only)\n" "execute_process(\n  COMMAND\n",
            "CTest did not execute a body",
        ),
        (
            "scripts/run_pytest.py",
            'name.upper().startswith("PYTEST_")',
            'name.upper().startswith("UNRELATED_")',
            "pytest did not execute a body",
        ),
    ),
)
def test_test_runner_semantic_probe_rejects_execution_bypass(
    tmp_path, relative, old, new, expected
):
    _copy_test_runner_probe_contract(tmp_path)
    path = tmp_path / relative
    text = path.read_text(encoding="utf-8")
    if relative.startswith("cmake/"):
        offset = text.rfind(old)
        assert offset > 0
        text = text[:offset] + text[offset:].replace(old, new, 1)
    else:
        assert old in text
        text = text.replace(old, new, 1)
    path.write_text(text, encoding="utf-8")
    errors = []

    infra.check_test_runner_semantics(tmp_path, errors)

    assert any(expected in error for error in errors)


def test_cmake_zero_test_guard_does_not_match_ten_tests(tmp_path, monkeypatch):
    cmake = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8")
        + "\nset_tests_properties(test_semantic_regression PROPERTIES "
        'ENVIRONMENT "GTEST_FILTER=Semantic.*" '
        f'FAIL_REGULAR_EXPRESSION "{infra.SAFE_ZERO_TEST_PATTERN}")\n',
        encoding="utf-8",
    )
    (tmp_path / "tests" / "regression" / "test_semantic.cpp").write_text(
        """
#include <iostream>

int main()
{
  std::cout << "10 tests from 1 test suite ran." << std::endl;
  return 0;
}
""".lstrip(),
        encoding="utf-8",
    )
    monkeypatch.setitem(
        infra.APPROVED_CTEST_SELECTIONS,
        "test_semantic_regression",
        {
            "arguments": (),
            "environment": ("GTEST_FILTER=Semantic.*",),
        },
    )
    errors = []
    infra.check_cmake_test_graph(tmp_path, build, errors)
    assert errors == []

    result = subprocess.run(
        [
            "cmake",
            "--build",
            str(build),
            "--target",
            "tests_and_run",
            "--config",
            "Release",
        ],
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stdout + result.stderr


def test_cmake_pytest_target_ignores_repository_conftest(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    test_source = tmp_path / "python" / "tests" / "test_semantic.py"
    test_source.write_text(
        """
from pathlib import Path


def test_semantic():
    Path(__file__).with_name("semantic-body-ran").write_text("ran")
""".lstrip(),
        encoding="utf-8",
    )
    (test_source.parent / "conftest.py").write_text(
        """
def pytest_collection_modifyitems(items):
    items.clear()


def pytest_sessionfinish(session):
    session.exitstatus = 0
""".lstrip(),
        encoding="utf-8",
    )
    errors = []
    infra.check_cmake_test_graph(tmp_path, build, errors)
    assert errors == []

    result = subprocess.run(
        [
            "cmake",
            "--build",
            str(build),
            "--target",
            "pytest",
            "--config",
            "Release",
        ],
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert (test_source.parent / "semantic-body-ran").is_file()


def test_cmake_semantic_graph_probe_rejects_explicit_local_pytest_plugin(
    tmp_path,
):
    build = _configure_semantic_graph_fixture(tmp_path)
    test_source = tmp_path / "python" / "tests" / "test_semantic.py"
    test_source.write_text(
        'pytest_plugins = ["evil"]\n\n\ndef test_semantic():\n    pass\n',
        encoding="utf-8",
    )
    (test_source.parent / "evil.py").write_text(
        "def pytest_collection_modifyitems(items):\n    items.clear()\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "declares `pytest_plugins`; local plugin loading is not allowed" in error
        for error in errors
    )


def test_cmake_semantic_graph_ignores_unreferenced_stale_ctest_file(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    stale = build / "tests" / "stale"
    stale.mkdir()
    (stale / "CTestTestfile.cmake").write_text(
        'add_test([=[stale_list_only]=] "test_semantic" ' '"--gtest_list_tests")\n',
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors == []


def test_cmake_semantic_graph_probe_rejects_unowned_test_source(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    (tmp_path / "tests" / "unit" / "test_orphan.cpp").write_text(
        "int main(void) { return 0; }\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "neither configured nor explicitly approved as inactive" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_rejects_nested_early_return(tmp_path):
    cmake = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        "return()\n" + cmake.read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "neither configured nor explicitly approved as inactive" in error
        for error in errors
    )


@pytest.mark.parametrize(
    "inactive_declaration",
    (
        """
if(FALSE)
  add_executable(test_orphan test_orphan.cpp)
endif()
""",
        """
function(register_orphan)
  add_executable(test_orphan test_orphan.cpp)
endfunction()
""",
    ),
)
def test_cmake_semantic_graph_probe_rejects_inactive_source_owner(
    tmp_path, inactive_declaration
):
    cmake = tmp_path / "tests" / "unit" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8") + inactive_declaration,
        encoding="utf-8",
    )
    (tmp_path / "tests" / "unit" / "test_orphan.cpp").write_text(
        "int main(void) { return 0; }\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "neither configured nor explicitly approved as inactive" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_accepts_explicit_inactive_source_policy(
    tmp_path,
):
    cmake = tmp_path / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "add_subdirectory(tests)",
            'set(DART_USE_SYSTEM_IMGUI ON CACHE BOOL "" FORCE)\n'
            "add_library(dart-external-imgui INTERFACE)\n"
            "add_subdirectory(tests)",
            1,
        ),
        encoding="utf-8",
    )
    regression = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    regression.write_text(
        regression.read_text(encoding="utf-8")
        + """
if(TARGET dart-external-imgui)
  if(NOT DART_USE_SYSTEM_IMGUI)
    dart_add_test("regression" test_Issue2516)
  endif()
endif()
""",
        encoding="utf-8",
    )
    (tmp_path / "tests" / "regression" / "test_Issue2516.cpp").write_text(
        "int main(void) { return 0; }\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors == []


def test_cmake_semantic_graph_probe_rejects_library_only_runtime_test(tmp_path):
    cmake = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "add_executable(test_semantic_regression test_semantic.cpp)",
            "add_library(test_semantic_regression STATIC test_semantic.cpp)",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any("belong only to non-executable targets" in error for error in errors)


def _add_compile_only_semantic_fixture(
    root: Path, *, target_type: str = "OBJECT", dependency: bool = True
) -> None:
    unit = root / "tests" / "unit" / "CMakeLists.txt"
    dependency_name = " UNIT_math_LegacyConvexHullC" if dependency else ""
    unit.write_text(
        unit.read_text(encoding="utf-8").replace(
            "set(unit_tests UNIT_semantic PARENT_SCOPE)",
            "add_subdirectory(math)\n"
            f"set(unit_tests UNIT_semantic{dependency_name} PARENT_SCOPE)",
            1,
        ),
        encoding="utf-8",
    )
    math = root / "tests" / "unit" / "math"
    math.mkdir()
    (math / "CMakeLists.txt").write_text(
        "\n" * 10 + f"add_library(UNIT_math_LegacyConvexHullC {target_type} "
        "test_LegacyConvexHullC.c)\n",
        encoding="utf-8",
    )
    (math / "test_LegacyConvexHullC.c").write_text(
        "int legacy_convex_hull_c(void) { return 0; }\n",
        encoding="utf-8",
    )


def test_cmake_semantic_graph_accepts_exact_compile_only_contract(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    _add_compile_only_semantic_fixture(tmp_path)
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors == []


@pytest.mark.parametrize(
    ("target_type", "dependency"),
    (("STATIC", True), ("OBJECT", False)),
)
def test_cmake_semantic_graph_rejects_compile_only_contract_drift(
    tmp_path, target_type, dependency
):
    build = _configure_semantic_graph_fixture(tmp_path)
    _add_compile_only_semantic_fixture(
        tmp_path, target_type=target_type, dependency=dependency
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "compile-only C++ test sources do not match their exact object-target "
        "contract" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_rejects_pytest_non_execution_flags(tmp_path):
    cmake = tmp_path / "python" / "tests" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "${dartpy_test_files} -v",
            "${dartpy_test_files} -v --collect-only",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "configured `pytest` does not originate from its validated source "
        "contract" in error
        or "expanded `pytest` command does not invoke" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("old", "new"),
    (
        (
            '"${PROJECT_SOURCE_DIR}/scripts/run_pytest.py"',
            '"${PROJECT_SOURCE_DIR}/scripts/Noop.py"',
        ),
        (
            '"${Python3_EXECUTABLE}" -I '
            '"${PROJECT_SOURCE_DIR}/scripts/run_pytest.py"',
            '"${Python3_EXECUTABLE}" -m pytest',
        ),
    ),
)
def test_cmake_semantic_graph_probe_rejects_unpinned_pytest_target(tmp_path, old, new):
    cmake = tmp_path / "python" / "tests" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    text = cmake.read_text(encoding="utf-8")
    assert old in text
    cmake.write_text(text.replace(old, new, 1), encoding="utf-8")
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "configured `pytest` does not originate from its validated source "
        "contract" in error
        or "expanded `pytest` command does not invoke" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_rejects_local_pytest_shadow(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    (tmp_path / "python" / "tests" / "pytest.py").write_text(
        '"""Local module that must not shadow the installed pytest package."""\n',
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "pytest target resolution does not match the trusted" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_does_not_execute_spoofed_pytest(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    marker = tmp_path / "pytest-shadow-executed"
    (tmp_path / "python" / "tests" / "pytest.py").write_text(
        "from pathlib import Path\n"
        f"Path({str(marker)!r}).write_text('executed')\n"
        "__file__ = '/trusted/pixi/site-packages/pytest/__init__.py'\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "pytest target resolution does not match the trusted" in error
        for error in errors
    )
    assert not marker.exists()


def test_cmake_semantic_graph_probe_rejects_pytest_package_shadow(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    marker = tmp_path / "pytest-package-shadow-executed"
    package = tmp_path / "python" / "tests" / "pytest"
    package.mkdir()
    (package / "__init__.py").write_text(
        "from pathlib import Path\n" f"Path({str(marker)!r}).write_text('executed')\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "pytest target resolution does not match the trusted" in error
        for error in errors
    )
    assert not marker.exists()


def test_cmake_semantic_graph_probe_rejects_configured_pytest_collection_only(
    tmp_path,
):
    build = _configure_semantic_graph_fixture(tmp_path)
    (tmp_path / "pyproject.toml").write_text(
        '[tool.pytest.ini_options]\naddopts = ["--collect-only"]\n',
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "pinned pytest options do not match the safe branch contract" in error
        for error in errors
    )


def test_cmake_semantic_graph_pins_root_pytest_config(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    (tmp_path / "python" / "tests" / "pytest.ini").write_text(
        "[pytest]\naddopts = --collect-only\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)
    result = subprocess.run(
        ["cmake", "--build", str(build), "--target", "pytest"],
        capture_output=True,
        text=True,
    )

    assert errors == []
    assert result.returncode == 0, result.stdout + result.stderr
    assert "1 passed" in result.stdout + result.stderr


def test_cmake_semantic_graph_uses_exact_test_directory_boundaries(tmp_path):
    tests_cmake = tmp_path / "tests" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    shadow = tmp_path / "tests" / "unit_shadow"
    shadow.mkdir()
    (shadow / "CMakeLists.txt").write_text(
        "add_executable(UNIT_shadow semantic.c)\n",
        encoding="utf-8",
    )
    (shadow / "semantic.c").write_text(
        "int main(void) { return 0; }\n",
        encoding="utf-8",
    )
    tests_cmake.write_text(
        tests_cmake.read_text(encoding="utf-8").replace(
            "add_subdirectory(unit)",
            "add_subdirectory(unit)\n" "add_subdirectory(unit_shadow)",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors == []


def test_cmake_semantic_graph_probe_accepts_msvc_runtime_branch(tmp_path):
    cmake = tmp_path / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "add_subdirectory(tests)",
            "set(MSVC TRUE)\nadd_subdirectory(tests)",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors == []


def test_cmake_semantic_graph_probe_rejects_pytest_fallback(tmp_path):
    cmake = tmp_path / "python/tests/CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "if(DARTPY_PYTEST_FOUND)",
            "set(DARTPY_PYTEST_FOUND FALSE)\nif(DARTPY_PYTEST_FOUND)",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any("`pytest` does not depend on `dartpy`" in error for error in errors)
    assert any("validated source contract" in error for error in errors)


@pytest.mark.parametrize(
    ("needle", "injection"),
    (
        (
            "add_custom_target(ALL DEPENDS ${all_targets})",
            "cmake_language(CALL return)\n",
        ),
        (
            "add_custom_target(ALL DEPENDS ${all_targets})",
            "include_guard(GLOBAL)\ninclude_guard(GLOBAL)\n",
        ),
        (
            "add_custom_target(ALL DEPENDS ${all_targets})",
            "function(add_custom_target)\nendfunction()\n",
        ),
        (
            "if(BUILD_TESTING)",
            "function(list)\nendfunction()\n",
        ),
        (
            "if(BUILD_TESTING)",
            "set(BUILD_TESTING OFF)\n",
        ),
        (
            "add_custom_target(ALL DEPENDS ${all_targets})",
            (
                "add_custom_target(ALL)\n"
                "function(add_custom_target)\n"
                "endfunction()\n"
            ),
        ),
        (
            "add_subdirectory(tests)",
            (
                "function(add_custom_target)\n"
                '  if("${ARGV0}" STREQUAL "tests_and_run")\n'
                "    _add_custom_target(\n"
                "      tests_and_run\n"
                "      DEPENDS ${integration_tests} ${regression_tests} "
                "${unit_tests}\n"
                "    )\n"
                "  else()\n"
                "    _add_custom_target(${ARGV})\n"
                "  endif()\n"
                "endfunction()\n"
            ),
        ),
        (
            "add_subdirectory(tests)",
            'set(CMAKE_CTEST_COMMAND "${CMAKE_COMMAND};-E;true")\n',
        ),
        (
            "add_subdirectory(tests)",
            (
                'set(CMAKE_CTEST_COMMAND "${CMAKE_COMMAND}" '
                'CACHE FILEPATH "" FORCE)\n'
            ),
        ),
        (
            "add_subdirectory(python/tests)",
            'set(Python3_EXECUTABLE "${CMAKE_COMMAND}")\n',
        ),
        (
            "add_subdirectory(python/tests)",
            (
                'set(_Python3_EXECUTABLE "${CMAKE_COMMAND}" '
                'CACHE INTERNAL "" FORCE)\n'
                'set(Python3_EXECUTABLE "${CMAKE_COMMAND}")\n'
            ),
        ),
        (
            "add_subdirectory(python/tests)",
            'set(DART_PYTHONPATH "${CMAKE_BINARY_DIR}/unrelated")\n',
        ),
        (
            "add_subdirectory(python/tests)",
            'set(DART_PYTHONPATH "$<TARGET_FILE_DIR:dartpy>")\n',
        ),
    ),
)
def test_cmake_semantic_graph_probe_rejects_effective_bypasses(
    tmp_path, needle, injection
):
    cmake = tmp_path / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            needle,
            injection + needle,
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors
    assert all(error.startswith("CMake semantic test graph:") for error in errors)


def test_cmake_semantic_graph_probe_fails_closed_on_reconfigure_error(tmp_path):
    cmake = tmp_path / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8") + "\nunknown_graph_command()\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any("configure probe failed" in error for error in errors)


def test_test_gate_contract_requires_multiconfig_ctest_selection(tmp_path):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / "tests/CMakeLists.txt"
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "-DDART_CTEST_CONFIGURATION=$<CONFIG>",
            "-DDART_CTEST_CONFIGURATION=${CMAKE_BUILD_TYPE}",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        "tests/CMakeLists.txt: missing `test-all` graph marker "
        "`cmake/DARTRunCTest.cmake`" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("old", "new", "expected"),
    (
        (
            'set(DART_DARTPY_BUILD_DIR "$<TARGET_FILE_DIR:dartpy>")',
            'set(DART_DARTPY_BUILD_DIR "${DART_PYTHON_BUILD_DIR}/dartpy")',
            "`DART_DARTPY_BUILD_DIR` must derive from `$<TARGET_FILE_DIR:dartpy>`",
        ),
        (
            'set(DART_PYTHONPATH "${DART_DARTPY_BUILD_DIR}\\\\;'
            '${DART_PYTHON_BUILD_DIR}")',
            'set(DART_PYTHONPATH "${DART_DARTPY_BUILD_DIR}")',
            "`DART_PYTHONPATH` must combine the target output directory and "
            "Python build root under `if(WIN32)`",
        ),
        (
            'set(DART_PYTHONPATH "${DART_DARTPY_BUILD_DIR}:'
            '${DART_PYTHON_BUILD_DIR}")',
            'set(DART_PYTHONPATH "${DART_DARTPY_BUILD_DIR}")',
            "`DART_PYTHONPATH` must combine the target output directory and "
            "Python build root under the non-Windows branch",
        ),
    ),
)
def test_test_gate_contract_requires_config_aware_dartpy_path(
    tmp_path, old, new, expected
):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / "python/CMakeLists.txt"
    text = cmake.read_text(encoding="utf-8")
    assert old in text
    cmake.write_text(text.replace(old, new, 1), encoding="utf-8")
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(expected in error for error in errors)


def test_test_gate_contract_requires_dartpy_path_before_tests(tmp_path):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / "python/CMakeLists.txt"
    text = cmake.read_text(encoding="utf-8")
    old = (
        "add_subdirectory(dartpy)\n"
        'set(DART_DARTPY_BUILD_DIR "$<TARGET_FILE_DIR:dartpy>")'
    )
    new = (
        'set(DART_DARTPY_BUILD_DIR "$<TARGET_FILE_DIR:dartpy>")\n'
        "add_subdirectory(dartpy)"
    )
    assert old in text
    cmake.write_text(text.replace(old, new, 1), encoding="utf-8")
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        "define `dartpy`, derive its configuration-aware output path" in error
        for error in errors
    )


def test_test_gate_contract_rejects_stale_task_handoff_semantics(tmp_path):
    _copy_test_gate_contract(tmp_path)
    handoff = tmp_path / "docs/dev_tasks/example/HANDOFF.md"
    handoff.parent.mkdir(parents=True)
    handoff.write_text(
        "- If `test-all` reports a Debug nanobind failure, clear its cache.\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert errors == [
        "docs/dev_tasks/example/HANDOFF.md:1: `test-all` is a Release-only "
        "build task; remove Debug/nanobind guidance"
    ]


def test_test_gate_contract_rejects_main_only_nanobind_cache_advice(tmp_path):
    _copy_test_gate_contract(tmp_path)
    packet = tmp_path / "docs/dev_tasks/example/07-work-packet.md"
    packet.parent.mkdir(parents=True)
    packet.write_text(
        '- Debug "nanobind not found" is the known poisoned-CMakeCache issue.\n',
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert errors == [
        "docs/dev_tasks/example/07-work-packet.md:1: remove stale main-only "
        "nanobind cache guidance"
    ]


@pytest.mark.parametrize(
    "claim",
    [
        "`pixi run test-all` only builds and does not run tests.",
        "`test-all` does not run CTest or pytest.",
        "`test-all` is not runtime coverage.",
    ],
)
def test_test_gate_contract_rejects_false_no_runtime_claims(tmp_path, claim):
    _copy_test_gate_contract(tmp_path)
    handoff = tmp_path / "docs/dev_tasks/example/HANDOFF.md"
    handoff.parent.mkdir(parents=True)
    handoff.write_text(f"- {claim}\n", encoding="utf-8")
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert errors == [
        "docs/dev_tasks/example/HANDOFF.md:1: `test-all` must not be "
        "described as build-only or lacking runtime coverage"
    ]


@pytest.mark.parametrize(
    "claim",
    [
        "`pixi run test-all` is the full lint/build/test aggregate.",
        "`test-all` runs lint before building and testing.",
    ],
)
def test_test_gate_contract_rejects_false_lint_claims(tmp_path, claim):
    _copy_test_gate_contract(tmp_path)
    handoff = tmp_path / "docs/dev_tasks/example/HANDOFF.md"
    handoff.parent.mkdir(parents=True)
    handoff.write_text(f"- {claim}\n", encoding="utf-8")
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert errors == [
        "docs/dev_tasks/example/HANDOFF.md:1: `test-all` must not be "
        "described as providing lint coverage"
    ]


def test_test_gate_contract_accepts_runtime_graph_claim(tmp_path):
    _copy_test_gate_contract(tmp_path)
    handoff = tmp_path / "docs/dev_tasks/example/HANDOFF.md"
    handoff.parent.mkdir(parents=True)
    handoff.write_text(
        "- `test-all` provides C++ and Python runtime coverage through the "
        "CMake `ALL` graph; run lint separately.\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert errors == []


def test_agent_hook_path_routing_is_bounded():
    assert hook.is_ai_infrastructure_path("docs/ai/README.md")
    assert hook.is_ai_infrastructure_path("docs/onboarding/architecture.md")
    assert hook.is_ai_infrastructure_path("docs/onboarding/testing.md")
    assert hook.is_ai_infrastructure_path(".codex/agents/dart_scout.toml")
    assert hook.is_ai_infrastructure_path(".github/workflows/ci_macos.yml")
    assert hook.is_ai_infrastructure_path("python/tests/unit/gui/test_agent_capture.py")
    assert hook.is_ai_infrastructure_path(
        "python/tests/unit/gui/test_agent_debug_overlay.py"
    )
    assert hook.is_ai_infrastructure_path(".gitignore")
    assert hook.is_ai_infrastructure_path("pixi.toml")
    assert hook.is_ai_infrastructure_path("dart/new_module/AGENTS.md")
    for path in (
        "CMakeLists.txt",
        "cmake/DARTRunCTest.cmake",
        "python/CMakeLists.txt",
        "python/tests/CMakeLists.txt",
        "scripts/run_pytest.py",
        "tests/CMakeLists.txt",
    ):
        assert hook.is_ai_infrastructure_path(path)
    assert not hook.is_ai_infrastructure_path("dart/dynamics/BodyNode.cpp")


def test_nested_agent_instructions_are_drift_sources(tmp_path):
    nested = tmp_path / "dart" / "new_module" / "AGENTS.md"
    nested.parent.mkdir(parents=True)
    nested.write_text("nested instructions\n")

    assert nested in infra.source_paths(tmp_path)


def test_non_git_snapshot_enforces_nested_instruction_budget(tmp_path):
    root_agents = tmp_path / "AGENTS.md"
    nested = tmp_path / "dart" / "new_module" / "AGENTS.md"
    nested.parent.mkdir(parents=True)
    root_agents.write_text("r" * (infra.MAX_AGENT_INSTRUCTION_BYTES // 2 + 1))
    nested.write_text("n" * (infra.MAX_AGENT_INSTRUCTION_BYTES // 2 + 1))
    errors = []

    infra.check_instruction_budget(tmp_path, errors)

    assert any("instruction chain" in error for error in errors)


def test_untracked_personal_skill_is_not_related_dart_infrastructure(tmp_path):
    personal = tmp_path / ".agents" / "skills" / "personal-helper" / "SKILL.md"
    personal.parent.mkdir(parents=True)
    personal.write_text("---\nname: personal-helper\n---\n")

    assert not hook.is_related_worktree_ai_path(
        tmp_path, ".agents/skills/personal-helper/SKILL.md"
    )


def test_one_shot_setup_is_bounded_to_sync_and_hook_install():
    commands = setup.setup_commands()
    assert [command[1] for command in commands] == [
        "scripts/sync_ai_commands.py",
        "scripts/install_git_hooks.py",
        "scripts/check_ai_infrastructure.py",
    ]
    assert commands[-1][-1] == "--check"


def test_broken_relative_markdown_link_is_rejected(tmp_path):
    doc = tmp_path / "docs" / "ai" / "README.md"
    doc.parent.mkdir(parents=True)
    doc.write_text("[missing](../missing.md)\n")

    errors = infra.markdown_link_errors(tmp_path, doc)

    assert any("broken link" in error for error in errors)


def test_dev_task_evidence_policy_rejects_tracked_assets_and_links(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    ignore = tmp_path / "docs" / "dev_tasks" / ".gitignore"
    task = tmp_path / "docs" / "dev_tasks" / "example"
    asset = task / "assets" / "raw.csv"
    asset.parent.mkdir(parents=True)
    ignore.write_text("**/assets/\n", encoding="utf-8")
    asset.write_text("raw\n", encoding="utf-8")
    (task / "README.md").write_text("![](assets/raw.csv)\n", encoding="utf-8")
    outside = tmp_path / "docs" / "guide.md"
    outside.write_text(
        "![raw][task-raw]\n" "[task-raw]:\n" "  dev_tasks/example/assets/raw.csv\n",
        encoding="utf-8",
    )
    subprocess.run(
        ["git", "-C", str(tmp_path), "add", "docs/dev_tasks/.gitignore"],
        check=True,
    )
    subprocess.run(["git", "-C", str(tmp_path), "add", "docs/guide.md"], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "add", "-f", "docs/dev_tasks/example"],
        check=True,
    )

    errors = infra.dev_task_evidence_policy_errors(tmp_path)

    assert any("generated task evidence is tracked" in error for error in errors)
    assert sum("link targets ignored task evidence" in error for error in errors) == 2


def test_dev_task_evidence_policy_accepts_inline_paths_and_docs_assets(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    ignore = tmp_path / "docs" / "dev_tasks" / ".gitignore"
    task = tmp_path / "docs" / "dev_tasks" / "example"
    published = tmp_path / "docs" / "assets" / "published.png"
    task.mkdir(parents=True)
    published.parent.mkdir(parents=True)
    ignore.write_text("**/assets/\n", encoding="utf-8")
    published.write_bytes(b"png")
    (task / "README.md").write_text(
        "Local `assets/raw.csv`; [published](../../assets/published.png).\n",
        encoding="utf-8",
    )
    subprocess.run(["git", "-C", str(tmp_path), "add", "."], check=True)

    assert infra.dev_task_evidence_policy_errors(tmp_path) == []


def test_dev_task_evidence_policy_requires_role_based_ignore_rule(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    ignore = tmp_path / "docs" / "dev_tasks" / ".gitignore"
    ignore.parent.mkdir(parents=True)
    ignore.write_text("# missing policy\n", encoding="utf-8")
    subprocess.run(["git", "-C", str(tmp_path), "add", "."], check=True)

    errors = infra.dev_task_evidence_policy_errors(tmp_path)

    assert any("missing role-based `**/assets/` rule" in error for error in errors)


def test_dev_task_evidence_policy_rejects_untracked_policy_file(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    ignore = tmp_path / "docs" / "dev_tasks" / ".gitignore"
    ignore.parent.mkdir(parents=True)
    ignore.write_text("**/assets/\n", encoding="utf-8")

    errors = infra.dev_task_evidence_policy_errors(tmp_path)

    assert any("task evidence policy is not tracked" in error for error in errors)


def test_dev_task_evidence_policy_rejects_ineffective_negation(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    ignore = tmp_path / "docs" / "dev_tasks" / ".gitignore"
    ignore.parent.mkdir(parents=True)
    ignore.write_text("**/assets/\n!**/assets/\n!**/assets/**\n", encoding="utf-8")
    subprocess.run(["git", "-C", str(tmp_path), "add", "."], check=True)

    errors = infra.dev_task_evidence_policy_errors(tmp_path)

    assert any(
        "docs/dev_tasks/.gitignore:2: staged negation rule" in error for error in errors
    )
    assert any("task assets are not effectively ignored" in error for error in errors)


def test_dev_task_evidence_policy_rejects_nested_task_unignore(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    root_ignore = tmp_path / "docs" / "dev_tasks" / ".gitignore"
    task = tmp_path / "docs" / "dev_tasks" / "example"
    task.mkdir(parents=True)
    root_ignore.write_text("**/assets/\n", encoding="utf-8")
    (task / ".gitignore").write_text(
        "!assets/\nassets/*\n!assets/*.csv\n", encoding="utf-8"
    )
    (task / "README.md").write_text("task\n", encoding="utf-8")
    subprocess.run(["git", "-C", str(tmp_path), "add", "."], check=True)

    errors = infra.dev_task_evidence_policy_errors(tmp_path)

    assert any(
        "docs/dev_tasks/example/.gitignore:3: staged negation rule" in error
        and "!assets/*.csv" in error
        for error in errors
    )


def test_dev_task_evidence_policy_validates_staged_policy_content(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    ignore = tmp_path / "docs" / "dev_tasks" / ".gitignore"
    ignore.parent.mkdir(parents=True)
    ignore.write_text("# staged policy is invalid\n", encoding="utf-8")
    subprocess.run(["git", "-C", str(tmp_path), "add", "."], check=True)
    ignore.write_text("**/assets/\n", encoding="utf-8")

    errors = infra.dev_task_evidence_policy_errors(tmp_path)

    assert any("missing role-based `**/assets/` rule" in error for error in errors)


def test_dev_task_evidence_policy_reads_staged_markdown_content(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    ignore = tmp_path / "docs" / "dev_tasks" / ".gitignore"
    task = tmp_path / "docs" / "dev_tasks" / "example"
    task.mkdir(parents=True)
    ignore.write_text("**/assets/\n", encoding="utf-8")
    readme = task / "README.md"
    readme.write_text("[raw](assets/raw.csv)\n", encoding="utf-8")
    subprocess.run(["git", "-C", str(tmp_path), "add", "."], check=True)
    readme.write_text("Local `assets/raw.csv`.\n", encoding="utf-8")

    errors = infra.dev_task_evidence_policy_errors(tmp_path)

    assert any("link targets ignored task evidence" in error for error in errors)


def test_dev_task_evidence_policy_fails_closed_outside_git(tmp_path):
    ignore = tmp_path / "docs" / "dev_tasks" / ".gitignore"
    ignore.parent.mkdir(parents=True)
    ignore.write_text("**/assets/\n", encoding="utf-8")

    errors = infra.dev_task_evidence_policy_errors(tmp_path)

    assert any("unable to enumerate tracked files" in error for error in errors)


def test_expected_agent_set_is_small_and_release_specific():
    assert infra.EXPECTED_AGENTS == {
        "dart_release_auditor",
        "dart_reviewer",
        "dart_scout",
    }


def test_model_upgrade_workflow_is_bounded_and_future_model_scalable():
    workflow = (ROOT / ".claude" / "commands" / "dart-model-upgrade.md").read_text()

    assert "workflow is itself an audit surface" in workflow
    assert "do not clone the workflow" in workflow
    assert "apply/adapt/omit" in workflow
    assert "durable project context" in workflow
    assert "representative DART 6 physics investigation" in workflow
    assert "Images are never the" in workflow
    assert "sole correctness oracle" in workflow
    assert "configured CMake File API result" in workflow
    assert "source-marker matches" in workflow


def test_ultrawork_uses_explicit_delegation_and_lean_prompt_shape():
    workflow = (ROOT / ".claude" / "commands" / "dart-ultrawork.md").read_text()

    assert "Delegate only when the user explicitly requested it" in workflow
    assert "execute packets serially" in workflow
    assert "Do not repeat this workflow's logistics" in workflow
    assert "Sol Ultra" not in workflow


def _write_agent_profiles(root, *, malformed_instructions=False):
    agents_dir = root / ".codex" / "agents"
    agents_dir.mkdir(parents=True)
    for name in sorted(infra.EXPECTED_AGENTS):
        instructions = (
            "1"
            if malformed_instructions and name == "dart_scout"
            else '"Inputs: bounded evidence. Output: findings."'
        )
        (agents_dir / f"{name}.toml").write_text(
            f'name = "{name}"\n'
            'description = "Read-only test profile"\n'
            'sandbox_mode = "read-only"\n'
            f"developer_instructions = {instructions}\n"
        )


def test_malformed_codex_config_and_agent_toml_return_errors(tmp_path):
    (tmp_path / ".codex").mkdir()
    (tmp_path / ".codex" / "config.toml").write_text('agents = "bad"\n')
    _write_agent_profiles(tmp_path, malformed_instructions=True)
    errors = []

    infra.check_codex_config(tmp_path, errors)

    assert ".codex/config.toml: agents must be a table" in errors
    assert any("instructions need Inputs and Output" in error for error in errors)


def test_boolean_codex_agent_limits_are_rejected(tmp_path):
    (tmp_path / ".codex").mkdir()
    (tmp_path / ".codex" / "config.toml").write_text(
        "[agents]\nmax_threads = 4\nmax_depth = true\n"
    )
    _write_agent_profiles(tmp_path)
    errors = []

    infra.check_codex_config(tmp_path, errors)

    assert ".codex/config.toml: agents.max_depth must be 1" in errors


@pytest.mark.parametrize(
    "key",
    (
        "model",
        "model_reasoning_effort",
        "review_model",
        "approval_policy",
        "sandbox_mode",
    ),
)
def test_project_codex_runtime_pins_are_rejected(tmp_path, key):
    (tmp_path / ".codex").mkdir()
    (tmp_path / ".codex" / "config.toml").write_text(
        f'{key} = "pinned"\n[agents]\nmax_threads = 4\nmax_depth = 1\n'
    )
    _write_agent_profiles(tmp_path)
    errors = []

    infra.check_codex_config(tmp_path, errors)

    assert any("project config must not pin" in error for error in errors)


def test_unexpected_project_codex_runtime_key_is_rejected(tmp_path):
    (tmp_path / ".codex").mkdir()
    (tmp_path / ".codex" / "config.toml").write_text(
        'experimental_override = "on"\n'
        "[agents]\n"
        "max_threads = 4\n"
        "max_depth = 1\n"
    )
    _write_agent_profiles(tmp_path)
    errors = []

    infra.check_codex_config(tmp_path, errors)

    assert ".codex/config.toml: root keys must equal agents" in errors


@pytest.mark.parametrize("key", ("model", "model_reasoning_effort", "review_model"))
def test_custom_agent_model_pins_are_rejected_and_reported(tmp_path, key):
    (tmp_path / ".codex").mkdir()
    (tmp_path / ".codex" / "config.toml").write_text(
        "[agents]\nmax_threads = 4\nmax_depth = 1\n"
    )
    _write_agent_profiles(tmp_path)
    profile = tmp_path / ".codex" / "agents" / "dart_scout.toml"
    profile.write_text(profile.read_text() + f'{key} = "pinned"\n')
    errors = []

    infra.check_codex_config(tmp_path, errors)
    pins = infra._model_harness_inventory(tmp_path)["model_pins"]["custom_agents"]

    assert any("keys must equal" in error for error in errors)
    assert any("inherit the parent model" in error for error in errors)
    assert pins == [
        {
            "path": ".codex/agents/dart_scout.toml",
            "keys": [key],
        }
    ]


def test_unexpected_custom_agent_key_is_rejected(tmp_path):
    (tmp_path / ".codex").mkdir()
    (tmp_path / ".codex" / "config.toml").write_text(
        "[agents]\nmax_threads = 4\nmax_depth = 1\n"
    )
    _write_agent_profiles(tmp_path)
    profile = tmp_path / ".codex" / "agents" / "dart_scout.toml"
    profile.write_text(profile.read_text() + 'temperature = "high"\n')
    errors = []

    infra.check_codex_config(tmp_path, errors)

    assert any("keys must equal" in error for error in errors)


def test_doctor_report_inventories_model_context_and_visual_harness():
    report = infra.doctor_report(ROOT)

    assert report["schema_version"] == 1
    assert report["profile"] == {
        "name": "release-6.20",
        "cpp_standard": "C++17",
        "python_binding": "pybind11",
        "io_namespace": "dart::utils",
        "gui_backend": "OSG",
    }
    assert report["inventory"]["model_harness"]["model_pins"] == {
        "project": [],
        "project_agents": [],
        "custom_agents": [],
    }
    assert report["inventory"]["model_harness"]["generated_skill_metadata_chars"] > 0
    assert report["inventory"]["durable_context"]["owners"]["count"] == 5
    visual = report["inventory"]["visual_verification"]
    assert visual["backend"] == "OSG"
    assert "agent-capture" in visual["tasks"]
    assert "image-verdict" in visual["tasks"]
    assert "verification-bundle" in visual["tasks"]
    json.dumps(report)


def test_malformed_hook_json_returns_errors_instead_of_tracebacks(tmp_path):
    (tmp_path / ".codex").mkdir()
    (tmp_path / ".codex" / "hooks.json").write_text(
        json.dumps(
            {
                "hooks": {
                    "PreToolUse": [
                        {"matcher": "^Bash$", "hooks": [1]},
                    ]
                }
            }
        )
    )
    (tmp_path / ".claude").mkdir()
    (tmp_path / ".claude" / "settings.json").write_text(
        json.dumps(
            {
                "hooks": {
                    "PreToolUse": [
                        {"matcher": "Bash", "hooks": [{"command": 1}]},
                    ]
                }
            }
        )
    )
    errors = []

    infra.check_hooks(tmp_path, errors)

    assert ".codex/hooks.json: expected one bounded PreToolUse hook" in errors
    assert ".claude/settings.json: invalid or stale hook wiring" in errors


def test_hooks_require_cross_platform_commands(tmp_path):
    (tmp_path / ".codex").mkdir()
    (tmp_path / ".codex" / "hooks.json").write_text(
        json.dumps(
            {
                "hooks": {
                    "PreToolUse": [
                        {
                            "matcher": "^Bash$",
                            "hooks": [
                                {
                                    "type": "command",
                                    "command": infra.CODEX_HOOK_COMMAND,
                                    "timeout": 15,
                                    "statusMessage": "Checking DART commit command",
                                }
                            ],
                        }
                    ]
                }
            }
        )
    )
    (tmp_path / ".claude").mkdir()
    (tmp_path / ".claude" / "settings.json").write_text(
        json.dumps(
            {
                "hooks": {
                    "PreToolUse": [
                        {
                            "matcher": "Bash",
                            "hooks": [
                                {
                                    "type": "command",
                                    "command": "/usr/bin/python3 stale.py",
                                }
                            ],
                        }
                    ]
                }
            }
        )
    )
    for path in (
        tmp_path / "scripts" / "install_git_hooks.py",
        tmp_path / ".claude" / "hooks" / "pre-commit-guard.sh",
    ):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("scripts/check_agent_hook.py --profile staged\n")
    errors = []

    infra.check_hooks(tmp_path, errors)

    assert ".codex/hooks.json: Windows hook command wiring is stale" in errors
    assert ".claude/settings.json: invalid or stale hook wiring" in errors


def test_release_hook_validation_rejects_unsafe_suffix_and_extra_keys(tmp_path):
    for relative in (
        ".codex/hooks.json",
        ".claude/settings.json",
        ".claude/hooks/pre-commit-guard.sh",
        ".claude/hooks/pre-commit-guard.ps1",
        "scripts/install_git_hooks.py",
        "scripts/pretool_guard_bridge.py",
    ):
        destination = tmp_path / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text((ROOT / relative).read_text())
    hooks_path = tmp_path / ".codex" / "hooks.json"
    data = json.loads(hooks_path.read_text())
    handler = data["hooks"]["PreToolUse"][0]["hooks"][0]
    handler["command"] += " && git push"
    handler["extra"] = "ignored"
    hooks_path.write_text(json.dumps(data))
    errors = []

    infra.check_hooks(tmp_path, errors)

    assert ".codex/hooks.json: hook command wiring is stale" in errors
    assert any("handler keys must equal" in error for error in errors)


def _write_visual_runtime_fixtures(root):
    overlay_path = root / "python/tests/unit/gui/test_agent_debug_overlay.py"
    overlay_path.parent.mkdir(parents=True, exist_ok=True)
    overlay_path.write_text(
        'pytest.fail("no off-screen GL context despite a configured DISPLAY")\n'
        "assert with_overlay != base\n"
        "assert after_clear == base\n"
    )
    capture_path = root / "python/tests/unit/gui/test_agent_capture.py"
    capture_path.write_text(
        "test_run_capture_smoke_writes_stills_and_sidecar\n"
        "test_run_capture_debug_layers_change_pixels_end_to_end\n"
        'factory="claim_capture_scene:make_world"\n'
        "pytest.fail(str(error))\n"
        'assert "--factory claim_capture_scene:make_world"\n'
        'debug_layers = ["contacts", "collision_bounds", "labels"]\n'
        'plain_artifact["camera"] == combined_artifact["camera"]\n'
        "assert _changed_pixel_count(plain_image, combined_image) >= 128\n"
        "agent_debug_overlay.CONTACT_POINT_RGB\n"
        "assert contact_pixels >= 4\n"
        "for layer in debug_layers\n"
        'assert debug["layers"] == [layer]\n'
        "assert _changed_pixel_count(plain_image, debug_image) >= 32\n"
    )
    (root / "pixi.toml").write_text(
        "test-agent-debug-overlay =\n"
        "test_contacts_layer_marks_points_and_normals\n"
        "test_engine_rendered_overlay_changes_pixels\n"
        "test_run_capture_smoke_writes_stills_and_sidecar\n"
        "test_run_capture_debug_layers_change_pixels_end_to_end\n"
    )


def test_ci_wiring_requires_native_windows_hook_smoke(tmp_path):
    workflows = tmp_path / ".github" / "workflows"
    workflows.mkdir(parents=True)
    (workflows / "ci_ubuntu.yml").write_text(
        "pixi run check-ai-commands\n"
        "pixi run check-ai-infra\n"
        "pixi run test-ai-infra\n"
        "scripts/check_ai_infrastructure.py --scenarios\n"
        "      - name: Agent visual verification smoke\n"
        "if: matrix.build_type == 'Release'\n"
        "xvfb-run\n"
        "bash -eu -o pipefail <<'VISUAL_SMOKE'\n"
        "pixi run agent-capture\n"
        "--scene box_on_ground --steps 250 --focus box --auto-views 1\n"
        "--layers contacts collision_bounds labels\n"
        "--width 320 --height 240\n"
        "--out /tmp/dart-agent-visual-smoke --prefix smoke\n"
        "pixi run image-verdict\n"
        "/tmp/dart-agent-visual-smoke/smoke_auto0.png\n"
        "pixi run agent-capture\n"
        "--scene dart_shape_contacts --steps 500\n"
        "--camera-azimuth 1.5708 --camera-elevation 0.35\n"
        "--camera-distance 4.5 --camera-target 0 0 0.2\n"
        "--layers contacts collision_bounds labels\n"
        "--width 640 --height 480\n"
        "--out /tmp/dart-agent-visual-smoke --prefix dart\n"
        "pixi run image-verdict\n"
        "/tmp/dart-agent-visual-smoke/dart_main.png\n"
        "xvfb-run\n"
        "pixi run test-agent-debug-overlay\n"
    )
    (workflows / "ci_windows.yml").write_text(
        "Native Windows hook smoke\n"
        'pixi run python -c "import sys; print(sys.executable)"\n'
        "$launcher\n"
        "$hookCommand\n"
        "& (Join-Path (git rev-parse --show-toplevel)\n"
        '$ErrorActionPreference = "Continue"\n'
        '"git status"\n'
        "DART_HOOK_DRY_RUN\n"
        '"git commit --no-verify -m x"\n'
        "$diagnosticOutput\n"
        "$diagnosticExit -ne 0\n"
        "$successExit -ne 0\n"
        "-File $launcher\n"
        "$rawSuccessExit\n"
        "$rawBlockedExit\n"
        "cmd.exe /c exit 0\n"
    )
    _write_visual_runtime_fixtures(tmp_path)
    errors = []

    infra.check_ci_wiring(tmp_path, errors)

    assert errors == [
        ".github/workflows/ci_windows.yml: missing native hook smoke marker "
        "`$blockedExit -ne 2`"
    ]


def test_ci_wiring_requires_semantic_ai_completion_task(tmp_path):
    workflow = tmp_path / ".github" / "workflows" / "ci_ubuntu.yml"
    workflow.parent.mkdir(parents=True)
    workflow.write_text(
        (ROOT / ".github/workflows/ci_ubuntu.yml")
        .read_text(encoding="utf-8")
        .replace(
            "pixi run check-ai-infra",
            "pixi run python scripts/check_ai_infrastructure.py --check",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_ci_wiring(tmp_path, errors)

    assert any(
        ".github/workflows/ci_ubuntu.yml: missing AI check "
        "`pixi run check-ai-infra`" in error
        for error in errors
    )


@pytest.mark.parametrize(
    "marker",
    (
        "Agent visual verification smoke",
        "xvfb-run",
        "bash -eu -o pipefail <<'VISUAL_SMOKE'",
        "pixi run agent-capture",
        "--scene box_on_ground --steps 250 --focus box --auto-views 1",
        "--scene dart_shape_contacts --steps 500",
        "--camera-azimuth 1.5708 --camera-elevation 0.35",
        "--camera-distance 4.5 --camera-target 0 0 0.2",
        "--layers contacts collision_bounds labels",
        "--width 320 --height 240",
        "--width 640 --height 480",
        "--out /tmp/dart-agent-visual-smoke --prefix smoke",
        "--out /tmp/dart-agent-visual-smoke --prefix dart",
        "pixi run image-verdict",
        "/tmp/dart-agent-visual-smoke/smoke_auto0.png",
        "/tmp/dart-agent-visual-smoke/dart_main.png",
        "pixi run test-agent-debug-overlay",
    ),
)
def test_ci_wiring_requires_visual_verification_smoke(tmp_path, marker):
    workflows = tmp_path / ".github" / "workflows"
    workflows.mkdir(parents=True)
    ubuntu = (ROOT / ".github/workflows/ci_ubuntu.yml").read_text()
    if marker == "Agent visual verification smoke":
        mutated = ubuntu.replace(marker, "missing-visual-marker", 1)
    else:
        before, separator, visual_and_after = ubuntu.partition(
            "- name: Agent visual verification smoke"
        )
        assert separator
        mutated = (
            before
            + separator
            + visual_and_after.replace(marker, "missing-visual-marker", 1)
        )
    (workflows / "ci_ubuntu.yml").write_text(mutated)
    (workflows / "ci_windows.yml").write_text(
        (ROOT / ".github/workflows/ci_windows.yml").read_text()
    )
    _write_visual_runtime_fixtures(tmp_path)
    errors = []

    infra.check_ci_wiring(tmp_path, errors)

    assert any("missing visual smoke" in error and marker in error for error in errors)


@pytest.mark.parametrize(
    ("mutation", "expected"),
    (
        ("condition", "exactly the Release matrix entry"),
        ("condition-suffix", "exactly the Release matrix entry"),
        ("soft-fail", "must not use continue-on-error"),
    ),
)
def test_visual_smoke_cannot_be_skipped_or_soft_failed(tmp_path, mutation, expected):
    workflows = tmp_path / ".github" / "workflows"
    workflows.mkdir(parents=True)
    ubuntu = (ROOT / ".github/workflows/ci_ubuntu.yml").read_text()
    if mutation == "condition":
        ubuntu = ubuntu.replace(
            "if: matrix.build_type == 'Release'",
            "if: false",
            1,
        )
    elif mutation == "condition-suffix":
        ubuntu = ubuntu.replace(
            "if: matrix.build_type == 'Release'",
            "if: matrix.build_type == 'Release' && false",
            1,
        )
    else:
        ubuntu = ubuntu.replace(
            "- name: Agent visual verification smoke",
            "- name: Agent visual verification smoke\n        continue-on-error: true",
            1,
        )
    (workflows / "ci_ubuntu.yml").write_text(ubuntu)
    (workflows / "ci_windows.yml").write_text(
        (ROOT / ".github/workflows/ci_windows.yml").read_text()
    )
    _write_visual_runtime_fixtures(tmp_path)
    errors = []

    infra.check_ci_wiring(tmp_path, errors)

    assert any(expected in error for error in errors)


@pytest.mark.parametrize(
    "marker",
    (
        'pytest.fail("no off-screen GL context despite a configured DISPLAY")',
        "assert with_overlay != base",
        "assert after_clear == base",
    ),
)
def test_ci_wiring_requires_non_skippable_overlay_pixel_assertions(tmp_path, marker):
    workflows = tmp_path / ".github" / "workflows"
    workflows.mkdir(parents=True)
    (workflows / "ci_ubuntu.yml").write_text(
        (ROOT / ".github/workflows/ci_ubuntu.yml").read_text()
    )
    (workflows / "ci_windows.yml").write_text(
        (ROOT / ".github/workflows/ci_windows.yml").read_text()
    )
    _write_visual_runtime_fixtures(tmp_path)
    path = tmp_path / "python/tests/unit/gui/test_agent_debug_overlay.py"
    path.write_text(
        (ROOT / "python/tests/unit/gui/test_agent_debug_overlay.py")
        .read_text()
        .replace(marker, "missing-overlay-marker", 1)
    )
    errors = []

    infra.check_ci_wiring(tmp_path, errors)

    assert any(
        "missing non-skippable overlay marker" in error and marker in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("relative", "marker", "expected"),
    (
        (
            "python/tests/unit/gui/test_agent_capture.py",
            "test_run_capture_smoke_writes_stills_and_sidecar",
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            "test_run_capture_debug_layers_change_pixels_end_to_end",
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            'factory="claim_capture_scene:make_world"',
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            "pytest.fail(str(error))",
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            'assert "--factory claim_capture_scene:make_world"',
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            'debug_layers = ["contacts", "collision_bounds", "labels"]',
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            'plain_artifact["camera"] == combined_artifact["camera"]',
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            "assert _changed_pixel_count(plain_image, combined_image) >= 128",
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            "agent_debug_overlay.CONTACT_POINT_RGB",
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            "assert contact_pixels >= 4",
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            "for layer in debug_layers",
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            'assert debug["layers"] == [layer]',
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            "assert _changed_pixel_count(plain_image, debug_image) >= 32",
            "missing non-skippable factory-capture marker",
        ),
        (
            "pixi.toml",
            "test-agent-debug-overlay =",
            "missing visual verification task marker",
        ),
        (
            "pixi.toml",
            "test_contacts_layer_marks_points_and_normals",
            "missing visual verification task marker",
        ),
        (
            "pixi.toml",
            "test_engine_rendered_overlay_changes_pixels",
            "missing visual verification task marker",
        ),
        (
            "pixi.toml",
            "test_run_capture_smoke_writes_stills_and_sidecar",
            "missing visual verification task marker",
        ),
        (
            "pixi.toml",
            "test_run_capture_debug_layers_change_pixels_end_to_end",
            "missing visual verification task marker",
        ),
    ),
)
def test_ci_wiring_requires_factory_capture_and_visual_task(
    tmp_path, relative, marker, expected
):
    workflows = tmp_path / ".github" / "workflows"
    workflows.mkdir(parents=True)
    (workflows / "ci_ubuntu.yml").write_text(
        (ROOT / ".github/workflows/ci_ubuntu.yml").read_text()
    )
    (workflows / "ci_windows.yml").write_text(
        (ROOT / ".github/workflows/ci_windows.yml").read_text()
    )
    _write_visual_runtime_fixtures(tmp_path)
    path = tmp_path / relative
    path.write_text(path.read_text().replace(marker, "missing-runtime-marker", 1))
    errors = []

    infra.check_ci_wiring(tmp_path, errors)

    assert any(expected in error and marker in error for error in errors)


@pytest.mark.parametrize(
    ("relative", "marker"),
    (
        (
            ".claude/hooks/pre-commit-guard.ps1",
            "$global:LASTEXITCODE = $null",
        ),
        (
            ".claude/hooks/pre-commit-guard.ps1",
            "$payload = [Console]::In.ReadToEnd()",
        ),
        (
            ".claude/hooks/pre-commit-guard.ps1",
            "$pipelineInput = @($input | ForEach-Object { $_ })",
        ),
        (
            ".claude/hooks/pre-commit-guard.ps1",
            "$pipelineInput.Count -gt 0",
        ),
        (
            ".claude/hooks/pre-commit-guard.ps1",
            "$OutputEncoding = New-Object System.Text.UTF8Encoding($false)",
        ),
        (
            "scripts/pretool_guard_bridge.py",
            'env["CLAUDE_PROJECT_DIR"] = str(root)',
        ),
        (
            "scripts/pretool_guard_bridge.py",
            "def may_invoke_git_commit(command: str) -> bool",
        ),
        (
            "scripts/pretool_guard_bridge.py",
            "if not may_invoke_git_commit(command):",
        ),
    ),
)
def test_windows_hook_components_cannot_be_missing_or_drifted(
    tmp_path, relative, marker
):
    for source in (
        ".codex/hooks.json",
        ".claude/settings.json",
        ".claude/hooks/pre-commit-guard.sh",
        ".claude/hooks/pre-commit-guard.ps1",
        "scripts/install_git_hooks.py",
        "scripts/pretool_guard_bridge.py",
    ):
        destination = tmp_path / source
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text((ROOT / source).read_text())
    path = tmp_path / relative
    original = path.read_text()
    path.unlink()
    missing_errors = []

    infra.check_hooks(tmp_path, missing_errors)

    assert any(relative in error for error in missing_errors)
    path.write_text(original.replace(marker, ""))
    drift_errors = []

    infra.check_hooks(tmp_path, drift_errors)

    assert any(marker in error for error in drift_errors)


def test_windows_launcher_does_not_shadow_native_exit_code():
    launcher = (ROOT / ".claude/hooks/pre-commit-guard.ps1").read_text()

    assert "$LASTEXITCODE = $null" not in launcher
    assert "$global:LASTEXITCODE = $null" in launcher
    assert "$nativeExitCode = $global:LASTEXITCODE" in launcher


@pytest.mark.parametrize("input_key", ("command", "cmd"))
def test_native_pretool_forwards_payload_to_shared_guard(
    tmp_path, monkeypatch, input_key
):
    guard = tmp_path / ".claude" / "hooks" / "pre-commit-guard.sh"
    guard.parent.mkdir(parents=True)
    guard.write_text("#!/bin/sh\nexit 0\n")
    bash = Path("C:/Program Files/Git/bin/bash.exe")
    calls = []
    monkeypatch.setattr(bridge, "find_git_bash", lambda: bash)
    monkeypatch.setattr(
        bridge.subprocess,
        "run",
        lambda args, **kwargs: calls.append((args, kwargs))
        or subprocess.CompletedProcess(args, 0, b"", b""),
    )
    payload = json.dumps({"tool_input": {input_key: "git commit -m x"}}).encode()

    assert bridge.forward(tmp_path, payload) == 0
    assert calls[0][0] == [
        str(bash),
        "--noprofile",
        "--norc",
        str(guard),
    ]
    assert calls[0][1]["input"] == payload
    assert calls[0][1]["cwd"] == tmp_path
    assert calls[0][1]["env"]["CLAUDE_PROJECT_DIR"] == str(tmp_path)
    assert calls[0][1]["env"]["CODEX_PROJECT_DIR"] == str(tmp_path)
    assert calls[0][1]["env"]["DART_HOOK_PYTHON"] == str(
        Path(sys.executable).resolve()
    ).replace("\\", "/")


@pytest.mark.parametrize("command", ("git status", "echo hello"))
def test_native_pretool_noncommit_bypasses_missing_git_bash(
    tmp_path, monkeypatch, command
):
    monkeypatch.setattr(bridge, "find_git_bash", lambda: None)
    payload = json.dumps({"tool_input": {"command": command}}).encode()

    assert bridge.forward(tmp_path, payload) == 0


@pytest.mark.parametrize(
    "command",
    (
        "GIT COMMIT -m x",
        "g---i---t\nc---o---m---m---i---t",
    ),
)
def test_native_pretool_commit_without_git_bash_fails_closed(
    tmp_path, monkeypatch, capsys, command
):
    monkeypatch.setattr(bridge, "find_git_bash", lambda: None)
    payload = json.dumps({"tool_input": {"command": command}}).encode()

    assert bridge.forward(tmp_path, payload) == 2
    assert "Git Bash is required" in capsys.readouterr().err


def test_native_pretool_maps_guard_failure_to_codex_block(
    tmp_path, monkeypatch, capsys
):
    guard = tmp_path / ".claude" / "hooks" / "pre-commit-guard.sh"
    guard.parent.mkdir(parents=True)
    guard.write_text("#!/bin/sh\nexit 2\n")
    monkeypatch.setattr(bridge, "find_git_bash", lambda: Path("/test/git-bash"))
    monkeypatch.setattr(
        bridge.subprocess,
        "run",
        lambda args, **kwargs: subprocess.CompletedProcess(
            args, 2, b"guard output\n", b"guard error\n"
        ),
    )
    payload = json.dumps({"tool_input": {"command": "git commit"}}).encode()

    assert bridge.forward(tmp_path, payload) == 2
    captured = capsys.readouterr()
    assert "guard output" in captured.err
    assert "guard error" in captured.err


def test_native_pretool_rejects_malformed_payload(tmp_path):
    assert bridge.forward(tmp_path, b"{}") == 2


@pytest.mark.parametrize(
    ("updated", "expected"),
    (
        (b"updated\r\n", 0),
        (b"updated \r\n", 2),
    ),
)
def test_staged_diff_check_accepts_crlf_but_rejects_trailing_space(
    tmp_path, updated, expected
):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.email", "test@example.com"],
        check=True,
    )
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.name", "DART Test"],
        check=True,
    )
    path = tmp_path / "legacy.txt"
    path.write_bytes(b"base\r\n")
    subprocess.run(["git", "-C", str(tmp_path), "add", "legacy.txt"], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "commit", "-q", "-m", "base"], check=True
    )
    path.write_bytes(updated)
    subprocess.run(["git", "-C", str(tmp_path), "add", "legacy.txt"], check=True)

    assert hook.run_staged(tmp_path) == expected


def test_mixed_staged_and_unstaged_ai_file_is_rejected(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.email", "test@example.com"],
        check=True,
    )
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.name", "DART Test"],
        check=True,
    )
    path = tmp_path / "docs" / "ai" / "example.md"
    path.parent.mkdir(parents=True)
    path.write_text("base\n")
    subprocess.run(["git", "-C", str(tmp_path), "add", "."], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "commit", "-q", "-m", "base"], check=True
    )

    path.write_text("staged\n")
    subprocess.run(
        ["git", "-C", str(tmp_path), "add", "docs/ai/example.md"], check=True
    )
    path.write_text("unstaged final\n")

    assert hook.run_staged(tmp_path) == 2


def test_staged_source_cannot_be_hidden_by_unstaged_generated_output(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.email", "test@example.com"],
        check=True,
    )
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.name", "DART Test"],
        check=True,
    )
    source = tmp_path / ".claude" / "skills" / "dart-example" / "SKILL.md"
    generated = tmp_path / ".agents" / "skills" / "dart-example" / "SKILL.md"
    source.parent.mkdir(parents=True)
    generated.parent.mkdir(parents=True)
    source.write_text("base source\n")
    generated.write_text("base generated\n")
    subprocess.run(["git", "-C", str(tmp_path), "add", "."], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "commit", "-q", "-m", "base"], check=True
    )

    source.write_text("staged source\n")
    subprocess.run(
        [
            "git",
            "-C",
            str(tmp_path),
            "add",
            ".claude/skills/dart-example/SKILL.md",
        ],
        check=True,
    )
    generated.write_text("unstaged generated fix\n")

    assert hook.run_staged(tmp_path) == 2


def test_staged_ai_deletion_runs_infrastructure_checks(tmp_path, monkeypatch):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.email", "test@example.com"],
        check=True,
    )
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.name", "DART Test"],
        check=True,
    )
    path = tmp_path / "docs" / "ai" / "example.md"
    path.parent.mkdir(parents=True)
    path.write_text("base\n")
    subprocess.run(["git", "-C", str(tmp_path), "add", "."], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "commit", "-q", "-m", "base"], check=True
    )
    path.unlink()
    subprocess.run(["git", "-C", str(tmp_path), "add", "-u"], check=True)
    calls = []

    def record(command, root, env=None):
        calls.append((command, root))
        return 0

    monkeypatch.setattr(hook, "run_checked", record)

    assert hook.run_staged(tmp_path) == 0
    assert len(calls) == 2


def test_staged_ai_deletion_with_untracked_replacement_is_rejected(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.email", "test@example.com"],
        check=True,
    )
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.name", "DART Test"],
        check=True,
    )
    path = tmp_path / "docs" / "ai" / "example.md"
    path.parent.mkdir(parents=True)
    path.write_text("base\n")
    subprocess.run(["git", "-C", str(tmp_path), "add", "."], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "commit", "-q", "-m", "base"], check=True
    )
    subprocess.run(
        ["git", "-C", str(tmp_path), "rm", "-q", "docs/ai/example.md"], check=True
    )
    path.parent.mkdir(parents=True)
    path.write_text("untracked replacement\n")

    assert hook.run_staged(tmp_path) == 2


def test_staged_hook_binds_source_repo_for_task_evidence_policy(tmp_path, monkeypatch):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    policy = tmp_path / "docs" / "dev_tasks" / ".gitignore"
    task = tmp_path / "docs" / "dev_tasks" / "example"
    task.mkdir(parents=True)
    policy.write_text("**/assets/\n", encoding="utf-8")
    (task / "README.md").write_text("Local evidence only.\n", encoding="utf-8")
    subprocess.run(["git", "-C", str(tmp_path), "add", "."], check=True)
    policy_results = []

    def run_policy(command, snapshot, env=None):
        if command[1].endswith("sync_ai_commands.py"):
            assert env is None
            return 0
        assert command[1].endswith("check_ai_infrastructure.py")
        assert env is not None
        assert Path(env["GIT_DIR"]).is_dir()
        assert Path(env["GIT_WORK_TREE"]).resolve() == snapshot.resolve()
        with monkeypatch.context() as git_environment:
            git_environment.setenv("GIT_DIR", env["GIT_DIR"])
            git_environment.setenv("GIT_WORK_TREE", env["GIT_WORK_TREE"])
            git_environment.delenv("GIT_PREFIX", raising=False)
            if "GIT_INDEX_FILE" in env:
                git_environment.setenv("GIT_INDEX_FILE", env["GIT_INDEX_FILE"])
            errors = infra.dev_task_evidence_policy_errors(snapshot)
        policy_results.append(errors)
        return 2 if errors else 0

    monkeypatch.setattr(hook, "run_checked", run_policy)

    assert hook.run_staged(tmp_path) == 0
    assert policy_results[-1] == []

    asset = task / "assets" / "raw.csv"
    asset.parent.mkdir(parents=True)
    asset.write_text("raw\n", encoding="utf-8")
    subprocess.run(
        ["git", "-C", str(tmp_path), "add", "-f", "docs/dev_tasks/example/assets"],
        check=True,
    )

    assert hook.run_staged(tmp_path) == 2
    assert any(
        "generated task evidence is tracked" in error for error in policy_results[-1]
    )


def test_staged_repository_environment_rejects_non_git_source(tmp_path):
    source = tmp_path / "source"
    snapshot = tmp_path / "snapshot"
    source.mkdir()
    snapshot.mkdir()

    with pytest.raises(RuntimeError, match="source repository"):
        hook.staged_repository_environment(source, snapshot)


def test_staged_rename_out_of_ai_scope_still_runs_checks(tmp_path, monkeypatch):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.email", "test@example.com"],
        check=True,
    )
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.name", "DART Test"],
        check=True,
    )
    source = tmp_path / "docs" / "ai" / "example.md"
    source.parent.mkdir(parents=True)
    source.write_text("base\n")
    subprocess.run(["git", "-C", str(tmp_path), "add", "."], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "commit", "-q", "-m", "base"], check=True
    )
    (tmp_path / "misc").mkdir()
    subprocess.run(
        [
            "git",
            "-C",
            str(tmp_path),
            "mv",
            "docs/ai/example.md",
            "misc/example.md",
        ],
        check=True,
    )
    calls = []

    def record(command, root, env=None):
        calls.append((command, root))
        return 0

    monkeypatch.setattr(hook, "run_checked", record)

    assert hook.run_staged(tmp_path) == 0
    assert len(calls) == 2


def test_unstaged_rename_out_of_ai_scope_blocks_other_staged_ai_work(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.email", "test@example.com"],
        check=True,
    )
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.name", "DART Test"],
        check=True,
    )
    first = tmp_path / "docs" / "ai" / "first.md"
    second = tmp_path / "docs" / "ai" / "second.md"
    first.parent.mkdir(parents=True)
    first.write_text("first\n")
    second.write_text("second\n")
    subprocess.run(["git", "-C", str(tmp_path), "add", "."], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "commit", "-q", "-m", "base"], check=True
    )
    second.write_text("staged second\n")
    subprocess.run(["git", "-C", str(tmp_path), "add", "docs/ai/second.md"], check=True)
    (tmp_path / "misc").mkdir()
    first.rename(tmp_path / "misc" / "first.md")

    assert hook.run_staged(tmp_path) == 2
