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


def test_release_scenarios_are_exercisable():
    assert infra.exercise_scenarios(ROOT) == []


def test_simulation_scenario_routes_text_first_and_visual_policy():
    scenarios = {scenario["id"]: scenario for scenario in _scenario_data()["scenarios"]}
    scenario = scenarios["simulation-verification"]

    assert len(scenarios) == 7
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
        ("docs/ai/verification.md", "required renderer is unavailable"),
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

    assert any(relative in error and "simulation route marker" in error for error in errors)


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


def test_agent_hook_path_routing_is_bounded():
    assert hook.is_ai_infrastructure_path("docs/ai/README.md")
    assert hook.is_ai_infrastructure_path("docs/onboarding/architecture.md")
    assert hook.is_ai_infrastructure_path("docs/onboarding/testing.md")
    assert hook.is_ai_infrastructure_path(".codex/agents/dart_scout.toml")
    assert hook.is_ai_infrastructure_path(".github/workflows/ci_macos.yml")
    assert hook.is_ai_infrastructure_path(
        "python/tests/unit/gui/test_agent_capture.py"
    )
    assert hook.is_ai_infrastructure_path(
        "python/tests/unit/gui/test_agent_debug_overlay.py"
    )
    assert hook.is_ai_infrastructure_path(".gitignore")
    assert hook.is_ai_infrastructure_path("pixi.toml")
    assert hook.is_ai_infrastructure_path("dart/new_module/AGENTS.md")
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


def test_expected_agent_set_is_small_and_release_specific():
    assert infra.EXPECTED_AGENTS == {
        "dart_release_auditor",
        "dart_reviewer",
        "dart_scout",
    }


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
        "scripts/check_ai_infrastructure.py --check\n"
        "tests/test_sync_ai_commands.py\n"
        "scripts/check_ai_infrastructure.py --scenarios\n"
        "      - name: Agent visual verification smoke\n"
        "if: matrix.build_type == 'Release'\n"
        "xvfb-run\n"
        "pixi run agent-capture\n"
        "--scene box_on_ground --steps 250 --focus box --auto-views 1\n"
        "--layers contacts collision_bounds labels\n"
        "--width 320 --height 240\n"
        "--out /tmp/dart-agent-visual-smoke --prefix smoke\n"
        "pixi run image-verdict\n"
            "/tmp/dart-agent-visual-smoke/smoke_auto0.png\n"
            "xvfb-run\n"
            "pixi run test-agent-debug-overlay\n"
    )
    (workflows / "ci_windows.yml").write_text(
        "Native Windows hook smoke\n"
        'pixi run python -c "import sys; print(sys.executable)"\n'
        "$launcher\n"
        "$hookCommand\n"
        "$input | &\n"
        '$ErrorActionPreference = "Continue"\n'
        '"git status"\n'
        "DART_HOOK_DRY_RUN\n"
        '"git commit --no-verify -m x"\n'
        "$diagnosticOutput\n"
        "check_agent_hook.py --profile staged\n"
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


@pytest.mark.parametrize(
    "marker",
    (
        "Agent visual verification smoke",
        "xvfb-run",
        "pixi run agent-capture",
        "--scene box_on_ground --steps 250 --focus box --auto-views 1",
        "--layers contacts collision_bounds labels",
        "--width 320 --height 240",
        "--out /tmp/dart-agent-visual-smoke --prefix smoke",
        "pixi run image-verdict",
        "/tmp/dart-agent-visual-smoke/smoke_auto0.png",
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
        mutated = before + separator + visual_and_after.replace(
            marker, "missing-visual-marker", 1
        )
    (workflows / "ci_ubuntu.yml").write_text(
        mutated
    )
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
def test_visual_smoke_cannot_be_skipped_or_soft_failed(
    tmp_path, mutation, expected
):
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
def test_ci_wiring_requires_non_skippable_overlay_pixel_assertions(
    tmp_path, marker
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
            "$pipelineInput = @($input)",
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

    def record(command, root):
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

    def record(command, root):
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
