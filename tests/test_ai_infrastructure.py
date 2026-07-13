"""Focused tests for the read-only DART AI-infrastructure checks."""

from __future__ import annotations

import ast
import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS))

import ai_doctor  # noqa: E402
import ai_infrastructure as infra  # noqa: E402
import install_git_hooks  # noqa: E402
import lint_cmake  # noqa: E402
import lint_toml  # noqa: E402
import sync_ai_commands as sync  # noqa: E402


@pytest.mark.parametrize(
    "script_name",
    ("ai_doctor.py", "ai_infrastructure.py", "sync_ai_commands.py"),
)
def test_ai_runtime_scripts_support_python_3_11_syntax(script_name):
    path = SCRIPTS / script_name
    ast.parse(
        path.read_text(encoding="utf-8"),
        filename=str(path),
        feature_version=11,
    )


def _write(root: Path, relative: str, text: str = "fixture\n") -> Path:
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    return path


def test_cmake_lint_ignores_tool_managed_claude_worktrees(tmp_path):
    _write(tmp_path, "CMakeLists.txt", "project(dart_fixture)\n")
    _write(tmp_path, ".claude/hooks/CMakeLists.txt", "project(hook_fixture)\n")
    _write(
        tmp_path,
        ".claude/worktrees/topic/tools/freebsd/patches/patch-CMakeLists.cmake",
        "--- CMakeLists.txt.orig\n",
    )

    assert lint_cmake.find_cmake_files(tmp_path) == [
        Path(".claude/hooks/CMakeLists.txt"),
        Path("CMakeLists.txt"),
    ]


def test_toml_lint_ignores_tool_managed_claude_worktrees(tmp_path):
    _write(tmp_path, "pixi.toml", "[project]\nname = 'dart-fixture'\n")
    _write(
        tmp_path,
        ".claude/agents/reviewer.toml",
        "name = 'tracked-reviewer'\n",
    )
    _write(
        tmp_path,
        ".claude/worktrees/topic/pixi.toml",
        "[project]\nname = 'nested-topic'\n",
    )

    assert lint_toml.find_toml_files(tmp_path) == [
        Path(".claude/agents/reviewer.toml"),
        Path("pixi.toml"),
    ]


def test_pixi_prettier_tasks_exclude_tool_managed_claude_worktrees(tmp_path):
    pixi = (ROOT / "pixi.toml").read_text()
    (tmp_path / "pixi.toml").write_text(
        pixi.replace("!**/.claude/worktrees/**", "missing-worktree-exclusion", 1)
    )

    errors = infra.check_pixi_references(tmp_path, "main")

    assert any("must exclude .claude/worktrees/**" in error for error in errors)


def _agent_text(name: str) -> str:
    return f'''\
name = "{name}"
description = "Bounded read-only fixture agent."
sandbox_mode = "read-only"
developer_instructions = """
Read-only. Input is explicit. Output cites evidence. Never mutate files.
"""
'''


def _skill_text(name: str) -> str:
    markers = ""
    if name == "dart-verify-sim":
        markers = """
text oracle
agent-capture
image-verdict
sole correctness oracle
model/scene loading
collision/contact/constraints
stepping, GUI/rendering
GUI/rendering
visual examples
rendering is unavailable
replacement evidence
bm-boxes-headless
Xvfb
--factory module:callable
"""
    return f"""\
---
name: {name}
description: "DART Fixture: bounded test skill"
---

# {name}
{markers}
"""


def _profile_data(profile: str) -> dict:
    fingerprint = infra.PROFILE_FINGERPRINTS[profile]
    if profile == "main":
        markers = ["C++23", "DART 7"]
        required_paths = [
            "docs/readthedocs/architecture.md",
            "docs/design/simulation_solver_architecture.md",
            ".claude/skills/dart-architecture/SKILL.md",
        ]
        forbidden_markers = ["C++17 compatibility profile"]
        forbidden_paths = [".claude/commands/dart-release-ci-fix.md"]
    else:
        markers = ["release-6.20", "DART 6.20"]
        required_paths = [
            "docs/onboarding/release-management.md",
            ".claude/commands/dart-release-ci-fix.md",
            ".claude/commands/dart-backport-pr.md",
        ]
        forbidden_markers = ["DART 7 C++23 clean-break profile"]
        forbidden_paths = [".claude/skills/dart-architecture/SKILL.md"]
    downstream_gates = [
        (
            "pixi run -e cuda test-all"
            if profile == "main"
            else "pixi run -e gazebo test-gz"
        )
    ]
    return {
        "schema_version": 1,
        "profile": profile,
        **fingerprint,
        "required_markers": markers,
        "required_paths": required_paths,
        "forbidden_markers": forbidden_markers,
        "forbidden_paths": forbidden_paths,
        "downstream_gates": downstream_gates,
    }


def _scenario_data(profile: str) -> dict:
    data = json.loads((ROOT / infra.SCENARIO_MANIFEST).read_text())
    if profile == "main":
        return data
    data["profile"] = profile
    scenarios = {scenario["id"]: scenario for scenario in data["scenarios"]}
    forbidden = [".claude/skills/dart-architecture/SKILL.md"]
    for scenario in scenarios.values():
        scenario["forbidden_paths"] = forbidden
    scenarios["orientation"]["prompt_class"] = "read-only DART 6.20 orientation"
    scenarios["small-change"].update(
        {
            "prompt_class": "one bounded DART 6 compatibility change",
            "start_dir": "dart",
            "instruction_chain": ["AGENTS.md"],
            "owner_docs": ["CONTRIBUTING.md", "docs/onboarding/contributing.md"],
        }
    )
    scenarios["failure-diagnosis"].update(
        {
            "prompt_class": "root-cause a failing release CI check",
            "expected_route": {
                "kind": "workflow",
                "name": "dart-release-ci-fix",
                "path": ".agents/skills/dart-release-ci-fix/SKILL.md",
            },
            "owner_docs": [
                "docs/onboarding/ci-cd.md",
                "docs/onboarding/release-management.md",
            ],
        }
    )
    scenarios["component-work"].update(
        {
            "prompt_class": "DART 6 compatibility component work",
            "instruction_chain": ["AGENTS.md"],
            "expected_route": {
                "kind": "domain_skill",
                "name": "dart-build",
                "path": ".agents/skills/dart-build/SKILL.md",
            },
            "owner_docs": [
                "docs/onboarding/building.md",
                "docs/onboarding/testing.md",
            ],
        }
    )
    scenarios["simulation-verification"].update(
        {
            "prompt_class": "verify DART 6 behavior with text and OSG evidence",
            "start_dir": "dart",
            "instruction_chain": ["AGENTS.md"],
            "owner_docs": [
                "docs/onboarding/testing.md",
                "docs/ai/verification.md",
            ],
            "focused_gates": ["pixi run test"],
            "full_gates": ["pixi run test-all"],
        }
    )
    return data


def _generate_adapters(root: Path) -> None:
    commands = sorted((root / ".claude" / "commands").glob("*.md"))
    skills = sorted((root / ".claude" / "skills").glob("*/SKILL.md"))
    names = {path.stem for path in commands} | {path.parent.name for path in skills}
    for source in commands:
        relative = str(source.relative_to(root))
        _write(
            root,
            f".opencode/command/{source.name}",
            sync.add_auto_gen_header(source.read_text(), relative),
        )
        _write(
            root,
            f".agents/skills/{source.stem}/SKILL.md",
            sync.add_auto_gen_header(sync.render_codex_command_skill(source), relative),
        )
    for source in skills:
        relative = str(source.relative_to(root))
        _write(
            root,
            f".agents/skills/{source.parent.name}/SKILL.md",
            sync.add_auto_gen_header(source.read_text(), relative),
        )
    _write(
        root,
        ".agents/skills/.dart-generated.json",
        json.dumps(
            {
                "schema_version": 1,
                "generator": "scripts/sync_ai_commands.py",
                "paths": [f"{name}/SKILL.md" for name in sorted(names)],
            }
        )
        + "\n",
    )


def make_repo(tmp_path: Path, profile: str) -> Path:
    root = tmp_path / profile.replace(".", "-")
    root.mkdir()
    subprocess.run(["git", "init", "-q", str(root)], check=True)
    marker = (
        "DART 7 C++23 repository instructions\n"
        if profile == "main"
        else "DART 6.20 release-6.20 repository instructions\n"
    )
    _write(root, "AGENTS.md", marker)
    _write(root, ".codex/AGENTS.md", "Codex scoped instructions\n")
    profile_data = _profile_data(profile)
    _write(root, infra.BRANCH_PROFILE, json.dumps(profile_data))
    manifest_text = json.dumps(_scenario_data(profile))
    _write(root, infra.SCENARIO_MANIFEST, manifest_text)
    _write(
        root,
        ".codex/config.toml",
        "[agents]\nmax_threads = 4\nmax_depth = 1\n",
    )
    _write(
        root,
        ".codex/hooks.json",
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
                                    "commandWindows": (
                                        infra.CODEX_HOOK_COMMAND_WINDOWS
                                    ),
                                    "timeout": 10,
                                    "statusMessage": infra.CODEX_HOOK_STATUS,
                                }
                            ],
                        }
                    ]
                }
            }
        ),
    )
    _write(
        root,
        ".claude/settings.json",
        json.dumps(
            {
                "hooks": {
                    "PreToolUse": [
                        {
                            "matcher": "Bash",
                            "hooks": [
                                {
                                    "type": "command",
                                    "command": infra.CLAUDE_HOOK_COMMAND,
                                    "shell": "bash",
                                }
                            ],
                        }
                    ]
                }
            }
        ),
    )
    _write(root, ".claude/hooks/pre-commit-guard.sh", "#!/bin/sh\nexit 0\n")
    _write(
        root,
        ".claude/hooks/pre-commit-guard.ps1",
        "\n".join(infra.WINDOWS_LAUNCHER_MARKERS),
    )
    _write(
        root,
        "scripts/pretool_guard_bridge.py",
        "\n".join(infra.WINDOWS_BRIDGE_MARKERS),
    )
    _write(
        root,
        ".github/workflows/ci_lint.yml",
        "- run: pixi run check-lint\n- run: pixi run test-ai-infra\n",
    )
    _write(
        root,
        ".github/workflows/ci_windows.yml",
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
        "$blockedExit -ne 2\n"
        "-File $launcher\n"
        "$rawSuccessExit\n"
        "$rawBlockedExit\n"
        "cmd.exe /c exit 0\n",
    )
    _write(
        root,
        ".github/workflows/ci_ubuntu.yml",
        "packages: xvfb\n"
        "- run: pixi run test-py\n"
        "      - name: Agent visual verification smoke\n"
        "xvfb-run\n"
        "pixi run agent-capture\n"
        "--scene box_on_ground --steps 250 --focus box --auto-views 1\n"
        "--layers contacts collision_bounds labels\n"
        "--width 320 --height 240\n"
        "--out /tmp/dart-agent-visual-smoke --prefix smoke\n"
        "pixi run image-verdict\n"
        "/tmp/dart-agent-visual-smoke/smoke_auto0.png\n"
        "xvfb-run -a -s '-screen 0 640x480x24' \\\n"
        "            env DART_REQUIRE_VISUAL_E2E=1 "
        "pixi run test-agent-visual-e2e\n"
        "  filament-gui-smoke:\n",
    )
    _write(
        root,
        "python/tests/unit/test_agent_capture.py",
        "def test_run_capture_smoke_writes_stills_and_sidecar():\n"
        '    layers=["contacts", "labels"]\n'
        "test_run_capture_debug_layers_change_pixels_end_to_end\n"
        'debug_layers = ["contacts", "collision_bounds", "labels"]\n'
        "for layer in debug_layers\n"
        "assert _changed_pixel_count(plain_image, combined_image) >= 128\n"
        "assert _changed_pixel_count(plain_image, debug_image) >= 32\n"
        "DART_REQUIRE_VISUAL_E2E\n"
        "dartpy.gui.OffscreenRenderer is unavailable\n"
        'pytest.fail("visual e2e is required but DISPLAY is unavailable")\n',
    )
    _write(
        root,
        "python/tests/unit/gui/test_offscreen_render.py",
        "test_debug_scene_overlay_lifecycle_on_shared_renderer\n"
        'layers=("grid", "body_frames", "contacts")\n'
        "_bytes_over_tolerance(debugged, plain) >= 256\n"
        "_bytes_over_tolerance(emptied, plain) <= 64\n",
    )
    for name in infra.AGENT_NAMES:
        _write(root, f".codex/agents/{name}.toml", _agent_text(name))
    for path in profile_data["required_paths"]:
        if not (root / path).exists():
            _write(root, path)
    manifest = json.loads(manifest_text)
    tasks = {"noop", "test-agent-visual-e2e"}
    for gate in profile_data["downstream_gates"]:
        match = infra.PIXIRUN_RE.fullmatch(gate)
        assert match
        tasks.add(match.group("task"))
    for scenario in manifest["scenarios"]:
        start_dir = root / scenario["start_dir"]
        start_dir.mkdir(parents=True, exist_ok=True)
        for agents_path in scenario["instruction_chain"]:
            if not (root / agents_path).exists():
                _write(root, agents_path)
        for owner_path in scenario["owner_docs"]:
            if not (root / owner_path).exists():
                _write(root, owner_path)
        kind = scenario["expected_route"]["kind"]
        route = scenario["expected_route"]["name"]
        if kind == "workflow":
            source = root / ".claude" / "commands" / f"{route}.md"
            route_marker = (
                "\nroute through `dart-verify-sim`\n"
                if route == "dart-new-task"
                else ""
            )
            _write(
                root,
                str(source.relative_to(root)),
                f"---\ndescription: {route} fixture\n---\n\n# {route}\n"
                f"{route_marker}",
            )
        elif kind == "domain_skill":
            source = root / ".claude" / "skills" / route / "SKILL.md"
            _write(root, str(source.relative_to(root)), _skill_text(route))
        recovery = scenario["recovery"].partition("#")[0]
        if not (root / recovery).exists():
            _write(root, recovery)
        for gate in (*scenario["focused_gates"], *scenario["full_gates"]):
            match = infra.PIXIRUN_RE.fullmatch(gate)
            assert match
            tasks.add(match.group("task"))
    task_lines = "\n".join(f'{task} = {{ cmd = "true" }}' for task in sorted(tasks))
    task_lines = task_lines.replace(
        'test-agent-visual-e2e = { cmd = "true" }',
        "test-agent-visual-e2e = { cmd = "
        '"test_run_capture_debug_layers_change_pixels_end_to_end" }',
    )
    _write(root, "pixi.toml", f"[tasks]\n{task_lines}\n")

    consumer_markers = {
        "AGENTS.md": "docs/onboarding/agent-sim-verification.md\ndart-verify-sim\n",
        ".codex/agents/dart_scout.toml": (
            "collision/contact/constraints\ntext correctness oracle\n"
            "claim-tied assessed visual\n"
        ),
        ".codex/agents/dart_reviewer.toml": (
            "text-first evidence\nvisual/debug-layer\n"
        ),
        ".claude/commands/dart-ultrawork.md": ("routes through `dart-verify-sim`\n"),
        ".claude/commands/dart-resume.md": "route through `dart-verify-sim`\n",
        ".claude/commands/dart-pr.md": "use `dart-verify-sim`\n",
        ".claude/commands/dart-manage-pr.md": "Visual verification\n",
        ".claude/commands/dart-review-pr.md": (
            "require the `dart-verify-sim` text oracle\n"
            "accepting a screenshot alone\n"
        ),
        ".claude/commands/dart-new-task.md": ("record why it is not applicable\n"),
        ".claude/skills/dart-build/SKILL.md": "dart-verify-sim\n",
        ".claude/skills/dart-test/SKILL.md": (
            "load `dart-verify-sim`\nunavailable or not applicable\n"
        ),
        ".claude/skills/dart-io/SKILL.md": (
            "also load `dart-verify-sim`\nclaim-tied visual corroboration\n"
        ),
        ".claude/skills/dart-python/SKILL.md": (
            "also load `dart-verify-sim`\ncollision/contact/constraints\n"
            "GUI/rendering output\nvisual exception\n"
        ),
        ".claude/skills/dart-ci/SKILL.md": (
            "also load `dart-verify-sim`\nvisual exception\n"
        ),
        ".claude/commands/dart-downstream-fix.md": (
            "route through `dart-verify-sim`\nvisual exception\n"
        ),
        ".claude/commands/dart-backport-pr.md": (
            "target branch's `dart-verify-sim`\nvisual exception\n"
        ),
        ".claude/commands/dart-fix-ci.md": (
            "use `dart-verify-sim`\nvisual exception\n"
        ),
        ".claude/commands/dart-fix-issue.md": (
            "route through `dart-verify-sim`\ncollision/contact/constraints\n"
            "visual exception\n"
        ),
        "docs/ai/verification.md": (
            "agent-capture\ntext correctness\nIf rendering is unavailable\n"
            "name replacement evidence\n"
        ),
    }
    for relative, markers in consumer_markers.items():
        path = root / relative
        if path.exists():
            content = path.read_text()
        elif relative.startswith(".claude/commands/"):
            name = path.stem
            content = f"---\ndescription: {name} fixture\n---\n\n# {name}\n"
        elif relative.startswith(".claude/skills/"):
            content = _skill_text(path.parent.name)
        else:
            content = "fixture\n"
        if relative.startswith(".codex/agents/"):
            content = content.replace('\n"""\n', f'\n{markers}\n"""\n')
        else:
            content += markers
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    _generate_adapters(root)
    return root


@pytest.mark.parametrize("profile", infra.PROFILES)
def test_minimal_branch_profiles_pass(tmp_path, profile):
    root = make_repo(tmp_path, profile)

    assert infra.run_checks(root, profile) == []


def test_static_infrastructure_check_stays_within_hook_budget(tmp_path):
    root = make_repo(tmp_path, "main")

    started = time.monotonic()
    errors = infra.run_checks(root, "main")

    assert errors == []
    assert time.monotonic() - started < 2


def test_auto_profile_uses_manifest_for_topic_or_detached_worktree(tmp_path):
    root = make_repo(tmp_path, "release-6.20")

    assert infra.detect_profile(root, "auto") == "release-6.20"


def test_current_main_contract_passes():
    assert infra.run_checks(ROOT, "main") == []


@pytest.mark.parametrize(
    ("profile", "component", "failure", "release"),
    [
        ("main", "dart-architecture", "dart-fix-ci", "dart-backport-pr"),
        ("release-6.20", "dart-build", "dart-release-ci-fix", "dart-backport-pr"),
    ],
)
def test_seven_scenarios_route_deterministically(
    tmp_path, profile, component, failure, release
):
    root = make_repo(tmp_path, profile)

    results = infra.scenario_results(root, profile)

    assert len(results) == 7
    assert [result["id"] for result in results] == list(infra.SCENARIO_IDS)
    routes = {result["id"]: result["expected_route"]["name"] for result in results}
    assert routes["component-work"] == component
    assert routes["simulation-verification"] == "dart-verify-sim"
    assert routes["failure-diagnosis"] == failure
    assert routes["release-maintenance"] == release
    assert all(result["valid"] for result in results)


@pytest.mark.parametrize(
    ("field", "value", "expected"),
    [
        ("prompt_class", "", "prompt_class must be non-empty"),
        ("start_dir", "missing", "start_dir does not exist"),
        (
            "instruction_chain",
            ["AGENTS.md", "missing/AGENTS.md"],
            "AGENTS chain mismatch",
        ),
        ("owner_docs", ["docs/missing.md"], "owner path is missing"),
        (
            "expected_route",
            {
                "kind": "workflow",
                "name": "dart-missing-command",
                "path": ".agents/skills/dart-missing-command/SKILL.md",
            },
            "workflow route is missing",
        ),
        (
            "expected_route",
            {
                "kind": "agent",
                "name": "dart_scout",
                "path": ".codex/agents/wrong.toml",
            },
            "expected_route path must equal",
        ),
        ("specialist_agent", "unknown", "unknown specialist_agent"),
        ("permitted_scopes", ["**"], "unbounded entry"),
        (
            "focused_gates",
            ["pixi run missing-task"],
            "focused_gates references missing Pixi task",
        ),
        ("full_gates", [], "full_gates must be a non-empty list"),
        ("recovery", "docs/missing.md", "recovery pointer is missing"),
        (
            "forbidden_paths",
            ["AGENTS.md"],
            "forbidden branch path is present",
        ),
    ],
)
def test_scenario_contract_field_mutations_fail(tmp_path, field, value, expected):
    root = make_repo(tmp_path, "main")
    manifest_path = root / infra.SCENARIO_MANIFEST
    manifest = json.loads(manifest_path.read_text())
    manifest["scenarios"][0][field] = value
    manifest_path.write_text(json.dumps(manifest))

    _, errors = infra.validate_scenarios(root, "main")

    assert any(expected in error for error in errors)


def test_scenario_order_mutation_fails(tmp_path):
    root = make_repo(tmp_path, "main")
    manifest_path = root / infra.SCENARIO_MANIFEST
    manifest = json.loads(manifest_path.read_text())
    scenarios = manifest["scenarios"]
    scenarios[0], scenarios[1] = scenarios[1], scenarios[0]
    manifest_path.write_text(json.dumps(manifest))

    _, errors = infra.validate_scenarios(root, "main")

    assert any("scenario ids/order" in error for error in errors)


def test_scenario_manifest_rejects_boolean_schema_version(tmp_path):
    root = make_repo(tmp_path, "main")
    manifest_path = root / infra.SCENARIO_MANIFEST
    manifest = json.loads(manifest_path.read_text())
    manifest["schema_version"] = True
    manifest_path.write_text(json.dumps(manifest))

    _, errors = infra.validate_scenarios(root, "main")

    assert any("schema_version must equal 1" in error for error in errors)


@pytest.mark.parametrize(
    ("field", "value", "expected"),
    [
        ("profile", "release-6.20", "profile must equal 'main'"),
        ("cpp_standard", "C++17", "cpp_standard must equal 'C++23'"),
        ("required_markers", ["missing"], "missing marker 'missing'"),
        ("required_paths", ["docs/missing.md"], "missing branch-owned path"),
        ("forbidden_markers", ["DART 7"], "contains forbidden marker"),
        ("forbidden_paths", ["AGENTS.md"], "forbidden branch path is present"),
        (
            "downstream_gates",
            ["pixi run missing-task"],
            "references missing Pixi task 'missing-task'",
        ),
    ],
)
def test_branch_profile_field_mutations_fail(tmp_path, field, value, expected):
    root = make_repo(tmp_path, "main")
    profile_path = root / infra.BRANCH_PROFILE
    profile = json.loads(profile_path.read_text())
    profile[field] = value
    profile_path.write_text(json.dumps(profile))

    errors = infra.check_branch_profile(root, "main")

    assert any(expected in error for error in errors)


def test_branch_profile_rejects_boolean_schema_version(tmp_path):
    root = make_repo(tmp_path, "main")
    profile_path = root / infra.BRANCH_PROFILE
    profile = json.loads(profile_path.read_text())
    profile["schema_version"] = True
    profile_path.write_text(json.dumps(profile))

    errors = infra.check_branch_profile(root, "main")

    assert any("schema_version must equal 1" in error for error in errors)


def test_branch_profile_rejects_external_paths_and_malformed_lists(tmp_path):
    root = make_repo(tmp_path, "main")
    profile_path = root / infra.BRANCH_PROFILE
    profile = json.loads(profile_path.read_text())
    profile["required_paths"] = [str(tmp_path / "outside.md")]
    profile["downstream_gates"] = 7
    profile_path.write_text(json.dumps(profile))

    errors = infra.check_branch_profile(root, "main")

    assert any("required path escapes repository" in error for error in errors)
    assert any(
        "downstream_gates must be a non-empty string list" in error for error in errors
    )


def test_scenario_rejects_external_owner_recovery_and_scope_paths(tmp_path):
    root = make_repo(tmp_path, "main")
    outside = tmp_path / "outside.md"
    outside.write_text("outside\n")
    manifest_path = root / infra.SCENARIO_MANIFEST
    manifest = json.loads(manifest_path.read_text())
    scenario = manifest["scenarios"][0]
    scenario["owner_docs"] = [str(outside)]
    scenario["recovery"] = str(outside)
    scenario["permitted_scopes"] = ["../outside"]
    manifest_path.write_text(json.dumps(manifest))

    errors = infra.check_scenarios(root, "main")

    assert any("owner path escapes repository" in error for error in errors)
    assert any("recovery pointer escapes repository" in error for error in errors)
    assert any("permitted_scopes contains a path escape" in error for error in errors)


def test_scenario_malformed_route_type_reports_error_without_crashing(tmp_path):
    root = make_repo(tmp_path, "main")
    manifest_path = root / infra.SCENARIO_MANIFEST
    manifest = json.loads(manifest_path.read_text())
    manifest["scenarios"][0]["expected_route"]["kind"] = []
    manifest_path.write_text(json.dumps(manifest))

    errors = infra.check_scenarios(root, "main")

    assert any("workflow kind/name is invalid" in error for error in errors)


def test_scenario_external_agent_route_reports_error_without_crashing(tmp_path):
    root = make_repo(tmp_path, "main")
    manifest_path = root / infra.SCENARIO_MANIFEST
    manifest = json.loads(manifest_path.read_text())
    manifest["scenarios"][0]["expected_route"] = {
        "kind": "agent",
        "name": "/tmp/escape",
        "path": "/tmp/escape.toml",
    }
    manifest_path.write_text(json.dumps(manifest))

    errors = infra.check_scenarios(root, "main")

    assert any("unknown custom agent route" in error for error in errors)


def test_nested_agents_chain_budget_is_enforced(tmp_path):
    root = make_repo(tmp_path, "main")
    _write(root, "dart/solver/AGENTS.md", "x" * infra.MAX_AGENTS_BYTES)

    errors = infra.check_agents_chains(root)

    assert any("AGENTS chain exceeds" in error for error in errors)


def test_pixi_reference_check_includes_feature_and_target_tasks(tmp_path):
    root = make_repo(tmp_path, "main")
    _write(
        root,
        "pixi.toml",
        """
[tasks]
root-task = { cmd = "true" }
[feature.example.tasks]
feature-task = { cmd = "true" }
[feature.example.target.linux-64.tasks]
target-task = { cmd = "true" }
""",
    )
    _write(
        root,
        "docs/ai/README.md",
        "`pixi run root-task`; `pixi run -e example feature-task`; "
        "`pixi run target-task`; `pixi run missing-task`.\n",
    )

    errors = infra.check_pixi_references(root, "main")

    assert errors == ["docs/ai/README.md: references missing Pixi task 'missing-task'"]


@pytest.mark.parametrize(
    "invocation",
    (
        "pixi run missing-task",
        "pixi run --locked missing-task",
        "pixi run --frozen missing-task",
        "pixi run -e example missing-task",
        "pixi run --environment example missing-task",
        "pixi run --frozen --environment example missing-task",
    ),
)
def test_pixi_reference_check_covers_supported_flag_spellings(tmp_path, invocation):
    root = make_repo(tmp_path, "main")
    _write(root, "docs/ai/README.md", f"`{invocation}`\n")

    errors = infra.check_pixi_references(root, "main")

    assert errors == ["docs/ai/README.md: references missing Pixi task 'missing-task'"]


def test_pixi_reference_check_skips_other_branch_section(tmp_path):
    root = make_repo(tmp_path, "main")
    _write(
        root,
        "docs/ai/README.md",
        "## DART 6 (release-6.20)\n"
        "### Details\n"
        "```sh\n# fenced comment\n```\n"
        "`pixi run release-only` and `docs/release-only.md`.\n",
    )

    assert infra.check_pixi_references(root, "main") == []
    assert infra.check_path_references(root, "main") == []


def test_path_reference_check_is_bounded_to_inline_code(tmp_path):
    root = make_repo(tmp_path, "main")
    _write(
        root,
        "docs/ai/README.md",
        "Plain docs/missing is prose.\n"
        "URL https://github.com/dartsim/dart/pull/123 is external.\n"
        "Tracked `docs/readthedocs/architecture.md` exists.\n"
        "Broken `docs/does-not-exist.md` is checked.\n",
    )

    assert infra.check_path_references(root) == [
        "docs/ai/README.md: references missing path 'docs/does-not-exist.md'"
    ]


def test_path_reference_check_rejects_existing_escape(tmp_path):
    root = make_repo(tmp_path, "main")
    outside = tmp_path / "outside.md"
    outside.write_text("outside\n")
    relative_escape = f"docs/../../{outside.name}"
    _write(root, "docs/ai/README.md", f"Broken `{relative_escape}`.\n")

    errors = infra.check_path_references(root, "main")

    assert errors == [
        "docs/ai/README.md: referenced path escapes repository " f"'{relative_escape}'"
    ]


def test_non_readme_ai_owner_drift_is_checked(tmp_path):
    root = make_repo(tmp_path, "main")
    _write(
        root,
        "docs/ai/extra-owner.md",
        "`pixi run missing-owner-task` and `docs/missing-owner.md`.\n",
    )

    assert infra.check_pixi_references(root, "main") == [
        "docs/ai/extra-owner.md: references missing Pixi task 'missing-owner-task'"
    ]
    assert infra.check_path_references(root) == [
        "docs/ai/extra-owner.md: references missing path 'docs/missing-owner.md'"
    ]


def test_nested_agents_task_and_path_drift_is_checked(tmp_path):
    root = make_repo(tmp_path, "main")
    _write(
        root,
        "dart/solver/AGENTS.md",
        "`pixi run missing-nested-task` and `docs/missing-nested.md`.\n",
    )

    assert any(
        "dart/solver/AGENTS.md: references missing Pixi task 'missing-nested-task'"
        in error
        for error in infra.check_pixi_references(root, "main")
    )
    assert any(
        "dart/solver/AGENTS.md: references missing path 'docs/missing-nested.md'"
        in error
        for error in infra.check_path_references(root)
    )


def test_hook_config_rejects_unbounded_or_mutating_command(tmp_path):
    root = make_repo(tmp_path, "main")
    path = root / ".codex" / "hooks.json"
    data = json.loads(path.read_text())
    handler = data["hooks"]["PreToolUse"][0]["hooks"][0]
    handler["timeout"] = 31
    handler["command"] += " && git push"
    path.write_text(json.dumps(data))

    errors = infra.check_codex_hooks(root)

    assert any("canonical read-only guard invocation" in error for error in errors)
    assert any("timeout must equal 10" in error for error in errors)


def test_hook_config_requires_exact_windows_override(tmp_path):
    root = make_repo(tmp_path, "main")
    path = root / ".codex" / "hooks.json"
    data = json.loads(path.read_text())
    del data["hooks"]["PreToolUse"][0]["hooks"][0]["commandWindows"]
    path.write_text(json.dumps(data))

    errors = infra.check_codex_hooks(root)

    assert any("commandWindows" in error for error in errors)


@pytest.mark.parametrize(
    ("relative", "marker"),
    (
        (
            ".claude/hooks/pre-commit-guard.ps1",
            "$LASTEXITCODE = $null",
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
    root = make_repo(tmp_path, "main")
    path = root / relative
    original = path.read_text()
    path.unlink()

    missing_errors = infra.check_codex_hooks(root)

    assert any(relative in error for error in missing_errors)
    path.write_text(original.replace(marker, ""))

    drift_errors = infra.check_codex_hooks(root)

    assert any(marker in error for error in drift_errors)


def test_claude_hook_requires_cross_platform_bash_guard(tmp_path):
    root = make_repo(tmp_path, "main")
    path = root / ".claude" / "settings.json"
    data = json.loads(path.read_text())
    handler = data["hooks"]["PreToolUse"][0]["hooks"][0]
    handler["command"] = "/usr/bin/python3 scripts/check_agent_hook.py"
    handler.pop("shell")
    path.write_text(json.dumps(data))

    errors = infra.check_claude_hooks(root)

    assert any("project-root guard" in error for error in errors)
    assert any("shell must equal 'bash'" in error for error in errors)


def test_ci_wiring_requires_focused_ai_infrastructure_tests(tmp_path):
    root = make_repo(tmp_path, "main")
    path = root / ".github" / "workflows" / "ci_lint.yml"
    path.write_text("- run: pixi run check-lint\n")

    errors = infra.check_ci_wiring(root)

    assert errors == [
        ".github/workflows/ci_lint.yml: missing AI CI command "
        "'pixi run test-ai-infra'"
    ]


def test_ci_wiring_requires_native_windows_hook_smoke(tmp_path):
    root = make_repo(tmp_path, "main")
    path = root / ".github" / "workflows" / "ci_windows.yml"
    path.write_text(path.read_text().replace("$blockedExit -ne 2", ""))

    errors = infra.check_ci_wiring(root)

    assert errors == [
        ".github/workflows/ci_windows.yml: missing native hook smoke marker "
        "'$blockedExit -ne 2'"
    ]


@pytest.mark.parametrize(
    ("relative", "marker"),
    [
        (
            "python/tests/unit/test_agent_capture.py",
            "test_run_capture_smoke_writes_stills_and_sidecar",
        ),
        (
            "python/tests/unit/test_agent_capture.py",
            'layers=["contacts", "labels"]',
        ),
        (
            "python/tests/unit/test_agent_capture.py",
            "test_run_capture_debug_layers_change_pixels_end_to_end",
        ),
        (
            "python/tests/unit/test_agent_capture.py",
            'debug_layers = ["contacts", "collision_bounds", "labels"]',
        ),
        (
            "python/tests/unit/test_agent_capture.py",
            "for layer in debug_layers",
        ),
        (
            "python/tests/unit/test_agent_capture.py",
            "assert _changed_pixel_count(plain_image, combined_image) >= 128",
        ),
        (
            "python/tests/unit/test_agent_capture.py",
            "assert _changed_pixel_count(plain_image, debug_image) >= 32",
        ),
        ("python/tests/unit/test_agent_capture.py", "DART_REQUIRE_VISUAL_E2E"),
        (
            "python/tests/unit/test_agent_capture.py",
            "dartpy.gui.OffscreenRenderer is unavailable",
        ),
        (
            "python/tests/unit/test_agent_capture.py",
            'pytest.fail("visual e2e is required but DISPLAY is unavailable")',
        ),
        ("pixi.toml", "test-agent-visual-e2e ="),
        (
            "pixi.toml",
            "test_run_capture_debug_layers_change_pixels_end_to_end",
        ),
    ],
)
def test_ci_wiring_requires_visual_capture_smoke(tmp_path, relative, marker):
    root = make_repo(tmp_path, "main")
    path = root / relative
    path.write_text(path.read_text().replace(marker, "missing-visual-marker", 1))

    errors = infra.check_ci_wiring(root)

    assert any(relative in error and "visual CI marker" in error for error in errors)


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
        "xvfb-run -a -s '-screen 0 640x480x24' \\\n"
        "            env DART_REQUIRE_VISUAL_E2E=1 "
        "pixi run test-agent-visual-e2e",
    ),
)
def test_ci_wiring_requires_explicit_visual_verification_step(tmp_path, marker):
    root = make_repo(tmp_path, "main")
    path = root / ".github/workflows/ci_ubuntu.yml"
    content = path.read_text()
    if marker == "Agent visual verification smoke":
        mutated = content.replace(marker, "missing-visual-marker", 1)
    else:
        before, separator, visual_and_after = content.partition(
            "- name: Agent visual verification smoke"
        )
        assert separator
        mutated = (
            before
            + separator
            + visual_and_after.replace(marker, "missing-visual-marker", 1)
        )
    path.write_text(mutated)

    errors = infra.check_ci_wiring(root)

    expected_marker = "test-agent-visual-e2e" if "\n" in marker else marker
    assert any(
        "visual CI marker" in error and expected_marker in error for error in errors
    )


@pytest.mark.parametrize(
    ("mutation", "expected"),
    (
        ("condition", "must not use an if condition"),
        ("soft-fail", "must not use continue-on-error"),
    ),
)
def test_visual_smoke_cannot_be_skipped_or_soft_failed(tmp_path, mutation, expected):
    root = make_repo(tmp_path, "main")
    path = root / ".github/workflows/ci_ubuntu.yml"
    content = path.read_text()
    replacement = "- name: Agent visual verification smoke\n        "
    if mutation == "condition":
        replacement += "if: false"
    else:
        replacement += "continue-on-error: true"
    path.write_text(
        content.replace("- name: Agent visual verification smoke", replacement, 1)
    )

    errors = infra.check_ci_wiring(root)

    assert any(expected in error for error in errors)


@pytest.mark.parametrize(
    "marker",
    (
        "test_debug_scene_overlay_lifecycle_on_shared_renderer",
        'layers=("grid", "body_frames", "contacts")',
        "_bytes_over_tolerance(debugged, plain) >= 256",
        "_bytes_over_tolerance(emptied, plain) <= 64",
    ),
)
def test_ci_wiring_requires_engine_overlay_pixel_regression(tmp_path, marker):
    root = make_repo(tmp_path, "main")
    relative = "python/tests/unit/gui/test_offscreen_render.py"
    path = root / relative
    path.write_text(path.read_text().replace(marker, "missing-overlay-marker", 1))

    errors = infra.check_ci_wiring(root)

    assert any(
        relative in error and "visual CI marker" in error and marker in error
        for error in errors
    )


def test_hook_config_rejects_extra_handler_keys(tmp_path):
    root = make_repo(tmp_path, "main")
    path = root / ".codex" / "hooks.json"
    data = json.loads(path.read_text())
    data["hooks"]["PreToolUse"][0]["hooks"][0]["extra"] = "ignored"
    path.write_text(json.dumps(data))

    errors = infra.check_codex_hooks(root)

    assert any("handler keys must equal" in error for error in errors)


def test_json_schema_checks_report_non_object_root_without_crashing(tmp_path):
    root = make_repo(tmp_path, "main")
    (root / ".codex" / "hooks.json").write_text("[]\n")

    errors = infra.check_codex_hooks(root)

    assert any("JSON root must be an object" in error for error in errors)


def test_config_schema_reports_non_table_agents_without_crashing(tmp_path):
    root = make_repo(tmp_path, "main")
    (root / ".codex" / "config.toml").write_text('agents = "wrong"\n')

    errors = infra.check_codex_config(root)

    assert any("agents must be a table" in error for error in errors)


@pytest.mark.parametrize(
    "content",
    (
        "[agents]\nmax_threads = true\nmax_depth = 1\n",
        "[agents]\nmax_threads = 4\nmax_depth = true\n",
        "[agents]\nmax_threads = 4.0\nmax_depth = 1\n",
        "[agents]\nmax_threads = 4\nmax_depth = 1.0\n",
    ),
)
def test_config_schema_requires_exact_integer_limits(tmp_path, content):
    root = make_repo(tmp_path, "main")
    (root / ".codex" / "config.toml").write_text(content)

    assert infra.check_codex_config(root)


@pytest.mark.parametrize(
    "path",
    (
        ".gitignore",
        "dart/common/AGENTS.md",
        "docs/onboarding/ai-tools.md",
        ".github/workflows/ci_lint.yml",
        ".github/workflows/ci_ubuntu.yml",
        ".github/workflows/ci_windows.yml",
        "python/tests/unit/test_agent_capture.py",
        "python/tests/unit/gui/test_offscreen_render.py",
        "scripts/pretool_guard_bridge.py",
    ),
)
def test_ai_infrastructure_path_includes_structural_surfaces(path):
    assert infra.is_ai_infrastructure_path(path)


def test_generated_adapter_manifest_is_required(tmp_path):
    root = make_repo(tmp_path, "main")
    (root / ".agents/skills/.dart-generated.json").unlink()

    errors = infra.check_generated_adapters(root)

    assert any(".dart-generated.json" in error for error in errors)


@pytest.mark.parametrize(
    ("field", "value"),
    (("schema_version", True), ("paths", ["-bad/SKILL.md"])),
)
def test_generated_adapter_manifest_rejects_shared_schema_edge_cases(
    tmp_path, field, value
):
    root = make_repo(tmp_path, "main")
    manifest_path = root / ".agents/skills/.dart-generated.json"
    manifest = json.loads(manifest_path.read_text())
    manifest[field] = value
    manifest_path.write_text(json.dumps(manifest))

    errors = infra.check_generated_adapters(root)

    assert errors


def test_missing_generated_adapter_is_rejected(tmp_path):
    root = make_repo(tmp_path, "main")
    adapter = root / ".agents/skills/dart-new-task/SKILL.md"
    adapter.unlink()

    errors = infra.check_generated_adapters(root)

    assert any(
        "dart-new-task/SKILL.md: missing generated skill" in error for error in errors
    )


def test_marker_owned_generated_skill_outside_manifest_is_rejected(tmp_path):
    root = make_repo(tmp_path, "main")
    stale = root / ".agents" / "skills" / "dart-stale" / "SKILL.md"
    _write(
        root,
        str(stale.relative_to(root)),
        sync.add_auto_gen_header(
            '---\nname: dart-stale\ndescription: "DART Stale: fixture"\n---\n',
            ".claude/skills/dart-stale/SKILL.md",
        ),
    )

    errors = infra.check_generated_adapters(root)

    assert any("orphaned generated skill" in error for error in errors)


def test_generated_adapter_checker_rejects_symlinked_expected_skill(tmp_path):
    root = make_repo(tmp_path, "main")
    target = root / ".agents" / "skills" / "dart-new-task"
    target.joinpath("SKILL.md").unlink()
    target.rmdir()
    outside = tmp_path / "external-skill"
    outside.mkdir()
    (outside / "SKILL.md").write_text("external\n")
    target.symlink_to(outside, target_is_directory=True)

    errors = infra.check_generated_adapters(root)

    assert any(
        "generated skill path" in error and "symlink" in error for error in errors
    )


def test_non_object_scenario_entry_is_rejected(tmp_path):
    root = make_repo(tmp_path, "main")
    path = root / infra.SCENARIO_MANIFEST
    data = json.loads(path.read_text())
    data["scenarios"].append("junk")
    path.write_text(json.dumps(data))

    errors = infra.check_scenarios(root, "main")

    assert any("scenarios[7] must be an object" in error for error in errors)


def test_scenario_cli_fails_on_global_manifest_error(tmp_path):
    root = make_repo(tmp_path, "main")
    path = root / infra.SCENARIO_MANIFEST
    data = json.loads(path.read_text())
    data["scenarios"].append("junk")
    path.write_text(json.dumps(data))

    run = subprocess.run(
        [
            sys.executable,
            str(SCRIPTS / "exercise_agent_scenarios.py"),
            "--profile",
            "main",
            "--root",
            str(root),
        ],
        capture_output=True,
        text=True,
    )

    assert run.returncode == 1
    assert "scenarios[7] must be an object" in run.stdout


def test_simulation_scenario_requires_text_and_visual_evidence_policy(tmp_path):
    root = make_repo(tmp_path, "main")
    path = root / infra.SCENARIO_MANIFEST
    data = json.loads(path.read_text())
    scenario = next(
        item for item in data["scenarios"] if item["id"] == "simulation-verification"
    )
    scenario["evidence_policy"] = "screenshot-only"
    path.write_text(json.dumps(data))

    errors = infra.check_scenarios(root, "main")

    assert any("wrong evidence policy" in error for error in errors)


def test_main_simulation_scenario_covers_all_applicable_surfaces():
    data = json.loads((ROOT / infra.SCENARIO_MANIFEST).read_text())
    scenario = next(
        item for item in data["scenarios"] if item["id"] == "simulation-verification"
    )

    assert scenario["full_gates"] == ["pixi run test-py", "pixi run test-all"]
    assert {
        "dart/simulation",
        "dart/dynamics",
        "dart/collision",
        "dart/constraint",
        "dart/gui",
        "dart/io",
        "python",
        "examples",
        "tutorials",
        "matching tests",
        "temporary claim-tied evidence",
    } <= set(scenario["permitted_scopes"])


@pytest.mark.parametrize(
    ("mutation", "expected"),
    [
        ("prompt", "wrong claim-dependent prompt"),
        ("route", "must route to dart-verify-sim"),
        ("scope", "must cover simulation"),
        ("focused_gate", "focused correctness gate"),
        ("full_gate", "full correctness gates"),
    ],
)
def test_simulation_scenario_semantic_mutations_are_rejected(
    tmp_path, mutation, expected
):
    root = make_repo(tmp_path, "main")
    path = root / infra.SCENARIO_MANIFEST
    data = json.loads(path.read_text())
    scenario = next(
        item for item in data["scenarios"] if item["id"] == "simulation-verification"
    )
    if mutation == "prompt":
        scenario["prompt_class"] = "take a screenshot"
    elif mutation == "route":
        scenario["expected_route"] = {
            "kind": "domain_skill",
            "name": "dart-build",
            "path": ".agents/skills/dart-build/SKILL.md",
        }
    elif mutation == "scope":
        scenario["permitted_scopes"].remove("dart/gui")
    elif mutation == "focused_gate":
        scenario["focused_gates"] = ["pixi run test-py"]
    else:
        scenario["full_gates"] = ["pixi run test-all"]
    path.write_text(json.dumps(data))

    errors = infra.check_scenarios(root, "main")

    assert any(expected in error for error in errors)


@pytest.mark.parametrize(
    "marker",
    [
        "text oracle",
        "agent-capture",
        "image-verdict",
        "sole correctness oracle",
        "model/scene loading",
        "collision/contact/constraints",
        "stepping, GUI/rendering",
        "GUI/rendering",
        "visual examples",
        "rendering is unavailable",
        "replacement evidence",
    ],
)
def test_simulation_skill_contract_markers_are_required(tmp_path, marker):
    root = make_repo(tmp_path, "main")
    source = root / ".claude/skills/dart-verify-sim/SKILL.md"
    source.write_text(source.read_text().replace(marker, "missing-marker", 1))

    errors = infra.check_scenarios(root, "main")

    assert any("missing contract marker" in error for error in errors)


@pytest.mark.parametrize(
    ("relative", "marker"),
    [
        ("AGENTS.md", "docs/onboarding/agent-sim-verification.md"),
        (".codex/agents/dart_scout.toml", "text correctness oracle"),
        (".codex/agents/dart_scout.toml", "collision/contact/constraints"),
        (".codex/agents/dart_reviewer.toml", "text-first evidence"),
        (".claude/commands/dart-new-task.md", "route through `dart-verify-sim`"),
        (".claude/commands/dart-new-task.md", "record why it is not applicable"),
        (".claude/commands/dart-ultrawork.md", "routes through `dart-verify-sim`"),
        (".claude/commands/dart-resume.md", "route through `dart-verify-sim`"),
        (".claude/commands/dart-pr.md", "use `dart-verify-sim`"),
        (".claude/commands/dart-manage-pr.md", "Visual verification"),
        (
            ".claude/commands/dart-review-pr.md",
            "require the `dart-verify-sim` text oracle",
        ),
        (".claude/commands/dart-review-pr.md", "accepting a screenshot alone"),
        (".claude/skills/dart-build/SKILL.md", "dart-verify-sim"),
        (".claude/skills/dart-test/SKILL.md", "load `dart-verify-sim`"),
        (".claude/skills/dart-test/SKILL.md", "unavailable or not applicable"),
        (".claude/skills/dart-io/SKILL.md", "also load `dart-verify-sim`"),
        (".claude/skills/dart-io/SKILL.md", "claim-tied visual corroboration"),
        (".claude/skills/dart-python/SKILL.md", "also load `dart-verify-sim`"),
        (
            ".claude/skills/dart-python/SKILL.md",
            "collision/contact/constraints",
        ),
        (".claude/skills/dart-python/SKILL.md", "GUI/rendering output"),
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
            "target branch's `dart-verify-sim`",
        ),
        (".claude/commands/dart-backport-pr.md", "visual exception"),
        (".claude/commands/dart-fix-ci.md", "use `dart-verify-sim`"),
        (".claude/commands/dart-fix-ci.md", "visual exception"),
        (
            ".claude/commands/dart-fix-issue.md",
            "route through `dart-verify-sim`",
        ),
        (
            ".claude/commands/dart-fix-issue.md",
            "collision/contact/constraints",
        ),
        (".claude/commands/dart-fix-issue.md", "visual exception"),
        ("docs/ai/verification.md", "If rendering is unavailable"),
        ("docs/ai/verification.md", "name replacement evidence"),
    ],
)
def test_simulation_consumer_routes_are_required(tmp_path, relative, marker):
    root = make_repo(tmp_path, "main")
    path = root / relative
    path.write_text(path.read_text().replace(marker, "missing-marker", 1))

    errors = infra.check_scenarios(root, "main")

    assert any(
        relative in error and "simulation route marker" in error for error in errors
    )


def test_scenario_cli_accepts_seven_valid_routes(tmp_path):
    root = make_repo(tmp_path, "main")

    run = subprocess.run(
        [
            sys.executable,
            str(SCRIPTS / "exercise_agent_scenarios.py"),
            "--profile",
            "main",
            "--root",
            str(root),
        ],
        capture_output=True,
        text=True,
    )

    assert run.returncode == 0, run.stdout
    assert "Exercised 7 scenarios" in run.stdout


def test_stale_generated_adapters_are_rejected(tmp_path):
    root = make_repo(tmp_path, "main")
    source = root / ".claude/commands/dart-new-task.md"
    source.write_text(source.read_text() + "changed\n")

    errors = infra.check_generated_adapters(root)

    assert any(
        "dart-new-task.md: generated command is stale" in error for error in errors
    )
    assert any(
        "dart-new-task/SKILL.md: generated skill is stale" in error for error in errors
    )


def test_unowned_legacy_canonical_skill_collision_is_rejected(tmp_path):
    root = make_repo(tmp_path, "main")
    _write(root, ".codex/skills/dart-new-task/SKILL.md", "unowned legacy skill\n")

    errors = infra.check_generated_adapters(root)

    assert any("legacy path collides with canonical" in error for error in errors)


def test_legacy_skill_symlink_is_reported_without_dereference(tmp_path):
    root = make_repo(tmp_path, "main")
    legacy = root / ".codex" / "skills" / "dart-stale" / "SKILL.md"
    legacy.parent.mkdir(parents=True)
    legacy.symlink_to(tmp_path / "missing-legacy-target")

    errors = infra.check_generated_adapters(root)

    assert any("legacy skill path must not be a symlink" in error for error in errors)


def test_legacy_skill_root_symlink_is_reported_without_dereference(tmp_path):
    root = make_repo(tmp_path, "main")
    outside = tmp_path / "external-legacy"
    external = outside / "dart-stale" / "SKILL.md"
    external.parent.mkdir(parents=True)
    external.write_text("external\n")
    legacy_root = root / ".codex" / "skills"
    legacy_root.parent.mkdir(parents=True, exist_ok=True)
    legacy_root.symlink_to(outside, target_is_directory=True)

    errors = infra.check_generated_adapters(root)

    assert ".codex/skills: legacy skill root must not be a symlink" in errors
    assert external.read_text() == "external\n"


def test_generated_root_must_be_a_directory(tmp_path):
    root = make_repo(tmp_path, "main")
    generated = root / ".agents" / "skills"
    for child in generated.iterdir():
        if child.is_dir():
            for nested in child.iterdir():
                nested.unlink()
            child.rmdir()
        else:
            child.unlink()
    generated.rmdir()
    generated.write_text("not a directory\n")

    assert infra.check_generated_adapters(root) == [
        ".agents/skills: generated root must be a directory"
    ]


def _content_hashes(root: Path) -> dict[str, str]:
    return {
        str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in root.rglob("*")
        if path.is_file() and ".git" not in path.relative_to(root).parts
    }


def test_doctor_report_is_read_only(tmp_path):
    root = make_repo(tmp_path, "main")
    before = _content_hashes(root)

    result = ai_doctor.report(root, "main")

    assert result["ok"]
    assert result["schema_version"] == 1
    assert result["profile"] == "main"
    assert result["inventory"]["instructions"]["scenario_chains"]
    assert result["inventory"]["generated_skills"]["manifest"] == (
        ".agents/skills/.dart-generated.json"
    )
    assert result["inventory"]["custom_agents"]["count"] == 3
    assert result["inventory"]["setup"]["path"] == "scripts/setup_ai.py"
    assert result["inventory"]["hooks"]["windows_launcher"] == (
        ".claude/hooks/pre-commit-guard.ps1"
    )
    assert result["inventory"]["hooks"]["windows_bridge"] == (
        "scripts/pretool_guard_bridge.py"
    )
    assert not result["inventory"]["git_hook"]["current"]
    assert "check-ai-infra" in result["inventory"]["tasks"]["ai_tasks"]
    assert set(result["trust"]) == {"user_config", "project", "project_hook"}
    assert _content_hashes(root) == before


def test_doctor_reports_current_managed_git_hook(tmp_path):
    root = make_repo(tmp_path, "main")
    hook = root / ".git" / "hooks" / "pre-commit"
    hook.parent.mkdir(parents=True, exist_ok=True)
    hook.write_text(install_git_hooks.HOOK_TEMPLATE)
    hook.chmod(0o755)

    result = ai_doctor.report(root, "main")

    assert result["inventory"]["git_hook"]["current"]
    assert not any("managed Git pre-commit hook" in item for item in result["warnings"])


def test_doctor_rejects_truncated_managed_git_hook(tmp_path):
    root = make_repo(tmp_path, "main")
    hook = root / ".git" / "hooks" / "pre-commit"
    hook.parent.mkdir(parents=True, exist_ok=True)
    hook.write_text("#!/bin/sh\n# DART-MANAGED-HOOK v4\n")
    hook.chmod(0o755)

    result = ai_doctor.report(root, "main")

    assert not result["inventory"]["git_hook"]["current"]
    assert "pixi run install-hooks" in {item["command"] for item in result["recovery"]}


def test_doctor_custom_hookspath_recovery_does_not_recommend_installer(tmp_path):
    root = make_repo(tmp_path, "main")
    subprocess.run(
        ["git", "-C", str(root), "config", "core.hooksPath", ".custom-hooks"],
        check=True,
    )

    result = ai_doctor.report(root, "main")
    commands = {item["command"] for item in result["recovery"]}

    assert "python3 scripts/check_agent_hook.py --profile staged" in commands
    assert "pixi run install-hooks" not in commands


def test_doctor_recovery_does_not_claim_setup_repairs_tracked_hook(tmp_path):
    root = make_repo(tmp_path, "main")
    path = root / ".codex" / "hooks.json"
    data = json.loads(path.read_text())
    data["hooks"]["PreToolUse"][0]["hooks"][0]["command"] += "; rm -rf /tmp/x"
    path.write_text(json.dumps(data))

    result = ai_doctor.report(root, "main")
    commands = [item["command"] for item in result["recovery"]]

    assert (
        "git diff -- .codex/hooks.json .claude/hooks/pre-commit-guard.sh "
        ".claude/hooks/pre-commit-guard.ps1 scripts/pretool_guard_bridge.py" in commands
    )
    assert "pixi run python scripts/setup_ai.py" not in commands
    assert "pixi run install-hooks" in commands


def test_doctor_recovery_names_all_hook_sources_for_launcher_drift(tmp_path):
    root = make_repo(tmp_path, "main")
    (root / ".claude" / "hooks" / "pre-commit-guard.ps1").unlink()

    result = ai_doctor.report(root, "main")
    commands = [item["command"] for item in result["recovery"]]

    assert (
        "git diff -- .codex/hooks.json .claude/hooks/pre-commit-guard.sh "
        ".claude/hooks/pre-commit-guard.ps1 scripts/pretool_guard_bridge.py" in commands
    )


def test_doctor_recovery_is_keyed_to_generated_failure(tmp_path):
    root = make_repo(tmp_path, "main")
    (root / ".agents/skills/.dart-generated.json").unlink()

    result = ai_doctor.report(root, "main")

    commands = [item["command"] for item in result["recovery"]]
    assert not result["ok"]
    assert "pixi run sync-ai-commands" in commands
    assert "pixi run test-ai-infra" in commands


def test_doctor_trust_requires_current_project_hook_hash(tmp_path, monkeypatch):
    root = make_repo(tmp_path, "main")
    codex_home = tmp_path / "codex-home"
    codex_home.mkdir()
    config = codex_home / "config.toml"
    monkeypatch.setenv("CODEX_HOME", str(codex_home))
    config.write_text(
        '[hooks.state."other:/somewhere/.codex/hooks.json:pre_tool_use:0:0"]\n'
        'trusted_hash = "sha256:other"\n'
    )

    assert ai_doctor._codex_trust(root)["project_hook"] == (
        "ambiguous-hook-hash-observed"
    )

    source = f"project:{(root / '.codex/hooks.json').resolve()}:pre_tool_use:0:0"
    config.write_text(
        f'[hooks.state.{json.dumps(source)}]\ntrusted_hash = "sha256:current"\n'
    )

    assert ai_doctor._codex_trust(root)["project_hook"] == "trusted-hash-observed"
