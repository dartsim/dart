import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_dart7_clean_break_policy.py"
GZ_WORKFLOW = ROOT / ".github" / "workflows" / "ci_gz_physics.yml"
PIXI = ROOT / "pixi.toml"
RELEASE_ROADMAP = ROOT / "docs" / "onboarding" / "release-roadmap.md"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "check_dart7_clean_break_policy", SCRIPT
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write(tmp_path, name, text):
    path = tmp_path / name
    path.write_text(text, encoding="utf-8")
    return path


def test_current_repo_clean_break_policy_passes():
    module = _load_module()

    assert module.find_violations(GZ_WORKFLOW, PIXI, RELEASE_ROADMAP) == []


def test_gz_workflow_rejects_main_push_branch(tmp_path):
    module = _load_module()
    workflow = _write(
        tmp_path,
        "ci_gz_physics.yml",
        GZ_WORKFLOW.read_text(encoding="utf-8").replace(
            '      - "release-*"',
            '      - "main"',
            1,
        ),
    )

    messages = [violation.message for violation in module.check_gz_workflow(workflow)]

    assert any("push branches must be release-* only" in message for message in messages)


def test_gz_workflow_rejects_schedule_trigger(tmp_path):
    module = _load_module()
    workflow = _write(
        tmp_path,
        "ci_gz_physics.yml",
        GZ_WORKFLOW.read_text(encoding="utf-8").replace(
            "  workflow_dispatch:",
            "  schedule:\n    - cron: '0 0 * * *'\n  workflow_dispatch:",
        ),
    )

    messages = [violation.message for violation in module.check_gz_workflow(workflow)]

    assert "CI gz-physics must not run on a main-branch schedule" in messages


def test_release_roadmap_must_name_pinned_gz_branch(tmp_path):
    module = _load_module()
    roadmap = _write(
        tmp_path,
        "release-roadmap.md",
        RELEASE_ROADMAP.read_text(encoding="utf-8").replace(
            "gz-physics9_9.0.0", "gz-physics-next"
        ),
    )

    messages = [
        violation.message
        for violation in module.check_release_roadmap(
            roadmap, module.pinned_gz_physics_branch(PIXI)
        )
    ]

    assert any("must document the pinned gz-physics branch" in m for m in messages)
