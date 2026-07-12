"""Tests for scripts/sync_ai_commands.py."""

import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "sync_ai_commands.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("sync_ai_commands", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write_command(path, required):
    required_lines = "\n".join(f"@{entry}" for entry in required)
    path.write_text(
        '---\ndescription: docs\nargument-hint: "<topic>"\n---\n\n'
        "## Required Reading\n\n"
        f"{required_lines}\n\n"
        "## Workflow\n\n1. Work.\n\n"
        "## Output\n\n- Done.\n",
        encoding="utf-8",
    )


def _write_skill(path, name="dart-test", description="DART Test: test workflow"):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        f'---\nname: {name}\ndescription: "{description}"\n---\n\n# Test\n',
        encoding="utf-8",
    )


def _temporary_skill_repo(tmp_path, module):
    skill_source = tmp_path / ".claude" / "skills"
    command_source = tmp_path / ".claude" / "commands"
    target = tmp_path / ".agents" / "skills"
    command_source.mkdir(parents=True)
    _write_skill(skill_source / "dart-test" / "SKILL.md")
    module.get_repo_root = lambda: tmp_path
    return skill_source, command_source, target


def _stub_sync_all_validators(module, monkeypatch):
    for name in (
        "check_capability_parity",
        "validate_style_and_budget",
        "validate_skill_repository_references",
        "validate_command_structure",
        "validate_ai_docs",
    ):
        monkeypatch.setattr(module, name, lambda _root: True)


def test_dart_docs_update_required_reading_guard_passes(tmp_path):
    module = _load_module()
    command = tmp_path / "dart-docs-update.md"
    _write_command(
        command,
        [
            "AGENTS.md",
            "docs/README.md",
            "docs/AGENTS.md",
            "docs/information-architecture.md",
        ],
    )

    assert module.docs_update_required_reading_errors(command) == []


def test_dart_docs_update_required_reading_guard_rejects_missing_ia(tmp_path):
    module = _load_module()
    command = tmp_path / "dart-docs-update.md"
    _write_command(command, ["AGENTS.md", "docs/README.md", "docs/AGENTS.md"])

    failures = module.docs_update_required_reading_errors(command)

    assert any("docs/information-architecture.md" in failure for failure in failures)


def test_dart_new_task_required_reading_guard_passes(tmp_path):
    module = _load_module()
    command = tmp_path / "dart-new-task.md"
    _write_command(
        command,
        [
            "AGENTS.md",
            "docs/dev_tasks/README.md",
            "docs/information-architecture.md",
        ],
    )

    assert module.new_task_required_reading_errors(command) == []


def test_dart_new_task_required_reading_guard_rejects_missing_ia(tmp_path):
    module = _load_module()
    command = tmp_path / "dart-new-task.md"
    _write_command(command, ["AGENTS.md", "docs/dev_tasks/README.md"])

    failures = module.new_task_required_reading_errors(command)

    assert any("docs/information-architecture.md" in failure for failure in failures)


def test_codex_sync_preserves_unowned_skill_and_writes_ownership_manifest(tmp_path):
    module = _load_module()
    skill_source, command_source, target = _temporary_skill_repo(tmp_path, module)
    user_skill = target / "personal-helper" / "SKILL.md"
    _write_skill(user_skill, "personal-helper", "Personal Helper: local workflow")
    original_user_content = user_skill.read_text()

    synced, changed, unchanged, orphans = module.sync_codex_skills(
        skill_source, command_source, target
    )

    assert not synced
    assert (changed, unchanged, orphans) == (1, 0, 0)
    assert user_skill.read_text() == original_user_content
    manifest = json.loads((target / module.GENERATED_SKILLS_MANIFEST).read_text())
    assert manifest == {
        "schema_version": module.GENERATED_SKILLS_SCHEMA_VERSION,
        "generator": "scripts/sync_ai_commands.py",
        "paths": ["dart-test/SKILL.md"],
    }


def test_generated_header_round_trip_preserves_markdown_spacing():
    module = _load_module()
    source = '---\nname: dart-test\ndescription: "DART Test: test"\n---\n\n# Test\n'

    generated = module.add_auto_gen_header(source, ".claude/skills/dart-test/SKILL.md")

    assert "---\n\n<!-- AUTO-GENERATED FILE" in generated
    assert module.strip_auto_gen_header(generated) == source


def test_parity_and_style_ignore_unowned_user_skill(tmp_path):
    module = _load_module()
    skill_source, command_source, target = _temporary_skill_repo(tmp_path, module)
    user_skill = target / "personal-helper" / "SKILL.md"
    _write_skill(user_skill, "personal-helper", "invalid local metadata")
    module.sync_codex_skills(skill_source, command_source, target)

    assert module.check_capability_parity(tmp_path)
    assert module.validate_style_and_budget(tmp_path)


def test_codex_sync_refuses_to_overwrite_unowned_name_collision(tmp_path):
    module = _load_module()
    skill_source, command_source, target = _temporary_skill_repo(tmp_path, module)
    target_skill = target / "dart-test" / "SKILL.md"
    _write_skill(target_skill, "dart-test", "DART Test: user-owned workflow")
    original_content = target_skill.read_text()

    synced, changed, _, _ = module.sync_codex_skills(
        skill_source, command_source, target
    )

    assert not synced
    assert changed == -1
    assert target_skill.read_text() == original_content
    assert not (target / module.GENERATED_SKILLS_MANIFEST).exists()


def test_codex_sync_rejects_symlinked_generated_skill_without_external_write(
    tmp_path,
):
    module = _load_module()
    skill_source, command_source, target = _temporary_skill_repo(tmp_path, module)
    outside = tmp_path / "outside"
    outside.mkdir()
    external = outside / "SKILL.md"
    external.write_text("external\n")
    target.mkdir(parents=True)
    (target / "dart-test").symlink_to(outside, target_is_directory=True)

    synced, changed, _, _ = module.sync_codex_skills(
        skill_source, command_source, target
    )

    assert not synced
    assert changed == -1
    assert external.read_text() == "external\n"


def test_codex_sync_rejects_symlinked_manifest_without_external_write(tmp_path):
    module = _load_module()
    skill_source, command_source, target = _temporary_skill_repo(tmp_path, module)
    target.mkdir(parents=True)
    external = tmp_path / "outside-manifest.json"
    external.write_text("{}\n")
    (target / module.GENERATED_SKILLS_MANIFEST).symlink_to(external)

    synced, changed, _, _ = module.sync_codex_skills(
        skill_source, command_source, target
    )

    assert not synced
    assert changed == -1
    assert external.read_text() == "{}\n"


def test_codex_sync_rejects_non_directory_generated_root(tmp_path):
    module = _load_module()
    skill_source, command_source, target = _temporary_skill_repo(tmp_path, module)
    target.parent.mkdir(parents=True)
    target.write_text("not a directory\n")

    synced, changed, _, _ = module.sync_codex_skills(
        skill_source, command_source, target
    )

    assert not synced
    assert changed == -1


def test_codex_sync_check_detects_missing_and_mismatched_adapters(tmp_path):
    module = _load_module()
    skill_source, command_source, target = _temporary_skill_repo(tmp_path, module)
    module.sync_codex_skills(skill_source, command_source, target)

    target_skill = target / "dart-test" / "SKILL.md"
    target_skill.write_text(target_skill.read_text() + "\nlocal drift\n")
    assert not module.sync_codex_skills(
        skill_source, command_source, target, check_only=True
    )[0]

    target_skill.unlink()
    assert not module.sync_codex_skills(
        skill_source, command_source, target, check_only=True
    )[0]

    retained_note = target_skill.parent / "notes.txt"
    retained_note.write_text("keep me\n")
    module.sync_codex_skills(skill_source, command_source, target)
    assert target_skill.exists()
    assert retained_note.read_text() == "keep me\n"


def test_codex_sync_check_detects_manifest_mismatch(tmp_path):
    module = _load_module()
    skill_source, command_source, target = _temporary_skill_repo(tmp_path, module)
    module.sync_codex_skills(skill_source, command_source, target)
    manifest_path = target / module.GENERATED_SKILLS_MANIFEST
    manifest = json.loads(manifest_path.read_text())
    manifest["paths"] = []
    manifest_path.write_text(json.dumps(manifest))

    assert not module.sync_codex_skills(
        skill_source, command_source, target, check_only=True
    )[0]


@pytest.mark.parametrize(
    ("field", "value"),
    (("schema_version", True), ("paths", ["-bad/SKILL.md"])),
)
def test_generated_manifest_rejects_shared_schema_edge_cases(tmp_path, field, value):
    module = _load_module()
    skill_source, command_source, target = _temporary_skill_repo(tmp_path, module)
    module.sync_codex_skills(skill_source, command_source, target)
    manifest_path = target / module.GENERATED_SKILLS_MANIFEST
    manifest = json.loads(manifest_path.read_text())
    manifest[field] = value
    manifest_path.write_text(json.dumps(manifest))

    _, errors, _ = module.read_generated_skills_manifest(target)

    assert errors


def test_codex_sync_removes_only_manifest_owned_orphan_skill(tmp_path):
    module = _load_module()
    skill_source, command_source, target = _temporary_skill_repo(tmp_path, module)
    module.sync_codex_skills(skill_source, command_source, target)
    source_skill = skill_source / "dart-test" / "SKILL.md"
    source_skill.unlink()
    source_skill.parent.rmdir()
    retained_note = target / "dart-test" / "notes.txt"
    retained_note.write_text("user note\n")

    synced, _, _, orphans = module.sync_codex_skills(
        skill_source, command_source, target
    )

    assert not synced
    assert orphans == 1
    assert not (target / "dart-test" / "SKILL.md").exists()
    assert retained_note.read_text() == "user note\n"
    manifest = json.loads((target / module.GENERATED_SKILLS_MANIFEST).read_text())
    assert manifest["paths"] == []


def test_codex_sync_removes_marker_owned_orphan_omitted_from_manifest(tmp_path):
    module = _load_module()
    skill_source, command_source, target = _temporary_skill_repo(tmp_path, module)
    module.sync_codex_skills(skill_source, command_source, target)
    stale = target / "dart-stale" / "SKILL.md"
    stale.parent.mkdir()
    stale.write_text(
        module.add_auto_gen_header(
            '---\nname: dart-stale\ndescription: "DART Stale: fixture"\n---\n',
            ".claude/skills/dart-stale/SKILL.md",
        )
    )

    assert not module.sync_codex_skills(
        skill_source, command_source, target, check_only=True
    )[0]
    synced, _, _, orphans = module.sync_codex_skills(
        skill_source, command_source, target
    )

    assert not synced
    assert orphans == 1
    assert not stale.exists()


def test_codex_sync_refuses_to_delete_owned_orphan_without_marker(tmp_path):
    module = _load_module()
    skill_source, command_source, target = _temporary_skill_repo(tmp_path, module)
    module.sync_codex_skills(skill_source, command_source, target)
    source_skill = skill_source / "dart-test" / "SKILL.md"
    source_skill.unlink()
    source_skill.parent.rmdir()
    target_skill = target / "dart-test" / "SKILL.md"
    target_skill.write_text(
        '---\nname: dart-test\ndescription: "DART Test: retained"\n---\n'
    )
    retained_content = target_skill.read_text()

    synced, changed, _, orphans = module.sync_codex_skills(
        skill_source, command_source, target
    )

    assert not synced
    assert changed == -1
    assert orphans == 1
    assert target_skill.read_text() == retained_content


def test_codex_sync_refuses_owned_orphan_skill_symlink(tmp_path):
    module = _load_module()
    skill_source, command_source, target = _temporary_skill_repo(tmp_path, module)
    module.sync_codex_skills(skill_source, command_source, target)
    source_skill = skill_source / "dart-test" / "SKILL.md"
    source_skill.unlink()
    source_skill.parent.rmdir()
    target_skill = target / "dart-test" / "SKILL.md"
    target_skill.unlink()
    target_skill.symlink_to(tmp_path / "missing-owned-orphan")

    synced, changed, _, orphans = module.sync_codex_skills(
        skill_source, command_source, target
    )

    assert not synced
    assert changed == -1
    assert orphans == 0
    assert target_skill.is_symlink()


def test_codex_sync_rejects_source_folder_and_metadata_name_mismatch(tmp_path):
    module = _load_module()
    skill_source, command_source, target = _temporary_skill_repo(tmp_path, module)
    _write_skill(
        skill_source / "dart-test" / "SKILL.md",
        "dart-other",
        "DART Other: mismatched name",
    )

    synced, changed, _, _ = module.sync_codex_skills(
        skill_source, command_source, target
    )

    assert not synced
    assert changed == -1
    assert not (target / "dart-test" / "SKILL.md").exists()
    assert not (target / module.GENERATED_SKILLS_MANIFEST).exists()


def test_legacy_cleanup_removes_generated_adapter_and_preserves_user_skill(tmp_path):
    module = _load_module()
    legacy = tmp_path / ".codex" / "skills"
    generated = legacy / "dart-test" / "SKILL.md"
    user_skill = legacy / "personal-helper" / "SKILL.md"
    generated.parent.mkdir(parents=True)
    generated.write_text(
        module.add_auto_gen_header(
            '---\nname: dart-test\ndescription: "DART Test: generated"\n---\n',
            ".claude/skills/dart-test/SKILL.md",
        )
    )
    _write_skill(user_skill, "personal-helper", "Personal Helper: local workflow")

    clean, removed, conflicts = module.cleanup_legacy_codex_skills(
        legacy, {"dart-test"}
    )

    assert not clean
    assert removed == 1
    assert conflicts == 0
    assert not generated.exists()
    assert user_skill.exists()


@pytest.mark.parametrize("directory_symlink", (False, True))
def test_legacy_cleanup_preserves_symlinks_without_dereference(
    tmp_path, directory_symlink, capsys
):
    module = _load_module()
    legacy = tmp_path / ".codex" / "skills"
    legacy.mkdir(parents=True)
    path = legacy / "dart-stale" / "SKILL.md"
    if directory_symlink:
        outside = tmp_path / "outside"
        outside.mkdir()
        (legacy / "dart-stale").symlink_to(outside, target_is_directory=True)
    else:
        path.parent.mkdir()
        path.symlink_to(tmp_path / "missing-target")

    clean, removed, conflicts = module.cleanup_legacy_codex_skills(
        legacy, {"dart-stale"}, check_only=True
    )

    assert not clean
    assert removed == 0
    assert conflicts == 1
    assert "is a symlink; preserved without reading" in capsys.readouterr().out
    assert path.is_symlink() or path.parent.is_symlink()


def test_legacy_cleanup_rejects_symlinked_root_without_external_delete(
    tmp_path, capsys
):
    module = _load_module()
    outside = tmp_path / "outside-skills"
    external = outside / "dart-stale" / "SKILL.md"
    external.parent.mkdir(parents=True)
    external.write_text(
        module.add_auto_gen_header(
            '---\nname: dart-stale\ndescription: "DART Stale: fixture"\n---\n',
            ".claude/skills/dart-stale/SKILL.md",
        )
    )
    legacy = tmp_path / ".codex" / "skills"
    legacy.parent.mkdir(parents=True)
    legacy.symlink_to(outside, target_is_directory=True)
    original = external.read_text()

    clean, removed, conflicts = module.cleanup_legacy_codex_skills(
        legacy, {"dart-stale"}
    )

    assert not clean
    assert (removed, conflicts) == (0, 1)
    assert "legacy skill root is a symlink" in capsys.readouterr().out
    assert external.read_text() == original


def test_legacy_cleanup_rejects_non_directory_root(tmp_path, capsys):
    module = _load_module()
    legacy = tmp_path / ".codex" / "skills"
    legacy.parent.mkdir(parents=True)
    legacy.write_text("not a directory\n")

    clean, removed, conflicts = module.cleanup_legacy_codex_skills(legacy, set())

    assert not clean
    assert (removed, conflicts) == (0, 1)
    assert "legacy skill root is not a directory" in capsys.readouterr().out


def test_sync_all_preserves_legacy_catalog_when_canonical_preflight_fails(
    tmp_path, monkeypatch
):
    module = _load_module()
    _, _, target = _temporary_skill_repo(tmp_path, module)
    _write_skill(
        target / "dart-test" / "SKILL.md",
        "dart-test",
        "DART Test: unowned collision",
    )
    legacy_skill = tmp_path / ".codex" / "skills" / "dart-test" / "SKILL.md"
    legacy_skill.parent.mkdir(parents=True)
    legacy_skill.write_text(
        module.add_auto_gen_header(
            '---\nname: dart-test\ndescription: "DART Test: generated"\n---\n',
            ".claude/skills/dart-test/SKILL.md",
        )
    )
    _stub_sync_all_validators(module, monkeypatch)

    assert not module.sync_all()
    assert legacy_skill.exists()


def test_unowned_legacy_canonical_name_is_preserved_and_blocks_sync_all(
    tmp_path, monkeypatch, capsys
):
    module = _load_module()
    skill_source, command_source, target = _temporary_skill_repo(tmp_path, module)
    module.sync_codex_skills(skill_source, command_source, target)
    legacy_skill = tmp_path / ".codex" / "skills" / "dart-test" / "SKILL.md"
    _write_skill(
        legacy_skill,
        "dart-test",
        "DART Test: unowned legacy collision",
    )
    original_content = legacy_skill.read_text()
    _stub_sync_all_validators(module, monkeypatch)

    assert not module.sync_all()
    assert legacy_skill.read_text() == original_content
    assert "CONFLICT: dart-test/SKILL.md is unowned" in capsys.readouterr().out


def test_rendered_codex_command_skill_has_current_metadata_and_catalog_path(tmp_path):
    module = _load_module()
    command = tmp_path / "dart-example.md"
    _write_command(command, ["AGENTS.md"])

    rendered = module.render_codex_command_skill(command)
    metadata = module.parse_skill_frontmatter(rendered)

    assert metadata == {
        "name": "dart-example",
        "description": "DART Example: docs",
    }
    assert ".agents/skills/" in rendered
    assert ".codex/skills/" not in rendered


def test_skill_reference_guard_detects_stale_pixi_task_and_repo_path(tmp_path):
    module = _load_module()
    (tmp_path / "pixi.toml").write_text('[tasks]\nconfig = "true"\n')
    skill_path = tmp_path / ".claude" / "skills" / "dart-test" / "SKILL.md"
    _write_skill(skill_path)
    skill_path.write_text(
        skill_path.read_text()
        + "\n`pixi run configure`\n\nSee `dart/io/Missing.hpp`.\n"
    )
    module.get_repo_root = lambda: tmp_path

    errors = module.skill_repository_reference_errors(tmp_path)

    assert any("Pixi task `configure` does not exist" in error for error in errors)
    assert any(
        "repo path `dart/io/Missing.hpp` does not exist" in error for error in errors
    )


def test_skill_reference_guard_rejects_existing_repo_path_escape(tmp_path):
    module = _load_module()
    (tmp_path / "pixi.toml").write_text('[tasks]\nconfig = "true"\n')
    outside = tmp_path.parent / "outside.md"
    outside.write_text("outside\n")
    skill_path = tmp_path / ".claude" / "skills" / "dart-test" / "SKILL.md"
    _write_skill(skill_path)
    skill_path.write_text(skill_path.read_text() + "\n`docs/../../outside.md`\n")
    module.get_repo_root = lambda: tmp_path

    errors = module.skill_repository_reference_errors(tmp_path)

    assert any("escapes the repository" in error for error in errors)


def test_skill_reference_guard_ignores_other_branch_section(tmp_path):
    module = _load_module()
    (tmp_path / "pixi.toml").write_text('[tasks]\nconfig = "true"\n')
    skill_path = tmp_path / ".claude" / "skills" / "dart-test" / "SKILL.md"
    _write_skill(skill_path)
    skill_path.write_text(
        skill_path.read_text()
        + "\n## DART 6 (release-6.20)\n\n"
        + "`pixi run release-only-task` and `dart/io/release_only.hpp`\n\n"
        + "### Details\n\nThe release-only section continues.\n\n"
        + "```sh\n# a fenced comment is not a heading\n"
        + "pixi run still-release-only\n```\n\n"
        + "## DART 7 (`main`)\n\n`pixi run config`\n"
    )
    module.get_repo_root = lambda: tmp_path

    assert module.skill_repository_reference_errors(tmp_path, profile="main") == []
    release_errors = module.skill_repository_reference_errors(
        tmp_path, profile="release-6.20"
    )
    assert any("release-only-task" in error for error in release_errors)
    assert any("still-release-only" in error for error in release_errors)
    assert any("dart/io/release_only.hpp" in error for error in release_errors)


def test_branch_profile_manifest_wins_on_generic_release_topic_branch(tmp_path):
    module = _load_module()
    profile_path = tmp_path / "docs" / "ai" / "branch-profile.json"
    profile_path.parent.mkdir(parents=True)
    profile_path.write_text(json.dumps({"profile": "release-6.20"}))

    assert module.detect_branch_profile(tmp_path) == "release-6.20"


def test_checked_in_source_skill_tasks_and_paths_are_current():
    module = _load_module()

    assert module.skill_repository_reference_errors(ROOT) == []
    build_skill = (ROOT / ".claude" / "skills" / "dart-build" / "SKILL.md").read_text()
    io_skill = (ROOT / ".claude" / "skills" / "dart-io" / "SKILL.md").read_text()
    ci_skill = (ROOT / ".claude" / "skills" / "dart-ci" / "SKILL.md").read_text()
    assert "pixi run configure" not in build_skill
    assert "dart/io/Read.hpp" not in io_skill
    assert "tests/unit/io/test_Read.cpp" not in io_skill
    assert "## Expected CI Times" not in ci_skill


def test_ai_tools_new_command_template_matches_enforced_structure():
    content = (ROOT / "docs" / "onboarding" / "ai-tools.md").read_text()
    template = content.split("### Adding a New Command", 1)[1].split(
        "### Adding a New Skill", 1
    )[0]

    assert "argument-hint:" in template
    assert "## Required Reading" in template
    assert "## Workflow" in template
    assert "## Output" in template
    assert "agent: build" not in template


def test_shared_agent_surfaces_override_global_ignore_rules():
    shared_paths = (
        ".agents/AGENTS.md",
        ".agents/skills/.dart-generated.json",
        ".agents/skills/dart-build/SKILL.md",
        ".codex/AGENTS.md",
        ".codex/config.toml",
        ".codex/hooks.json",
        ".codex/agents/dart_scout.toml",
    )

    for path in shared_paths:
        result = subprocess.run(
            ["git", "check-ignore", "--no-index", "-q", "--", path],
            cwd=ROOT,
        )
        assert result.returncode == 1, f"shared agent surface is ignored: {path}"


def test_agent_allowlists_keep_private_payloads_ignored():
    private_paths = (
        ".agents/skills/private.txt",
        ".agents/skills/dart-local/private.txt",
        ".agents/skills/dart-local/nested/private.key",
        ".codex/agents/private.txt",
        ".codex/agents/private/runtime.key",
    )

    for path in private_paths:
        result = subprocess.run(
            ["git", "check-ignore", "--no-index", "-q", "--", path],
            cwd=ROOT,
        )
        assert result.returncode == 0, f"private agent payload is unignored: {path}"
