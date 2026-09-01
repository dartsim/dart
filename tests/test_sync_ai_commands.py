"""Regression tests for safe AI adapter generation."""

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "sync_ai_commands.py"
SPEC = importlib.util.spec_from_file_location("sync_ai_commands", SCRIPT)
assert SPEC and SPEC.loader
sync = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(sync)


def _sources(root: Path) -> tuple[Path, Path]:
    skills = root / ".claude" / "skills"
    commands = root / ".claude" / "commands"
    skill = skills / "dart-alpha" / "SKILL.md"
    skill.parent.mkdir(parents=True, exist_ok=True)
    commands.mkdir(parents=True, exist_ok=True)
    skill.write_text(
        '---\nname: dart-alpha\ndescription: "DART Alpha: focused test skill"\n'
        "---\n\n# Alpha\n"
    )
    return skills, commands


def _run(root: Path, monkeypatch, *, check_only: bool = False):
    skills, commands = _sources(root)
    target = root / ".agents" / "skills"
    monkeypatch.setattr(sync, "get_repo_root", lambda: root)
    return sync.sync_codex_skills(skills, commands, target, check_only=check_only)


def test_manifest_owned_cleanup_preserves_unrelated_skill(tmp_path, monkeypatch):
    result = _run(tmp_path, monkeypatch)
    assert result[1] == 1
    target = tmp_path / ".agents" / "skills"
    manifest_path = target / sync.GENERATED_MANIFEST
    assert json.loads(manifest_path.read_text())["paths"] == ["dart-alpha/SKILL.md"]

    unrelated = target / "personal-skill" / "SKILL.md"
    unrelated.parent.mkdir()
    unrelated.write_text("personal content\n")
    (tmp_path / ".claude" / "skills" / "dart-alpha" / "SKILL.md").unlink()
    (tmp_path / ".claude" / "skills" / "dart-alpha").rmdir()

    # Reuse the now-empty source directories without recreating dart-alpha.
    monkeypatch.setattr(sync, "get_repo_root", lambda: tmp_path)
    result = sync.sync_codex_skills(
        tmp_path / ".claude" / "skills",
        tmp_path / ".claude" / "commands",
        target,
    )
    assert result[1] == 0
    assert not (target / "dart-alpha").exists()
    assert unrelated.read_text() == "personal content\n"
    assert json.loads(manifest_path.read_text())["paths"] == []
    (tmp_path / ".opencode" / "command").mkdir(parents=True)
    assert sync.check_capability_parity(tmp_path)


def test_refuses_to_overwrite_unowned_skill(tmp_path, monkeypatch):
    skills, commands = _sources(tmp_path)
    target = tmp_path / ".agents" / "skills"
    existing = target / "dart-alpha" / "SKILL.md"
    existing.parent.mkdir(parents=True)
    existing.write_text("user owned\n")
    monkeypatch.setattr(sync, "get_repo_root", lambda: tmp_path)

    result = sync.sync_codex_skills(skills, commands, target)

    assert not result[0]
    assert result[1] == -1
    assert existing.read_text() == "user owned\n"


def test_auto_generated_header_does_not_replace_manifest_ownership(
    tmp_path, monkeypatch
):
    skills, commands = _sources(tmp_path)
    target = tmp_path / ".agents" / "skills"
    existing = target / "dart-alpha" / "SKILL.md"
    existing.parent.mkdir(parents=True)
    existing.write_text(
        sync.add_auto_gen_header("unowned\n", ".claude/skills/dart-alpha/SKILL.md")
    )
    before = existing.read_bytes()
    monkeypatch.setattr(sync, "get_repo_root", lambda: tmp_path)

    result = sync.sync_codex_skills(skills, commands, target)

    assert result[1] == -1
    assert existing.read_bytes() == before


def test_collision_preflight_prevents_partial_updates(tmp_path, monkeypatch):
    skills, commands = _sources(tmp_path)
    beta = skills / "dart-beta" / "SKILL.md"
    beta.parent.mkdir()
    beta.write_text(
        '---\nname: dart-beta\ndescription: "DART Beta: focused test skill"\n'
        "---\n\n# Beta\n"
    )
    target = tmp_path / ".agents" / "skills"
    alpha = target / "dart-alpha" / "SKILL.md"
    alpha.parent.mkdir(parents=True)
    alpha.write_text(
        sync.add_auto_gen_header("old alpha\n", ".claude/skills/dart-alpha/SKILL.md")
    )
    collision = target / "dart-beta" / "SKILL.md"
    collision.parent.mkdir()
    collision.write_text("user beta\n")
    (target / sync.GENERATED_MANIFEST).write_text(
        sync.generated_manifest_content({"dart-alpha/SKILL.md"})
    )
    before = alpha.read_bytes()
    monkeypatch.setattr(sync, "get_repo_root", lambda: tmp_path)

    result = sync.sync_codex_skills(skills, commands, target)

    assert result[1] == -1
    assert alpha.read_bytes() == before
    assert collision.read_text() == "user beta\n"


def test_unowned_generated_looking_extra_blocks_without_writes(tmp_path, monkeypatch):
    skills, commands = _sources(tmp_path)
    target = tmp_path / ".agents" / "skills"
    stale = target / "dart-stale" / "SKILL.md"
    stale.parent.mkdir(parents=True)
    stale.write_text(
        sync.add_auto_gen_header("stale\n", ".claude/skills/dart-stale/SKILL.md")
    )
    monkeypatch.setattr(sync, "get_repo_root", lambda: tmp_path)

    result = sync.sync_codex_skills(skills, commands, target)

    assert result[1] == -1
    assert not (target / "dart-alpha").exists()
    assert stale.exists()


def test_check_mode_detects_manifest_drift(tmp_path, monkeypatch):
    assert _run(tmp_path, monkeypatch)[1] == 1
    manifest = tmp_path / ".agents" / "skills" / sync.GENERATED_MANIFEST
    data = json.loads(manifest.read_text())
    data["paths"] = []
    manifest.write_text(json.dumps(data) + "\n")

    result = _run(tmp_path, monkeypatch, check_only=True)

    assert not result[0]


def test_unsafe_manifest_path_is_rejected_without_deletion(tmp_path, monkeypatch):
    skills, commands = _sources(tmp_path)
    target = tmp_path / ".agents" / "skills"
    target.mkdir(parents=True)
    outside = tmp_path / "outside.txt"
    outside.write_text("keep\n")
    (target / sync.GENERATED_MANIFEST).write_text(
        json.dumps(
            {
                "schema_version": sync.GENERATED_MANIFEST_SCHEMA_VERSION,
                "generator": "scripts/sync_ai_commands.py",
                "paths": ["../../outside.txt"],
            }
        )
    )
    monkeypatch.setattr(sync, "get_repo_root", lambda: tmp_path)

    result = sync.sync_codex_skills(skills, commands, target)

    assert not result[0]
    assert outside.read_text() == "keep\n"


def test_non_object_manifest_returns_error_without_traceback(tmp_path, monkeypatch):
    skills, commands = _sources(tmp_path)
    target = tmp_path / ".agents" / "skills"
    target.mkdir(parents=True)
    (target / sync.GENERATED_MANIFEST).write_text("[]\n")
    monkeypatch.setattr(sync, "get_repo_root", lambda: tmp_path)

    result = sync.sync_codex_skills(skills, commands, target)

    assert not result[0]
    assert result[1] == -1


def test_boolean_manifest_schema_version_is_rejected(tmp_path, monkeypatch):
    skills, commands = _sources(tmp_path)
    target = tmp_path / ".agents" / "skills"
    target.mkdir(parents=True)
    (target / sync.GENERATED_MANIFEST).write_text(
        json.dumps(
            {
                "schema_version": True,
                "generator": "scripts/sync_ai_commands.py",
                "paths": [],
            }
        )
    )
    monkeypatch.setattr(sync, "get_repo_root", lambda: tmp_path)

    result = sync.sync_codex_skills(skills, commands, target)

    assert not result[0]
    assert result[1] == -1


def test_symlinked_generated_target_is_rejected_without_external_write(
    tmp_path, monkeypatch
):
    skills, commands = _sources(tmp_path)
    target = tmp_path / ".agents" / "skills"
    target.mkdir(parents=True)
    outside = tmp_path / "outside"
    outside.mkdir()
    external_skill = outside / "SKILL.md"
    external_skill.write_text("external content\n")
    (target / "dart-alpha").symlink_to(outside, target_is_directory=True)
    (target / sync.GENERATED_MANIFEST).write_text(
        sync.generated_manifest_content({"dart-alpha/SKILL.md"})
    )
    monkeypatch.setattr(sync, "get_repo_root", lambda: tmp_path)

    result = sync.sync_codex_skills(skills, commands, target)

    assert not result[0]
    assert result[1] == -1
    assert external_skill.read_text() == "external content\n"


def test_symlinked_generated_root_is_rejected_without_external_write(
    tmp_path, monkeypatch
):
    skills, commands = _sources(tmp_path)
    agents_dir = tmp_path / ".agents"
    agents_dir.mkdir()
    unrelated = tmp_path / "unrelated-owned-dir"
    unrelated.mkdir()
    external_skill = unrelated / "dart-alpha" / "SKILL.md"
    external_skill.parent.mkdir()
    external_skill.write_text("unrelated content\n")
    target = agents_dir / "skills"
    target.symlink_to(unrelated, target_is_directory=True)
    monkeypatch.setattr(sync, "get_repo_root", lambda: tmp_path)

    result = sync.sync_codex_skills(skills, commands, target)

    assert not result[0]
    assert result[1] == -1
    assert external_skill.read_text() == "unrelated content\n"
    assert not (unrelated / sync.GENERATED_MANIFEST).exists()


def test_command_renderer_drops_tool_specific_agent_metadata(tmp_path, monkeypatch):
    monkeypatch.setattr(sync, "get_repo_root", lambda: tmp_path)
    command = tmp_path / ".claude" / "commands" / "dart-demo.md"
    command.parent.mkdir(parents=True)
    command.write_text(
        '---\ndescription: demonstrate routing\nargument-hint: "<x>"\n'
        "agent: build\n---\n\n## Required Reading\n\n@AGENTS.md\n\n"
        "## Workflow\n\nRun it.\n\n## Output\n\nEvidence.\n"
    )

    rendered = sync.render_codex_command_skill(command)

    assert "agent: build" not in rendered
    assert "name: dart-demo" in rendered


def test_unknown_capability_mention_scan_flags_absent_workflow(tmp_path):
    commands = tmp_path / ".claude" / "commands"
    commands.mkdir(parents=True)
    (commands / "dart-sample.md").write_text(
        "Use `dart-nonexistent-workflow`, `/dart-missing-slash`, and "
        "`$dart-missing-codex decide` with the `dart-demos` app, "
        "`/dart-sample`, and `$dart-sample audit`.\n"
        "```bash\n/dart-missing-fenced x\n$dart-sample run\n"
        "ls /tmp/dart-not-a-capability\n```\n"
        "Also try **/dart-missing-bold**, _/dart-missing-italic_, and\n"
        "_$dart-sample_ forms. Use dart-missing-bare for plain prose.\n",
        encoding="utf-8",
    )

    errors = sync.unknown_capability_mention_errors(tmp_path, {"dart-sample"})

    prefix = ".claude/commands/dart-sample.md: unknown capability "
    assert errors == [
        prefix + "`dart-missing-bare`",
        prefix + "`dart-missing-bold`",
        prefix + "`dart-missing-codex`",
        prefix + "`dart-missing-fenced`",
        prefix + "`dart-missing-italic`",
        prefix + "`dart-missing-slash`",
        prefix + "`dart-nonexistent-workflow`",
    ]


def test_checked_in_sources_mention_only_existing_capabilities():
    command_names = sync.list_command_names(ROOT / ".claude" / "commands")
    skill_names, _ = sync.list_skill_names(ROOT / ".claude" / "skills")

    assert (
        sync.unknown_capability_mention_errors(ROOT, command_names | skill_names) == []
    )


def test_unknown_capability_scan_allowlists_cmake_component_names(tmp_path):
    commands = tmp_path / ".claude" / "commands"
    commands.mkdir(parents=True)
    (tmp_path / "dart" / "collision").mkdir(parents=True)
    (tmp_path / "dart" / "CMakeLists.txt").write_text(
        "set(components dart-gui-osg dart-utils-urdf)\n", encoding="utf-8"
    )
    (tmp_path / "dart" / "collision" / "CMakeLists.txt").write_text(
        "add_library(dart-collision-native)\n", encoding="utf-8"
    )
    (commands / "dart-sample.md").write_text(
        "Link `dart-gui-osg`, `dart-utils-urdf`, and `dart-collision-native` "
        "from `dart-sample`.\n",
        encoding="utf-8",
    )

    assert sync.unknown_capability_mention_errors(tmp_path, {"dart-sample"}) == []


def test_unknown_capability_scan_flags_prefixed_component_names(tmp_path):
    commands = tmp_path / ".claude" / "commands"
    commands.mkdir(parents=True)
    (tmp_path / "dart").mkdir()
    (tmp_path / "dart" / "CMakeLists.txt").write_text(
        "set(components dart-gui-osg)\n", encoding="utf-8"
    )
    (commands / "dart-sample.md").write_text(
        "Link `dart-gui-osg` in prose, but never run `/dart-gui-osg` or "
        "`$dart-gui-osg` from `dart-sample`.\n",
        encoding="utf-8",
    )

    errors = sync.unknown_capability_mention_errors(tmp_path, {"dart-sample"})

    assert errors == [
        ".claude/commands/dart-sample.md: unknown capability `dart-gui-osg`"
    ]


def test_component_allowlist_does_not_mask_current_capabilities(tmp_path):
    commands = tmp_path / ".claude" / "commands"
    commands.mkdir(parents=True)
    (tmp_path / "dart").mkdir()
    (tmp_path / "dart" / "CMakeLists.txt").write_text(
        "set(components dart-collide)\n", encoding="utf-8"
    )
    (commands / "dart-sample.md").write_text(
        "Use `dart-collide` and `/dart-collide` from `dart-sample`.\n",
        encoding="utf-8",
    )

    with_capability = sync.unknown_capability_mention_errors(
        tmp_path, {"dart-sample", "dart-collide"}
    )
    without_capability = sync.unknown_capability_mention_errors(
        tmp_path, {"dart-sample"}
    )

    assert with_capability == []
    assert without_capability == [
        ".claude/commands/dart-sample.md: unknown capability `dart-collide`"
    ]
