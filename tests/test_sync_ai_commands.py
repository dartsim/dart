"""Tests for scripts/sync_ai_commands.py."""

import importlib.util
import sys
from pathlib import Path

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
