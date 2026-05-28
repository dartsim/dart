import importlib.util
import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "check_ipc_scene_manifest.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "check_ipc_scene_manifest",
        SCRIPT,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _scene_row(upstream_path, alias_of):
    return {
        "upstream_path": upstream_path,
        "upstream_commit": "test-commit",
        "alias_of": alias_of,
        "family": "tutorial",
        "topic": "relative-upstream-dir",
        "priority": "P1",
        "dart_target_type": "test",
        "status": "planned",
        "dart_artifact": "test fixture",
        "required_assets_or_importer": ["txt-scene"],
        "expected_invariant": "relative upstream-dir aliases validate",
        "dart_command_or_ctest_or_benchmark": "pytest",
        "visual_evidence_requirement": "not-required",
        "benchmark_profile_artifact": "not-required",
        "notes_or_gap": "test row",
    }


def test_relative_upstream_dir_validates_symlink_aliases(tmp_path, monkeypatch):
    module = _load_module()
    upstream_dir = tmp_path / "IPC"
    scene_dir = upstream_dir / "input" / "tutorial"
    scene_dir.mkdir(parents=True)
    target = scene_dir / "target.txt"
    alias = scene_dir / "alias.txt"
    target.write_text("target scene\n", encoding="utf-8")
    try:
        os.symlink("target.txt", alias)
    except OSError as exc:
        pytest.skip(f"symlink creation is unavailable: {exc}")

    monkeypatch.setattr(module, "EXPECTED_COMMIT", "test-commit")
    monkeypatch.setattr(module, "EXPECTED_SCENE_COUNT", 1)
    monkeypatch.setattr(module, "EXPECTED_FAMILY_COUNTS", {"tutorial": 1})
    monkeypatch.setattr(module, "upstream_head", lambda _: "test-commit")
    monkeypatch.setattr(
        module,
        "tracked_txt_paths",
        lambda _: ["input/tutorial/alias.txt"],
    )
    monkeypatch.chdir(tmp_path)

    manifest = {
        "summary": {
            "family_counts": {"tutorial": 1},
            "target_type_counts": {"test": 1},
        },
        "scenes": [
            _scene_row(
                upstream_path="input/tutorial/alias.txt",
                alias_of="input/tutorial/target.txt",
            )
        ],
    }
    errors = []

    module.check_scene_rows(manifest, Path("IPC"), errors)

    assert errors == []
