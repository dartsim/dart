import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "check_plan083_cpu_scene_corpus.py"
MANIFEST = (
    ROOT
    / "docs"
    / "plans"
    / "083-unified-newton-barrier-multibody"
    / "cpu-scene-corpus.json"
)


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "check_plan083_cpu_scene_corpus",
        SCRIPT,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_plan083_cpu_scene_corpus_manifest_is_complete() -> None:
    module = _load_module()
    manifest = module.load_manifest(MANIFEST)

    assert module.validate_manifest(manifest) == []


def test_plan083_cpu_scene_corpus_requires_py_demo_commands(monkeypatch) -> None:
    module = _load_module()
    monkeypatch.setattr(module, "EXPECTED_ROW_IDS", {"unb-fig-01"})
    monkeypatch.setattr(module, "PY_DEMO_ROW_IDS", {"unb-fig-01"})
    monkeypatch.setattr(module, "BENCHMARK_ROW_IDS", set())

    manifest = {
        "schema_version": 1,
        "source": {},
        "summary": {
            "row_count": 1,
            "status_counts": {"in-progress": 1},
            "target_type_counts": {"py-demo": 1},
        },
        "scenes": [
            {
                "row_id": "unb-fig-01",
                "source_ref": "Fig. 1",
                "dart_target_type": "py-demo",
                "priority": "P0",
                "status": "in-progress",
                "py_demo_category": "not-required",
                "py_demo_scene_ids": [],
                "smoke_command": "not-required",
                "long_horizon_visual_command": "not-required",
                "visual_evidence_artifact": "not-required",
                "benchmark_command": "not-required",
                "benchmark_profile_artifact": "not-required",
                "expected_invariant": "finite",
                "limitation_status": "test limitation",
                "notes_or_gap": "test rationale",
            }
        ],
    }

    errors = module.validate_manifest(manifest)

    assert "unb-fig-01: py-demo row needs a scene id" in errors
    assert "unb-fig-01: py-demo row needs a category" in errors
    assert "unb-fig-01: py-demo row needs a py-demos smoke" in errors
    assert "unb-fig-01: py-demo row needs a capture command" in errors
    assert "unb-fig-01: py-demo row needs visual evidence path" in errors


def test_plan083_cpu_scene_corpus_requires_benchmark_packet(monkeypatch) -> None:
    module = _load_module()
    monkeypatch.setattr(module, "EXPECTED_ROW_IDS", {"unb-fig-24"})
    monkeypatch.setattr(module, "PY_DEMO_ROW_IDS", set())
    monkeypatch.setattr(module, "BENCHMARK_ROW_IDS", {"unb-fig-24"})

    manifest = {
        "schema_version": 1,
        "source": {},
        "summary": {
            "row_count": 1,
            "status_counts": {"planned": 1},
            "target_type_counts": {"benchmark-report": 1},
        },
        "scenes": [
            {
                "row_id": "unb-fig-24",
                "source_ref": "Fig. 24",
                "dart_target_type": "benchmark-report",
                "priority": "P2",
                "status": "planned",
                "py_demo_category": "not-required",
                "py_demo_scene_ids": [],
                "smoke_command": "not-required",
                "long_horizon_visual_command": "not-required",
                "visual_evidence_artifact": "not-required",
                "benchmark_command": "not-required",
                "benchmark_profile_artifact": "not-required",
                "expected_invariant": "timing fields",
                "limitation_status": "test limitation",
                "notes_or_gap": "test rationale",
            }
        ],
    }

    errors = module.validate_manifest(manifest)

    assert "unb-fig-24: benchmark row needs a command" in errors
    assert "unb-fig-24: benchmark row needs a profile path" in errors
