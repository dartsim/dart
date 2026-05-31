import importlib.util
import sys
from pathlib import Path


def _load_script(name):
    script = Path(__file__).resolve().parents[1] / "scripts" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, script)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_rigid_ipc_classifier_marks_paper_figures_as_visual_examples():
    module = _load_script("generate_rigid_ipc_fixture_manifest")

    row = module.classify_entry(
        "fixtures/paper-figures/01-expanding-lock-box.json", "fixture"
    )

    assert row["family"] == "paper-figure"
    assert row["dart_target_type"] == "example"
    assert row["visual_evidence_requirement"] == "headless-filament-required"


def test_rigid_ipc_classifier_marks_ccd_data_as_tests():
    module = _load_script("generate_rigid_ipc_fixture_manifest")

    row = module.classify_entry(
        "tests/data/wrecking-ball/ccd-test-001.json", "test-data"
    )

    assert row["family"] == "ccd-data-wrecking-ball"
    assert row["dart_target_type"] == "test"
    assert "time of impact" in row["expected_invariant"]


def test_rigid_ipc_manifest_marks_audited_root_ccd_rows_implemented(tmp_path):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = tmp_path / "tests" / "data" / "ccd-test-000.json"
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"type":"ee"}')

    row = module.row_for_path("tests/data/ccd-test-000.json", "test-data", tmp_path)

    assert row["status"] == "implemented"
    assert "EvaluatesAuditedUpstreamRootCcdRowsAsMisses" in row["dart_artifact"]
    assert "full-step miss outcome" in row["expected_invariant"]
    assert "corpus-scale interval-root parity remains tracked" in row["notes_or_gap"]


def test_rigid_ipc_manifest_leaves_uncovered_ccd_rows_planned(tmp_path):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = tmp_path / "tests" / "data" / "kinematic" / "ccd-test-007.json"
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"type":"ee"}')

    row = module.row_for_path(
        "tests/data/kinematic/ccd-test-007.json", "test-data", tmp_path
    )

    assert row["status"] == "planned"


def test_rigid_ipc_manifest_marks_first_kinematic_ccd_rows_implemented(tmp_path):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = tmp_path / "tests" / "data" / "kinematic" / "ccd-test-000.json"
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"type":"ee"}')

    row = module.row_for_path(
        "tests/data/kinematic/ccd-test-000.json", "test-data", tmp_path
    )

    assert row["status"] == "implemented"
    assert "EvaluatesAuditedKinematicRowsWithoutZeroTimeHits" in row["dart_artifact"]
    assert "zero-time hits" in row["expected_invariant"]


def test_rigid_ipc_asset_normalization_prefers_mesh_root(tmp_path):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    (tmp_path / "meshes").mkdir()
    (tmp_path / "meshes" / "cube.obj").write_text("o cube\n")

    assert module.normalize_asset("cube.obj", tmp_path) == "meshes/cube.obj"


def test_rigid_ipc_validator_rejects_unclassified_rows():
    module = _load_script("check_rigid_ipc_fixture_manifest")
    manifest = {
        "schema_version": 1,
        "source": {"commit": module.EXPECTED_COMMIT},
        "summary": {
            "source_kind_counts": {"fixture": 1},
            "family_counts": {"unclassified": 1},
            "target_type_counts": {"unclassified": 1},
        },
        "entries": [
            {
                "upstream_path": "fixtures/bad.json",
                "upstream_commit": module.EXPECTED_COMMIT,
                "source_kind": "fixture",
                "alias_of": "",
                "family": "unclassified",
                "topic": "missing",
                "priority": "P0",
                "dart_target_type": "unclassified",
                "status": "planned",
                "dart_artifact": "missing",
                "required_assets_or_importer": [],
                "expected_invariant": "missing",
                "dart_command_or_ctest_or_benchmark": "missing",
                "visual_evidence_requirement": "not-required",
                "benchmark_profile_artifact": "not-required",
                "notes_or_gap": "test row",
            }
        ],
    }

    errors = module.validate_manifest(manifest, strict_counts=False)

    assert any("dart_target_type is unclassified" in error for error in errors)
    assert any("missing family" in error for error in errors)
