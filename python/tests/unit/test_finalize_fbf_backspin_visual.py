import argparse
import copy
import csv
import importlib.util
import io
import json
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts/finalize_fbf_backspin_visual.py"
TEST_EXECUTABLE = Path(sys.executable).resolve()


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "finalize_fbf_backspin_visual", SCRIPT
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_v3_defaults_bind_the_checker_texture_contract():
    module = _load_module()

    assert module.DEFAULT_BUNDLE.name == "fig03_backspin_current_v3"
    assert module.SCHEMA_VERSION == "dart.fbf_backspin_visual_bundle/v3"
    assert module.MANUAL_SCHEMA_VERSION == ("dart.fbf_backspin_manual_inspection/v3")
    assert module.EXPECTED_CONFIGURATION["orientation_cue"] == (
        "renderer-applied high-contrast 6x4 checker texture with coral "
        "registration tile"
    )
    assert len(module.CAPTURE_PATHS) == 148
    assert len(module.STAGING_PATHS) == 140
    assert len(module.DURABLE_STILL_PATHS) == 3
    assert len(module.EXPECTED_FINAL_PATHS) == 20
    assert module.MANUAL_ARTIFACT_PATHS >= module.DURABLE_STILL_PATHS
    assert not (module.STAGING_PATHS & module.EXPECTED_FINAL_PATHS)


def test_capture_command_and_provenance_bind_runtime_checker_resources(tmp_path):
    module = _load_module()
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    module.write_json(bundle / "run-summary.json", {"pass": True})
    (bundle / "capture.stdout.txt").write_text("renderer log\n", encoding="utf-8")
    (bundle / "capture.stderr.txt").write_text("", encoding="utf-8")
    python = TEST_EXECUTABLE
    runner = TEST_EXECUTABLE
    demo = TEST_EXECUTABLE
    ffmpeg = TEST_EXECUTABLE
    ffprobe = TEST_EXECUTABLE
    resources = module._checker_runtime_resource_identity()
    argv = module._capture_argv(python, runner, demo, bundle, ffmpeg, ffprobe)
    payload = {
        "argv": argv,
        "returncode": 0,
        "run_summary_path": "run-summary.json",
        "run_summary_sha256": module.sha256(bundle / "run-summary.json"),
        "run_summary_validated": True,
        "stdout_path": "capture.stdout.txt",
        "stdout_sha256": module.sha256(bundle / "capture.stdout.txt"),
        "stderr_path": "capture.stderr.txt",
        "stderr_sha256": module.sha256(bundle / "capture.stderr.txt"),
        "runtime_resources_before": resources,
        "runtime_resources_after": resources,
    }

    assert (
        module._validate_capture_provenance(
            payload,
            bundle=bundle,
            python=python,
            runner=runner,
            demo=demo,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
        )
        == payload
    )
    assert argv[2:5] == ["run", "--scenario", "backspin"]
    assert argv[-4:] == [
        "--python",
        str(python),
        "--out",
        str(bundle / "run-summary.json"),
    ]

    tampered = copy.deepcopy(payload)
    tampered["runtime_resources_before"]["backspin_checker_mesh"]["sha256"] = "0" * 64
    with pytest.raises(ValueError, match="capture-time provenance changed"):
        module._validate_capture_provenance(
            tampered,
            bundle=bundle,
            python=python,
            runner=runner,
            demo=demo,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
        )


def _trace_rows(module):
    rows = []
    for step in range(module.TOTAL_STEPS + 1):
        if step == 0:
            x = 0.0
            vx = 4.0
        elif step <= 48:
            x = step / 48.0
            vx = 1.0
        else:
            x = 1.0 - 0.02 * (step - 48)
            vx = -1.0
        rows.append(
            {
                "step": str(step),
                "time": str(step / 60.0),
                "scenario": "backspin",
                "solver": "exact_fbf",
                "body": "backspin_sphere_body",
                "x": str(x),
                "y": "0.0",
                "z": "0.25",
                "vx": str(vx),
                "vy": "0.0",
                "vz": "0.0",
                "up_z": "1.0",
                "contacts": "0" if step in (0, 120) else "1",
                "exact_solves": str(max(0, step - (step >= 120))),
                "warm_starts": str(max(0, step - 10)),
                "fallbacks": "0",
                "residual": "nan" if step == 0 else "5e-7",
                "status": "not_run" if step == 0 else "success",
            }
        )
    return rows


def _trace_text(module, rows=None):
    stream = io.StringIO()
    writer = csv.DictWriter(stream, fieldnames=module.TRACE_COLUMNS)
    writer.writeheader()
    writer.writerows(rows if rows is not None else _trace_rows(module))
    return stream.getvalue()


def test_trace_parser_accepts_bounded_translational_contract():
    module = _load_module()

    parsed = module.parse_trace_text(_trace_text(module))

    summary = parsed["summary"]
    assert summary["row_count"] == 131
    assert summary["maximum_forward_travel"]["step"] == 48
    assert summary["first_negative_vx"]["step"] == 49
    assert summary["contact_free_post_initial_steps"] == [120]
    assert summary["selected_panel_states"][0]["residual"] is None
    assert summary["signed_angular_direction_proven"] is False
    assert summary["strict_rigid_body_rest_proven"] is False
    assert parsed["solver_projection"][120]["contacts"] == 0


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda rows: rows.pop(), "expected 131 rows"),
        (lambda rows: rows[10].update(step="11"), "noncontiguous step"),
        (lambda rows: rows[10].update(time="1.0"), "is not step/60"),
        (lambda rows: rows[10].update(scenario="turntable"), "scenario"),
        (lambda rows: rows[10].update(solver="boxed_lcp"), "identity mismatch"),
        (lambda rows: rows[10].update(x="nan"), "nonfinite x"),
        (lambda rows: rows[10].update(status="failure"), "status"),
        (lambda rows: rows[10].update(residual="1.1e-6"), "exceeds"),
        (lambda rows: rows[10].update(fallbacks="1"), "fallback"),
        (
            lambda rows: [row.update(vx="1.0") for row in rows[1:]],
            "never reversed",
        ),
        (lambda rows: rows[-1].update(contacts="0"), "not in contact"),
    ],
)
def test_trace_parser_rejects_invalid_contracts(mutation, message):
    module = _load_module()
    rows = _trace_rows(module)
    mutation(rows)

    with pytest.raises(ValueError, match=message):
        module.parse_trace_text(_trace_text(module, rows))


def test_trace_parser_rejects_noncanonical_header():
    module = _load_module()
    text = _trace_text(module).replace("step,time", "time,step", 1)

    with pytest.raises(ValueError, match="unexpected trace columns"):
        module.parse_trace_text(text)


def test_payload_hash_refuses_nonfinite_json():
    module = _load_module()

    with pytest.raises(ValueError, match="Out of range float"):
        module._payload_sha256({"bad": float("nan")})


def _manual_record(module, root):
    artifacts = []
    for index, relative in enumerate(sorted(module.MANUAL_ARTIFACT_PATHS)):
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(f"artifact-{index}".encode())
        artifacts.append(
            {
                "path": relative,
                "sha256": module.sha256(path),
                "observation": f"inspected {relative}",
            }
        )
    return {
        "schema_version": module.MANUAL_SCHEMA_VERSION,
        "manual_inspected": True,
        "pass": True,
        "verdicts": copy.deepcopy(module.MANUAL_VERDICTS),
        "representative_artifacts": artifacts,
    }


def _durable_still_fixture(module, root):
    frames = {}
    shots = []
    for step in module.DURABLE_STILL_STEPS:
        relative = f"backspin/frames/step_{step:06d}.png"
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(f"raw-backspin-{step}".encode())
        frames[str(step)] = {
            "path": str(path.resolve()),
            "sha256": module.sha256(path),
        }
        shots.append({"step": step, "path": str(path.resolve())})
    metadata = {
        "schedule": {"id": "backspin"},
        "timeline_validation": {"frames": frames},
    }
    module.write_json(root / "backspin/metadata.json", metadata)
    module.write_json(
        root / "run-summary.json",
        {"results": [copy.deepcopy(metadata)]},
    )
    module.write_json(root / "backspin/timeline.json", {"shots": shots})
    return module._materialize_durable_stills(root)


def test_manual_inspection_binds_exact_artifacts_and_nonclaims(tmp_path):
    module = _load_module()
    record = _manual_record(module, tmp_path)
    module.write_json(tmp_path / "manual-inspection.json", record)

    assert module.validate_manual_inspection(tmp_path) == record


def test_manual_checker_verdicts_and_nonclaims_are_frozen():
    module = _load_module()

    assert module.MANUAL_VERDICTS["checker_texture_legible"] is True
    assert module.MANUAL_VERDICTS["coral_registration_tile_visible"] is True
    assert module.MANUAL_VERDICTS["texture_loading_warnings_observed"] is False
    assert module.MANUAL_VERDICTS["signed_angular_direction_proven_by_media"] is False
    assert module.MANUAL_VERDICTS["paper_parity"] is False
    assert module.MANUAL_VERDICTS["timing_verdict"] is None
    assert module.MANUAL_VERDICTS["realtime_verdict"] is None


def test_durable_stills_bind_timeline_and_run_summary_hashes(tmp_path):
    module = _load_module()
    records = _durable_still_fixture(module, tmp_path)

    assert module._validate_durable_stills(tmp_path, records) == records
    assert [item["step"] for item in records] == [0, 1, 2]
    assert all(
        item["sha256"]
        == item["timeline_frame_sha256"]
        == item["run_summary_frame_sha256"]
        for item in records
    )


@pytest.mark.parametrize("mutation", ["missing", "tampered"])
def test_durable_stills_fail_closed_when_missing_or_tampered(tmp_path, mutation):
    module = _load_module()
    records = _durable_still_fixture(module, tmp_path)
    still = tmp_path / records[0]["path"]
    if mutation == "missing":
        still.unlink()
    else:
        still.write_bytes(b"tampered")

    with pytest.raises(ValueError, match="durable still"):
        module._validate_durable_stills(tmp_path, records)


def test_sealed_capture_provenance_binds_pruned_staging_and_stills(tmp_path):
    module = _load_module()
    records = _durable_still_fixture(module, tmp_path)
    for relative in module.STAGING_PATHS:
        path = tmp_path / relative
        if not path.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(relative.encode())
    manifest = module._build_staging_manifest(tmp_path)
    staging = {item["path"]: item for item in manifest["artifacts"]}
    executable = TEST_EXECUTABLE
    resources = module._checker_runtime_resource_identity()
    payload = {
        "argv": module._capture_argv(
            executable,
            executable,
            executable,
            tmp_path,
            executable,
            executable,
        ),
        "returncode": 0,
        "run_summary_path": "run-summary.json",
        "run_summary_sha256": module.sha256(tmp_path / "run-summary.json"),
        "run_summary_validated": True,
        "stdout_path": "capture.stdout.txt",
        "stdout_sha256": staging["capture.stdout.txt"]["sha256"],
        "stderr_path": "capture.stderr.txt",
        "stderr_sha256": staging["capture.stderr.txt"]["sha256"],
        "runtime_resources_before": resources,
        "runtime_resources_after": resources,
        "durable_stills": records,
        "pruned_staging": manifest,
    }
    module._prune_staging_outputs(tmp_path, manifest)

    assert (
        module._validate_capture_provenance(
            payload,
            bundle=tmp_path,
            python=executable,
            runner=executable,
            demo=executable,
            ffmpeg=executable,
            ffprobe=executable,
            sealed=True,
        )
        == payload
    )

    tampered = copy.deepcopy(payload)
    tampered["stdout_sha256"] = "0" * 64
    with pytest.raises(ValueError, match="capture-time provenance changed"):
        module._validate_capture_provenance(
            tampered,
            bundle=tmp_path,
            python=executable,
            runner=executable,
            demo=executable,
            ffmpeg=executable,
            ffprobe=executable,
            sealed=True,
        )


@pytest.mark.parametrize("mutation", ["promote", "wrong_hash", "extra_path"])
def test_manual_inspection_fails_closed(tmp_path, mutation):
    module = _load_module()
    record = _manual_record(module, tmp_path)
    if mutation == "promote":
        record["verdicts"]["signed_angular_direction_proven_by_media"] = True
    elif mutation == "wrong_hash":
        record["representative_artifacts"][0]["sha256"] = "0" * 64
    else:
        record["representative_artifacts"].append(
            {
                "path": "unexpected.png",
                "sha256": "0" * 64,
                "observation": "unexpected",
            }
        )
    module.write_json(tmp_path / "manual-inspection.json", record)

    with pytest.raises(ValueError):
        module.validate_manual_inspection(tmp_path)


def test_artifact_index_requires_exact_membership(tmp_path):
    module = _load_module()
    (tmp_path / "nested").mkdir()
    (tmp_path / "a.txt").write_text("a", encoding="utf-8")
    (tmp_path / "nested/b.txt").write_text("b", encoding="utf-8")
    index = module.artifact_index(tmp_path)

    module.validate_artifact_index(tmp_path, index)

    (tmp_path / "unexpected.txt").write_text("unexpected", encoding="utf-8")
    with pytest.raises(ValueError, match="membership mismatch"):
        module.validate_artifact_index(tmp_path, index)


def test_artifact_index_rejects_symlink(tmp_path):
    module = _load_module()
    target = tmp_path / "target.txt"
    target.write_text("target", encoding="utf-8")
    (tmp_path / "link.txt").symlink_to(target)

    with pytest.raises(ValueError, match="symlink"):
        module.artifact_index(tmp_path)


@pytest.mark.parametrize("entrypoint", ["finalize", "verify_only"])
def test_entrypoints_reject_root_and_ancestor_symlinks(tmp_path, entrypoint):
    module = _load_module()
    target = tmp_path / "target"
    target.mkdir()
    root_link = tmp_path / "bundle-link"
    root_link.symlink_to(target, target_is_directory=True)

    with pytest.raises(ValueError, match="bundle root is a symlink"):
        getattr(module, entrypoint)(argparse.Namespace(bundle=root_link))

    real_parent = tmp_path / "real-parent"
    (real_parent / "bundle").mkdir(parents=True)
    linked_parent = tmp_path / "linked-parent"
    linked_parent.symlink_to(real_parent, target_is_directory=True)
    with pytest.raises(ValueError, match="passes through a symlink"):
        getattr(module, entrypoint)(argparse.Namespace(bundle=linked_parent / "bundle"))


def test_fresh_bundle_membership_allows_capture_to_be_created(tmp_path):
    module = _load_module()

    module._validate_bundle_paths(
        tmp_path,
        complete=False,
        allow_missing_capture_provenance=True,
        allow_missing_capture=True,
    )

    with pytest.raises(ValueError, match="capture bundle is incomplete"):
        module._validate_bundle_paths(tmp_path, complete=False)


def test_fresh_capture_can_pause_only_for_manual_inspection(tmp_path):
    module = _load_module()
    for relative in module.CAPTURE_PATHS - {"manual-inspection.json"}:
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"captured")

    module._validate_bundle_paths(
        tmp_path,
        complete=False,
        allow_missing_manual=True,
    )

    with pytest.raises(ValueError, match=r"\['manual-inspection.json'\]"):
        module._validate_bundle_paths(tmp_path, complete=False)


def test_sealed_membership_rejects_surviving_staging_and_extra_files(tmp_path):
    module = _load_module()
    for relative in module.EXPECTED_FINAL_PATHS:
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"durable")

    module._validate_bundle_paths(tmp_path, complete=True)

    (tmp_path / "capture.stderr.txt").write_bytes(b"staging")
    with pytest.raises(ValueError, match="unexpected entries"):
        module._validate_bundle_paths(tmp_path, complete=True)
    (tmp_path / "capture.stderr.txt").unlink()

    (tmp_path / "unexpected.txt").write_bytes(b"extra")
    with pytest.raises(ValueError, match="unexpected entries"):
        module._validate_bundle_paths(tmp_path, complete=True)


def test_pruned_staging_manifest_is_exact_and_rejects_survivors(tmp_path):
    module = _load_module()
    for relative in module.STAGING_PATHS:
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(relative.encode())
    manifest = module._build_staging_manifest(tmp_path)

    module._validate_staging_manifest(tmp_path, manifest, require_absent=False)
    tampered = copy.deepcopy(manifest)
    tampered["artifacts"][0]["sha256"] = "0" * 64
    with pytest.raises(ValueError, match="staging artifact changed"):
        module._validate_staging_manifest(tmp_path, tampered, require_absent=False)

    module._prune_staging_outputs(tmp_path, manifest)
    module._validate_staging_manifest(tmp_path, manifest, require_absent=True)
    (tmp_path / "capture.stderr.txt").write_bytes(b"survived")
    with pytest.raises(ValueError, match="survived seal"):
        module._validate_staging_manifest(tmp_path, manifest, require_absent=True)


def test_bundle_transaction_restores_staging_after_seal_failure(tmp_path):
    module = _load_module()
    for relative in module.STAGING_PATHS:
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(relative.encode())
    manifest = module._build_staging_manifest(tmp_path)

    with pytest.raises(ValueError, match="injected seal failure"):
        with module._bundle_transaction(tmp_path):
            module._prune_staging_outputs(tmp_path, manifest)
            raise ValueError("injected seal failure")

    module._validate_staging_manifest(tmp_path, manifest, require_absent=False)


@pytest.mark.parametrize("replacement_kind", ["regular_file", "symlink"])
def test_bundle_transaction_restores_after_root_path_replacement(
    tmp_path, replacement_kind
):
    module = _load_module()
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    (bundle / "marker.txt").write_text("before\n", encoding="utf-8")
    outside = tmp_path / "outside"
    outside.mkdir()
    sentinel = outside / "sentinel.txt"
    sentinel.write_text("outside\n", encoding="utf-8")

    with pytest.raises(RuntimeError, match="injected replacement failure"):
        with module._bundle_transaction(bundle):
            shutil.rmtree(bundle)
            if replacement_kind == "regular_file":
                bundle.write_text("replacement\n", encoding="utf-8")
            else:
                bundle.symlink_to(outside, target_is_directory=True)
            raise RuntimeError("injected replacement failure")

    assert bundle.is_dir()
    assert not bundle.is_symlink()
    assert (bundle / "marker.txt").read_text(encoding="utf-8") == "before\n"
    assert sentinel.read_text(encoding="utf-8") == "outside\n"
    assert not list(tmp_path.glob(".bundle.backup-*"))


def test_fresh_capture_cleanup_removes_stale_manual_and_stills(tmp_path):
    module = _load_module()
    stale_paths = {
        "manual-inspection.json",
        *module.DURABLE_STILL_PATHS,
    }
    for relative in stale_paths:
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"stale")
    generated = tmp_path / "REPORT.md"
    generated.write_bytes(b"generated")

    module._clean_capture_outputs(tmp_path)

    assert not any((tmp_path / relative).exists() for relative in stale_paths)
    assert generated.read_bytes() == b"generated"


def test_solver_projection_comparison_is_exact():
    module = _load_module()
    projection = module.parse_trace_text(_trace_text(module))["solver_projection"]

    result = module._compare_solver_projections(projection, copy.deepcopy(projection))

    assert result["pass"] is True
    assert result["byte_identical"] is True
    assert result["full_state_trace_equivalence"] is False

    changed = copy.deepcopy(projection)
    changed[120]["contacts"] = 1
    with pytest.raises(ValueError, match="differs at 120"):
        module._compare_solver_projections(projection, changed)


def test_current_checker_texture_is_visual_only_by_source_contract():
    module = _load_module()

    result = module._validate_visual_only_source()

    assert result["pass"] is True
    assert result["shape"] == "MeshShape"
    assert result["aspect"] == "VisualAspect_only"
    assert result["mesh_uri"] == ("dart://sample/obj/fbf_backspin_checker_sphere.obj")
    assert result["texture_dimensions"] == [64, 32]
    assert result["checker_grid"] == [6, 4]
    assert result["palette"] == {
        "ivory": [244, 241, 228],
        "charcoal": [32, 36, 43],
        "coral": [255, 93, 115],
    }
    assert result["registration_tile"] == "coral"
    assert result["camera_home"] == {
        "eye": [-0.5, -1.25, 5.5],
        "center": [-0.5, 0.0, 0.2],
        "up": [0.0, 1.0, 0.0],
        "nearly_perpendicular_to_spin_axis": True,
    }
    assert result["mesh_vertex_count"] == 559
    assert result["mesh_uv_count"] == 559
    assert result["mesh_normal_count"] == 559
    assert result["mesh_triangle_count"] == 960
    assert result["mesh_outward_winding"] is True
    assert result["mesh_uv_triangles_non_degenerate"] is True
    assert result["texture_column_width_range"] == [10, 11]
    assert result["texture_row_height_range"] == [8, 8]
    assert result["checker_layout_validated"] is True
    assert result["registration_tile_coordinates"] == [0, 1]
    assert result["registration_tile_count"] == 1
    assert result["physical_shape_aspects"] == [
        "CollisionAspect",
        "DynamicsAspect",
    ]
    assert result["physics_neutral_by_source_contract"] is True
    assert result["full_state_equivalence_proven"] is False
    assert result["mesh_sha256"] == module.sha256(module.CHECKER_MESH)
    assert result["material_sha256"] == module.sha256(module.CHECKER_MATERIAL)
    assert result["texture_sha256"] == module.sha256(module.CHECKER_TEXTURE)


def _reverse_first_checker_face(text):
    lines = text.splitlines()
    first_face = next(
        index for index, line in enumerate(lines) if line.startswith("f ")
    )
    tag, a, b, c = lines[first_face].split()
    lines[first_face] = f"{tag} {a} {c} {b}"
    return "\n".join(lines) + "\n"


def _collapse_first_checker_face_uvs(text):
    lines = text.splitlines()
    texture_coordinate_lines = [
        index for index, line in enumerate(lines) if line.startswith("vt ")
    ]
    for obj_index in (496, 2, 1):
        lines[texture_coordinate_lines[obj_index - 1]] = "vt 0.500000000 0.500000000"
    return "\n".join(lines) + "\n"


@pytest.mark.parametrize(
    ("asset", "mutation", "message"),
    [
        (
            "mesh",
            lambda text: text.replace(
                "mtllib fbf_backspin_checker_sphere.mtl", "mtllib missing.mtl"
            ),
            "OBJ material/UV contract changed",
        ),
        (
            "material",
            lambda text: text.replace(
                "map_Kd fbf_backspin_checker.ppm", "map_Kd missing.ppm"
            ),
            "MTL texture binding changed",
        ),
        (
            "texture",
            lambda text: text.replace("64 32", "32 64", 1),
            "PPM header changed",
        ),
        (
            "mesh",
            lambda text: text.replace("v 0.195090322", "v 0.295090322", 1),
            "OBJ radius changed",
        ),
        (
            "mesh",
            lambda text: text.replace("496/496/496", "560/560/560", 1),
            "OBJ face index is out of range",
        ),
        (
            "mesh",
            _reverse_first_checker_face,
            "OBJ face winding is not outward",
        ),
        (
            "mesh",
            _collapse_first_checker_face_uvs,
            "OBJ has a degenerate UV triangle",
        ),
        (
            "texture",
            lambda text: text.replace("244 241 228", "32 36 43", 1),
            "PPM tile layout changed",
        ),
        (
            "texture",
            lambda text: text.replace("32 36 43", "255 93 115", 1),
            "PPM tile layout changed",
        ),
    ],
)
def test_checker_source_contract_fails_closed_on_asset_mutation(
    tmp_path, monkeypatch, asset, mutation, message
):
    module = _load_module()
    paths = {
        "mesh": tmp_path / "checker.obj",
        "material": tmp_path / "checker.mtl",
        "texture": tmp_path / "checker.ppm",
    }
    originals = {
        "mesh": module.CHECKER_MESH,
        "material": module.CHECKER_MATERIAL,
        "texture": module.CHECKER_TEXTURE,
    }
    for name, path in paths.items():
        text = originals[name].read_text(encoding="ascii")
        path.write_text(mutation(text) if name == asset else text, encoding="ascii")
    monkeypatch.setattr(module, "CHECKER_MESH", paths["mesh"])
    monkeypatch.setattr(module, "CHECKER_MATERIAL", paths["material"])
    monkeypatch.setattr(module, "CHECKER_TEXTURE", paths["texture"])

    with pytest.raises(ValueError, match=message):
        module._validate_visual_only_source()


def test_source_identity_binds_checker_assets():
    module = _load_module()
    sources = module._source_identity(
        trace_binary=TEST_EXECUTABLE,
        demo=TEST_EXECUTABLE,
        runner=module.DEFAULT_RUNNER,
        ffmpeg=TEST_EXECUTABLE,
        ffprobe=TEST_EXECUTABLE,
        python=TEST_EXECUTABLE,
    )

    for key, path in {
        "backspin_checker_mesh": module.CHECKER_MESH,
        "backspin_checker_material": module.CHECKER_MATERIAL,
        "backspin_checker_texture": module.CHECKER_TEXTURE,
    }.items():
        assert sources[key] == {
            "path": str(path.resolve()),
            "sha256": module.sha256(path),
        }


def test_report_describes_checker_texture_and_preserves_aliasing_boundary():
    module = _load_module()
    parsed = module.parse_trace_text(_trace_text(module))
    capture = {
        "captured_frames": 131,
        "worst_residual": 5e-7,
        "media": {
            "mp4": {
                "stream_contract": {
                    "width": 1300,
                    "height": 506,
                    "frame_rate": "30/1",
                    "frame_count": 66,
                }
            },
            "gif": {
                "stream_contract": {
                    "width": 960,
                    "height": 374,
                    "frame_rate": "15/1",
                    "frame_count": 33,
                }
            },
        },
    }
    projection = module._compare_solver_projections(
        parsed["solver_projection"], parsed["solver_projection"]
    )

    report = module._report_markdown({"trace": parsed["summary"]}, capture, projection)

    assert "checker texture" in report
    assert "high-contrast 6x4" in report
    assert "coral registration tile" in report
    assert "sampled media do not prove signed angular direction" in report
    assert "cyan stripe" not in report


def test_trace_argv_freezes_the_explicit_tracked_contract():
    module = _load_module()
    argv = module._trace_argv(Path("/trace"))

    assert argv == [
        "/trace",
        "backspin",
        "exact_fbf",
        "1",
        "130",
        "nan",
        "tracked",
        "default",
        "default",
        "1",
        "dart_best",
        "dart",
        "default",
        "0",
        "0",
        "default",
    ]


def test_verify_only_replays_trace_without_disposable_capture_staging(
    tmp_path, monkeypatch
):
    module = _load_module()
    bundle = tmp_path / "bundle"
    trace_path = bundle / "traces/backspin.csv"
    trace_path.parent.mkdir(parents=True)
    trace_path.write_text("stored\n", encoding="utf-8")
    (bundle / "traces/backspin.stderr.txt").write_text("", encoding="utf-8")
    monkeypatch.setattr(
        module, "_require_file", lambda path, label: Path(path).resolve()
    )
    monkeypatch.setattr(
        module,
        "verify_finalized",
        lambda *args, **kwargs: {"pass": True, "status": "verified"},
    )
    monkeypatch.setattr(
        module,
        "_run_command",
        lambda *args, **kwargs: subprocess.CompletedProcess([], 0, "stored\n", ""),
    )
    args = argparse.Namespace(
        bundle=bundle,
        trace_binary=Path("/trace"),
        demo=Path("/demo"),
        runner=Path("/runner"),
        ffmpeg=Path("/ffmpeg"),
        ffprobe=Path("/ffprobe"),
        python=Path("/python"),
        trace_timeout=1.0,
        verification_timeout=1.0,
    )

    result = module.verify_only(args)

    assert result["live_trace_replay"] is True
    assert result["live_capture_reverification"] is False
    assert result["durable_capture_reverification"] is True
    assert not any((bundle / relative).exists() for relative in module.STAGING_PATHS)


def test_json_reader_rejects_nonfinite_numbers(tmp_path):
    module = _load_module()
    path = tmp_path / "bad.json"
    path.write_text('{"value": NaN}\n', encoding="utf-8")

    with pytest.raises(ValueError, match="non-finite JSON number"):
        module.read_json(path)
