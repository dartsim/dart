import argparse
import copy
import importlib.util
import json
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts/finalize_fbf_author_card_house_construction.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "finalize_fbf_author_card_house_construction", SCRIPT
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _body(index, *, mobile, role):
    return {
        "name": f"{role}_{index}",
        "role": role,
        "size_m": [1.0, 1.0, 1.0],
        "mass_kg": 25.0 if mobile else 256.0,
        "friction": 0.8,
        "mobile": mobile,
        "initial_pose": {
            "translation": [float(index), 0.0, 1.0],
            "rotation": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        },
        "initial_linear_velocity_m_s": [0.0, 0.0, 0.0],
        "initial_angular_velocity_rad_s": [0.0, 0.0, 0.0],
    }


def _contract(module):
    return {
        "schema_version": module.CONTRACT_SCHEMA_VERSION,
        "kind": "configuration_only",
        "author_source": {
            "repository": module.EXPECTED_AUTHOR_REPOSITORY,
            "commit": module.EXPECTED_AUTHOR_COMMIT,
            "tree": module.EXPECTED_AUTHOR_TREE,
            "card_house_run_blob": "1" * 40,
            "card_house_run_py_sha256": "2" * 64,
            "fbf_config_py_sha256": "3" * 64,
            "solver_fbf_py_sha256": "4" * 64,
        },
        "configuration_spec_source_sha256": module.sha256(module.CONFIG_SPEC),
        "binary_binding": {
            "role": "dart_demos",
            "implementation_source_sha256": module.sha256(module.DEMO_SOURCE),
        },
        "source_configuration": {
            "scene": {
                "demo_scene_id": module.SCENE_ID,
                "levels": 5,
                "leaning_cards": 30,
                "bridges": 10,
                "cards": 40,
                "cubes": 4,
            },
            "cards": {
                "lean_size_m": [0.04, 1.25, 2.5],
                "bridge_size_m": [2.5, 1.25, 0.04],
                "density_kg_m3": 200.0,
                "mass_kg": 25.0,
                "lean_from_horizontal_degrees": 65.0,
                "lean_from_vertical_degrees": 25.0,
                "bridge_angle_degrees": -1.0,
                "tent_half_gap_m": 0.55,
                "tent_width_m": 2.2,
                "tent_height_m": 2.41660616977186,
                "source_mobile": True,
            },
            "cubes": {
                "size_m": [0.8, 0.8, 0.8],
                "density_kg_m3": 500.0,
                "mass_kg": 256.0,
                "initial_height_m": 13.0830308488593,
                "source_initially_kinematic": True,
            },
            "contact": {"friction": 0.8, "gap_m": 0.005},
            "schedule": {
                "display_time_step_seconds": 1.0 / 60.0,
                "substeps_per_frame": 4,
                "runtime_time_step_seconds": 1.0 / 240.0,
                "release_frame": 400,
                "release_substep": 1600,
                "total_frames": 800,
                "total_substeps": 3200,
            },
            "solver": {
                "type": "fbf_exact_coulomb",
                "max_contacts": 4096,
                "max_outer": 200,
                "outer_tol": 1.0e-6,
                "residual_check_interval": 5,
                "inner_solver": "block_gs",
                "inner_gs_sweeps": 10,
                "inner_max_iter": 200,
                "inner_tol": 1.0e-6,
                "gamma": None,
                "adaptive_gamma": True,
                "gamma_c": 5.0,
                "gamma_max": 1.0e6,
                "armijo_rho_high": 0.9,
                "armijo_shrink": 0.7,
                "armijo_max_backtracks": 8,
                "plateau_patience": 5,
                "plateau_rtol": 0.01,
                "warm_start": True,
                "warm_start_match_radius": 0.02,
                "warm_start_normal_cosine": 0.9,
                "warm_start_max_age": 3,
                "warm_start_gamma_cap": 1.0e4,
                "baumgarte_erp": 0.0,
                "project_after_correction": False,
                "termination_residual": "coulomb_rel",
                "termination_tol": 1.0e-6,
            },
        },
        "dart_observation": {
            "world": {
                "time_step_seconds": 1.0 / 240.0,
                "gravity_m_s2": [0.0, 0.0, -9.81],
                "simulation_threads": 1,
                "deactivation_enabled": False,
            },
            "collision": {
                "detector": "native",
                "contact_manifold": "four_point_planar",
                "max_contacts": 4096,
                "max_contacts_per_pair": 4,
            },
            "solver_adapter": {"type": "exact_fbf"},
            "ground": {"shape": "plane", "friction": 0.8, "mobile": False},
            "cards": [_body(i, mobile=True, role="card") for i in range(40)],
            "cubes": [_body(i, mobile=False, role="cube") for i in range(4)],
        },
        "adapter_boundaries": {
            "source_contact_gap_recorded_m": 0.005,
            "source_contact_gap_semantics_implemented": False,
            "source_solver_backend_semantics_implemented": False,
        },
        "claim_boundary": copy.deepcopy(module.EXPECTED_CLAIM_BOUNDARY),
        "visual_style": {
            "renderer_only": True,
            "description": "restrained alternating paper colors for layer legibility",
        },
    }


def _diagnostics():
    return {
        "available": True,
        "solver": "ExactCoulombFbfConstraintSolver",
        "status": "not_run",
        "fbf_status": "not_run",
        "residual": None,
        "best_residual": None,
        "iterations": 0,
        "total_iterations": 0,
        "exact_solves": 0,
        "exact_attempts": 0,
        "accepted_at_cap": 0,
        "worst_residual": None,
        "exact_failures": 0,
        "boxed_lcp_fallbacks": 0,
        "warm_starts": 0,
        "contacts": 0,
    }


def _write_contrast_png(path, width=1280, height=720):
    from _image_tools import write_png

    shades = (0, 110, 255)
    pixels = bytearray()
    for _y in range(height):
        for x in range(width):
            shade = shades[x % len(shades)]
            pixels.extend((shade, shade, shade))
    write_png(path, width, height, bytes(pixels))


def _schedule(module, demo, root):
    return {
        "id": module.SCHEDULE_ID,
        "title": "Author-pinned five-level construction inspection",
        "scene": module.SCENE_ID,
        "source_segment": "paper_tables_6_7_no_video_segment",
        "collision_detector": "native",
        "collision_detector_override": False,
        "configuration": copy.deepcopy(module.EXPECTED_CONFIGURATION),
        "total_steps": 0,
        "time_step_seconds": 1.0 / 240.0,
        "capture_steps": [0],
        "panel_steps": [0],
        "panel_labels": [module.EXPECTED_PANEL_LABEL],
        "actions": [],
        "output": {
            "directory": str(root / module.CAPTURE_DIR),
            "timeline": str(root / module.CAPTURE_DIR / "timeline.json"),
            "panel": str(root / module.CAPTURE_DIR / "panel.png"),
            "mp4": None,
            "gif": None,
        },
        "runnable": True,
        "adapter_gap": None,
        "long_run": False,
        "actual_simulator_required": True,
        "generated_imagery_allowed": False,
        "paper_comparable": False,
        "known_mismatches": copy.deepcopy(module.EXPECTED_MISMATCHES),
        "known_gate_blockers": [],
        "evidence_ready": True,
        "demo_argv": module._expected_demo_argv(demo, root),
        "demo_command": "unused",
    }


def _capture_fixture(module, root):
    demo = root / "dart-demos"
    demo.write_bytes(b"demo")
    contract = _contract(module)
    module.write_json(root / "contract.json", contract)
    output = root / module.CAPTURE_DIR
    frame = output / "frames/step_000000.png"
    panel_frame = output / "panel_frames/step_000000.png"
    panel = output / "panel.png"
    _write_contrast_png(frame)
    _write_contrast_png(panel_frame, 660, 506)
    _write_contrast_png(panel, 676, 538)
    compose = {
        "schema_version": "dart.image_compose/v1",
        "mode": "side-by-side",
        "inputs": [str(panel_frame)],
        "out": str(panel),
        "labels": [module.EXPECTED_PANEL_LABEL],
        "width": 676,
        "height": 538,
        "stats": {},
        "pass": True,
    }
    module.write_json(output / "panel.compose.json", compose)
    diagnostics = _diagnostics()
    timeline = {
        "schema_version": "dart.demos_headless_timeline/v1",
        "scene": module.SCENE_ID,
        "active_scene": module.SCENE_ID,
        "total_steps": 0,
        "completed_steps": 0,
        "width": 1280,
        "height": 720,
        "collision_detector": "native",
        "event_order": "captures_before_actions_at_each_completed_step",
        "runtime_command": " ".join(module._expected_demo_argv(demo, root)),
        "physics_contract": copy.deepcopy(contract),
        "steps": [{"step": 0, "sim_time": 0.0, "solver_diagnostics": diagnostics}],
        "shots": [
            {
                "step": 0,
                "sim_time": 0.0,
                "path": str(frame),
                "success": True,
                "sequence": 0,
                "solver_diagnostics": diagnostics,
            }
        ],
        "actions": [],
        "events": [{"sequence": 0, "step": 0, "type": "shot"}],
        "solver_diagnostics": diagnostics,
    }
    module.write_json(output / "timeline.json", timeline)
    stored_timeline = {
        "sidecar": str(output / "timeline.json"),
        "shot_count": 1,
        "action_count": 0,
        "unique_frame_hashes": 1,
        "step_count": 1,
        "steps": {"0": {"sim_time": 0.0, "solver_diagnostics": diagnostics}},
        "frames": {
            "0": {
                "path": str(frame),
                "sha256": module.sha256(frame),
                "non_blank": {"pass": True},
                "world_viewport": {"non_blank": {"pass": True}},
            }
        },
        "final_solver_diagnostics": diagnostics,
    }
    result = {
        "schema_version": module.RUNNER_SCHEMA_VERSION,
        "kind": "capture_result",
        "schedule": _schedule(module, demo, root),
        "runtime": {
            "demo_argv": module._expected_demo_argv(demo, root),
            "demo_path": str(demo),
            "demo_sha256": module.sha256(demo),
            "ffmpeg": "/fake/ffmpeg",
            "ffprobe": "/fake/ffprobe",
        },
        "timeline_validation": stored_timeline,
        "panel_validation": {
            "command": [],
            "path": str(panel),
            "sha256": module.sha256(panel),
            "size_bytes": panel.stat().st_size,
            "source_frames": [
                {"path": str(panel_frame), "sha256": module.sha256(panel_frame)}
            ],
            "compose_manifest_path": str(output / "panel.compose.json"),
            "compose_manifest_sha256": module.sha256(output / "panel.compose.json"),
            "compose_manifest": compose,
            "verdict": {"pass": True},
        },
        "media_validation": [],
        "actual_simulator": True,
        "generated_imagery": False,
        "paper_comparable": False,
        "automated_semantic_outcome_validated": False,
        "semantic_outcome_gate": module.EXPECTED_SEMANTIC_OUTCOME_GATE,
        "known_mismatches": copy.deepcopy(module.EXPECTED_MISMATCHES),
        "pass": True,
    }
    module.write_json(output / "metadata.json", result)
    summary = {
        "schema_version": module.RUNNER_SCHEMA_VERSION,
        "kind": "capture_run",
        "results": [result],
        "failures": [],
        "group_outputs": [],
        "group_skips": [],
        "pass": True,
    }
    module.write_json(root / "run-summary.json", summary)
    verification = {
        "schema_version": module.RUNNER_SCHEMA_VERSION,
        "kind": "verification",
        "results": [
            {
                "schedule": module.SCHEDULE_ID,
                "timeline": stored_timeline,
                "panel": {
                    "pass": True,
                    "image": {"path": str(panel), "width": 676, "height": 538},
                    "checks": {"non_blank": {"pass": True}},
                },
                "media": [],
                "metadata_path": str(output / "metadata.json"),
                "metadata_sha256": module.sha256(output / "metadata.json"),
                "pass": True,
            }
        ],
        "group_outputs": [],
        "pass": True,
    }
    module.write_json(root / "verification.json", verification)
    return demo, verification


def _provenance_fixture(module, root, identity):
    contract = _contract(module)
    module.write_json(root / "contract.json", contract)
    verification = module.read_json(root / "verification.json")
    frame_sha = module.sha256(root / module.DURABLE_STILL)
    return {
        "capture_argv": module._capture_argv(
            root / "python",
            root / "runner",
            root / "dart-demos",
            root,
            root / "ffmpeg",
            root / "ffprobe",
        ),
        "capture_returncode": 0,
        "run_summary_path": "run-summary.json",
        "run_summary_sha256": module.sha256(root / "run-summary.json"),
        "run_summary_validated": True,
        "capture_stdout_sha256": "1" * 64,
        "capture_stdout_ends_with_run_summary": True,
        "capture_stderr_sha256": module.EMPTY_SHA256,
        "contract_query_argv": module._contract_argv(root / "dart-demos"),
        "contract_query_returncode": 0,
        "contract_path": "contract.json",
        "contract_sha256": module.sha256(root / "contract.json"),
        "contract_payload_sha256_before": module._payload_sha256(contract),
        "contract_payload_sha256_after": module._payload_sha256(contract),
        "contract_stdout_sha256_before": "2" * 64,
        "contract_stdout_sha256_after": "2" * 64,
        "contract_stdout_byte_identical_before_after": True,
        "contract_stderr_sha256_before": module.EMPTY_SHA256,
        "contract_stderr_sha256_after": module.EMPTY_SHA256,
        "verification_argv": module._verification_argv(
            root / "python",
            root / "runner",
            root / "dart-demos",
            root,
            root / "ffmpeg",
            root / "ffprobe",
        ),
        "verification_returncode": 0,
        "verification_path": "verification.json",
        "verification_sha256": module.sha256(root / "verification.json"),
        "verification_payload_sha256": module._payload_sha256(verification),
        "verification_stdout_sha256": "3" * 64,
        "verification_stderr_sha256": module.EMPTY_SHA256,
        "durable_still_path": module.DURABLE_STILL,
        "durable_still_sha256": frame_sha,
        "staging_pruned": True,
        "source_identity_before": identity,
        "source_identity_after": identity,
    }


def test_defaults_and_claim_boundary_are_construction_only():
    module = _load_module()

    assert module.DEFAULT_BUNDLE.name == "card_house_author_5_construction_current_v1"
    assert module.SCHEDULE_ID == "card_house_author_5_construction"
    assert module.SCENE_ID == "fbf_author_card_house_5_construction"
    assert len(module.CAPTURE_PATHS) == 9
    assert module.DURABLE_STILL in module.CAPTURE_PATHS
    assert module.CAPTURE_PATHS.isdisjoint(module.STAGING_PATHS)
    assert module.EXPECTED_METADATA_FLAGS["configuration_port_valid"] is True
    for key in (
        "trajectory_valid",
        "solver_valid",
        "physical_outcome_valid",
        "fig06_parity",
        "video06_parity",
        "paper_timing_valid",
        "canonical_fig06_deliverable",
        "canonical_video06_deliverable",
        "dynamics_executed",
    ):
        assert module.EXPECTED_METADATA_FLAGS[key] is False


def test_configuration_contract_accepts_exact_current_shape():
    module = _load_module()
    payload = _contract(module)

    assert module.validate_configuration_contract(payload) is payload


@pytest.mark.parametrize(
    ("path", "value", "message"),
    [
        (("schema_version",), "wrong", "identity"),
        (("kind",), "physics", "identity"),
        (("author_source", "repository"), "forged", "author-source"),
        (("author_source", "commit"), "0" * 40, "author-source"),
        (("author_source", "card_house_run_blob"), "bad", "author-source"),
        (("configuration_spec_source_sha256",), "0" * 64, "current spec"),
        (
            ("binary_binding", "implementation_source_sha256"),
            "0" * 64,
            "current demo source",
        ),
        (("source_configuration", "scene", "cards"), 39, "scene contract"),
        (
            ("source_configuration", "schedule", "release_substep"),
            1599,
            "schedule",
        ),
        (("source_configuration", "contact", "gap_m"), 0.0, "contact"),
        (
            ("source_configuration", "solver", "warm_start_match_radius"),
            0.03,
            "solver",
        ),
        (("dart_observation", "cards", 0, "mobile"), False, "mobility"),
        (("dart_observation", "cubes", 0, "mobile"), True, "mobility"),
        (
            (
                "dart_observation",
                "cards",
                0,
                "initial_linear_velocity_m_s",
            ),
            [1.0, 0.0, 0.0],
            "nonzero",
        ),
        (
            ("adapter_boundaries", "source_contact_gap_semantics_implemented"),
            True,
            "adapter",
        ),
        (("claim_boundary", "trajectory_valid"), True, "claim boundary"),
        (("visual_style", "renderer_only"), False, "visual-style"),
    ],
)
def test_configuration_contract_rejects_mutations(path, value, message):
    module = _load_module()
    payload = _contract(module)
    target = payload
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value

    with pytest.raises(ValueError, match=message):
        module.validate_configuration_contract(payload)


def test_configuration_contract_rejects_extra_keys_and_body_counts():
    module = _load_module()
    extra = _contract(module)
    extra["paper_parity"] = True
    with pytest.raises(ValueError, match="exact keys"):
        module.validate_configuration_contract(extra)

    missing_card = _contract(module)
    missing_card["dart_observation"]["cards"].pop()
    with pytest.raises(ValueError, match="40 cards"):
        module.validate_configuration_contract(missing_card)

    extra_cube = _contract(module)
    extra_cube["dart_observation"]["cubes"].append(_body(4, mobile=False, role="cube"))
    with pytest.raises(ValueError, match="4 cubes"):
        module.validate_configuration_contract(extra_cube)


def test_zero_step_diagnostics_are_fail_closed():
    module = _load_module()
    diagnostics = _diagnostics()

    assert (
        module._validate_zero_step_diagnostics(diagnostics, label="test") is diagnostics
    )
    for key, value in (
        ("exact_attempts", 1),
        ("contacts", 1),
        ("residual", 0.0),
        ("status", "success"),
        ("available", False),
    ):
        changed = copy.deepcopy(diagnostics)
        changed[key] = value
        with pytest.raises(ValueError):
            module._validate_zero_step_diagnostics(changed, label="test")


def test_strict_image_gate_requires_nonblank_and_contrast(tmp_path):
    module = _load_module()
    passing = tmp_path / "passing.png"
    _write_contrast_png(passing, 300, 300)

    verdict = module._strict_image_verdict(passing, label="fixture")
    assert verdict["checks"]["non_blank"]["pass"] is True
    assert verdict["checks"]["contrast"]["pass"] is True
    assert verdict["construction_contrast_gate"]["pass"] is True
    assert verdict["construction_contrast_gate"]["bright_population_required"] is False

    from _image_tools import write_png

    restrained = tmp_path / "restrained.png"
    restrained_pixels = bytearray()
    for _y in range(300):
        for x in range(300):
            shade = (20, 100, 160)[x % 3]
            restrained_pixels.extend((shade, shade, shade))
    write_png(restrained, 300, 300, bytes(restrained_pixels))
    restrained_verdict = module._strict_image_verdict(
        restrained, label="restrained fixture"
    )
    assert restrained_verdict["checks"]["contrast"]["pass"] is False
    assert restrained_verdict["construction_contrast_gate"]["pass"] is True

    flat = tmp_path / "flat.png"
    write_png(flat, 300, 300, bytes([100, 100, 100]) * 300 * 300)
    with pytest.raises(ValueError, match="strict image verdict failed"):
        module._strict_image_verdict(flat, label="flat fixture")


def test_capture_bundle_accepts_exact_step_zero_fixture(tmp_path):
    module = _load_module()
    demo, verification = _capture_fixture(module, tmp_path)

    summary = module.validate_capture_bundle(
        tmp_path, demo=demo, verification=verification
    )

    assert summary["capture_step"] == 0
    assert summary["simulation_substeps_executed"] == 0
    assert summary["configuration_only"] is True
    assert summary["trajectory_valid"] is False
    assert summary["solver_valid"] is False
    assert summary["physical_outcome_valid"] is False
    assert summary["step_zero_frame"]["strict_nonblank_contrast_verdict"]["pass"]
    assert summary["panel"]["strict_nonblank_contrast_verdict"]["pass"]


def test_capture_bundle_accepts_v2_semantic_provenance_metadata(tmp_path):
    module = _load_module()
    demo, verification = _capture_fixture(module, tmp_path)
    metadata_path = tmp_path / module.CAPTURE_DIR / "metadata.json"
    metadata = module.read_json(metadata_path)
    metadata["schema_version"] = module.RUNNER_CAPTURE_RESULT_SCHEMA_VERSION
    contract = module.read_json(tmp_path / "contract.json")
    provenance = module.build_semantic_physics_provenance(contract)
    metadata["runtime"]["scene_physics_provenance"] = {
        "schema_version": module.SEMANTIC_PROVENANCE_SCHEMA_VERSION,
        "contract_schema_version": provenance.schema_version,
        "family": provenance.family,
        "query_argv": module._runner_contract_argv(demo),
        "contract_sha256": module._payload_sha256(contract),
        "semantic_physics_sha256": provenance.semantic_sha256,
        "broad_implementation_sha256": (provenance.broad_implementation_sha256),
        "sidecar_contract_match": True,
    }
    module.write_json(metadata_path, metadata)

    run_summary = module.read_json(tmp_path / "run-summary.json")
    run_summary["results"] = [metadata]
    module.write_json(tmp_path / "run-summary.json", run_summary)
    verification["results"][0]["metadata_sha256"] = module.sha256(metadata_path)

    summary = module.validate_capture_bundle(
        tmp_path, demo=demo, verification=verification
    )

    assert summary["capture_step"] == 0


def test_pruned_capture_uses_durable_still_and_rejects_missing_or_tampered(
    tmp_path,
):
    module = _load_module()
    accepted = tmp_path / "accepted"
    accepted.mkdir()
    demo, verification = _capture_fixture(module, accepted)
    module._promote_and_prune_capture_staging(accepted)

    summary = module.validate_capture_bundle(
        accepted, demo=demo, verification=verification
    )
    assert summary["step_zero_frame"]["path"] == module.DURABLE_STILL
    assert summary["step_zero_frame"]["durable_promoted_copy"] is True
    assert not (accepted / module.CAPTURE_DIR / "frames").exists()
    assert not (accepted / module.CAPTURE_DIR / "panel_frames").exists()

    (accepted / module.DURABLE_STILL).unlink()
    with pytest.raises(ValueError, match="durable step-zero still is missing"):
        module.validate_capture_bundle(accepted, demo=demo, verification=verification)

    tampered = tmp_path / "tampered"
    tampered.mkdir()
    demo, verification = _capture_fixture(module, tampered)
    module._promote_and_prune_capture_staging(tampered)
    (tampered / module.DURABLE_STILL).write_bytes(b"tampered")
    with pytest.raises(ValueError, match="frame binding changed"):
        module.validate_capture_bundle(tampered, demo=demo, verification=verification)


def test_pruned_capture_provenance_and_invocations_are_exact(tmp_path, monkeypatch):
    module = _load_module()
    demo, _ = _capture_fixture(module, tmp_path)
    module._promote_and_prune_capture_staging(tmp_path)
    identity = {"bound": {"path": "/source", "sha256": "a" * 64}}
    provenance = _provenance_fixture(module, tmp_path, identity)
    module.write_json(tmp_path / "capture-provenance.json", provenance)
    monkeypatch.setattr(module, "_source_identity", lambda **_kwargs: identity)
    kwargs = {
        "bundle": tmp_path,
        "demo": demo,
        "runner": tmp_path / "runner",
        "ffmpeg": tmp_path / "ffmpeg",
        "ffprobe": tmp_path / "ffprobe",
        "python": tmp_path / "python",
    }

    assert module._validate_capture_provenance(provenance, **kwargs) == provenance
    invocations = {
        "schema_version": module.INVOCATIONS_SCHEMA_VERSION,
        "contract_query": {
            "argv": module._contract_argv(demo),
            "returncode": 0,
            "stdout_sha256": provenance["contract_stdout_sha256_after"],
            "stderr_sha256": provenance["contract_stderr_sha256_after"],
            "payload_path": "contract.json",
            "payload_sha256": module.sha256(tmp_path / "contract.json"),
        },
        "capture": {
            "argv": module._capture_argv(
                tmp_path / "python",
                tmp_path / "runner",
                demo,
                tmp_path,
                tmp_path / "ffmpeg",
                tmp_path / "ffprobe",
            ),
            "returncode": 0,
            "stdout_sha256": provenance["capture_stdout_sha256"],
            "stderr_sha256": provenance["capture_stderr_sha256"],
            "run_summary_path": "run-summary.json",
            "run_summary_sha256": module.sha256(tmp_path / "run-summary.json"),
        },
        "capture_verification": {
            "argv": module._verification_argv(
                tmp_path / "python",
                tmp_path / "runner",
                demo,
                tmp_path,
                tmp_path / "ffmpeg",
                tmp_path / "ffprobe",
            ),
            "returncode": 0,
            "stdout_path": "verification.json",
            "stdout_sha256": module.sha256(tmp_path / "verification.json"),
            "raw_stdout_sha256": provenance["verification_stdout_sha256"],
            "stderr_sha256": provenance["verification_stderr_sha256"],
        },
    }
    module.write_json(tmp_path / "invocations.json", invocations)
    assert (
        module._validate_invocations(
            tmp_path,
            demo=demo,
            runner=tmp_path / "runner",
            ffmpeg=tmp_path / "ffmpeg",
            ffprobe=tmp_path / "ffprobe",
            python=tmp_path / "python",
        )
        == invocations
    )

    changed = copy.deepcopy(provenance)
    changed["staging_pruned"] = False
    with pytest.raises(ValueError, match="capture-time provenance changed"):
        module._validate_capture_provenance(changed, **kwargs)

    changed = copy.deepcopy(provenance)
    changed["paper_parity"] = True
    with pytest.raises(ValueError, match="provenance is malformed"):
        module._validate_capture_provenance(changed, **kwargs)


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (
            lambda module, root: _mutate_json(
                module,
                root / "run-summary.json",
                lambda payload: payload["results"][0].update(paper_parity=True),
            ),
            "result exact keys",
        ),
        (
            lambda module, root: _mutate_json(
                module,
                root / "run-summary.json",
                lambda payload: payload["results"][0]["schedule"].update(
                    canonical_deliverable=True
                ),
            ),
            "schedule exact keys",
        ),
        (
            lambda module, root: _mutate_json(
                module,
                root / "run-summary.json",
                lambda payload: payload["results"][0]["schedule"].update(total_steps=1),
            ),
            "total_steps",
        ),
        (
            lambda module, root: _mutate_json(
                module,
                root / "run-summary.json",
                lambda payload: payload["results"][0].update(
                    media_validation=[{"kind": "mp4"}]
                ),
            ),
            "unexpectedly has media",
        ),
        (
            lambda module, root: _mutate_json(
                module,
                root / module.CAPTURE_DIR / "timeline.json",
                lambda payload: payload["steps"][0]["solver_diagnostics"].update(
                    exact_attempts=1
                ),
            ),
            "unexpected execution",
        ),
        (
            lambda module, root: _mutate_json(
                module,
                root / "verification.json",
                lambda payload: payload["results"][0].update(media=[{"kind": "mp4"}]),
            ),
            "verification result changed",
        ),
    ],
)
def test_capture_bundle_rejects_promotions_or_execution(tmp_path, mutate, message):
    module = _load_module()
    demo, _ = _capture_fixture(module, tmp_path)
    mutate(module, tmp_path)

    with pytest.raises(ValueError, match=message):
        module.validate_capture_bundle(tmp_path, demo=demo)


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (
            lambda payload: payload.pop("physics_contract"),
            "timeline physics contract is missing",
        ),
        (
            lambda payload: payload["physics_contract"]["binary_binding"].update(
                implementation_source_sha256="0" * 64
            ),
            "current demo source",
        ),
        (
            lambda payload: payload["physics_contract"]["dart_observation"][
                "world"
            ].update(time_step_seconds=1.0 / 60.0),
            "differs from the separately queried contract",
        ),
        (
            lambda payload: payload["physics_contract"]["source_configuration"][
                "scene"
            ].update(levels=5.0),
            "payload digest differs from the separately queried contract",
        ),
    ],
)
def test_capture_bundle_requires_capture_bound_physics_contract(
    tmp_path, mutate, message
):
    module = _load_module()
    demo, _ = _capture_fixture(module, tmp_path)
    _mutate_json(
        module,
        tmp_path / module.CAPTURE_DIR / "timeline.json",
        mutate,
    )

    with pytest.raises(ValueError, match=message):
        module.validate_capture_bundle(tmp_path, demo=demo)


def _mutate_json(module, path, mutate):
    payload = module.read_json(path)
    mutate(payload)
    module.write_json(path, payload)


def test_manual_template_and_validation_bind_exact_order_and_hashes(tmp_path):
    module = _load_module()
    for relative in module.MANUAL_ARTIFACT_PATHS:
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(relative.encode())
    template = module._manual_template(tmp_path)
    assert template["manual_inspected"] is False
    assert template["pass"] is False
    assert [item["path"] for item in template["representative_artifacts"]] == list(
        module.MANUAL_ARTIFACT_PATHS
    )

    accepted = copy.deepcopy(template)
    accepted["manual_inspected"] = True
    accepted["pass"] = True
    accepted["verdicts"] = copy.deepcopy(module.MANUAL_VERDICTS)
    for item in accepted["representative_artifacts"]:
        item["observation"] = f"inspected {item['path']}"
    module.write_json(tmp_path / "manual-inspection.json", accepted)
    assert module.validate_manual_inspection(tmp_path) == accepted

    reordered = copy.deepcopy(accepted)
    reordered["representative_artifacts"].reverse()
    module.write_json(tmp_path / "manual-inspection.json", reordered)
    with pytest.raises(ValueError, match="paths changed"):
        module.validate_manual_inspection(tmp_path)

    empty = copy.deepcopy(accepted)
    empty["representative_artifacts"][0]["observation"] = ""
    module.write_json(tmp_path / "manual-inspection.json", empty)
    with pytest.raises(ValueError, match="observation missing"):
        module.validate_manual_inspection(tmp_path)

    stale = copy.deepcopy(accepted)
    stale["representative_artifacts"][0]["sha256"] = "0" * 64
    module.write_json(tmp_path / "manual-inspection.json", stale)
    with pytest.raises(ValueError, match="artifact changed"):
        module.validate_manual_inspection(tmp_path)


def test_artifact_index_and_path_membership_reject_extra_and_symlink(tmp_path):
    module = _load_module()
    for relative in module.EXPECTED_FINAL_PATHS:
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(relative.encode())
    module._validate_bundle_paths(tmp_path, complete=True)
    index = module.artifact_index(tmp_path)
    module.write_json(tmp_path / "artifact-index.json", index)
    assert index["artifact_count"] == len(module.EXPECTED_FINAL_PATHS) - 2
    module.validate_artifact_index(tmp_path, index)

    durable = tmp_path / module.DURABLE_STILL
    durable_bytes = durable.read_bytes()
    durable.unlink()
    with pytest.raises(ValueError, match="missing files"):
        module._validate_bundle_paths(tmp_path, complete=True)
    durable.write_bytes(durable_bytes)

    ignored_staging = tmp_path / module.CAPTURE_DIR / "frames/step_000000.png"
    ignored_staging.parent.mkdir()
    ignored_staging.write_bytes(b"must be pruned")
    with pytest.raises(ValueError, match="unexpected directories"):
        module._validate_bundle_paths(tmp_path, complete=True)
    ignored_staging.unlink()
    ignored_staging.parent.rmdir()

    extra = tmp_path / "canonical-fig06.png"
    extra.write_bytes(b"forbidden")
    with pytest.raises(ValueError, match="unexpected files"):
        module._validate_bundle_paths(tmp_path, complete=True)
    extra.unlink()

    target = tmp_path / f"{module.CAPTURE_DIR}/panel.png"
    target.unlink()
    target.symlink_to(tmp_path / "REPORT.md")
    with pytest.raises(ValueError, match="symlink"):
        module._validate_bundle_paths(tmp_path, complete=True)


@pytest.mark.parametrize("entrypoint", ["finalize", "verify_only"])
def test_entrypoints_reject_root_and_ancestor_symlinks(tmp_path, entrypoint):
    module = _load_module()

    def invoke(bundle, *, ancestor):
        if entrypoint == "finalize":
            return module.finalize(
                argparse.Namespace(
                    bundle=bundle,
                    reuse_current_capture=not ancestor,
                )
            )
        return module.verify_only(argparse.Namespace(bundle=bundle))

    target = tmp_path / "target"
    target.mkdir()
    root_link = tmp_path / "bundle-link"
    root_link.symlink_to(target, target_is_directory=True)
    with pytest.raises(ValueError, match="bundle root is a symlink"):
        invoke(root_link, ancestor=False)

    real_parent = tmp_path / "real-parent"
    real_parent.mkdir()
    linked_parent = tmp_path / "linked-parent"
    linked_parent.symlink_to(real_parent, target_is_directory=True)
    ancestor_bundle = linked_parent / (
        "new-bundle" if entrypoint == "finalize" else "bundle"
    )
    if entrypoint == "verify_only":
        (real_parent / "bundle").mkdir()
    with pytest.raises(ValueError, match="passes through a symlink"):
        invoke(ancestor_bundle, ancestor=True)
    assert not (real_parent / "new-bundle").exists()


def test_bundle_transaction_rolls_back_and_commits(tmp_path):
    module = _load_module()
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    marker = bundle / "marker"
    marker.write_text("before", encoding="utf-8")

    with pytest.raises(RuntimeError, match="boom"):
        with module._bundle_transaction(bundle):
            marker.write_text("during", encoding="utf-8")
            (bundle / "extra").write_text("extra", encoding="utf-8")
            raise RuntimeError("boom")
    assert marker.read_text(encoding="utf-8") == "before"
    assert not (bundle / "extra").exists()

    with module._bundle_transaction(bundle) as commit:
        marker.write_text("after", encoding="utf-8")
        commit()
    assert marker.read_text(encoding="utf-8") == "after"


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


def test_command_runner_is_strict_and_times_out():
    module = _load_module()

    completed = module._run_command([sys.executable, "-c", "print('ok')"], timeout=5.0)
    assert completed.returncode == 0
    assert completed.stdout == "ok\n"

    with pytest.raises(ValueError, match="exit 3"):
        module._run_command(
            [
                sys.executable,
                "-c",
                "import sys; print('bad', file=sys.stderr); sys.exit(3)",
            ],
            timeout=5.0,
        )
    with pytest.raises(subprocess.TimeoutExpired):
        module._run_command(
            [sys.executable, "-c", "import time; time.sleep(5)"],
            timeout=0.05,
        )


def test_command_interfaces_are_exact(tmp_path):
    module = _load_module()
    paths = {
        name: tmp_path / name
        for name in ("python", "runner", "demo", "ffmpeg", "ffprobe")
    }
    for path in paths.values():
        path.write_bytes(b"x")

    assert module._contract_argv(paths["demo"]) == [
        str(paths["demo"]),
        "--fbf-author-card-house-contract",
        module.SCENE_ID,
    ]
    capture = module._capture_argv(
        paths["python"],
        paths["runner"],
        paths["demo"],
        tmp_path,
        paths["ffmpeg"],
        paths["ffprobe"],
    )
    assert capture[capture.index("--scenario") + 1] == module.SCHEDULE_ID
    assert capture[capture.index("--out") + 1] == str(tmp_path / "run-summary.json")
    verification = module._verification_argv(
        paths["python"],
        paths["runner"],
        paths["demo"],
        tmp_path,
        paths["ffmpeg"],
        paths["ffprobe"],
    )
    assert verification[2] == "verify"
    assert "--out" not in verification


def test_ldd_parser_rejects_unresolved_and_limits_to_build_tree(tmp_path):
    module = _load_module()
    build = tmp_path / "build"
    build.mkdir()
    library = build / "libdart.so.6.20"
    library.write_bytes(b"dart")
    outside = tmp_path / "libc.so"
    outside.write_bytes(b"libc")
    output = f"libdart.so => {library} (0x0)\n" f"libc.so => {outside} (0x0)\n"

    assert module._parse_ldd_in_tree_paths(output, build_root=build) == [library]
    with pytest.raises(ValueError, match="unresolved runtime dependency"):
        module._parse_ldd_in_tree_paths("libdart.so => not found", build_root=build)


def test_report_and_manual_interface_keep_all_promotions_false(tmp_path):
    module = _load_module()
    report = module._report_markdown(
        contract=_contract(module),
        capture={"capture_step": 0, "simulation_substeps_executed": 0},
        manual={
            "schema_version": module.MANUAL_SCHEMA_VERSION,
            "pass": True,
        },
    )

    assert "Configuration port valid: true" in report
    assert "Trajectory valid: false" in report
    assert "Solver valid: false" in report
    assert "Physical outcome valid: false" in report
    assert "Fig. 6 parity: false" in report
    assert "Video 06 parity: false" in report
    assert "Paper timing valid: false" in report
    assert "Canonical Fig. 6 or video deliverable: false" in report


def test_parser_defaults_and_mutual_exclusion(tmp_path, capsys):
    module = _load_module()
    parsed = module._parser().parse_args([])
    assert parsed.bundle == module.DEFAULT_BUNDLE
    assert parsed.reuse_current_capture is False
    assert parsed.verify_only is False

    result = module.main(
        [
            "--bundle",
            str(tmp_path),
            "--verify-only",
            "--reuse-current-capture",
        ]
    )
    assert result == 2
    assert "mutually exclusive" in capsys.readouterr().err
