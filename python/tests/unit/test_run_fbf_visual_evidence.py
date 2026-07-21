import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "run_fbf_visual_evidence.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("run_fbf_visual_evidence", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _test_schedule(module, *, action=False, exact_required=False):
    actions = (module.ScheduledAction(0, "p", "test action"),) if action else ()
    return module.CaptureSchedule(
        id="test_scene",
        scene="test_scene",
        title="Test scene",
        source_segment="test",
        total_steps=2,
        frame_stride=1,
        panel_steps=(0, 2),
        panel_labels=("start", "end"),
        configuration=(("fixture", "test"),),
        mismatches=("Synthetic unit-test sidecar, not paper evidence.",),
        actions=actions,
        width=640,
        height=480,
        encode_mp4=True,
        exact_fbf_required=exact_required,
    )


def _write_frames(module, schedule, output_dir):
    sys.path.insert(0, str(ROOT / "scripts"))
    from _image_tools import write_png

    for step in schedule.capture_steps:
        path = module._frame_path(output_dir, step)
        pixels = bytearray([20 + step, 40 + step, 60 + step] * (640 * 480))
        pixels[0:3] = bytes((200 - step, 100 + step, 30 + step))
        viewport_pixel = (100 * 640 + 270) * 3
        pixels[viewport_pixel : viewport_pixel + 3] = bytes(
            (180 - step, 90 + step, 20 + step)
        )
        write_png(path, 640, 480, bytes(pixels))


def _write_sidecar(module, schedule, output_dir, *, action_before_shot=False):
    shots = []
    actions = []
    events = []
    step_records = []
    sequence = 0
    for step in range(schedule.total_steps + 1):
        diagnostics = (
            {
                "solver": module.EXACT_SOLVER_NAME,
                "available": True,
                "status": "not_run" if step == 0 else "success",
                "fbf_status": "not_run" if step == 0 else "success",
                "residual": None if step == 0 else 5e-7,
                "worst_residual": None if step == 0 else 5e-7,
                "exact_attempts": step,
                "exact_solves": step,
                "accepted_at_cap": 0,
                "contacts": 0 if step == 0 else 1,
                "exact_failures": 0,
                "boxed_lcp_fallbacks": 0,
            }
            if schedule.exact_fbf_required
            else (
                {
                    "solver": module.BOXED_SOLVER_NAME,
                    "available": False,
                    "gap": module.BOXED_DIAGNOSTICS_GAP,
                }
                if schedule.solver_lane == "boxed"
                else {
                    "solver": module.EXACT_SOLVER_NAME,
                    "available": False,
                    "gap": "exact diagnostics are not required by this test",
                }
            )
        )
        step_records.append(
            {
                "step": step,
                "sim_time": schedule.time_at_step(step),
                "solver_diagnostics": dict(diagnostics),
            }
        )
        same_step_actions = [
            action for action in schedule.actions if action.step == step
        ]
        if action_before_shot:
            for action in same_step_actions:
                actions.append(
                    {
                        "sequence": sequence,
                        "step": action.step,
                        "key": action.key,
                        "success": True,
                    }
                )
                events.append(
                    {
                        "sequence": sequence,
                        "type": "action",
                        "step": action.step,
                        "key": action.key,
                        "success": True,
                    }
                )
                sequence += 1
        if step in schedule.capture_steps:
            shots.append(
                {
                    "sequence": sequence,
                    "step": step,
                    "path": str(module._frame_path(output_dir, step)),
                    "sim_time": schedule.time_at_step(step),
                    "success": True,
                    "solver_diagnostics": dict(diagnostics),
                }
            )
            events.append(
                {
                    "sequence": sequence,
                    "type": "shot",
                    "step": step,
                    "path": str(module._frame_path(output_dir, step)),
                    "success": True,
                }
            )
            sequence += 1
        if not action_before_shot:
            for action in same_step_actions:
                actions.append(
                    {
                        "sequence": sequence,
                        "step": action.step,
                        "key": action.key,
                        "success": True,
                    }
                )
                events.append(
                    {
                        "sequence": sequence,
                        "type": "action",
                        "step": action.step,
                        "key": action.key,
                        "success": True,
                    }
                )
                sequence += 1
    sidecar = {
        "schema_version": module.SIDECAR_SCHEMA_VERSION,
        "scene": schedule.scene,
        "active_scene": schedule.scene,
        "runtime_command": module._shell_command(
            module.build_demo_command(schedule, Path("dart-demos"), output_dir)
        ),
        "total_steps": schedule.total_steps,
        "completed_steps": schedule.total_steps,
        "width": schedule.width,
        "height": schedule.height,
        "collision_detector": schedule.collision_detector,
        "event_order": "captures_before_actions_at_each_completed_step",
        "steps": step_records,
        "shots": shots,
        "actions": actions,
        "events": events,
        "solver_diagnostics": dict(step_records[-1]["solver_diagnostics"]),
    }
    if schedule.exact_fbf_required:
        sidecar["headless_exact_fbf_fail_fast"] = {
            "enabled": True,
            "residual_tolerance": module.EXACT_FBF_RESIDUAL_TOLERANCE,
            "triggered": False,
            "step": None,
            "reason": None,
        }
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "timeline.json").write_text(json.dumps(sidecar), encoding="utf-8")


def _write_failed_sidecar(module, schedule, output_dir, demo, *, step=1):
    diagnostics = {
        "boxed_lcp_fallbacks": 0,
        "exact_failures": 1,
        "accepted_at_cap": 0,
        "exact_attempts": 1,
        "exact_solves": 0,
        "residual": None,
        "worst_residual": None,
    }
    clean_diagnostics = {
        "boxed_lcp_fallbacks": 0,
        "exact_failures": 0,
        "accepted_at_cap": 0,
        "exact_attempts": 0,
        "exact_solves": 0,
        "residual": None,
        "worst_residual": None,
    }
    steps = [
        {
            "step": item_step,
            "sim_time": schedule.time_at_step(item_step),
            "solver_diagnostics": dict(
                diagnostics if item_step == step else clean_diagnostics
            ),
        }
        for item_step in range(step + 1)
    ]
    shots = []
    actions = []
    events = []
    sequence = 0
    for item_step in range(step):
        if item_step in schedule.capture_steps:
            shot = {
                "sequence": sequence,
                "step": item_step,
                "path": str(module._frame_path(output_dir, item_step)),
                "sim_time": schedule.time_at_step(item_step),
                "success": True,
                "solver_diagnostics": dict(steps[item_step]["solver_diagnostics"]),
            }
            shots.append(shot)
            events.append(
                {
                    "sequence": sequence,
                    "type": "shot",
                    "step": item_step,
                    "path": shot["path"],
                    "success": True,
                }
            )
            sequence += 1
        for scheduled_action in schedule.actions:
            if scheduled_action.step != item_step:
                continue
            action = {
                "sequence": sequence,
                "step": item_step,
                "key": scheduled_action.key,
                "success": True,
            }
            actions.append(action)
            events.append(
                {
                    "sequence": sequence,
                    "type": "action",
                    "step": item_step,
                    "key": scheduled_action.key,
                    "success": True,
                }
            )
            sequence += 1
    sidecar = {
        "schema_version": module.SIDECAR_SCHEMA_VERSION,
        "scene": schedule.scene,
        "active_scene": schedule.scene,
        "runtime_command": module._shell_command(
            module.build_demo_command(schedule, demo, output_dir)
        ),
        "total_steps": schedule.total_steps,
        "completed_steps": step,
        "width": schedule.width,
        "height": schedule.height,
        "collision_detector": schedule.collision_detector,
        "event_order": "captures_before_actions_at_each_completed_step",
        "steps": steps,
        "shots": shots,
        "actions": actions,
        "events": events,
        "solver_diagnostics": dict(diagnostics),
        "headless_exact_fbf_fail_fast": {
            "enabled": True,
            "residual_tolerance": module.EXACT_FBF_RESIDUAL_TOLERANCE,
            "triggered": True,
            "step": step,
            "reason": "exact_failure",
        },
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "timeline.json").write_text(json.dumps(sidecar), encoding="utf-8")


def _literal_arch_physics_contract(module, *, solver_lane="exact"):
    identity_pose = {
        "translation": [0.0, 0.0, 0.0],
        "rotation": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    }
    exact_options = {
        "fallback_to_boxed_lcp_enabled": False,
        "constraint_regularization_enabled": False,
        "matrix_free_operator_enabled": False,
        "contact_row_operator_enabled": True,
        "dense_contact_row_snapshot_enabled": False,
        "warm_start_enabled": True,
        "step_size_persistence_enabled": False,
        "step_size_recovery_growth_factor": 1.05,
        "warm_start_match_distance": 0.025,
        "diagonal_seed_enabled": False,
        "matrix_free_seed_enabled": False,
        "projected_gradient_retry_enabled": False,
        "dense_residual_polish_enabled": False,
        "max_outer_iterations": 5000,
        "accept_outer_max_iterations": True,
        "tolerance": 1e-6,
        "initial_step_size": None,
        "cap_initial_step_size_at_safe_bound": True,
        "step_size_scale": 35.0,
        "outer_relaxation": 1.1,
        "coupling_variation_tolerance": 0.9,
        "shrink_factor": 0.7,
        "max_step_shrink_iterations": 20,
        "adaptive_step_size_enabled": True,
        "spectral_iterations": 10,
        "inner_max_sweeps": 30,
        "inner_local_solver": "exact_metric_projection",
        "run_fixed_inner_sweeps": True,
        "accept_inner_max_iterations": True,
        "inner_local_iterations": 1,
        "inner_tolerance": 1e-10,
        "inner_local_tolerance": 1e-12,
        "inner_diagonal_regularization": 0.0,
        "projected_gradient_max_iterations": 400,
        "projected_gradient_tolerance": 1e-12,
        "dense_residual_polish_iterations": 8,
        "dense_residual_polish_line_search_iterations": 8,
        "dense_residual_polish_regularization": 1e-9,
        "max_residual_history_samples": 0,
        "max_residual_history_records": 0,
    }
    cross_step_options = {
        "warm_start_match_mode": "either_body_local_feature",
        "warm_start_normal_cosine": 0.9,
        "strict_warm_start_match_distance": False,
        "warm_start_max_age": -1,
        "persistent_step_size_safe_bound_scale": 1.0,
        "minimum_step_size": None,
        "maximum_step_size": None,
        "warm_start_residual_threshold": None,
        "warm_start_step_size_cap": None,
        "persist_uncapped_step_size_on_warm_start_cap": False,
        "require_residual_improvement_for_unconverged_cache_save": False,
    }
    triangles = [
        [0, 1, 2],
        [0, 2, 3],
        [4, 6, 5],
        [4, 7, 6],
        [0, 4, 5],
        [0, 5, 1],
        [1, 5, 6],
        [1, 6, 2],
        [2, 6, 7],
        [2, 7, 3],
        [3, 7, 4],
        [3, 4, 0],
    ]
    stones = []
    for index in range(25):
        stones.append(
            {
                "name": f"masonry_arch_stone_{index}",
                "mobile": index not in (0, 24),
                "friction": 0.8,
                "primary_friction": 0.8,
                "secondary_friction": 0.8,
                "restitution": 0.0,
                "primary_slip_compliance": -1.0,
                "secondary_slip_compliance": -1.0,
                "first_friction_direction": [0.0, 0.0, 0.0],
                "default_friction_direction_frame": True,
                "mass_kg": 1.0 + index,
                "local_center_of_mass": [0.0, 0.0, 0.0],
                "moment_kg_m2": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "body_pose": identity_pose,
                "linear_velocity": [0.0, 0.0, 0.0],
                "angular_velocity": [0.0, 0.0, 0.0],
                "collision_shape_count": 1,
                "collision_shape_local_pose": identity_pose,
                "vertices": [
                    [float(vertex & 1), float((vertex >> 1) & 1), float(vertex >> 2)]
                    for vertex in range(8)
                ],
                "triangles": triangles,
            }
        )
    exact = solver_lane == "exact"
    contract = {
        "schema_version": "dart.fbf_literal_masonry_arch_physics_contract/v1",
        "kind": "physics_control",
        "source_binding": {
            "spec_sha256": module._sha256(
                ROOT / "examples/demos/scenes/FbfLiteralMasonryArchSpec.hpp"
            ),
            "implementation_sha256": module._sha256(
                ROOT / "examples/demos/scenes/FbfPaperFrictionScene.cpp"
            ),
            "geometry_sha256": module._sha256(
                ROOT / "dart/math/detail/MasonryArchGeometry.hpp"
            ),
            "solver_options_sha256": module._sha256(
                ROOT / "dart/constraint/ExactCoulombFbfConstraintSolver.hpp"
            ),
        },
        "world": {
            "name": "fbf_literal_masonry_arch_25",
            "time_step_seconds": 1.0 / 60.0,
            "gravity_m_s2": [0.0, 0.0, -9.81],
            "simulation_threads": 1,
            "deactivation_enabled": False,
            "skeleton_count": 26,
        },
        "declared_spec": {
            "density_kg_m3": 1000.0,
            "friction": 0.8,
            "barrier_gap_policy": "omit_source_offsets",
            "end_face_expansion_m": 1e-6,
            "downward_shift_m": 0.001001,
            "contact_erp": 0.0,
        },
        "collision": {
            "detector": "native",
            "contact_manifold": "four_point_planar",
            "native_detector_observed": True,
            "max_contacts": 400,
            "max_contacts_per_pair": 8,
        },
        "solver": {
            "lane": "exact_fbf" if exact else "boxed_lcp",
            "split_impulse_enabled": True,
            "colored_block_gauss_seidel_enabled": exact,
            "participant_affinity_enabled": exact,
            "exact_options": exact_options if exact else None,
            "cross_step_options": cross_step_options if exact else None,
        },
        "process_state": {"observed_contact_erp": 0.0},
        "ground": {
            "mobile": False,
            "plane_shape_observed": True,
            "friction": 0.8,
            "primary_friction": 0.8,
            "secondary_friction": 0.8,
            "restitution": 0.0,
            "primary_slip_compliance": -1.0,
            "secondary_slip_compliance": -1.0,
            "first_friction_direction": [0.0, 0.0, 0.0],
            "default_friction_direction_frame": True,
            "local_center_of_mass": [0.0, 0.0, 0.0],
            "body_pose": identity_pose,
            "linear_velocity": [0.0, 0.0, 0.0],
            "angular_velocity": [0.0, 0.0, 0.0],
            "collision_shape_count": 1,
            "collision_shape_local_pose": identity_pose,
            "plane_normal": [0.0, 0.0, 1.0],
            "plane_offset": 0.0,
        },
        "stones": stones,
    }
    fingerprint = module._literal_masonry_arch_geometry_fingerprint(contract)
    contract["physical_geometry_fingerprint"] = {
        "algorithm": "fnv1a64_q1e-10_le_v1",
        "value": fingerprint,
    }
    return contract


def _author_masonry_arch_adapter_contract(module, *, solver_lane="exact"):
    return {
        "schema_version": ("dart.fbf_author_masonry_arch_crown_impact_dart_adapter/v1"),
        "kind": "source_configuration_dynamics_adapter",
        "source_binding": {
            "repository": "https://github.com/matthcsong/fbf-sca-2026",
            "commit": "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0",
            "run_py_sha256": (
                "7e9158240267bb0ec1d0316b1badd4f3c8e1cd10270322de2e205cfea96f6f73"
            ),
            "mesh_tree_sha256": (
                "a3f4e35073a2f4e74837fff277cd923f104b6af57f2cf995cf7524fe498e483d"
            ),
            "configuration_spec_sha256": module._sha256(
                ROOT / "examples/demos/scenes/FbfAuthorMasonryArchSpec.hpp"
            ),
            "dart_adapter_sha256": module._sha256(
                ROOT / "examples/demos/scenes/FbfAuthorMasonryArchDartAdapter.hpp"
            ),
            "demo_implementation_sha256": module._sha256(
                ROOT / "examples/demos/scenes/FbfPaperFrictionScene.cpp"
            ),
        },
        "source_configuration": {
            "coordinate_scale": 1.0,
            "coordinate_units": "author_raw_numeric_values",
            "stones": 25,
            "fixed_springers": 2,
            "cubes": 3,
            "cube_edge": 3.0,
            "cube_mass": 54000.0,
            "friction": 0.8,
            "contact_gap": 0.005,
            "shape_stiffness": 10000.0,
            "shape_damping": 1000.0,
            "display_time_step_seconds": 1.0 / 60.0,
            "substeps_per_frame": 4,
            "runtime_time_step_seconds": 1.0 / 240.0,
            "release_frame": 400,
            "release_substep": 1600,
            "evidence_frames": 500,
            "evidence_substeps": 2000,
        },
        "dart_adapter": {
            "scene_id": "fbf_author_masonry_arch_25_crown_impact_current_source",
            "world": {
                "time_step_seconds": 1.0 / 240.0,
                "gravity_coordinate_units_per_s2": [0.0, 0.0, -9.81],
                "simulation_threads": 1,
                "deactivation_enabled": False,
            },
            "collision": {
                "detector": "native",
                "contact_manifold": "four_point_planar",
                "observed_four_point_planar": True,
                "max_contacts": 4096,
                "max_contacts_per_pair": 8,
            },
            "solver": module._expected_author_masonry_arch_solver_contract(solver_lane),
            "process_state": {"observed_contact_erp": 0.0},
            "inventory": {
                "stones": 25,
                "mobile_stones": 23,
                "cubes": 3,
                "cubes_released": False,
            },
            "schedule": {
                "evidence_runner_action_completed_step": 1600,
                "release_action_key": "p",
                "interactive_action_semantics": "immediate_on_invocation",
            },
        },
        "claim_boundary": {
            "source_numeric_geometry_mass_friction_and_initial_state_ported_to_dart": True,
            "source_release_action_ported_to_dart": True,
            "source_release_schedule_declared_for_evidence_runner": True,
            "interactive_demo_auto_releases_at_source_step": False,
            "source_collision_semantics_equivalent": False,
            "source_contact_gap_semantics_equivalent": False,
            "source_solver_backend_equivalent": False,
            "source_float32_semantics_equivalent": False,
            "trajectory_equivalent": False,
            "physical_outcome_equivalent": False,
            "fig07_parity": False,
            "video07_parity": False,
            "timing_comparable": False,
            "paper_parity": False,
        },
    }


def test_schedule_matrix_covers_every_requested_visual_case():
    module = _load_module()

    assert set(module.SCHEDULES) == {
        "incline",
        "backspin",
        "turntable_mu02_omega2",
        "turntable_mu02_omega5",
        "turntable_mu05_omega2",
        "turntable_mu05_omega5",
        "turntable_author_mu02_omega2",
        "turntable_author_mu02_omega5",
        "turntable_author_mu05_omega2",
        "turntable_author_mu05_omega5",
        "painleve_mu05",
        "painleve_mu055",
        "card_house_26",
        "masonry_arch_25_literal_standing",
        "masonry_arch_25_author_crown_impact_current_source",
        "masonry_arch_25",
        "masonry_arch_101",
        "card_house_10_construction",
        "card_house_author_5_construction",
        "card_house_10_dynamics",
    }
    assert module.TURN_TABLE_MEMBERS == (
        "turntable_mu02_omega2",
        "turntable_mu02_omega5",
        "turntable_mu05_omega2",
        "turntable_mu05_omega5",
    )
    assert module.AUTHOR_TURN_TABLE_MEMBERS == (
        "turntable_author_mu02_omega2",
        "turntable_author_mu02_omega5",
        "turntable_author_mu05_omega2",
        "turntable_author_mu05_omega5",
    )
    assert module.PAINLEVE_MEMBERS == ("painleve_mu05", "painleve_mu055")


def test_incline_mismatch_distinguishes_render_and_trace_contact_counts():
    module = _load_module()
    mismatches = module.SCHEDULES["incline"].mismatches

    assert any(
        "eight contacts per post-initial step in aggregate" in mismatch
        and "three contacts per post-initial step" in mismatch
        and "four per cell in the paper timing row" in mismatch
        for mismatch in mismatches
    )
    assert all(
        "current collision frontend reports three contacts per cell" not in mismatch
        for mismatch in mismatches
    )


def test_solver_lane_resolution_derives_boxed_and_records_static_skips():
    module = _load_module()
    source = [
        module.SCHEDULES["backspin"],
        module.SCHEDULES["turntable_author_mu02_omega2"],
        module.SCHEDULES["card_house_10_construction"],
    ]

    resolved, skips = module._resolve_solver_lanes(source, "both")

    assert [schedule.id for schedule in resolved] == [
        "backspin",
        "turntable_author_mu02_omega2",
        "backspin__boxed",
        "card_house_10_construction",
    ]
    assert {
        (skip["schedule_id"], skip["requested_solver_lane"], skip["reason"])
        for skip in skips
    } == {
        ("card_house_10_construction", "exact", "scene is boxed-only"),
        ("turntable_author_mu02_omega2", "boxed", "scene is exact-only"),
    }


def test_boxed_schedule_command_and_plan_bind_pre_run_toggle(tmp_path):
    module = _load_module()
    schedule = module._derive_boxed_schedule(module.SCHEDULES["backspin"])
    output_dir = tmp_path / schedule.id

    command = module.build_demo_command(schedule, Path("dart-demos"), output_dir)
    plan = module.schedule_plan(schedule, Path("dart-demos"), tmp_path)

    action_index = command.index("--headless-action")
    shot_index = command.index("--headless-shot-at")
    assert command[action_index : action_index + 2] == ["--headless-action", "e"]
    assert action_index < shot_index
    assert plan["solver_lane"] == "boxed"
    assert plan["source_schedule_id"] == "backspin"
    assert plan["expected_solver"] == module.BOXED_SOLVER_NAME
    assert plan["comparison_capture"] is True
    assert plan["evidence_ready"] is False
    assert plan["output"]["directory"].endswith("backspin__boxed")


def test_fail_fast_flag_is_routed_only_to_required_exact_schedules(tmp_path):
    module = _load_module()
    exact = _test_schedule(module, exact_required=True)
    nonrequired = _test_schedule(module, exact_required=False)
    boxed = module._derive_boxed_schedule(exact)

    exact_command = module.build_demo_command(exact, Path("dart-demos"), tmp_path)
    nonrequired_command = module.build_demo_command(
        nonrequired, Path("dart-demos"), tmp_path
    )
    boxed_command = module.build_demo_command(boxed, Path("dart-demos"), tmp_path)

    assert exact_command.count(module.HEADLESS_EXACT_FBF_FAIL_FAST_FLAG) == 1
    assert module.HEADLESS_EXACT_FBF_FAIL_FAST_FLAG not in nonrequired_command
    assert module.HEADLESS_EXACT_FBF_FAIL_FAST_FLAG not in boxed_command


def test_strict_exact_sidecar_requires_nontriggered_fail_fast_state(tmp_path):
    module = _load_module()
    schedule = _test_schedule(module, exact_required=True)
    output_dir = tmp_path / schedule.id
    _write_frames(module, schedule, output_dir)
    _write_sidecar(module, schedule, output_dir)
    report = module.validate_sidecar(schedule, output_dir)
    assert report["headless_exact_fbf_fail_fast"] == {
        "legacy_artifact": False,
        "state": {
            "enabled": True,
            "residual_tolerance": module.EXACT_FBF_RESIDUAL_TOLERANCE,
            "triggered": False,
            "step": None,
            "reason": None,
        },
    }
    sidecar_path = output_dir / "timeline.json"
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
    del sidecar["headless_exact_fbf_fail_fast"]
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")

    with pytest.raises(ValueError, match="fail-fast state is missing"):
        module.validate_sidecar(schedule, output_dir)


def test_failed_strict_capture_validates_partial_sidecar_and_skips_media(
    tmp_path, monkeypatch
):
    module = _load_module()
    schedule = _test_schedule(module, exact_required=True)
    output_root = tmp_path / "captures"
    output_dir = output_root / schedule.id
    output_dir.mkdir(parents=True)
    timeline = output_dir / "timeline.json"
    timeline.write_text("stale", encoding="utf-8")
    demo = tmp_path / "dart-demos"
    ffmpeg = tmp_path / "ffmpeg"
    ffprobe = tmp_path / "ffprobe"
    python = tmp_path / "python"
    for executable in (demo, ffmpeg, ffprobe, python):
        executable.write_bytes(b"executable")

    def fail_demo(argv, **_kwargs):
        assert not timeline.exists()
        _write_failed_sidecar(module, schedule, output_dir, Path(argv[0]))
        raise module.subprocess.CalledProcessError(1, argv)

    def reject_finalization(*_args, **_kwargs):
        pytest.fail("fail-fast capture must not finalize panels or media")

    monkeypatch.setattr(module, "_run", fail_demo)
    monkeypatch.setattr(module, "_prepare_panel_frames", reject_finalization)
    monkeypatch.setattr(module, "_compose_panel", reject_finalization)
    monkeypatch.setattr(module, "_encode_media", reject_finalization)

    with pytest.raises(
        ValueError,
        match="exact-FBF fail-fast triggered at completed step 1: exact_failure",
    ):
        module.run_schedule(
            schedule,
            demo=demo,
            output_root=output_root,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
            python=python,
            allow_long=False,
        )

    assert timeline.exists()
    assert not (output_dir / "metadata.json").exists()


def test_partial_fail_fast_sidecar_enforces_reason_priority(tmp_path):
    module = _load_module()
    schedule = _test_schedule(module, exact_required=True)
    output_dir = tmp_path / schedule.id
    demo = tmp_path / "dart-demos"
    _write_failed_sidecar(module, schedule, output_dir, demo)
    sidecar_path = output_dir / "timeline.json"
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
    sidecar["steps"][-1]["solver_diagnostics"]["boxed_lcp_fallbacks"] = 1
    sidecar["solver_diagnostics"]["boxed_lcp_fallbacks"] = 1
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")

    with pytest.raises(ValueError, match="does not match diagnostics priority"):
        module._validate_failed_exact_fbf_sidecar(
            schedule, output_dir, expected_demo=demo
        )


def test_partial_fail_fast_sidecar_rejects_an_earlier_trigger(tmp_path):
    module = _load_module()
    schedule = _test_schedule(module, exact_required=True)
    output_dir = tmp_path / schedule.id
    demo = tmp_path / "dart-demos"
    _write_failed_sidecar(module, schedule, output_dir, demo, step=2)
    sidecar_path = output_dir / "timeline.json"
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
    earlier = sidecar["steps"][1]["solver_diagnostics"]
    earlier["exact_attempts"] = 1
    earlier["exact_failures"] = 1
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")

    with pytest.raises(ValueError, match="should have triggered earlier at step 1"):
        module._validate_failed_exact_fbf_sidecar(
            schedule, output_dir, expected_demo=demo
        )


def test_partial_fail_fast_sidecar_rejects_cumulative_counter_regression(tmp_path):
    module = _load_module()
    schedule = _test_schedule(module, exact_required=True)
    output_dir = tmp_path / schedule.id
    demo = tmp_path / "dart-demos"
    _write_failed_sidecar(module, schedule, output_dir, demo, step=2)
    sidecar_path = output_dir / "timeline.json"
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
    earlier = sidecar["steps"][1]["solver_diagnostics"]
    earlier.update(
        {
            "exact_attempts": 2,
            "exact_solves": 2,
            "residual": 5e-7,
            "worst_residual": 5e-7,
        }
    )
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")

    with pytest.raises(ValueError, match="cumulative exact_attempts regressed"):
        module._validate_failed_exact_fbf_sidecar(
            schedule, output_dir, expected_demo=demo
        )


@pytest.mark.parametrize(
    ("case", "message"),
    [
        ("missing_shot", "shots do not match"),
        ("failed_shot", "shot prefix differs"),
        ("failed_action", "action prefix differs"),
        ("missing_event", "events do not match"),
        ("failed_event", "event prefix differs"),
    ],
)
def test_partial_fail_fast_sidecar_rejects_invalid_prior_event(tmp_path, case, message):
    module = _load_module()
    schedule = _test_schedule(
        module, action=case == "failed_action", exact_required=True
    )
    output_dir = tmp_path / schedule.id
    demo = tmp_path / "dart-demos"
    _write_failed_sidecar(module, schedule, output_dir, demo)
    sidecar_path = output_dir / "timeline.json"
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
    if case == "missing_shot":
        sidecar["shots"].pop(0)
        sidecar["events"].pop(0)
    elif case == "failed_shot":
        sidecar["shots"][0]["success"] = False
        sidecar["events"][0]["success"] = False
    elif case == "failed_action":
        sidecar["actions"][0]["success"] = False
        sidecar["events"][1]["success"] = False
    elif case == "missing_event":
        sidecar["events"].pop(0)
    else:
        sidecar["events"][0]["success"] = False
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")

    with pytest.raises(ValueError, match=message):
        module._validate_failed_exact_fbf_sidecar(
            schedule, output_dir, expected_demo=demo
        )


def test_fail_fast_reason_allows_no_attempt_residual_placeholders():
    module = _load_module()

    assert (
        module._expected_fail_fast_reason(
            {
                "boxed_lcp_fallbacks": 0,
                "exact_failures": 0,
                "accepted_at_cap": 0,
                "exact_attempts": 0,
                "residual": 2.0,
                "worst_residual": None,
            }
        )
        is None
    )


def test_partial_fail_fast_sidecar_rejects_trigger_step_event(tmp_path):
    module = _load_module()
    schedule = _test_schedule(module, exact_required=True)
    output_dir = tmp_path / schedule.id
    demo = tmp_path / "dart-demos"
    _write_failed_sidecar(module, schedule, output_dir, demo)
    sidecar_path = output_dir / "timeline.json"
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
    sidecar["events"].append({"sequence": 0, "type": "shot", "step": 1})
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")

    with pytest.raises(ValueError, match="events exist at or after"):
        module._validate_failed_exact_fbf_sidecar(
            schedule, output_dir, expected_demo=demo
        )


def test_boxed_sidecar_binds_solver_and_unavailable_exact_diagnostics(tmp_path):
    module = _load_module()
    schedule = module._derive_boxed_schedule(
        _test_schedule(module, exact_required=True)
    )
    output_dir = tmp_path / schedule.id
    _write_frames(module, schedule, output_dir)
    _write_sidecar(module, schedule, output_dir)

    report = module.validate_sidecar(schedule, output_dir)

    assert report["final_solver_diagnostics"] == {
        "solver": module.BOXED_SOLVER_NAME,
        "available": False,
        "gap": module.BOXED_DIAGNOSTICS_GAP,
    }
    assert report["pass"] is True


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("solver", "ExactCoulombFbfConstraintSolver", "active solver"),
        ("available", True, "exposes exact-FBF diagnostics"),
        ("gap", "unknown solver", "diagnostic gap"),
    ],
)
def test_boxed_sidecar_rejects_unbound_solver_contract(tmp_path, field, value, message):
    module = _load_module()
    schedule = module._derive_boxed_schedule(
        _test_schedule(module, exact_required=True)
    )
    output_dir = tmp_path / schedule.id
    _write_frames(module, schedule, output_dir)
    _write_sidecar(module, schedule, output_dir)
    sidecar_path = output_dir / "timeline.json"
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
    for item in sidecar["steps"]:
        item["solver_diagnostics"][field] = value
    for shot in sidecar["shots"]:
        shot["solver_diagnostics"][field] = value
    sidecar["solver_diagnostics"][field] = value
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")

    with pytest.raises(ValueError, match=message):
        module.validate_sidecar(schedule, output_dir)


def test_main_plan_both_emits_lane_separated_schedule_ids(tmp_path, capsys):
    module = _load_module()

    result = module.main(
        [
            "plan",
            "--scenario",
            "backspin",
            "--solver-lane",
            "both",
            "--output-root",
            str(tmp_path),
        ]
    )
    payload = json.loads(capsys.readouterr().out)

    assert result == 0
    assert payload["requested_solver_lane"] == "both"
    assert [schedule["id"] for schedule in payload["schedules"]] == [
        "backspin",
        "backspin__boxed",
    ]
    assert set(payload["group_outputs"]) == {"backspin__exact_vs_boxed"}
    comparison = payload["group_outputs"]["backspin__exact_vs_boxed"]
    assert comparison["members"] == ["backspin", "backspin__boxed"]
    assert comparison["solver_lane"] == "both"
    assert comparison["labels"] == list(module.SOLVER_COMPARISON_LABELS)
    assert comparison["panel_labels"] == list(module.SOLVER_COMPARISON_LABELS)
    assert comparison["layout"] == "two synchronized solver lanes"
    assert comparison["panel_step"] == module.SCHEDULES["backspin"].panel_steps[-1]
    assert payload["solver_lane_skips"] == []


@pytest.mark.parametrize(
    ("solver_lane", "expected_schedule_id"),
    [("exact", "backspin"), ("boxed", "backspin__boxed")],
)
def test_main_plan_single_lane_excludes_solver_comparison(
    tmp_path, capsys, solver_lane, expected_schedule_id
):
    module = _load_module()

    result = module.main(
        [
            "plan",
            "--scenario",
            "backspin",
            "--solver-lane",
            solver_lane,
            "--output-root",
            str(tmp_path),
        ]
    )
    payload = json.loads(capsys.readouterr().out)

    assert result == 0
    assert payload["requested_solver_lane"] == solver_lane
    assert [schedule["id"] for schedule in payload["schedules"]] == [
        expected_schedule_id
    ]
    assert payload["group_outputs"] == {}


def test_demo_sidecar_records_exact_and_boxed_solver_identity():
    source = (ROOT / "examples/demos/DemoHost.cpp").read_text(encoding="utf-8")

    assert "#include <dart/constraint/BoxedLcpConstraintSolver.hpp>" in source
    assert 'snapshot.solver = "BoxedLcpConstraintSolver"' in source
    assert 'snapshot.solver = "ExactCoulombFbfConstraintSolver"' in source
    assert "writeJsonString(out, snapshot.solver);" in source


def test_demo_parameter_scene_factories_are_declared_and_registered():
    source = (ROOT / "examples/demos/scenes/FbfPaperFrictionScene.cpp").read_text(
        encoding="utf-8"
    )
    shared_specs = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (
            ROOT / "examples/demos/scenes/FbfAuthorTurntableSpec.hpp",
            ROOT / "examples/demos/scenes/FbfAuthorMasonryArchSpec.hpp",
            ROOT / "examples/demos/scenes/FbfAuthorMasonryArchDartAdapter.hpp",
        )
    )
    declarations = (ROOT / "examples/demos/scenes/Scenes.hpp").read_text(
        encoding="utf-8"
    )
    registry = (ROOT / "examples/demos/Registry.cpp").read_text(encoding="utf-8")
    normalized_source = " ".join(source.split())
    normalized_declarations = " ".join(declarations.split())
    factories = {
        "makeFbfPaperTurntableMu02Omega2Scene": ("fbf_paper_turntable_mu_0_2_omega_2"),
        "makeFbfPaperTurntableMu02Omega5Scene": ("fbf_paper_turntable_mu_0_2_omega_5"),
        "makeFbfPaperTurntableMu05Omega5Scene": ("fbf_paper_turntable_mu_0_5_omega_5"),
        "makeFbfAuthorTurntableMu02Omega2Scene": (
            "fbf_author_turntable_mu_0_2_omega_2"
        ),
        "makeFbfAuthorTurntableMu02Omega5Scene": (
            "fbf_author_turntable_mu_0_2_omega_5"
        ),
        "makeFbfAuthorTurntableMu05Omega2Scene": (
            "fbf_author_turntable_mu_0_5_omega_2"
        ),
        "makeFbfAuthorTurntableMu05Omega5Scene": (
            "fbf_author_turntable_mu_0_5_omega_5"
        ),
        "makeFbfPaperPainleveMu055Scene": "fbf_paper_painleve_mu_0_55",
        "makeFbfPaperCardHouse10DynamicScene": ("fbf_paper_card_house_10_dynamic"),
        "makeFbfPaperMasonryArch25LiteralStandingScene": (
            "fbf_paper_masonry_arch_25_literal_standing"
        ),
        "makeFbfAuthorMasonryArch25CrownImpactCurrentSourceScene": (
            "fbf_author_masonry_arch_25_crown_impact_current_source"
        ),
    }

    for factory, scene_id in factories.items():
        assert f"DemoScene {factory}()" in normalized_source
        assert scene_id in source or scene_id in shared_specs
        assert f"DemoScene {factory}();" in normalized_declarations
        assert registry.count(f"scenes.push_back({factory}());") == 1

    assert "makeFbfPaperCardHouse10Scene" in registry
    assert "makeFbfPaperCardHouse10DynamicScene" in registry
    assert "known to saturate and is not the natural manifold" in source


def test_fbf_scene_docs_verifier_includes_author_and_paper_scenes():
    source = (ROOT / "examples/demos/DemoHost.cpp").read_text(encoding="utf-8")
    start = source.index("int DemoHost::verifyFbfSceneDocs() const")
    end = source.index(
        "//==============================================================================",
        start,
    )
    verifier = source[start:end]

    assert 'startsWith(scene.id, "fbf_")' in verifier
    assert 'startsWith(scene.id, "fbf_paper_")' not in verifier
    assert "FBF research scene(s)" in verifier


def test_backspin_checker_texture_is_visual_only_and_renderer_bound():
    source = (ROOT / "examples/demos/scenes/FbfPaperFrictionScene.cpp").read_text(
        encoding="utf-8"
    )
    start = source.index("void addBackspinCheckerTexture")
    end = source.index(
        "//==============================================================================",
        start,
    )
    helper = source[start:end]

    assert '"dart://sample/obj/fbf_backspin_checker_sphere.obj"' in helper
    assert "DartResourceRetriever::create()" in helper
    assert "MeshShape::loadMesh(meshUri, retriever)" in helper
    assert "createShapeNodeWith<VisualAspect>(checker)" in helper
    assert "CollisionAspect" not in helper
    assert "DynamicsAspect" not in helper
    assert "addBackspinCheckerTexture(body);" in source

    sphere_start = source.index("SkeletonPtr createBackspinSphere()")
    sphere_end = source.index(
        "//==============================================================================",
        sphere_start,
    )
    sphere = source[sphere_start:sphere_end]
    assert "createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape)" in sphere
    assert "setShapeInertia(body, shape);" in sphere
    assert "addBackspinCheckerTexture(body);" in sphere

    obj = (ROOT / "data/obj/fbf_backspin_checker_sphere.obj").read_text(
        encoding="utf-8"
    )
    material = (ROOT / "data/obj/fbf_backspin_checker_sphere.mtl").read_text(
        encoding="utf-8"
    )
    texture = (ROOT / "data/obj/fbf_backspin_checker.ppm").read_text(encoding="utf-8")
    assert "mtllib fbf_backspin_checker_sphere.mtl" in obj
    assert "usemtl FbfBackspinChecker" in obj
    assert "\nvt " in obj
    assert "map_Kd fbf_backspin_checker.ppm" in material
    assert "Ke 0.180000 0.180000 0.180000" in material
    assert texture.startswith("P3\n")
    assert "64 32\n255\n" in texture
    assert "6x4 high-contrast checker" in texture

    scene_start = source.index("DemoScene makeFbfPaperBackspinScene()")
    scene_end = source.index(
        "//==============================================================================",
        scene_start,
    )
    scene = source[scene_start:scene_end]
    assert "::osg::Vec3d(-0.5, -1.25, 5.5)" in scene
    assert "::osg::Vec3d(-0.5, 0.0, 0.2)" in scene
    assert "::osg::Vec3d(0.0, 1.0, 0.0)" in scene

    module = _load_module()
    configuration = dict(module.SCHEDULES["backspin"].configuration)
    assert configuration["orientation_cue"] == (
        "renderer-applied high-contrast 6x4 checker texture with coral "
        "registration tile"
    )


def test_source_video_segments_are_contiguous_and_match_82_seconds():
    module = _load_module()

    assert module.VIDEO_SEGMENTS[0].start_seconds == 0
    assert module.VIDEO_SEGMENTS[-1].end_seconds == 82
    assert all(
        left.end_seconds == right.start_seconds
        for left, right in zip(module.VIDEO_SEGMENTS, module.VIDEO_SEGMENTS[1:])
    )
    assert "top: mu=.2 omega=2,5" in module.VIDEO_SEGMENTS[3].layout
    assert module.VIDEO_SEGMENTS[5].layout == (
        "Ours/MuJoCo/Kamino horizontal three-column row"
    )


def test_every_schedule_has_explicit_non_parity_metadata():
    module = _load_module()

    for schedule in module.SCHEDULES.values():
        plan = module.schedule_plan(schedule, Path("dart-demos"), Path("/tmp/out"))
        assert plan["actual_simulator_required"] is True
        assert plan["generated_imagery_allowed"] is False
        assert plan["paper_comparable"] is False
        assert plan["known_mismatches"]
        assert plan["runnable"] == (plan["adapter_gap"] is None)

    card_plan = module.schedule_plan(
        module.SCHEDULES["card_house_26"], Path("dart-demos"), Path("/tmp/out")
    )
    assert card_plan["evidence_ready"] is False
    assert "1.244e-5" in card_plan["known_gate_blockers"][0]

    card10_plan = module.schedule_plan(
        module.SCHEDULES["card_house_10_dynamics"],
        Path("dart-demos"),
        Path("/tmp/out"),
    )
    assert card10_plan["runnable"] is True
    assert card10_plan["evidence_ready"] is False
    assert (
        "512-contact budget is known to saturate"
        in card10_plan["known_gate_blockers"][1]
    )


def test_parameterized_schedules_use_stable_runnable_scene_ids():
    module = _load_module()

    blocked = {
        schedule.id for schedule in module.SCHEDULES.values() if not schedule.runnable
    }
    assert blocked == set()
    assert {
        name: module.SCHEDULES[name].scene
        for name in (
            "turntable_mu02_omega2",
            "turntable_mu02_omega5",
            "turntable_mu05_omega2",
            "turntable_mu05_omega5",
            "turntable_author_mu02_omega2",
            "turntable_author_mu02_omega5",
            "turntable_author_mu05_omega2",
            "turntable_author_mu05_omega5",
            "painleve_mu05",
            "painleve_mu055",
            "card_house_10_construction",
            "card_house_author_5_construction",
            "card_house_10_dynamics",
            "masonry_arch_25_literal_standing",
            "masonry_arch_25_author_crown_impact_current_source",
        )
    } == {
        "turntable_mu02_omega2": "fbf_paper_turntable_mu_0_2_omega_2",
        "turntable_mu02_omega5": "fbf_paper_turntable_mu_0_2_omega_5",
        "turntable_mu05_omega2": "fbf_paper_turntable",
        "turntable_mu05_omega5": "fbf_paper_turntable_mu_0_5_omega_5",
        "turntable_author_mu02_omega2": "fbf_author_turntable_mu_0_2_omega_2",
        "turntable_author_mu02_omega5": "fbf_author_turntable_mu_0_2_omega_5",
        "turntable_author_mu05_omega2": "fbf_author_turntable_mu_0_5_omega_2",
        "turntable_author_mu05_omega5": "fbf_author_turntable_mu_0_5_omega_5",
        "painleve_mu05": "fbf_paper_painleve",
        "painleve_mu055": "fbf_paper_painleve_mu_0_55",
        "card_house_10_construction": "fbf_paper_card_house_10",
        "card_house_author_5_construction": ("fbf_author_card_house_5_construction"),
        "card_house_10_dynamics": "fbf_paper_card_house_10_dynamic",
        "masonry_arch_25_literal_standing": (
            "fbf_paper_masonry_arch_25_literal_standing"
        ),
        "masonry_arch_25_author_crown_impact_current_source": (
            "fbf_author_masonry_arch_25_crown_impact_current_source"
        ),
    }
    command = module.build_demo_command(
        module.SCHEDULES["turntable_mu02_omega2"],
        Path("dart-demos"),
        Path("/tmp/out"),
    )
    assert command[command.index("--scene") + 1] == (
        "fbf_paper_turntable_mu_0_2_omega_2"
    )


def test_capture_schedules_select_the_required_collision_frontend():
    module = _load_module()
    output_root = Path("/tmp/out")

    native_members = set(module.AUTHOR_TURN_TABLE_MEMBERS) | {
        "card_house_author_5_construction",
        "masonry_arch_25_literal_standing",
        "masonry_arch_25_author_crown_impact_current_source",
    }
    legacy_members = set(module.SCHEDULES) - native_members
    assert {
        module.SCHEDULES[member].collision_detector for member in legacy_members
    } == {"dart"}

    legacy = module.SCHEDULES["turntable_mu02_omega2"]
    legacy_command = module.build_demo_command(
        legacy, Path("dart-demos"), output_root / legacy.id
    )
    legacy_plan = module.schedule_plan(legacy, Path("dart-demos"), output_root)
    assert legacy_command[legacy_command.index("--collision-detector") + 1] == "dart"
    assert legacy_plan["collision_detector"] == "dart"
    assert legacy_plan["collision_detector_override"] is True

    for member in native_members:
        schedule = module.SCHEDULES[member]
        command = module.build_demo_command(
            schedule, Path("dart-demos"), output_root / schedule.id
        )
        plan = module.schedule_plan(schedule, Path("dart-demos"), output_root)
        assert schedule.collision_detector == "native"
        assert schedule.collision_detector_override is False
        assert "--collision-detector" not in command
        assert plan["collision_detector"] == "native"
        assert plan["collision_detector_override"] is False


def test_literal_arch_contract_binds_exact_and_boxed_physics(tmp_path, monkeypatch):
    module = _load_module()
    exact = module.SCHEDULES["masonry_arch_25_literal_standing"]
    boxed = module._derive_boxed_schedule(exact)
    exact_contract = _literal_arch_physics_contract(module)
    monkeypatch.setattr(
        module,
        "LITERAL_MASONRY_ARCH_GEOMETRY_FINGERPRINT",
        exact_contract["physical_geometry_fingerprint"]["value"],
    )

    exact_report = module._validate_literal_masonry_arch_contract(
        exact,
        {"physics_contract": exact_contract},
        sidecar_path=tmp_path / "exact.json",
    )
    boxed_report = module._validate_literal_masonry_arch_contract(
        boxed,
        {
            "physics_contract": _literal_arch_physics_contract(
                module, solver_lane="boxed"
            )
        },
        sidecar_path=tmp_path / "boxed.json",
    )

    assert exact_report["solver_lane"] == "exact_fbf"
    assert boxed_report["solver_lane"] == "boxed_lcp"
    assert exact_report["stone_count"] == boxed_report["stone_count"] == 25
    assert exact_report["scoped_erp"] == boxed_report["scoped_erp"] == 0.0


def test_author_arch_adapter_contract_binds_exact_and_boxed_initial_state(tmp_path):
    module = _load_module()
    exact = module.SCHEDULES["masonry_arch_25_author_crown_impact_current_source"]
    boxed = module._derive_boxed_schedule(exact)
    exact_report = module._validate_author_masonry_arch_adapter_contract(
        exact,
        {"physics_contract": _author_masonry_arch_adapter_contract(module)},
        sidecar_path=tmp_path / "exact.json",
    )
    boxed_report = module._validate_author_masonry_arch_adapter_contract(
        boxed,
        {
            "physics_contract": _author_masonry_arch_adapter_contract(
                module, solver_lane="boxed"
            )
        },
        sidecar_path=tmp_path / "boxed.json",
    )

    assert exact_report["solver_lane"] == "exact_fbf"
    assert boxed_report["solver_lane"] == "boxed_lcp"
    assert exact_report["stone_count"] == boxed_report["stone_count"] == 25
    assert exact_report["cube_count"] == boxed_report["cube_count"] == 3
    assert exact_report["release_action_completed_step"] == 1600
    assert exact_report["scoped_erp"] == boxed_report["scoped_erp"] == 0.0
    assert exact_report["physical_outcome_validated"] is False


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        (("source_binding", "configuration_spec_sha256"), "0" * 64, "hashes"),
        (("source_configuration", "coordinate_scale"), 0.01, "configuration"),
        (("source_configuration", "cube_edge"), 0.03, "configuration"),
        (("dart_adapter", "collision", "contact_manifold"), "compact", "collision"),
        (("dart_adapter", "solver", "lane"), "boxed_lcp", "solver"),
        (
            ("dart_adapter", "solver", "exact_options", "step_size_scale"),
            10.0,
            "solver",
        ),
        (
            ("dart_adapter", "solver", "exact_options", "inner_max_sweeps"),
            120,
            "solver",
        ),
        (
            (
                "dart_adapter",
                "solver",
                "cross_step_options",
                "warm_start_match_mode",
            ),
            "ordered_body_b_local_feature",
            "solver",
        ),
        (("dart_adapter", "process_state", "observed_contact_erp"), 0.01, "ERP"),
        (("dart_adapter", "inventory", "cubes"), 4, "inventory"),
        (
            (
                "dart_adapter",
                "schedule",
                "evidence_runner_action_completed_step",
            ),
            1601,
            "release schedule",
        ),
        (("claim_boundary", "fig07_parity"), True, "claim boundary"),
    ],
)
def test_author_arch_adapter_contract_rejects_mutation(tmp_path, field, value, message):
    module = _load_module()
    schedule = module.SCHEDULES["masonry_arch_25_author_crown_impact_current_source"]
    contract = _author_masonry_arch_adapter_contract(module)
    target = contract
    for key in field[:-1]:
        target = target[key]
    target[field[-1]] = value

    with pytest.raises(ValueError, match=message):
        module._validate_author_masonry_arch_adapter_contract(
            schedule,
            {"physics_contract": contract},
            sidecar_path=tmp_path / "timeline.json",
        )


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        (("source_binding", "spec_sha256"), "0" * 64, "source hashes"),
        (("process_state", "observed_contact_erp"), 0.01, "scoped ERP"),
        (("collision", "contact_manifold"), "compact", "collision contract"),
        (("stones", 0, "mobile"), True, "physical properties"),
        (("stones", 1, "vertices"), [], "wedge vertices"),
        (("stones", 1, "mass_kg"), 123.0, "physical geometry"),
        (("stones", 1, "body_pose", "translation", 0), 0.25, "physical geometry"),
        (("stones", 1, "vertices", 0, 0), 0.25, "physical geometry"),
        (("stones", 1, "triangles", 0, 0), 1, "physical geometry"),
        (
            ("physical_geometry_fingerprint", "value"),
            "0" * 16,
            "geometry fingerprint",
        ),
    ],
)
def test_literal_arch_contract_rejects_mutated_runtime_physics(
    tmp_path, monkeypatch, field, value, message
):
    module = _load_module()
    schedule = module.SCHEDULES["masonry_arch_25_literal_standing"]
    contract = json.loads(json.dumps(_literal_arch_physics_contract(module)))
    monkeypatch.setattr(
        module,
        "LITERAL_MASONRY_ARCH_GEOMETRY_FINGERPRINT",
        contract["physical_geometry_fingerprint"]["value"],
    )
    target = contract
    for key in field[:-1]:
        target = target[key]
    target[field[-1]] = value

    with pytest.raises(ValueError, match=message):
        module._validate_literal_masonry_arch_contract(
            schedule,
            {"physics_contract": contract},
            sidecar_path=tmp_path / "timeline.json",
        )


def test_published_panel_instants_and_completed_step_actions_are_fixed():
    module = _load_module()

    assert module.SCHEDULES["backspin"].panel_steps == (0, 10, 50, 120, 130)
    assert module.SCHEDULES["incline"].panel_steps == (0, 30, 60, 90, 120)
    card = module.SCHEDULES["card_house_26"]
    assert card.panel_steps == (0, 120, 402, 420, 600)
    assert card.actions == (
        module.ScheduledAction(402, "p", "launch four reconstructed projectiles"),
    )
    arch = module.SCHEDULES["masonry_arch_25"]
    assert arch.actions[0].step == 240
    literal_arch = module.SCHEDULES["masonry_arch_25_literal_standing"]
    assert literal_arch.total_steps == 600
    assert literal_arch.frame_stride == 10
    assert literal_arch.panel_steps == (0, 120, 300, 600)
    assert literal_arch.actions == ()

    author_impact = module.SCHEDULES[
        "masonry_arch_25_author_crown_impact_current_source"
    ]
    assert author_impact.total_steps == 2000
    assert author_impact.time_step_seconds == pytest.approx(1.0 / 240.0)
    assert author_impact.frame_stride == 8
    assert len(author_impact.video_steps) == 251
    assert author_impact.panel_steps == (0, 1200, 1600, 1945, 2000)
    assert author_impact.actions == (
        module.ScheduledAction(1600, "p", "release the three existing source cubes"),
    )
    assert "pre-release" in author_impact.panel_labels[2]


def test_author_masonry_impact_schedule_is_fail_closed_and_source_pinned():
    module = _load_module()
    schedule = module.SCHEDULES["masonry_arch_25_author_crown_impact_current_source"]
    plan = module.schedule_plan(schedule, Path("dart-demos"), Path("/tmp/out"))
    command = module.build_demo_command(schedule, Path("dart-demos"), Path("/tmp/out"))
    configuration = schedule.configuration_dict()

    assert schedule.scene == "fbf_author_masonry_arch_25_crown_impact_current_source"
    assert schedule.source_segment == "masonry_arch_25"
    assert schedule.supported_solver_lanes == module.SOLVER_LANES
    assert schedule.exact_fbf_required is True
    assert schedule.collision_detector == "native"
    assert schedule.collision_detector_override is False
    assert configuration["author_commit"] == (
        "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0"
    )
    assert configuration["release_substep"] == "1600"
    assert configuration["total_substeps"] == "2000"
    assert schedule.known_gate_blockers
    assert plan["evidence_ready"] is False
    assert "--collision-detector" not in command
    assert module.HEADLESS_EXACT_FBF_FAIL_FAST_FLAG in command
    assert command[command.index("--steps") + 1] == "2000"
    shot_value = f"1600:{module._frame_path(Path('/tmp/out'), 1600)}"
    assert command.index(shot_value) < command.index("1600:p")

    boxed = module._derive_boxed_schedule(schedule)
    assert boxed.source_schedule_id == schedule.id
    assert boxed.solver_lane == "boxed"
    assert boxed.exact_fbf_required is False


def test_demo_command_requests_shots_before_same_step_actions():
    module = _load_module()
    schedule = module.SCHEDULES["card_house_26"]
    command = module.build_demo_command(schedule, Path("dart-demos"), Path("/tmp/card"))

    shot_value = f"402:{module._frame_path(Path('/tmp/card'), 402)}"
    shot_index = command.index(shot_value)
    action_index = command.index("402:p")
    assert shot_index < action_index
    assert command[command.index("--threads") + 1] == "1"
    assert command[command.index("--steps") + 1] == "600"


def test_plan_reports_all_parameter_adapters_as_runnable():
    module = _load_module()
    plan = module.build_plan(
        module.SCHEDULES.values(), Path("dart-demos"), Path("/tmp/out")
    )

    assert plan["schema_version"] == module.SCHEMA_VERSION
    assert plan["pass"] is True
    assert plan["all_schedules_runnable"] is True
    assert plan["blocked_schedules"] == []
    assert plan["group_outputs"]["turntable"]["layout"] == "2x2 in source order"
    assert plan["group_outputs"]["turntable"]["members"] == list(
        module.TURN_TABLE_MEMBERS
    )
    assert plan["group_outputs"]["turntable_author"]["members"] == list(
        module.AUTHOR_TURN_TABLE_MEMBERS
    )
    author_group = plan["group_outputs"]["turntable_author"]
    assert "panel_step" not in author_group
    assert author_group["panel_sources"] == [
        {
            "member": member,
            "step": step,
            "time_seconds": module.SCHEDULES[member].time_at_step(step),
        }
        for member, step in zip(module.AUTHOR_TURN_TABLE_MEMBERS, (136, 120, 360, 90))
    ]
    assert author_group["labels"] == [
        "mu=0.2, omega=2 rad/s",
        "mu=0.2, omega=5 rad/s",
        "mu=0.5, omega=2 rad/s",
        "mu=0.5, omega=5 rad/s",
    ]
    assert author_group["panel_labels"] == [
        "MU 0.2 OMEGA 2 T 2.27S",
        "MU 0.2 OMEGA 5 T 2.00S",
        "MU 0.5 OMEGA 2 T 6.00S",
        "MU 0.5 OMEGA 5 T 1.50S",
    ]
    assert plan["group_outputs"]["painleve"]["labels"] == [
        "MU .5 SLIDE REST",
        "MU .55 SHORT TRAVEL TUMBLE",
    ]


def test_author_turntable_schedule_is_source_pinned_and_real_time_sampled():
    module = _load_module()
    expected_outcomes = {
        "turntable_author_mu02_omega2": "ejected",
        "turntable_author_mu02_omega5": "ejected",
        "turntable_author_mu05_omega2": "retained_through_6s",
        "turntable_author_mu05_omega5": "ejected",
    }

    for member in module.AUTHOR_TURN_TABLE_MEMBERS:
        schedule = module.SCHEDULES[member]
        configuration = schedule.configuration_dict()
        assert schedule.total_steps == 360
        assert schedule.frame_stride == 2
        assert schedule.panel_steps == (0, 30, 60, 180, 360)
        assert len(schedule.video_steps) == 181
        assert configuration["author_commit"] == (
            "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0"
        )
        assert configuration["drop_height_m"] == "0.2"
        assert configuration["settle_seconds"] == "0.5"
        assert configuration["smoothstep_ramp_seconds"] == "0.5"
        assert configuration["outcome"] == expected_outcomes[member]

    group = module.GROUP_OUTPUTS["turntable_author"]
    assert group.panel_step is None
    assert group.panel_steps == (136, 120, 360, 90)
    assert group.resolved_panel_steps == (136, 120, 360, 90)
    assert group.resolved_panel_labels == group.panel_labels
    assert group.members == module.AUTHOR_TURN_TABLE_MEMBERS


def test_author_card_house_construction_schedule_has_explicit_clock_and_boundary():
    module = _load_module()
    schedule = module.SCHEDULES["card_house_author_5_construction"]
    configuration = schedule.configuration_dict()
    plan = module.schedule_plan(schedule, Path("dart-demos"), Path("/tmp/out"))
    command = module.build_demo_command(schedule, Path("dart-demos"), Path("/tmp/out"))

    assert schedule.scene == "fbf_author_card_house_5_construction"
    assert schedule.total_steps == 0
    assert schedule.time_step_seconds == pytest.approx(1.0 / 240.0)
    assert schedule.capture_steps == (0,)
    assert schedule.panel_steps == (0,)
    assert schedule.video_steps == ()
    assert schedule.encode_mp4 is False
    assert schedule.encode_gif is False
    assert schedule.expect_motion is False
    assert schedule.exact_fbf_required is False
    assert schedule.collision_detector == "native"
    assert schedule.collision_detector_override is False
    assert configuration == {
        "levels": "5",
        "mode": "static source-configuration port",
        "simulation_time_step_seconds": "1/240",
        "display_time_step_seconds": "1/60",
        "substeps_per_display_frame": "4",
        "release_substep": "1600",
        "total_substeps": "3200",
        "evidence_scope": "configuration-only",
        "parity_scope": "non-parity",
    }
    assert plan["time_step_seconds"] == pytest.approx(1.0 / 240.0)
    assert plan["actual_simulator_required"] is True
    assert plan["generated_imagery_allowed"] is False
    assert plan["paper_comparable"] is False
    assert plan["output"]["mp4"] is None
    assert plan["output"]["gif"] is None
    assert "--collision-detector" not in command
    assert command[command.index("--steps") + 1] == "0"
    assert command.count("--headless-shot-at") == 1
    assert any("configuration" in mismatch for mismatch in schedule.mismatches)
    assert any("not the authors'" in mismatch for mismatch in schedule.mismatches)


def test_group_output_spec_rejects_ambiguous_or_misaligned_panel_steps():
    module = _load_module()
    group = module.GROUP_OUTPUTS["turntable_author"]

    with pytest.raises(ValueError, match="either one shared panel step"):
        module.dataclasses.replace(group, panel_step=360)
    with pytest.raises(ValueError, match="panel steps and members differ"):
        module.dataclasses.replace(group, panel_steps=(136, 120, 360))
    with pytest.raises(ValueError, match="must be nonnegative"):
        module.dataclasses.replace(group, panel_steps=(136, 120, 360, -1))
    with pytest.raises(ValueError, match="panel labels and members differ"):
        module.dataclasses.replace(group, panel_labels=("one",))


def test_solver_comparison_group_derivation_binds_exact_then_boxed():
    module = _load_module()
    schedule = module.SCHEDULES["backspin"]

    group = module._derive_solver_comparison_group(schedule)

    assert group.id == "backspin__exact_vs_boxed"
    assert group.source_segment == schedule.source_segment
    assert group.members == ("backspin", "backspin__boxed")
    assert group.labels == module.SOLVER_COMPARISON_LABELS
    assert group.resolved_panel_labels == module.SOLVER_COMPARISON_LABELS
    assert group.layout == "side-by-side"
    assert group.panel_step == schedule.panel_steps[-1]
    assert group.solver_lane == "both"
    assert group.source_group_id is None

    assert module._solver_comparison_groups([schedule], "both") == [group]
    assert module._solver_comparison_groups([schedule], "exact") == []
    assert module._solver_comparison_groups([schedule], "boxed") == []


def test_solver_comparison_group_rejects_unsupported_sources_and_contracts():
    module = _load_module()
    group = module._derive_solver_comparison_group(module.SCHEDULES["backspin"])

    with pytest.raises(ValueError, match="cannot be derived"):
        module._derive_solver_comparison_group(
            module.SCHEDULES["turntable_author_mu02_omega2"]
        )
    with pytest.raises(ValueError, match="exact then boxed"):
        module.dataclasses.replace(group, members=tuple(reversed(group.members)))
    with pytest.raises(ValueError, match="identify only the solver lanes"):
        module.dataclasses.replace(group, labels=("EXACT", "BOXED"))
    with pytest.raises(ValueError, match="panel labels must identify"):
        module.dataclasses.replace(group, panel_labels=("REST", "TUMBLE"))
    with pytest.raises(ValueError, match="unsupported solver lane"):
        module.dataclasses.replace(group, solver_lane="hybrid")


def test_boxed_painleve_group_uses_solver_neutral_parameter_labels():
    module = _load_module()
    exact = module.GROUP_OUTPUTS["painleve"]

    boxed = module._derive_boxed_group(exact)

    assert exact.labels == (
        "MU .5 SLIDE REST",
        "MU .55 SHORT TRAVEL TUMBLE",
    )
    assert boxed.labels == (
        "EXISTING BOXED LCP | PAINLEVE PROXY MU=.5",
        "EXISTING BOXED LCP | PAINLEVE PROXY MU=.55",
    )
    assert boxed.resolved_panel_labels == boxed.labels
    assert boxed.solver_lane == "boxed"
    assert boxed.source_group_id == "painleve"
    assert all(
        outcome not in " ".join(boxed.labels).lower()
        for outcome in ("rest", "travel", "tumble")
    )


@pytest.mark.parametrize("lane_order", ["mixed", "reversed"])
def test_solver_comparison_group_rejects_mixed_or_reversed_lanes(tmp_path, lane_order):
    module = _load_module()
    exact = module.SCHEDULES["backspin"]
    boxed = module._derive_boxed_schedule(exact)
    group = module._derive_solver_comparison_group(exact)

    if lane_order == "mixed":
        schedules = [
            exact,
            module.dataclasses.replace(
                boxed,
                solver_lane="exact",
                supported_solver_lanes=("exact",),
            ),
        ]
    else:
        schedules = [
            module.dataclasses.replace(
                exact,
                solver_lane="boxed",
                supported_solver_lanes=("boxed",),
                exact_fbf_required=False,
            ),
            module.dataclasses.replace(
                boxed,
                solver_lane="exact",
                supported_solver_lanes=("exact",),
            ),
        ]

    with pytest.raises(ValueError, match="solver lanes are missing or out of order"):
        module._group_member_contract(
            group,
            schedules,
            output_root=tmp_path,
            ffprobe=Path("ffprobe"),
        )


def _write_group_member_artifacts(module, group, output_root, durations):
    probes = {}
    demo = output_root / "dart-demos"
    demo.write_bytes(b"demo")
    for member, panel_step, duration in zip(
        group.members, group.resolved_panel_steps, durations
    ):
        schedule = module.SCHEDULES[member]
        output_dir = output_root / member
        frame_path = module._frame_path(output_dir, panel_step)
        frame_path.parent.mkdir(parents=True, exist_ok=True)
        frame_path.write_bytes(f"frame:{member}".encode())
        timeline_path = output_dir / "timeline.json"
        timeline_path.write_text(f'{{"member":"{member}"}}', encoding="utf-8")
        clip_path = output_dir / "clip.mp4"
        clip_path.write_bytes(f"clip:{member}".encode())
        runtime = {
            "demo_path": str(demo),
            "demo_sha256": module._sha256(demo),
        }
        visual_resource_snapshot = module._visual_resource_snapshot(schedule)
        visual_resources = module._bind_visual_resource_snapshots(
            visual_resource_snapshot, visual_resource_snapshot
        )
        if visual_resources is not None:
            runtime["visual_resources"] = visual_resources
        metadata = {
            "schema_version": module.SCHEMA_VERSION,
            "kind": "capture_result",
            "schedule": module.schedule_plan(schedule, demo, output_root),
            "runtime": runtime,
            "timeline_validation": {
                "sidecar": str(timeline_path),
                "steps": {
                    str(panel_step): {"sim_time": schedule.time_at_step(panel_step)},
                },
                "frames": {
                    str(panel_step): {
                        "path": str(frame_path),
                        "sha256": module._sha256(frame_path),
                    }
                },
            },
            "media_validation": [
                {
                    "kind": "mp4",
                    "path": str(clip_path),
                    "sha256": module._sha256(clip_path),
                }
            ],
            "actual_simulator": True,
            "generated_imagery": False,
            "paper_comparable": False,
            "automated_semantic_outcome_validated": False,
            "semantic_outcome_gate": module.CAPTURE_SEMANTIC_OUTCOME_GATE,
            "known_mismatches": list(schedule.mismatches),
            "pass": True,
        }
        metadata_path = output_dir / "metadata.json"
        metadata_path.write_text(json.dumps(metadata), encoding="utf-8")
        expected_stream = module._expected_media_stream(schedule, "mp4")
        probes[clip_path] = {
            "format": {"duration": str(duration)},
            "streams": [
                {
                    "codec_name": "h264",
                    "pix_fmt": "yuv420p",
                    "width": schedule.crop[0],
                    "height": schedule.crop[1],
                    "r_frame_rate": "30/1",
                    "nb_frames": str(expected_stream["frame_count"]),
                }
            ],
        }
    return probes


def test_group_member_contract_enforces_source_order_and_synchronized_duration(
    tmp_path, monkeypatch
):
    module = _load_module()
    group = module.GROUP_OUTPUTS["turntable"]
    probes = _write_group_member_artifacts(
        module,
        group,
        tmp_path,
        [121 / 30, 121 / 30, 121 / 30, 121 / 30],
    )
    monkeypatch.setattr(module, "_probe_media", lambda path, _ffprobe: probes[path])
    schedules = [module.SCHEDULES[member] for member in group.members]

    contract = module._group_member_contract(
        group, schedules, output_root=tmp_path, ffprobe=Path("ffprobe")
    )

    assert [member["id"] for member in contract["members"]] == list(group.members)
    assert [member["label"] for member in contract["members"]] == list(group.labels)
    assert contract["duration_seconds"] == pytest.approx(121 / 30)
    with pytest.raises(ValueError, match="out of source order"):
        module._group_member_contract(
            group,
            list(reversed(schedules)),
            output_root=tmp_path,
            ffprobe=Path("ffprobe"),
        )

    probes[tmp_path / group.members[-1] / "clip.mp4"]["format"]["duration"] = "4.2"
    with pytest.raises(ValueError, match="duration .* differs from the schedule"):
        module._group_member_contract(
            group, schedules, output_root=tmp_path, ffprobe=Path("ffprobe")
        )


def test_author_group_contract_binds_outcome_aware_steps_and_times(
    tmp_path, monkeypatch
):
    module = _load_module()
    group = module.GROUP_OUTPUTS["turntable_author"]
    probes = _write_group_member_artifacts(
        module,
        group,
        tmp_path,
        [181 / 30, 181 / 30, 181 / 30, 181 / 30],
    )
    monkeypatch.setattr(module, "_probe_media", lambda path, _ffprobe: probes[path])
    schedules = [module.SCHEDULES[member] for member in group.members]

    contract = module._group_member_contract(
        group, schedules, output_root=tmp_path, ffprobe=Path("ffprobe")
    )

    assert contract["frame_count"] == 181
    assert len(contract["video_steps"]) == 181
    assert contract["output_fps"] == 30
    assert [member["panel_source_step"] for member in contract["members"]] == [
        136,
        120,
        360,
        90,
    ]
    assert [
        member["panel_source_time_seconds"] for member in contract["members"]
    ] == pytest.approx([136 / 60, 2.0, 6.0, 1.5])
    assert [
        Path(member["panel_source_frame"]).name for member in contract["members"]
    ] == [
        "step_000136.png",
        "step_000120.png",
        "step_000360.png",
        "step_000090.png",
    ]

    first_metadata_path = tmp_path / group.members[0] / "metadata.json"
    first_metadata = json.loads(first_metadata_path.read_text(encoding="utf-8"))
    first_metadata["timeline_validation"]["steps"]["136"]["sim_time"] = 2.0
    first_metadata_path.write_text(json.dumps(first_metadata), encoding="utf-8")
    with pytest.raises(ValueError, match="panel source simulation time changed"):
        module._group_member_contract(
            group, schedules, output_root=tmp_path, ffprobe=Path("ffprobe")
        )


def test_author_group_contract_rejects_uncaptured_panel_source_step(tmp_path):
    module = _load_module()
    group = module.dataclasses.replace(
        module.GROUP_OUTPUTS["turntable_author"],
        panel_steps=(135, 120, 360, 90),
    )
    schedules = [module.SCHEDULES[member] for member in group.members]

    with pytest.raises(ValueError, match="not a captured instant"):
        module._group_member_contract(
            group, schedules, output_root=tmp_path, ffprobe=Path("ffprobe")
        )


def test_group_member_contract_rejects_agreeing_stretched_frame_counts(
    tmp_path, monkeypatch
):
    module = _load_module()
    group = module.GROUP_OUTPUTS["turntable"]
    probes = _write_group_member_artifacts(
        module,
        group,
        tmp_path,
        [121 / 30, 121 / 30, 121 / 30, 121 / 30],
    )
    for probe in probes.values():
        probe["streams"][0]["nb_frames"] = "122"
        probe["format"]["duration"] = str(122 / 30)
    monkeypatch.setattr(module, "_probe_media", lambda path, _ffprobe: probes[path])

    with pytest.raises(ValueError, match="frame count 122 does not match 121"):
        module._group_member_contract(
            group,
            [module.SCHEDULES[member] for member in group.members],
            output_root=tmp_path,
            ffprobe=Path("ffprobe"),
        )


def test_group_member_contract_rejects_mixed_schedule_time_steps(tmp_path):
    module = _load_module()
    group = module.GROUP_OUTPUTS["turntable"]
    schedules = [module.SCHEDULES[member] for member in group.members]
    schedules[-1] = module.dataclasses.replace(
        schedules[-1], time_step_seconds=1.0 / 240.0
    )

    with pytest.raises(ValueError, match="member time-step mismatch"):
        module._group_member_contract(
            group,
            schedules,
            output_root=tmp_path,
            ffprobe=Path("ffprobe"),
        )


@pytest.mark.parametrize("group_id", ["turntable", "turntable_author", "painleve"])
def test_group_panel_uses_labeled_source_order_and_full_decode(
    tmp_path, monkeypatch, group_id
):
    module = _load_module()
    group = module.GROUP_OUTPUTS[group_id]
    schedules = [module.SCHEDULES[member] for member in group.members]
    output_root = tmp_path / "captures"
    output_dir = output_root / "groups" / group.id
    output_dir.mkdir(parents=True)
    for schedule, panel_step in zip(schedules, group.resolved_panel_steps):
        source = module._frame_path(output_root / schedule.id, panel_step)
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_bytes(f"source:{schedule.id}".encode())

    sys.path.insert(0, str(ROOT / "scripts"))
    from _image_tools import write_png

    real_run = module._run
    calls = []

    def write_test_image(path, width, height):
        pixels = bytearray(bytes((20, 40, 60)) * (width * height))
        pixels[0:3] = bytes((220, 110, 30))
        write_png(path, width, height, bytes(pixels))

    def fake_run(argv, **kwargs):
        argv = [str(value) for value in argv]
        calls.append(argv)
        if len(argv) > 1 and argv[1].endswith("image_compose.py"):
            return real_run(argv, **kwargs)
        if any("vstack=inputs=2[outv]" in value for value in argv):
            write_test_image(Path(argv[-1]), 1344, 1076)
        elif "-vf" in argv and any(value.startswith("crop=") for value in argv):
            write_test_image(Path(argv[-1]), 660, 506)
        return None

    monkeypatch.setattr(module, "_run", fake_run)

    report = module._compose_group_panel(
        group,
        schedules,
        output_root=output_root,
        output_dir=output_dir,
        ffmpeg=Path("ffmpeg"),
        python=Path(sys.executable),
    )

    assert report["member_order"] == list(group.members)
    assert report["labels"] == list(group.resolved_panel_labels)
    assert report["full_decode"] == "pass"
    if group.panel_steps is not None:
        assert "panel_step" not in report
        assert [source["member"] for source in report["source_frames"]] == list(
            group.members
        )
        assert [source["step"] for source in report["source_frames"]] == list(
            group.panel_steps
        )
        assert [source["time_seconds"] for source in report["source_frames"]] == [
            schedule.time_at_step(step)
            for schedule, step in zip(schedules, group.panel_steps)
        ]
    else:
        assert report["panel_step"] == group.panel_step
    flattened_labels = [
        label
        for row in report["row_compositions"]
        for label in row["manifest"]["labels"]
    ]
    assert flattened_labels == list(group.resolved_panel_labels)
    assert any(command[-2:] == ["null", module.os.devnull] for command in calls)
    if group.layout == "2x2":
        assert report["stack_command"] is not None
        assert report["verdict"]["image"]["width"] == 1344
        assert report["verdict"]["image"]["height"] == 1076
    else:
        assert report["stack_command"] is None
        assert report["verdict"]["image"]["width"] == 1344
        assert report["verdict"]["image"]["height"] == 538


@pytest.mark.parametrize(
    ("group_id", "expected_dimensions", "stack_token"),
    [
        ("turntable", (1320, 1060), "[top][bottom]vstack"),
        ("painleve", (1320, 530), "[v0][v1]hstack"),
    ],
)
def test_group_mp4_command_labels_every_member_and_is_fully_decoded(
    tmp_path, monkeypatch, group_id, expected_dimensions, stack_token
):
    module = _load_module()
    group = module.GROUP_OUTPUTS[group_id]
    members = []
    for member in group.members:
        clip = tmp_path / member / "clip.mp4"
        clip.parent.mkdir(parents=True)
        clip.write_bytes(member.encode())
        members.append({"id": member, "clip_path": str(clip)})
    contract = {
        "members": members,
        "tile_width": 660,
        "tile_height": 506,
        "duration_seconds": 4.0,
        "duration_tolerance_seconds": 0.05,
        "frame_count": 121,
        "frame_rate": "30/1",
    }
    output_dir = tmp_path / "groups" / group.id
    output_dir.mkdir(parents=True)
    calls = []

    def fake_run(argv, **_kwargs):
        argv = [str(value) for value in argv]
        calls.append(argv)
        if "libx264" in argv:
            Path(argv[-1]).write_bytes(b"group-mp4")
        return None

    def fake_probe(path, _ffprobe):
        assert path == output_dir / "clip.mp4"
        return {
            "format": {"duration": "4.0"},
            "streams": [
                {
                    "codec_name": "h264",
                    "pix_fmt": "yuv420p",
                    "width": expected_dimensions[0],
                    "height": expected_dimensions[1],
                    "r_frame_rate": "30/1",
                    "nb_frames": "121",
                }
            ],
        }

    monkeypatch.setattr(module, "_run", fake_run)
    monkeypatch.setattr(module, "_probe_media", fake_probe)

    report = module._encode_group_media(
        group,
        contract,
        output_dir=output_dir,
        ffmpeg=Path("ffmpeg"),
        ffprobe=Path("ffprobe"),
    )

    encode_command = report["command"]
    filter_graph = encode_command[encode_command.index("-filter_complex") + 1]
    assert stack_token in filter_graph
    assert all(label in filter_graph for label in group.labels)
    assert report["full_decode"] == "pass"
    assert calls[-1][-2:] == ["null", module.os.devnull]


def test_run_creates_group_only_after_all_members_succeed(
    tmp_path, monkeypatch, capsys
):
    module = _load_module()
    calls = []

    def fake_run_schedule(schedule, **_kwargs):
        return {"schedule": {"id": schedule.id}, "pass": True}

    def fake_run_group(group, schedules, **_kwargs):
        calls.append((group.id, tuple(schedule.id for schedule in schedules)))
        return {"group_id": group.id, "member_order": list(group.members), "pass": True}

    monkeypatch.setattr(module, "run_schedule", fake_run_schedule)
    monkeypatch.setattr(module, "run_group_output", fake_run_group)
    arguments = ["run", "--output-root", str(tmp_path)]
    for member in module.TURN_TABLE_MEMBERS:
        arguments.extend(("--scenario", member))

    assert module.main(arguments) == 0
    payload = json.loads(capsys.readouterr().out)
    assert calls == [("turntable", module.TURN_TABLE_MEMBERS)]
    assert payload["group_outputs"][0]["member_order"] == list(
        module.TURN_TABLE_MEMBERS
    )

    calls.clear()
    assert (
        module.main(
            [
                "run",
                "--output-root",
                str(tmp_path),
                "--scenario",
                module.TURN_TABLE_MEMBERS[0],
            ]
        )
        == 0
    )
    partial_payload = json.loads(capsys.readouterr().out)
    assert calls == []
    assert partial_payload["group_outputs"] == []

    stale_metadata = tmp_path / "groups" / "turntable" / "metadata.json"
    stale_metadata.parent.mkdir(parents=True)
    stale_metadata.write_text('{"pass":true}', encoding="utf-8")

    def fail_one_member(schedule, **_kwargs):
        if schedule.id == module.TURN_TABLE_MEMBERS[-1]:
            raise ValueError("member failed")
        return {"schedule": {"id": schedule.id}, "pass": True}

    monkeypatch.setattr(module, "run_schedule", fail_one_member)
    failed_arguments = ["run", "--keep-going", "--output-root", str(tmp_path)]
    for member in module.TURN_TABLE_MEMBERS:
        failed_arguments.extend(("--scenario", member))
    assert module.main(failed_arguments) == 1
    failed_payload = json.loads(capsys.readouterr().out)
    assert failed_payload["group_skips"][0]["failed_members"] == [
        module.TURN_TABLE_MEMBERS[-1]
    ]
    assert not stale_metadata.exists()


def test_run_both_orchestrates_solver_comparison_after_both_members(
    tmp_path, monkeypatch, capsys
):
    module = _load_module()
    schedule_calls = []
    group_calls = []

    def fake_run_schedule(schedule, **_kwargs):
        schedule_calls.append(schedule.id)
        return {"schedule": {"id": schedule.id}, "pass": True}

    def fake_run_group(group, schedules, **_kwargs):
        member_ids = tuple(schedule.id for schedule in schedules)
        group_calls.append((group.id, group.solver_lane, member_ids))
        return {"group_id": group.id, "member_order": list(group.members), "pass": True}

    monkeypatch.setattr(module, "run_schedule", fake_run_schedule)
    monkeypatch.setattr(module, "run_group_output", fake_run_group)

    assert (
        module.main(
            [
                "run",
                "--scenario",
                "backspin",
                "--solver-lane",
                "both",
                "--output-root",
                str(tmp_path),
            ]
        )
        == 0
    )
    payload = json.loads(capsys.readouterr().out)

    assert schedule_calls == ["backspin", "backspin__boxed"]
    assert group_calls == [
        (
            "backspin__exact_vs_boxed",
            "both",
            ("backspin", "backspin__boxed"),
        )
    ]
    assert payload["group_outputs"][0]["group_id"] == ("backspin__exact_vs_boxed")


def test_existing_verification_rejects_changed_requested_demo_hash(
    tmp_path, monkeypatch
):
    module = _load_module()
    schedule = _test_schedule(module)
    output_root = tmp_path / "captures"
    output_dir = output_root / schedule.id
    output_dir.mkdir(parents=True)
    demo = tmp_path / "dart-demos"
    demo.write_bytes(b"current-demo")
    panel = output_dir / "panel.png"
    panel.write_bytes(b"panel")
    clip = output_dir / "clip.mp4"
    clip.write_bytes(b"clip")
    manifest_path = output_dir / "panel.compose.json"
    manifest_path.write_text("{}", encoding="utf-8")
    expected_command = module.build_demo_command(schedule, demo, output_dir)
    metadata = {
        "schema_version": module.SCHEMA_VERSION,
        "kind": "capture_result",
        "schedule": module.schedule_plan(schedule, demo, output_root),
        "runtime": {
            "demo_path": str(demo),
            "demo_argv": expected_command,
            "demo_sha256": "stale-demo-hash",
        },
        "actual_simulator": True,
        "generated_imagery": False,
        "paper_comparable": False,
        "automated_semantic_outcome_validated": False,
        "semantic_outcome_gate": module.CAPTURE_SEMANTIC_OUTCOME_GATE,
        "known_mismatches": list(schedule.mismatches),
        "pass": True,
    }
    (output_dir / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")
    monkeypatch.setattr(
        module,
        "validate_sidecar",
        lambda *_args, **_kwargs: {"sidecar": "timeline.json", "frames": {}},
    )
    monkeypatch.setattr(
        module,
        "build_verdict",
        lambda _path: {"pass": True, "reasons": []},
    )
    monkeypatch.setattr(
        module,
        "_validate_media",
        lambda *_args, **_kwargs: [
            {
                "kind": "mp4",
                "path": str(clip),
                "sha256": module._sha256(clip),
            }
        ],
    )

    with pytest.raises(ValueError, match="requested demo hash changed"):
        module._verify_existing(
            schedule,
            demo=demo,
            output_root=output_root,
            ffmpeg=Path("ffmpeg"),
            ffprobe=Path("ffprobe"),
        )


def test_existing_verification_accepts_legacy_exact_artifact_without_flag(
    tmp_path, monkeypatch
):
    module = _load_module()
    schedule = _test_schedule(module, exact_required=True)
    output_root = tmp_path / "captures"
    output_dir = output_root / schedule.id
    demo = tmp_path / "dart-demos"
    demo.write_bytes(b"demo")
    demo = demo.resolve()
    _write_frames(module, schedule, output_dir)
    _write_sidecar(module, schedule, output_dir)
    legacy_command = module._legacy_demo_command(
        module.build_demo_command(schedule, demo, output_dir)
    )
    sidecar_path = output_dir / "timeline.json"
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
    sidecar["runtime_command"] = module._shell_command(legacy_command)
    del sidecar["headless_exact_fbf_fail_fast"]
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")
    timeline = module.validate_sidecar(
        schedule,
        output_dir,
        expected_demo=demo,
        allow_legacy_fail_fast=True,
    )
    assert timeline["headless_exact_fbf_fail_fast"]["legacy_artifact"] is True

    panel = output_dir / "panel.png"
    panel.write_bytes(b"panel")
    panel_sources = []
    for step in schedule.panel_steps:
        source = module._panel_frame_path(output_dir, step)
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_bytes(f"panel-source:{step}".encode())
        panel_sources.append({"path": str(source), "sha256": module._sha256(source)})
    compose_manifest_path = output_dir / "panel.compose.json"
    compose_manifest_path.write_text("{}", encoding="utf-8")
    plan = module.schedule_plan(schedule, demo, output_root)
    plan["demo_argv"] = legacy_command
    plan["demo_command"] = module._shell_command(legacy_command)
    metadata = {
        "schema_version": module.SCHEMA_VERSION,
        "kind": "capture_result",
        "schedule": plan,
        "runtime": {
            "demo_path": str(demo),
            "demo_argv": legacy_command,
            "demo_sha256": module._sha256(demo),
        },
        "timeline_validation": timeline,
        "panel_validation": {
            "path": str(panel),
            "sha256": module._sha256(panel),
            "source_frames": panel_sources,
            "compose_manifest_path": str(compose_manifest_path),
            "compose_manifest_sha256": module._sha256(compose_manifest_path),
            "compose_manifest": {},
        },
        "media_validation": [],
        "actual_simulator": True,
        "generated_imagery": False,
        "paper_comparable": False,
        "automated_semantic_outcome_validated": False,
        "semantic_outcome_gate": module.CAPTURE_SEMANTIC_OUTCOME_GATE,
        "known_mismatches": list(schedule.mismatches),
        "pass": True,
    }
    (output_dir / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")
    build_verdict = module.build_verdict
    monkeypatch.setattr(
        module,
        "build_verdict",
        lambda path: (
            {"pass": True, "reasons": []} if path == panel else build_verdict(path)
        ),
    )
    monkeypatch.setattr(module, "_validate_media", lambda *_args, **_kwargs: [])

    report = module._verify_existing(
        schedule,
        demo=demo,
        output_root=output_root,
        ffmpeg=Path("ffmpeg"),
        ffprobe=Path("ffprobe"),
    )

    assert report["pass"] is True


@pytest.mark.parametrize(
    ("scope", "key", "mutated"),
    [
        ("schedule", "source_segment", "different-segment"),
        ("schedule", "configuration", {"evidence_scope": "overclaimed"}),
        ("schedule", "total_steps", 999),
        ("schedule", "time_step_seconds", 1.0 / 240.0),
        ("schedule", "capture_steps", [0]),
        ("schedule", "panel_steps", [1]),
        ("schedule", "panel_labels", ["invented"]),
        ("schedule", "collision_detector", "native"),
        ("schedule", "collision_detector_override", False),
        ("schedule", "actual_simulator_required", False),
        ("schedule", "generated_imagery_allowed", True),
        ("schedule", "paper_comparable", True),
        ("schedule", "known_mismatches", []),
        ("schedule", "known_gate_blockers", ["invented blocker"]),
        ("schedule", "evidence_ready", False),
        ("capture", "actual_simulator", False),
        ("capture", "generated_imagery", True),
        ("capture", "paper_comparable", True),
        ("capture", "automated_semantic_outcome_validated", True),
        ("capture", "semantic_outcome_gate", "automatically validated"),
        ("capture", "known_mismatches", []),
    ],
)
def test_existing_verification_rejects_claim_boundary_mutations(
    tmp_path, monkeypatch, scope, key, mutated
):
    module = _load_module()
    schedule = _test_schedule(module)
    output_root = tmp_path / "captures"
    output_dir = output_root / schedule.id
    output_dir.mkdir(parents=True)
    demo = tmp_path / "dart-demos"
    demo.write_bytes(b"demo")
    (output_dir / "panel.png").write_bytes(b"panel")
    expected_command = module.build_demo_command(schedule, demo, output_dir)
    metadata = {
        "schema_version": module.SCHEMA_VERSION,
        "kind": "capture_result",
        "schedule": module.schedule_plan(schedule, demo, output_root),
        "runtime": {
            "demo_path": str(demo),
            "demo_argv": expected_command,
            "demo_sha256": module._sha256(demo),
        },
        "actual_simulator": True,
        "generated_imagery": False,
        "paper_comparable": False,
        "automated_semantic_outcome_validated": False,
        "semantic_outcome_gate": module.CAPTURE_SEMANTIC_OUTCOME_GATE,
        "known_mismatches": list(schedule.mismatches),
        "pass": True,
    }
    target = metadata["schedule"] if scope == "schedule" else metadata
    target[key] = mutated
    (output_dir / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")
    monkeypatch.setattr(
        module,
        "validate_sidecar",
        lambda *_args, **_kwargs: {"sidecar": "timeline.json", "frames": {}},
    )
    monkeypatch.setattr(
        module, "build_verdict", lambda _path: {"pass": True, "reasons": []}
    )
    monkeypatch.setattr(module, "_validate_media", lambda *_args, **_kwargs: [])

    with pytest.raises(ValueError, match=rf"claim boundary {key} changed"):
        module._verify_existing(
            schedule,
            demo=demo,
            output_root=output_root,
            ffmpeg=Path("ffmpeg"),
            ffprobe=Path("ffprobe"),
        )


@pytest.mark.parametrize(
    ("key", "mutated"),
    [
        ("source_segment", "different-segment"),
        ("solver_lane", "boxed"),
        ("source_group_id", "different-group"),
        ("actual_simulator", False),
        ("generated_imagery", True),
        ("paper_comparable", True),
        ("automated_semantic_outcome_validated", True),
        ("semantic_outcome_gate", "automatically validated"),
    ],
)
def test_existing_group_verification_rejects_claim_boundary_mutations(
    tmp_path, monkeypatch, key, mutated
):
    module = _load_module()
    group = module.GROUP_OUTPUTS["turntable_author"]
    metadata_path = tmp_path / "groups" / group.id / "metadata.json"
    metadata_path.parent.mkdir(parents=True)
    metadata = {
        "schema_version": module.SCHEMA_VERSION,
        "kind": "group_capture_result",
        "group_id": group.id,
        "source_segment": group.source_segment,
        "solver_lane": group.solver_lane,
        "source_group_id": group.source_group_id,
        "layout": group.layout,
        "member_order": list(group.members),
        "labels": list(group.labels),
        "actual_simulator": True,
        "generated_imagery": False,
        "paper_comparable": False,
        "automated_semantic_outcome_validated": False,
        "semantic_outcome_gate": module._group_semantic_outcome_gate(group),
        "pass": True,
    }
    metadata[key] = mutated
    metadata_path.write_text(json.dumps(metadata), encoding="utf-8")
    monkeypatch.setattr(module, "_group_member_contract", lambda *_args, **_kwargs: {})

    with pytest.raises(ValueError, match=rf"group claim boundary {key} changed"):
        module._verify_existing_group(
            group,
            [module.SCHEDULES[member] for member in group.members],
            output_root=tmp_path,
            ffmpeg=Path("ffmpeg"),
            ffprobe=Path("ffprobe"),
        )


def test_author_capture_binds_obj_and_mtl_before_and_after_demo(tmp_path, monkeypatch):
    module = _load_module()
    schedule = module.SCHEDULES[module.AUTHOR_TURN_TABLE_MEMBERS[0]]
    obj = tmp_path / "turntable.obj"
    mtl = tmp_path / "turntable.mtl"
    obj.write_bytes(b"obj-v1")
    mtl.write_bytes(b"mtl-v1")
    monkeypatch.setattr(
        module,
        "AUTHOR_TURN_TABLE_VISUAL_RESOURCES",
        (("turntable_disc_obj", obj), ("turntable_disc_mtl", mtl)),
    )
    before = module._visual_resource_snapshot(schedule)
    binding = module._bind_visual_resource_snapshots(before, before)
    assert set(binding) == {"turntable_disc_obj", "turntable_disc_mtl"}
    assert binding["turntable_disc_obj"]["path"] == str(obj.resolve())
    assert binding["turntable_disc_mtl"]["path"] == str(mtl.resolve())

    demo = tmp_path / "dart-demos"
    ffmpeg = tmp_path / "ffmpeg"
    ffprobe = tmp_path / "ffprobe"
    python = tmp_path / "python"
    for executable in (demo, ffmpeg, ffprobe, python):
        executable.write_bytes(b"executable")

    def mutate_during_capture(_argv, **_kwargs):
        mtl.write_bytes(b"mtl-v2")
        return module.subprocess.CompletedProcess([], 0)

    monkeypatch.setattr(module, "_run", mutate_during_capture)
    with pytest.raises(
        ValueError,
        match="turntable_disc_mtl changed during capture",
    ):
        module.run_schedule(
            schedule,
            demo=demo,
            output_root=tmp_path / "captures",
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
            python=python,
            allow_long=False,
        )


def test_author_reuse_verification_rejects_visual_resource_hash_drift(
    tmp_path, monkeypatch
):
    module = _load_module()
    schedule = module.SCHEDULES[module.AUTHOR_TURN_TABLE_MEMBERS[0]]
    obj = tmp_path / "turntable.obj"
    mtl = tmp_path / "turntable.mtl"
    obj.write_bytes(b"obj-v1")
    mtl.write_bytes(b"mtl-v1")
    monkeypatch.setattr(
        module,
        "AUTHOR_TURN_TABLE_VISUAL_RESOURCES",
        (("turntable_disc_obj", obj), ("turntable_disc_mtl", mtl)),
    )
    snapshot = module._visual_resource_snapshot(schedule)
    visual_resources = module._bind_visual_resource_snapshots(snapshot, snapshot)

    output_root = tmp_path / "captures"
    output_dir = output_root / schedule.id
    output_dir.mkdir(parents=True)
    demo = tmp_path / "dart-demos"
    demo.write_bytes(b"demo")
    (output_dir / "panel.png").write_bytes(b"panel")
    expected_command = module.build_demo_command(schedule, demo, output_dir)
    metadata = {
        "schema_version": module.SCHEMA_VERSION,
        "kind": "capture_result",
        "schedule": module.schedule_plan(schedule, demo, output_root),
        "runtime": {
            "demo_path": str(demo),
            "demo_argv": expected_command,
            "demo_sha256": module._sha256(demo),
            "visual_resources": visual_resources,
        },
        "actual_simulator": True,
        "generated_imagery": False,
        "paper_comparable": False,
        "automated_semantic_outcome_validated": False,
        "semantic_outcome_gate": module.CAPTURE_SEMANTIC_OUTCOME_GATE,
        "known_mismatches": list(schedule.mismatches),
        "pass": True,
    }
    (output_dir / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")
    monkeypatch.setattr(
        module,
        "validate_sidecar",
        lambda *_args, **_kwargs: {"sidecar": "timeline.json", "frames": {}},
    )
    monkeypatch.setattr(
        module, "build_verdict", lambda _path: {"pass": True, "reasons": []}
    )
    monkeypatch.setattr(module, "_validate_media", lambda *_args, **_kwargs: [])

    obj.write_bytes(b"obj-v2")
    with pytest.raises(ValueError, match="visual resource binding changed"):
        module._verify_existing(
            schedule,
            demo=demo,
            output_root=output_root,
            ffmpeg=Path("ffmpeg"),
            ffprobe=Path("ffprobe"),
        )


@pytest.mark.parametrize(
    ("tamper", "message"),
    [("panel", "panel hash changed"), ("media", "mp4 hash binding changed")],
)
def test_existing_verification_binds_panel_and_media_to_fresh_metadata(
    tmp_path, monkeypatch, tamper, message
):
    module = _load_module()
    schedule = _test_schedule(module)
    output_root = tmp_path / "captures"
    output_dir = output_root / schedule.id
    output_dir.mkdir(parents=True)
    demo = tmp_path / "dart-demos"
    demo.write_bytes(b"current-demo")
    panel = output_dir / "panel.png"
    panel.write_bytes(b"panel")
    clip = output_dir / "clip.mp4"
    clip.write_bytes(b"clip")
    panel_sources = []
    for step in schedule.panel_steps:
        source = module._panel_frame_path(output_dir, step)
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_bytes(f"panel-source:{step}".encode())
        panel_sources.append({"path": str(source), "sha256": module._sha256(source)})
    manifest_path = output_dir / "panel.compose.json"
    manifest_path.write_text('{"pass":true}', encoding="utf-8")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    expected_command = module.build_demo_command(schedule, demo, output_dir)
    metadata = {
        "schema_version": module.SCHEMA_VERSION,
        "kind": "capture_result",
        "schedule": module.schedule_plan(schedule, demo, output_root),
        "runtime": {
            "demo_path": str(demo),
            "demo_argv": expected_command,
            "demo_sha256": module._sha256(demo),
        },
        "timeline_validation": {"sidecar": "timeline.json", "frames": {}},
        "panel_validation": {
            "path": str(panel),
            "sha256": module._sha256(panel),
            "source_frames": panel_sources,
            "compose_manifest_path": str(manifest_path),
            "compose_manifest_sha256": module._sha256(manifest_path),
            "compose_manifest": manifest,
        },
        "media_validation": [
            {"kind": "mp4", "path": str(clip), "sha256": module._sha256(clip)}
        ],
        "actual_simulator": True,
        "generated_imagery": False,
        "paper_comparable": False,
        "automated_semantic_outcome_validated": False,
        "semantic_outcome_gate": module.CAPTURE_SEMANTIC_OUTCOME_GATE,
        "known_mismatches": list(schedule.mismatches),
        "pass": True,
    }
    (output_dir / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")
    monkeypatch.setattr(
        module,
        "validate_sidecar",
        lambda *_args, **_kwargs: {"sidecar": "timeline.json", "frames": {}},
    )
    monkeypatch.setattr(
        module,
        "build_verdict",
        lambda _path: {"pass": True, "reasons": []},
    )

    def validate_media(*_args, **_kwargs):
        return [
            {
                "kind": "mp4",
                "path": str(clip),
                "sha256": module._sha256(clip),
            }
        ]

    monkeypatch.setattr(module, "_validate_media", validate_media)
    report = module._verify_existing(
        schedule,
        demo=demo,
        output_root=output_root,
        ffmpeg=Path("ffmpeg"),
        ffprobe=Path("ffprobe"),
    )
    assert report["pass"] is True

    target = panel if tamper == "panel" else clip
    target.write_bytes(target.read_bytes() + b"tamper")
    with pytest.raises(ValueError, match=message):
        module._verify_existing(
            schedule,
            demo=demo,
            output_root=output_root,
            ffmpeg=Path("ffmpeg"),
            ffprobe=Path("ffprobe"),
        )


def test_dry_run_does_not_create_output(tmp_path, capsys):
    module = _load_module()
    output_root = tmp_path / "must_not_exist"

    result = module.main(
        [
            "run",
            "--dry-run",
            "--scenario",
            "card_house_26",
            "--output-root",
            str(output_root),
        ]
    )

    assert result == 0
    assert not output_root.exists()
    payload = json.loads(capsys.readouterr().out)
    assert payload["schedules"][0]["long_run"] is True


def test_sidecar_validation_decodes_nonblank_ordered_frames(tmp_path):
    module = _load_module()
    schedule = _test_schedule(module)
    output_dir = tmp_path / schedule.id
    _write_frames(module, schedule, output_dir)
    _write_sidecar(module, schedule, output_dir)

    report = module.validate_sidecar(schedule, output_dir)

    assert report["pass"] is True
    assert report["shot_count"] == 3
    assert report["unique_frame_hashes"] == 3
    assert all(frame["non_blank"]["pass"] for frame in report["frames"].values())


def test_sidecar_validation_uses_the_explicit_schedule_time_step(tmp_path):
    module = _load_module()
    schedule = module.dataclasses.replace(
        _test_schedule(module), time_step_seconds=1.0 / 240.0
    )
    output_dir = tmp_path / schedule.id
    _write_frames(module, schedule, output_dir)
    _write_sidecar(module, schedule, output_dir)

    report = module.validate_sidecar(schedule, output_dir)

    assert report["steps"]["1"]["sim_time"] == pytest.approx(1.0 / 240.0)
    assert report["steps"]["2"]["sim_time"] == pytest.approx(2.0 / 240.0)

    sidecar_path = output_dir / "timeline.json"
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
    sidecar["steps"][1]["sim_time"] = 1.0 / 60.0
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")
    with pytest.raises(ValueError, match="completed-step time mismatch at step 1"):
        module.validate_sidecar(schedule, output_dir)


@pytest.mark.parametrize(
    "time_step_seconds",
    [0.0, -1.0 / 60.0, float("nan"), float("inf"), True],
)
def test_capture_schedule_rejects_invalid_explicit_time_step(time_step_seconds):
    module = _load_module()

    with pytest.raises(
        ValueError, match="time_step_seconds must be finite and positive"
    ):
        module.dataclasses.replace(
            _test_schedule(module), time_step_seconds=time_step_seconds
        )


def test_sidecar_validation_uses_scheduled_collision_frontend(tmp_path):
    module = _load_module()
    schedule = module.dataclasses.replace(
        _test_schedule(module), collision_detector="native"
    )
    output_dir = tmp_path / schedule.id
    _write_frames(module, schedule, output_dir)
    _write_sidecar(module, schedule, output_dir)

    report = module.validate_sidecar(schedule, output_dir)
    assert report["pass"] is True

    sidecar_path = output_dir / "timeline.json"
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
    sidecar["collision_detector"] = "dart"
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")

    with pytest.raises(ValueError, match="collision detector differs"):
        module.validate_sidecar(schedule, output_dir)


def test_exact_sidecar_allows_unavailable_pre_solve_step_zero(tmp_path):
    module = _load_module()
    schedule = _test_schedule(module, exact_required=True)
    output_dir = tmp_path / schedule.id
    _write_frames(module, schedule, output_dir)
    _write_sidecar(module, schedule, output_dir)

    report = module.validate_sidecar(schedule, output_dir)

    assert report["pass"] is True
    assert report["final_solver_diagnostics"]["available"] is True


def test_exact_sidecar_rejects_unavailable_post_solve_diagnostics(tmp_path):
    module = _load_module()
    schedule = _test_schedule(module, exact_required=True)
    output_dir = tmp_path / schedule.id
    _write_frames(module, schedule, output_dir)
    _write_sidecar(module, schedule, output_dir)
    sidecar_path = output_dir / "timeline.json"
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
    unavailable = {
        "solver": module.EXACT_SOLVER_NAME,
        "available": False,
        "gap": "unexpected post-solve gap",
    }
    sidecar["steps"][1]["solver_diagnostics"] = unavailable
    sidecar["shots"][1]["solver_diagnostics"] = unavailable
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")

    with pytest.raises(ValueError, match="step 1.*unavailable"):
        module.validate_sidecar(schedule, output_dir)


def test_exact_sidecar_rejects_contact_free_noop_mislabeled_as_evidence(tmp_path):
    module = _load_module()
    schedule = _test_schedule(module, exact_required=True)
    output_dir = tmp_path / schedule.id
    _write_frames(module, schedule, output_dir)
    _write_sidecar(module, schedule, output_dir)
    sidecar_path = output_dir / "timeline.json"
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
    never_run = {
        "solver": module.EXACT_SOLVER_NAME,
        "available": True,
        "status": "not_run",
        "fbf_status": "not_run",
        "residual": None,
        "worst_residual": None,
        "exact_attempts": 0,
        "exact_solves": 0,
        "accepted_at_cap": 0,
        "contacts": 0,
        "exact_failures": 0,
        "boxed_lcp_fallbacks": 0,
    }
    for item in sidecar["steps"]:
        item["solver_diagnostics"] = dict(never_run)
    for shot in sidecar["shots"]:
        shot["solver_diagnostics"] = dict(never_run)
    sidecar["solver_diagnostics"] = dict(never_run)
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")

    with pytest.raises(ValueError, match="never attempted a contact group"):
        module.validate_sidecar(schedule, output_dir)


def test_exact_sidecar_allows_airborne_not_run_prefix(tmp_path):
    module = _load_module()
    schedule = _test_schedule(module, exact_required=True)
    output_dir = tmp_path / schedule.id
    _write_frames(module, schedule, output_dir)
    _write_sidecar(module, schedule, output_dir)
    sidecar_path = output_dir / "timeline.json"
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
    airborne = {
        "solver": module.EXACT_SOLVER_NAME,
        "available": True,
        "status": "not_run",
        "fbf_status": "not_run",
        "residual": None,
        "worst_residual": None,
        "exact_attempts": 0,
        "exact_solves": 0,
        "accepted_at_cap": 0,
        "contacts": 0,
        "exact_failures": 0,
        "boxed_lcp_fallbacks": 0,
    }
    sidecar["steps"][1]["solver_diagnostics"] = dict(airborne)
    sidecar["shots"][1]["solver_diagnostics"] = dict(airborne)
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")

    report = module.validate_sidecar(schedule, output_dir)

    assert report["pass"] is True


def test_exact_sidecar_rejects_stale_success_on_later_contact_frame(tmp_path):
    module = _load_module()
    schedule = _test_schedule(module, exact_required=True)
    output_dir = tmp_path / schedule.id
    _write_frames(module, schedule, output_dir)
    _write_sidecar(module, schedule, output_dir)
    sidecar_path = output_dir / "timeline.json"
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
    sidecar["steps"][2]["solver_diagnostics"]["exact_attempts"] = 1
    sidecar["steps"][2]["solver_diagnostics"]["exact_solves"] = 1
    sidecar["shots"][2]["solver_diagnostics"] = dict(
        sidecar["steps"][2]["solver_diagnostics"]
    )
    sidecar["solver_diagnostics"] = dict(sidecar["steps"][2]["solver_diagnostics"])
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")

    with pytest.raises(ValueError, match="did not advance exact attempt/solve"):
        module.validate_sidecar(schedule, output_dir)


def test_exact_sidecar_rejects_unsampled_contact_step_without_new_attempt(tmp_path):
    module = _load_module()
    schedule = module.dataclasses.replace(
        _test_schedule(module, exact_required=True), frame_stride=2
    )
    output_dir = tmp_path / schedule.id
    _write_frames(module, schedule, output_dir)
    _write_sidecar(module, schedule, output_dir)
    sidecar_path = output_dir / "timeline.json"
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))

    # Step 1 is not captured. Reuse its cumulative counters at step 2 to
    # reproduce the old sampled-shot-only validator's false acceptance.
    stale = sidecar["steps"][2]["solver_diagnostics"]
    stale["exact_attempts"] = 1
    stale["exact_solves"] = 1
    sidecar["shots"][-1]["solver_diagnostics"] = dict(stale)
    sidecar["solver_diagnostics"] = dict(stale)
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")

    with pytest.raises(
        ValueError,
        match="trajectory step 2.*did not advance exact attempt/solve",
    ):
        module.validate_sidecar(schedule, output_dir)


def test_exact_sidecar_accepts_multiple_exact_groups_per_contact_step(tmp_path):
    module = _load_module()
    schedule = _test_schedule(module, exact_required=True)
    output_dir = tmp_path / schedule.id
    _write_frames(module, schedule, output_dir)
    _write_sidecar(module, schedule, output_dir)
    sidecar_path = output_dir / "timeline.json"
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))

    cumulative_groups = {1: 2, 2: 5}
    for step, count in cumulative_groups.items():
        diagnostics = sidecar["steps"][step]["solver_diagnostics"]
        diagnostics.update(
            {
                "contacts": count,
                "exact_attempts": count,
                "exact_solves": count,
            }
        )
        matching_shot = next(shot for shot in sidecar["shots"] if shot["step"] == step)
        matching_shot["solver_diagnostics"] = dict(diagnostics)
    sidecar["solver_diagnostics"] = dict(sidecar["steps"][-1]["solver_diagnostics"])
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")

    report = module.validate_sidecar(schedule, output_dir)

    assert report["final_solver_diagnostics"]["exact_attempts"] == 5
    assert report["pass"] is True


@pytest.mark.parametrize(
    ("updates", "message"),
    [
        ({"status": "max_iterations_accepted"}, "solver status"),
        ({"fbf_status": "max_iterations"}, "FBF status"),
        ({"residual": 1.1e-6}, "exceeds 1e-6"),
        ({"residual": None}, "unavailable/non-finite"),
    ],
)
def test_exact_sidecar_rejects_nonconverged_diagnostics(tmp_path, updates, message):
    module = _load_module()
    schedule = _test_schedule(module, exact_required=True)
    output_dir = tmp_path / schedule.id
    _write_frames(module, schedule, output_dir)
    _write_sidecar(module, schedule, output_dir)
    sidecar_path = output_dir / "timeline.json"
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
    sidecar["steps"][-1]["solver_diagnostics"].update(updates)
    sidecar["shots"][-1]["solver_diagnostics"].update(updates)
    sidecar["solver_diagnostics"].update(updates)
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")

    with pytest.raises(ValueError, match=message):
        module.validate_sidecar(schedule, output_dir)


def test_exact_sidecar_rejects_bad_unsampled_completed_step(tmp_path):
    module = _load_module()
    schedule = module.dataclasses.replace(
        _test_schedule(module, exact_required=True), frame_stride=2
    )
    output_dir = tmp_path / schedule.id
    _write_frames(module, schedule, output_dir)
    _write_sidecar(module, schedule, output_dir)
    sidecar_path = output_dir / "timeline.json"
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
    diagnostics = sidecar["steps"][1]["solver_diagnostics"]
    diagnostics.update(
        {
            "status": "max_iterations_accepted",
            "fbf_status": "max_iterations",
            "residual": 2e-5,
            "worst_residual": 2e-5,
            "accepted_at_cap": 1,
        }
    )
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")

    with pytest.raises(ValueError, match="trajectory step 1.*accepted_at_cap=1"):
        module.validate_sidecar(schedule, output_dir)


def test_exact_sidecar_rejects_nonlast_group_high_residual(tmp_path):
    module = _load_module()
    schedule = _test_schedule(module, exact_required=True)
    output_dir = tmp_path / schedule.id
    _write_frames(module, schedule, output_dir)
    _write_sidecar(module, schedule, output_dir)
    sidecar_path = output_dir / "timeline.json"
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
    for step in (1, 2):
        diagnostics = sidecar["steps"][step]["solver_diagnostics"]
        diagnostics["worst_residual"] = 8e-5
        matching_shot = next(shot for shot in sidecar["shots"] if shot["step"] == step)
        matching_shot["solver_diagnostics"]["worst_residual"] = 8e-5
    sidecar["solver_diagnostics"]["worst_residual"] = 8e-5
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")

    with pytest.raises(ValueError, match="trajectory worst residual.*exceeds"):
        module.validate_sidecar(schedule, output_dir)


def test_sidecar_validation_rejects_action_before_same_step_shot(tmp_path):
    module = _load_module()
    schedule = _test_schedule(module, action=True)
    output_dir = tmp_path / schedule.id
    _write_frames(module, schedule, output_dir)
    _write_sidecar(module, schedule, output_dir, action_before_shot=True)

    with pytest.raises(ValueError, match="action precedes"):
        module.validate_sidecar(schedule, output_dir)


def test_sidecar_validation_rejects_failed_action(tmp_path):
    module = _load_module()
    schedule = _test_schedule(module, action=True)
    output_dir = tmp_path / schedule.id
    _write_frames(module, schedule, output_dir)
    _write_sidecar(module, schedule, output_dir)
    sidecar_path = output_dir / "timeline.json"
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
    sidecar["actions"][0]["success"] = False
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")

    with pytest.raises(ValueError, match="scheduled action failed"):
        module.validate_sidecar(schedule, output_dir)


def test_sidecar_validation_rejects_uniform_frame(tmp_path):
    module = _load_module()
    schedule = _test_schedule(module)
    output_dir = tmp_path / schedule.id
    _write_frames(module, schedule, output_dir)
    sys.path.insert(0, str(ROOT / "scripts"))
    from _image_tools import write_png

    write_png(
        module._frame_path(output_dir, 1), 640, 480, bytes((20, 20, 20)) * (640 * 480)
    )
    _write_sidecar(module, schedule, output_dir)

    with pytest.raises(ValueError, match="blank/uniform"):
        module.validate_sidecar(schedule, output_dir)


def test_ffconcat_preserves_simulation_timing(tmp_path):
    module = _load_module()
    schedule = module.SCHEDULES["incline"]
    output_dir = tmp_path / "incline"
    output_dir.mkdir()

    concat = module._write_ffconcat(schedule, output_dir)
    text = concat.read_text(encoding="utf-8")

    assert "file 'frames/step_000000.png'" in text
    assert "file 'frames/step_000120.png'" in text
    assert "duration 0.033333333333333333" in text
    assert text.count("file 'frames/step_000120.png'") == 1


def test_ffconcat_uses_the_explicit_schedule_time_step(tmp_path):
    module = _load_module()
    schedule = module.dataclasses.replace(
        module.SCHEDULES["incline"], time_step_seconds=1.0 / 240.0
    )
    output_dir = tmp_path / "incline_240hz"
    output_dir.mkdir()

    concat = module._write_ffconcat(schedule, output_dir)
    text = concat.read_text(encoding="utf-8")

    assert "duration 0.0083333333333333332" in text
    assert "duration 0.033333333333333333" not in text


def test_expected_media_contract_pins_dimensions_rate_and_frame_count():
    module = _load_module()

    mp4 = module._expected_media_stream(module.SCHEDULES["backspin"], "mp4")
    gif = module._expected_media_stream(module.SCHEDULES["backspin"], "gif")

    assert (mp4["width"], mp4["height"]) == (1300, 506)
    assert mp4["frame_rate_rational"] == "30/1"
    assert mp4["frame_count"] == 66
    assert (gif["width"], gif["height"]) == (960, 374)
    assert gif["frame_rate_rational"] == "15/1"
    assert gif["frame_count"] == 33

    literal = module.SCHEDULES["masonry_arch_25_literal_standing"]
    assert module._expected_media_stream(literal, "mp4")["frame_count"] == 301
    assert module._expected_media_stream(literal, "gif")["frame_count"] == 151


def test_media_encoder_caps_endpoint_inclusive_ffmpeg_streams(tmp_path, monkeypatch):
    module = _load_module()
    schedule = module.dataclasses.replace(
        module.SCHEDULES["masonry_arch_25_literal_standing"], encode_gif=True
    )
    output_dir = tmp_path / schedule.id
    output_dir.mkdir()
    calls = []
    monkeypatch.setattr(
        module,
        "_run",
        lambda command, **_kwargs: calls.append([str(item) for item in command]),
    )

    outputs = module._encode_media(schedule, output_dir, Path("ffmpeg"))

    assert [output["kind"] for output in outputs] == ["mp4", "gif"]
    assert len(calls) == 2
    mp4_cap = calls[0].index("-frames:v")
    gif_cap = calls[1].index("-frames:v")
    assert calls[0][mp4_cap + 1] == "301"
    assert calls[1][gif_cap + 1] == "151"


def test_expected_media_contract_uses_the_explicit_schedule_time_step():
    module = _load_module()
    schedule = module.dataclasses.replace(
        module.SCHEDULES["backspin"], time_step_seconds=1.0 / 240.0
    )

    mp4 = module._expected_media_stream(schedule, "mp4")
    gif = module._expected_media_stream(schedule, "gif")

    assert mp4["frame_count"] == 17
    assert gif["frame_count"] == 9


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("width", 22, "dimensions"),
        ("r_frame_rate", "25/1", "frame rate"),
        ("nb_frames", "3", "frame count"),
        ("codec_name", "mpeg4", "codec"),
        ("pix_fmt", "yuv444p", "pixel format"),
    ],
)
def test_media_validation_rejects_wrong_stream_contract(
    tmp_path, monkeypatch, field, value, message
):
    module = _load_module()
    schedule = _test_schedule(module)
    path = tmp_path / "clip.mp4"
    path.write_bytes(b"media")
    expected = module._expected_media_stream(schedule, "mp4")
    stream = {
        "codec_name": "h264",
        "pix_fmt": "yuv420p",
        "width": expected["width"],
        "height": expected["height"],
        "r_frame_rate": expected["frame_rate_rational"],
        "nb_frames": str(expected["frame_count"]),
    }
    stream[field] = value
    monkeypatch.setattr(
        module,
        "_probe_media",
        lambda _path, _ffprobe: {
            "format": {
                "duration": str(float(expected["frame_count"] / expected["frame_rate"]))
            },
            "streams": [stream],
        },
    )
    monkeypatch.setattr(module, "_run", lambda *_args, **_kwargs: None)

    with pytest.raises(ValueError, match=message):
        module._validate_media(
            schedule,
            [{"kind": "mp4", "path": path, "command": []}],
            Path("ffmpeg"),
            Path("ffprobe"),
        )


def test_media_probe_requests_codec_and_pixel_format(monkeypatch):
    module = _load_module()
    calls = []

    def fake_run(argv, **_kwargs):
        calls.append([str(value) for value in argv])
        return module.subprocess.CompletedProcess(
            argv, returncode=0, stdout='{"format": {}, "streams": []}', stderr=""
        )

    monkeypatch.setattr(module, "_run", fake_run)

    assert module._probe_media(Path("clip.mp4"), Path("ffprobe")) == {
        "format": {},
        "streams": [],
    }
    show_entries = calls[0][calls[0].index("-show_entries") + 1]
    assert "codec_name" in show_entries
    assert "pix_fmt" in show_entries


def test_media_validation_records_h264_yuv420p_probe_without_schema_drift(
    tmp_path, monkeypatch
):
    module = _load_module()
    schedule = _test_schedule(module)
    path = tmp_path / "clip.mp4"
    path.write_bytes(b"media")
    expected = module._expected_media_stream(schedule, "mp4")
    monkeypatch.setattr(
        module,
        "_probe_media",
        lambda _path, _ffprobe: {
            "format": {
                "duration": str(float(expected["frame_count"] / expected["frame_rate"]))
            },
            "streams": [
                {
                    "codec_name": "h264",
                    "pix_fmt": "yuv420p",
                    "width": expected["width"],
                    "height": expected["height"],
                    "r_frame_rate": expected["frame_rate_rational"],
                    "nb_frames": str(expected["frame_count"]),
                }
            ],
        },
    )
    monkeypatch.setattr(module, "_run", lambda *_args, **_kwargs: None)

    reports = module._validate_media(
        schedule,
        [{"kind": "mp4", "path": path, "command": []}],
        Path("ffmpeg"),
        Path("ffprobe"),
    )

    assert set(reports[0]["stream_contract"]) == {
        "width",
        "height",
        "frame_rate",
        "frame_rate_rational",
        "frame_count",
    }
    assert reports[0]["probe"]["streams"][0]["codec_name"] == "h264"
    assert reports[0]["probe"]["streams"][0]["pix_fmt"] == "yuv420p"
    assert reports[0]["full_decode"] == "pass"


def test_coverage_audit_checks_visual_schedules_and_source_segments():
    module = _load_module()

    report = module.audit_coverage_contract()

    assert report["pass"] is True
    assert report["capture_path_complete"] is True
    assert report["evidence_complete"] is False
    assert report["video_segments"]["video.04_turntable"] == (35, 50)
    assert "card_house_26" in report["known_gate_blocked_schedules"]
    assert "masonry_arch_25" in report["known_gate_blocked_schedules"]
    assert (
        "masonry_arch_25_author_crown_impact_current_source"
        in report["known_gate_blocked_schedules"]
    )
    assert "masonry_arch_101" in report["known_gate_blocked_schedules"]
    assert report["required_schedule_count"] == 13


def test_audited_local_source_hashes_are_pinned():
    module = _load_module()

    assert module.AUDITED_VIDEO_SHA256 == (
        "d5356e1b31487be62b75af05efbfecdb70ad5d98501a8efd378fcedf066e4794"
    )
    assert module.AUDITED_TEASER_SHA256 == (
        "99527da7a84f7b9ac0031f794d9b16adadfba846d2165e7da22fd51d986c8db0"
    )
