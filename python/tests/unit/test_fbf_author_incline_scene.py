"""Focused contract tests for the current-source seven-cell incline scene."""

from __future__ import annotations

import copy
import importlib.util
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "run_fbf_visual_evidence.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "run_fbf_visual_evidence_author_incline_test", SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _contract(module, *, solver_lane="exact"):
    sine = math.sin(module.AUTHOR_INCLINE_THETA_RAD)
    cosine = math.cos(module.AUTHOR_INCLINE_THETA_RAD)
    rotation = [
        [cosine, 0.0, sine],
        [0.0, 1.0, 0.0],
        [-sine, 0.0, cosine],
    ]
    moment = [
        [1000.0 / 6.0, 0.0, 0.0],
        [0.0, 1000.0 / 6.0, 0.0],
        [0.0, 0.0, 1000.0 / 6.0],
    ]
    cells = []
    for index, (prefix, mu) in enumerate(
        zip(module.AUTHOR_INCLINE_STATE_PREFIXES, module.AUTHOR_INCLINE_MU_CELLS)
    ):
        lane_y = (index - 3) * module.AUTHOR_INCLINE_LANE_SPACING_M
        cells.append(
            {
                "index": index,
                "mu": mu,
                "state_prefix": prefix,
                "lane_translation_y_m": lane_y,
                "plane": {
                    "skeleton_name": f"author_incline_plane_{prefix}",
                    "body_name": f"author_incline_plane_{prefix}_body",
                    "mobile": False,
                    "size_m": [10.0, 3.0, 0.1],
                    "initial_pose": {
                        "translation": [-sine * 0.05, lane_y, -cosine * 0.05],
                        "rotation": rotation,
                    },
                    "friction": mu,
                },
                "cube": {
                    "skeleton_name": f"author_incline_cube_{prefix}",
                    "body_name": f"author_incline_cube_{prefix}_body",
                    "mobile": True,
                    "size_m": [1.0, 1.0, 1.0],
                    "initial_pose": {
                        "translation": [
                            sine
                            * module.AUTHOR_INCLINE_INITIAL_CENTER_NORMAL_DISTANCE_M,
                            lane_y,
                            cosine
                            * module.AUTHOR_INCLINE_INITIAL_CENTER_NORMAL_DISTANCE_M,
                        ],
                        "rotation": rotation,
                    },
                    "initial_linear_velocity_m_s": [0.0, 0.0, 0.0],
                    "initial_angular_velocity_rad_s": [0.0, 0.0, 0.0],
                    "friction": mu,
                    "mass_kg": 1000.0,
                    "moment_kg_m2": moment,
                },
            }
        )

    exact = solver_lane == "exact"
    return {
        "schema_version": module.AUTHOR_INCLINE_CONTRACT_SCHEMA_VERSION,
        "kind": "current_source_configuration_dynamics_adapter",
        "source_binding": {
            "repository": "https://github.com/matthcsong/fbf-sca-2026",
            "commit": "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0",
            "tree": "ffcdafb61adeda2239c8366d054b548b50d26685",
            "runner_path": "paper_examples/cube-on-incline/run.py",
            "runner_blob": "63cfc28dca1f6c65ce4a27dbfa239cba154580c6",
            "runner_sha256": (
                "881d486f25d85f9ae197bf4164e110b6bc39775c498433f27ec4407ca09ebf82"
            ),
            "configuration_spec_sha256": module._sha256(
                ROOT / "examples/demos/scenes/FbfAuthorInclineSpec.hpp"
            ),
            "exact_solver_options_sha256": module._sha256(
                ROOT / "dart/constraint/ExactCoulombFbfConstraintSolver.hpp"
            ),
            "demo_implementation_sha256": module._sha256(
                ROOT / "examples/demos/scenes/FbfPaperFrictionScene.cpp"
            ),
        },
        "source_configuration": {
            "mu_values": list(module.AUTHOR_INCLINE_MU_CELLS),
            "mu_grid_provenance": {
                "kind": "operator_selected_figure_1_grid",
                "source_default_mu": 0.5,
                "selection_interface": "supported_--mu_cli",
                "source_runs_each_mu_independently": True,
            },
            "tan_theta": 0.5,
            "theta_rad": module.AUTHOR_INCLINE_THETA_RAD,
            "gravity_m_s2": 9.81,
            "plane_half_extents_m": [5.0, 1.5, 0.05],
            "plane_full_dimensions_m": [10.0, 3.0, 0.1],
            "cube_half_extent_m": 0.5,
            "cube_full_dimensions_m": [1.0, 1.0, 1.0],
            "cube_density_kg_m3": 1000.0,
            "cube_mass_kg": 1000.0,
            "initial_geometric_separation_m": 0.001,
            "initial_center_normal_distance_m": 0.501,
            "source_cube_shape_gap_m": 0.01,
            "time_step_seconds": 1.0 / 60.0,
            "duration_seconds": 2.0,
            "total_steps": 120,
        },
        "dart_adapter": {
            "scene_id": "fbf_author_incline_sweep_current_source",
            "world": {
                "time_step_seconds": 1.0 / 60.0,
                "gravity_m_s2": [0.0, 0.0, -9.81],
                "simulation_threads": 1,
                "deactivation_enabled": False,
            },
            "layout": {
                "kind": "simultaneous_y_translated_lanes",
                "lane_spacing_m": 3.4,
                "source_cells_run_independently": True,
                "translation_axis": "y",
                "geometry_otherwise_unchanged": True,
            },
            "collision": {
                "detector": "native",
                "contact_manifold": "four_point_planar",
                "max_contacts": 28,
                "max_contacts_per_pair": 4,
            },
            "solver": {
                "lane": "exact_fbf" if exact else "boxed_lcp",
                "configuration_policy": "existing_small_fixture_exact_fbf_adapter",
                "split_impulse_enabled": False,
                "exact_options": (
                    {
                        "max_outer_iterations": 500,
                        "tolerance": 1e-6,
                        "inner_max_sweeps": 120,
                        "inner_local_iterations": 32,
                        "step_size_scale": 2.0,
                        "fallback_to_boxed_lcp_enabled": False,
                    }
                    if exact
                    else None
                ),
            },
            "cells": cells,
        },
        "adapter_boundaries": {
            "source_initial_geometric_separation_represented": True,
            "source_shape_gap_semantics_implemented": False,
            "source_collision_backend_implemented": False,
            "source_solver_backend_semantics_implemented": False,
            "source_float32_semantics_implemented": False,
            "dart_native_four_point_planar_is_adapter_choice": True,
            "seven_cells_run_simultaneously_in_source": False,
        },
        "claim_boundary": {
            "current_source_geometry_clock_mu_grid_port": True,
            "historical_paper_invocation_known": False,
            "trajectory_valid": False,
            "physical_outcome_valid": False,
            "trajectory_equivalence": False,
            "solver_equivalence": False,
            "physical_outcome_equivalence": False,
            "video_parity": False,
            "timing_comparability": False,
            "paper_parity": False,
        },
    }


def _hermite_terminal_state(final_value, final_velocity, time_seconds):
    duration = 2.0
    phase = time_seconds / duration
    value = (-2.0 * phase**3 + 3.0 * phase**2) * final_value + (
        phase**3 - phase**2
    ) * duration * final_velocity
    velocity = (
        (-6.0 * phase**2 + 6.0 * phase) * final_value
        + (3.0 * phase**2 - 2.0 * phase) * duration * final_velocity
    ) / duration
    return value, velocity


def _oracle_trace(module, *, terminal_state=None):
    sine = math.sin(module.AUTHOR_INCLINE_THETA_RAD)
    cosine = math.cos(module.AUTHOR_INCLINE_THETA_RAD)
    orientation_w = math.cos(0.5 * module.AUTHOR_INCLINE_THETA_RAD)
    orientation_y = math.sin(0.5 * module.AUTHOR_INCLINE_THETA_RAD)
    terminal_state = terminal_state or module.AUTHOR_INCLINE_SOURCE_FBF_TERMINAL_STATE
    trajectory = []
    for step in range(121):
        time_seconds = step / 60.0
        normal_distance, normal_velocity = _hermite_terminal_state(
            0.499565 - 0.501, 0.000263, time_seconds
        )
        normal_distance += 0.501
        state = {"world_time_seconds": time_seconds}
        for index, (prefix, mu, expected_terminal) in enumerate(
            zip(
                module.AUTHOR_INCLINE_STATE_PREFIXES,
                module.AUTHOR_INCLINE_MU_CELLS,
                terminal_state,
            )
        ):
            tangential_displacement, tangential_velocity = _hermite_terminal_state(
                expected_terminal[0], expected_terminal[1], time_seconds
            )
            x = cosine * tangential_displacement + sine * normal_distance
            z = -sine * tangential_displacement + cosine * normal_distance
            vx = cosine * tangential_velocity + sine * normal_velocity
            vz = -sine * tangential_velocity + cosine * normal_velocity
            state.update(
                {
                    f"{prefix}_mu": mu,
                    f"{prefix}_position_x_m": x,
                    f"{prefix}_position_y_m": (index - 3) * 3.4,
                    f"{prefix}_position_z_m": z,
                    f"{prefix}_orientation_w": orientation_w,
                    f"{prefix}_orientation_x": 0.0,
                    f"{prefix}_orientation_y": orientation_y,
                    f"{prefix}_orientation_z": 0.0,
                    f"{prefix}_linear_velocity_x_m_s": vx,
                    f"{prefix}_linear_velocity_y_m_s": 0.0,
                    f"{prefix}_linear_velocity_z_m_s": vz,
                    f"{prefix}_angular_velocity_x_rad_s": 0.0,
                    f"{prefix}_angular_velocity_y_rad_s": 0.0,
                    f"{prefix}_angular_velocity_z_rad_s": 0.0,
                    f"{prefix}_tangential_displacement_m": tangential_displacement,
                    f"{prefix}_tangential_velocity_m_s": tangential_velocity,
                    f"{prefix}_normal_distance_m": normal_distance,
                    f"{prefix}_normal_velocity_m_s": normal_velocity,
                    f"{prefix}_contact_count": 0 if step < 2 else 4,
                }
            )
        trajectory.append(
            {"step": step, "sim_time": time_seconds, "scene_state": state}
        )
    return trajectory


def _shift_terminal_tangent(
    module, trajectory, prefix, *, displacement=0.0, velocity=0.0
):
    state = trajectory[-1]["scene_state"]
    sine = math.sin(module.AUTHOR_INCLINE_THETA_RAD)
    cosine = math.cos(module.AUTHOR_INCLINE_THETA_RAD)
    state[f"{prefix}_position_x_m"] += cosine * displacement
    state[f"{prefix}_position_z_m"] -= sine * displacement
    state[f"{prefix}_tangential_displacement_m"] += displacement
    state[f"{prefix}_linear_velocity_x_m_s"] += cosine * velocity
    state[f"{prefix}_linear_velocity_z_m_s"] -= sine * velocity
    state[f"{prefix}_tangential_velocity_m_s"] += velocity


def _shift_normal(module, trajectory, step, prefix, delta):
    state = trajectory[step]["scene_state"]
    sine = math.sin(module.AUTHOR_INCLINE_THETA_RAD)
    cosine = math.cos(module.AUTHOR_INCLINE_THETA_RAD)
    state[f"{prefix}_position_x_m"] += sine * delta
    state[f"{prefix}_position_z_m"] += cosine * delta
    state[f"{prefix}_normal_distance_m"] += delta


def test_schedule_pins_grid_clock_and_exact_boxed_pair(tmp_path, capsys):
    module = _load_module()
    schedule = module.SCHEDULES[module.AUTHOR_INCLINE_SCHEDULE_ID]

    assert schedule.scene == "fbf_author_incline_sweep_current_source"
    assert schedule.total_steps == 120
    assert schedule.time_step_seconds == pytest.approx(1.0 / 60.0)
    assert schedule.panel_steps == (0, 30, 60, 90, 120)
    configuration = schedule.configuration_dict()
    assert configuration["mu_cells"] == ("0.3,0.4,0.45,0.5,0.55,0.6,0.8")
    assert configuration["source_fbf_terminal_reference_sha256"] == (
        "f5cc26d2b0ca542b2b98f7fe94a8e2f7f7c9b7cccb3d23c35234ebe45d0d9d12"
    )
    assert configuration["source_fbf_terminal_oracle_projection_sha256"] == (
        "e8b3b5c93a543480bae5c2f50106ecc1b137f65337cc1e725ef8c840efdb8921"
    )
    assert configuration["source_configured_convergence"] == (
        "839/840 flags true; not strict; mu=.55 step_idx=1 false"
    )
    assert configuration["source_mu055_history_sha256"] == (
        "c0aa2d65cbbee24447e7ece9aa97bf83da4cc666ccf16da7edd6874abc22422f"
    )
    assert module._author_incline_oracle_projection_sha256() == (
        "e8b3b5c93a543480bae5c2f50106ecc1b137f65337cc1e725ef8c840efdb8921"
    )
    projection = module.AUTHOR_INCLINE_SOURCE_FBF_ORACLE_PROJECTION
    assert [cell["mu"] for cell in projection["terminal_cells"]] == [
        0.3,
        0.4,
        0.45,
        0.5,
        0.55,
        0.6,
        0.8,
    ]
    assert projection["terminal_cells"][0]["tangential_displacement_m"] == (
        3.5392831695743054
    )
    assert projection["terminal_cells"][-1]["tangential_velocity_m_s"] == (
        2.925503089273808e-8
    )
    assert projection["configured_convergence"] == {
        "metric": "coulomb_rel",
        "comparison": "<",
        "tolerance": 1e-6,
        "converged_steps": 839,
        "total_steps": 840,
        "exceptions": [
            {
                "mu": 0.55,
                "step_idx": 1,
                "configured_converged": False,
                "outer_iters": 200,
                "max_outer": 200,
                "num_contacts": 4,
                "natural_final_residual": 3.273267262002487e-8,
                "terminal_r_coulomb": 1.5311460572898186e-6,
            }
        ],
    }
    assert schedule.collision_detector == "native"
    assert schedule.collision_detector_override is False
    assert module.REQUIRED_VIDEO_SCHEDULES["incline"] == (
        "incline",
        module.AUTHOR_INCLINE_SCHEDULE_ID,
    )

    exact_command = module.build_demo_command(schedule, Path("dart-demos"), tmp_path)
    assert module.HEADLESS_EXACT_FBF_FAIL_FAST_FLAG in exact_command
    assert "--collision-detector" not in exact_command
    boxed = module._derive_boxed_schedule(schedule)
    boxed_command = module.build_demo_command(boxed, Path("dart-demos"), tmp_path)
    assert boxed.pre_run_actions == ("e",)
    assert module.HEADLESS_EXACT_FBF_FAIL_FAST_FLAG not in boxed_command
    comparison = module._derive_solver_comparison_group(schedule)
    assert comparison.members == (schedule.id, boxed.id)

    assert (
        module.main(
            [
                "plan",
                "--scenario",
                schedule.id,
                "--solver-lane",
                "both",
                "--output-root",
                str(tmp_path),
            ]
        )
        == 0
    )
    plan = __import__("json").loads(capsys.readouterr().out)
    assert [item["id"] for item in plan["schedules"]] == [schedule.id, boxed.id]
    assert set(plan["group_outputs"]) == {
        f"{schedule.id}{module.SOLVER_COMPARISON_SUFFIX}"
    }


def test_scene_is_registered_documented_and_has_persistent_mu_color_legend():
    scene_source = (ROOT / "examples/demos/scenes/FbfPaperFrictionScene.cpp").read_text(
        encoding="utf-8"
    )
    spec_source = (ROOT / "examples/demos/scenes/FbfAuthorInclineSpec.hpp").read_text(
        encoding="utf-8"
    )
    registry = (ROOT / "examples/demos/Registry.cpp").read_text(encoding="utf-8")
    readme = (ROOT / "examples/demos/README.md").read_text(encoding="utf-8")

    scene_start = scene_source.index(
        "DemoScene makeFbfAuthorInclineSweepCurrentSourceScene()"
    )
    scene_end = scene_source.index("DemoScene makeFbfPaperBackspinScene()", scene_start)
    scene = scene_source[scene_start:scene_end]
    label_start = scene_source.index(
        "::osg::ref_ptr<::osg::Group> createAuthorInclineLaneLabels"
    )
    label_end = scene_source.index(
        "fbf_author_incline::SolverLane authorInclineSolverLane", label_start
    )
    label_helper = scene_source[label_start:label_end]
    assert "fbf_author_incline::kCells" in scene
    assert "ImGui::ColorButton(" in scene
    assert 'ImGui::Text("mu=%.2g", cell.friction)' in scene
    assert "createAuthorInclineLaneLabels(guiScale)" in scene
    assert "viewer->getRootGroup()->addChild(labels)" in scene
    assert "ctx.addTeardown([viewer, labels]" in scene
    assert "configureAuthorInclineSolver);" in scene
    assert "fbf_author_incline::kCells" in label_helper
    assert "new ::osgText::Text()" in label_helper
    assert "SCREEN_COORDS" in label_helper
    assert 'text << "mu="' in label_helper
    assert "GL_DEPTH_TEST, ::osg::StateAttribute::OFF" in label_helper
    assert "fbf_author_incline::createWorld(authorInclineSolverLane(mode))" in (
        scene_source
    )
    assert "options.fallbackToBoxedLcp = kDartFallbackToBoxedLcpEnabled" in (
        spec_source
    )
    assert "makeFbfAuthorInclineSweepCurrentSourceScene()" in registry
    assert "fbf_author_incline_sweep_current_source" in readme
    assert "The older `fbf_paper_incline` two-cell" in readme
    assert "paper-threshold regression lane" in readme


def test_contract_accepts_exact_and_boxed_and_binds_all_cell_names(tmp_path):
    module = _load_module()
    exact = module.SCHEDULES[module.AUTHOR_INCLINE_SCHEDULE_ID]
    boxed = module._derive_boxed_schedule(exact)

    exact_report = module._validate_schedule_physics_contract(
        exact,
        {"physics_contract": _contract(module)},
        sidecar_path=tmp_path / "exact.json",
    )
    boxed_report = module._validate_schedule_physics_contract(
        boxed,
        {"physics_contract": _contract(module, solver_lane="boxed")},
        sidecar_path=tmp_path / "boxed.json",
    )

    assert exact_report["solver_lane"] == "exact_fbf"
    assert boxed_report["solver_lane"] == "boxed_lcp"
    assert exact_report["mu_values"] == list(module.AUTHOR_INCLINE_MU_CELLS)
    assert exact_report["cell_count"] == 7
    assert (
        _contract(module)["dart_adapter"]["solver"]["exact_options"][
            "fallback_to_boxed_lcp_enabled"
        ]
        is False
    )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (
            lambda contract: contract["source_configuration"]["mu_values"].pop(),
            "source configuration",
        ),
        (
            lambda contract: contract["source_configuration"][
                "mu_grid_provenance"
            ].update(source_default_mu=0.3),
            "source configuration",
        ),
        (
            lambda contract: contract["dart_adapter"]["cells"][-1]["cube"].update(
                body_name="wrong"
            ),
            "cell grid or naming",
        ),
        (
            lambda contract: contract["dart_adapter"]["world"].update(
                time_step_seconds=1.0 / 120.0
            ),
            "world policy",
        ),
        (
            lambda contract: contract["adapter_boundaries"].update(
                source_float32_semantics_implemented=True
            ),
            "adapter boundaries",
        ),
        (
            lambda contract: contract["dart_adapter"]["solver"]["exact_options"].update(
                fallback_to_boxed_lcp_enabled=True
            ),
            "solver contract",
        ),
    ],
)
def test_contract_validation_fails_closed(tmp_path, mutation, message):
    module = _load_module()
    schedule = module.SCHEDULES[module.AUTHOR_INCLINE_SCHEDULE_ID]
    contract = _contract(module)
    mutation(contract)

    with pytest.raises(ValueError, match=message):
        module._validate_schedule_physics_contract(
            schedule,
            {"physics_contract": contract},
            sidecar_path=tmp_path / "timeline.json",
        )


@pytest.mark.parametrize("solver_lane", ["exact", "boxed"])
def test_scene_state_trace_validates_lane_specific_terminal_outcome(
    tmp_path, solver_lane
):
    module = _load_module()
    exact = module.SCHEDULES[module.AUTHOR_INCLINE_SCHEDULE_ID]
    schedule = exact if solver_lane == "exact" else module._derive_boxed_schedule(exact)
    trajectory = _oracle_trace(module)

    report = module._validate_author_incline_scene_state_trace(
        schedule,
        trajectory,
        sidecar_path=tmp_path / "timeline.json",
        expected_last_step=120,
    )

    assert len(module.AUTHOR_INCLINE_SCENE_STATE_FIELDS) == 134
    assert report["sample_count"] == 121
    assert report["complete_120_step_trace"] is True
    assert report["mu_values"] == list(module.AUTHOR_INCLINE_MU_CELLS)
    assert report["physical_outcome_validated"] is True
    assert report["claim_scope"] == "current_source_fbf_terminal_outcome_slice"
    assert report["solver_lane"] == solver_lane
    assert set(report["terminal_state"]) == set(module.AUTHOR_INCLINE_STATE_PREFIXES)
    assert report["terminal_state"]["mu_0_3"]["outcome"] == "slide"
    assert report["terminal_state"]["mu_0_5"]["outcome"] == "stick"
    assert report["maximum_terminal_displacement_absolute_delta_m"] == 0.0
    assert report["maximum_terminal_velocity_absolute_delta_m_s"] == 0.0
    assert report["source_reference_provenance"]["commit"] == (
        "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0"
    )
    assert report["source_oracle_projection_sha256"] == (
        "e8b3b5c93a543480bae5c2f50106ecc1b137f65337cc1e725ef8c840efdb8921"
    )
    assert report["source_reference_strictly_converged"] is False
    assert (
        report["source_reference_provenance"]["configured_convergence_history_sha256"]
        == "c0aa2d65cbbee24447e7ece9aa97bf83da4cc666ccf16da7edd6874abc22422f"
    )
    assert (
        report["source_reference_provenance"]["source_reference_strictly_converged"]
        is False
    )
    assert report["claim_boundary"] == {
        "current_source_fbf_terminal_outcome_slice_valid": True,
        "source_reference_strictly_converged": False,
        "source_trajectory_equivalence": False,
        "source_backend_equivalence": False,
        "solver_equivalence": False,
        "physical_outcome_equivalence": False,
        "paper_parity": False,
    }


@pytest.mark.parametrize("solver_lane", ["exact", "boxed"])
def test_capture_claim_promotes_only_scoped_terminal_outcome(tmp_path, solver_lane):
    module = _load_module()
    exact = module.SCHEDULES[module.AUTHOR_INCLINE_SCHEDULE_ID]
    schedule = exact if solver_lane == "exact" else module._derive_boxed_schedule(exact)
    outcome = module._validate_author_incline_scene_state_trace(
        schedule,
        _oracle_trace(module),
        sidecar_path=tmp_path / "timeline.json",
        expected_last_step=120,
    )
    metadata = {
        "schedule": module.schedule_plan(schedule, Path("dart-demos"), tmp_path),
        "timeline_validation": {
            "author_incline_scene_state_metrics": outcome,
        },
        "actual_simulator": True,
        "generated_imagery": False,
        "paper_comparable": False,
        "automated_current_source_fbf_terminal_outcome_slice_validated": True,
        "current_source_fbf_terminal_outcome_slice": outcome,
        "automated_semantic_outcome_validated": False,
        "semantic_outcome_gate": module.CAPTURE_SEMANTIC_OUTCOME_GATE,
        "known_mismatches": list(schedule.mismatches),
    }

    module._validate_capture_claim_boundary(
        metadata,
        schedule,
        metadata_path=tmp_path / "metadata.json",
    )
    assert metadata["automated_semantic_outcome_validated"] is False

    mutated = copy.deepcopy(metadata)
    mutated["automated_current_source_fbf_terminal_outcome_slice_validated"] = False
    with pytest.raises(
        ValueError,
        match=(
            "capture claim boundary "
            "automated_current_source_fbf_terminal_outcome_slice_validated changed"
        ),
    ):
        module._validate_capture_claim_boundary(
            mutated,
            schedule,
            metadata_path=tmp_path / "mutated-metadata.json",
        )


def test_capture_claim_rejects_broadened_terminal_outcome(tmp_path):
    module = _load_module()
    schedule = module.SCHEDULES[module.AUTHOR_INCLINE_SCHEDULE_ID]
    outcome = module._validate_author_incline_scene_state_trace(
        schedule,
        _oracle_trace(module),
        sidecar_path=tmp_path / "timeline.json",
        expected_last_step=120,
    )
    outcome["claim_boundary"]["solver_equivalence"] = True
    metadata = {
        "schedule": module.schedule_plan(schedule, Path("dart-demos"), tmp_path),
        "timeline_validation": {
            "author_incline_scene_state_metrics": outcome,
        },
        "actual_simulator": True,
        "generated_imagery": False,
        "paper_comparable": False,
        "automated_current_source_fbf_terminal_outcome_slice_validated": True,
        "current_source_fbf_terminal_outcome_slice": outcome,
        "automated_semantic_outcome_validated": False,
        "semantic_outcome_gate": module.CAPTURE_SEMANTIC_OUTCOME_GATE,
        "known_mismatches": list(schedule.mismatches),
    }

    with pytest.raises(ValueError, match="terminal claim boundary changed"):
        module._validate_capture_claim_boundary(
            metadata,
            schedule,
            metadata_path=tmp_path / "metadata.json",
        )


def test_solver_comparison_group_claim_binds_ordered_terminal_slices(tmp_path):
    module = _load_module()
    exact = module.SCHEDULES[module.AUTHOR_INCLINE_SCHEDULE_ID]
    boxed = module._derive_boxed_schedule(exact)
    group = module._derive_solver_comparison_group(exact)
    outcomes = [
        module._validate_author_incline_scene_state_trace(
            schedule,
            _oracle_trace(module),
            sidecar_path=tmp_path / f"{schedule.solver_lane}.json",
            expected_last_step=120,
        )
        for schedule in (exact, boxed)
    ]
    metadata = {
        "source_segment": group.source_segment,
        "solver_lane": group.solver_lane,
        "source_group_id": group.source_group_id,
        "actual_simulator": True,
        "generated_imagery": False,
        "paper_comparable": False,
        "automated_current_source_fbf_terminal_outcome_slice_validated": True,
        "current_source_fbf_terminal_outcome_slices": outcomes,
        "automated_semantic_outcome_validated": False,
        "semantic_outcome_gate": module._group_semantic_outcome_gate(group),
    }

    module._validate_group_claim_boundary(
        metadata,
        group,
        metadata_path=tmp_path / "group-metadata.json",
    )
    assert [report["solver_lane"] for report in outcomes] == ["exact", "boxed"]
    assert metadata["automated_semantic_outcome_validated"] is False

    mutated = copy.deepcopy(metadata)
    mutated["current_source_fbf_terminal_outcome_slices"].reverse()
    with pytest.raises(ValueError, match="terminal outcome claim changed"):
        module._validate_group_claim_boundary(
            mutated,
            group,
            metadata_path=tmp_path / "mutated-group-metadata.json",
        )


def test_source_oracle_projection_digest_fails_closed(tmp_path, monkeypatch):
    module = _load_module()
    schedule = module.SCHEDULES[module.AUTHOR_INCLINE_SCHEDULE_ID]
    projection = copy.deepcopy(module.AUTHOR_INCLINE_SOURCE_FBF_ORACLE_PROJECTION)
    projection["terminal_cells"][0]["tangential_displacement_m"] += 0.01
    monkeypatch.setattr(
        module, "AUTHOR_INCLINE_SOURCE_FBF_ORACLE_PROJECTION", projection
    )

    with pytest.raises(ValueError, match="source oracle projection"):
        module._validate_author_incline_scene_state_trace(
            schedule,
            _oracle_trace(module),
            sidecar_path=tmp_path / "changed-oracle.json",
            expected_last_step=120,
        )


def test_stationary_trace_is_not_positive_evidence(tmp_path):
    module = _load_module()
    schedule = module.SCHEDULES[module.AUTHOR_INCLINE_SCHEDULE_ID]
    stationary_terminal = tuple(
        (0.0, 0.0) for _ in module.AUTHOR_INCLINE_STATE_PREFIXES
    )
    trajectory = _oracle_trace(module, terminal_state=stationary_terminal)

    with pytest.raises(ValueError, match="terminal oracle"):
        module._validate_author_incline_scene_state_trace(
            schedule,
            trajectory,
            sidecar_path=tmp_path / "stationary.json",
            expected_last_step=120,
        )


def test_exact_and_boxed_schedules_independently_validate_the_source_oracle(tmp_path):
    module = _load_module()
    exact = module.SCHEDULES[module.AUTHOR_INCLINE_SCHEDULE_ID]
    boxed = module._derive_boxed_schedule(exact)

    exact_report = module._validate_author_incline_scene_state_trace(
        exact,
        _oracle_trace(module),
        sidecar_path=tmp_path / "exact.json",
        expected_last_step=120,
    )
    boxed_report = module._validate_author_incline_scene_state_trace(
        boxed,
        _oracle_trace(module),
        sidecar_path=tmp_path / "boxed.json",
        expected_last_step=120,
    )
    assert exact_report["solver_lane"] == "exact"
    assert boxed_report["solver_lane"] == "boxed"
    assert (
        exact_report["source_reference_provenance"]
        == boxed_report["source_reference_provenance"]
    )


def test_slide_stick_ordering_is_independently_fail_closed(tmp_path, monkeypatch):
    module = _load_module()
    schedule = module.SCHEDULES[module.AUTHOR_INCLINE_SCHEDULE_ID]
    reversed_slides = list(module.AUTHOR_INCLINE_SOURCE_FBF_TERMINAL_STATE)
    reversed_slides[0], reversed_slides[1] = reversed_slides[1], reversed_slides[0]
    reversed_slides = tuple(reversed_slides)
    trajectory = _oracle_trace(module, terminal_state=reversed_slides)
    monkeypatch.setattr(
        module,
        "AUTHOR_INCLINE_SOURCE_FBF_TERMINAL_STATE",
        reversed_slides,
    )

    with pytest.raises(ValueError, match="slide/stick outcome ordering"):
        module._validate_author_incline_scene_state_trace(
            schedule,
            trajectory,
            sidecar_path=tmp_path / "reversed.json",
            expected_last_step=120,
        )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (
            lambda trajectory: trajectory[120]["scene_state"].pop(
                "mu_0_8_contact_count"
            ),
            "fields changed",
        ),
        (
            lambda trajectory: trajectory[60]["scene_state"].update(mu_0_8_mu=0.7),
            "mu grid",
        ),
        (
            lambda trajectory: trajectory[30]["scene_state"].update(
                mu_0_45_tangential_displacement_m=1.0
            ),
            "derived incline state",
        ),
        (
            lambda trajectory: trajectory[10].update(sim_time=0.5),
            "time mismatch",
        ),
        (
            lambda trajectory: trajectory[60]["scene_state"].update(
                mu_0_4_position_y_m=-6.79
            ),
            "lateral drift",
        ),
        (
            lambda trajectory: trajectory[60]["scene_state"].update(
                mu_0_5_orientation_w=1.0,
                mu_0_5_orientation_y=0.0,
            ),
            "upright-orientation",
        ),
        (
            lambda trajectory: trajectory[60]["scene_state"].update(
                mu_0_55_angular_velocity_x_rad_s=0.1
            ),
            "angular-speed",
        ),
        (
            lambda trajectory: trajectory[120]["scene_state"].update(
                mu_0_6_contact_count=0
            ),
            "contact participation",
        ),
    ],
)
def test_scene_state_validation_fails_closed(tmp_path, mutation, message):
    module = _load_module()
    schedule = module.SCHEDULES[module.AUTHOR_INCLINE_SCHEDULE_ID]
    trajectory = _oracle_trace(module)
    mutation(trajectory)

    with pytest.raises(ValueError, match=message):
        module._validate_author_incline_scene_state_trace(
            schedule,
            trajectory,
            sidecar_path=tmp_path / "timeline.json",
            expected_last_step=120,
        )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (
            lambda module, trajectory: _shift_normal(
                module, trajectory, 60, "mu_0_3", -0.01
            ),
            "normal-motion bound",
        ),
        (
            lambda module, trajectory: _shift_terminal_tangent(
                module, trajectory, "mu_0_45", displacement=0.02
            ),
            "terminal oracle",
        ),
        (
            lambda module, trajectory: _shift_terminal_tangent(
                module, trajectory, "mu_0_5", velocity=0.02
            ),
            "terminal oracle",
        ),
    ],
)
def test_physically_consistent_adversarial_mutations_fail(tmp_path, mutation, message):
    module = _load_module()
    schedule = module.SCHEDULES[module.AUTHOR_INCLINE_SCHEDULE_ID]
    trajectory = _oracle_trace(module)
    mutation(module, trajectory)

    with pytest.raises(ValueError, match=message):
        module._validate_author_incline_scene_state_trace(
            schedule,
            trajectory,
            sidecar_path=tmp_path / "timeline.json",
            expected_last_step=120,
        )
