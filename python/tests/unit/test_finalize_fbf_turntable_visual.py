import argparse
import copy
import csv
import importlib.util
import io
import json
import shutil
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts/finalize_fbf_turntable_visual.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "finalize_fbf_turntable_visual", SCRIPT
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _trace_rows(module, scenario, *, strict_negative=False):
    expected = module.SCENARIOS[scenario]["expected_outcome"]
    rows = []
    for step in range(module.TOTAL_STEPS + 1):
        if expected == "retained_through_6s":
            x = 1.0 - 0.0005 * step
            y = 0.002 * step
            z = 0.12
            contacts = 0 if step == 0 else 4
            exact_solves = step
        else:
            x = 1.0 + 0.02 * step
            y = 0.005 * step
            z = 0.25 if step <= 60 else 0.25 - 0.01 * (step - 60)
            contacts = 0 if step == 0 or step > 60 else 4
            exact_solves = min(step, 60)
        status = "not_run" if step == 0 else "success"
        residual = "nan" if step == 0 else "5e-7"
        if strict_negative and step == module.TOTAL_STEPS:
            status = "max_iterations_accepted"
            residual = "3e-4"
        rows.append(
            {
                "step": str(step),
                "time": str(step / 60.0),
                "scenario": scenario,
                "solver": "exact_fbf",
                "body": "turntable_rider_body",
                "x": str(x),
                "y": str(y),
                "z": str(z),
                "vx": "0.0",
                "vy": "0.0",
                "vz": "0.0",
                "up_z": "1.0",
                "contacts": str(contacts),
                "exact_solves": str(exact_solves),
                "warm_starts": str(max(0, exact_solves - 1)),
                "fallbacks": "0",
                "residual": residual,
                "status": status,
            }
        )
    return rows


def _trace_text(module, scenario, rows=None, *, strict_negative=False):
    stream = io.StringIO()
    writer = csv.DictWriter(stream, fieldnames=module.TRACE_COLUMNS)
    writer.writeheader()
    writer.writerows(
        rows
        if rows is not None
        else _trace_rows(module, scenario, strict_negative=strict_negative)
    )
    return stream.getvalue()


def _parsed_matrix(module, *, strict_negative_scenario=None):
    parsed = {lane: {} for lane in module.LANES}
    for lane in module.LANES:
        for scenario in module.SCENARIOS:
            parsed[lane][scenario] = module.parse_trace_text(
                _trace_text(
                    module,
                    scenario,
                    strict_negative=(
                        lane == "paper_cpu_native"
                        and scenario == strict_negative_scenario
                    ),
                ),
                scenario,
                lane=lane,
            )
    return parsed


def _physics_contract(module, scenario, *, solver_contract, binary_role):
    configuration = module.SCENARIOS[scenario]
    scenario_index = list(module.SCENARIOS).index(scenario)
    mass = 500.0 * 0.3 * 0.3 * 0.3
    moment = mass * (0.3 * 0.3 + 0.3 * 0.3) / 12.0
    solver = {
        "dart_best": {
            "type": "exact_fbf",
            "contract": "dart_best",
            "split_impulse_enabled": False,
            "max_outer_iterations": 500,
            "tolerance": 1.0e-6,
            "inner_max_sweeps": 120,
            "inner_local_iterations": 32,
            "step_size_scale": 2.0,
            "warm_start_enabled": True,
            "fallback_to_boxed_lcp_enabled": True,
            "projected_gradient_retry_enabled": True,
            "dense_residual_polish_enabled": True,
            "contact_row_operator_enabled": True,
            "dense_contact_row_snapshot_enabled": True,
        },
        "paper_cpu": {
            "type": "exact_fbf",
            "contract": "paper_cpu",
            "split_impulse_enabled": False,
            "max_outer_iterations": 200,
            "tolerance": 1.0e-6,
            "inner_max_sweeps": 10,
            "inner_local_iterations": 1,
            "step_size_scale": 1.0,
            "warm_start_enabled": True,
            "fallback_to_boxed_lcp_enabled": False,
            "projected_gradient_retry_enabled": False,
            "dense_residual_polish_enabled": False,
            "contact_row_operator_enabled": True,
            "dense_contact_row_snapshot_enabled": False,
        },
    }[solver_contract]
    implementation_source = (
        module.DEMO_SOURCE if binary_role == "dart_demos" else module.TRACE_SOURCE
    )
    friction = float(configuration["mu"])
    return {
        "schema_version": module.PHYSICS_CONTRACT_SCHEMA_VERSION,
        "kind": module.PHYSICS_CONTRACT_KIND,
        "author_source": {
            "commit": module.AUTHOR_COMMIT,
            "turntable_run_py_sha256": module.AUTHOR_TURNTABLE_SOURCE_SHA256,
        },
        "physics_spec_source_sha256": module.sha256(
            module.AUTHOR_TURNTABLE_SPEC_SOURCE
        ),
        "binary_binding": {
            "role": binary_role,
            "implementation_source_sha256": module.sha256(implementation_source),
        },
        "scenario": {
            "trace_id": scenario,
            "demo_scene_id": configuration["scene"],
            "friction": friction,
            "angular_velocity_rad_s": float(configuration["omega"]),
            "expected_outcome": module.PHYSICS_EXPECTED_OUTCOMES[scenario_index],
        },
        "world": {
            "time_step_seconds": module.DT_SECONDS,
            "gravity_m_s2": [0.0, 0.0, -9.81],
            "simulation_threads": 1,
            "deactivation_enabled": False,
        },
        "collision": {
            "detector": "native",
            "contact_manifold": "four_point_planar",
            "max_contacts": 4,
            "max_contacts_per_pair": 4,
        },
        "solver": solver,
        "support": {
            "shape": "cylinder",
            "mobile": False,
            "radius_m": 2.0,
            "height_m": 0.1,
            "friction": friction,
            "initial_pose": {
                "translation": [0.0, 0.0, 0.05],
                "rotation": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            },
        },
        "rider": {
            "shape": "box",
            "size_m": [0.3, 0.3, 0.3],
            "friction": friction,
            "mass_kg": mass,
            "moment_kg_m2": [
                [moment, 0.0, 0.0],
                [0.0, moment, 0.0],
                [0.0, 0.0, moment],
            ],
            "initial_pose": {
                "translation": [1.0, 0.0, 0.455],
                "rotation": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            },
            "initial_linear_velocity_m_s": [0.0, 0.0, 0.0],
            "initial_angular_velocity_rad_s": [0.0, 0.0, 0.0],
        },
        "control": {
            "settle_duration_seconds": 0.5,
            "ramp_duration_seconds": 0.5,
            "duration_seconds": 6.0,
            "ramp": "cubic_smoothstep",
        },
        "visual_asset_identity": None,
    }


def _physics_contract_matrix(module):
    return {
        "demo": {
            scenario: _physics_contract(
                module,
                scenario,
                solver_contract="dart_best",
                binary_role="dart_demos",
            )
            for scenario in module.SCENARIOS
        },
        "trace": {
            solver_contract: {
                scenario: _physics_contract(
                    module,
                    scenario,
                    solver_contract=solver_contract,
                    binary_role="fbf_paper_trace",
                )
                for scenario in module.SCENARIOS
            }
            for solver_contract in ("dart_best", "paper_cpu")
        },
    }


def test_defaults_freeze_source_order_and_two_trace_lanes():
    module = _load_module()

    assert module.DEFAULT_BUNDLE.name == "fig04_turntable_author_current_v1"
    assert module.SCHEMA_VERSION == "dart.fbf_turntable_visual_bundle/v1"
    assert module.CAPTURE_IDS == (
        "turntable_author_mu02_omega2",
        "turntable_author_mu02_omega5",
        "turntable_author_mu05_omega2",
        "turntable_author_mu05_omega5",
    )
    assert module.GROUP_LABELS == (
        "mu=0.2, omega=2 rad/s",
        "mu=0.2, omega=5 rad/s",
        "mu=0.5, omega=2 rad/s",
        "mu=0.5, omega=5 rad/s",
    )
    assert module.GROUP_PANEL_STEPS == (136, 120, 360, 90)
    assert module.GROUP_PANEL_LABELS == (
        "MU 0.2 OMEGA 2 T 2.27S",
        "MU 0.2 OMEGA 5 T 2.00S",
        "MU 0.5 OMEGA 2 T 6.00S",
        "MU 0.5 OMEGA 5 T 1.50S",
    )
    assert module.MANUAL_VERDICTS["mu05_omega2_retained_through_6s_visible"] is True
    assert "mu05_omega2_captured_visible" not in module.MANUAL_VERDICTS
    assert module.LANES["current_visual"] == {
        "solver_contract": "dart_best",
        "collision_frontend": "native",
        "role": "visual_compatible_author_source_port",
    }
    assert module.LANES["paper_cpu_native"]["solver_contract"] == "paper_cpu"
    assert module.LANES["paper_cpu_native"]["collision_frontend"] == "native"
    assert len(module.FRAME_STEPS) == 181
    assert module.OUTPUT_FPS == 30
    assert module.VIDEO_FRAME_COUNT == 181
    assert module.VIDEO_DURATION_SECONDS == pytest.approx(181 / 30)
    assert len(module.TRACE_PATHS) == 16
    assert len(module.CAPTURE_PATHS) == 38
    assert len(module.STAGING_PATHS) == 751
    assert len(module.EXPECTED_FINAL_PATHS) == 60
    assert module.CAPTURE_PATHS.isdisjoint(module.STAGING_PATHS)
    assert all("/frames/" not in path for path in module.EXPECTED_FINAL_PATHS)
    assert all("/panel_frames/" not in path for path in module.EXPECTED_FINAL_PATHS)
    assert all(
        path.endswith(".png") for path in module.OUTCOME_STILL_PATH_BY_CAPTURE.values()
    )


def test_trace_and_capture_commands_freeze_contracts_and_order(tmp_path):
    module = _load_module()
    trace = Path("/trace")

    current = module._trace_argv(
        trace, "turntable_author_mu_0_2_omega_2", "current_visual"
    )
    strict = module._trace_argv(
        trace, "turntable_author_mu_0_2_omega_2", "paper_cpu_native"
    )
    assert current[10:12] == ["dart_best", "native"]
    assert strict[10:12] == ["paper_cpu", "native"]
    assert current[:6] == [
        "/trace",
        "turntable_author_mu_0_2_omega_2",
        "exact_fbf",
        "1",
        "360",
        "nan",
    ]

    argv = module._capture_argv(
        Path("/python"),
        Path("/runner"),
        Path("/demo"),
        tmp_path,
        Path("/ffmpeg"),
        Path("/ffprobe"),
    )
    selected = [
        argv[index + 1] for index, value in enumerate(argv) if value == "--scenario"
    ]
    assert selected == list(module.CAPTURE_IDS)
    assert argv[-4:] == [
        "--python",
        "/python",
        "--out",
        str(tmp_path / "run-summary.json"),
    ]
    assert module._expected_schedule_output(tmp_path, module.CAPTURE_IDS[0]) == {
        "directory": str(tmp_path / module.CAPTURE_IDS[0]),
        "timeline": str(tmp_path / module.CAPTURE_IDS[0] / "timeline.json"),
        "panel": str(tmp_path / module.CAPTURE_IDS[0] / "panel.png"),
        "mp4": str(tmp_path / module.CAPTURE_IDS[0] / "clip.mp4"),
        "gif": None,
    }
    demo_argv = module._expected_demo_argv(
        tmp_path,
        capture_id=module.CAPTURE_IDS[0],
        scene=module.SCENARIOS["turntable_author_mu_0_2_omega_2"]["scene"],
        demo=Path("/demo"),
    )
    assert "--collision-detector" not in demo_argv
    assert [
        demo_argv[index + 1]
        for index, value in enumerate(demo_argv)
        if value == "--headless-shot-at"
    ] == [
        f"{step}:{tmp_path / module.CAPTURE_IDS[0] / 'frames' / f'step_{step:06d}.png'}"
        for step in module.FRAME_STEPS
    ]


def test_source_identities_bind_native_cylinder_and_dispatch(tmp_path, monkeypatch):
    module = _load_module()
    expected = {
        "native_cylinder_source",
        "native_cylinder_header",
        "native_narrow_phase_source",
        "native_narrow_phase_header",
    }

    capture = module._capture_source_identity(runner=SCRIPT)
    trace = module._trace_source_identity()
    assert expected <= set(capture)
    assert expected <= set(trace)
    assert {
        "author_turntable_spec_source",
        "demo_cmake_source",
        "demo_main_source",
        "demo_scene_header",
    } <= set(capture)
    assert {
        "author_turntable_spec_source",
        "trace_cmake_source",
        "trace_source",
    } <= set(trace)
    assert {
        "author_turntable_visual_obj",
        "author_turntable_visual_mtl",
    } <= set(capture)
    assert capture["author_turntable_visual_obj"] == {
        "relative_path": "data/obj/fbf_author_turntable_disc.obj",
        "size_bytes": module.AUTHOR_TURNTABLE_VISUAL_OBJ.stat().st_size,
        "sha256": module.sha256(module.AUTHOR_TURNTABLE_VISUAL_OBJ),
    }
    assert capture["author_turntable_visual_mtl"] == {
        "relative_path": "data/obj/fbf_author_turntable_disc.mtl",
        "size_bytes": module.AUTHOR_TURNTABLE_VISUAL_MTL.stat().st_size,
        "sha256": module.sha256(module.AUTHOR_TURNTABLE_VISUAL_MTL),
    }

    runtime = tmp_path / "runtime"
    runtime.write_bytes(b"runtime")
    monkeypatch.setattr(
        module, "_in_tree_runtime_dependency_identity", lambda *args, **kwargs: {}
    )
    full = module._source_identity(
        trace_binary=runtime,
        demo=runtime,
        runner=SCRIPT,
        ffmpeg=runtime,
        ffprobe=runtime,
        python=runtime,
    )
    assert expected <= set(full)
    assert {
        "author_turntable_spec_source",
        "demo_cmake_source",
        "demo_main_source",
        "demo_scene_header",
        "trace_cmake_source",
    } <= set(full)
    assert {
        "author_turntable_visual_obj",
        "author_turntable_visual_mtl",
    } <= set(full)

    changed = tmp_path / "CylinderCollision.cpp"
    changed.write_bytes(module.NATIVE_CYLINDER_SOURCE.read_bytes() + b"\n")
    before = trace["native_cylinder_source"]["sha256"]
    monkeypatch.setattr(module, "NATIVE_CYLINDER_SOURCE", changed)
    after = module._trace_source_identity()["native_cylinder_source"]["sha256"]
    assert after != before


@pytest.mark.parametrize(
    "resource_key",
    ["author_turntable_visual_obj", "author_turntable_visual_mtl"],
)
def test_finalized_source_identity_detects_each_renderer_resource_change(
    tmp_path, monkeypatch, resource_key
):
    module = _load_module()
    runtime = tmp_path / "runtime"
    runtime.write_bytes(b"runtime")
    monkeypatch.setattr(
        module, "_in_tree_runtime_dependency_identity", lambda *args, **kwargs: {}
    )
    current = module._source_identity(
        trace_binary=runtime,
        demo=runtime,
        runner=SCRIPT,
        ffmpeg=runtime,
        ffprobe=runtime,
        python=runtime,
    )
    persisted = copy.deepcopy(current)
    persisted[resource_key]["sha256"] = "0" * 64

    assert persisted != module._source_identity(
        trace_binary=runtime,
        demo=runtime,
        runner=SCRIPT,
        ffmpeg=runtime,
        ffprobe=runtime,
        python=runtime,
    )


def test_ldd_parser_binds_in_tree_dependencies_and_rejects_missing(tmp_path):
    module = _load_module()
    build_root = tmp_path / "build"
    library = build_root / "lib" / "libdart.so.6.19"
    library.parent.mkdir(parents=True)
    library.write_bytes(b"libdart")
    external = tmp_path / "external.so"
    external.write_bytes(b"external")
    output = (
        f"libdart.so.6.19 => {library} (0x1)\n" f"libexternal.so => {external} (0x2)\n"
    )

    assert module._parse_ldd_in_tree_paths(output, build_root=build_root) == [library]
    with pytest.raises(ValueError, match="unresolved runtime dependency"):
        module._parse_ldd_in_tree_paths(
            "libdart.so => not found\n", build_root=build_root
        )


def test_world_view_motion_allows_repeated_post_ejection_frames():
    module = _load_module()
    hashes = {step: f"frame-{step}" for step in module.FRAME_STEPS}
    for step in module.FRAME_STEPS:
        if step >= 180:
            hashes[step] = "post-ejection-static"

    assert module._validate_world_view_motion_hashes(hashes, capture_id="ejected") == 91

    entirely_static = {step: "same" for step in module.FRAME_STEPS}
    with pytest.raises(ValueError, match="entirely static"):
        module._validate_world_view_motion_hashes(entirely_static, capture_id="static")

    missing_keyframe_motion = dict(hashes)
    missing_keyframe_motion[30] = missing_keyframe_motion[0]
    with pytest.raises(ValueError, match="keyframes 0/30"):
        module._validate_world_view_motion_hashes(
            missing_keyframe_motion, capture_id="bad-keyframe"
        )


@pytest.mark.parametrize("scenario", list(_load_module().SCENARIOS))
def test_current_trace_parser_accepts_expected_outcome_matrix(scenario):
    module = _load_module()

    parsed = module.parse_trace_text(
        _trace_text(module, scenario), scenario, lane="current_visual"
    )

    summary = parsed["summary"]
    assert summary["row_count"] == 361
    assert summary["solver_contract"] == "dart_best"
    assert summary["collision_frontend"] == "native"
    assert summary["solver_contract_valid"] is True
    assert summary["expected_outcome_match"] is True
    assert summary["physical_outcome_authority"] is True
    assert summary["selected_panel_states"][0]["residual"] is None
    assert len(parsed["solver_projection"]) == 361


def test_retained_capture_schedule_uses_bounded_finite_horizon_wording():
    module = _load_module()
    scenario = "turntable_author_mu_0_5_omega_2"

    parsed = module.parse_trace_text(
        _trace_text(module, scenario), scenario, lane="current_visual"
    )

    assert module.SCENARIOS[scenario]["expected_outcome"] == "retained_through_6s"
    assert parsed["summary"]["expected_outcome"] == "retained_through_6s"
    assert parsed["summary"]["classified_outcome"] == "captured"
    assert parsed["summary"]["expected_outcome_match"] is True


def test_current_trace_parser_accepts_airborne_not_run_prefix():
    module = _load_module()
    scenario = "turntable_author_mu_0_2_omega_2"
    rows = _trace_rows(module, scenario)
    for step in range(1, 13):
        rows[step].update(
            contacts="0",
            exact_solves="0",
            warm_starts="0",
            residual="nan",
            status="not_run",
        )

    parsed = module.parse_trace_text(
        _trace_text(module, scenario, rows), scenario, lane="current_visual"
    )

    assert parsed["summary"]["solver_contract_valid"] is True
    assert parsed["solver_projection"][12]["status"] == "not_run"
    assert parsed["solver_projection"][13]["status"] == "success"


def test_trace_parser_rejects_not_run_after_solver_started():
    module = _load_module()
    scenario = "turntable_author_mu_0_2_omega_2"
    rows = _trace_rows(module, scenario)
    rows[20].update(
        contacts="0",
        exact_solves="20",
        warm_starts="19",
        residual="nan",
        status="not_run",
    )

    with pytest.raises(ValueError, match="not_run step 20 follows an exact solve"):
        module.parse_trace_text(
            _trace_text(module, scenario, rows), scenario, lane="current_visual"
        )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda rows: rows.pop(), "expected 361 rows"),
        (lambda rows: rows[10].update(step="11"), "noncontiguous step"),
        (lambda rows: rows[10].update(time="1.0"), "is not step/60"),
        (lambda rows: rows[10].update(scenario="backspin"), "scenario"),
        (lambda rows: rows[10].update(solver="boxed_lcp"), "identity mismatch"),
        (lambda rows: rows[10].update(x="nan"), "nonfinite x"),
        (lambda rows: rows[10].update(status="unknown"), "unknown step"),
        (lambda rows: rows[10].update(residual="-1.0"), "residual is negative"),
        (
            lambda rows: rows[10].update(residual="1.1e-6"),
            "solver contract failed",
        ),
        (lambda rows: rows[10].update(fallbacks="1"), "fallback"),
        (
            lambda rows: rows[-1].update(x="1.0", y="0.0", z="-10.0"),
            "outcome",
        ),
    ],
)
def test_current_trace_parser_fails_closed(mutation, message):
    module = _load_module()
    scenario = "turntable_author_mu_0_2_omega_2"
    rows = _trace_rows(module, scenario)
    mutation(rows)

    with pytest.raises(ValueError, match=message):
        module.parse_trace_text(
            _trace_text(module, scenario, rows), scenario, lane="current_visual"
        )


def test_vertical_fallthrough_does_not_count_as_author_turntable_ejection():
    module = _load_module()
    scenario = "turntable_author_mu_0_2_omega_2"
    rows = _trace_rows(module, scenario)
    rows[-1].update(x="1.1", y="0.0", z="-125.0", contacts="0")

    with pytest.raises(ValueError, match="outcome"):
        module.parse_trace_text(
            _trace_text(module, scenario, rows), scenario, lane="current_visual"
        )


def test_fallthrough_then_outward_motion_does_not_count_as_ejection():
    module = _load_module()
    scenario = "turntable_author_mu_0_2_omega_2"
    rows = _trace_rows(module, scenario)
    assert float(rows[-1]["x"]) > 2.15
    rows[30].update(z="-1.0")

    with pytest.raises(ValueError, match="invalid_support_fallthrough"):
        module.parse_trace_text(
            _trace_text(module, scenario, rows), scenario, lane="current_visual"
        )


def test_retained_cell_cannot_return_after_a_prior_source_exit():
    module = _load_module()
    scenario = "turntable_author_mu_0_5_omega_2"
    rows = _trace_rows(module, scenario)
    rows[100].update(x="2.2", y="0.0", contacts="0")

    with pytest.raises(ValueError, match="outcome"):
        module.parse_trace_text(
            _trace_text(module, scenario, rows), scenario, lane="current_visual"
        )


def test_strict_negative_is_retained_but_cannot_enter_current_lane():
    module = _load_module()
    scenario = "turntable_author_mu_0_5_omega_5"
    text = _trace_text(module, scenario, strict_negative=True)

    strict = module.parse_trace_text(text, scenario, lane="paper_cpu_native")
    summary = strict["summary"]
    assert summary["solver_contract"] == "paper_cpu"
    assert summary["collision_frontend"] == "native"
    assert summary["solver_contract_valid"] is False
    assert summary["solver_contract_failure_steps"] == [360]
    assert summary["accepted_at_cap_steps"] == [360]
    assert summary["physical_outcome_authority"] is False
    assert module._expected_trace_returncode(summary) == 1

    passing_strict = module.parse_trace_text(
        _trace_text(module, scenario), scenario, lane="paper_cpu_native"
    )
    assert module._expected_trace_returncode(passing_strict["summary"]) == 0

    with pytest.raises(ValueError, match="current visual trace solver contract failed"):
        module.parse_trace_text(text, scenario, lane="current_visual")


def test_trace_runner_allows_exit_one_only_for_separate_strict_negative(
    monkeypatch,
):
    module = _load_module()
    process = module.subprocess.CompletedProcess(
        args=["trace"], returncode=1, stdout="trace", stderr="diagnostic"
    )
    monkeypatch.setattr(module.subprocess, "run", lambda *args, **kwargs: process)

    assert (
        module._run_trace_command(["trace"], lane="paper_cpu_native", timeout=1.0)
        is process
    )
    with pytest.raises(ValueError, match="trace command failed with exit 1"):
        module._run_trace_command(["trace"], lane="current_visual", timeout=1.0)

    process.returncode = 2
    with pytest.raises(ValueError, match="trace command failed with exit 2"):
        module._run_trace_command(["trace"], lane="paper_cpu_native", timeout=1.0)


def test_strict_lane_still_rejects_unknown_status_and_fallback():
    module = _load_module()
    scenario = "turntable_author_mu_0_5_omega_5"
    rows = _trace_rows(module, scenario, strict_negative=True)
    rows[-1]["status"] = "failure"
    with pytest.raises(ValueError, match="unknown step"):
        module.parse_trace_text(
            _trace_text(module, scenario, rows),
            scenario,
            lane="paper_cpu_native",
        )

    rows = _trace_rows(module, scenario, strict_negative=True)
    rows[-1]["fallbacks"] = "1"
    with pytest.raises(ValueError, match="fallback"):
        module.parse_trace_text(
            _trace_text(module, scenario, rows),
            scenario,
            lane="paper_cpu_native",
        )


def test_trace_summary_keeps_lanes_separate_and_requires_exact_matrix():
    module = _load_module()
    failing = "turntable_author_mu_0_5_omega_5"
    parsed = _parsed_matrix(module, strict_negative_scenario=failing)

    summary = module.summarize_trace_lanes(parsed)

    assert summary["render_binding_lane"] == "current_visual"
    assert summary["separate_diagnostic_lane"] == "paper_cpu_native"
    assert summary["cross_lane_substitution_allowed"] is False
    assert summary["lanes"]["current_visual"]["all_solver_contract_valid"] is True
    assert summary["lanes"]["paper_cpu_native"]["all_solver_contract_valid"] is False
    assert summary["lanes"]["current_visual"]["physical_outcome_authority"] is True
    assert summary["lanes"]["paper_cpu_native"]["physical_outcome_authority"] is False

    incomplete = copy.deepcopy(parsed)
    incomplete["current_visual"].pop(next(iter(module.SCENARIOS)))
    with pytest.raises(ValueError, match="scenario matrix is incomplete"):
        module.summarize_trace_lanes(incomplete)

    crossed = copy.deepcopy(parsed)
    scenario = next(iter(module.SCENARIOS))
    crossed["current_visual"][scenario] = crossed["paper_cpu_native"][scenario]
    with pytest.raises(ValueError, match="cross-lane"):
        module.summarize_trace_lanes(crossed)


def test_projection_comparison_is_exact_and_current_lane_only():
    module = _load_module()
    scenario = "turntable_author_mu_0_5_omega_2"
    projection = module.parse_trace_text(
        _trace_text(module, scenario), scenario, lane="current_visual"
    )["solver_projection"]

    result = module._compare_solver_projections(
        projection, copy.deepcopy(projection), scenario=scenario
    )

    assert result["byte_identical"] is True
    assert result["core_projection_equivalent"] is True
    assert result["mismatch_count"] == 0
    assert result["lane"] == "current_visual"
    assert result["paper_cpu_native_compared"] is False
    assert result["full_state_trace_equivalence"] is False

    warm_start_drift = copy.deepcopy(projection)
    warm_start_drift[120]["warm_starts"] += 1
    disclosed = module._compare_solver_projections(
        projection, warm_start_drift, scenario=scenario
    )
    assert disclosed["byte_identical"] is False
    assert disclosed["core_projection_equivalent"] is True
    assert disclosed["mismatch_steps"] == [120]
    assert disclosed["mismatch_fields"] == ["warm_starts"]

    core_changed = copy.deepcopy(projection)
    core_changed[120]["contacts"] += 1
    with pytest.raises(ValueError, match="core projection differs at step 120"):
        module._compare_solver_projections(projection, core_changed, scenario=scenario)


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


def test_manual_inspection_binds_source_order_artifacts_and_nonclaims(tmp_path):
    module = _load_module()
    record = _manual_record(module, tmp_path)
    module.write_json(tmp_path / "manual-inspection.json", record)

    assert module.validate_manual_inspection(tmp_path) == record
    assert record["verdicts"]["paper_cpu_native_lane_kept_separate"] is True
    assert record["verdicts"]["rendered_demo_and_trace_full_state_equivalent"] is False
    assert record["verdicts"]["paper_parity"] is False
    assert record["verdicts"]["timing_verdict"] is None


@pytest.mark.parametrize("mutation", ["promote", "wrong_hash", "extra_path"])
def test_manual_inspection_fails_closed(tmp_path, mutation):
    module = _load_module()
    record = _manual_record(module, tmp_path)
    if mutation == "promote":
        record["verdicts"]["paper_parity"] = True
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


def test_outcome_still_is_hash_bound_to_timeline_and_fails_closed(tmp_path):
    module = _load_module()
    capture_id = module.CAPTURE_IDS[0]
    step = module.GROUP_PANEL_STEP_BY_CAPTURE[capture_id]
    frame_path = tmp_path / capture_id / "frames" / f"step_{step:06d}.png"
    frame_path.parent.mkdir(parents=True)
    frame_path.write_bytes(b"timeline outcome")
    timeline = {
        "frames": {
            str(step): {
                "path": str(frame_path),
                "sha256": module.sha256(frame_path),
            }
        }
    }
    still = tmp_path / module.OUTCOME_STILL_PATH_BY_CAPTURE[capture_id]
    still.parent.mkdir(parents=True)
    still.write_bytes(frame_path.read_bytes())

    binding = module._validate_outcome_still(
        tmp_path,
        capture_id=capture_id,
        timeline=timeline,
    )
    assert binding["sha256"] == timeline["frames"][str(step)]["sha256"]
    assert binding["timeline_frame_sha256"] == binding["sha256"]

    still.write_bytes(b"tampered")
    with pytest.raises(ValueError, match="differs from timeline"):
        module._validate_outcome_still(
            tmp_path,
            capture_id=capture_id,
            timeline=timeline,
        )
    still.unlink()
    with pytest.raises(ValueError, match="differs from timeline"):
        module._validate_outcome_still(
            tmp_path,
            capture_id=capture_id,
            timeline=timeline,
        )


def _capture_summary(module):
    return {
        "schema_version": module.RUNNER_SCHEMA_VERSION,
        "kind": "capture_run",
        "results": [
            {"schedule": {"id": capture_id}} for capture_id in module.CAPTURE_IDS
        ],
        "group_outputs": [
            {
                "group_id": module.GROUP_ID,
                "member_order": list(module.CAPTURE_IDS),
                "labels": list(module.GROUP_LABELS),
                "layout": "2x2",
            }
        ],
        "group_skips": [],
        "failures": [],
        "pass": True,
    }


def _write_outcome_stills(module, bundle):
    bindings = []
    for index, capture_id in enumerate(module.CAPTURE_IDS):
        step = module.GROUP_PANEL_STEP_BY_CAPTURE[capture_id]
        frame = bundle / capture_id / "frames" / f"step_{step:06d}.png"
        frame.parent.mkdir(parents=True, exist_ok=True)
        frame.write_bytes(f"outcome-{index}".encode())
        timeline = {
            "frames": {
                str(step): {
                    "path": str(frame),
                    "sha256": module.sha256(frame),
                }
            }
        }
        module.write_json(
            bundle / capture_id / "metadata.json",
            {"timeline_validation": timeline},
        )
        still = bundle / module.OUTCOME_STILL_PATH_BY_CAPTURE[capture_id]
        still.parent.mkdir(parents=True, exist_ok=True)
        still.write_bytes(frame.read_bytes())
        bindings.append(
            module._validate_outcome_still(
                bundle,
                capture_id=capture_id,
                timeline=timeline,
            )
        )
    return bindings


def test_capture_run_shape_requires_exact_source_order():
    module = _load_module()
    payload = _capture_summary(module)

    module._validate_capture_run_shape(payload)

    swapped = copy.deepcopy(payload)
    swapped["results"][0], swapped["results"][1] = (
        swapped["results"][1],
        swapped["results"][0],
    )
    with pytest.raises(ValueError, match="source order"):
        module._validate_capture_run_shape(swapped)

    relabeled = copy.deepcopy(payload)
    relabeled["group_outputs"][0]["labels"][0] = "WRONG"
    with pytest.raises(ValueError, match="source-order contract"):
        module._validate_capture_run_shape(relabeled)


def test_capture_provenance_binds_command_logs_summary_runtime_and_sources(
    tmp_path, monkeypatch
):
    module = _load_module()
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    summary = _capture_summary(module)
    module.write_json(bundle / "run-summary.json", summary)
    (bundle / "capture.stdout.txt").write_text(
        "renderer log\n" + json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (bundle / "capture.stderr.txt").write_text("", encoding="utf-8")
    outcome_stills = _write_outcome_stills(module, bundle)
    runtime = {"demo_binary": {"path": "/demo", "sha256": "1" * 64}}
    sources = {"visual_runner": {"path": "/runner", "sha256": "2" * 64}}
    monkeypatch.setattr(module, "_capture_runtime_identity", lambda **kwargs: runtime)
    monkeypatch.setattr(module, "_capture_source_identity", lambda **kwargs: sources)
    python = runner = demo = ffmpeg = ffprobe = Path("/bin/true")
    payload = {
        "argv": module._capture_argv(python, runner, demo, bundle, ffmpeg, ffprobe),
        "cwd": str(module.ROOT),
        "returncode": 0,
        "run_summary_path": "run-summary.json",
        "run_summary_sha256": module.sha256(bundle / "run-summary.json"),
        "run_summary_validated": True,
        "capture_stdout_sha256": module.sha256(bundle / "capture.stdout.txt"),
        "capture_stdout_ends_with_run_summary": True,
        "capture_stderr_sha256": module.sha256(bundle / "capture.stderr.txt"),
        "verification_stderr_sha256": None,
        "outcome_stills": outcome_stills,
        "staging_pruned": False,
        "runtime_resources_before": runtime,
        "runtime_resources_after": runtime,
        "capture_sources_before": sources,
        "capture_sources_after": sources,
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

    tampered = copy.deepcopy(payload)
    tampered["capture_sources_before"]["visual_runner"]["sha256"] = "0" * 64
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

    (bundle / "capture.stdout.txt").write_text("{}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="transitional capture logs changed"):
        module._validate_capture_provenance(
            payload,
            bundle=bundle,
            python=python,
            runner=runner,
            demo=demo,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
        )


@pytest.mark.parametrize(
    "resource_key",
    ["author_turntable_visual_obj", "author_turntable_visual_mtl"],
)
def test_capture_reuse_rejects_each_changed_renderer_resource(
    tmp_path, monkeypatch, resource_key
):
    module = _load_module()
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    summary = _capture_summary(module)
    module.write_json(bundle / "run-summary.json", summary)
    (bundle / "capture.stdout.txt").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (bundle / "capture.stderr.txt").write_text("", encoding="utf-8")
    outcome_stills = _write_outcome_stills(module, bundle)
    runtime = {"demo_binary": {"path": "/demo", "sha256": "1" * 64}}
    stored_sources = {
        "visual_runner": {"path": "/runner", "sha256": "2" * 64},
        "author_turntable_visual_obj": {
            "relative_path": "data/obj/fbf_author_turntable_disc.obj",
            "size_bytes": 1,
            "sha256": "3" * 64,
        },
        "author_turntable_visual_mtl": {
            "relative_path": "data/obj/fbf_author_turntable_disc.mtl",
            "size_bytes": 1,
            "sha256": "4" * 64,
        },
    }
    current_sources = copy.deepcopy(stored_sources)
    current_sources[resource_key]["sha256"] = "0" * 64
    monkeypatch.setattr(module, "_capture_runtime_identity", lambda **kwargs: runtime)
    monkeypatch.setattr(
        module, "_capture_source_identity", lambda **kwargs: current_sources
    )
    python = runner = demo = ffmpeg = ffprobe = Path("/bin/true")
    payload = {
        "argv": module._capture_argv(python, runner, demo, bundle, ffmpeg, ffprobe),
        "cwd": str(module.ROOT),
        "returncode": 0,
        "run_summary_path": "run-summary.json",
        "run_summary_sha256": module.sha256(bundle / "run-summary.json"),
        "run_summary_validated": True,
        "capture_stdout_sha256": module.sha256(bundle / "capture.stdout.txt"),
        "capture_stdout_ends_with_run_summary": True,
        "capture_stderr_sha256": module.sha256(bundle / "capture.stderr.txt"),
        "verification_stderr_sha256": None,
        "outcome_stills": outcome_stills,
        "staging_pruned": False,
        "runtime_resources_before": runtime,
        "runtime_resources_after": runtime,
        "capture_sources_before": stored_sources,
        "capture_sources_after": stored_sources,
    }

    with pytest.raises(ValueError, match="capture-time provenance changed"):
        module._validate_capture_provenance(
            payload,
            bundle=bundle,
            python=python,
            runner=runner,
            demo=demo,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
        )


def test_trace_matrix_provenance_binds_binary_and_both_collision_sources(
    monkeypatch,
):
    module = _load_module()
    runtime = {"trace_binary": {"path": "/trace", "sha256": "1" * 64}}
    sources = {
        "trace_source": {"path": "/trace.cpp", "sha256": "2" * 64},
        "dart_collision_source": {"path": "/dart.cpp", "sha256": "3" * 64},
        "native_collision_source": {"path": "/native.cpp", "sha256": "4" * 64},
    }
    monkeypatch.setattr(module, "_trace_runtime_identity", lambda **kwargs: runtime)
    monkeypatch.setattr(module, "_trace_source_identity", lambda: sources)
    payload = {
        "cwd": str(module.ROOT),
        "runtime_resources_before": runtime,
        "runtime_resources_after": runtime,
        "trace_sources_before": sources,
        "trace_sources_after": sources,
    }

    assert (
        module._validate_trace_matrix_provenance(payload, trace_binary=Path("/trace"))
        == payload
    )

    tampered = copy.deepcopy(payload)
    tampered["trace_sources_after"]["native_collision_source"]["sha256"] = "0" * 64
    with pytest.raises(ValueError, match="trace-matrix provenance changed"):
        module._validate_trace_matrix_provenance(tampered, trace_binary=Path("/trace"))


def test_capture_solver_diagnostics_reject_negative_residual():
    module = _load_module()
    diagnostics = {
        "available": True,
        "exact_attempts": 360,
        "exact_solves": 360,
        "accepted_at_cap": 0,
        "exact_failures": 0,
        "boxed_lcp_fallbacks": 0,
        "status": "success",
        "worst_residual": 9.0e-7,
    }

    assert (
        module._validate_final_solver_diagnostics(
            diagnostics, capture_id="turntable_author_mu02_omega2"
        )
        == diagnostics
    )
    diagnostics["worst_residual"] = -1.0e-7
    with pytest.raises(ValueError, match="solver diagnostics changed"):
        module._validate_final_solver_diagnostics(
            diagnostics, capture_id="turntable_author_mu02_omega2"
        )


def test_artifact_index_requires_exact_membership_and_no_symlinks(tmp_path):
    module = _load_module()
    (tmp_path / "nested").mkdir()
    (tmp_path / "a.txt").write_text("a", encoding="utf-8")
    (tmp_path / "nested/b.txt").write_text("b", encoding="utf-8")
    index = module.artifact_index(tmp_path)
    module.validate_artifact_index(tmp_path, index)

    (tmp_path / "unexpected.txt").write_text("unexpected", encoding="utf-8")
    with pytest.raises(ValueError, match="membership changed"):
        module.validate_artifact_index(tmp_path, index)

    link_root = tmp_path / "links"
    link_root.mkdir()
    target = link_root / "target.txt"
    target.write_text("target", encoding="utf-8")
    (link_root / "link.txt").symlink_to(target)
    with pytest.raises(ValueError, match="symlink"):
        module.artifact_index(link_root)


def test_bundle_membership_rejects_unknown_files_and_symlinks(tmp_path):
    module = _load_module()
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    module._validate_bundle_paths(bundle, complete=False, require_capture=False)

    (bundle / "unexpected.txt").write_text("bad", encoding="utf-8")
    with pytest.raises(ValueError, match="unexpected entries"):
        module._validate_bundle_paths(bundle, complete=False, require_capture=False)
    (bundle / "unexpected.txt").unlink()

    target = tmp_path / "target.txt"
    target.write_text("target", encoding="utf-8")
    (bundle / "capture.stdout.txt").symlink_to(target)
    with pytest.raises(ValueError, match="symlink"):
        module._validate_bundle_paths(bundle, complete=False, require_capture=False)


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


def test_sealed_membership_excludes_staging_and_requires_durable_stills(tmp_path):
    module = _load_module()
    bundle = tmp_path / "bundle"
    for relative in module.EXPECTED_FINAL_PATHS:
        path = bundle / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(relative.encode())

    module._validate_bundle_paths(bundle, complete=True, require_capture=True)

    staging = bundle / module.CAPTURE_IDS[0] / "frames" / "step_000000.png"
    staging.parent.mkdir(parents=True)
    staging.write_bytes(b"ignored staging")
    with pytest.raises(ValueError, match="unexpected entries"):
        module._validate_bundle_paths(bundle, complete=True, require_capture=True)
    staging.unlink()
    staging.parent.rmdir()

    still = bundle / module.OUTCOME_STILL_PATH_BY_CAPTURE[module.CAPTURE_IDS[0]]
    still.unlink()
    with pytest.raises(ValueError, match="capture bundle is incomplete"):
        module._validate_bundle_paths(bundle, complete=True, require_capture=True)


def test_pruning_removes_only_disposable_turntable_staging(tmp_path):
    module = _load_module()
    durable = tmp_path / module.OUTCOME_STILL_PATH_BY_CAPTURE[module.CAPTURE_IDS[0]]
    durable.parent.mkdir(parents=True)
    durable.write_bytes(b"durable")
    for relative in (
        "capture.stdout.txt",
        "capture.stderr.txt",
        "verification.stderr.txt",
        f"{module.CAPTURE_IDS[0]}/video_frames.ffconcat",
        f"{module.CAPTURE_IDS[0]}/frames/step_000000.png",
        f"{module.CAPTURE_IDS[0]}/panel_frames/step_000000.png",
    ):
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"staging")

    module._prune_capture_staging(tmp_path)

    assert durable.read_bytes() == b"durable"
    assert not any((tmp_path / relative).exists() for relative in module.STAGING_PATHS)


def test_reuse_failure_restores_previous_bundle_bytes(tmp_path, monkeypatch):
    module = _load_module()
    bundle = tmp_path / "bundle"
    old_files = {
        "capture-provenance.json": b'{"staging_pruned": false}\n',
        "metadata.json": b"old metadata\n",
        "artifact-index.json": b"old index\n",
        "REPORT.md": b"old report\n",
        "traces/current_visual/turntable_author_mu_0_2_omega_2.csv": b"old trace\n",
    }
    for relative, contents in old_files.items():
        path = bundle / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(contents)

    monkeypatch.setattr(module, "_validate_bundle_paths", lambda *args, **kwargs: None)
    args = argparse.Namespace(
        bundle=bundle,
        reuse_current_capture=True,
        trace_binary=tmp_path / "missing-trace-binary",
    )

    with pytest.raises(ValueError, match="trace binary is not a regular file"):
        module.finalize(args)

    restored = {
        path.relative_to(bundle).as_posix(): path.read_bytes()
        for path in bundle.rglob("*")
        if path.is_file()
    }
    assert restored == old_files
    assert not list(tmp_path.glob(".bundle.backup-*"))


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


def test_fresh_capture_preserves_intentional_pending_manual_state(
    tmp_path, monkeypatch
):
    module = _load_module()
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    (bundle / "REPORT.md").write_bytes(b"stale report\n")
    run_summary = {"kind": "capture_run", "pass": True}

    monkeypatch.setattr(module, "_validate_bundle_paths", lambda *args, **kwargs: None)
    monkeypatch.setattr(
        module, "_require_file", lambda path, label: Path(path).resolve()
    )
    monkeypatch.setattr(
        module,
        "_validate_capture_source_contract",
        lambda *args, **kwargs: {"pass": True},
    )
    monkeypatch.setattr(
        module, "_capture_runtime_identity", lambda *args, **kwargs: {"runtime": "same"}
    )
    monkeypatch.setattr(
        module, "_capture_source_identity", lambda *args, **kwargs: {"source": "same"}
    )
    monkeypatch.setattr(module, "_capture_argv", lambda *args, **kwargs: ["capture"])
    monkeypatch.setattr(module, "_validate_capture_run_shape", lambda payload: None)
    monkeypatch.setattr(module, "_promote_outcome_stills", lambda path: [])
    monkeypatch.setattr(
        module, "_validate_capture_provenance", lambda *args, **kwargs: None
    )

    def run_capture(argv, *, timeout):
        module.write_json(bundle / "run-summary.json", run_summary)
        stdout = json.dumps(run_summary, indent=2, sort_keys=True) + "\n"
        return argparse.Namespace(returncode=0, stdout=stdout, stderr="")

    monkeypatch.setattr(module, "_run_command", run_capture)
    args = argparse.Namespace(
        bundle=bundle,
        reuse_current_capture=False,
        trace_binary=tmp_path / "trace",
        demo=tmp_path / "demo",
        runner=tmp_path / "runner",
        ffmpeg=tmp_path / "ffmpeg",
        ffprobe=tmp_path / "ffprobe",
        python=tmp_path / "python",
        capture_timeout=1.0,
    )

    with pytest.raises(ValueError, match="manually inspect"):
        module.finalize(args)

    assert not (bundle / "REPORT.md").exists()
    assert module.read_json(bundle / "run-summary.json") == run_summary
    provenance = module.read_json(bundle / "capture-provenance.json")
    assert provenance["staging_pruned"] is False
    assert (bundle / "capture.stdout.txt").is_file()
    assert not list(tmp_path.glob(".bundle.backup-*"))


def test_current_source_contract_binds_four_cell_layout(monkeypatch):
    module = _load_module()
    matrix = _physics_contract_matrix(module)
    monkeypatch.setattr(
        module,
        "_query_author_turntable_physics_contracts",
        lambda **kwargs: copy.deepcopy(matrix),
    )

    contract = module._validate_capture_source_contract(
        demo=SCRIPT,
        trace_binary=SCRIPT,
        python=module.DEFAULT_PYTHON,
    )

    assert contract["pass"] is True
    assert contract["member_order"] == list(module.CAPTURE_IDS)
    assert contract["labels"] == list(module.GROUP_LABELS)
    assert contract["layout"] == "2x2"
    assert contract["panel_steps"] == list(module.GROUP_PANEL_STEPS)
    assert contract["panel_labels"] == list(module.GROUP_PANEL_LABELS)
    assert "panel_step" not in contract
    assert contract["support_geometry"] == (
        "author_pinned_cylinder_radius_2_height_0.1"
    )
    assert contract["source_support_geometry"] == "cylinder_radius_2_height_0.1"
    assert contract["cube_edge_length_m"] == 0.3
    assert contract["cube_density_kg_m3"] == 500.0
    assert contract["ramp_duration_seconds"] == 0.5
    assert contract["capture_duration_seconds"] == 6.0
    assert contract["capture_collision_frontend"] == "native_four_point_planar"
    assert contract["collision_override_applied_after_scene_install"] is False
    assert contract["author_numerical_source_available"] is True
    assert contract["author_turntable_renderer_assets_available"] is True
    assert contract["physics_contract_visual_asset_identity"] is None
    assert contract["renderer_resources"]["top_wedge_count"] == 8
    assert (
        contract["renderer_resources"]["registration_material"] == "FbfTurntableCoral"
    )
    assert contract["visual_attachment"]["visual_aspect_only"] is True
    assert contract["runner_group_contract"]["panel_steps"] == list(
        module.GROUP_PANEL_STEPS
    )
    assert contract["physics_contract_queries"] == matrix
    assert contract["demo_query_source_sha256"] == module.sha256(
        module.DEMO_MAIN_SOURCE
    )
    assert contract["trace_query_source_sha256"] == module.sha256(module.TRACE_SOURCE)
    assert contract["author_source"]["commit"] == module.AUTHOR_COMMIT
    assert contract["author_source"]["scene_source_sha256"]["turntable/run.py"] == (
        module.AUTHOR_TURNTABLE_SOURCE_SHA256
    )


def test_current_source_contract_rejects_queried_parameter_swap(monkeypatch):
    module = _load_module()
    matrix = _physics_contract_matrix(module)
    matrix["demo"]["turntable_author_mu_0_2_omega_5"]["scenario"][
        "angular_velocity_rad_s"
    ] = 2.0
    monkeypatch.setattr(
        module,
        "_query_author_turntable_physics_contracts",
        lambda **kwargs: matrix,
    )

    with pytest.raises(ValueError, match="physics scenario contract changed"):
        module._validate_capture_source_contract(
            demo=SCRIPT,
            trace_binary=SCRIPT,
            python=module.DEFAULT_PYTHON,
        )


def test_author_turntable_capture_accepts_and_revalidates_v2_provenance():
    module = _load_module()
    scenario = next(iter(module.SCENARIOS))
    scene = module.SCENARIOS[scenario]["scene"]
    capture_id = module.SCENARIOS[scenario]["capture_id"]
    contract = _physics_contract_matrix(module)["demo"][scenario]
    binding = module._expected_scene_physics_provenance(
        contract,
        demo=SCRIPT,
        scene=scene,
    )

    module._validate_scene_physics_provenance(
        capture_schema_version=module.RUNNER_CAPTURE_RESULT_SCHEMA_VERSION,
        runtime={"scene_physics_provenance": binding},
        contract=contract,
        demo=SCRIPT,
        scene=scene,
        capture_id=capture_id,
    )
    module._validate_scene_physics_provenance(
        capture_schema_version=module.RUNNER_SCHEMA_VERSION,
        runtime={},
        contract=contract,
        demo=SCRIPT,
        scene=scene,
        capture_id=capture_id,
    )

    with pytest.raises(ValueError, match="semantic physics provenance changed"):
        module._validate_scene_physics_provenance(
            capture_schema_version=module.RUNNER_CAPTURE_RESULT_SCHEMA_VERSION,
            runtime={},
            contract=contract,
            demo=SCRIPT,
            scene=scene,
            capture_id=capture_id,
        )
    changed = copy.deepcopy(binding)
    changed["semantic_physics_sha256"] = "f" * 64
    with pytest.raises(ValueError, match="semantic physics provenance changed"):
        module._validate_scene_physics_provenance(
            capture_schema_version=module.RUNNER_CAPTURE_RESULT_SCHEMA_VERSION,
            runtime={"scene_physics_provenance": changed},
            contract=contract,
            demo=SCRIPT,
            scene=scene,
            capture_id=capture_id,
        )


def test_author_turntable_contract_is_deferred_and_solver_is_locked():
    module = _load_module()
    source = module.DEMO_SOURCE.read_text(encoding="utf-8")
    start = source.index("DemoScene makeFbfAuthorTurntableParameterizedScene(")
    end = source.index("DemoScene makeFbfAuthorCardHouseParameterizedScene(", start)
    author_scene = source[start:end]

    assert "setup.physicsContractProvider = [world, scenario]" in author_scene
    assert "Toggle exact/boxed" not in author_scene
    assert (
        "renderSolverControls(world, state, 4u, 4u, false, false, true, false);"
        in author_scene
    )


def test_demo_host_snapshots_runtime_contract_before_timeline_steps():
    module = _load_module()
    source = module.DEMO_HOST_SOURCE.read_text(encoding="utf-8")
    start = source.index("int DemoHost::runHeadlessTimeline(")
    end = source.index("int DemoHost::run()", start)
    timeline = source[start:end]

    prepare = timeline.index("prepareHeadlessRun(")
    snapshot = timeline.index("mCurrentPhysicsContractProvider()")
    first_step = timeline.index(
        "for (std::size_t step = 0u; step <= requestedSteps; ++step)"
    )
    assert prepare < snapshot < first_step
    assert timeline.count("mCurrentPhysicsContractProvider()") == 1


def test_runner_source_contract_matches_finalizer_panel_contract():
    module = _load_module()

    contract = module._query_runner_author_group_contract(
        python=module.DEFAULT_PYTHON,
        runner=module.DEFAULT_RUNNER,
        demo=SCRIPT,
    )

    assert contract["member_order"] == list(module.CAPTURE_IDS)
    assert contract["labels"] == list(module.GROUP_LABELS)
    assert contract["panel_steps"] == list(module.GROUP_PANEL_STEPS)
    assert contract["panel_labels"] == list(module.GROUP_PANEL_LABELS)
    assert module.sha256(module.AUTHOR_TURNTABLE_SPEC_SOURCE) == (
        _physics_contract_matrix(module)["demo"][next(iter(module.SCENARIOS))][
            "physics_spec_source_sha256"
        ]
    )


def test_runner_source_contract_rejects_panel_label_drift(monkeypatch):
    module = _load_module()
    panel_sources = [
        {
            "member": capture_id,
            "step": step,
            "time_seconds": step * module.DT_SECONDS,
        }
        for capture_id, step in zip(module.CAPTURE_IDS, module.GROUP_PANEL_STEPS)
    ]
    plan = {
        "schema_version": module.RUNNER_SCHEMA_VERSION,
        "kind": "capture_plan",
        "pass": True,
        "schedules": [{"id": capture_id} for capture_id in module.CAPTURE_IDS],
        "group_outputs": {
            module.GROUP_ID: {
                "members": list(module.CAPTURE_IDS),
                "labels": list(module.GROUP_LABELS),
                "layout": "2x2 in source order",
                "panel_sources": panel_sources,
                "panel_labels": ["WRONG", *module.GROUP_PANEL_LABELS[1:]],
            }
        },
    }
    monkeypatch.setattr(module, "_query_json_command", lambda *args, **kwargs: plan)

    with pytest.raises(ValueError, match="visual-runner group contract changed"):
        module._query_runner_author_group_contract(
            python=SCRIPT,
            runner=SCRIPT,
            demo=SCRIPT,
        )


def test_demo_and_trace_contract_queries_use_dedicated_cli(monkeypatch):
    module = _load_module()
    calls = []

    def fake_query(argv, *, label):
        calls.append((list(argv), label))
        return {}

    monkeypatch.setattr(module, "_query_json_command", fake_query)
    scenario = next(iter(module.SCENARIOS))

    module._query_demo_author_turntable_physics_contract(SCRIPT, scenario=scenario)
    module._query_trace_author_turntable_physics_contract(
        SCRIPT,
        scenario=scenario,
        solver_contract="paper_cpu",
    )

    assert calls[0][0] == [
        str(SCRIPT),
        "--fbf-author-turntable-contract",
        module.SCENARIOS[scenario]["scene"],
    ]
    assert calls[1][0] == [
        str(SCRIPT),
        "--author-turntable-contract",
        scenario,
        "paper_cpu",
    ]


@pytest.mark.parametrize(
    ("mutation", "match"),
    [
        ("visual_asset", "shared physics identity changed"),
        ("source_hash", "source-to-binary binding changed"),
    ],
)
def test_physics_contract_fails_closed_on_identity_mutation(mutation, match):
    module = _load_module()
    scenario = next(iter(module.SCENARIOS))
    contract = _physics_contract(
        module,
        scenario,
        solver_contract="dart_best",
        binary_role="dart_demos",
    )
    if mutation == "visual_asset":
        contract["visual_asset_identity"] = {"sha256": "0" * 64}
    else:
        contract["binary_binding"]["implementation_source_sha256"] = "0" * 64

    with pytest.raises(ValueError, match=match):
        module._validate_author_turntable_physics_contract(
            contract,
            scenario=scenario,
            solver_contract="dart_best",
            binary_role="dart_demos",
            implementation_source=module.DEMO_SOURCE,
        )


def test_capture_sidecar_physics_contract_matches_live_demo_query(
    tmp_path, monkeypatch
):
    module = _load_module()
    scenario = next(iter(module.SCENARIOS))
    contract = _physics_contract(
        module,
        scenario,
        solver_contract="dart_best",
        binary_role="dart_demos",
    )
    sidecar = tmp_path / "timeline.json"
    module.write_json(sidecar, {"physics_contract": contract})
    monkeypatch.setattr(
        module,
        "_query_demo_author_turntable_physics_contract",
        lambda *args, **kwargs: copy.deepcopy(contract),
    )

    assert (
        module._validate_capture_sidecar_physics_contract(
            sidecar, demo=SCRIPT, scenario=scenario
        )
        == contract
    )

    live_drift = copy.deepcopy(contract)
    live_drift["world"]["gravity_m_s2"][2] -= 1.0e-13
    monkeypatch.setattr(
        module,
        "_query_demo_author_turntable_physics_contract",
        lambda *args, **kwargs: live_drift,
    )
    with pytest.raises(ValueError, match="timeline physics contract changed"):
        module._validate_capture_sidecar_physics_contract(
            sidecar, demo=SCRIPT, scenario=scenario
        )


def test_turntable_visual_resources_prove_segmented_registration_wedge():
    module = _load_module()

    contract = module._validate_turntable_visual_resources()

    assert contract["pass"] is True
    assert contract["top_wedge_count"] == 8
    assert contract["top_triangles_per_wedge"] == 8
    assert contract["top_material_order"] == [
        "FbfTurntableCoral",
        "FbfTurntableLight",
        "FbfTurntableDark",
        "FbfTurntableLight",
        "FbfTurntableDark",
        "FbfTurntableLight",
        "FbfTurntableDark",
        "FbfTurntableLight",
    ]
    assert contract["material_diffuse_rgb"]["FbfTurntableCoral"] == [
        0.94,
        0.31,
        0.27,
    ]


@pytest.mark.parametrize("resource", ["obj", "mtl"])
def test_turntable_visual_resources_reject_obj_and_mtl_mutations(
    tmp_path, monkeypatch, resource
):
    module = _load_module()
    if resource == "obj":
        mutated = tmp_path / module.AUTHOR_TURNTABLE_VISUAL_OBJ.name
        mutated.write_text(
            module.AUTHOR_TURNTABLE_VISUAL_OBJ.read_text(encoding="utf-8").replace(
                "usemtl FbfTurntableCoral",
                "usemtl FbfTurntableDark",
                1,
            ),
            encoding="utf-8",
        )
        monkeypatch.setattr(module, "AUTHOR_TURNTABLE_VISUAL_OBJ", mutated)
        match = "material sectors changed"
    else:
        mutated = tmp_path / module.AUTHOR_TURNTABLE_VISUAL_MTL.name
        mutated.write_text(
            module.AUTHOR_TURNTABLE_VISUAL_MTL.read_text(encoding="utf-8").replace(
                "Kd 0.940000 0.310000 0.270000",
                "Kd 0.390000 0.420000 0.450000",
                1,
            ),
            encoding="utf-8",
        )
        monkeypatch.setattr(module, "AUTHOR_TURNTABLE_VISUAL_MTL", mutated)
        match = "diffuse colors changed"

    with pytest.raises(ValueError, match=match):
        module._validate_turntable_visual_resources()


def test_turntable_visual_attachment_rejects_collision_aspect(tmp_path, monkeypatch):
    module = _load_module()
    mutated = tmp_path / module.DEMO_SOURCE.name
    mutated.write_text(
        module.DEMO_SOURCE.read_text(encoding="utf-8").replace(
            "createShapeNodeWith<VisualAspect>(visualShape)",
            "createShapeNodeWith<VisualAspect, CollisionAspect>(visualShape)",
            1,
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(module, "DEMO_SOURCE", mutated)

    with pytest.raises(ValueError, match="resource attachment contract changed"):
        module._validate_turntable_visual_attachment()


def test_group_media_command_freezes_input_and_label_order(tmp_path):
    module = _load_module()
    command = module._expected_group_media_command(tmp_path, Path("/ffmpeg"))

    assert (
        module._validate_group_media_command(
            command, root=tmp_path, ffmpeg=Path("/ffmpeg")
        )
        == command
    )
    swapped = list(command)
    swapped[6], swapped[8] = swapped[8], swapped[6]
    with pytest.raises(ValueError, match="source/label order changed"):
        module._validate_group_media_command(
            swapped, root=tmp_path, ffmpeg=Path("/ffmpeg")
        )


def test_report_discloses_lane_separation_and_nonclaims():
    module = _load_module()
    parsed = _parsed_matrix(
        module, strict_negative_scenario="turntable_author_mu_0_5_omega_5"
    )
    capture_projections = {
        scenario: copy.deepcopy(parsed["current_visual"][scenario]["solver_projection"])
        for scenario in module.SCENARIOS
    }
    capture_projections["turntable_author_mu_0_2_omega_5"][97]["warm_starts"] -= 1
    summary = module._build_trace_summary(
        parsed, {"capture_solver_projections": capture_projections}
    )
    capture = {
        "group": {
            "width": 1320,
            "height": 1060,
            "frame_rate": "30/1",
            "frame_count": 181,
        }
    }

    report = module._report_markdown(summary, capture)

    assert "source-ordered 2x2 matrix" in report
    assert "dart_best contract" in report
    assert "paper_cpu/Native traces are retained in a separate directory" in report
    assert "cannot replace a current_visual trace" in report
    assert "Full six-field projection byte identity holds" in report
    assert "3 of four cells" in report
    assert "warm_starts" in report
    assert "not Warp/Newton trace parity" in report
    assert "retained on the support through 6 s" in report
    assert "not a zero-slip or co-rotation claim" in report
    assert "captured mu=.5" not in report


def test_json_reader_and_payload_hash_reject_nonfinite_numbers(tmp_path):
    module = _load_module()
    path = tmp_path / "bad.json"
    path.write_text('{"value": NaN}\n', encoding="utf-8")

    with pytest.raises(ValueError, match="non-finite JSON number"):
        module.read_json(path)
    with pytest.raises(ValueError, match="Out of range float"):
        module._payload_sha256({"value": float("nan")})


def test_verify_only_and_reuse_are_mutually_exclusive(capsys):
    module = _load_module()

    assert module.main(["--verify-only", "--reuse-current-capture"]) == 2
    assert "mutually exclusive" in capsys.readouterr().err
