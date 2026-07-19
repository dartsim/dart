import hashlib
import importlib.util
import json
import math
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "run_fbf_cpu_evidence.py"
TRACE_SOURCE = ROOT / "tests" / "benchmark" / "integration" / "fbf_paper_trace.cpp"
FIXTURE_SOURCE = (
    ROOT / "tests" / "integration" / "test_ExactCoulombFbfPaperFixtures.cpp"
)
COLORED_TEST_CPUS = (4, 6, 8, 10)


def _load_module():
    spec = importlib.util.spec_from_file_location("run_fbf_cpu_evidence", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _row(module, *, step, wall_ms, repetition="1", status="success"):
    row = {column: "0" for column in module.TRACE_COLUMNS}
    row.update(
        {
            "step": str(step),
            "time": str(step / 60.0),
            "scenario": "backspin",
            "solver": "exact_fbf",
            "solver_contract": "paper_cpu",
            "precision_contract": "float64_vs_paper_float32",
            "scene_contract": "paper_parameters_with_dart_contact_frontend",
            "baumgarte_contract": "dart_velocity_baumgarte_author_parameters_unavailable",
            "collision_frontend": "dart",
            "inner_local_solver": "exact_metric",
            "inner_sweeps_requested": "10",
            "fixed_inner_sweeps_requested": "1",
            "step_size_persistence_enabled": "1",
            "step_size_recovery_growth_factor": "1.05",
            "step_size_persistence_used": "1",
            "step_size_persistence_request": "0.2",
            "row_operator_request": "contact_row_no_dense_snapshot",
            "row_operator_mode": "contact_row_no_dense_snapshot",
            "max_outer_iterations": "200",
            "tolerance": "1e-6",
            "accept_outer_max_iterations": "1",
            "inner_local_iterations": "1",
            "adaptive_step_size_enabled": "1",
            "warm_start_enabled": "1",
            "projected_gradient_retry_enabled": "0",
            "dense_residual_polish_enabled": "0",
            "fallback_to_boxed_lcp_enabled": "0",
            "diagonal_seed_enabled": "0",
            "matrix_free_seed_enabled": "0",
            "step_size_scale": "1",
            "outer_relaxation": "1",
            "initial_gamma_contract": "automatic_safe_bound",
            "split_impulse_enabled": "0",
            "exact_contact_row_logical_cpus_to_date": "none",
            "max_phase_contact_row_logical_cpus_to_date": "none",
            "max_card_center_displacement_from_initial": "nan",
            "min_card_orientation_alignment_from_initial": "nan",
            "projectile_card_contacts": "0",
            "wall_ms": str(wall_ms),
            "requested_threads": "1",
            "actual_threads": "1",
            "contacts": "1",
            "unique_colliding_body_pairs": "1",
            "penetration_depth_min": "0.001",
            "penetration_depth_median": "0.001",
            "penetration_depth_p95": "0.001",
            "penetration_depth_max": "0.001",
            "exact_diagnostics_contract": "last_exact_group_public_getters_warm_fraction_over_step_contacts",
            "step_exact_solves": "1",
            "step_exact_failures": "0" if status == "success" else "1",
            "step_fallbacks": "0",
            "step_fbf_iterations": "10",
            "residual": "1e-7",
            "status": status,
            "residual_primal_feasibility": "1e-8",
            "residual_dual_feasibility": "2e-8",
            "residual_complementarity": "3e-8",
            "accepted_gamma": "0.25",
            "safe_gamma": "0.2",
            "shrink_iterations": "2",
            "coupling_variation_ratio": "0.4",
            "warm_start_matched_contacts": "1",
            "warm_start_matched_fraction": "1",
            "finite_state": "1",
            "repetition": repetition,
            "process_returncode": "0",
            "warmup": "0",
            "affinity_source": "explicit_taskset",
            "affinity_logical_cpus": "4",
            "affinity_logical_cpu_count": "1",
            "affinity_physical_core_count": "1",
            "affinity_one_logical_per_physical_core": "true",
            "affinity_physical_core_keys": "0:4",
            "affinity_logical_cpu_physical_core_keys": "4=0:4",
            "affinity_package_ids": "0",
            "affinity_package_count": "1",
            "affinity_smt_sibling_counts": "2",
            "affinity_core_classes_khz": "4000000",
            "affinity_scaling_governors": "performance",
            "paper_hardware_contract": "linux_x86_64_vs_paper_apple_silicon",
        }
    )
    return row


def _physical_trajectory_rows(module, scenario, *, wall_ms=2.0):
    steps = module.SCENARIO_TRAJECTORY_CONTRACT[scenario][0]
    rows = []
    for step in range(1, steps + 1):
        row = _row(module, step=step, wall_ms=wall_ms)
        row.update(
            {
                "scenario": scenario,
                "tracked_body": module.PHYSICAL_OUTCOME_TRACKED_BODIES[scenario],
            }
        )
        rows.append(row)

    final = rows[-1]
    if scenario.startswith("incline_mu_"):
        initial_x, initial_z, downhill_x, downhill_z = module._incline_expected_state()
        displacement = (
            0.0
            if scenario == "incline_mu_0_5"
            else module._incline_sliding_displacement()
        )
        final.update(
            {
                "x": str(initial_x + downhill_x * displacement),
                "z": str(initial_z + downhill_z * displacement),
            }
        )
    elif scenario == "backspin":
        final.update(
            {
                "vx": str(module._backspin_expected_linear_velocity()),
                "z": "0.25",
                "contacts": "1",
            }
        )
    elif scenario.startswith("turntable_"):
        final.update(
            {
                "x": ("1.0" if scenario == "turntable_mu_0_5_omega_2" else "2.0"),
                "y": "0.0",
                "z": "0.1",
                "vx": "0.0",
                "vy": ("2.0" if scenario == "turntable_mu_0_5_omega_2" else "0.0"),
                "contacts": "1",
            }
        )
    elif scenario == "card_house_26_settle_projectile_full":
        for row in rows:
            in_settle = int(row["step"]) <= module.CARD_HOUSE_FULL_SETTLE_STEPS
            after_projectile_contact = int(row["step"]) >= 450
            row.update(
                {
                    "phase": "settle" if in_settle else "projectile",
                    "card_count": str(module.CARD_HOUSE_FULL_CARD_COUNT),
                    "projectile_count": (
                        "0"
                        if in_settle
                        else str(module.CARD_HOUSE_FULL_PROJECTILE_COUNT)
                    ),
                    "min_card_axis_up": "0.0",
                    "min_center_height": "0.48",
                    "max_card_horizontal_travel": "1.62",
                    "max_projectile_speed": "0.0" if in_settle else "4.0",
                    "max_card_center_displacement_from_initial": (
                        "0.01" if not after_projectile_contact else "0.30"
                    ),
                    "min_card_orientation_alignment_from_initial": (
                        "0.99" if not after_projectile_contact else "0.40"
                    ),
                    "projectile_card_contacts": (
                        "1" if int(row["step"]) == 450 else "0"
                    ),
                    "contacts": "155",
                }
            )
        final.update(
            {
                "min_card_axis_up": "-0.75",
                "min_center_height": "0.30",
                "max_card_horizontal_travel": "1.95",
            }
        )
    elif scenario == module.LITERAL_WEDGE_ARCH_SCENARIO:
        for index, row in enumerate(rows):
            row.update(
                {
                    "scene_contract": module.LITERAL_WEDGE_ARCH_SCENE_CONTRACT,
                    "collision_frontend": "native",
                    "split_impulse_enabled": "1",
                    "contacts": str(module.LITERAL_WEDGE_ARCH_CONTACT_COUNT),
                    "unique_colliding_body_pairs": str(
                        module.LITERAL_WEDGE_ARCH_COLLIDING_BODY_PAIR_COUNT
                    ),
                    "x": str(2.8e-5 * index / max(len(rows) - 1, 1)),
                    "y": "0",
                    "z": "2.5",
                    "up_z": "0.999999998",
                    "max_arch_body_displacement_from_initial": (
                        str(2.8e-5 * (index + 1) / len(rows))
                    ),
                    "min_arch_body_orientation_alignment_from_initial": ("0.999999998"),
                }
            )
    elif scenario == "painleve_mu_0_5":
        final.update({"up_z": "0.9", "z": "0.4"})
    else:
        final.update({"up_z": "0.5", "z": "0.4"})
    return rows


def _set_affinity(
    rows,
    cpus,
    *,
    packages=None,
    cores=None,
    core_classes_khz=None,
    governors=None,
    smt_sibling_counts=None,
):
    packages = packages or [0] * len(cpus)
    cores = cores or list(cpus)
    core_classes_khz = core_classes_khz or [4000000] * len(cpus)
    governors = governors or ["performance"] * len(cpus)
    smt_sibling_counts = smt_sibling_counts or [2] * len(cpus)
    physical_keys = sorted(
        {f"{package}:{core}" for package, core in zip(packages, cores)}
    )
    for row in rows:
        row.update(
            {
                "affinity_logical_cpus": ",".join(str(cpu) for cpu in cpus),
                "affinity_logical_cpu_count": str(len(cpus)),
                "affinity_physical_core_count": str(len(physical_keys)),
                "affinity_one_logical_per_physical_core": str(
                    len(physical_keys) == len(cpus)
                ).lower(),
                "affinity_physical_core_keys": ";".join(physical_keys),
                "affinity_logical_cpu_physical_core_keys": ";".join(
                    f"{cpu}={package}:{core}"
                    for cpu, package, core in zip(cpus, packages, cores)
                ),
                "affinity_package_ids": ";".join(
                    str(value) for value in sorted(set(packages))
                ),
                "affinity_package_count": str(len(set(packages))),
                "affinity_smt_sibling_counts": ";".join(
                    str(value) for value in sorted(set(smt_sibling_counts))
                ),
                "affinity_core_classes_khz": ";".join(
                    str(value) for value in sorted(set(core_classes_khz))
                ),
                "affinity_scaling_governors": ";".join(sorted(set(governors))),
            }
        )


def _affinity_record(
    scenario,
    threads,
    cpus,
    *,
    frontend="native",
    packages=None,
    cores=None,
    core_classes_khz=None,
    governors=None,
    smt_sibling_counts=None,
    expected_rows=600,
    repetition=1,
    warmup=False,
    returncode=0,
    complete_rows=True,
):
    packages = packages or [0] * len(cpus)
    cores = cores or list(cpus)
    core_classes_khz = core_classes_khz or [4000000] * len(cpus)
    governors = governors or ["performance"] * len(cpus)
    smt_sibling_counts = smt_sibling_counts or [2] * len(cpus)
    mapping = {
        str(cpu): f"{package}:{core}"
        for cpu, package, core in zip(cpus, packages, cores)
    }
    physical_keys = sorted(set(mapping.values()))
    return {
        "scenario": scenario,
        "collision_frontend": frontend,
        "threads": threads,
        "repetition": repetition,
        "warmup": warmup,
        "returncode": returncode,
        "complete_rows": complete_rows,
        "expected_rows": expected_rows,
        "paper_hardware_contract": "linux_x86_64_vs_paper_apple_silicon",
        "cpu_affinity": {
            "source": "explicit_taskset",
            "logical_cpus": list(cpus),
            "logical_cpu_count": len(cpus),
            "physical_core_count": len(physical_keys),
            "one_logical_per_physical_core": len(physical_keys) == len(cpus),
            "logical_cpu_physical_core_keys": mapping,
            "physical_core_keys": physical_keys,
            "package_ids": sorted(set(packages)),
            "package_count": len(set(packages)),
            "smt_sibling_counts": sorted(set(smt_sibling_counts)),
            "core_classes_khz": sorted(set(core_classes_khz)),
            "scaling_governors": sorted(set(governors)),
        },
    }


def _card_scaling_evidence(
    module,
    *,
    baseline_cpu=4,
    candidate_cpus=(4, 6, 8, 10),
    candidate_packages=None,
    candidate_cores=None,
    candidate_core_classes_khz=None,
    candidate_governors=None,
    candidate_smt_sibling_counts=None,
):
    scenario = "card_house_26_settle_projectile_full"
    one_rows = _physical_trajectory_rows(module, scenario, wall_ms=4.0)
    candidate_rows = _physical_trajectory_rows(module, scenario, wall_ms=2.0)
    for row in one_rows:
        row.update(
            {
                "solver_contract": "dart_best",
                "collision_frontend": "native",
                "step_contact_row_delassus_products": "10",
                "max_contact_row_participants_to_date": "1",
            }
        )
    candidate_cpu_text = ";".join(str(cpu) for cpu in candidate_cpus)
    for row in candidate_rows:
        row.update(
            {
                "requested_threads": "4",
                "actual_threads": "4",
                "solver_contract": (
                    "dart_best_threaded_world_exact_contact_row_parallel_capable"
                ),
                "collision_frontend": "native",
                "step_contact_row_delassus_products": "10",
                "step_parallel_contact_row_delassus_products": "10",
                "max_contact_row_participants_to_date": "4",
                "exact_contact_row_logical_cpus_to_date": candidate_cpu_text,
                "max_phase_contact_row_logical_cpus_to_date": candidate_cpu_text,
            }
        )
    _set_affinity(one_rows, [baseline_cpu])
    _set_affinity(
        candidate_rows,
        list(candidate_cpus),
        packages=candidate_packages,
        cores=candidate_cores,
        core_classes_khz=candidate_core_classes_khz,
        governors=candidate_governors,
        smt_sibling_counts=candidate_smt_sibling_counts,
    )
    records = [
        _affinity_record(scenario, 1, [baseline_cpu]),
        _affinity_record(
            scenario,
            4,
            list(candidate_cpus),
            packages=candidate_packages,
            cores=candidate_cores,
            core_classes_khz=candidate_core_classes_khz,
            governors=candidate_governors,
            smt_sibling_counts=candidate_smt_sibling_counts,
        ),
    ]
    return one_rows, candidate_rows, records


def test_parse_case_and_threads():
    module = _load_module()

    assert module._parse_case("backspin:300") == module.Case("backspin", 300)
    assert module._parse_case("masonry_arch_25_literal_wedge:600") == module.Case(
        "masonry_arch_25_literal_wedge", 600
    )
    assert module._parse_threads("1,4,4,8") == (1, 4, 8)
    with pytest.raises(Exception):
        module._parse_case("backspin")
    with pytest.raises(Exception):
        module._parse_threads("1,0")
    assert module._parse_cpu_list_for("4:4,6,8,10") == (4, "4,6,8,10")
    with pytest.raises(Exception):
        module._parse_cpu_list_for("4:four")


@pytest.mark.parametrize("value", ["nan", "inf", "-inf"])
def test_cli_rejects_nonfinite_timeout(value):
    module = _load_module()

    with pytest.raises(SystemExit):
        module._parse_args(
            [
                "--binary",
                "/tmp/fbf_paper_trace",
                "--output-dir",
                "/tmp/fbf-evidence",
                "--case",
                "backspin:1",
                "--timeout",
                value,
            ]
        )


@pytest.mark.parametrize("value", ["inf", "-inf", "NaN", "0", "-1", "text"])
def test_cli_rejects_invalid_explicit_initial_gamma(value):
    module = _load_module()

    with pytest.raises(SystemExit):
        module._parse_args(
            [
                "--binary",
                "/tmp/fbf_paper_trace",
                "--output-dir",
                "/tmp/fbf-evidence",
                "--case",
                "backspin:1",
                "--initial-gamma",
                value,
            ]
        )

    assert module._parse_initial_gamma("nan") == "nan"
    assert module._parse_initial_gamma("0.125") == "0.125"


@pytest.mark.parametrize("value", ["../../escape:1", "/tmp/escape:1"])
def test_parse_case_rejects_traversal_and_absolute_scenarios(value):
    module = _load_module()

    with pytest.raises(Exception, match="canonical"):
        module._parse_case(value)


def test_slug_and_raw_output_paths_are_confined(tmp_path):
    module = _load_module()
    invocation = module.Invocation(module.Case("../../escape", 1), 1, 1, False)

    slug = module._slug(invocation)
    assert "/" not in slug
    assert ".." not in slug
    with pytest.raises(ValueError, match="must be relative"):
        module._raw_output_path(tmp_path, "/tmp/escape.csv")
    with pytest.raises(ValueError, match="escapes output directory"):
        module._raw_output_path(tmp_path, "../escape.csv")


def test_all_published_cpu_timing_targets_are_recorded():
    module = _load_module()

    expected = {
        "backspin": 6.0,
        "incline_mu_0_5": 5.5,
        "incline_mu_0_4": 5.3,
        "painleve_mu_0_5": 7.0,
        "painleve_mu_0_55": 6.4,
        "turntable_mu_0_5_omega_2": 6.8,
        "turntable_mu_0_5_omega_5": 3.1,
        "turntable_mu_0_2_omega_2": 3.7,
        "turntable_mu_0_2_omega_5": 3.1,
        "card_house_26_reduced_contact": 199.0,
        "masonry_arch_25_reduced_contact": 595.0,
        "masonry_arch_101_reduced_contact": 1234.0,
        "card_house_10_level": 853.0,
    }
    for scenario, target_ms in expected.items():
        assert module.PAPER_REFERENCE_MS[scenario] == target_ms

    assert module.PAPER_CONVERGENCE_REFERENCE["backspin"][1:] == (25.0, 30.0)
    assert module.PAPER_CONVERGENCE_REFERENCE["turntable_mu_0_2_omega_5"][1:] == (
        0.0,
        10.0,
    )


def test_parse_trace_rejects_schema_drift():
    module = _load_module()
    values = {column: "0" for column in module.TRACE_COLUMNS}
    values.update({"step": "1", "requested_threads": "1", "actual_threads": "1"})
    valid = (
        ",".join(module.TRACE_COLUMNS)
        + "\n"
        + ",".join(values[column] for column in module.TRACE_COLUMNS)
    )

    assert len(module._parse_trace(valid)) == 1
    with pytest.raises(ValueError, match="unexpected"):
        module._parse_trace("step,wall_ms\n1,2")
    assert module.SCHEMA_VERSION == 8
    assert len(module.TRACE_COLUMNS_V4) == 60
    assert module.TRACE_COLUMNS[:60] == module.TRACE_COLUMNS_V4
    assert module.TRACE_COLUMNS[-8:-5] == module.TRACE_PARALLEL_COLUMNS_V6
    assert module.TRACE_COLUMNS[-5:] == module.TRACE_RESIDENCY_AND_CARD_COLUMNS_V7
    assert len(module.TRACE_COLUMNS) == 83
    assert len(module.TRACE_COLORED_COLUMNS) == 95


def test_static_trace_default_header_matches_v7_parser_exactly():
    module = _load_module()
    source = TRACE_SOURCE.read_text(encoding="utf-8")
    function = re.search(
        r"void printPerformanceHeader\(SolverContract contract\)\s*\{\s*"
        r"std::printf\((.*?)\);\s*if \(contract",
        source,
        re.DOTALL,
    )
    assert function is not None
    literals = re.findall(r'"((?:\\.|[^"\\])*)"', function.group(1))
    header = bytes("".join(literals), "utf-8").decode("unicode_escape").strip()

    assert tuple(header.split(",")) == module.TRACE_COLUMNS
    assert hashlib.sha256(header.encode()).hexdigest() == (
        "7fbad44f7d610d1e80c2b64f1d28bff0ef7cf918a50ca307ac208a5e5a1acab9"
    )
    assert hashlib.sha256((header + "\n").encode()).hexdigest() == (
        "396c866d782e626b5c5b52e74c392de1175f5e46dfd90b809d1f219417092e50"
    )


def test_static_trace_colored_extension_matches_v8_parser_exactly():
    module = _load_module()
    source = TRACE_SOURCE.read_text(encoding="utf-8")
    function = re.search(
        r"if \(contract == SolverContract::DartBestColoredBgs\)\s*\{\s*"
        r"std::printf\((.*?)\);\s*\}",
        source,
        re.DOTALL,
    )
    assert function is not None
    literals = re.findall(r'"((?:\\.|[^"\\])*)"', function.group(1))
    extension = bytes("".join(literals), "utf-8").decode("unicode_escape")

    assert extension.startswith(",")
    assert tuple(extension[1:].split(",")) == module.TRACE_COLORED_BGS_COLUMNS_V8
    assert module.TRACE_COLORED_COLUMNS == (
        *module.TRACE_COLUMNS,
        *module.TRACE_COLORED_BGS_COLUMNS_V8,
    )
    colored_header = ",".join(module.TRACE_COLORED_COLUMNS)
    assert hashlib.sha256((colored_header + "\n").encode()).hexdigest() == (
        "424195336cb42753179130c9f6fcba3a8ddea9bf669cfa70c0f77fcb4c6335a5"
    )


def _colored_row(module, *, threads=4):
    row = _row(module, step=1, wall_ms=2.0)
    row.update({column: "0" for column in module.TRACE_COLORED_BGS_COLUMNS_V8})
    row.update(
        {
            "solver_contract": (
                "dart_best_nonpaper_colored_inner_bgs"
                if threads == 1
                else "dart_best_nonpaper_colored_inner_bgs_threaded_world"
            ),
            "inner_sweeps_requested": "120",
            "fixed_inner_sweeps_requested": "0",
            "diagonal_seed_enabled": "1",
            "matrix_free_seed_enabled": "1",
            "requested_threads": str(threads),
            "actual_threads": str(threads),
            "inner_bgs_schedule_contract": (
                "dart_deterministic_manifold_colored_bgs_nonpaper"
            ),
            "last_exact_colored_bgs_used": "1",
            "last_exact_colored_bgs_solves": "10",
            "last_exact_colored_bgs_dispatches": "0" if threads == 1 else "1",
            "last_exact_colored_bgs_max_participants": str(threads),
            "last_exact_colored_bgs_manifolds": "24",
            "last_exact_colored_bgs_colors": "3",
            "last_exact_colored_bgs_max_manifolds_per_color": "8",
            "exact_colored_bgs_logical_cpus": (
                "none"
                if threads == 1
                else ";".join(map(str, COLORED_TEST_CPUS[:threads]))
            ),
            "max_phase_exact_colored_bgs_logical_cpus": (
                "none"
                if threads == 1
                else ";".join(map(str, COLORED_TEST_CPUS[:threads]))
            ),
            "max_arch_body_displacement_from_initial": "nan",
            "min_arch_body_orientation_alignment_from_initial": "nan",
        }
    )
    return row


def _literal_colored_row(module, *, threads=1):
    row = _colored_row(module, threads=threads)
    row.update(
        {
            "scenario": module.LITERAL_WEDGE_ARCH_SCENARIO,
            "scene_contract": module.LITERAL_WEDGE_ARCH_SCENE_CONTRACT,
            "collision_frontend": "native",
            "inner_local_solver": "exact_metric",
            "inner_sweeps_requested": "30",
            "fixed_inner_sweeps_requested": "1",
            "step_size_persistence_enabled": "0",
            "step_size_persistence_used": "0",
            "step_size_persistence_request": "nan",
            "max_outer_iterations": "5000",
            "inner_local_iterations": "1",
            "adaptive_step_size_enabled": "1",
            "warm_start_enabled": "1",
            "diagonal_seed_enabled": "0",
            "matrix_free_seed_enabled": "0",
            "step_size_scale": "35",
            "outer_relaxation": "1.1",
            "initial_gamma_contract": "automatic_safe_bound",
            "split_impulse_enabled": "1",
            "max_arch_body_displacement_from_initial": "2.8e-5",
            "min_arch_body_orientation_alignment_from_initial": "0.999999998",
        }
    )
    return row


def _literal_colored_trajectory_rows(module, *, threads, wall_ms):
    rows = _physical_trajectory_rows(
        module, module.LITERAL_WEDGE_ARCH_SCENARIO, wall_ms=wall_ms
    )
    template = _literal_colored_row(module, threads=threads)
    option_fields = (
        "solver_contract",
        "requested_threads",
        "actual_threads",
        "inner_local_solver",
        "inner_sweeps_requested",
        "fixed_inner_sweeps_requested",
        "step_size_persistence_enabled",
        "step_size_persistence_used",
        "step_size_persistence_request",
        "max_outer_iterations",
        "inner_local_iterations",
        "adaptive_step_size_enabled",
        "warm_start_enabled",
        "projected_gradient_retry_enabled",
        "dense_residual_polish_enabled",
        "fallback_to_boxed_lcp_enabled",
        "diagonal_seed_enabled",
        "matrix_free_seed_enabled",
        "step_size_scale",
        "outer_relaxation",
        "initial_gamma_contract",
    )
    arch_metric_fields = {
        "max_arch_body_displacement_from_initial",
        "min_arch_body_orientation_alignment_from_initial",
    }
    for row in rows:
        row.update({field: template[field] for field in option_fields})
        row.update(
            {
                field: template[field]
                for field in module.TRACE_COLORED_BGS_COLUMNS_V8
                if field not in arch_metric_fields
            }
        )
    cpus = COLORED_TEST_CPUS[:threads]
    _set_affinity(rows, list(cpus))
    return rows


def test_colored_trace_schema_is_opt_in_and_cross_rejected():
    module = _load_module()
    row = _colored_row(module)
    text = (
        ",".join(module.TRACE_COLORED_COLUMNS)
        + "\n"
        + ",".join(row[column] for column in module.TRACE_COLORED_COLUMNS)
    )

    assert len(module._parse_trace(text, "dart_best_colored_bgs")) == 1
    with pytest.raises(ValueError, match="unexpected"):
        module._parse_trace(text)


def test_colored_contract_cli_requires_exact_solver_and_forwards_contract():
    module = _load_module()
    base = [
        "--binary",
        "/tmp/fbf_paper_trace",
        "--output-dir",
        "/tmp/fbf-evidence",
        "--case",
        "backspin:1",
        "--contract",
        "dart_best_colored_bgs",
    ]
    args = module._parse_args([*base, "--threads", "1,4"])
    invocation = module.Invocation(module.Case("backspin", 1), 4, 1, False)

    assert "dart_best_colored_bgs" in module._command(args, invocation)
    with pytest.raises(SystemExit):
        module._parse_args([*base, "--solver", "boxed_lcp"])


def test_literal_wedge_cli_requires_native_collision_frontend():
    module = _load_module()
    base = [
        "--binary",
        "/tmp/fbf_paper_trace",
        "--output-dir",
        "/tmp/fbf-evidence",
        "--case",
        "masonry_arch_25_literal_wedge:600",
        "--contract",
        "dart_best_colored_bgs",
    ]

    with pytest.raises(SystemExit):
        module._parse_args(base)
    args = module._parse_args([*base, "--collision-frontend", "native"])

    assert args.case == [module.Case(module.LITERAL_WEDGE_ARCH_SCENARIO, 600)]
    assert args.collision_frontend == "native"


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("last_exact_colored_bgs_used", "0", "unused colored path"),
        (
            "last_exact_colored_bgs_dispatches",
            "0",
            "persistent-dispatch count",
        ),
        (
            "last_exact_colored_bgs_dispatches",
            "2",
            "persistent-dispatch count",
        ),
        (
            "last_exact_colored_bgs_max_participants",
            "3",
            "participants do not match",
        ),
        (
            "exact_colored_bgs_logical_cpus",
            "4;6;8;10;12",
            "logical CPUs exceed",
        ),
    ],
)
def test_colored_trace_counter_invariants(field, value, message):
    module = _load_module()
    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "backspin:1",
            "--contract",
            "dart_best_colored_bgs",
            "--threads",
            "4",
            "--cpu-list",
            "4,6,8,10,12",
        ]
    )
    invocation = module.Invocation(module.Case("backspin", 1), 4, 1, False)
    row = _colored_row(module)
    row[field] = value

    with pytest.raises(ValueError, match=message):
        module._validate_colored_bgs_rows([row], args, invocation)


def test_colored_trace_accepts_one_thread_baseline_and_four_thread_dispatch():
    module = _load_module()
    for threads in (1, 4):
        args = module._parse_args(
            [
                "--binary",
                "/tmp/fbf_paper_trace",
                "--output-dir",
                "/tmp/fbf-evidence",
                "--case",
                "backspin:1",
                "--contract",
                "dart_best_colored_bgs",
                "--threads",
                str(threads),
                "--cpu-list",
                "4" if threads == 1 else "4,6,8,10",
            ]
        )
        invocation = module.Invocation(module.Case("backspin", 1), threads, 1, False)
        module._validate_invocation_rows(
            [_colored_row(module, threads=threads)], args, invocation
        )


def test_colored_trace_rejects_one_thread_or_ineligible_dispatch():
    module = _load_module()
    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "backspin:1",
            "--contract",
            "dart_best_colored_bgs",
            "--threads",
            "1",
            "--cpu-list",
            "4",
        ]
    )
    invocation = module.Invocation(module.Case("backspin", 1), 1, 1, False)
    one_thread = _colored_row(module, threads=1)
    one_thread["last_exact_colored_bgs_dispatches"] = "1"

    with pytest.raises(ValueError, match="one-thread colored baseline"):
        module._validate_colored_bgs_rows([one_thread], args, invocation)

    args.threads = (4,)
    args.cpu_list = "4,6,8,10"
    invocation = module.Invocation(module.Case("backspin", 1), 4, 1, False)
    ineligible = _colored_row(module, threads=4)
    ineligible.update(
        {
            "step_exact_solves": "0",
            "step_exact_failures": "0",
            "last_exact_colored_bgs_used": "-1",
            "last_exact_colored_bgs_solves": "0",
            "last_exact_colored_bgs_dispatches": "1",
            "last_exact_colored_bgs_max_participants": "0",
            "last_exact_colored_bgs_manifolds": "0",
            "last_exact_colored_bgs_colors": "0",
            "last_exact_colored_bgs_max_manifolds_per_color": "0",
            "exact_colored_bgs_logical_cpus": "none",
            "max_phase_exact_colored_bgs_logical_cpus": "none",
        }
    )
    with pytest.raises(ValueError, match="unavailable unless exactly one exact group"):
        module._validate_colored_bgs_rows([ineligible], args, invocation)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        (
            "step_parallel_contact_row_delassus_products",
            "1",
            "legacy parallel contact-row work",
        ),
        (
            "max_contact_row_participants_to_date",
            "2",
            "multiple legacy contact-row participants",
        ),
        (
            "exact_contact_row_logical_cpus_to_date",
            "4",
            "legacy contact-row CPU residency",
        ),
        (
            "max_phase_contact_row_logical_cpus_to_date",
            "4",
            "legacy contact-row CPU residency",
        ),
    ],
)
def test_colored_trace_rejects_legacy_contact_row_parallel_evidence(
    field, value, message
):
    module = _load_module()
    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "backspin:1",
            "--contract",
            "dart_best_colored_bgs",
            "--threads",
            "4",
            "--cpu-list",
            "4,6,8,10",
        ]
    )
    invocation = module.Invocation(module.Case("backspin", 1), 4, 1, False)
    row = _colored_row(module)
    row[field] = value

    with pytest.raises(ValueError, match=message):
        module._validate_colored_bgs_rows([row], args, invocation)


@pytest.mark.parametrize("threads", [1, 4])
def test_colored_trace_accepts_eligible_schedule_without_zero_iteration_work(threads):
    module = _load_module()
    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "masonry_arch_25_literal_wedge:1",
            "--contract",
            "dart_best_colored_bgs",
            "--collision-frontend",
            "native",
            "--threads",
            str(threads),
            "--cpu-list",
            "4" if threads == 1 else "4,6,8,10",
        ]
    )
    invocation = module.Invocation(
        module.Case(module.LITERAL_WEDGE_ARCH_SCENARIO, 1), threads, 1, False
    )
    row = _literal_colored_row(module, threads=threads)
    row.update(
        {
            "step_fbf_iterations": "0",
            "last_exact_colored_bgs_used": "0",
            "last_exact_colored_bgs_solves": "0",
            "last_exact_colored_bgs_dispatches": "0" if threads == 1 else "1",
            "last_exact_colored_bgs_max_participants": (
                "0" if threads == 1 else str(threads)
            ),
            "exact_colored_bgs_logical_cpus": (
                "none"
                if threads == 1
                else ";".join(map(str, COLORED_TEST_CPUS[:threads]))
            ),
            "max_phase_exact_colored_bgs_logical_cpus": (
                "none"
                if threads == 1
                else ";".join(map(str, COLORED_TEST_CPUS[:threads]))
            ),
        }
    )

    module._validate_invocation_rows([row], args, invocation)

    row["step_fbf_iterations"] = "1"
    with pytest.raises(ValueError, match="positive-iteration exact attempt"):
        module._validate_invocation_rows([row], args, invocation)


@pytest.mark.parametrize("dispatches", ["0", "2"])
def test_colored_zero_iteration_eligible_multithread_attempt_dispatches_once(
    dispatches,
):
    module = _load_module()
    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "masonry_arch_25_literal_wedge:1",
            "--contract",
            "dart_best_colored_bgs",
            "--collision-frontend",
            "native",
            "--threads",
            "4",
            "--cpu-list",
            "4,6,8,10",
        ]
    )
    invocation = module.Invocation(
        module.Case(module.LITERAL_WEDGE_ARCH_SCENARIO, 1), 4, 1, False
    )
    row = _literal_colored_row(module, threads=4)
    row.update(
        {
            "step_fbf_iterations": "0",
            "last_exact_colored_bgs_used": "0",
            "last_exact_colored_bgs_solves": "0",
            "last_exact_colored_bgs_dispatches": dispatches,
        }
    )

    with pytest.raises(ValueError, match="zero-iteration colored dispatch"):
        module._validate_colored_bgs_rows([row], args, invocation)


def test_literal_zero_iteration_exact_attempt_rejects_missing_colored_schedule():
    module = _load_module()
    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "masonry_arch_25_literal_wedge:1",
            "--contract",
            "dart_best_colored_bgs",
            "--collision-frontend",
            "native",
            "--threads",
            "4",
            "--cpu-list",
            "4,6,8,10",
        ]
    )
    invocation = module.Invocation(
        module.Case(module.LITERAL_WEDGE_ARCH_SCENARIO, 1), 4, 1, False
    )
    row = _literal_colored_row(module, threads=4)
    row.update(
        {
            "step_fbf_iterations": "0",
            "last_exact_colored_bgs_used": "0",
            "last_exact_colored_bgs_solves": "0",
            "last_exact_colored_bgs_dispatches": "0",
            "last_exact_colored_bgs_max_participants": "0",
            "last_exact_colored_bgs_manifolds": "0",
            "last_exact_colored_bgs_colors": "0",
            "last_exact_colored_bgs_max_manifolds_per_color": "0",
            "exact_colored_bgs_logical_cpus": "none",
            "max_phase_exact_colored_bgs_logical_cpus": "none",
        }
    )

    with pytest.raises(ValueError, match="retain its 24-manifold colored schedule"):
        module._validate_invocation_rows([row], args, invocation)


def test_colored_trace_rejects_any_positive_work_row_with_narrow_schedule():
    module = _load_module()
    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "backspin:2",
            "--contract",
            "dart_best_colored_bgs",
            "--threads",
            "4",
            "--cpu-list",
            "4,6,8,10",
        ]
    )
    invocation = module.Invocation(module.Case("backspin", 2), 4, 1, False)
    wide = _colored_row(module)
    narrow = _colored_row(module)
    narrow["step"] = "2"
    narrow["last_exact_colored_bgs_max_manifolds_per_color"] = "3"

    with pytest.raises(ValueError, match="width is below"):
        module._validate_invocation_rows([wide, narrow], args, invocation)


@pytest.mark.parametrize(
    ("field", "invalid"),
    [
        ("scene_contract", "source_derived_box_arch"),
        ("max_outer_iterations", "4999"),
        ("inner_sweeps_requested", "120"),
        ("fixed_inner_sweeps_requested", "0"),
        ("inner_local_solver", "projected_gradient"),
        ("inner_local_iterations", "2"),
        ("adaptive_step_size_enabled", "0"),
        ("warm_start_enabled", "0"),
        ("step_size_persistence_enabled", "1"),
        ("projected_gradient_retry_enabled", "1"),
        ("dense_residual_polish_enabled", "1"),
        ("fallback_to_boxed_lcp_enabled", "1"),
        ("diagonal_seed_enabled", "1"),
        ("matrix_free_seed_enabled", "1"),
        ("tolerance", "2e-6"),
        ("step_size_scale", "34"),
        ("outer_relaxation", "1"),
        ("initial_gamma_contract", "explicit_capped_safe_bound"),
        ("split_impulse_enabled", "0"),
    ],
)
def test_literal_wedge_colored_contract_validates_tuned_exact_options(field, invalid):
    module = _load_module()
    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "masonry_arch_25_literal_wedge:1",
            "--contract",
            "dart_best_colored_bgs",
            "--collision-frontend",
            "native",
            "--cpu-list",
            "4",
        ]
    )
    invocation = module.Invocation(
        module.Case(module.LITERAL_WEDGE_ARCH_SCENARIO, 1), 1, 1, False
    )
    row = _literal_colored_row(module)

    module._validate_invocation_rows([row], args, invocation)
    row[field] = invalid

    with pytest.raises(ValueError, match=field.replace("_", ".*")):
        module._validate_invocation_rows([row], args, invocation)


def test_literal_wedge_colored_contract_rejects_disabled_persistence_use():
    module = _load_module()
    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "masonry_arch_25_literal_wedge:1",
            "--contract",
            "dart_best_colored_bgs",
            "--collision-frontend",
            "native",
            "--cpu-list",
            "4",
        ]
    )
    invocation = module.Invocation(
        module.Case(module.LITERAL_WEDGE_ARCH_SCENARIO, 1), 1, 1, False
    )
    row = _literal_colored_row(module)
    row.update(
        {
            "step_size_persistence_used": "1",
            "step_size_persistence_request": "0.2",
        }
    )

    with pytest.raises(ValueError, match="disabled step-size persistence"):
        module._validate_invocation_rows([row], args, invocation)


@pytest.mark.parametrize(
    "value",
    ["", "None", "1,2", "01", "1;01", "2;1", "1;1", "-1", "1;", " 1"],
)
@pytest.mark.parametrize(
    "field",
    [
        "exact_contact_row_logical_cpus_to_date",
        "max_phase_contact_row_logical_cpus_to_date",
    ],
)
def test_runtime_cpu_residency_ids_use_strict_sorted_unique_grammar(field, value):
    module = _load_module()
    row = _row(module, step=1, wall_ms=2.0)
    row[field] = value

    with pytest.raises(ValueError, match="logical CPU residency field"):
        module._validate_logical_cpu_residency_fields([row])


def test_runtime_cpu_residency_accepts_none_or_sorted_unique_nonnegative_ids():
    module = _load_module()
    row = _row(module, step=1, wall_ms=2.0)

    for value in ("none", "0", "0;2;17"):
        row["exact_contact_row_logical_cpus_to_date"] = value
        row["max_phase_contact_row_logical_cpus_to_date"] = value
        module._validate_logical_cpu_residency_fields([row])


def test_static_trace_cli_guards_negative_unsigned_and_extra_arguments():
    source = TRACE_SOURCE.read_text(encoding="utf-8")
    parse_size = re.search(r"bool parseSizeArg\(.*?\n\}\n\n", source, re.DOTALL)
    assert parse_size is not None
    assert "value[0] == '-'" in parse_size.group(0)
    assert "parsed > std::numeric_limits<std::size_t>::max()" in parse_size.group(0)
    assert "if (argc > 16)" in source
    assert "solverMode != SolverMode::ExactFbf" in source
    assert "simulationThreads != 1u" in source
    assert "collisionFrontend != CollisionFrontend::Native" in source


def test_physical_outcome_thresholds_match_paper_fixture_assertions():
    module = _load_module()
    source = FIXTURE_SOURCE.read_text(encoding="utf-8")

    assert "constexpr double kInclineTan = 0.5;" in source
    assert "EXPECT_NEAR(exactStick.displacement, 0.0, 2e-2)" in source
    assert (
        "EXPECT_NEAR(exactSlide.displacement, analyticalSlidingDisplacement(0.4), 0.2)"
        in source
    )
    assert "EXPECT_EQ(exactStick.contactSteps, exactStick.trajectorySteps);" in source
    assert "EXPECT_EQ(exactSlide.contactSteps, exactSlide.trajectorySteps);" in source
    assert "EXPECT_LE(exactStick.maxPenetration, 0.02);" in source
    assert "EXPECT_LE(exactSlide.maxPenetration, 0.02);" in source
    assert "EXPECT_NEAR(exact.linearVelocity, expectedLinearVelocity, 0.5)" in source
    assert "EXPECT_NEAR(exact.height, kBackspinRadius, 3e-2)" in source
    assert "result.radius < 1.25 && result.height > 0.05" in source
    assert "EXPECT_LE(std::abs(exact.radialVelocity), 0.2);" in source
    assert "EXPECT_LE(exact.tangentialCoRotationError, 0.05)" in source
    assert "return result.radius > 1.75 || result.height < 0.0;" in source
    assert "result.uprightness > 0.85 && result.height > 0.35" in source
    assert "result.uprightness < 0.55 || result.height < 0.35" in source
    assert module.INCLINE_STICK_DISPLACEMENT_TOLERANCE == 2e-2
    assert module.INCLINE_MAX_PENETRATION == 2e-2
    assert module.TURNTABLE_CAPTURE_RADIUS_MAX == 1.25
    assert module.TURNTABLE_CAPTURE_HEIGHT_MIN == 0.05
    assert module.TURNTABLE_CAPTURE_RADIAL_VELOCITY_MAX == 0.2
    assert module.TURNTABLE_CAPTURE_COROTATION_SPEED_ERROR_MAX == 0.05
    assert module.TURNTABLE_EJECTION_RADIUS_MIN == 1.75
    assert module.PAINLEVE_UPRIGHT_UP_Z_MIN == 0.85
    assert module.PAINLEVE_TUMBLED_UP_Z_MAX == 0.55
    assert module.PAINLEVE_HEIGHT_THRESHOLD == 0.35


def test_percentile_interpolates():
    module = _load_module()

    assert module._percentile([1.0, 2.0, 3.0], 0.5) == 2.0
    assert math.isclose(module._percentile([1.0, 2.0], 0.95), 1.95)
    assert math.isnan(module._percentile([], 0.95))


def test_summary_reports_raw_speed_without_claiming_paper_parity():
    module = _load_module()
    rows = [
        _row(module, step=1, wall_ms=2.0),
        _row(module, step=2, wall_ms=4.0),
    ]

    summary = module._summarize_group(rows, failed_processes=0)

    assert summary["mean_step_ms"] == 3.0
    assert summary["p95_step_ms"] == pytest.approx(3.9)
    assert summary["realtime_factor"] == pytest.approx((1000.0 / 60.0) / 3.0)
    assert summary["raw_mean_below_realtime"] is True
    assert summary["mean_realtime_target_met"] is None
    assert "not_full_scenario_trajectory" in summary["realtime_contract_reasons"]
    assert summary["paper_timing_comparable"] is False
    assert summary["paper_target_evaluated"] is False
    assert summary["paper_ratio_to_reference"] is None
    assert summary["paper_mean_target_met"] is None
    assert "precision_mismatch" in summary["paper_comparison_note"]
    assert summary["all_solver_steps_successful"] is True


def test_summary_marks_failed_solver_step():
    module = _load_module()
    rows = [_row(module, step=1, wall_ms=2.0, status="fbf_failed")]

    summary = module._summarize_group(rows, failed_processes=1)

    assert summary["exact_failures"] == 1
    assert summary["all_solver_steps_successful"] is False


@pytest.mark.parametrize(
    "invalid",
    [None, "corrupt", "0.5", "-1", str(1 << 64)],
    ids=("missing", "text", "fractional", "negative", "out-of-range"),
)
def test_summary_rejects_corrupt_step_fallbacks_instead_of_coercing_zero(invalid):
    module = _load_module()
    row = _row(module, step=1, wall_ms=2.0)
    if invalid is None:
        del row["step_fallbacks"]
    else:
        row["step_fallbacks"] = invalid

    with pytest.raises(ValueError, match="step_fallbacks"):
        module._summarize_group([row], failed_processes=0)


@pytest.mark.parametrize(
    "field",
    [
        "step",
        "requested_threads",
        "actual_threads",
        "contacts",
        "unique_colliding_body_pairs",
        "step_exact_solves",
        "step_warm_starts",
        "step_exact_failures",
        "step_fallbacks",
        "step_fbf_iterations",
        "card_count",
        "projectile_count",
        "finite_state",
    ],
)
def test_every_core_trace_counter_is_strictly_parsed(field):
    module = _load_module()
    row = _row(module, step=1, wall_ms=2.0)
    row[field] = "not-an-integer"

    with pytest.raises(ValueError, match=field):
        module._validate_integer_fields([row])


def test_contact_free_exact_step_is_not_a_solver_failure_or_residual_sample():
    module = _load_module()
    contact_row = _row(module, step=1, wall_ms=2.0)
    contact_free_row = _row(module, step=2, wall_ms=1.0, status="no_exact_group")
    contact_free_row.update(
        {
            "contacts": "0",
            "unique_colliding_body_pairs": "0",
            "exact_diagnostics_contract": "unavailable_no_exact_group_this_step",
            "step_exact_solves": "0",
            "step_exact_failures": "0",
            "step_fallbacks": "0",
            "step_fbf_iterations": "0",
            "residual": "nan",
            "residual_primal_feasibility": "nan",
            "residual_dual_feasibility": "nan",
            "residual_complementarity": "nan",
            "accepted_gamma": "nan",
            "safe_gamma": "nan",
            "shrink_iterations": "-1",
            "coupling_variation_ratio": "nan",
            "warm_start_matched_contacts": "-1",
            "warm_start_matched_fraction": "nan",
            "step_size_persistence_used": "-1",
            "step_size_persistence_request": "nan",
            "row_operator_mode": "not_run",
        }
    )

    summary = module._summarize_group(
        [contact_row, contact_free_row], failed_processes=0
    )

    assert summary["all_solver_steps_successful"] is True
    assert summary["residual_pass_fraction"] == 1.0
    assert summary["total_fbf_iterations"] == 10


def test_multi_group_last_diagnostics_are_excluded_from_residual_claims():
    module = _load_module()
    row = _row(module, step=1, wall_ms=2.0)
    row["exact_diagnostics_contract"] = (
        "last_exact_group_only_multi_group_noncomparable"
    )

    summary = module._summarize_group([row], failed_processes=0)

    assert math.isnan(summary["max_residual"])
    assert math.isnan(summary["residual_pass_fraction"])
    assert math.isnan(summary["total_shrink_iterations"])


def test_native_frontend_requires_explicit_exact_native_scene_contract():
    module = _load_module()
    row = _row(module, step=1, wall_ms=2.0)
    row.update(
        {
            "collision_frontend": "native",
            "precision_contract": "paper_float32",
            "scene_contract": "paper_exact",
            "baumgarte_contract": "paper_exact",
        }
    )

    summary = module._summarize_group([row], failed_processes=0)

    assert summary["paper_timing_comparable"] is False
    assert (
        "scene_or_frontend_mismatch:native:paper_exact"
        in summary["paper_comparison_note"]
    )

    row["scene_contract"] = "paper_exact_native_collision_frontend"
    comparison_note = module._summarize_group([row], 0)["paper_comparison_note"]
    assert "scene_or_frontend_mismatch" not in comparison_note
    assert "hardware_mismatch" in comparison_note


def test_summary_aggregates_collision_and_exact_diagnostics():
    module = _load_module()
    first = _row(module, step=1, wall_ms=2.0)
    second = _row(module, step=2, wall_ms=3.0)
    second.update(
        {
            "unique_colliding_body_pairs": "3",
            "penetration_depth_p95": "0.003",
            "penetration_depth_max": "0.004",
            "residual_primal_feasibility": "4e-8",
            "residual_dual_feasibility": "5e-8",
            "residual_complementarity": "6e-8",
            "accepted_gamma": "0.1",
            "safe_gamma": "0.4",
            "shrink_iterations": "5",
            "coupling_variation_ratio": "0.8",
            "warm_start_matched_fraction": "0.5",
        }
    )

    summary = module._summarize_group([first, second], failed_processes=0)

    assert set(summary) == set(module.SUMMARY_COLUMNS)
    assert summary["min_unique_colliding_body_pairs"] == 1
    assert summary["max_unique_colliding_body_pairs"] == 3
    assert summary["max_penetration_depth"] == pytest.approx(0.004)
    assert summary["max_residual_primal_feasibility"] == pytest.approx(4e-8)
    assert summary["min_accepted_gamma"] == pytest.approx(0.1)
    assert summary["max_safe_gamma"] == pytest.approx(0.4)
    assert summary["total_shrink_iterations"] == 7
    assert summary["max_coupling_variation_ratio"] == pytest.approx(0.8)
    assert summary["mean_warm_start_matched_fraction"] == pytest.approx(0.75)
    assert summary["inner_local_solver"] == "exact_metric"
    assert summary["fixed_inner_sweeps_requested"] == "1"
    assert summary["step_size_persistence_used_steps"] == 2
    assert summary["min_step_size_persistence_request"] == pytest.approx(0.2)
    assert summary["row_operator_request"] == "contact_row_no_dense_snapshot"
    assert summary["row_operator_modes"] == "contact_row_no_dense_snapshot"


def test_collision_frontend_cli_is_forwarded_to_trace():
    module = _load_module()
    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "backspin:1",
            "--collision-frontend",
            "native",
        ]
    )
    invocation = module.Invocation(module.Case("backspin", 1), 1, 1, False)

    assert args.collision_frontend == "native"
    assert module._command(args, invocation)[-2:] == ["native", "default"]


def test_local_solver_override_is_dart_best_only_and_forwarded_to_trace():
    module = _load_module()
    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "backspin:1",
            "--local-solver",
            "projected_gradient",
        ]
    )
    invocation = module.Invocation(module.Case("backspin", 1), 1, 1, False)

    assert args.local_solver == "projected_gradient"
    assert module._command(args, invocation)[-1] == "projected_gradient"

    with pytest.raises(SystemExit):
        module._parse_args(
            [
                "--binary",
                "/tmp/fbf_paper_trace",
                "--output-dir",
                "/tmp/fbf-evidence",
                "--case",
                "backspin:1",
                "--contract",
                "paper_cpu",
                "--collision-frontend",
                "native",
                "--local-solver",
                "inverse_euclidean",
            ]
        )
    with pytest.raises(SystemExit):
        module._parse_args(
            [
                "--binary",
                "/tmp/fbf_paper_trace",
                "--output-dir",
                "/tmp/fbf-evidence",
                "--case",
                "backspin:1",
                "--solver",
                "boxed_lcp",
                "--local-solver",
                "projected_gradient",
            ]
        )


def test_dart_best_default_local_solver_resolves_and_validates_exact_metric(
    monkeypatch,
):
    module = _load_module()
    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "backspin:1",
        ]
    )
    invocation = module.Invocation(module.Case("backspin", 1), 1, 1, False)
    row = _row(module, step=1, wall_ms=2.0)
    row.update(
        {
            "solver_contract": "dart_best",
            "inner_local_solver": "exact_metric",
            "inner_sweeps_requested": "120",
            "fixed_inner_sweeps_requested": "0",
            "row_operator_request": "contact_row_no_dense_snapshot",
            "row_operator_mode": "contact_row_no_dense_snapshot",
        }
    )

    assert module._resolved_local_solver(args) == "exact_metric"
    module._validate_invocation_rows([row], args, invocation)
    row["inner_local_solver"] = "inverse_euclidean"
    with pytest.raises(ValueError, match="inner_local_solver mismatch"):
        module._validate_invocation_rows([row], args, invocation)

    monkeypatch.setattr(module, "_sha256", lambda _: "binary-sha256")
    monkeypatch.setattr(module, "_cpu_metadata", lambda: {})
    monkeypatch.setattr(module, "_git_metadata", lambda _: {})
    monkeypatch.setattr(module, "_run_text", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(module, "_read_text", lambda _: None)
    monkeypatch.setattr(
        module,
        "_runtime_identity",
        lambda _binary: {"resolved_regular_files": {"libdart.so": {}}},
    )
    taskset_identity = {
        "name": "taskset",
        "path": "/tmp/taskset-alias",
        "resolved_path": "/opt/pinned/taskset",
        "size_bytes": 10,
        "sha256": "a" * 64,
    }
    args.executed_tool_closure = {"taskset": taskset_identity}
    args.executed_tool_identity_rechecks = ["after_backspin-n1-t1-rep001"]
    metadata = module._metadata(args, ROOT)
    assert metadata["configuration"]["resolved_local_solver"] == "exact_metric"
    assert metadata["source_identity"] == {
        "runner_path": "scripts/run_fbf_cpu_evidence.py",
        "runner_sha256": "binary-sha256",
        "trace_source_path": "tests/benchmark/integration/fbf_paper_trace.cpp",
        "trace_source_sha256": "binary-sha256",
    }
    assert metadata["runtime_identity"]["resolved_regular_files"]
    assert metadata["executed_tool_closure"] == {"taskset": taskset_identity}
    assert metadata["executed_tool_identity_rechecks"] == [
        "after_backspin-n1-t1-rep001"
    ]
    assert "contact-row kernel parallel dispatch" in metadata["threading_policy"]
    assert "lifetime CPU-ID union is audit-only" in metadata["threading_policy"]
    assert "not proof of perfect simultaneity" in metadata["threading_policy"]


@pytest.mark.parametrize(
    ("flag", "value"),
    [
        ("--initial-gamma", "0.1"),
        ("--warm-start", "0"),
        ("--split-impulse", "1"),
        ("--local-solver", "exact_metric"),
    ],
)
def test_paper_cpu_rejects_every_exposed_solver_override(flag, value):
    module = _load_module()

    with pytest.raises(SystemExit):
        module._parse_args(
            [
                "--binary",
                "/tmp/fbf_paper_trace",
                "--output-dir",
                "/tmp/fbf-evidence",
                "--case",
                "backspin:1",
                "--contract",
                "paper_cpu",
                "--collision-frontend",
                "native",
                flag,
                value,
            ]
        )


def test_paper_cpu_profile_is_fail_closed_to_exact_native_single_thread():
    module = _load_module()
    base = [
        "--binary",
        "/tmp/fbf_paper_trace",
        "--output-dir",
        "/tmp/fbf-evidence",
        "--case",
        "backspin:1",
        "--contract",
        "paper_cpu",
    ]

    accepted = module._parse_args(
        [*base, "--solver", "exact_fbf", "--collision-frontend", "native"]
    )
    assert accepted.solver == "exact_fbf"
    assert accepted.threads == (1,)
    assert accepted.collision_frontend == "native"

    with pytest.raises(SystemExit):
        module._parse_args([*base, "--collision-frontend", "dart"])
    with pytest.raises(SystemExit):
        module._parse_args([*base, "--collision-frontend", "native", "--threads", "2"])
    with pytest.raises(SystemExit):
        module._parse_args(
            [*base, "--collision-frontend", "native", "--solver", "boxed_lcp"]
        )


def test_invocation_validation_checks_solver_and_persistence_contract():
    module = _load_module()
    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "backspin:1",
            "--local-solver",
            "projected_gradient",
        ]
    )
    invocation = module.Invocation(module.Case("backspin", 1), 1, 1, False)
    row = _row(module, step=1, wall_ms=2.0)
    row.update(
        {
            "solver_contract": "dart_best",
            "inner_local_solver": "projected_gradient",
            "inner_sweeps_requested": "120",
            "fixed_inner_sweeps_requested": "0",
            "row_operator_request": "contact_row_no_dense_snapshot",
            "row_operator_mode": "contact_row_no_dense_snapshot",
        }
    )

    module._validate_invocation_rows([row], args, invocation)
    row["inner_local_solver"] = "inverse_euclidean"
    with pytest.raises(ValueError, match="inner_local_solver mismatch"):
        module._validate_invocation_rows([row], args, invocation)
    row["inner_local_solver"] = "projected_gradient"
    row["step_size_persistence_request"] = "nan"
    with pytest.raises(ValueError, match="without a positive finite request"):
        module._validate_invocation_rows([row], args, invocation)


@pytest.mark.parametrize(
    ("field", "invalid"),
    [
        ("max_outer_iterations", "199"),
        ("tolerance", "2e-6"),
        ("accept_outer_max_iterations", "0"),
        ("inner_local_iterations", "2"),
        ("adaptive_step_size_enabled", "0"),
        ("warm_start_enabled", "0"),
        ("projected_gradient_retry_enabled", "1"),
        ("dense_residual_polish_enabled", "1"),
        ("fallback_to_boxed_lcp_enabled", "1"),
        ("diagonal_seed_enabled", "1"),
        ("matrix_free_seed_enabled", "1"),
        ("step_size_scale", "2"),
        ("outer_relaxation", "1.5"),
        ("initial_gamma_contract", "explicit_capped_safe_bound"),
        ("split_impulse_enabled", "1"),
    ],
)
def test_paper_cpu_invocation_validates_every_resolved_knob(field, invalid):
    module = _load_module()
    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "backspin:1",
            "--contract",
            "paper_cpu",
            "--collision-frontend",
            "native",
        ]
    )
    invocation = module.Invocation(module.Case("backspin", 1), 1, 1, False)
    row = _row(module, step=1, wall_ms=2.0)
    row["collision_frontend"] = "native"

    module._validate_invocation_rows([row], args, invocation)
    row[field] = invalid
    with pytest.raises(ValueError, match=field):
        module._validate_invocation_rows([row], args, invocation)
    summary = module._summarize_group([row], failed_processes=0)
    assert f"{field}_mismatch" in summary["paper_workload_contract_reasons"]
    assert summary["paper_target_evaluated"] is False


def test_paper_cpu_split_impulse_validation_is_scenario_specific():
    module = _load_module()
    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "card_house_26_settle_projectile_full:1",
            "--contract",
            "paper_cpu",
            "--collision-frontend",
            "native",
        ]
    )
    invocation = module.Invocation(
        module.Case("card_house_26_settle_projectile_full", 1), 1, 1, False
    )
    row = _row(module, step=1, wall_ms=2.0)
    row.update(
        {
            "scenario": "card_house_26_settle_projectile_full",
            "collision_frontend": "native",
            "split_impulse_enabled": "1",
        }
    )

    module._validate_invocation_rows([row], args, invocation)
    row["split_impulse_enabled"] = "0"
    with pytest.raises(ValueError, match="split_impulse_enabled"):
        module._validate_invocation_rows([row], args, invocation)


def test_taskset_identity_resolves_and_hashes_exact_executable(tmp_path, monkeypatch):
    module = _load_module()
    executable = tmp_path / "taskset-real"
    executable.write_bytes(b"taskset executable")
    alias = tmp_path / "taskset-alias"
    alias.symlink_to(executable)
    monkeypatch.setattr(module.shutil, "which", lambda name: str(alias))

    identity = module._tool_identity("taskset")

    assert identity == {
        "name": "taskset",
        "path": str(alias),
        "resolved_path": str(executable),
        "size_bytes": executable.stat().st_size,
        "sha256": module._sha256(executable),
    }


def test_taskset_identity_rejects_missing_tool(monkeypatch):
    module = _load_module()
    monkeypatch.setattr(module.shutil, "which", lambda name: None)

    with pytest.raises(RuntimeError, match="required tool is unavailable: taskset"):
        module._tool_identity("taskset")


def test_taskset_identity_rejects_nonregular_tool(tmp_path, monkeypatch):
    module = _load_module()
    directory = tmp_path / "taskset-directory"
    directory.mkdir()
    monkeypatch.setattr(module.shutil, "which", lambda name: str(directory))

    with pytest.raises(RuntimeError, match="not a regular file"):
        module._tool_identity("taskset")


def test_taskset_identity_rejects_broken_resolution(tmp_path, monkeypatch):
    module = _load_module()
    alias = tmp_path / "taskset-alias"
    alias.symlink_to(tmp_path / "missing-taskset")
    monkeypatch.setattr(module.shutil, "which", lambda name: str(alias))

    with pytest.raises(RuntimeError, match="tool cannot be resolved"):
        module._tool_identity("taskset")


def test_unpinned_run_binds_an_empty_executed_tool_closure(monkeypatch):
    module = _load_module()
    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "backspin:1",
        ]
    )
    monkeypatch.setattr(
        module.shutil,
        "which",
        lambda name: pytest.fail(f"unexpected tool lookup: {name}"),
    )

    assert module._bind_executed_tool_closure(args) == {}


def test_main_rejects_missing_taskset_before_child_execution(tmp_path, monkeypatch):
    module = _load_module()
    child_commands = []
    monkeypatch.setattr(module.shutil, "which", lambda name: None)
    monkeypatch.setattr(
        module.subprocess,
        "run",
        lambda command, **kwargs: child_commands.append(command),
    )

    with pytest.raises(SystemExit, match="required tool is unavailable: taskset"):
        module.main(
            [
                "--binary",
                sys.executable,
                "--output-dir",
                str(tmp_path / "evidence"),
                "--case",
                "backspin:1",
                "--cpu-list",
                "0",
            ]
        )

    assert child_commands == []
    assert not (tmp_path / "evidence").exists()


def test_thread_specific_cpu_lists_use_bound_resolved_taskset(monkeypatch):
    module = _load_module()
    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "backspin:1",
            "--threads",
            "1,4",
            "--cpu-list-for",
            "1:4",
            "--cpu-list-for",
            "4:4,6,8,10",
        ]
    )
    taskset_identity = {
        "name": "taskset",
        "path": "/tmp/taskset-alias",
        "resolved_path": "/opt/pinned/taskset",
        "size_bytes": 10,
        "sha256": "a" * 64,
    }
    args.executed_tool_closure = {"taskset": taskset_identity}
    monkeypatch.setattr(
        module.shutil,
        "which",
        lambda name: "/opt/substituted/taskset",
    )

    one = module.Invocation(module.Case("backspin", 1), 1, 1, False)
    four = module.Invocation(module.Case("backspin", 1), 4, 1, False)
    assert module._command(args, one)[:3] == [
        "/opt/pinned/taskset",
        "--cpu-list",
        "4",
    ]
    assert module._command(args, four)[:3] == [
        "/opt/pinned/taskset",
        "--cpu-list",
        "4,6,8,10",
    ]


def test_cpu_pinning_rejects_an_unbound_taskset_identity():
    module = _load_module()
    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "backspin:1",
            "--cpu-list",
            "4",
        ]
    )
    invocation = module.Invocation(module.Case("backspin", 1), 1, 1, False)

    with pytest.raises(RuntimeError, match="bound taskset identity"):
        module._command(args, invocation)


@pytest.mark.parametrize(
    ("warmup", "repetition", "changed_field", "expected_stage"),
    [
        (True, 1, "sha256", "backspin-n1-t1-warmup001"),
        (False, 2, "resolved_path", "backspin-n1-t1-rep002"),
    ],
)
def test_taskset_identity_drift_fails_closed_after_each_invocation_stage(
    tmp_path,
    monkeypatch,
    warmup,
    repetition,
    changed_field,
    expected_stage,
):
    module = _load_module()
    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "backspin:1",
            "--cpu-list",
            "4",
        ]
    )
    taskset_identity = {
        "name": "taskset",
        "path": "/tmp/taskset-alias",
        "resolved_path": "/opt/pinned/taskset",
        "size_bytes": 10,
        "sha256": "a" * 64,
    }
    changed_identity = dict(taskset_identity)
    changed_identity[changed_field] = (
        "b" * 64 if changed_field == "sha256" else "/opt/substituted/taskset"
    )
    args.executed_tool_closure = {"taskset": taskset_identity}
    monkeypatch.setattr(module, "_tool_identity", lambda name: changed_identity)
    monkeypatch.setattr(module, "_invocation_affinity", lambda *unused: {})
    monkeypatch.setattr(module, "_paper_hardware_contract", lambda: "test")
    monkeypatch.setattr(
        module.subprocess,
        "run",
        lambda *args, **kwargs: module.subprocess.CompletedProcess(
            args[0], 17, "", "failed"
        ),
    )
    raw_dir = tmp_path / "raw"
    raw_dir.mkdir()
    invocation = module.Invocation(module.Case("backspin", 1), 1, repetition, warmup)

    with pytest.raises(
        RuntimeError,
        match=f"taskset executable identity drifted after {expected_stage}",
    ):
        module._run_invocation(args, invocation, raw_dir)

    assert args.executed_tool_identity_rechecks == []


def test_stable_taskset_identity_records_each_completed_recheck(tmp_path, monkeypatch):
    module = _load_module()
    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "backspin:1",
            "--cpu-list",
            "4",
        ]
    )
    taskset_identity = {
        "name": "taskset",
        "path": "/tmp/taskset-alias",
        "resolved_path": "/opt/pinned/taskset",
        "size_bytes": 10,
        "sha256": "a" * 64,
    }
    args.executed_tool_closure = {"taskset": taskset_identity}
    monkeypatch.setattr(module, "_tool_identity", lambda name: taskset_identity)
    monkeypatch.setattr(module, "_invocation_affinity", lambda *unused: {})
    monkeypatch.setattr(module, "_paper_hardware_contract", lambda: "test")
    monkeypatch.setattr(
        module.subprocess,
        "run",
        lambda *args, **kwargs: module.subprocess.CompletedProcess(
            args[0], 17, "", "failed"
        ),
    )
    raw_dir = tmp_path / "raw"
    raw_dir.mkdir()
    invocation = module.Invocation(module.Case("backspin", 1), 1, 1, False)

    module._run_invocation(args, invocation, raw_dir)

    assert args.executed_tool_identity_rechecks == ["after_backspin-n1-t1-rep001"]


def test_invocation_slug_keeps_warmups_and_case_lengths_distinct():
    module = _load_module()
    case = module.Case("backspin", 240)

    first = module.Invocation(case, 4, 1, True)
    second = module.Invocation(case, 4, 2, True)
    measured = module.Invocation(case, 4, 1, False)

    assert module._slug(first) == "backspin-n240-t4-warmup001"
    assert module._slug(second) == "backspin-n240-t4-warmup002"
    assert module._slug(measured) == "backspin-n240-t4-rep001"


def test_invocation_row_validation_requires_complete_matching_trajectory():
    module = _load_module()
    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "backspin:2",
        ]
    )
    invocation = module.Invocation(module.Case("backspin", 2), 1, 1, False)
    rows = [
        _row(module, step=1, wall_ms=2.0),
        _row(module, step=2, wall_ms=2.0),
    ]
    rows[0]["solver_contract"] = "dart_best"
    rows[1]["solver_contract"] = "dart_best"
    for row in rows:
        row.update(
            {
                "inner_local_solver": "exact_metric",
                "inner_sweeps_requested": "120",
                "fixed_inner_sweeps_requested": "0",
                "row_operator_request": "contact_row_no_dense_snapshot",
                "row_operator_mode": "contact_row_no_dense_snapshot",
            }
        )

    module._validate_invocation_rows(rows, args, invocation)
    with pytest.raises(ValueError, match="emitted 1 rows"):
        module._validate_invocation_rows(rows[:1], args, invocation)
    rows[1]["step"] = "3"
    with pytest.raises(ValueError, match="step sequence"):
        module._validate_invocation_rows(rows, args, invocation)


def test_duplicate_case_scenarios_are_rejected():
    module = _load_module()
    with pytest.raises(SystemExit):
        module._parse_args(
            [
                "--binary",
                "/tmp/fbf_paper_trace",
                "--output-dir",
                "/tmp/fbf-evidence",
                "--case",
                "backspin:2",
                "--case",
                "backspin:3",
            ]
        )


def test_unavailable_exact_diagnostics_do_not_aggregate_fake_zero_shrinks():
    module = _load_module()
    row = _row(module, step=1, wall_ms=2.0, status="boxed_lcp")
    row.update(
        {
            "solver": "boxed_lcp",
            "exact_diagnostics_contract": "unavailable_boxed_lcp",
            "residual_primal_feasibility": "nan",
            "residual_dual_feasibility": "nan",
            "residual_complementarity": "nan",
            "accepted_gamma": "nan",
            "safe_gamma": "nan",
            "shrink_iterations": "-1",
            "coupling_variation_ratio": "nan",
            "warm_start_matched_contacts": "-1",
            "warm_start_matched_fraction": "nan",
            "inner_local_solver": "not_applicable",
            "inner_sweeps_requested": "-1",
            "fixed_inner_sweeps_requested": "-1",
            "step_size_persistence_enabled": "-1",
            "step_size_recovery_growth_factor": "nan",
            "step_size_persistence_used": "-1",
            "step_size_persistence_request": "nan",
            "row_operator_request": "not_applicable",
            "row_operator_mode": "boxed_lcp",
        }
    )

    summary = module._summarize_group([row], failed_processes=0)

    assert math.isnan(summary["total_shrink_iterations"])
    assert math.isnan(summary["mean_warm_start_matched_fraction"])
    assert summary["all_solver_steps_successful"] is True


def test_paper_target_is_only_evaluated_for_a_valid_disclosed_contract():
    module = _load_module()
    rows = []
    for step in range(1, 121):
        row = _row(module, step=step, wall_ms=5.0)
        row.update(
            {
                "scenario": "incline_mu_0_5",
                "precision_contract": "paper_float32",
                "scene_contract": "paper_exact_native_collision_frontend",
                "baumgarte_contract": "paper_exact",
                "collision_frontend": "native",
                "contacts": "4",
                "paper_hardware_contract": (
                    "apple_silicon_family_exact_model_unpublished"
                ),
            }
        )
        rows.append(row)

    summary = module._summarize_group(rows, failed_processes=0)

    assert summary["full_trajectory_evidence"] is True
    assert summary["paper_workload_contract_valid"] is False
    assert summary["paper_timing_comparable"] is False
    assert summary["paper_target_evaluated"] is False
    assert summary["paper_ratio_to_reference"] is None
    assert summary["paper_mean_target_met"] is None
    assert (
        "paper_inner_block_formula_unpublished"
        in summary["paper_workload_contract_reasons"]
    )
    assert (
        "paper_step_size_recovery_rule_unpublished"
        in summary["paper_workload_contract_reasons"]
    )

    rows[0]["precision_contract"] = "float64_vs_paper_float32"
    invalid = module._summarize_group(rows, failed_processes=0)
    assert invalid["paper_workload_contract_valid"] is False
    assert invalid["paper_target_evaluated"] is False
    assert invalid["paper_ratio_to_reference"] is None
    assert invalid["paper_mean_target_met"] is None


def test_realtime_target_is_unevaluated_for_an_incomplete_trajectory():
    module = _load_module()
    row = _row(module, step=1, wall_ms=2.0)
    record = {
        "warmup": False,
        "complete_rows": False,
        "returncode": 1,
        "expected_rows": 120,
    }

    summary = module._summarize_group(
        [row], failed_processes=1, records=[record], expected_repetitions=1
    )

    assert summary["full_trajectory_evidence"] is False
    assert summary["raw_mean_below_realtime"] is True
    assert summary["realtime_contract_valid"] is False
    assert summary["mean_realtime_target_met"] is None
    assert "incomplete_measured_trajectory" in summary["realtime_contract_reasons"]


def test_truncated_failed_trajectory_retains_rows_but_cannot_validate():
    module = _load_module()
    rows = [
        _row(module, step=1, wall_ms=0.1),
        _row(module, step=2, wall_ms=0.1),
    ]
    for row in rows:
        row.update(
            {
                "scenario": "incline_mu_0_5",
                "tracked_body": "incline_cube_body",
                "process_returncode": "9",
            }
        )
    record = {
        "warmup": False,
        "complete_rows": False,
        "returncode": 9,
        "expected_rows": 120,
    }

    summary = module._summarize_group(
        rows,
        failed_processes=1,
        records=[record],
        expected_repetitions=1,
    )

    assert summary["sample_steps"] == 2
    assert summary["failed_processes"] == 1
    assert summary["complete_requested_trajectory_evidence"] is False
    assert summary["physical_outcome_valid"] is False
    assert (
        summary["physical_outcome_reasons"]
        == "physical_outcome_incomplete_repetition:1"
    )
    assert summary["realtime_contract_valid"] is False
    assert summary["mean_realtime_target_met"] is None


def test_max_iteration_status_is_accepted_but_not_successful():
    module = _load_module()
    row = _row(
        module,
        step=1,
        wall_ms=2.0,
        status="max_iterations_accepted",
    )
    row["step_exact_failures"] = "0"

    summary = module._summarize_group([row], failed_processes=0)

    assert summary["all_solver_steps_accepted"] is True
    assert summary["all_solver_steps_successful"] is False


def test_realtime_threshold_is_strictly_less_than_one_sixtieth_second():
    module = _load_module()
    rows = _physical_trajectory_rows(
        module,
        "incline_mu_0_5",
        wall_ms=module.REALTIME_THRESHOLD_MS,
    )
    for row in rows:
        row["contacts"] = "4"

    summary = module._summarize_group(rows, failed_processes=0)

    assert summary["realtime_contract_valid"] is True
    assert summary["raw_mean_below_realtime"] is False
    assert summary["mean_realtime_target_met"] is False
    assert summary["all_steps_realtime_target_met"] is False


def test_single_core_claim_requires_affinity_to_exactly_one_physical_core():
    module = _load_module()
    rows = _physical_trajectory_rows(module, "incline_mu_0_5")
    for row in rows:
        row["contacts"] = "4"

    assert module._summarize_group(rows, 0)["single_core_claim_valid"] is True
    for row in rows:
        row["affinity_physical_core_count"] = "2"
    assert module._summarize_group(rows, 0)["single_core_claim_valid"] is False
    for row in rows:
        row["affinity_physical_core_count"] = "unknown"
    assert module._summarize_group(rows, 0)["single_core_claim_valid"] is False


def test_paper_contact_match_requires_trajectory_wide_equality():
    module = _load_module()
    rows = []
    for step in range(1, 121):
        row = _row(module, step=step, wall_ms=2.0)
        row.update({"scenario": "incline_mu_0_5", "contacts": "4"})
        rows.append(row)

    assert module._summarize_group(rows, 0)["contacts_match_paper_reference"] is True
    rows[0]["contacts"] = "1"
    summary = module._summarize_group(rows, 0)
    assert summary["max_contacts"] == 4
    assert summary["contacts_match_paper_reference"] is False
    assert (
        "published_contact_count_not_matched"
        in summary["paper_workload_contract_reasons"]
    )


@pytest.mark.parametrize(
    "scenario",
    [
        "incline_mu_0_5",
        "incline_mu_0_4",
        "backspin",
        "turntable_mu_0_5_omega_2",
        "turntable_mu_0_5_omega_5",
        "turntable_mu_0_2_omega_2",
        "turntable_mu_0_2_omega_5",
        "painleve_mu_0_5",
        "painleve_mu_0_55",
        "card_house_26_settle_projectile_full",
        "masonry_arch_25_literal_wedge",
    ],
)
def test_physical_outcome_contracts_accept_expected_grid(scenario):
    module = _load_module()
    rows = _physical_trajectory_rows(module, scenario)

    valid, reasons, details = module._evaluate_physical_outcome(rows, scenario, 1)

    assert valid is True
    assert reasons == "valid"
    assert "rep1:passed=true" in details


@pytest.mark.parametrize(
    ("scenario", "wrong_final"),
    [
        ("incline_mu_0_5", {"x": "-1.11803398875", "z": "0.219134661795"}),
        ("incline_mu_0_4", {"x": "-0.219134661795", "z": "0.43826932359"}),
        ("backspin", {"vx": "4.0"}),
        ("turntable_mu_0_5_omega_2", {"x": "2.0", "contacts": "1"}),
        ("turntable_mu_0_2_omega_2", {"x": "1.0", "contacts": "1"}),
        ("painleve_mu_0_5", {"up_z": "0.5", "z": "0.4"}),
        ("painleve_mu_0_55", {"up_z": "0.9", "z": "0.4"}),
        ("masonry_arch_25_literal_wedge", {"contacts": "95"}),
    ],
)
def test_fixture_derived_physical_outcomes_reject_finite_wrong_states(
    scenario, wrong_final
):
    module = _load_module()
    rows = _physical_trajectory_rows(module, scenario, wall_ms=0.1)
    rows[-1].update(wrong_final)

    valid, reasons, details = module._evaluate_physical_outcome(rows, scenario, 1)

    assert valid is False
    assert reasons == "physical_outcome_mismatch:rep1"
    assert "rep1:passed=false" in details


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("scene_contract", "source_derived_box_arch"),
        ("collision_frontend", "dart"),
        ("split_impulse_enabled", "0"),
        ("unique_colliding_body_pairs", "23"),
        ("up_z", "0.998"),
        ("x", "0.002"),
    ],
)
def test_literal_wedge_physical_contract_rejects_wrong_scene_or_stability(field, value):
    module = _load_module()
    rows = _physical_trajectory_rows(module, module.LITERAL_WEDGE_ARCH_SCENARIO)
    rows[-1][field] = value

    valid, reasons, details = module._evaluate_physical_outcome(
        rows, module.LITERAL_WEDGE_ARCH_SCENARIO, 1
    )

    assert valid is False
    assert reasons == "physical_outcome_mismatch:rep1"
    assert "reconstructed_literal_wedge_nonpaper=true" in details


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("max_arch_body_displacement_from_initial", "0.0010001"),
        ("max_arch_body_displacement_from_initial", "nan"),
        ("min_arch_body_orientation_alignment_from_initial", "0.9989999"),
        ("min_arch_body_orientation_alignment_from_initial", "nan"),
    ],
)
def test_literal_wedge_physical_contract_checks_constructed_t0_metrics_on_first_row(
    field, value
):
    module = _load_module()
    rows = _physical_trajectory_rows(module, module.LITERAL_WEDGE_ARCH_SCENARIO)
    rows[0][field] = value

    valid, reasons, details = module._evaluate_physical_outcome(
        rows, module.LITERAL_WEDGE_ARCH_SCENARIO, 1
    )

    assert valid is False
    assert reasons == "physical_outcome_mismatch:rep1"
    assert "whole_arch_stable_from_initial=false" in details


def test_literal_wedge_physical_contract_fails_closed_without_arch_metrics():
    module = _load_module()
    rows = _physical_trajectory_rows(module, module.LITERAL_WEDGE_ARCH_SCENARIO)
    for row in rows:
        row.pop("max_arch_body_displacement_from_initial")
        row.pop("min_arch_body_orientation_alignment_from_initial")

    valid, reasons, details = module._evaluate_physical_outcome(
        rows, module.LITERAL_WEDGE_ARCH_SCENARIO, 1
    )

    assert valid is False
    assert reasons == "physical_outcome_mismatch:rep1"
    assert "finite_arch_initial_pose_metrics=false" in details


def test_literal_wedge_contract_is_full_trajectory_but_never_a_paper_reference():
    module = _load_module()

    assert module.SCENARIO_TRAJECTORY_CONTRACT[module.LITERAL_WEDGE_ARCH_SCENARIO] == (
        600,
        "reconstructed_literal_wedge_full_10_seconds_at_60_hz_nonpaper",
    )
    assert module.LITERAL_WEDGE_ARCH_SCENARIO not in module.PAPER_TRAJECTORY_CONTRACT
    assert module.LITERAL_WEDGE_ARCH_SCENARIO not in module.PAPER_REFERENCE_MS
    assert module.LITERAL_WEDGE_ARCH_SCENARIO not in module.PAPER_REFERENCE_CONTACTS
    assert module.LITERAL_WEDGE_ARCH_SCENARIO in module.TRACE_SPLIT_IMPULSE_SCENARIOS
    assert (
        module.LITERAL_WEDGE_ARCH_SCENARIO
        not in module.PAPER_CPU_SPLIT_IMPULSE_SCENARIOS
    )


def test_literal_wedge_colored_summary_can_validate_nonpaper_realtime_trajectory():
    module = _load_module()
    rows = _literal_colored_trajectory_rows(module, threads=1, wall_ms=8.0)
    # A fully converged cross-step warm start can pass the outer residual at
    # iteration zero. The eligible schedule remains visible, but that step
    # must not manufacture colored work or dispatch evidence.
    rows[1].update(
        {
            "step_fbf_iterations": "0",
            "last_exact_colored_bgs_used": "0",
            "last_exact_colored_bgs_solves": "0",
            "last_exact_colored_bgs_dispatches": "0",
            "last_exact_colored_bgs_max_participants": "0",
            "exact_colored_bgs_logical_cpus": "none",
            "max_phase_exact_colored_bgs_logical_cpus": "none",
        }
    )

    summary = module._summarize_group(rows, failed_processes=0)

    assert summary["full_trajectory_evidence"] is True
    assert summary["scenario_trajectory_steps"] == 600
    assert summary["physical_outcome_valid"] is True
    assert summary["min_contacts"] == module.LITERAL_WEDGE_ARCH_CONTACT_COUNT
    assert summary["max_contacts"] == module.LITERAL_WEDGE_ARCH_CONTACT_COUNT
    assert summary["realtime_contract_valid"] is True
    assert summary["colored_bgs_used_steps"] == 599
    assert summary["mean_realtime_target_met"] is True
    assert summary["single_core_claim_valid"] is True
    assert summary["paper_target_evaluated"] is False
    assert (
        "nonpaper_colored_inner_bgs_schedule"
        in summary["paper_workload_contract_reasons"]
    )


def test_missing_literal_zero_iteration_schedule_invalidates_multicore_and_scaling():
    module = _load_module()
    one_rows = _literal_colored_trajectory_rows(module, threads=1, wall_ms=8.0)
    four_rows = _literal_colored_trajectory_rows(module, threads=4, wall_ms=4.0)
    one_rows[1].update(
        {
            "step_fbf_iterations": "0",
            "last_exact_colored_bgs_used": "0",
            "last_exact_colored_bgs_solves": "0",
            "last_exact_colored_bgs_dispatches": "0",
            "last_exact_colored_bgs_max_participants": "0",
            "exact_colored_bgs_logical_cpus": "none",
            "max_phase_exact_colored_bgs_logical_cpus": "none",
        }
    )
    four_rows[1].update(
        {
            "step_fbf_iterations": "0",
            "last_exact_colored_bgs_used": "0",
            "last_exact_colored_bgs_solves": "0",
            "last_exact_colored_bgs_dispatches": "0",
            "last_exact_colored_bgs_max_participants": "0",
            "last_exact_colored_bgs_manifolds": "0",
            "last_exact_colored_bgs_colors": "0",
            "last_exact_colored_bgs_max_manifolds_per_color": "0",
            "exact_colored_bgs_logical_cpus": "none",
            "max_phase_exact_colored_bgs_logical_cpus": "none",
        }
    )
    scenario = module.LITERAL_WEDGE_ARCH_SCENARIO
    records = [
        _affinity_record(scenario, 1, [4]),
        _affinity_record(scenario, 4, list(COLORED_TEST_CPUS)),
    ]

    summaries = module._summaries([*one_rows, *four_rows], records=records)
    by_threads = {row["requested_threads"]: row for row in summaries}

    assert by_threads[1]["realtime_contract_valid"] is True
    assert by_threads[1]["single_core_claim_valid"] is True
    assert by_threads[4]["colored_bgs_dispatch_valid"] is False
    assert (
        "literal_contact_schedule_unobservable"
        in by_threads[4]["colored_bgs_dispatch_reasons"]
    )
    assert by_threads[4]["realtime_contract_valid"] is False
    assert by_threads[4]["multicore_claim_valid"] is False
    assert by_threads[4]["scaling_pair_valid"] is False
    assert by_threads[4]["validated_speedup_vs_one_thread"] is None


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("vx", "0.21"),
        ("vy", "1.89"),
    ],
)
def test_captured_turntable_requires_small_radial_speed_and_corotation(field, value):
    module = _load_module()
    rows = _physical_trajectory_rows(module, "turntable_mu_0_5_omega_2")
    rows[-1][field] = value

    valid, reasons, details = module._evaluate_physical_outcome(
        rows, "turntable_mu_0_5_omega_2", 1
    )

    assert valid is False
    assert reasons == "physical_outcome_mismatch:rep1"
    assert "radial_velocity=" in details
    assert "corotation_speed_error=" in details


def test_turntable_contact_loss_without_outward_or_fallen_state_is_not_ejection():
    module = _load_module()
    rows = _physical_trajectory_rows(module, "turntable_mu_0_2_omega_2")
    rows[-1].update({"x": "1.0", "y": "0.0", "z": "0.1", "contacts": "0"})

    valid, reasons, details = module._evaluate_physical_outcome(
        rows, "turntable_mu_0_2_omega_2", 1
    )

    assert valid is False
    assert reasons == "physical_outcome_mismatch:rep1"
    assert "expected=ejected,radius=1" in details
    assert "contacts=0" in details


@pytest.mark.parametrize(
    ("step", "field", "value"),
    [
        (200, "min_card_axis_up", "-0.5"),
        (402, "projectile_count", "4"),
        (403, "projectile_count", "0"),
        (600, "finite_state", "0"),
    ],
)
def test_card_house_full_contract_rejects_invalid_phase_or_premature_collapse(
    step, field, value
):
    module = _load_module()
    rows = _physical_trajectory_rows(module, "card_house_26_settle_projectile_full")
    rows[step - 1][field] = value

    valid, reasons, details = module._evaluate_physical_outcome(
        rows, "card_house_26_settle_projectile_full", 1
    )

    assert valid is False
    assert reasons == "physical_outcome_mismatch:rep1"
    assert "rep1:passed=false" in details


def test_card_house_full_contract_requires_projectile_driven_card_response():
    module = _load_module()
    rows = _physical_trajectory_rows(module, "card_house_26_settle_projectile_full")
    rows[-1].update(
        {
            "min_card_axis_up": "0.0",
            "min_center_height": "0.48",
            "max_card_horizontal_travel": "1.62",
        }
    )

    valid, reasons, details = module._evaluate_physical_outcome(
        rows, "card_house_26_settle_projectile_full", 1
    )

    assert valid is False
    assert reasons == "physical_outcome_mismatch:rep1"
    assert "impact_response=false" in details


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("max_card_center_displacement_from_initial", "0.051"),
        ("min_card_orientation_alignment_from_initial", "0.949"),
    ],
)
def test_card_house_reconstruction_gate_rejects_settle_motion(field, value):
    module = _load_module()
    rows = _physical_trajectory_rows(module, "card_house_26_settle_projectile_full")
    rows[200][field] = value

    valid, reasons, details = module._evaluate_physical_outcome(
        rows, "card_house_26_settle_projectile_full", 1
    )

    assert valid is False
    assert reasons == "physical_outcome_mismatch:rep1"
    assert "reconstruction_specific_card_gate=true" in details
    assert "reconstructed_settle_stable=false" in details


def test_card_house_response_requires_projectile_card_contact_and_post_contact_motion():
    module = _load_module()
    rows = _physical_trajectory_rows(module, "card_house_26_settle_projectile_full")
    for row in rows:
        row["projectile_card_contacts"] = "0"

    valid, _, details = module._evaluate_physical_outcome(
        rows, "card_house_26_settle_projectile_full", 1
    )

    assert valid is False
    assert "projectile_card_contact_observed=false" in details

    rows = _physical_trajectory_rows(module, "card_house_26_settle_projectile_full")
    for row in rows[449:]:
        row["max_card_center_displacement_from_initial"] = "0.01"
        row["min_card_orientation_alignment_from_initial"] = "0.99"
    valid, _, details = module._evaluate_physical_outcome(
        rows, "card_house_26_settle_projectile_full", 1
    )

    assert valid is False
    assert "projectile_card_contact_observed=true" in details
    assert "post_contact_reconstructed_response=false" in details


@pytest.mark.parametrize(
    ("step", "field", "value"),
    [
        (60, "contacts", "0"),
        (60, "penetration_depth_max", "0.021"),
        (60, "penetration_depth_max", "nan"),
    ],
)
def test_incline_requires_sustained_contact_and_bounded_penetration(step, field, value):
    module = _load_module()
    rows = _physical_trajectory_rows(module, "incline_mu_0_5")
    rows[step - 1][field] = value

    valid, reasons, details = module._evaluate_physical_outcome(
        rows, "incline_mu_0_5", 1
    )

    assert valid is False
    assert reasons == "physical_outcome_mismatch:rep1"
    assert "sustained_contact=" in details
    assert "max_penetration=" in details


def test_physical_outcome_requires_every_complete_repetition_to_pass():
    module = _load_module()
    first = _physical_trajectory_rows(module, "painleve_mu_0_5")
    second = _physical_trajectory_rows(module, "painleve_mu_0_5")
    for row in second:
        row["repetition"] = "2"
    second[-1].update({"up_z": "0.5", "z": "0.4"})

    valid, reasons, details = module._evaluate_physical_outcome(
        [*first, *second], "painleve_mu_0_5", 2
    )

    assert valid is False
    assert reasons == "physical_outcome_mismatch:rep2"
    assert "rep1:passed=true" in details
    assert "rep2:passed=false" in details


def test_uncontracted_scene_cannot_receive_physical_or_realtime_verdict():
    module = _load_module()
    row = _row(module, step=1, wall_ms=0.1)
    row["scenario"] = "card_house_26_reduced_contact"

    summary = module._summarize_group([row], failed_processes=0)

    assert summary["physical_outcome_valid"] is None
    assert (
        summary["physical_outcome_reasons"] == "physical_outcome_contract_unavailable"
    )
    assert summary["realtime_contract_valid"] is False
    assert summary["mean_realtime_target_met"] is None
    assert (
        "physical_outcome_contract_unavailable" in summary["realtime_contract_reasons"]
    )


def test_fast_finite_wrong_outcome_and_uncontrolled_affinity_are_raw_only():
    module = _load_module()
    rows = _physical_trajectory_rows(module, "incline_mu_0_5", wall_ms=0.1)
    rows[-1].update({"x": "-1.11803398875", "z": "0.219134661795"})

    wrong = module._summarize_group(rows, failed_processes=0)

    assert wrong["raw_mean_below_realtime"] is True
    assert wrong["physical_outcome_valid"] is False
    assert wrong["realtime_contract_valid"] is False
    assert wrong["mean_realtime_target_met"] is None
    assert wrong["single_core_claim_valid"] is False

    rows = _physical_trajectory_rows(module, "incline_mu_0_5", wall_ms=0.1)
    for row in rows:
        row["affinity_source"] = "inherited_process_affinity"
    uncontrolled = module._summarize_group(rows, failed_processes=0)

    assert uncontrolled["physical_outcome_valid"] is True
    assert uncontrolled["controlled_affinity_valid"] is False
    assert uncontrolled["realtime_contract_valid"] is False
    assert uncontrolled["mean_realtime_target_met"] is None
    assert uncontrolled["single_core_claim_valid"] is False
    assert (
        "controlled_affinity_contract_invalid"
        in uncontrolled["realtime_contract_reasons"]
    )


def test_observed_parallel_realtime_requires_one_logical_per_physical_core():
    module = _load_module()
    rows = _physical_trajectory_rows(
        module, "card_house_26_settle_projectile_full", wall_ms=0.1
    )
    for row in rows:
        row.update(
            {
                "requested_threads": "4",
                "actual_threads": "4",
                "solver_contract": (
                    "dart_best_threaded_world_exact_contact_row_parallel_capable"
                ),
                "collision_frontend": "native",
                "step_contact_row_delassus_products": "10",
                "step_parallel_contact_row_delassus_products": "10",
                "max_contact_row_participants_to_date": "4",
                "exact_contact_row_logical_cpus_to_date": "4;5;6;7",
                "max_phase_contact_row_logical_cpus_to_date": "4;5;6;7",
            }
        )
    _set_affinity(rows, [4, 5, 6, 7], cores=[0, 0, 1, 1])
    shared_record = _affinity_record(
        "card_house_26_settle_projectile_full",
        4,
        [4, 5, 6, 7],
        cores=[0, 0, 1, 1],
    )

    shared_cores = module._summarize_group(
        rows, failed_processes=0, records=[shared_record]
    )

    assert shared_cores["physical_outcome_valid"] is True
    assert shared_cores["controlled_affinity_valid"] is False
    assert shared_cores["realtime_contract_valid"] is False
    assert shared_cores["multicore_claim_valid"] is False

    for row in rows:
        row.update(
            {
                "exact_contact_row_logical_cpus_to_date": "4;6;8;10",
                "max_phase_contact_row_logical_cpus_to_date": "4;6;8;10",
            }
        )
    _set_affinity(rows, [4, 6, 8, 10])
    separate_record = _affinity_record(
        "card_house_26_settle_projectile_full", 4, [4, 6, 8, 10]
    )
    separate_cores = module._summarize_group(
        rows, failed_processes=0, records=[separate_record]
    )

    assert separate_cores["controlled_affinity_valid"] is True
    assert separate_cores["realtime_contract_valid"] is True
    assert separate_cores["parallel_dispatch_valid"] is True
    assert separate_cores["runtime_cpu_residency_valid"] is True
    assert separate_cores["multicore_claim_valid"] is True


def test_colored_multicore_claim_uses_separate_dispatch_and_residency_gates():
    module = _load_module()
    rows = _physical_trajectory_rows(
        module, "card_house_26_settle_projectile_full", wall_ms=0.1
    )
    for row in rows:
        row.update({column: "0" for column in module.TRACE_COLORED_BGS_COLUMNS_V8})
        row.update(
            {
                "requested_threads": "4",
                "actual_threads": "4",
                "solver_contract": (
                    "dart_best_nonpaper_colored_inner_bgs_threaded_world"
                ),
                "collision_frontend": "native",
                "step_contact_row_delassus_products": "10",
                "step_parallel_contact_row_delassus_products": "0",
                "max_contact_row_participants_to_date": "1",
                "exact_contact_row_logical_cpus_to_date": "none",
                "max_phase_contact_row_logical_cpus_to_date": "none",
                "inner_bgs_schedule_contract": (
                    "dart_deterministic_manifold_colored_bgs_nonpaper"
                ),
                "last_exact_colored_bgs_used": "1",
                "last_exact_colored_bgs_solves": "10",
                "last_exact_colored_bgs_dispatches": "1",
                "last_exact_colored_bgs_max_participants": "4",
                "last_exact_colored_bgs_manifolds": "24",
                "last_exact_colored_bgs_colors": "3",
                "last_exact_colored_bgs_max_manifolds_per_color": "8",
                "exact_colored_bgs_logical_cpus": "4;6;8;10",
                "max_phase_exact_colored_bgs_logical_cpus": "4;6;8;10",
            }
        )
    _set_affinity(rows, [4, 6, 8, 10])
    record = _affinity_record("card_house_26_settle_projectile_full", 4, [4, 6, 8, 10])

    summary = module._summarize_group(rows, 0, records=[record])

    assert summary["parallel_dispatch_valid"] is False
    assert summary["colored_bgs_dispatch_valid"] is True
    assert summary["runtime_cpu_residency_valid"] is True
    assert summary["multicore_claim_valid"] is True
    assert summary["parallelism_contract"] == (
        "exact_fbf_nonpaper_colored_inner_bgs_observed"
    )
    assert summary["paper_target_evaluated"] is False
    assert (
        "nonpaper_colored_inner_bgs_schedule"
        in summary["paper_workload_contract_reasons"]
    )


@pytest.mark.parametrize(
    ("field", "value", "reason"),
    [
        (
            "last_exact_colored_bgs_dispatches",
            "2",
            "colored_persistent_dispatch_count_mismatch",
        ),
        (
            "last_exact_colored_bgs_max_manifolds_per_color",
            "3",
            "colored_schedule_width_below_thread_count",
        ),
        (
            "step_parallel_contact_row_delassus_products",
            "1",
            "legacy_parallel_contact_row_work_observed",
        ),
        (
            "max_contact_row_participants_to_date",
            "2",
            "legacy_contact_row_participants_observed",
        ),
        (
            "exact_contact_row_logical_cpus_to_date",
            "4",
            "legacy_contact_row_cpu_residency_observed",
        ),
        (
            "last_exact_colored_bgs_colors",
            "25",
            "colored_schedule_dimensions_inconsistent",
        ),
    ],
)
def test_colored_summary_fails_closed_on_per_step_dispatch_schedule_or_legacy_leak(
    field, value, reason
):
    module = _load_module()
    rows = _physical_trajectory_rows(
        module, "card_house_26_settle_projectile_full", wall_ms=0.1
    )
    for row in rows:
        row.update({column: "0" for column in module.TRACE_COLORED_BGS_COLUMNS_V8})
        row.update(
            {
                "requested_threads": "4",
                "actual_threads": "4",
                "solver_contract": (
                    "dart_best_nonpaper_colored_inner_bgs_threaded_world"
                ),
                "collision_frontend": "native",
                "step_contact_row_delassus_products": "10",
                "max_contact_row_participants_to_date": "1",
                "inner_bgs_schedule_contract": (
                    "dart_deterministic_manifold_colored_bgs_nonpaper"
                ),
                "last_exact_colored_bgs_used": "1",
                "last_exact_colored_bgs_solves": "10",
                "last_exact_colored_bgs_dispatches": "1",
                "last_exact_colored_bgs_max_participants": "4",
                "last_exact_colored_bgs_manifolds": "24",
                "last_exact_colored_bgs_colors": "3",
                "last_exact_colored_bgs_max_manifolds_per_color": "8",
                "exact_colored_bgs_logical_cpus": "4;6;8;10",
                "max_phase_exact_colored_bgs_logical_cpus": "4;6;8;10",
            }
        )
    rows[0][field] = value
    _set_affinity(rows, [4, 6, 8, 10])
    record = _affinity_record("card_house_26_settle_projectile_full", 4, [4, 6, 8, 10])

    summary = module._summarize_group(rows, 0, records=[record])

    assert summary["colored_bgs_dispatch_valid"] is False
    assert reason in summary["colored_bgs_dispatch_reasons"]
    assert summary["realtime_contract_valid"] is False
    assert summary["multicore_claim_valid"] is False


def test_lifetime_cpu_union_is_audit_only_without_per_phase_residency():
    module = _load_module()
    rows = _physical_trajectory_rows(
        module, "card_house_26_settle_projectile_full", wall_ms=0.1
    )
    for row in rows:
        row.update(
            {
                "requested_threads": "4",
                "actual_threads": "4",
                "solver_contract": (
                    "dart_best_threaded_world_exact_contact_row_parallel_capable"
                ),
                "collision_frontend": "native",
                "step_contact_row_delassus_products": "10",
                "step_parallel_contact_row_delassus_products": "10",
                "max_contact_row_participants_to_date": "4",
                "exact_contact_row_logical_cpus_to_date": "4;6;8;10",
                "max_phase_contact_row_logical_cpus_to_date": "none",
            }
        )
    _set_affinity(rows, [4, 6, 8, 10])
    record = _affinity_record("card_house_26_settle_projectile_full", 4, [4, 6, 8, 10])

    summary = module._summarize_group(rows, 0, records=[record])

    assert summary["parallel_dispatch_valid"] is True
    assert summary["observed_exact_contact_row_logical_cpu_count"] == 4
    assert summary["observed_exact_contact_row_physical_core_count"] == 4
    assert summary["runtime_cpu_residency_valid"] is False
    assert (
        "per_parallel_phase_core_residency_unobserved"
        in summary["runtime_cpu_residency_reasons"]
    )
    assert summary["multicore_claim_valid"] is False


def test_every_complete_repetition_must_observe_requested_cores_in_one_phase():
    module = _load_module()
    _, first_rows, records = _card_scaling_evidence(module)
    second_rows = [dict(row) for row in first_rows]
    for row in second_rows:
        row["repetition"] = "2"
        row["max_phase_contact_row_logical_cpus_to_date"] = "4;6"
    records = [
        records[1],
        _affinity_record(
            "card_house_26_settle_projectile_full",
            4,
            [4, 6, 8, 10],
            repetition=2,
        ),
    ]

    summary = module._summarize_group(
        [*first_rows, *second_rows],
        0,
        records=records,
        expected_repetitions=2,
    )

    assert summary["parallel_dispatch_valid"] is True
    assert summary["phase_residency_min_logical_cpu_count"] == 2
    assert summary["phase_residency_min_physical_core_count"] == 2
    assert summary["runtime_cpu_residency_valid"] is False
    assert (
        "phase_logical_cpu_count_mismatch:rep2"
        in summary["runtime_cpu_residency_reasons"]
    )
    assert summary["multicore_claim_valid"] is False


def test_parallelism_report_escapes_multi_repetition_phase_separator(tmp_path):
    module = _load_module()
    _, first_rows, records = _card_scaling_evidence(module)
    second_rows = [dict(row) for row in first_rows]
    for row in second_rows:
        row["repetition"] = "2"
    records = [
        records[1],
        _affinity_record(
            "card_house_26_settle_projectile_full",
            4,
            [4, 6, 8, 10],
            repetition=2,
        ),
    ]
    summary = module._summarize_group(
        [*first_rows, *second_rows],
        0,
        records=records,
        expected_repetitions=2,
    )
    report_path = tmp_path / "REPORT.md"

    module._write_report(report_path, [summary])
    report = report_path.read_text(encoding="utf-8")

    assert (
        summary["phase_residency_logical_cpus_by_repetition"]
        == "rep1=4;6;8;10|rep2=4;6;8;10"
    )
    assert "rep1=4;6;8;10<br>rep2=4;6;8;10" in report
    assert "rep1=4;6;8;10|rep2=4;6;8;10" not in report
    assert "Mean realtime met | Every-step deadline met" in report
    assert "Mean realtime met` is a throughput-average verdict" in report


def test_threaded_capability_without_observed_parallel_products_is_not_multicore():
    module = _load_module()
    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "backspin:1",
            "--threads",
            "4",
            "--contract",
            "dart_best",
        ]
    )
    invocation = module.Invocation(module.Case("backspin", 1), 4, 1, False)
    row = _row(module, step=1, wall_ms=2.0)
    row.update(
        {
            "requested_threads": "4",
            "actual_threads": "4",
            "solver_contract": (
                "dart_best_threaded_world_exact_contact_row_parallel_capable"
            ),
            "inner_local_solver": "exact_metric",
            "inner_sweeps_requested": "120",
            "fixed_inner_sweeps_requested": "0",
            "row_operator_request": "contact_row_no_dense_snapshot",
            "row_operator_mode": "contact_row_no_dense_snapshot",
            "step_contact_row_delassus_products": "10",
            "step_parallel_contact_row_delassus_products": "0",
            "max_contact_row_participants_to_date": "1",
            "affinity_logical_cpus": "4,6,8,10",
            "affinity_logical_cpu_count": "4",
            "affinity_physical_core_count": "4",
        }
    )

    module._validate_invocation_rows([row], args, invocation)
    summary = module._summarize_group([row], failed_processes=0)
    assert summary["parallelism_contract"] == "exact_fbf_contact_row_serial_observed"
    assert summary["multicore_claim_valid"] is False

    row["step_parallel_contact_row_delassus_products"] = "1"
    with pytest.raises(ValueError, match="without multiple participants"):
        module._validate_invocation_rows([row], args, invocation)


def test_invocation_rejects_nonmonotonic_unattributed_or_out_of_affinity_residency():
    module = _load_module()
    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "backspin:2",
            "--threads",
            "4",
            "--contract",
            "dart_best",
            "--cpu-list-for",
            "4:4,6,8,10",
        ]
    )
    invocation = module.Invocation(module.Case("backspin", 2), 4, 1, False)
    rows = []
    for step, logical_cpus, participants in (
        (1, "4;6", 2),
        (2, "4;6;8;10", 4),
    ):
        row = _row(module, step=step, wall_ms=2.0)
        row.update(
            {
                "requested_threads": "4",
                "actual_threads": "4",
                "solver_contract": (
                    "dart_best_threaded_world_exact_contact_row_parallel_capable"
                ),
                "inner_sweeps_requested": "120",
                "fixed_inner_sweeps_requested": "0",
                "step_contact_row_delassus_products": "10",
                "step_parallel_contact_row_delassus_products": "10",
                "max_contact_row_participants_to_date": str(participants),
                "exact_contact_row_logical_cpus_to_date": logical_cpus,
                "max_phase_contact_row_logical_cpus_to_date": logical_cpus,
            }
        )
        rows.append(row)

    module._validate_invocation_rows(rows, args, invocation)

    decreasing = [dict(row) for row in rows]
    decreasing[0]["exact_contact_row_logical_cpus_to_date"] = "4;6;8;10"
    decreasing[0]["max_phase_contact_row_logical_cpus_to_date"] = "4;6;8;10"
    decreasing[0]["max_contact_row_participants_to_date"] = "4"
    decreasing[1]["exact_contact_row_logical_cpus_to_date"] = "4;6"
    decreasing[1]["max_phase_contact_row_logical_cpus_to_date"] = "4;6"
    with pytest.raises(ValueError, match="logical CPU set decreased"):
        module._validate_invocation_rows(decreasing, args, invocation)

    unattributed_growth = [dict(row) for row in rows]
    unattributed_growth[1]["step_parallel_contact_row_delassus_products"] = "0"
    with pytest.raises(ValueError, match="logical CPU set grew"):
        module._validate_invocation_rows(unattributed_growth, args, invocation)

    outside_affinity = [dict(row) for row in rows]
    outside_affinity[1]["exact_contact_row_logical_cpus_to_date"] = "4;6;8;12"
    outside_affinity[1]["max_phase_contact_row_logical_cpus_to_date"] = "4;6;8;12"
    with pytest.raises(ValueError, match="outside invocation affinity"):
        module._validate_invocation_rows(outside_affinity, args, invocation)


def test_scaling_matrix_reports_raw_speedup_but_withholds_validated_speedup():
    module = _load_module()
    one_rows = _physical_trajectory_rows(module, "incline_mu_0_5", wall_ms=4.0)
    four_rows = _physical_trajectory_rows(module, "incline_mu_0_5", wall_ms=2.0)
    for one in one_rows:
        one.update(
            {
                "contacts": "4",
                "solver_contract": "dart_best",
            }
        )
    for four in four_rows:
        four.update(
            {
                "contacts": "4",
                "requested_threads": "4",
                "actual_threads": "4",
                "solver_contract": (
                    "dart_best_threaded_world_exact_contact_row_parallel_capable"
                ),
            }
        )
    _set_affinity(four_rows, [4, 6, 8, 10])
    rows = [*one_rows, *four_rows]

    summaries = module._summaries(rows, records=[])
    by_threads = {row["requested_threads"]: row for row in summaries}

    assert by_threads[1]["validated_speedup_vs_one_thread"] == 1.0
    assert by_threads[4]["raw_speedup_vs_one_thread"] == pytest.approx(2.0)
    assert by_threads[4]["multicore_claim_valid"] is False
    assert by_threads[4]["validated_speedup_vs_one_thread"] is None


def test_scaling_matrix_validates_speedup_only_with_observed_exact_kernel_work():
    module = _load_module()
    one_rows = _physical_trajectory_rows(
        module, "card_house_26_settle_projectile_full", wall_ms=4.0
    )
    four_rows = _physical_trajectory_rows(
        module, "card_house_26_settle_projectile_full", wall_ms=2.0
    )
    for row in one_rows:
        row.update(
            {
                "solver_contract": "dart_best",
                "collision_frontend": "native",
                "step_contact_row_delassus_products": "10",
                "max_contact_row_participants_to_date": "1",
            }
        )
    for row in four_rows:
        row.update(
            {
                "requested_threads": "4",
                "actual_threads": "4",
                "solver_contract": (
                    "dart_best_threaded_world_exact_contact_row_parallel_capable"
                ),
                "collision_frontend": "native",
                "step_contact_row_delassus_products": "10",
                "step_parallel_contact_row_delassus_products": "10",
                "max_contact_row_participants_to_date": "4",
                "exact_contact_row_logical_cpus_to_date": "4;6;8;10",
                "max_phase_contact_row_logical_cpus_to_date": "4;6;8;10",
            }
        )
    _set_affinity(one_rows, [4])
    _set_affinity(four_rows, [4, 6, 8, 10])
    records = [
        _affinity_record("card_house_26_settle_projectile_full", 1, [4]),
        _affinity_record("card_house_26_settle_projectile_full", 4, [4, 6, 8, 10]),
    ]

    summaries = module._summaries([*one_rows, *four_rows], records=records)
    by_threads = {row["requested_threads"]: row for row in summaries}

    assert by_threads[1]["single_core_claim_valid"] is True
    assert by_threads[4]["multicore_claim_valid"] is True
    assert by_threads[4]["parallelism_contract"] == (
        "exact_fbf_contact_row_parallel_observed"
    )
    assert by_threads[4]["parallel_contact_row_product_fraction"] == 1.0
    assert by_threads[4]["max_contact_row_participants"] == 4
    assert by_threads[4]["runtime_cpu_residency_valid"] is True
    assert by_threads[4]["scaling_pair_valid"] is True
    assert by_threads[4]["scaling_pair_reasons"] == "valid"
    assert by_threads[4]["validated_speedup_vs_one_thread"] == pytest.approx(2.0)


def test_hybrid_core_classes_fail_closed_for_multicore_and_scaling_claims():
    module = _load_module()
    one_rows, candidate_rows, records = _card_scaling_evidence(
        module,
        candidate_core_classes_khz=[4000000, 4000000, 3000000, 3000000],
    )

    summaries = module._summaries([*one_rows, *candidate_rows], records)
    candidate = {row["requested_threads"]: row for row in summaries}[4]

    assert candidate["controlled_affinity_valid"] is False
    assert "single_core_class_unproven" in candidate["controlled_affinity_reasons"]
    assert candidate["multicore_claim_valid"] is False
    assert candidate["scaling_pair_valid"] is False
    assert "scaling_core_class_mismatch" in candidate["scaling_pair_reasons"]
    assert candidate["validated_speedup_vs_one_thread"] is None


def test_scaling_pair_rejects_governor_mismatch_even_when_each_run_is_controlled():
    module = _load_module()
    one_rows, candidate_rows, records = _card_scaling_evidence(
        module,
        candidate_governors=["powersave"] * 4,
    )

    summaries = module._summaries([*one_rows, *candidate_rows], records)
    candidate = {row["requested_threads"]: row for row in summaries}[4]

    assert candidate["controlled_affinity_valid"] is True
    assert candidate["multicore_claim_valid"] is True
    assert candidate["scaling_pair_valid"] is False
    assert "scaling_governor_mismatch" in candidate["scaling_pair_reasons"]
    assert candidate["raw_speedup_vs_one_thread"] == pytest.approx(2.0)
    assert candidate["validated_speedup_vs_one_thread"] is None


def test_scaling_pair_rejects_nonnested_one_thread_cpu_list():
    module = _load_module()
    one_rows, candidate_rows, records = _card_scaling_evidence(
        module, candidate_cpus=(6, 8, 10, 12)
    )

    summaries = module._summaries([*one_rows, *candidate_rows], records)
    candidate = {row["requested_threads"]: row for row in summaries}[4]

    assert candidate["multicore_claim_valid"] is True
    assert candidate["scaling_pair_valid"] is False
    assert "baseline_cpu_list_not_nested" in candidate["scaling_pair_reasons"]
    assert candidate["raw_speedup_vs_one_thread"] == pytest.approx(2.0)
    assert candidate["validated_speedup_vs_one_thread"] is None


def test_scaling_pair_rejects_solver_workload_option_mismatch():
    module = _load_module()
    one_rows, candidate_rows, records = _card_scaling_evidence(module)
    for row in candidate_rows:
        row["outer_relaxation"] = "0.9"

    summaries = module._summaries([*one_rows, *candidate_rows], records)
    candidate = {row["requested_threads"]: row for row in summaries}[4]

    assert candidate["multicore_claim_valid"] is True
    assert candidate["scaling_pair_valid"] is False
    assert (
        "workload_option_mismatch:outer_relaxation" in candidate["scaling_pair_reasons"]
    )
    assert candidate["validated_speedup_vs_one_thread"] is None


def test_scaling_pair_requires_identical_measured_work_trajectory():
    module = _load_module()
    one_rows, candidate_rows, records = _card_scaling_evidence(module)
    for row in one_rows:
        row["step_contact_row_delassus_products"] = "20"

    summaries = module._summaries([*one_rows, *candidate_rows], records)
    by_threads = {row["requested_threads"]: row for row in summaries}

    assert by_threads[1]["single_core_claim_valid"] is True
    assert by_threads[4]["multicore_claim_valid"] is True
    assert by_threads[4]["scaling_pair_valid"] is False
    assert "measured_workload_trace_mismatch" in by_threads[4]["scaling_pair_reasons"]
    assert by_threads[4]["raw_speedup_vs_one_thread"] == pytest.approx(2.0)
    assert by_threads[4]["validated_speedup_vs_one_thread"] is None


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("step_parallel_contact_row_delassus_products", "1"),
        ("max_contact_row_participants_to_date", "2"),
        ("exact_contact_row_logical_cpus_to_date", "4"),
        ("max_phase_contact_row_logical_cpus_to_date", "4"),
        ("last_exact_colored_bgs_used", "0"),
        ("last_exact_colored_bgs_solves", "9"),
        ("last_exact_colored_bgs_manifolds", "23"),
        ("last_exact_colored_bgs_colors", "4"),
        ("last_exact_colored_bgs_max_manifolds_per_color", "7"),
        ("max_arch_body_displacement_from_initial", "3e-5"),
        ("min_arch_body_orientation_alignment_from_initial", "0.999999997"),
    ],
)
def test_colored_scaling_fingerprint_covers_per_step_work_and_schedule(field, value):
    module = _load_module()
    baseline = _literal_colored_row(module, threads=1)
    candidate = _literal_colored_row(module, threads=4)

    assert module._measured_workload_fingerprint(
        [baseline]
    ) == module._measured_workload_fingerprint([candidate])

    candidate[field] = value
    assert module._measured_workload_fingerprint(
        [baseline]
    ) != module._measured_workload_fingerprint([candidate])


def test_colored_scaling_fingerprint_omits_thread_specific_dispatch_evidence():
    module = _load_module()
    baseline = _literal_colored_row(module, threads=1)
    candidate = _literal_colored_row(module, threads=4)

    assert (
        baseline["last_exact_colored_bgs_dispatches"]
        != candidate["last_exact_colored_bgs_dispatches"]
    )
    assert (
        baseline["last_exact_colored_bgs_max_participants"]
        != candidate["last_exact_colored_bgs_max_participants"]
    )
    assert (
        baseline["exact_colored_bgs_logical_cpus"]
        != candidate["exact_colored_bgs_logical_cpus"]
    )
    assert module._measured_workload_fingerprint(
        [baseline]
    ) == module._measured_workload_fingerprint([candidate])


def test_failed_configured_warmup_invalidates_realtime_multicore_and_scaling():
    module = _load_module()
    one_rows, candidate_rows, records = _card_scaling_evidence(module)
    scenario = "card_house_26_settle_projectile_full"
    records.extend(
        [
            _affinity_record(scenario, 1, [4], warmup=True),
            _affinity_record(
                scenario,
                4,
                [4, 6, 8, 10],
                warmup=True,
                returncode=17,
                complete_rows=False,
            ),
        ]
    )

    summaries = module._summaries([*one_rows, *candidate_rows], records)
    by_threads = {row["requested_threads"]: row for row in summaries}

    assert by_threads[1]["warmup_contract_valid"] is True
    assert by_threads[4]["completed_warmup_trajectories"] == 0
    assert by_threads[4]["failed_warmup_processes"] == 1
    assert by_threads[4]["warmup_contract_valid"] is False
    assert (
        "warmup_trajectory_failure_or_incomplete"
        in by_threads[4]["realtime_contract_reasons"]
    )
    assert by_threads[4]["realtime_contract_valid"] is False
    assert by_threads[4]["multicore_claim_valid"] is False
    assert by_threads[4]["scaling_pair_valid"] is False
    assert by_threads[4]["validated_speedup_vs_one_thread"] is None


def test_scaling_pair_requires_matching_smt_sibling_topology():
    module = _load_module()
    one_rows, candidate_rows, records = _card_scaling_evidence(
        module, candidate_smt_sibling_counts=[1] * 4
    )

    summaries = module._summaries([*one_rows, *candidate_rows], records)
    candidate = {row["requested_threads"]: row for row in summaries}[4]

    assert candidate["controlled_affinity_valid"] is True
    assert candidate["multicore_claim_valid"] is True
    assert candidate["scaling_pair_valid"] is False
    assert "scaling_smt_topology_mismatch" in candidate["scaling_pair_reasons"]
    assert candidate["validated_speedup_vs_one_thread"] is None


def test_scaling_pair_requires_nested_cpu_physical_mapping_to_match():
    module = _load_module()
    one_rows, candidate_rows, records = _card_scaling_evidence(
        module, candidate_cores=[99, 6, 8, 10]
    )

    summaries = module._summaries([*one_rows, *candidate_rows], records)
    candidate = {row["requested_threads"]: row for row in summaries}[4]

    assert candidate["controlled_affinity_valid"] is True
    assert candidate["multicore_claim_valid"] is True
    assert candidate["scaling_pair_valid"] is False
    assert "nested_cpu_physical_mapping_mismatch" in candidate["scaling_pair_reasons"]
    assert candidate["validated_speedup_vs_one_thread"] is None


def test_cpu_list_parser_expands_ranges_and_rejects_ambiguous_lists():
    module = _load_module()

    assert module._expand_cpu_list("2-4,8") == (2, 3, 4, 8)
    with pytest.raises(Exception):
        module._expand_cpu_list("4-2")
    with pytest.raises(Exception):
        module._expand_cpu_list("2,2")


def test_affinity_topology_records_package_core_smt_class_and_governor(monkeypatch):
    module = _load_module()

    def fake_read_text(path):
        cpu = int(re.search(r"cpu([0-9]+)", str(path)).group(1))
        values = {
            "physical_package_id": "0",
            "core_id": str(cpu // 2),
            "thread_siblings_list": f"{cpu},{cpu + 64}",
            "cpuinfo_max_freq": "4000000",
            "scaling_governor": "performance",
        }
        return values.get(path.name)

    monkeypatch.setattr(module, "_read_text", fake_read_text)
    topology = module._logical_cpu_topology([4, 6])

    assert topology[4] == {
        "physical_package_id": "0",
        "core_id": "2",
        "thread_siblings_list": "4,68",
        "thread_sibling_count": 2,
        "cpuinfo_max_frequency_khz": 4000000,
        "scaling_governor": "performance",
    }

    args = module._parse_args(
        [
            "--binary",
            "/tmp/fbf_paper_trace",
            "--output-dir",
            "/tmp/fbf-evidence",
            "--case",
            "backspin:1",
            "--threads",
            "2",
            "--cpu-list",
            "4,6",
        ]
    )
    affinity = module._invocation_affinity(
        args, module.Invocation(module.Case("backspin", 1), 2, 1, False)
    )

    assert affinity["physical_core_keys"] == ["0:2", "0:3"]
    assert affinity["logical_cpu_physical_core_keys"] == {
        "4": "0:2",
        "6": "0:3",
    }
    assert affinity["package_ids"] == ["0"]
    assert affinity["package_count"] == 1
    assert affinity["smt_sibling_counts"] == [2]
    assert affinity["core_classes_khz"] == [4000000]
    assert affinity["scaling_governors"] == ["performance"]


def test_accepted_zero_row_failures_remain_visible_and_strict_json(
    monkeypatch, tmp_path
):
    module = _load_module()
    output_dir = tmp_path / "evidence"

    def failed_invocation(args, invocation, raw_dir):
        del raw_dir
        return [], {
            "scenario": invocation.case.scenario,
            "collision_frontend": args.collision_frontend,
            "cpu_list": None,
            "cpu_affinity": {
                "source": "test_affinity",
                "logical_cpus": [invocation.threads],
                "logical_cpu_count": 1,
                "physical_core_count": 1,
                "one_logical_per_physical_core": True,
            },
            "paper_hardware_contract": "linux_x86_64_vs_paper_apple_silicon",
            "steps": invocation.case.steps,
            "threads": invocation.threads,
            "repetition": invocation.repetition,
            "warmup": invocation.warmup,
            "command": ["fbf_paper_trace"],
            "returncode": 17,
            "timed_out": False,
            "rows": 0,
            "expected_rows": invocation.case.steps,
            "complete_rows": False,
            "stdout_file": "raw/failure.csv",
            "stderr_file": "raw/failure.stderr.txt",
        }

    monkeypatch.setattr(module, "_run_invocation", failed_invocation)
    source_identity = {"runner_sha256": "source"}
    runtime_identity = {"resolved_regular_files": {"libdart.so": {}}}
    binary_sha256 = module._sha256(Path(sys.executable))
    monkeypatch.setattr(
        module,
        "_metadata",
        lambda _args, _root: {
            "nonfinite_probe": math.inf,
            "source_identity": source_identity,
            "runtime_identity": runtime_identity,
            "binary": {"sha256": binary_sha256},
        },
    )
    monkeypatch.setattr(module, "_source_identity", lambda _root: source_identity)
    monkeypatch.setattr(module, "_runtime_identity", lambda _binary: runtime_identity)
    monkeypatch.setattr(module, "_run_text", lambda *_args, **_kwargs: str(ROOT))

    result = module.main(
        [
            "--binary",
            sys.executable,
            "--output-dir",
            str(output_dir),
            "--case",
            "backspin:1",
            "--case",
            "incline_mu_0_5:1",
            "--threads",
            "1,4",
            "--repetitions",
            "1",
            "--accept-nonzero",
        ]
    )

    def reject_nonfinite(token):
        raise AssertionError(f"non-RFC JSON constant: {token}")

    summaries = json.loads(
        (output_dir / "summary.json").read_text(encoding="utf-8"),
        parse_constant=reject_nonfinite,
    )
    metadata = json.loads(
        (output_dir / "metadata.json").read_text(encoding="utf-8"),
        parse_constant=reject_nonfinite,
    )
    invocations = json.loads(
        (output_dir / "invocations.json").read_text(encoding="utf-8"),
        parse_constant=reject_nonfinite,
    )
    artifact_index = json.loads(
        (output_dir / "artifact-index.json").read_text(encoding="utf-8"),
        parse_constant=reject_nonfinite,
    )
    report = (output_dir / "REPORT.md").read_text(encoding="utf-8")

    assert result == 0
    assert metadata["nonfinite_probe"] is None
    assert len(summaries) == 4
    assert len(invocations) == 4
    assert artifact_index["schema_version"] == (
        "dart.fbf_cpu_evidence_artifact_index/v1"
    )
    indexed_files = artifact_index["files"]
    assert "artifact-index.json" not in indexed_files
    assert set(indexed_files) == {
        "REPORT.md",
        "invocations.json",
        "metadata.json",
        "raw.csv",
        "summary.csv",
        "summary.json",
    }
    for relative, identity in indexed_files.items():
        path = output_dir / relative
        assert identity == {
            "sha256": module._sha256(path),
            "size_bytes": path.stat().st_size,
        }
    assert {record["returncode"] for record in invocations} == {17}
    for summary in summaries:
        assert set(summary) == set(module.SUMMARY_COLUMNS)
        assert summary["sample_steps"] == 0
        assert summary["failed_processes"] == 1
        assert summary["complete_requested_trajectory_evidence"] is False
        assert summary["all_solver_steps_successful"] is False
        assert summary["solver_statuses"] == "no_rows_process_failed"
        assert summary["mean_step_ms"] is None
        assert summary["raw_speedup_vs_one_thread"] is None
        assert "measured_process_failure" in summary["realtime_contract_reasons"]
    assert report.count("| backspin |") == 6
    assert report.count("| incline_mu_0_5 |") == 6
    assert "| Samples | Failed measured | Failed warmups | Complete |" in report
    assert "## Physical outcome audit" in report
    assert "## Exact-kernel parallelism audit" in report
    assert "Scaling pair valid/reasons" in report
    assert "not proof of perfect simultaneity" in report
