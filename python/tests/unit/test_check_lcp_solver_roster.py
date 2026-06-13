from __future__ import annotations

import ast
import csv
import importlib.util
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_lcp_solver_roster.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("check_lcp_solver_roster", SCRIPT)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _valid_evidence_row() -> dict[str, str]:
    return {
        "category": "Standard",
        "solver": "Dantzig",
        "problem_size": "12",
        "lcp_dimension": "12",
        "contact_count": "",
        "solver_identity_schema_version": "1",
        "solver_manifest_index": "1",
        "solver_family_pivoting": "1",
        "solver_family_projection": "0",
        "solver_family_newton": "0",
        "solver_family_other": "0",
        "time_ns": "10",
        "contract_ok": "1",
        "iterations": "0",
        "residual": "0",
        "complementarity": "0",
        "bound_violation": "0",
        "solver_supports_standard": "1",
        "solver_supports_boxed": "1",
        "solver_supports_friction_index": "1",
        "solver_supports_problem": "1",
        "problem_type_standard": "1",
        "problem_type_boxed": "0",
        "problem_type_friction_index": "0",
        "problem_type_invalid": "0",
    }


def test_lcp_solver_roster_surfaces_match() -> None:
    module = _load_module()

    module.check_roster()


def test_lcp_solver_roster_rejects_demo_profile_column_drift(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    stale_schema = dict(module.parse_demo_profile_evidence_schema())
    stale_columns = tuple(
        column
        for column in module.REQUIRED_EVIDENCE_COLUMNS
        if column != "time_ns"
    )
    stale_schema["_PERFORMANCE_PROFILE_EVIDENCE_REQUIRED_COLUMNS"] = stale_columns
    monkeypatch.setattr(
        module,
        "parse_demo_profile_evidence_schema",
        lambda: stale_schema,
    )

    with pytest.raises(
        AssertionError,
        match="profile evidence required columns do not match",
    ):
        module.check_demo_profile_evidence_required_columns()


def test_lcp_solver_roster_rejects_demo_profile_schema_drift(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    stale_schema = dict(module.parse_demo_profile_evidence_schema())
    stale_schema["_SOLVER_IDENTITY_SCHEMA_VERSION"] = 2
    monkeypatch.setattr(
        module,
        "parse_demo_profile_evidence_schema",
        lambda: stale_schema,
    )

    with pytest.raises(
        AssertionError,
        match="profile evidence schema does not match",
    ):
        module.check_demo_profile_evidence_required_columns()


def test_lcp_solver_roster_reads_demo_profile_evidence_schema_rows() -> None:
    module = _load_module()

    documented_fields: list[str] = []
    for row in module.parse_demo_profile_evidence_schema_rows():
        assert set(row) == {"fields", "meaning"}
        assert row["meaning"]
        documented_fields.extend(module._split_demo_reference_list(row["fields"]))

    assert documented_fields == list(module.REQUIRED_EVIDENCE_COLUMNS)
    module.check_demo_profile_evidence_schema_rows()


def test_lcp_solver_roster_rejects_stale_profile_evidence_schema_rows(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    stale_rows = [
        {"fields": "category, solver", "meaning": ""},
        {"fields": "category, stale_column", "meaning": "stale field"},
    ]
    monkeypatch.setattr(
        module,
        "parse_demo_profile_evidence_schema_rows",
        lambda: stale_rows,
    )

    with pytest.raises(
        AssertionError,
        match="profile evidence schema rows are out of sync",
    ) as exc_info:
        module.check_demo_profile_evidence_schema_rows()

    message = str(exc_info.value)
    assert "category, solver: missing fields ['meaning']" in message
    assert "duplicate documented profile evidence fields ['category']" in message
    assert "profile evidence schema rows do not match required columns" in message
    assert "stale_column" in message
    assert "time_ns" in message


def test_lcp_solver_roster_reads_demo_command_metadata() -> None:
    module = _load_module()

    command_metadata = module.parse_demo_command_metadata()
    benchmark_rows = list(command_metadata["_BENCHMARK_PACKET_ROWS"])
    representative_filter = module._demo_benchmark_filter_union(benchmark_rows)

    assert command_metadata["_BENCHMARK_COMMAND"] == (
        "pixi run bm lcp_compare -- --benchmark_filter=BM_LCP_COMPARE_SMOKE"
    )
    assert command_metadata["_REPRESENTATIVE_BENCHMARK_FILTER"] == (
        representative_filter
    )
    assert command_metadata["_REPRESENTATIVE_BENCHMARK_COMMAND"] == (
        "pixi run bm lcp_compare -- --benchmark_filter="
        f"'{representative_filter}'"
    )
    assert command_metadata["_PERFORMANCE_PROFILE_REFRESH_COMMAND"] == (
        "pixi run python scripts/lcp_performance_profile.py --run "
        "--cache build/lcp_profile_full.json "
        "--output docs/background/lcp/figures "
        "--benchmark-timeout 900"
    )
    assert command_metadata["_PERFORMANCE_PROFILE_SMOKE_COMMAND"] == (
        "pixi run python scripts/lcp_performance_profile.py --run "
        "--allow-partial "
        "--benchmark-filter BM_LcpCompare/Standard/Dantzig/12 "
        "--benchmark-min-time 0.01 "
        "--cache build/lcp_profile_smoke.json "
        "--output build/lcp_profile_smoke "
        "--benchmark-timeout 120"
    )
    module.check_demo_command_metadata()


def test_lcp_solver_roster_rejects_stale_demo_command_metadata(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    stale_metadata = {
        "_BENCHMARK_SMOKE_FILTER": "BM_LCP_COMPARE_SMOKE",
        "_BENCHMARK_COMMAND": (
            "pixi run bm lcp_compare -- --benchmark_filter=BM_LCP_STALE"
        ),
        "_BENCHMARK_PACKET_ROWS": [
            {
                "packet": "standard_spd_smoke",
                "benchmark_filter": "BM_LcpCompare/Standard",
            },
            {
                "packet": "batch_scale",
                "benchmark_filter": "BM_LcpBatch|BM_LcpGroupedBatch",
            },
        ],
        "_REPRESENTATIVE_BENCHMARK_FILTER": "BM_LcpCompare/Standard",
        "_REPRESENTATIVE_BENCHMARK_COMMAND": (
            "pixi run bm lcp_compare -- "
            "--benchmark_filter='BM_LcpCompare/Standard'"
        ),
        "_PERFORMANCE_PROFILE_REFRESH_COMMAND": (
            "pixi run python scripts/lcp_performance_profile.py --run "
            "--cache build/stale.json --output build/stale --benchmark-timeout 1"
        ),
        "_PERFORMANCE_PROFILE_SMOKE_COMMAND": (
            "pixi run python scripts/lcp_performance_profile.py --run "
            "--benchmark-filter BM_LcpMissing --benchmark-min-time 0.01 "
            "--cache build/lcp_profile_smoke.json "
            "--output build/lcp_profile_smoke --benchmark-timeout 120"
        ),
    }
    monkeypatch.setattr(
        module,
        "parse_demo_command_metadata",
        lambda: stale_metadata,
    )

    with pytest.raises(
        AssertionError,
        match="demo command metadata is out of sync",
    ) as exc_info:
        module.check_demo_command_metadata()

    message = str(exc_info.value)
    assert "_BENCHMARK_COMMAND is stale" in message
    assert "_REPRESENTATIVE_BENCHMARK_FILTER is stale" in message
    assert "BM_LcpBatch|BM_LcpGroupedBatch" in message
    assert "_REPRESENTATIVE_BENCHMARK_COMMAND is stale" in message
    assert "_PERFORMANCE_PROFILE_REFRESH_COMMAND is stale" in message
    assert "_PERFORMANCE_PROFILE_SMOKE_COMMAND is stale" in message
    assert "--allow-partial" in message


def test_lcp_solver_roster_reads_demo_performance_profile_rows() -> None:
    module = _load_module()

    rows_by_surface = {
        row["surface"]: row for row in module.parse_demo_performance_profile_rows()
    }

    assert set(rows_by_surface) == {"Standard", "Boxed", "FrictionIndex"}
    assert rows_by_surface["Standard"]["artifact"] == (
        "docs/background/lcp/figures/performance_profile_standard.csv"
    )
    assert rows_by_surface["Boxed"]["problem_sizes"] == "12, 24, 48"
    assert rows_by_surface["FrictionIndex"]["problem_sizes"] == "4, 16, 64"
    assert rows_by_surface["Standard"]["evidence_artifact"] == (
        "docs/background/lcp/figures/performance_profile_evidence.csv"
    )
    module.check_demo_performance_profiles()


def test_lcp_solver_roster_rejects_stale_performance_profile_rows(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    stale_rows = [
        {
            "surface": "Standard",
            "artifact": "docs/background/lcp/figures/missing_standard.csv",
            "evidence_artifact": "docs/background/lcp/figures/stale_evidence.csv",
            "problem_sizes": "12, 999",
            "current_leaders": "leaders",
            "current_laggards": "",
            "takeaway": "takeaway",
        },
        {
            "surface": "Standard",
            "artifact": "docs/background/lcp/figures/performance_profile_standard.csv",
            "evidence_artifact": (
                "docs/background/lcp/figures/performance_profile_evidence.csv"
            ),
            "problem_sizes": "12",
            "current_leaders": "leaders",
            "current_laggards": "laggards",
            "takeaway": "takeaway",
        },
        {
            "surface": "Unknown",
            "artifact": "docs/background/lcp/figures/performance_profile_unknown.csv",
            "evidence_artifact": (
                "docs/background/lcp/figures/performance_profile_evidence.csv"
            ),
            "problem_sizes": "bad",
            "current_leaders": "leaders",
            "current_laggards": "laggards",
            "takeaway": "takeaway",
        },
    ]
    monkeypatch.setattr(
        module, "parse_demo_performance_profile_rows", lambda: stale_rows
    )

    with pytest.raises(
        AssertionError,
        match="performance profile rows are out of sync",
    ) as exc_info:
        module.check_demo_performance_profiles()

    message = str(exc_info.value)
    assert "duplicate performance profile rows ['Standard']" in message
    assert "Standard: missing fields ['current_laggards']" in message
    assert "missing_standard.csv" in message
    assert "stale_evidence.csv" in message
    assert "Standard: problem_sizes [12, 999] do not match evidence" in message
    assert "Unknown: unknown profile surface" in message
    assert "missing performance profile surfaces ['Boxed', 'FrictionIndex']" in message


def test_lcp_solver_roster_reads_demo_benchmark_filter_tokens() -> None:
    module = _load_module()

    tokens = module.parse_demo_benchmark_filter_tokens()

    assert "BM_LCP_COMPARE_SMOKE" in tokens
    assert "BM_LcpCompare/Standard/Dantzig/12" in tokens
    assert "BM_LcpNewtonWarmStart" in tokens
    assert "BM_LcpNewtonWarmStartBatchSerial" in tokens
    assert "BM_LcpNewtonWarmStartBatchParallel" in tokens
    assert "BM_LcpJacobiSolverThreading" in tokens
    assert "BM_LcpRedBlackGaussSeidelSolverThreadingBanded" in tokens
    assert "BM_LcpBlockedJacobiSolverThreadingBanded" in tokens
    assert "BM_LcpMildIllConditioned" in tokens
    assert "BM_LcpMildIllConditionedBatchSerial" in tokens
    assert "BM_LcpMildIllConditionedBatchParallel" in tokens
    assert "BM_LcpNearSingular" in tokens
    assert "BM_LcpNearSingularBatchSerial" in tokens
    assert "BM_LcpNearSingularBatchParallel" in tokens
    assert "BM_LcpValidation_Serial_FrictionIndex" in tokens
    assert "BM_LcpValidation_Threaded_FrictionIndex" in tokens
    assert "BM_LcpWorldSeparatedStep_BoxedLcp" in tokens
    assert "BM_LcpWorldBoxStep_BoxedLcp" in tokens
    assert "BM_LcpWorldBilliardsStep_BoxedLcp" in tokens
    assert "BM_LcpArticulatedUnifiedContact" in tokens
    assert "BM_LcpCudaJacobiBatch" in tokens
    assert "BM_LcpCudaPgsGroupedBatch" in tokens
    assert "BM_LcpCudaJacobiWorldContactBatch" in tokens
    assert "BM_LcpCudaPgsWorldBoxContactGroupedBatch" in tokens
    assert "BM_LcpCudaJacobiArticulatedUnifiedContactGroupedBatch" in tokens
    assert "BM_LcpCudaPgsMixedContactGroupedBatch" in tokens


def test_lcp_solver_roster_rejects_unknown_demo_benchmark_filter(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    monkeypatch.setattr(
        module,
        "parse_demo_benchmark_filter_tokens",
        lambda: ["BM_LcpCompare/Standard", "BM_LcpMissingDemoPacket"],
    )

    with pytest.raises(
        AssertionError,
        match="benchmark filters do not match BM_LCP_COMPARE benchmarks",
    ):
        module.check_demo_benchmark_filters()


def test_lcp_solver_roster_rejects_uncovered_demo_benchmark_base(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    monkeypatch.setattr(
        module,
        "parse_lcp_compare_benchmark_bases",
        lambda: {"BM_LcpCoveredBatch", "BM_LcpMissingBatch"},
    )
    monkeypatch.setattr(
        module,
        "parse_demo_benchmark_filter_tokens",
        lambda: ["BM_LcpCovered"],
    )

    with pytest.raises(
        AssertionError,
        match="registered benchmarks are not covered by lcp_physics",
    ):
        module.check_demo_benchmark_filters()


def test_lcp_solver_roster_reads_demo_live_packet_links() -> None:
    module = _load_module()

    live_by_packet = {
        row["packet"]: row for row in module.parse_demo_live_packet_rows()
    }

    assert live_by_packet["Sliding friction"]["metric"] == "sliding speed, contacts"
    assert live_by_packet["Static-friction ramp"]["benchmark"] == (
        "active-set transition packets"
    )
    assert live_by_packet["Billiard collision"]["metric"] == (
        "symmetry, momentum, kinetic-energy error"
    )
    module.check_demo_live_packets()


def test_lcp_solver_roster_rejects_stale_live_packet_metadata(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    monkeypatch.setattr(
        module,
        "parse_demo_live_packet_rows",
        lambda: [
            {
                "packet": "Known live packet",
                "metric": "metric",
                "benchmark": "benchmark analog",
            },
            {
                "packet": "Duplicate live packet",
                "metric": "",
                "benchmark": "benchmark analog",
            },
            {
                "packet": "Duplicate live packet",
                "metric": "metric",
                "benchmark": "",
            },
            {
                "packet": "Uncovered live packet",
                "metric": "metric",
                "benchmark": "benchmark analog",
            },
        ],
    )
    monkeypatch.setattr(
        module,
        "parse_demo_representative_requirement_rows",
        lambda: [
            {
                "requirement": "Known requirement",
                "live_packet": "Known live packet",
                "benchmark_packet": "known_benchmark_packet",
                "metrics": "metric",
                "evidence": "evidence",
            }
        ],
    )

    with pytest.raises(
        AssertionError,
        match="live packet rows are out of sync",
    ) as exc_info:
        module.check_demo_live_packets()

    message = str(exc_info.value)
    assert "duplicate live packet rows ['Duplicate live packet']" in message
    assert "Duplicate live packet: missing fields ['metric']" in message
    assert "Duplicate live packet: missing fields ['benchmark']" in message
    assert "live packets missing representative requirement coverage" in message
    assert "Duplicate live packet" in message
    assert "Uncovered live packet" in message


def test_lcp_solver_roster_reads_demo_representative_requirement_links() -> None:
    module = _load_module()

    live_packets = {row["packet"] for row in module.parse_demo_live_packet_rows()}
    benchmark_packets = {
        row["packet"] for row in module.parse_demo_benchmark_packet_rows()
    }
    standalone_problem_cases = set(module.parse_demo_standalone_problem_case_names())
    requirement_by_name = {
        row["requirement"]: row
        for row in module.parse_demo_representative_requirement_rows()
    }

    assert "Sliding friction" in live_packets
    assert "world_stack" in benchmark_packets
    assert "mass_ratio_boxed" in standalone_problem_cases
    assert requirement_by_name["Scalability smoke"]["live_packet"] == (
        module.parse_demo_standalone_problem_suite_label()
    )
    assert set(
        module._split_demo_reference_list(
            requirement_by_name["High mass-ratio stack"]["benchmark_packet"]
        )
    ) == {"world_stack", "mass_ratio_boxed"}
    module.check_demo_representative_requirements()


def test_lcp_solver_roster_reads_demo_standalone_problem_cases() -> None:
    module = _load_module()

    cases_by_name = {
        row["name"]: row for row in module.parse_demo_standalone_problem_case_rows()
    }

    assert cases_by_name["standard_spd"]["label"] == "Standard SPD"
    assert cases_by_name["standard_spd"]["surface"] == "standard"
    assert cases_by_name["standard_spd"]["support_key"] == "standard"
    assert cases_by_name["standard_spd"]["make_problem"] == "_make_standard_spd_case"
    assert cases_by_name["standard_spd"]["tolerance"] == pytest.approx(1e-4)
    assert cases_by_name["mass_ratio_boxed"]["surface"] == "boxed"
    assert cases_by_name["mass_ratio_boxed"]["tolerance"] == pytest.approx(5e-4)
    assert cases_by_name["friction_index_contact"]["support_key"] == "findex"
    assert cases_by_name["active_friction_index_contact"]["challenge"] == (
        "two-contact active tangential bounds with coupled normals"
    )
    module.check_demo_standalone_problem_cases()


def test_lcp_solver_roster_rejects_stale_standalone_problem_cases(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    stale_rows = [
        {
            "name": "duplicate_case",
            "label": "Duplicate case",
            "surface": "standard",
            "support_key": "boxed",
            "challenge": "",
            "make_problem": "_missing_case",
            "tolerance": 0.0,
        },
        {
            "name": "duplicate_case",
            "label": "Duplicate case",
            "surface": "mystery",
            "support_key": "unknown",
            "challenge": "stale metadata",
            "make_problem": "_make_standard_spd_case",
            "tolerance": float("nan"),
        },
    ]
    monkeypatch.setattr(
        module,
        "parse_demo_standalone_problem_case_rows",
        lambda: stale_rows,
    )
    monkeypatch.setattr(module, "parse_demo_standalone_problem_suite_label", lambda: "")
    monkeypatch.setattr(
        module,
        "parse_demo_function_names",
        lambda: {"_make_standard_spd_case"},
    )

    with pytest.raises(
        AssertionError,
        match="standalone problem cases are out of sync",
    ) as exc_info:
        module.check_demo_standalone_problem_cases()

    message = str(exc_info.value)
    assert "_STANDALONE_PROBLEM_SUITE_LABEL is blank" in message
    assert "duplicate standalone problem names ['duplicate_case']" in message
    assert "duplicate standalone problem labels ['Duplicate case']" in message
    assert "duplicate_case: missing fields ['challenge']" in message
    assert "surface 'standard' does not match support_key 'boxed'" in message
    assert "unknown make_problem function '_missing_case'" in message
    assert "unknown surface 'mystery'" in message
    assert "unknown support_key 'unknown'" in message
    assert "tolerance must be positive and finite" in message
    assert "missing representative surfaces ['boxed', 'findex']" in message


def test_lcp_solver_roster_rejects_malformed_standalone_problem_case_ast(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    malformed_cases = ast.parse(
        "_STANDALONE_PROBLEM_CASES = ("
        "_StandaloneProblemCase(name='missing_fields'),"
        ")"
    ).body[0].value
    monkeypatch.setattr(
        module,
        "_assignment_value",
        lambda _module, _name: malformed_cases,
    )

    with pytest.raises(AssertionError, match="missing required keywords"):
        module.parse_demo_standalone_problem_case_rows()


def test_lcp_solver_roster_rejects_stale_requirement_live_packet(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    monkeypatch.setattr(
        module,
        "parse_demo_live_packet_rows",
        lambda: [{"packet": "Known live packet"}],
    )
    monkeypatch.setattr(
        module,
        "parse_demo_benchmark_packet_rows",
        lambda: [{"packet": "known_benchmark_packet"}],
    )
    monkeypatch.setattr(module, "parse_demo_standalone_problem_case_names", lambda: [])
    monkeypatch.setattr(
        module,
        "parse_demo_standalone_problem_suite_label",
        lambda: "Representative solver suite",
    )
    monkeypatch.setattr(
        module,
        "parse_demo_representative_requirement_rows",
        lambda: [
            {
                "requirement": "Stale live reference",
                "live_packet": "Missing live packet",
                "benchmark_packet": "known_benchmark_packet",
                "metrics": "metric",
                "evidence": "evidence",
            }
        ],
    )

    with pytest.raises(AssertionError, match="unknown live packet references"):
        module.check_demo_representative_requirements()


def test_lcp_solver_roster_rejects_stale_requirement_benchmark_packet(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    monkeypatch.setattr(
        module,
        "parse_demo_live_packet_rows",
        lambda: [{"packet": "Known live packet"}],
    )
    monkeypatch.setattr(
        module,
        "parse_demo_benchmark_packet_rows",
        lambda: [{"packet": "known_benchmark_packet"}],
    )
    monkeypatch.setattr(module, "parse_demo_standalone_problem_case_names", lambda: [])
    monkeypatch.setattr(
        module,
        "parse_demo_standalone_problem_suite_label",
        lambda: "Representative solver suite",
    )
    monkeypatch.setattr(
        module,
        "parse_demo_representative_requirement_rows",
        lambda: [
            {
                "requirement": "Stale benchmark reference",
                "live_packet": "Known live packet",
                "benchmark_packet": "missing_benchmark_packet",
                "metrics": "metric",
                "evidence": "evidence",
            }
        ],
    )

    with pytest.raises(
        AssertionError,
        match="unknown benchmark/standalone references",
    ):
        module.check_demo_representative_requirements()


def test_lcp_solver_roster_reads_demo_standalone_smoke_metadata() -> None:
    module = _load_module()

    metadata = module.parse_demo_standalone_smoke_metadata()

    assert metadata["_STANDALONE_LCP_SOLVERS_EXPOSED_IN_DARTPY"] is True
    assert metadata["_STANDALONE_SMOKE_EXPECTED"] == (1.0, 0.5, 2.0)
    assert metadata["_STANDALONE_SUCCESS_STATUSES"] == {"Success", "MaxIterations"}
    module.check_demo_standalone_smoke_metadata()


def test_lcp_solver_roster_rejects_stale_standalone_smoke_metadata(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    stale_metadata = {
        "_STANDALONE_LCP_SOLVERS_EXPOSED_IN_DARTPY": False,
        "_STANDALONE_SMOKE_EXPECTED": (1.0, 0.5, 3.0),
        "_STANDALONE_SUCCESS_STATUSES": {"Failure", "Success"},
    }
    monkeypatch.setattr(
        module,
        "parse_demo_standalone_smoke_metadata",
        lambda: stale_metadata,
    )

    with pytest.raises(
        AssertionError,
        match="standalone smoke metadata is out of sync",
    ) as exc_info:
        module.check_demo_standalone_smoke_metadata()

    message = str(exc_info.value)
    assert "_STANDALONE_LCP_SOLVERS_EXPOSED_IN_DARTPY must be True" in message
    assert "_STANDALONE_SMOKE_EXPECTED is stale" in message
    assert "_STANDALONE_SUCCESS_STATUSES is stale" in message
    assert "Failure" in message


def test_lcp_solver_roster_reads_demo_solver_guidance_links() -> None:
    module = _load_module()
    manifest = module.parse_cpp_manifest()

    guidance_by_family = {
        row["family"]: row for row in module.parse_demo_solver_guidance_rows()
    }
    guided_solvers = [
        solver
        for row in guidance_by_family.values()
        for solver in module._split_demo_solver_list(row["solvers"])
    ]

    assert guidance_by_family["Pivoting and direct"]["solvers"] == (
        "Direct, Dantzig, Lemke, Baraff"
    )
    assert "BoxedSemiSmoothNewton" in guided_solvers
    assert "Sap" in guided_solvers
    assert sorted(guided_solvers) == sorted(entry.name for entry in manifest)
    assert len(guided_solvers) == len(manifest)
    module.check_demo_solver_guidance(manifest)


def test_lcp_solver_roster_rejects_stale_solver_guidance(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    manifest = module.parse_cpp_manifest()
    stale_rows = [
        {
            "family": "Pivoting and direct",
            "solvers": "Dantzig, MissingSolver, Dantzig",
            "best_fit": "small rows",
            "strength": "reference solves",
            "tradeoff": "scales cubically",
            "evidence": "profile rows",
        }
    ]
    monkeypatch.setattr(module, "parse_demo_solver_guidance_rows", lambda: stale_rows)

    with pytest.raises(
        AssertionError,
        match="solver guidance rows are out of sync",
    ) as exc_info:
        module.check_demo_solver_guidance(manifest)

    message = str(exc_info.value)
    assert "unknown solvers ['MissingSolver']" in message
    assert "duplicate solver guidance entries ['Dantzig']" in message
    assert "missing solver guidance entries" in message
    assert "Lemke" in message


def test_lcp_solver_roster_reads_demo_advanced_solver_parameter_links() -> None:
    module = _load_module()
    manifest = module.parse_cpp_manifest()

    parameter_by_solver = {
        row["solver"]: row for row in module.parse_demo_advanced_solver_parameter_rows()
    }

    assert parameter_by_solver["Pgs"]["surface"] == "boxed/findex"
    assert parameter_by_solver["Pgs"]["benchmark_filter"] == "BM_LcpPgsRelaxationSweep"
    assert "epsilon_for_division" in parameter_by_solver["Pgs"]["parameters"]
    assert parameter_by_solver["SymmetricPsor"]["surface"] == "standard"
    assert parameter_by_solver["BoxedSemiSmoothNewton"]["benchmark_filter"] == (
        "BM_LcpBoxedSemiSmoothNewtonLineSearchSweep"
    )
    assert "regularization" in parameter_by_solver["Sap"]["parameters"]
    module.check_demo_advanced_solver_parameters(manifest)


def test_lcp_solver_roster_rejects_stale_advanced_solver_parameter_links(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    manifest = module.parse_cpp_manifest()
    stale_rows = [
        {
            "solver": "MissingSolver",
            "surface": "standard",
            "parameters": "epsilon_for_division",
            "benchmark_filter": "BM_LcpMissingParameterSweep",
        },
        {
            "solver": "InteriorPoint",
            "surface": "boxed/findex",
            "parameters": "sigma",
            "benchmark_filter": "BM_LcpInteriorPointPathSweep",
        },
    ]
    monkeypatch.setattr(
        module,
        "parse_demo_advanced_solver_parameter_rows",
        lambda: stale_rows,
    )

    with pytest.raises(
        AssertionError,
        match="advanced solver parameter rows are out of sync",
    ) as exc_info:
        module.check_demo_advanced_solver_parameters(manifest)

    message = str(exc_info.value)
    assert "MissingSolver: unknown solver" in message
    assert "unknown benchmark filters ['BM_LcpMissingParameterSweep']" in message
    assert "solver_parameter_sweeps packet ['BM_LcpMissingParameterSweep']" in message
    assert "InteriorPoint: surface 'boxed/findex' is not supported" in message


def test_lcp_solver_roster_rejects_extra_bound_solver_class(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    monkeypatch.setattr(
        module,
        "parse_bound_solver_classes",
        lambda: {
            "DantzigSolver": "DantzigSolver",
            "ExtraSolver": "ExtraSolver",
        },
    )

    with pytest.raises(
        AssertionError,
        match="dartpy bindings contain non-manifest LCP solver classes",
    ):
        module.check_bound_solver_classes(["DantzigSolver"])


def test_lcp_solver_roster_reads_bound_lcp_parameter_members() -> None:
    module = _load_module()
    members = module.parse_bound_lcp_parameter_members()

    assert "epsilon_for_division" in members["PgsSolverParameters"]
    assert "randomize_constraint_order" in members["PgsSolverParameters"]
    assert "rho_init" in members["AdmmSolverParameters"]
    assert "regularization" in members["SapSolverParameters"]
    assert (
        "max_friction_index_exact_solve_dimension"
        in members["BoxedSemiSmoothNewtonSolverParameters"]
    )


def test_lcp_solver_roster_reads_bound_parameterized_solver_classes() -> None:
    module = _load_module()
    bound = module.parse_bound_parameterized_solver_classes()

    assert bound["PgsSolver"] == "PgsSolver"
    assert bound["BoxedSemiSmoothNewtonSolver"] == "BoxedSemiSmoothNewtonSolver"
    assert bound["SapSolver"] == "SapSolver"
    assert "DantzigSolver" not in bound
    assert "StaggeringSolver" not in bound


def test_lcp_solver_roster_rejects_missing_math_stub_class(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    monkeypatch.setattr(
        module,
        "parse_math_stub_solver_classes",
        lambda: {"DantzigSolver"},
    )

    with pytest.raises(
        AssertionError,
        match="python/stubs/dartpy/math\\.pyi is missing solver classes",
    ):
        module.check_python_stub_solver_classes(["DantzigSolver", "LemkeSolver"])


def test_lcp_solver_roster_rejects_extra_math_stub_class(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    monkeypatch.setattr(
        module,
        "parse_math_stub_solver_classes",
        lambda: {"DantzigSolver", "LemkeSolver", "ExtraSolver"},
    )
    monkeypatch.setattr(
        module,
        "parse_math_stub_all_names",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )
    monkeypatch.setattr(
        module,
        "parse_init_stub_math_imports",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )
    monkeypatch.setattr(
        module,
        "parse_init_stub_all_names",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )

    with pytest.raises(
        AssertionError,
        match="python/stubs/dartpy/math\\.pyi contains non-manifest solver classes",
    ):
        module.check_python_stub_solver_classes(["DantzigSolver", "LemkeSolver"])


def test_lcp_solver_roster_rejects_missing_math_all_class(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    monkeypatch.setattr(
        module,
        "parse_math_stub_solver_classes",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )
    monkeypatch.setattr(module, "parse_math_stub_all_names", lambda: {"DantzigSolver"})
    monkeypatch.setattr(
        module,
        "parse_init_stub_math_imports",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )
    monkeypatch.setattr(
        module,
        "parse_init_stub_all_names",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )

    with pytest.raises(
        AssertionError,
        match="python/stubs/dartpy/math\\.pyi __all__ is missing solver classes",
    ):
        module.check_python_stub_solver_classes(["DantzigSolver", "LemkeSolver"])


def test_lcp_solver_roster_rejects_missing_init_stub_class(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    monkeypatch.setattr(
        module,
        "parse_math_stub_solver_classes",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )
    monkeypatch.setattr(
        module,
        "parse_math_stub_all_names",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )
    monkeypatch.setattr(
        module,
        "parse_init_stub_math_imports",
        lambda: {"DantzigSolver"},
    )
    monkeypatch.setattr(
        module,
        "parse_init_stub_all_names",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )

    with pytest.raises(
        AssertionError,
        match="python/stubs/dartpy/__init__\\.pyi is missing \\.math imports",
    ):
        module.check_python_stub_solver_classes(["DantzigSolver", "LemkeSolver"])


def test_lcp_solver_roster_rejects_missing_init_all_class(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    monkeypatch.setattr(
        module,
        "parse_math_stub_solver_classes",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )
    monkeypatch.setattr(
        module,
        "parse_math_stub_all_names",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )
    monkeypatch.setattr(
        module,
        "parse_init_stub_math_imports",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )
    monkeypatch.setattr(
        module,
        "parse_init_stub_all_names",
        lambda: {"DantzigSolver"},
    )

    with pytest.raises(
        AssertionError,
        match="python/stubs/dartpy/__init__\\.pyi __all__ is missing solver classes",
    ):
        module.check_python_stub_solver_classes(["DantzigSolver", "LemkeSolver"])


def test_lcp_solver_roster_rejects_missing_math_stub_lcp_api_member(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()

    monkeypatch.setattr(module, "parse_bound_lcp_parameter_members", lambda: {})
    monkeypatch.setattr(module, "parse_bound_parameterized_solver_classes", lambda: {})

    def fake_class_members(class_name: str) -> set[str]:
        members = set(module.REQUIRED_DARTPY_MATH_STUB_MEMBERS[class_name])
        if class_name == "LcpProblem":
            members.remove("get_friction_index_row_count")
        return members

    monkeypatch.setattr(module, "parse_math_stub_class_members", fake_class_members)

    with pytest.raises(
        AssertionError,
        match="LcpProblem is missing members \\['get_friction_index_row_count'\\]",
    ):
        module.check_python_stub_lcp_api_surface()


def test_lcp_solver_roster_rejects_missing_math_stub_parameter_member(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    monkeypatch.setattr(
        module,
        "parse_bound_lcp_parameter_members",
        lambda: {
            "BoxedSemiSmoothNewtonSolverParameters": {
                "max_friction_index_exact_solve_dimension"
            }
        },
    )
    monkeypatch.setattr(
        module,
        "parse_bound_parameterized_solver_classes",
        lambda: {"BoxedSemiSmoothNewtonSolver": "BoxedSemiSmoothNewtonSolver"},
    )

    def fake_class_members(class_name: str) -> set[str]:
        if class_name == "BoxedSemiSmoothNewtonSolverParameters":
            return set()
        return set(module.REQUIRED_DARTPY_MATH_STUB_MEMBERS[class_name])

    monkeypatch.setattr(module, "parse_math_stub_class_members", fake_class_members)

    with pytest.raises(
        AssertionError,
        match=(
            "BoxedSemiSmoothNewtonSolverParameters is missing members "
            "\\['max_friction_index_exact_solve_dimension'\\]"
        ),
    ):
        module.check_python_stub_lcp_api_surface()


def test_lcp_solver_roster_rejects_missing_parameter_class_for_parameterized_solver(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    monkeypatch.setattr(module, "parse_bound_lcp_parameter_members", lambda: {})
    monkeypatch.setattr(
        module,
        "parse_bound_parameterized_solver_classes",
        lambda: {"PgsSolver": "PgsSolver"},
    )

    def fake_class_members(class_name: str) -> set[str]:
        return set(module.REQUIRED_DARTPY_MATH_STUB_MEMBERS[class_name])

    monkeypatch.setattr(module, "parse_math_stub_class_members", fake_class_members)

    with pytest.raises(
        AssertionError,
        match="PgsSolver parameters property has no bound PgsSolverParameters class",
    ):
        module.check_python_stub_lcp_api_surface()


def test_lcp_solver_roster_rejects_wrong_solver_parameter_stub_annotation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    monkeypatch.setattr(
        module,
        "parse_bound_lcp_parameter_members",
        lambda: {"PgsSolverParameters": {"epsilon_for_division"}},
    )
    monkeypatch.setattr(
        module,
        "parse_bound_parameterized_solver_classes",
        lambda: {"PgsSolver": "PgsSolver"},
    )

    def fake_class_members(class_name: str) -> set[str]:
        if class_name == "PgsSolverParameters":
            return {"epsilon_for_division"}
        return set(module.REQUIRED_DARTPY_MATH_STUB_MEMBERS[class_name])

    monkeypatch.setattr(module, "parse_math_stub_class_members", fake_class_members)
    monkeypatch.setattr(
        module,
        "parse_math_stub_class_annotations",
        lambda _: {"parameters": "WrongParameters"},
    )

    with pytest.raises(
        AssertionError,
        match=(
            "PgsSolver\\.parameters is annotated as 'WrongParameters'; "
            "expected 'PgsSolverParameters'"
        ),
    ):
        module.check_python_stub_lcp_api_surface()


def test_lcp_profile_evidence_rejects_solver_identity_mismatch(
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = module.parse_cpp_manifest()
    path = tmp_path / "performance_profile_evidence.csv"

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=module.REQUIRED_EVIDENCE_COLUMNS)
        writer.writeheader()
        writer.writerow(_valid_evidence_row() | {"solver_manifest_index": "5"})

    with pytest.raises(AssertionError, match="solver_manifest_index"):
        module.check_performance_profile_evidence(manifest, path)


def test_lcp_profile_headers_reject_missing_native_solver(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = [
        module.SolverEntry("Dantzig", "Pivoting", True, False, False, "DantzigSolver"),
        module.SolverEntry("Lemke", "Pivoting", True, False, False, "LemkeSolver"),
    ]
    path = tmp_path / "performance_profile_standard.csv"
    path.write_text("tau,Dantzig\n1,1.0\n", encoding="utf-8")
    monkeypatch.setattr(module, "LCP_PROFILE_CSV_PATHS", {"standard": path})

    with pytest.raises(AssertionError, match="missing native standard solvers"):
        module.check_performance_profile_headers(manifest)


def test_lcp_profile_headers_reject_invalid_header(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = [
        module.SolverEntry("Dantzig", "Pivoting", True, False, False, "DantzigSolver"),
    ]
    path = tmp_path / "performance_profile_standard.csv"
    path.write_text("scale,Dantzig\n1,1.0\n", encoding="utf-8")
    monkeypatch.setattr(module, "LCP_PROFILE_CSV_PATHS", {"standard": path})

    with pytest.raises(AssertionError, match="invalid header"):
        module.check_performance_profile_headers(manifest)


def test_lcp_profile_headers_reject_unknown_solver(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = [
        module.SolverEntry("Dantzig", "Pivoting", True, False, False, "DantzigSolver"),
    ]
    path = tmp_path / "performance_profile_standard.csv"
    path.write_text("tau,Dantzig,Ghost\n1,1.0,2.0\n", encoding="utf-8")
    monkeypatch.setattr(module, "LCP_PROFILE_CSV_PATHS", {"standard": path})

    with pytest.raises(AssertionError, match="contains unknown solvers"):
        module.check_performance_profile_headers(manifest)


def test_lcp_profile_headers_reject_duplicate_solver_column(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = [
        module.SolverEntry("Dantzig", "Pivoting", True, False, False, "DantzigSolver"),
    ]
    path = tmp_path / "performance_profile_standard.csv"
    path.write_text("tau,Dantzig,Dantzig\n1,1.0,1.1\n", encoding="utf-8")
    monkeypatch.setattr(module, "LCP_PROFILE_CSV_PATHS", {"standard": path})

    with pytest.raises(AssertionError, match="contains duplicate solver columns"):
        module.check_performance_profile_headers(manifest)


def test_lcp_profile_headers_reject_non_native_solver(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = [
        module.SolverEntry("Dantzig", "Pivoting", True, False, False, "DantzigSolver"),
    ]
    path = tmp_path / "performance_profile_boxed.csv"
    path.write_text("tau,Dantzig\n1,1.0\n", encoding="utf-8")
    monkeypatch.setattr(module, "LCP_PROFILE_CSV_PATHS", {"boxed": path})

    with pytest.raises(AssertionError, match="contains non-native boxed solvers"):
        module.check_performance_profile_headers(manifest)


@pytest.mark.parametrize(
    ("csv_text", "expected_error"),
    [
        ("tau,Dantzig\n0,1.0\n", "invalid tau"),
        ("tau,Dantzig\n1,1.0\n1,1.0\n", "non-increasing tau"),
        ("tau,Dantzig\n1,1.1\n", "invalid profile value"),
        ("tau,Dantzig\n1\n", "columns; expected 2"),
    ],
)
def test_lcp_profile_headers_reject_invalid_profile_rows(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    csv_text: str,
    expected_error: str,
) -> None:
    module = _load_module()
    manifest = [
        module.SolverEntry("Dantzig", "Pivoting", True, False, False, "DantzigSolver"),
    ]
    path = tmp_path / "performance_profile_standard.csv"
    path.write_text(csv_text, encoding="utf-8")
    monkeypatch.setattr(module, "LCP_PROFILE_CSV_PATHS", {"standard": path})

    with pytest.raises(AssertionError, match=expected_error):
        module.check_performance_profile_headers(manifest)


def test_lcp_profile_evidence_rejects_missing_native_solver(
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = [
        module.SolverEntry("Dantzig", "Pivoting", True, False, False, "DantzigSolver"),
        module.SolverEntry("Lemke", "Pivoting", True, False, False, "LemkeSolver"),
    ]
    path = tmp_path / "performance_profile_evidence.csv"

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=module.REQUIRED_EVIDENCE_COLUMNS)
        writer.writeheader()
        writer.writerow(
            _valid_evidence_row()
            | {
                "solver_supports_boxed": "0",
                "solver_supports_friction_index": "0",
            }
        )

    with pytest.raises(AssertionError, match="missing native profile evidence"):
        module.check_performance_profile_evidence(manifest, path)


def test_lcp_profile_evidence_rejects_duplicate_column(
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = module.parse_cpp_manifest()
    path = tmp_path / "performance_profile_evidence.csv"
    fieldnames = (*module.REQUIRED_EVIDENCE_COLUMNS, "time_ns")

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerow(_valid_evidence_row())

    with pytest.raises(AssertionError, match="contains duplicate evidence columns"):
        module.check_performance_profile_evidence(manifest, path)


def test_lcp_profile_evidence_rejects_duplicate_row(
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = [
        module.SolverEntry("Dantzig", "Pivoting", True, False, False, "DantzigSolver"),
    ]
    row = _valid_evidence_row() | {
        "solver_supports_boxed": "0",
        "solver_supports_friction_index": "0",
    }
    path = tmp_path / "performance_profile_evidence.csv"

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=module.REQUIRED_EVIDENCE_COLUMNS)
        writer.writeheader()
        writer.writerow(row)
        writer.writerow(row)

    with pytest.raises(AssertionError, match="duplicate evidence row"):
        module.check_performance_profile_evidence(manifest, path)


def test_lcp_profile_evidence_rejects_invalid_metric(
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = module.parse_cpp_manifest()
    path = tmp_path / "performance_profile_evidence.csv"

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=module.REQUIRED_EVIDENCE_COLUMNS)
        writer.writeheader()
        writer.writerow(_valid_evidence_row() | {"bound_violation": ""})

    with pytest.raises(AssertionError, match="bound_violation"):
        module.check_performance_profile_evidence(manifest, path)


def test_lcp_profile_evidence_rejects_solver_family_mismatch(
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = module.parse_cpp_manifest()
    path = tmp_path / "performance_profile_evidence.csv"

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=module.REQUIRED_EVIDENCE_COLUMNS)
        writer.writeheader()
        writer.writerow(
            _valid_evidence_row()
            | {"solver_family_pivoting": "0", "solver_family_projection": "1"}
        )

    with pytest.raises(AssertionError, match="solver_family_projection"):
        module.check_performance_profile_evidence(manifest, path)


def test_lcp_profile_evidence_rejects_unsupported_solver_category(
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = module.parse_cpp_manifest()
    path = tmp_path / "performance_profile_evidence.csv"

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=module.REQUIRED_EVIDENCE_COLUMNS)
        writer.writeheader()
        writer.writerow(
            _valid_evidence_row()
            | {
                "category": "Boxed",
                "solver": "Lemke",
                "solver_manifest_index": "2",
                "solver_supports_standard": "1",
                "solver_supports_boxed": "0",
                "solver_supports_friction_index": "0",
                "solver_supports_problem": "0",
                "problem_type_standard": "0",
                "problem_type_boxed": "1",
            }
        )

    with pytest.raises(AssertionError, match="not native-supported"):
        module.check_performance_profile_evidence(manifest, path)


def test_lcp_profile_evidence_rejects_problem_dimension_mismatch(
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = module.parse_cpp_manifest()
    path = tmp_path / "performance_profile_evidence.csv"

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=module.REQUIRED_EVIDENCE_COLUMNS)
        writer.writeheader()
        writer.writerow(_valid_evidence_row() | {"lcp_dimension": "99"})

    with pytest.raises(AssertionError, match="lcp_dimension"):
        module.check_performance_profile_evidence(manifest, path)


def test_lcp_profile_evidence_rejects_nonfinite_integer_counter(
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = module.parse_cpp_manifest()
    path = tmp_path / "performance_profile_evidence.csv"

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=module.REQUIRED_EVIDENCE_COLUMNS)
        writer.writeheader()
        writer.writerow(_valid_evidence_row() | {"problem_size": "inf"})

    with pytest.raises(AssertionError, match="invalid problem_size 'inf'"):
        module.check_performance_profile_evidence(manifest, path)
