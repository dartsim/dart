import importlib.util
from pathlib import Path

import pytest


def load_allocator_check_module():
    repo_root = Path(__file__).resolve().parents[1]
    script_path = repo_root / "scripts" / "check_allocator_comparative_benchmarks.py"
    spec = importlib.util.spec_from_file_location(
        "allocator_comparative_check", script_path
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def aggregate_row(name, aggregate, cpu_time):
    return {
        "run_name": name,
        "run_type": "aggregate",
        "aggregate_name": aggregate,
        "cpu_time": cpu_time,
        "real_time": cpu_time,
        "time_unit": "ns",
    }


def stl_vector_rows(*, dart_time, foonathan_time, dart_cv, foonathan_cv):
    return [
        aggregate_row("BM_StlVector_DART/10000", "median", dart_time),
        aggregate_row("BM_StlVector_DART/10000", "cv", dart_cv),
        aggregate_row("BM_StlVector_Foonathan/10000", "median", foonathan_time),
        aggregate_row("BM_StlVector_Foonathan/10000", "cv", foonathan_cv),
    ]


def entt_registry_rows(*, dart_time, foonathan_time, dart_cv, foonathan_cv):
    return [
        aggregate_row("BM_EnttRegistry_DART/256", "median", dart_time),
        aggregate_row("BM_EnttRegistry_DART/256", "cv", dart_cv),
        aggregate_row("BM_EnttRegistry_Foonathan/256", "median", foonathan_time),
        aggregate_row("BM_EnttRegistry_Foonathan/256", "cv", foonathan_cv),
    ]


def entt_registry_build_rows(*, dart_time, foonathan_time, dart_cv, foonathan_cv):
    return [
        aggregate_row("BM_EnttRegistryBuild_DART/256", "median", dart_time),
        aggregate_row("BM_EnttRegistryBuild_DART/256", "cv", dart_cv),
        aggregate_row("BM_EnttRegistryBuild_Foonathan/256", "median", foonathan_time),
        aggregate_row("BM_EnttRegistryBuild_Foonathan/256", "cv", foonathan_cv),
    ]


def test_noisy_rows_fail_before_timing_ratio_is_used():
    module = load_allocator_check_module()

    failures, passes = module.evaluate_comparisons(
        stl_vector_rows(
            dart_time=90.0,
            foonathan_time=100.0,
            dart_cv=0.17,
            foonathan_cv=0.20,
        ),
        baseline_allocators=[("Foonathan",)],
        max_cv=0.10,
    )

    assert passes == []
    assert failures == [
        {
            "benchmark": "BM_StlVector/10000",
            "baseline": "Foonathan",
            "noisy_allocators": [("DART", 0.17), ("Foonathan", 0.20)],
            "max_cv": 0.10,
            "status": "NOISY",
        }
    ]


def test_stable_rows_still_use_strict_ratio():
    module = load_allocator_check_module()

    failures, passes = module.evaluate_comparisons(
        stl_vector_rows(
            dart_time=101.0,
            foonathan_time=100.0,
            dart_cv=0.02,
            foonathan_cv=0.02,
        ),
        baseline_allocators=[("Foonathan",)],
        max_cv=0.10,
    )

    assert passes == []
    assert failures[0]["status"] == "FAIL"
    assert failures[0]["ratio"] == 1.01


def test_noise_guard_allows_clean_fast_rows_to_pass():
    module = load_allocator_check_module()

    failures, passes = module.evaluate_comparisons(
        stl_vector_rows(
            dart_time=90.0,
            foonathan_time=100.0,
            dart_cv=0.02,
            foonathan_cv=0.02,
        ),
        baseline_allocators=[("Foonathan",)],
        max_cv=0.10,
    )

    assert failures == []
    assert passes[0]["status"] == "PASS"


def test_benchmark_filter_modes_are_explicit():
    module = load_allocator_check_module()

    assert (
        module.benchmark_filter_for_mode(
            include_entt_registry=False,
            only_entt_registry=False,
        )
        == module.DEFAULT_FILTER
    )
    assert (
        module.benchmark_filter_for_mode(
            include_entt_registry=True,
            only_entt_registry=False,
        )
        == module.ENTT_REGISTRY_FILTER
    )
    assert (
        module.benchmark_filter_for_mode(
            include_entt_registry=False,
            only_entt_registry=True,
        )
        == module.ENTT_REGISTRY_ONLY_FILTER
    )


def test_input_rows_are_filtered_by_entt_registry_mode():
    module = load_allocator_check_module()
    rows = (
        stl_vector_rows(
            dart_time=90.0,
            foonathan_time=100.0,
            dart_cv=0.02,
            foonathan_cv=0.02,
        )
        + entt_registry_rows(
            dart_time=130.0,
            foonathan_time=100.0,
            dart_cv=0.02,
            foonathan_cv=0.02,
        )
        + entt_registry_build_rows(
            dart_time=140.0,
            foonathan_time=100.0,
            dart_cv=0.02,
            foonathan_cv=0.02,
        )
    )

    default_rows = module.filter_benchmark_rows_for_mode(
        rows,
        include_entt_registry=False,
        only_entt_registry=False,
    )
    failures, passes = module.evaluate_comparisons(
        default_rows,
        baseline_allocators=[("Foonathan",)],
        max_cv=0.10,
    )

    assert failures == []
    assert [result["benchmark"] for result in passes] == ["BM_StlVector/10000"]

    include_rows = module.filter_benchmark_rows_for_mode(
        rows,
        include_entt_registry=True,
        only_entt_registry=False,
    )
    failures, passes = module.evaluate_comparisons(
        include_rows,
        baseline_allocators=[("Foonathan",)],
        max_cv=0.10,
    )

    assert [result["benchmark"] for result in passes] == ["BM_StlVector/10000"]
    assert [result["benchmark"] for result in failures] == [
        "BM_EnttRegistry/256",
        "BM_EnttRegistryBuild/256",
    ]

    only_rows = module.filter_benchmark_rows_for_mode(
        rows,
        include_entt_registry=False,
        only_entt_registry=True,
    )
    failures, passes = module.evaluate_comparisons(
        only_rows,
        baseline_allocators=[("Foonathan",)],
        max_cv=0.10,
    )

    assert passes == []
    assert [result["benchmark"] for result in failures] == [
        "BM_EnttRegistry/256",
        "BM_EnttRegistryBuild/256",
    ]


def test_entt_registry_modes_are_mutually_exclusive():
    module = load_allocator_check_module()

    with pytest.raises(SystemExit):
        module.parse_args(["--include-entt-registry", "--only-entt-registry"])
