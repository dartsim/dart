#!/usr/bin/env python3
"""Generate a static DART performance dashboard from benchmark JSON."""

from __future__ import annotations

import argparse
import html
import json
import math
import os
import re
import shutil
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

_UNIT_TO_NS = {"ns": 1.0, "us": 1e3, "ms": 1e6, "s": 1e9}
_EXTERNAL_COMPETITOR_BACKENDS = {
    "Chrono",
    "Drake",
    "Gazebo",
    "GzPhysics",
    "Isaac",
    "IsaacGym",
    "MuJoCo",
    "PyBullet",
    "RaiSim",
    "Simbody",
}
_EXTERNAL_COMPETITOR_EXAMPLE_ROWS = [
    "BM_Distance_BoxSphere_Native",
    "BM_Distance_BoxSphere_MuJoCo",
    "BM_KR5ForwardDynamics_DART/1",
    "BM_KR5ForwardDynamics_Drake/1",
]
_KNOWN_BACKENDS = {
    "Native",
    "FCL",
    "Bullet",
    "ODE",
    "SimBody",
    "Baseline",
    "Optimized",
    "DART",
    "Std",
    "StdPmr",
    "Pool",
    "Foonathan",
    "Eigen",
    "SIMD",
} | _EXTERNAL_COMPETITOR_BACKENDS
_PRIMARY_BACKENDS = ("Native", "Optimized", "DART", "Pool", "SIMD")
_SCHEMA_VERSION = 1
_CHANGE_THRESHOLD_PERCENT = 1.0
_EXPECTED_UPDATE_INTERVAL_HOURS = 96.0
_STALE_AFTER_HOURS = 120.0
_CANONICAL_WEBSITE_URL = "https://dart.readthedocs.io/en/latest/"
_PUBLIC_DASHBOARD_URL = "https://dartsim.github.io/dart/performance/"
_DASHBOARD_GUIDE_URL = (
    "https://dart.readthedocs.io/en/latest/community/performance_dashboard.html"
)
_SURFACE_CATALOG = [
    {
        "surface": "collision",
        "label": "Collision",
        "benchmark_paths": [
            "tests/benchmark/collision/",
            "tests/benchmark/collision/comparative/",
            "tests/benchmark/collision/scenarios/",
        ],
        "entrypoint": "pixi run bm-collision-check",
        "comparison_scope": "Native, FCL, Bullet, ODE where harnesses exist",
        "next_step": "Live first slice; keep extending reference-backed rows.",
        "external_competitor_candidates": [
            "MuJoCo collision queries",
            "Drake geometry queries",
            "Gazebo/gz-physics collision queries",
        ],
    },
    {
        "surface": "common",
        "label": "Common / Allocator",
        "benchmark_paths": ["tests/benchmark/common/"],
        "entrypoint": "pixi run bm-dashboard-surfaces --surface common",
        "comparison_scope": "Allocator strategy comparisons",
        "next_step": "Representative JSON rows are wired; broaden filters after first main run.",
        "external_competitor_candidates": [
            "standard allocator baselines",
            "PMR allocator baselines",
            "third-party allocator baselines",
        ],
    },
    {
        "surface": "dynamics",
        "label": "Dynamics",
        "benchmark_paths": ["tests/benchmark/dynamics/"],
        "entrypoint": "pixi run bm-dashboard-surfaces --surface dynamics",
        "comparison_scope": "DART internal dynamics and kinematics baselines",
        "next_step": "Representative JSON rows are wired; broaden filters after first main run.",
        "external_competitor_candidates": [
            "MuJoCo dynamics",
            "Drake MultibodyPlant",
            "Simbody",
            "Gazebo/gz-physics",
        ],
    },
    {
        "surface": "lcp",
        "label": "LCP Solver",
        "benchmark_paths": ["tests/benchmark/lcpsolver/"],
        "entrypoint": "pixi run bm-dashboard-surfaces --surface lcp",
        "comparison_scope": "Solver and matrix-kernel comparisons",
        "next_step": "Representative JSON rows are wired; broaden filters after first main run.",
        "external_competitor_candidates": [
            "Bullet MLCP",
            "ODE QuickStep",
            "MuJoCo solver baselines",
        ],
    },
    {
        "surface": "math",
        "label": "Math",
        "benchmark_paths": [
            "tests/benchmark/math/",
            "tests/benchmark/unit/bm_convhull.cpp",
        ],
        "entrypoint": "pixi run bm-dashboard-surfaces --surface math",
        "comparison_scope": "Math helper, spatial algebra, and convex hull baselines",
        "next_step": "Representative JSON rows are wired; broaden filters after first main run.",
        "external_competitor_candidates": [
            "Eigen baselines",
            "scalar helper baselines",
            "external geometry kernels",
        ],
    },
    {
        "surface": "simd",
        "label": "SIMD",
        "benchmark_paths": ["tests/benchmark/simd/"],
        "entrypoint": "pixi run bm-dashboard-surfaces --surface simd",
        "comparison_scope": "Scalar/SIMD math kernels where present",
        "next_step": "Representative JSON rows are wired; broaden filters after first main run.",
        "external_competitor_candidates": [
            "scalar baselines",
            "Eigen packet math",
            "std::simd or xsimd baselines",
        ],
    },
    {
        "surface": "simulation",
        "label": "Simulation",
        "benchmark_paths": [
            "tests/benchmark/integration/",
            "tests/benchmark/simulation/experimental/",
        ],
        "entrypoint": "pixi run bm-dashboard-surfaces --surface simulation",
        "comparison_scope": "World stepping, compute graph, ECS safety, integration baselines",
        "next_step": "Representative world-step rows are wired; broaden ECS and integration rows after first main run.",
        "external_competitor_candidates": [
            "MuJoCo simulation",
            "Drake simulation",
            "Gazebo/gz-physics",
            "PyBullet",
        ],
    },
    {
        "surface": "compute",
        "label": "Compute Graph",
        "benchmark_paths": [
            "tests/benchmark/simulation/experimental/bm_compute_graph.cpp",
            "tests/benchmark/compute/",
        ],
        "entrypoint": "pixi run bm-dashboard-surfaces --surface compute",
        "comparison_scope": "Compute graph scheduling, resource access, and task execution baselines",
        "next_step": "Representative compute graph rows are wired; broaden resource-access rows after first main run.",
        "external_competitor_candidates": [
            "Drake Systems framework",
            "Gazebo/gz-sim ECS update loop",
            "generic task graph schedulers",
        ],
    },
    {
        "surface": "gpu",
        "label": "GPU / Accelerators",
        "benchmark_paths": [
            "tests/benchmark/gpu/",
            "tests/benchmark/simulation/experimental/gpu/",
        ],
        "entrypoint": "future pixi run bm-dashboard-surfaces --surface gpu",
        "comparison_scope": "GPU kernels, accelerator backends, and host/device transfer baselines",
        "next_step": "Add reproducible GPU or accelerator benchmark JSON after the CPU dashboard is live.",
        "external_competitor_candidates": [
            "MuJoCo GPU-enabled workloads",
            "Drake accelerator experiments",
            "CUDA/OpenCL/Vulkan kernels",
        ],
    },
]


@dataclass
class MeasurementDraft:
    benchmark: str
    source_file: str
    family: str
    surface: str
    backend: str
    comparable_group: str
    median_ns: float | None = None
    mean_ns: float | None = None
    stddev_ns: float | None = None
    cpu_time_ns: float | None = None
    real_time_ns: float | None = None
    time_unit: str | None = None
    iterations: int | None = None
    repetitions: int | None = None
    threads: int | None = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        type=Path,
        action="append",
        default=[],
        help=(
            "Google Benchmark JSON file or directory. Directories expand to "
            "*.json. Defaults to .benchmark_results/*.json."
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="Directory where data.json, index.html, and summary.md are written.",
    )
    parser.add_argument(
        "--clean-output",
        action="store_true",
        help=(
            "Remove existing files under --output-dir before writing the "
            "dashboard. History is loaded before cleanup."
        ),
    )
    parser.add_argument(
        "--history",
        type=Path,
        default=None,
        help=(
            "Existing dashboard data.json to append to. Defaults to "
            "<output-dir>/data.json when it already exists."
        ),
    )
    parser.add_argument(
        "--seed-input",
        type=Path,
        action="append",
        default=[],
        help=(
            "Google Benchmark JSON file or directory used only when no dashboard "
            "history exists yet. May be passed more than once to seed multiple "
            "historical runs."
        ),
    )
    parser.add_argument(
        "--run-id",
        default=os.environ.get("GITHUB_RUN_ID"),
        help="Stable run identifier. Defaults to GITHUB_RUN_ID or local timestamp.",
    )
    parser.add_argument(
        "--run-at",
        default=None,
        help="ISO-8601 timestamp for this run. Defaults to the newest input date.",
    )
    parser.add_argument(
        "--branch",
        default=os.environ.get("GITHUB_REF_NAME", "local"),
        help="Branch name recorded in the dashboard data.",
    )
    parser.add_argument(
        "--sha",
        default=os.environ.get("GITHUB_SHA", "unknown"),
        help="Commit SHA recorded in the dashboard data.",
    )
    parser.add_argument(
        "--source-url",
        default=_default_source_url(),
        help="CI run or artifact URL recorded in the dashboard data.",
    )
    parser.add_argument(
        "--testbed",
        default=_default_testbed(),
        help="Runner/testbed name recorded in the dashboard data.",
    )
    parser.add_argument(
        "--title",
        default="DART Performance Dashboard",
        help="HTML page title.",
    )
    parser.add_argument(
        "--allow-empty",
        action="store_true",
        help="Render a dashboard run even when no benchmark JSON inputs exist.",
    )
    return parser.parse_args()


def _default_source_url() -> str | None:
    server = os.environ.get("GITHUB_SERVER_URL")
    repo = os.environ.get("GITHUB_REPOSITORY")
    run_id = os.environ.get("GITHUB_RUN_ID")
    if server and repo and run_id:
        return f"{server}/{repo}/actions/runs/{run_id}"
    return None


def _default_testbed() -> str:
    runner = os.environ.get("RUNNER_NAME")
    if runner:
        return runner
    os_name = os.environ.get("RUNNER_OS")
    arch = os.environ.get("RUNNER_ARCH")
    if os_name and arch:
        return f"{os_name}-{arch}"
    return "local"


def _iso_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _to_ns(value: Any, unit: str | None) -> float | None:
    if value is None:
        return None
    try:
        return float(value) * _UNIT_TO_NS.get(unit or "ns", 1.0)
    except (TypeError, ValueError):
        return None


def _row_name(row: dict[str, Any]) -> str:
    return str(row.get("run_name") or row.get("name") or "")


def _collect_inputs(inputs: list[Path], allow_empty: bool) -> list[Path]:
    paths: list[Path] = []
    candidates = inputs or [Path(".benchmark_results")]
    for candidate in candidates:
        if candidate.is_dir():
            for path in sorted(candidate.glob("*.json")):
                if _is_empty_file(path):
                    print(
                        f"Skipping empty benchmark input: {path}",
                        file=sys.stderr,
                    )
                    continue
                paths.append(path)
        elif candidate.is_file():
            if _is_empty_file(candidate):
                if allow_empty:
                    print(
                        f"Skipping empty benchmark input: {candidate}",
                        file=sys.stderr,
                    )
                    continue
                raise SystemExit(f"Benchmark input is empty: {candidate}")
            paths.append(candidate)
        elif allow_empty and candidate == Path(".benchmark_results"):
            continue
        else:
            raise SystemExit(f"Benchmark input not found: {candidate}")

    paths = sorted(dict.fromkeys(paths))
    if not paths:
        if allow_empty:
            return []
        raise SystemExit("No benchmark JSON inputs found.")
    return paths


def _is_empty_file(path: Path) -> bool:
    try:
        return path.stat().st_size == 0
    except OSError:
        return False


def _parse_backend(name: str) -> tuple[str, str]:
    head, separator, params = name.partition("/")
    prefix, backend_separator, backend = head.rpartition("_")
    if backend_separator and backend in _KNOWN_BACKENDS:
        comparable = prefix
        if separator:
            comparable += separator + params
        return comparable, backend
    return name, "Native"


def _taxonomy_family(group: str, source_file: str) -> str:
    if group.startswith("BM_Distance_"):
        return "Distance Queries"
    if group.startswith("BM_Raycast_") or group.startswith("BM_Scenario_RaycastBatch_"):
        return "Ray And Cast Queries"
    if group.startswith("BM_DartAdapter_") or group.startswith(
        "BM_NarrowPhaseAdapter_"
    ):
        return "Public Adapter And Package Path"
    if group.startswith("BM_Scenario_MeshHeavy"):
        return "Mesh, Convex, And Field Heavy Scenes"
    if group.startswith("BM_Scenario_MixedPrimitives_"):
        return "Broadphase And Pair Generation"
    if group.startswith("BM_NarrowPhase_") or group.startswith(
        "BM_NarrowPhaseRawReference_"
    ):
        return "Pair Narrow Phase"
    if "lcp" in source_file.lower() or "LCPSOLVER" in group:
        return "LCP Solver"
    if "simd" in source_file.lower() or "SIMD" in group:
        return "SIMD"
    if "alloc" in source_file.lower() or "Allocator" in group:
        return "Allocator"
    if "dynamics" in source_file.lower() or "Kinematics" in group:
        return "Dynamics"
    if "math" in source_file.lower():
        return "Math"
    source_lower = source_file.lower()
    if "compute" in source_lower or "ComputeGraph" in group:
        return "Compute"
    if "gpu" in source_lower or "cuda" in source_lower or "vulkan" in source_lower:
        return "GPU / Accelerators"
    if "simulation" in source_lower:
        return "Simulation"
    return "Unclassified"


def _surface_from_source(path: Path, executable: str | None) -> str:
    path_text = path.as_posix().lower()
    if "simulation" in path_text:
        return "simulation"
    if "compute" in path_text:
        return "compute"
    if "gpu" in path_text or "cuda" in path_text or "vulkan" in path_text:
        return "gpu"

    text = f"{path_text} {executable or ''}".lower()
    if (
        "collision" in text
        or "bm_comparative_" in text
        or "bm_scenarios_" in text
        or "narrow_phase" in text
        or "raycast" in text
    ):
        return "collision"
    if "dynamics" in text:
        return "dynamics"
    if "lcpsolver" in text or "lcp" in text:
        return "lcp"
    if "simd" in text:
        return "simd"
    if "allocator" in text:
        return "common"
    if "compute" in text:
        return "compute"
    if "gpu" in text or "cuda" in text or "vulkan" in text or "opencl" in text:
        return "gpu"
    if "simulation" in text:
        return "simulation"
    if "math" in text:
        return "math"
    return "unknown"


def _load_json(path: Path) -> dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise SystemExit(f"Failed to parse {path}: {exc}") from exc


def _merge_row(draft: MeasurementDraft, row: dict[str, Any]) -> None:
    time_unit = str(row.get("time_unit", draft.time_unit or "ns"))
    aggregate = row.get("aggregate_name")
    cpu_time_ns = _to_ns(row.get("cpu_time"), time_unit)
    real_time_ns = _to_ns(row.get("real_time"), time_unit)

    if aggregate == "median":
        draft.median_ns = cpu_time_ns
    elif aggregate == "mean":
        draft.mean_ns = cpu_time_ns
    elif aggregate == "stddev":
        draft.stddev_ns = cpu_time_ns
    elif row.get("run_type") != "aggregate":
        draft.cpu_time_ns = cpu_time_ns
        draft.real_time_ns = real_time_ns

    if draft.median_ns is None and draft.cpu_time_ns is not None:
        draft.median_ns = draft.cpu_time_ns
    if draft.mean_ns is None and draft.cpu_time_ns is not None:
        draft.mean_ns = draft.cpu_time_ns

    draft.time_unit = time_unit
    draft.iterations = _int_or_existing(row.get("iterations"), draft.iterations)
    draft.repetitions = _int_or_existing(row.get("repetitions"), draft.repetitions)
    draft.threads = _int_or_existing(row.get("threads"), draft.threads)


def _int_or_existing(value: Any, existing: int | None) -> int | None:
    if value is None:
        return existing
    try:
        return int(value)
    except (TypeError, ValueError):
        return existing


def load_measurements(
    paths: list[Path],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    contexts: list[dict[str, Any]] = []
    drafts: dict[tuple[str, str], MeasurementDraft] = {}

    for path in paths:
        data = _load_json(path)
        context = data.get("context", {})
        executable = context.get("executable")
        surface = _surface_from_source(path, executable)
        contexts.append({"source_file": path.name, **context})

        for row in data.get("benchmarks", []):
            if row.get("run_type") == "aggregate" and row.get("aggregate_name") not in {
                "mean",
                "median",
                "stddev",
            }:
                continue

            name = _row_name(row)
            if not name:
                continue

            comparable_group, backend = _parse_backend(name)
            family = _taxonomy_family(comparable_group, path.name)
            key = (path.name, name)
            if key not in drafts:
                drafts[key] = MeasurementDraft(
                    benchmark=name,
                    source_file=path.name,
                    family=family,
                    surface=surface,
                    backend=backend,
                    comparable_group=comparable_group,
                )
            _merge_row(drafts[key], row)

    measurements = [_measurement_dict(draft) for draft in drafts.values()]
    return measurements, contexts


def _measurement_dict(draft: MeasurementDraft) -> dict[str, Any]:
    return {
        "benchmark": draft.benchmark,
        "source_file": draft.source_file,
        "family": draft.family,
        "surface": draft.surface,
        "backend": draft.backend,
        "comparable_group": draft.comparable_group,
        "median_ns": draft.median_ns,
        "mean_ns": draft.mean_ns,
        "stddev_ns": draft.stddev_ns,
        "cpu_time_ns": draft.cpu_time_ns,
        "real_time_ns": draft.real_time_ns,
        "time_unit": draft.time_unit,
        "iterations": draft.iterations,
        "repetitions": draft.repetitions,
        "threads": draft.threads,
    }


def _new_run(
    args: argparse.Namespace,
    paths: list[Path],
    contexts: list[dict[str, Any]],
) -> dict[str, Any]:
    run_at = args.run_at or _newest_context_date(contexts) or _iso_now()
    run_id = args.run_id or f"local-{_slug_time(run_at)}"
    return {
        "run_id": run_id,
        "run_at": run_at,
        "branch": args.branch,
        "sha": args.sha,
        "source_url": args.source_url,
        "testbed": args.testbed,
        "input_files": [path.name for path in paths],
        "contexts": contexts,
    }


def _newest_context_date(contexts: list[dict[str, Any]]) -> str | None:
    dates = [str(context["date"]) for context in contexts if context.get("date")]
    if not dates:
        return None
    return sorted(dates)[-1]


def _slug_time(value: str) -> str:
    return re.sub(r"[^0-9A-Za-z]+", "-", value).strip("-")


def _load_history(path: Path | None) -> dict[str, Any]:
    if path is None or not path.exists():
        return {"schema_version": _SCHEMA_VERSION, "runs": [], "measurements": []}
    data = _load_json(path)
    if data.get("schema_version") != _SCHEMA_VERSION:
        raise SystemExit(
            f"Unsupported dashboard schema version in {path}: "
            f"{data.get('schema_version')}"
        )
    data.setdefault("runs", [])
    data.setdefault("measurements", [])
    return data


def _merge_history(
    history: dict[str, Any],
    run: dict[str, Any],
    measurements: list[dict[str, Any]],
) -> dict[str, Any]:
    run_id = run["run_id"]
    merged_runs = [item for item in history["runs"] if item.get("run_id") != run_id]
    merged_measurements = [
        item for item in history["measurements"] if item.get("run_id") != run_id
    ]

    for measurement in measurements:
        measurement["run_id"] = run_id

    merged_runs.append(run)
    merged_measurements.extend(measurements)
    merged_runs.sort(key=lambda item: item.get("run_at", ""))
    merged_measurements.sort(
        key=lambda item: (
            item.get("run_id", ""),
            item.get("surface", ""),
            item.get("family", ""),
            item.get("benchmark", ""),
        )
    )

    history["runs"] = merged_runs
    history["measurements"] = merged_measurements
    history["generated_at"] = _iso_now()
    history["latest_run_id"] = run_id
    _refresh_derived_fields(history)
    return history


def _seed_source_slug(paths: list[Path]) -> str:
    if not paths:
        return "unknown"
    parents = {path.parent.name for path in paths if path.parent.name}
    if len(parents) == 1:
        source = next(iter(parents))
    else:
        source = Path(paths[0]).stem
    return _slug_time(source) or "unknown"


def _seed_run(paths: list[Path], contexts: list[dict[str, Any]]) -> dict[str, Any]:
    run_at = _newest_context_date(contexts) or _iso_now()
    source_slug = _seed_source_slug(paths)
    return {
        "run_id": f"seed-{_slug_time(run_at)}-{source_slug}",
        "run_at": run_at,
        "branch": "seed-history",
        "sha": "seed-history",
        "source_url": None,
        "testbed": "seed-history",
        "input_files": [path.name for path in paths],
        "contexts": contexts,
    }


def _merge_seed_history(
    history: dict[str, Any],
    seed_inputs: list[Path],
) -> dict[str, Any]:
    if history.get("runs") or not seed_inputs:
        return history

    for seed_input in seed_inputs:
        seed_paths = _collect_inputs([seed_input], allow_empty=True)
        if not seed_paths:
            continue
        seed_measurements, seed_contexts = load_measurements(seed_paths)
        if not seed_measurements:
            continue
        history = _merge_history(
            history,
            _seed_run(seed_paths, seed_contexts),
            seed_measurements,
        )
    return history


def _latest_measurements(data: dict[str, Any]) -> list[dict[str, Any]]:
    latest_run_id = str(data.get("latest_run_id", ""))
    return [
        item
        for item in data.get("measurements", [])
        if str(item.get("run_id")) == latest_run_id
    ]


def _refresh_derived_fields(data: dict[str, Any]) -> dict[str, Any]:
    measurements = _latest_measurements(data)
    data["links"] = dashboard_links()
    data["service_decision"] = service_decision()
    data["service_decision_summary"] = service_decision_summary(
        data["service_decision"]
    )
    data["publication_contract"] = publication_contract()
    data["comparison_input_contract"] = comparison_input_contract()
    data["comparison_metric_contract"] = comparison_metric_contract()
    data["thresholds"] = threshold_policy()
    data["freshness"] = freshness_status(data)
    data["latest_summary"] = summarize_run(measurements)
    data["latest_comparisons"] = compare_latest(measurements)
    latest_backend_matrix = backend_matrix_latest(measurements)
    data["latest_backend_matrix"] = latest_backend_matrix
    data["latest_backend_summary"] = backend_summary_latest(latest_backend_matrix)
    data["latest_trends"] = trend_latest(data)
    data["latest_reference_trends"] = reference_trend_latest(data)
    data["trend_summary"] = trend_summary(data)
    data["testbed_summary"] = testbed_summary(data)
    data["comparison_coverage"] = comparison_coverage_latest(latest_backend_matrix)
    data["external_competitor_status"] = external_competitor_status(data)
    data["coverage"] = coverage_summary(data)
    return data


def dashboard_links() -> dict[str, str]:
    return {
        "canonical_website_url": _CANONICAL_WEBSITE_URL,
        "dashboard_url": _PUBLIC_DASHBOARD_URL,
        "status_url": f"{_PUBLIC_DASHBOARD_URL}status.json",
        "data_url": f"{_PUBLIC_DASHBOARD_URL}data.json",
        "summary_url": f"{_PUBLIC_DASHBOARD_URL}summary.md",
        "guide_url": _DASHBOARD_GUIDE_URL,
    }


def service_decision() -> list[dict[str, Any]]:
    return [
        {
            "role": "primary-dashboard",
            "service": "DART-owned GitHub Pages",
            "state": "selected",
            "fit": "high",
            "cost_model": "Free for DART's public GitHub repository.",
            "data_owner": "DART-owned static files in gh-pages/performance.",
            "ci_integration": (
                "Path-scoped main push, scheduled, and manual GitHub Actions "
                "publisher."
            ),
            "approval_gate": "Reviewed workflow merge plus maintainer-approved first run.",
            "rationale": (
                "Free public static hosting for DART's existing repository, "
                "durable gh-pages history, and full control over DART-specific "
                "benchmark taxonomy and native/reference ratios."
            ),
            "next_step": (
                "Publish gh-pages/performance from the reviewed main-branch "
                "workflow."
            ),
            "evidence_urls": [
                "https://docs.github.com/en/pages/getting-started-with-github-pages",
                "https://docs.github.com/en/pages/getting-started-with-github-pages/configuring-a-publishing-source-for-your-github-pages-site",
            ],
        },
        {
            "role": "external-history-and-thresholds",
            "service": "Bencher Cloud or Self-Hosted",
            "state": "recommended-after-maintainer-approval",
            "fit": "high",
            "cost_model": (
                "Free for public/open-source projects on Bencher Cloud; "
                "self-hosted free path is available."
            ),
            "data_owner": (
                "Companion service; DART-owned Pages data remains canonical."
            ),
            "ci_integration": (
                "Optional bencher CLI report from main using the cpp_google "
                "Google Benchmark adapter."
            ),
            "approval_gate": (
                "Maintainers configure BENCHER_PROJECT and BENCHER_API_KEY."
            ),
            "rationale": (
                "Free public/open-source path, GitHub Actions integration, "
                "REST API, threshold support, and cpp_google ingestion for "
                "DART's current Google Benchmark JSON."
            ),
            "next_step": (
                "Configure BENCHER_PROJECT and BENCHER_API_KEY, then report "
                "base-branch benchmark JSON without PR comments or required "
                "checks until enough baseline data exists."
            ),
            "evidence_urls": [
                "https://github.com/bencherdev/bencher/blob/main/services/console/src/pages/pricing.astro",
                "https://github.com/bencherdev/bencher/blob/main/services/console/src/content/docs-how-to/en/github-actions.mdx",
                "https://github.com/bencherdev/bencher/blob/main/services/console/src/chunks/docs-how-to/github-actions/en/base-branch.mdx",
                "https://github.com/bencherdev/bencher/blob/main/services/console/src/chunks/docs-explanation/adapters/en/cpp-google.mdx",
            ],
        },
        {
            "role": "microbenchmark-pr-pilot",
            "service": "CodSpeed",
            "state": "deferred-pilot",
            "fit": "medium",
            "cost_model": (
                "Free and unlimited on public repositories; macro runners have "
                "a free tier with extra open-source minutes by request."
            ),
            "data_owner": "External CodSpeed project; not DART's canonical dashboard data.",
            "ci_integration": (
                "CodSpeed GitHub Action with CodSpeed's C++ Google Benchmark "
                "integration/build mode."
            ),
            "approval_gate": (
                "Separate maintainer-approved pilot and build integration change."
            ),
            "rationale": (
                "Good fit for lower-noise C++ microbenchmark regression checks, "
                "but it requires CodSpeed's google_benchmark integration or "
                "build mode rather than DART's current wall-clock JSON path."
            ),
            "next_step": (
                "Pilot on pure microbenchmark targets after the DART-owned "
                "dashboard is live."
            ),
            "evidence_urls": [
                "https://codspeed.io/docs/features/seats-and-billing",
                "https://codspeed.io/docs/integrations/ci/github-actions",
                "https://codspeed.io/docs/guides/how-to-benchmark-cpp-with-google-benchmark",
            ],
        },
        {
            "role": "off-the-shelf-pages-action",
            "service": "github-action-benchmark",
            "state": "not-selected",
            "fit": "medium",
            "cost_model": (
                "Open-source GitHub Action using the repository's GitHub Pages "
                "branch and GitHub Actions runtime."
            ),
            "data_owner": (
                "Generated action data under a Pages branch path; DART-owned "
                "Pages data would still be the durable host."
            ),
            "ci_integration": (
                "GitHub Action can parse googlecpp output, update gh-pages, "
                "and generate a Chart.js dashboard."
            ),
            "approval_gate": (
                "Would require maintainers to accept an additional Pages-writing "
                "third-party action."
            ),
            "rationale": (
                "Close to DART's hosting and Google Benchmark needs, but the "
                "default data model does not own DART's native/reference ratio "
                "taxonomy, external competitor coverage, publication contract, "
                "or exact endpoint verification."
            ),
            "next_step": (
                "Keep as a future simplification candidate if DART no longer "
                "needs the custom comparison schema."
            ),
            "evidence_urls": [
                "https://github.com/benchmark-action/github-action-benchmark",
                "https://github.com/marketplace/actions/continuous-benchmark",
            ],
        },
        {
            "role": "static-benchmark-dashboard-framework",
            "service": "Airspeed Velocity",
            "state": "not-selected",
            "fit": "medium",
            "cost_model": (
                "Open-source BSD-licensed benchmarking tool with static-web "
                "reporting."
            ),
            "data_owner": (
                "ASV JSON results plus generated static HTML; DART-owned Pages "
                "data would remain canonical."
            ),
            "ci_integration": (
                "Would require an ASV benchmark suite and adapter layer rather "
                "than consuming DART's current Google Benchmark JSON directly."
            ),
            "approval_gate": (
                "Separate maintainer-approved benchmark-suite migration or "
                "adapter pilot."
            ),
            "rationale": (
                "ASV has strong static history dashboards and regression tools, "
                "but it is primarily a Python benchmark-suite workflow, so it "
                "would duplicate or wrap DART's existing C++ Google Benchmark "
                "pipeline for the first dashboard."
            ),
            "next_step": (
                "Keep as a future migration candidate if DART wants an ASV-style "
                "suite for Python/dartpy-heavy benchmarks."
            ),
            "evidence_urls": [
                "https://github.com/airspeed-velocity/asv",
                "https://asv.readthedocs.io/en/stable/using.html",
                "https://asv.readthedocs.io/en/stable/commands.html",
            ],
        },
        {
            "role": "self-hosted-performance-tracker",
            "service": "LLVM LNT",
            "state": "not-selected",
            "fit": "medium",
            "cost_model": (
                "Open-source self-hosted performance tracking software; DART "
                "would own server hosting and operations."
            ),
            "data_owner": (
                "Self-hosted LNT server/database plus generated reports; "
                "DART-owned Pages data would remain canonical."
            ),
            "ci_integration": (
                "Would require translating DART benchmark outputs into LNT "
                "report JSON and submitting runs to a maintained LNT instance."
            ),
            "approval_gate": (
                "Infrastructure ownership decision plus maintainer-approved "
                "schema/adapter work before use."
            ),
            "rationale": (
                "LNT has a proven web UI, JSON submission format, REST access, "
                "and custom test-suite support, but it is a heavier "
                "self-hosted server path than the first DART-owned static "
                "dashboard and would not directly preserve DART's current "
                "Google Benchmark JSON or native/reference/competitor schema."
            ),
            "next_step": (
                "Keep as a future lab-scale option if DART needs a "
                "compiler-style performance server beyond Pages plus Bencher."
            ),
            "evidence_urls": [
                "https://llvm.org/docs/lnt/",
                "https://llvm.org/docs/lnt/concepts.html",
                "https://llvm.org/docs/lnt/importing_data.html",
                "https://github.com/llvm/llvm-lnt",
            ],
        },
        {
            "role": "self-hosted-fallback",
            "service": "Conbench",
            "state": "fallback",
            "fit": "medium",
            "cost_model": "Open-source self-hosted stack; DART would own hosting costs.",
            "data_owner": "Self-hosted Conbench API and dashboard operated by DART.",
            "ci_integration": (
                "JSON submissions to a Conbench server API from DART CI."
            ),
            "approval_gate": "Infrastructure ownership decision before use.",
            "rationale": (
                "Open-source continuous benchmarking framework with an API and "
                "dashboard, but it adds hosting and operations ownership for "
                "the first DART dashboard."
            ),
            "next_step": (
                "Revisit only if Pages plus Bencher cannot cover DART's public "
                "history and threshold needs."
            ),
            "evidence_urls": [
                "https://conbench.github.io/conbench/",
                "https://github.com/conbench/conbench",
            ],
        },
        {
            "role": "public-ecosystem-campaigns",
            "service": "OpenBenchmarking.org / Phoronix Test Suite",
            "state": "not-selected",
            "fit": "low",
            "cost_model": (
                "Free public use is available when uploaded results are public "
                "or briefly embargoed."
            ),
            "data_owner": (
                "External OpenBenchmarking result store; DART-owned Pages data "
                "remains canonical."
            ),
            "ci_integration": (
                "Would require a Phoronix Test Suite profile or compatible "
                "suite in addition to DART's Google Benchmark JSON pipeline."
            ),
            "approval_gate": (
                "Separate maintainer-approved public benchmark campaign and "
                "profile/suite ownership."
            ),
            "rationale": (
                "Useful for broad public ecosystem comparisons and repeatable "
                "Phoronix Test Suite campaigns, but not the shortest path to "
                "DART's existing Google Benchmark, native/reference, and "
                "competitor-ratio dashboard."
            ),
            "next_step": (
                "Revisit after the DART-owned dashboard is live if maintainers "
                "want public Phoronix ecosystem campaigns."
            ),
            "evidence_urls": [
                "https://openbenchmarking.org/",
                "https://openbenchmarking.org/features",
            ],
        },
    ]


def service_decision_summary(
    rows: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    rows = rows or service_decision()
    by_role = {str(row.get("role")): row for row in rows}
    selected = by_role["primary-dashboard"]
    recommended = by_role["external-history-and-thresholds"]
    deferred_states = {"deferred-pilot", "not-selected"}
    deferred_services = [
        str(row.get("service"))
        for row in rows
        if str(row.get("state")) in deferred_states
    ]
    fallback_services = [
        str(row.get("service")) for row in rows if str(row.get("state")) == "fallback"
    ]
    return {
        "best_option": (
            "Use DART-owned GitHub Pages as the canonical static dashboard "
            "and durable data host, with Bencher as an opt-in external "
            "history/threshold companion after maintainer approval."
        ),
        "selected_host": selected.get("service"),
        "selected_host_role": selected.get("role"),
        "recommended_external_service": recommended.get("service"),
        "recommended_external_role": recommended.get("role"),
        "canonical_data_owner": selected.get("data_owner"),
        "free_open_source_fit": (
            "The selected GitHub Pages host is free for DART's public GitHub "
            "repository, and the recommended Bencher companion has a free "
            "public/open-source path."
        ),
        "deferred_services": deferred_services,
        "fallback_services": fallback_services,
        "approval_boundary": (
            "Remote pushes, PR creation, merges, workflow dispatches, Pages "
            "publication, and Bencher setup require explicit maintainer "
            "approval."
        ),
    }


def publication_contract() -> dict[str, Any]:
    return {
        "host": "GitHub Pages",
        "site_url": _PUBLIC_DASHBOARD_URL,
        "source_branch": "gh-pages",
        "source_path": "/",
        "dashboard_path": "performance/",
        "build_type": "legacy branch-source",
        "publisher_workflow": ".github/workflows/performance_dashboard.yml",
        "workflow_name": "Performance Dashboard",
        "publish_ref": "refs/heads/main",
        "schedule_crons": ["30 3 * * 0,3"],
        "expected_update_interval_hours": _EXPECTED_UPDATE_INTERVAL_HOURS,
        "stale_after_hours": _STALE_AFTER_HOURS,
        "requires_pages_build_request": True,
        "required_permissions": [
            "actions: read",
            "contents: write",
            "pages: write",
        ],
        "required_endpoints": [
            _PUBLIC_DASHBOARD_URL,
            f"{_PUBLIC_DASHBOARD_URL}status.json",
            f"{_PUBLIC_DASHBOARD_URL}data.json",
            f"{_PUBLIC_DASHBOARD_URL}summary.md",
        ],
        "required_website_links": [
            _PUBLIC_DASHBOARD_URL,
            f"{_PUBLIC_DASHBOARD_URL}status.json",
            _DASHBOARD_GUIDE_URL,
        ],
        "required_workflow_artifacts": [
            "performance-dashboard-site-<run_id>-<run_attempt>",
        ],
        "launch_checks": [
            "pixi run check-bm-dashboard-launch-preflight",
            "pixi run check-bm-dashboard-pages-branch",
            "pixi run check-bm-dashboard-pages-build",
            "pixi run check-bm-dashboard-workflow-registration",
            "pixi run check-bm-dashboard-workflow-run",
            "pixi run check-bm-dashboard-launch-live",
        ],
        "approval_boundary": (
            "Pushes, PR creation, merges, workflow dispatches, and Pages "
            "publication require explicit maintainer approval."
        ),
    }


def comparison_input_contract() -> dict[str, Any]:
    return {
        "format": "Google Benchmark JSON",
        "backend_suffix_rule": (
            "Name reproducible comparison rows as <benchmark>_<Backend> before "
            "optional Google Benchmark parameters. Rows with the same "
            "<benchmark> prefix share a comparable_group."
        ),
        "primary_backend_rule": (
            "DART-owned rows should use primary backends such as Native, DART, "
            "Optimized, Pool, or SIMD. External engines should use one of "
            "supported_external_backends."
        ),
        "supported_external_backends": sorted(_EXTERNAL_COMPETITOR_BACKENDS),
        "example_rows": list(_EXTERNAL_COMPETITOR_EXAMPLE_ROWS),
        "claim_rule": (
            "The dashboard treats an external engine as a live competitor only "
            "when matching Google Benchmark rows are present in dashboard input "
            "data for the same run and comparable_group."
        ),
    }


def comparison_metric_contract() -> dict[str, Any]:
    return {
        "time_unit": "nanoseconds",
        "primary_time_field": "primary_ns",
        "backend_time_field": "backend_ns",
        "ratio_field": "backend_vs_primary_ratio",
        "ratio_formula": "backend_ns / primary_ns",
        "latest_comparison_ratio_field": "ratio",
        "latest_comparison_ratio_formula": "native_ns / best_reference_ns",
        "interpretation": (
            "For backend matrix rows, ratio < 1 means the backend/reference "
            "row is faster than the primary DART row, ratio > 1 means the "
            "primary DART row is faster, and ratio == 1 means parity within "
            "the recorded precision."
        ),
        "latest_comparison_interpretation": (
            "For latest native/reference comparison rows, ratio < 1 is a DART "
            "lead over the best reference, ratio > 1 is DART behind the best "
            "reference, and null means no reproducible reference row exists."
        ),
        "status_meanings": {
            "primary": "The row is the selected primary DART-owned baseline.",
            "faster-than-primary": "The backend/reference row is faster than the primary row.",
            "slower-than-primary": "The backend/reference row is slower than the primary row.",
            "same-as-primary": "The backend/reference row matches the primary row.",
            "lead": "The primary DART row is faster than the best reference row.",
            "behind": "The primary DART row is slower than the best reference row.",
            "native-only": "No reproducible reference row exists for that group.",
        },
    }


def threshold_policy() -> dict[str, Any]:
    return {
        "mode": "informational",
        "required_gate": False,
        "primary_change_percent": _CHANGE_THRESHOLD_PERCENT,
        "reference_change_percent": _CHANGE_THRESHOLD_PERCENT,
        "description": (
            "Dashboard trend labels use this threshold for review signals only; "
            "they are not required PR gates until runner variance is characterized."
        ),
    }


def freshness_policy() -> dict[str, Any]:
    return {
        "mode": "ci-publisher",
        "update_triggers": [
            "path-scoped main push",
            "scheduled workflow",
            "manual workflow_dispatch",
        ],
        "expected_update_interval_hours": _EXPECTED_UPDATE_INTERVAL_HOURS,
        "stale_after_hours": _STALE_AFTER_HOURS,
        "schedule": "30 3 UTC on Sunday and Wednesday",
        "description": (
            "Dashboard data is expected to refresh from main pushes plus the "
            "scheduled/manual GitHub Actions publisher; stale status is an "
            "endpoint health signal."
        ),
    }


def freshness_status(data: dict[str, Any]) -> dict[str, Any]:
    policy = freshness_policy()
    latest_run = _latest_run(data)
    latest_run_at = latest_run.get("run_at") if latest_run else None
    generated_at = data.get("generated_at")
    latest_datetime = _parse_datetime(latest_run_at)
    generated_datetime = _parse_datetime(generated_at)

    if latest_run is None:
        state = "no-runs"
        age_hours = None
    elif latest_datetime is None or generated_datetime is None:
        state = "unknown"
        age_hours = None
    else:
        age_hours = max(
            0.0,
            (generated_datetime - latest_datetime).total_seconds() / 3600.0,
        )
        state = "fresh" if age_hours <= float(policy["stale_after_hours"]) else "stale"

    return {
        **policy,
        "state": state,
        "latest_run_age_hours": round(age_hours, 2) if age_hours is not None else None,
        "latest_run_at": latest_run_at,
        "generated_at": generated_at,
    }


def _parse_datetime(value: Any) -> datetime | None:
    if not value:
        return None
    text = str(value)
    if text.endswith("Z"):
        text = f"{text[:-1]}+00:00"
    try:
        parsed = datetime.fromisoformat(text)
    except ValueError:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)


def _can_preserve_history_on_empty_run(
    data: dict[str, Any], measurements: list[dict[str, Any]], allow_empty: bool
) -> bool:
    return allow_empty and not measurements and bool(data.get("runs"))


def summarize_run(measurements: list[dict[str, Any]]) -> dict[str, Any]:
    surfaces = Counter(item["surface"] for item in measurements)
    families = Counter(item["family"] for item in measurements)
    backends = Counter(item["backend"] for item in measurements)
    comparisons = compare_latest(measurements)
    statuses = Counter(item["status"] for item in comparisons)
    return {
        "measurements": len(measurements),
        "surfaces": dict(sorted(surfaces.items())),
        "families": dict(sorted(families.items())),
        "backends": dict(sorted(backends.items())),
        "comparisons": dict(sorted(statuses.items())),
    }


def compare_latest(measurements: list[dict[str, Any]]) -> list[dict[str, Any]]:
    groups: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for item in measurements:
        groups[item["comparable_group"]].append(item)

    comparisons: list[dict[str, Any]] = []
    for group, rows in groups.items():
        native = _primary_backend_row(rows)
        if native is None:
            continue

        primary_backend = native["backend"]
        references = [
            row
            for row in rows
            if row["backend"] != primary_backend and _metric_ns(row) is not None
        ]
        native_ns = _metric_ns(native)
        if native_ns is None:
            status = "needs-data"
            ratio = None
            reference = None
            reference_ns = None
        elif not references:
            status = "native-only"
            ratio = None
            reference = None
            reference_ns = None
        else:
            best_reference = min(
                references, key=lambda row: _metric_ns(row) or float("inf")
            )
            reference = best_reference["backend"]
            reference_ns = _metric_ns(best_reference)
            if reference_ns is None or reference_ns <= 0:
                status = "needs-data"
                ratio = None
            else:
                ratio = native_ns / reference_ns
                status = "lead" if ratio <= 1.0 else "behind"

        comparisons.append(
            {
                "status": status,
                "family": native["family"],
                "surface": native["surface"],
                "comparable_group": group,
                "primary_backend": primary_backend,
                "native_ns": native_ns,
                "best_reference_backend": reference,
                "best_reference_ns": reference_ns,
                "ratio": ratio,
                "source_file": native["source_file"],
            }
        )

    comparisons.sort(
        key=lambda item: (
            _status_rank(item["status"]),
            -(item["ratio"] or 0.0),
            item["family"],
            item["comparable_group"],
        )
    )
    return comparisons


def backend_matrix_latest(measurements: list[dict[str, Any]]) -> list[dict[str, Any]]:
    groups: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for item in measurements:
        groups[item["comparable_group"]].append(item)

    matrix: list[dict[str, Any]] = []
    for group, rows in groups.items():
        primary = _primary_backend_row(rows)
        if primary is None:
            continue

        primary_backend = str(primary["backend"])
        primary_ns = _metric_ns(primary)
        for backend in sorted({str(row["backend"]) for row in rows}):
            row = _best_backend_row(rows, backend)
            if row is None:
                continue
            backend_ns = _metric_ns(row)
            ratio = _backend_vs_primary_ratio(primary_ns, backend_ns)
            matrix.append(
                {
                    "status": _backend_matrix_status(
                        backend=backend,
                        primary_backend=primary_backend,
                        backend_vs_primary_ratio=ratio,
                    ),
                    "family": row["family"],
                    "surface": row["surface"],
                    "comparable_group": group,
                    "primary_backend": primary_backend,
                    "primary_ns": primary_ns,
                    "backend": backend,
                    "backend_ns": backend_ns,
                    "backend_vs_primary_ratio": ratio,
                    "mean_ns": row.get("mean_ns"),
                    "stddev_ns": row.get("stddev_ns"),
                    "stddev_percent": _stddev_percent(row),
                    "repetitions": row.get("repetitions"),
                    "iterations": row.get("iterations"),
                    "source_file": row.get("source_file"),
                }
            )

    matrix.sort(
        key=lambda item: (
            item["family"],
            item["comparable_group"],
            _backend_matrix_rank(item["status"]),
            item.get("backend_vs_primary_ratio") or float("inf"),
            item["backend"],
        )
    )
    return matrix


def _backend_vs_primary_ratio(
    primary_ns: float | None, backend_ns: float | None
) -> float | None:
    if primary_ns is None or backend_ns is None or primary_ns <= 0:
        return None
    return backend_ns / primary_ns


def _backend_matrix_status(
    backend: str, primary_backend: str, backend_vs_primary_ratio: float | None
) -> str:
    if backend == primary_backend:
        return "primary"
    if backend_vs_primary_ratio is None:
        return "needs-data"
    if backend_vs_primary_ratio < 0.99:
        return "faster-than-primary"
    if backend_vs_primary_ratio > 1.01:
        return "slower-than-primary"
    return "same-as-primary"


def _backend_matrix_rank(status: str) -> int:
    return {
        "primary": 0,
        "faster-than-primary": 1,
        "same-as-primary": 2,
        "slower-than-primary": 3,
        "needs-data": 4,
    }.get(status, 5)


def backend_summary_latest(
    backend_matrix: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    groups: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for item in backend_matrix:
        if item["status"] == "primary":
            continue
        if item.get("backend_vs_primary_ratio") is None:
            continue
        groups[(str(item["surface"]), str(item["backend"]))].append(item)

    rows: list[dict[str, Any]] = []
    for (surface, backend), items in groups.items():
        ratios = [float(item["backend_vs_primary_ratio"]) for item in items]
        primary_faster = [ratio for ratio in ratios if ratio > 1.01]
        backend_faster = [ratio for ratio in ratios if ratio < 0.99]
        same = [ratio for ratio in ratios if ratio >= 0.99 and ratio <= 1.01]
        best_for_primary = max(items, key=lambda item: item["backend_vs_primary_ratio"])
        best_for_backend = min(items, key=lambda item: item["backend_vs_primary_ratio"])
        rows.append(
            {
                "status": _backend_summary_status(
                    primary_faster=len(primary_faster),
                    backend_faster=len(backend_faster),
                    same=len(same),
                ),
                "surface": surface,
                "backend": backend,
                "groups": len(items),
                "primary_faster": len(primary_faster),
                "backend_faster": len(backend_faster),
                "same": len(same),
                "geomean_backend_vs_primary_ratio": _geomean(ratios),
                "min_backend_vs_primary_ratio": min(ratios),
                "max_backend_vs_primary_ratio": max(ratios),
                "strongest_primary_group": best_for_primary["comparable_group"],
                "strongest_primary_ratio": best_for_primary["backend_vs_primary_ratio"],
                "strongest_backend_group": best_for_backend["comparable_group"],
                "strongest_backend_ratio": best_for_backend["backend_vs_primary_ratio"],
            }
        )

    rows.sort(
        key=lambda item: (
            item["surface"],
            _backend_summary_rank(item["status"]),
            -(item.get("geomean_backend_vs_primary_ratio") or 0.0),
            item["backend"],
        )
    )
    return rows


def comparison_coverage_latest(
    backend_matrix: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    rows_by_surface: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for item in backend_matrix:
        rows_by_surface[str(item["surface"])].append(item)

    coverage: list[dict[str, Any]] = []
    for surface in _SURFACE_CATALOG:
        key = surface["surface"]
        rows = rows_by_surface.get(key, [])
        primary_backends = sorted(
            {str(item["backend"]) for item in rows if item["status"] == "primary"}
        )
        reference_rows = [
            item
            for item in rows
            if item["status"] != "primary"
            and item.get("backend_vs_primary_ratio") is not None
        ]
        reference_backends = sorted({str(item["backend"]) for item in reference_rows})
        comparison_groups = sorted(
            {str(item["comparable_group"]) for item in reference_rows}
        )
        external_competitor_rows = [
            item
            for item in reference_rows
            if str(item["backend"]) in _EXTERNAL_COMPETITOR_BACKENDS
        ]
        external_competitor_backends = sorted(
            {str(item["backend"]) for item in external_competitor_rows}
        )
        external_competitor_groups = sorted(
            {str(item["comparable_group"]) for item in external_competitor_rows}
        )
        if reference_rows:
            state = "live"
            scope = "reproducible reference backends in dashboard JSON"
        elif rows:
            state = "primary-only"
            scope = "primary benchmark rows only; add reproducible references before claiming comparisons"
        else:
            state = "queued"
            scope = "no latest benchmark rows"

        coverage.append(
            {
                "surface": key,
                "label": surface["label"],
                "state": state,
                "primary_backends": primary_backends,
                "reference_backends": reference_backends,
                "comparison_group_count": len(comparison_groups),
                "reference_row_count": len(reference_rows),
                "sample_groups": comparison_groups[:8],
                "scope": scope,
                "comparison_scope": surface["comparison_scope"],
                "entrypoint": surface["entrypoint"],
                "next_step": surface["next_step"],
                "external_competitor_state": (
                    "live reproducible external harness rows in dashboard JSON"
                    if external_competitor_rows
                    else "deferred until a reproducible external harness writes dashboard JSON"
                ),
                "external_competitor_backends": external_competitor_backends,
                "external_competitor_row_count": len(external_competitor_rows),
                "external_competitor_group_count": len(external_competitor_groups),
                "external_competitor_sample_groups": external_competitor_groups[:8],
                "external_competitor_candidates": surface[
                    "external_competitor_candidates"
                ],
            }
        )
    return coverage


def _backend_summary_status(primary_faster: int, backend_faster: int, same: int) -> str:
    if backend_faster == 0 and primary_faster > 0:
        return "primary-faster"
    if primary_faster == 0 and backend_faster > 0:
        return "reference-faster"
    if primary_faster or backend_faster:
        return "mixed"
    if same:
        return "same"
    return "needs-data"


def _backend_summary_rank(status: str) -> int:
    return {
        "reference-faster": 0,
        "mixed": 1,
        "needs-data": 2,
        "same": 3,
        "primary-faster": 4,
    }.get(status, 5)


def _geomean(values: list[float]) -> float | None:
    positive = [value for value in values if value > 0]
    if not positive:
        return None
    return math.exp(sum(math.log(value) for value in positive) / len(positive))


def _primary_backend_row(rows: list[dict[str, Any]]) -> dict[str, Any] | None:
    for backend in _PRIMARY_BACKENDS:
        match = _best_backend_row(rows, backend)
        if match is not None:
            return match
    return None


def _best_backend_row(
    rows: list[dict[str, Any]], backend: str
) -> dict[str, Any] | None:
    matches = [row for row in rows if row["backend"] == backend]
    if not matches:
        return None
    return min(matches, key=lambda row: _metric_ns(row) or float("inf"))


def _metric_ns(row: dict[str, Any]) -> float | None:
    return row.get("median_ns") or row.get("mean_ns") or row.get("cpu_time_ns")


def _stddev_percent(row: dict[str, Any]) -> float | None:
    stddev_ns = row.get("stddev_ns")
    mean_ns = row.get("mean_ns") or row.get("median_ns") or row.get("cpu_time_ns")
    if stddev_ns is None or mean_ns is None:
        return None
    try:
        mean_value = float(mean_ns)
        if mean_value <= 0:
            return None
        return float(stddev_ns) / mean_value * 100.0
    except (TypeError, ValueError):
        return None


def _status_rank(status: str) -> int:
    return {"behind": 0, "needs-data": 1, "lead": 2, "native-only": 3}.get(status, 4)


def _format_time(ns: float | None) -> str:
    if ns is None:
        return "-"
    if ns < 1_000:
        return f"{ns:.3g} ns"
    if ns < 1_000_000:
        return f"{ns / 1_000:.3g} us"
    if ns < 1_000_000_000:
        return f"{ns / 1_000_000:.3g} ms"
    return f"{ns / 1_000_000_000:.3g} s"


def _format_ratio(value: float | None) -> str:
    if value is None:
        return "-"
    return f"{value:.3f}"


def coverage_summary(data: dict[str, Any]) -> list[dict[str, Any]]:
    latest_run_id = str(data.get("latest_run_id", ""))
    latest_measurements = [
        item
        for item in data.get("measurements", [])
        if str(item.get("run_id")) == latest_run_id
    ]
    latest_by_surface = Counter(
        item.get("surface", "unknown") for item in latest_measurements
    )
    history_by_surface = Counter(
        item.get("surface", "unknown") for item in data.get("measurements", [])
    )
    latest_families: dict[str, set[str]] = defaultdict(set)
    history_runs: dict[str, set[str]] = defaultdict(set)
    for item in latest_measurements:
        latest_families[str(item.get("surface", "unknown"))].add(
            str(item.get("family", "Unclassified"))
        )
    for item in data.get("measurements", []):
        surface = str(item.get("surface", "unknown"))
        if item.get("run_id") is not None:
            history_runs[surface].add(str(item.get("run_id")))

    rows = []
    for surface in _SURFACE_CATALOG:
        key = surface["surface"]
        latest_count = latest_by_surface.get(key, 0)
        history_count = history_by_surface.get(key, 0)
        if latest_count:
            state = "live"
        elif history_count:
            state = "historical-only"
        else:
            state = "queued"
        rows.append(
            {
                **surface,
                "state": state,
                "latest_measurements": latest_count,
                "history_measurements": history_count,
                "history_runs": len(history_runs.get(key, set())),
                "latest_families": sorted(latest_families.get(key, set())),
            }
        )
    return rows


def trend_latest(data: dict[str, Any]) -> list[dict[str, Any]]:
    latest_run_id = data.get("latest_run_id")
    if not latest_run_id:
        return []

    latest_comparisons = _comparisons_for_run(data, str(latest_run_id))
    previous_by_group = _previous_comparisons_by_group(data, str(latest_run_id))
    trends: list[dict[str, Any]] = []

    for latest in latest_comparisons:
        group = latest["comparable_group"]
        previous = previous_by_group.get(group)
        latest_native_ns = latest.get("native_ns")
        previous_native_ns = previous.get("native_ns") if previous else None
        native_change_percent = _change_percent(previous_native_ns, latest_native_ns)
        latest_ratio = latest.get("ratio")
        previous_ratio = previous.get("ratio") if previous else None
        ratio_change_percent = _change_percent(previous_ratio, latest_ratio)

        trends.append(
            {
                "status": _trend_status(native_change_percent, previous),
                "threshold_status": _threshold_status(native_change_percent, previous),
                "threshold_percent": _CHANGE_THRESHOLD_PERCENT,
                "family": latest["family"],
                "surface": latest["surface"],
                "comparable_group": group,
                "latest_run_id": latest_run_id,
                "previous_run_id": previous.get("run_id") if previous else None,
                "latest_native_ns": latest_native_ns,
                "previous_native_ns": previous_native_ns,
                "native_change_percent": native_change_percent,
                "latest_ratio": latest_ratio,
                "previous_ratio": previous_ratio,
                "ratio_change_percent": ratio_change_percent,
                "primary_backend": latest.get("primary_backend"),
                "best_reference_backend": latest.get("best_reference_backend"),
                "series": _native_series(data, group),
            }
        )

    trends.sort(key=_trend_sort_key)
    return trends


def _comparisons_for_run(data: dict[str, Any], run_id: str) -> list[dict[str, Any]]:
    measurements = [
        item
        for item in data.get("measurements", [])
        if str(item.get("run_id")) == run_id
    ]
    comparisons = compare_latest(measurements)
    run = _run_by_id(data, run_id)
    run_at = run.get("run_at") if run else None
    for comparison in comparisons:
        comparison["run_id"] = run_id
        comparison["run_at"] = run_at
    return comparisons


def _backend_matrix_for_run(data: dict[str, Any], run_id: str) -> list[dict[str, Any]]:
    measurements = [
        item
        for item in data.get("measurements", [])
        if str(item.get("run_id")) == run_id
    ]
    matrix = backend_matrix_latest(measurements)
    run = _run_by_id(data, run_id)
    run_at = run.get("run_at") if run else None
    for item in matrix:
        item["run_id"] = run_id
        item["run_at"] = run_at
    return matrix


def _previous_comparisons_by_group(
    data: dict[str, Any], latest_run_id: str
) -> dict[str, dict[str, Any]]:
    runs = _sorted_runs(data)
    latest_index = next(
        (
            index
            for index, run in enumerate(runs)
            if str(run.get("run_id")) == latest_run_id
        ),
        None,
    )
    if latest_index is None:
        return {}

    previous: dict[str, dict[str, Any]] = {}
    for run in reversed(runs[:latest_index]):
        for comparison in _comparisons_for_run(data, str(run.get("run_id"))):
            previous.setdefault(comparison["comparable_group"], comparison)
    return previous


def _previous_backend_matrix_by_key(
    data: dict[str, Any], latest_run_id: str
) -> dict[tuple[str, str], dict[str, Any]]:
    runs = _sorted_runs(data)
    latest_index = next(
        (
            index
            for index, run in enumerate(runs)
            if str(run.get("run_id")) == latest_run_id
        ),
        None,
    )
    if latest_index is None:
        return {}

    previous: dict[tuple[str, str], dict[str, Any]] = {}
    for run in reversed(runs[:latest_index]):
        for item in _backend_matrix_for_run(data, str(run.get("run_id"))):
            key = (str(item["comparable_group"]), str(item["backend"]))
            previous.setdefault(key, item)
    return previous


def reference_trend_latest(data: dict[str, Any]) -> list[dict[str, Any]]:
    latest_run_id = data.get("latest_run_id")
    if not latest_run_id:
        return []

    latest_rows = [
        item
        for item in _backend_matrix_for_run(data, str(latest_run_id))
        if item.get("status") != "primary"
    ]
    previous_by_key = _previous_backend_matrix_by_key(data, str(latest_run_id))
    trends: list[dict[str, Any]] = []

    for latest in latest_rows:
        key = (str(latest["comparable_group"]), str(latest["backend"]))
        previous = previous_by_key.get(key)
        latest_backend_ns = latest.get("backend_ns")
        previous_backend_ns = previous.get("backend_ns") if previous else None
        backend_change_percent = _change_percent(previous_backend_ns, latest_backend_ns)
        latest_ratio = latest.get("backend_vs_primary_ratio")
        previous_ratio = previous.get("backend_vs_primary_ratio") if previous else None
        ratio_change_percent = _change_percent(previous_ratio, latest_ratio)
        trends.append(
            {
                "status": _reference_trend_status(backend_change_percent, previous),
                "threshold_status": _threshold_status(backend_change_percent, previous),
                "threshold_percent": _CHANGE_THRESHOLD_PERCENT,
                "family": latest["family"],
                "surface": latest["surface"],
                "comparable_group": latest["comparable_group"],
                "backend": latest["backend"],
                "primary_backend": latest["primary_backend"],
                "latest_run_id": latest_run_id,
                "previous_run_id": previous.get("run_id") if previous else None,
                "latest_backend_ns": latest_backend_ns,
                "previous_backend_ns": previous_backend_ns,
                "backend_change_percent": backend_change_percent,
                "latest_backend_vs_primary_ratio": latest_ratio,
                "previous_backend_vs_primary_ratio": previous_ratio,
                "ratio_change_percent": ratio_change_percent,
                "series": _backend_series(
                    data,
                    str(latest["comparable_group"]),
                    str(latest["backend"]),
                ),
            }
        )

    trends.sort(key=_reference_trend_sort_key)
    return trends


def _native_series(data: dict[str, Any], group: str) -> list[dict[str, Any]]:
    series: list[dict[str, Any]] = []
    for run in _sorted_runs(data):
        run_id = str(run.get("run_id"))
        comparison = next(
            (
                item
                for item in _comparisons_for_run(data, run_id)
                if item["comparable_group"] == group
            ),
            None,
        )
        if comparison is None:
            continue
        series.append(
            {
                "run_id": run_id,
                "run_at": run.get("run_at"),
                "native_ns": comparison.get("native_ns"),
                "ratio": comparison.get("ratio"),
                "status": comparison.get("status"),
            }
        )
    return series


def _backend_series(
    data: dict[str, Any], group: str, backend: str
) -> list[dict[str, Any]]:
    series: list[dict[str, Any]] = []
    for run in _sorted_runs(data):
        run_id = str(run.get("run_id"))
        item = next(
            (
                row
                for row in _backend_matrix_for_run(data, run_id)
                if str(row["comparable_group"]) == group
                and str(row["backend"]) == backend
            ),
            None,
        )
        if item is None:
            continue
        series.append(
            {
                "run_id": run_id,
                "run_at": run.get("run_at"),
                "backend_ns": item.get("backend_ns"),
                "backend_vs_primary_ratio": item.get("backend_vs_primary_ratio"),
                "status": item.get("status"),
            }
        )
    return series


def trend_summary(data: dict[str, Any]) -> dict[str, Any]:
    primary_rows = data.get("latest_trends", [])
    reference_rows = data.get("latest_reference_trends", [])
    if not isinstance(primary_rows, list):
        primary_rows = []
    if not isinstance(reference_rows, list):
        reference_rows = []
    return {
        "latest_run_id": data.get("latest_run_id"),
        "history_runs": len(data.get("runs", [])),
        "primary_trend_rows": len(primary_rows),
        "reference_trend_rows": len(reference_rows),
        "primary_status_counts": _count_values(primary_rows, "status"),
        "reference_status_counts": _count_values(reference_rows, "status"),
        "primary_threshold_counts": _count_values(primary_rows, "threshold_status"),
        "reference_threshold_counts": _count_values(reference_rows, "threshold_status"),
        "primary_largest_regression": _change_snapshot(
            primary_rows,
            "native_change_percent",
            "largest",
        ),
        "primary_largest_improvement": _change_snapshot(
            primary_rows,
            "native_change_percent",
            "smallest",
        ),
        "reference_largest_regression": _change_snapshot(
            reference_rows,
            "backend_change_percent",
            "largest",
        ),
        "reference_largest_improvement": _change_snapshot(
            reference_rows,
            "backend_change_percent",
            "smallest",
        ),
    }


def testbed_summary(data: dict[str, Any]) -> dict[str, Any]:
    latest_run = _latest_run(data) or {}
    runs = data.get("runs", [])
    contexts = latest_run.get("contexts", [])
    if not isinstance(contexts, list):
        contexts = []
    return {
        "latest_run_id": data.get("latest_run_id"),
        "latest_run_at": latest_run.get("run_at"),
        "testbed": latest_run.get("testbed"),
        "history_runs": len(runs),
        "history_testbeds": _unique_run_values(runs, "testbed"),
        "context_count": len(contexts),
        "source_file_count": len(_unique_context_values(contexts, "source_file")),
        "source_files": _unique_context_values(contexts, "source_file", limit=12),
        "host_names": _unique_context_values(contexts, "host_name", limit=8),
        "num_cpus": _unique_context_ints(contexts, "num_cpus"),
        "cpu_scaling_counts": _context_bool_counts(contexts, "cpu_scaling_enabled"),
        "aslr_counts": _context_bool_counts(contexts, "aslr_enabled"),
        "interpretation": (
            "Trend interpretation is strongest when testbed, host, CPU count, "
            "and scaling state remain stable across compared runs."
        ),
    }


def _unique_run_values(
    runs: list[dict[str, Any]],
    field: str,
    *,
    limit: int = 8,
) -> list[str]:
    return sorted(
        {
            str(item[field])
            for item in runs
            if isinstance(item, dict) and item.get(field)
        }
    )[:limit]


def _unique_context_values(
    contexts: list[Any],
    field: str,
    *,
    limit: int | None = None,
) -> list[str]:
    values = sorted(
        {
            str(item[field])
            for item in contexts
            if isinstance(item, dict) and item.get(field) is not None
        }
    )
    return values if limit is None else values[:limit]


def _unique_context_ints(contexts: list[Any], field: str) -> list[int]:
    values: set[int] = set()
    for item in contexts:
        if not isinstance(item, dict):
            continue
        value = item.get(field)
        if value is None:
            continue
        try:
            values.add(int(value))
        except (TypeError, ValueError):
            continue
    return sorted(values)


def _context_bool_counts(contexts: list[Any], field: str) -> dict[str, int]:
    counts = {"enabled": 0, "disabled": 0, "unknown": 0}
    for item in contexts:
        if not isinstance(item, dict):
            counts["unknown"] += 1
            continue
        value = item.get(field)
        if value is True:
            counts["enabled"] += 1
        elif value is False:
            counts["disabled"] += 1
        else:
            counts["unknown"] += 1
    return counts


def _count_values(rows: list[dict[str, Any]], field: str) -> dict[str, int]:
    counts = Counter(str(row.get(field, "unknown")) for row in rows)
    return {key: counts[key] for key in sorted(counts)}


def _change_snapshot(
    rows: list[dict[str, Any]], field: str, direction: str
) -> dict[str, Any] | None:
    candidates = [row for row in rows if _finite_number(row.get(field)) is not None]
    if not candidates:
        return None
    if direction == "largest":
        row = max(candidates, key=lambda item: float(item[field]))
    else:
        row = min(candidates, key=lambda item: float(item[field]))
    return {
        "surface": row.get("surface"),
        "family": row.get("family"),
        "comparable_group": row.get("comparable_group"),
        "backend": row.get("backend"),
        "status": row.get("status"),
        "threshold_status": row.get("threshold_status"),
        "change_percent": row.get(field),
        "latest_run_id": row.get("latest_run_id"),
        "previous_run_id": row.get("previous_run_id"),
    }


def _finite_number(value: Any) -> float | None:
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def _sorted_runs(data: dict[str, Any]) -> list[dict[str, Any]]:
    return sorted(
        data.get("runs", []),
        key=lambda item: (str(item.get("run_at", "")), str(item.get("run_id", ""))),
    )


def _run_by_id(data: dict[str, Any], run_id: str) -> dict[str, Any] | None:
    for run in data.get("runs", []):
        if str(run.get("run_id")) == run_id:
            return run
    return None


def _change_percent(previous: Any, latest: Any) -> float | None:
    if previous is None or latest is None:
        return None
    try:
        previous_value = float(previous)
        latest_value = float(latest)
    except (TypeError, ValueError):
        return None
    if previous_value <= 0:
        return None
    return ((latest_value / previous_value) - 1.0) * 100.0


def _trend_status(
    native_change_percent: float | None, previous: dict[str, Any] | None
) -> str:
    if previous is None:
        return "new"
    if native_change_percent is None:
        return "needs-data"
    if native_change_percent > _CHANGE_THRESHOLD_PERCENT:
        return "slower"
    if native_change_percent < -_CHANGE_THRESHOLD_PERCENT:
        return "faster"
    return "flat"


def _reference_trend_status(
    backend_change_percent: float | None, previous: dict[str, Any] | None
) -> str:
    if previous is None:
        return "new"
    if backend_change_percent is None:
        return "needs-data"
    if backend_change_percent > _CHANGE_THRESHOLD_PERCENT:
        return "reference-slower"
    if backend_change_percent < -_CHANGE_THRESHOLD_PERCENT:
        return "reference-faster"
    return "flat"


def _threshold_status(
    change_percent: float | None, previous: dict[str, Any] | None
) -> str:
    if previous is None:
        return "new"
    if change_percent is None:
        return "needs-data"
    if change_percent > _CHANGE_THRESHOLD_PERCENT:
        return "informational-regression"
    if change_percent < -_CHANGE_THRESHOLD_PERCENT:
        return "informational-improvement"
    return "within-threshold"


def _trend_sort_key(item: dict[str, Any]) -> tuple[int, float, str, str]:
    rank = {"slower": 0, "needs-data": 1, "new": 2, "flat": 3, "faster": 4}.get(
        str(item.get("status")), 5
    )
    change = item.get("native_change_percent")
    magnitude = abs(float(change)) if change is not None else 0.0
    return (
        rank,
        -magnitude,
        str(item.get("family")),
        str(item.get("comparable_group")),
    )


def _reference_trend_sort_key(item: dict[str, Any]) -> tuple[int, float, str, str, str]:
    rank = {
        "reference-slower": 0,
        "needs-data": 1,
        "new": 2,
        "flat": 3,
        "reference-faster": 4,
    }.get(str(item.get("status")), 5)
    change = item.get("backend_change_percent")
    magnitude = abs(float(change)) if change is not None else 0.0
    return (
        rank,
        -magnitude,
        str(item.get("family")),
        str(item.get("comparable_group")),
        str(item.get("backend")),
    )


def render_html(data: dict[str, Any], title: str) -> str:
    latest_run = _latest_run(data)
    summary = data.get("latest_summary", {})
    comparisons = data.get("latest_comparisons", [])
    backend_summary = data.get("latest_backend_summary", [])
    backend_matrix = data.get("latest_backend_matrix", [])
    trends = data.get("latest_trends", [])
    reference_trends = data.get("latest_reference_trends", [])
    summary_trends = data.get("trend_summary", trend_summary(data))
    testbed = data.get("testbed_summary", testbed_summary(data))
    coverage = data.get("coverage", [])
    comparison_coverage = data.get("comparison_coverage", [])
    service_summary_rows = "\n".join(
        _contract_row(key, value)
        for key, value in data.get(
            "service_decision_summary",
            service_decision_summary(data.get("service_decision", service_decision())),
        ).items()
    )
    service_decision_rows = "\n".join(
        _service_decision_row(item) for item in data.get("service_decision", [])
    )
    publication_contract_rows = "\n".join(
        _publication_contract_row(key, value)
        for key, value in data.get(
            "publication_contract", publication_contract()
        ).items()
    )
    metric_contract_rows = "\n".join(
        _contract_row(key, value)
        for key, value in data.get(
            "comparison_metric_contract", comparison_metric_contract()
        ).items()
    )
    external_competitor_status_rows = "\n".join(
        _contract_row(key, value)
        for key, value in data.get(
            "external_competitor_status", external_competitor_status(data)
        ).items()
    )
    trend_summary_rows = "\n".join(
        _contract_row(key, value) for key, value in summary_trends.items()
    )
    testbed_summary_rows = "\n".join(
        _contract_row(key, value) for key, value in testbed.items()
    )
    rows = "\n".join(_comparison_row(item) for item in comparisons[:200])
    backend_summary_rows = "\n".join(
        _backend_summary_row(item) for item in backend_summary[:200]
    )
    backend_matrix_rows = "\n".join(
        _backend_matrix_row(item) for item in backend_matrix[:400]
    )
    trend_rows = "\n".join(_trend_row(item) for item in trends[:200])
    reference_trend_rows = "\n".join(
        _reference_trend_row(item) for item in reference_trends[:400]
    )
    coverage_rows = "\n".join(_coverage_row(item) for item in coverage)
    comparison_coverage_rows = "\n".join(
        _comparison_coverage_row(item) for item in comparison_coverage
    )
    run_rows = "\n".join(
        _run_history_row(item, data) for item in reversed(_sorted_runs(data)[-20:])
    )
    cards = _summary_cards(summary, data)
    run_meta = _run_meta(latest_run)
    data_json = html.escape(json.dumps(data, indent=2, sort_keys=True))
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{html.escape(title)}</title>
  <style>
    :root {{
      color-scheme: light dark;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      line-height: 1.45;
    }}
    body {{
      margin: 0;
      color: #202124;
      background: #f7f8fa;
    }}
    main {{
      max-width: 1180px;
      margin: 0 auto;
      padding: 32px 20px 56px;
    }}
    h1 {{
      margin: 0 0 8px;
      font-size: 2rem;
      letter-spacing: 0;
    }}
    h2 {{
      margin-top: 32px;
      font-size: 1.25rem;
      letter-spacing: 0;
    }}
    .meta {{
      color: #596069;
      margin: 0 0 24px;
    }}
    .resources {{
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      margin: 0 0 24px;
    }}
    .resources a {{
      color: #174ea6;
      background: white;
      border: 1px solid #d7dce2;
      border-radius: 8px;
      padding: 6px 10px;
      text-decoration: none;
      font-size: 0.86rem;
    }}
    .resources a:hover {{
      text-decoration: underline;
    }}
    .cards {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
      gap: 12px;
    }}
    .card {{
      background: white;
      border: 1px solid #d7dce2;
      border-radius: 8px;
      padding: 14px 16px;
    }}
    .label {{
      color: #596069;
      font-size: 0.78rem;
      text-transform: uppercase;
    }}
    .value {{
      font-size: 1.6rem;
      font-weight: 650;
      margin-top: 4px;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      background: white;
      border: 1px solid #d7dce2;
      border-radius: 8px;
      overflow: hidden;
    }}
    th, td {{
      padding: 9px 10px;
      border-bottom: 1px solid #e7ebef;
      text-align: left;
      vertical-align: top;
      font-size: 0.88rem;
    }}
    th {{
      background: #edf2f7;
      font-size: 0.78rem;
      text-transform: uppercase;
      color: #3f4752;
    }}
    tr:last-child td {{
      border-bottom: 0;
    }}
    code {{
      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      font-size: 0.84rem;
    }}
    .status-lead {{
      color: #116329;
      font-weight: 650;
    }}
    .status-behind {{
      color: #9b1c1c;
      font-weight: 650;
    }}
    .status-native-only, .status-needs-data {{
      color: #6d4b00;
      font-weight: 650;
    }}
    .status-primary, .status-slower-than-primary {{
      color: #116329;
      font-weight: 650;
    }}
    .status-faster-than-primary {{
      color: #9b1c1c;
      font-weight: 650;
    }}
    .status-same-as-primary {{
      color: #5f6368;
      font-weight: 650;
    }}
    .status-primary-faster {{
      color: #116329;
      font-weight: 650;
    }}
    .status-reference-faster {{
      color: #9b1c1c;
      font-weight: 650;
    }}
    .status-mixed, .status-same {{
      color: #5f6368;
      font-weight: 650;
    }}
    .trend-slower, .trend-reference-slower {{
      color: #9b1c1c;
      font-weight: 650;
    }}
    .trend-faster, .trend-reference-faster {{
      color: #116329;
      font-weight: 650;
    }}
    .trend-flat, .trend-new, .trend-needs-data {{
      color: #5f6368;
      font-weight: 650;
    }}
    .trend-live {{
      color: #116329;
      font-weight: 650;
    }}
    .trend-queued, .trend-historical-only, .trend-primary-only {{
      color: #5f6368;
      font-weight: 650;
    }}
    .spark {{
      color: #1a73e8;
      display: block;
      height: 28px;
      width: 120px;
    }}
    .spark-empty {{
      color: #5f6368;
    }}
    details {{
      margin-top: 28px;
    }}
    pre {{
      overflow: auto;
      background: #101418;
      color: #f7f8fa;
      padding: 16px;
      border-radius: 8px;
    }}
    @media (prefers-color-scheme: dark) {{
      body {{
        color: #e8eaed;
        background: #111418;
      }}
      .meta {{
        color: #aab0b8;
      }}
      .resources a {{
        color: #8ab4f8;
        background: #171b21;
        border-color: #343a44;
      }}
      .card, table {{
        background: #171b21;
        border-color: #343a44;
      }}
      th {{
        background: #232933;
        color: #c6cbd3;
      }}
      td {{
        border-bottom-color: #2a3038;
      }}
    }}
  </style>
</head>
<body>
<main>
  <h1>{html.escape(title)}</h1>
  <p class="meta">{run_meta}</p>
  <nav class="resources" aria-label="Dashboard resources">
    <a href="status.json">status.json</a>
    <a href="data.json">data.json</a>
    <a href="summary.md">summary.md</a>
    <a href="{_DASHBOARD_GUIDE_URL}">Read the Docs guide</a>
  </nav>
  <section class="cards">{cards}</section>
  <h2>Testbed Summary</h2>
  <table>
    <thead>
      <tr>
        <th>Field</th>
        <th>Value</th>
      </tr>
    </thead>
    <tbody>
{testbed_summary_rows}
    </tbody>
  </table>
  <h2>Service Decision Summary</h2>
  <table>
    <thead>
      <tr>
        <th>Field</th>
        <th>Value</th>
      </tr>
    </thead>
    <tbody>
{service_summary_rows}
    </tbody>
  </table>
  <h2>Dashboard Service Decision</h2>
  <table>
    <thead>
      <tr>
        <th>Role</th>
        <th>Service</th>
        <th>State</th>
        <th>Fit</th>
        <th>Cost / OSS Fit</th>
        <th>Data Owner</th>
        <th>CI Integration</th>
        <th>Approval Gate</th>
        <th>Rationale</th>
        <th>Next Step</th>
      </tr>
    </thead>
    <tbody>
{service_decision_rows}
    </tbody>
  </table>
  <h2>Publication Contract</h2>
  <table>
    <thead>
      <tr>
        <th>Field</th>
        <th>Value</th>
      </tr>
    </thead>
    <tbody>
{publication_contract_rows}
    </tbody>
  </table>
  <h2>Comparison Metric Contract</h2>
  <table>
    <thead>
      <tr>
        <th>Field</th>
        <th>Value</th>
      </tr>
    </thead>
    <tbody>
{metric_contract_rows}
    </tbody>
  </table>
  <h2>External Competitor Status</h2>
  <table>
    <thead>
      <tr>
        <th>Field</th>
        <th>Value</th>
      </tr>
    </thead>
    <tbody>
{external_competitor_status_rows}
    </tbody>
  </table>
  <h2>Benchmark Surface Coverage</h2>
  <table>
    <thead>
      <tr>
        <th>State</th>
        <th>Surface</th>
        <th>Latest Rows</th>
        <th>History Runs</th>
        <th>Entrypoint</th>
        <th>Comparison Scope</th>
        <th>Next Step</th>
      </tr>
    </thead>
    <tbody>
{coverage_rows}
    </tbody>
  </table>
  <h2>Reference And Competitor Coverage</h2>
  <table>
    <thead>
      <tr>
        <th>State</th>
        <th>Surface</th>
        <th>Primary Backends</th>
        <th>Reference Backends</th>
        <th>Compared Groups</th>
        <th>Live Scope</th>
        <th>Comparison Target</th>
        <th>External Competitors</th>
        <th>Next Step</th>
      </tr>
    </thead>
    <tbody>
{comparison_coverage_rows}
    </tbody>
  </table>
  <h2>Trend Summary</h2>
  <table>
    <thead>
      <tr>
        <th>Field</th>
        <th>Value</th>
      </tr>
    </thead>
    <tbody>
{trend_summary_rows}
    </tbody>
  </table>
  <h2>Latest Changes vs Previous Run</h2>
  <table>
    <thead>
      <tr>
        <th>Status</th>
        <th>Family</th>
        <th>Benchmark Group</th>
        <th>Previous Primary</th>
        <th>Latest Primary</th>
        <th>Primary Change</th>
        <th>Threshold</th>
        <th>Latest Ratio</th>
        <th>Ratio Change</th>
        <th>Primary History</th>
      </tr>
    </thead>
    <tbody>
{trend_rows}
    </tbody>
  </table>
  <h2>Latest Reference Backend Changes vs Previous Run</h2>
  <table>
    <thead>
      <tr>
        <th>Status</th>
        <th>Family</th>
        <th>Benchmark Group</th>
        <th>Backend</th>
        <th>Previous Reference</th>
        <th>Latest Reference</th>
        <th>Reference Change</th>
        <th>Threshold</th>
        <th>Latest Backend/Primary</th>
        <th>Ratio Change</th>
        <th>Reference History</th>
      </tr>
    </thead>
    <tbody>
{reference_trend_rows}
    </tbody>
  </table>
  <h2>Latest Native/Reference Comparisons</h2>
  <table>
    <thead>
      <tr>
        <th>Status</th>
        <th>Family</th>
        <th>Benchmark Group</th>
        <th>Native/Primary</th>
        <th>Best Reference</th>
        <th>Ratio</th>
        <th>Source</th>
      </tr>
    </thead>
    <tbody>
{rows}
    </tbody>
  </table>
  <h2>Latest Reference Backend Summary</h2>
  <table>
    <thead>
      <tr>
        <th>Status</th>
        <th>Surface</th>
        <th>Backend</th>
        <th>Groups</th>
        <th>Primary Faster</th>
        <th>Reference Faster</th>
        <th>Geomean Backend/Primary</th>
        <th>Range</th>
        <th>Strongest Primary Group</th>
        <th>Strongest Reference Group</th>
      </tr>
    </thead>
    <tbody>
{backend_summary_rows}
    </tbody>
  </table>
  <h2>Latest Backend Matrix</h2>
  <table>
    <thead>
      <tr>
        <th>Status</th>
        <th>Family</th>
        <th>Benchmark Group</th>
        <th>Backend</th>
        <th>Primary</th>
        <th>Time</th>
        <th>Backend/Primary</th>
        <th>Mean / Stddev</th>
        <th>Source</th>
      </tr>
    </thead>
    <tbody>
{backend_matrix_rows}
    </tbody>
  </table>
  <h2>Run History</h2>
  <table>
    <thead>
      <tr>
        <th>Run</th>
        <th>Time</th>
        <th>Branch</th>
        <th>SHA</th>
        <th>Testbed</th>
        <th>Inputs</th>
      </tr>
    </thead>
    <tbody>
{run_rows}
    </tbody>
  </table>
  <details>
    <summary>Dashboard data JSON</summary>
    <pre>{data_json}</pre>
  </details>
</main>
</body>
</html>
"""


def _latest_run(data: dict[str, Any]) -> dict[str, Any] | None:
    run_id = data.get("latest_run_id")
    for run in data.get("runs", []):
        if run.get("run_id") == run_id:
            return run
    return data.get("runs", [])[-1] if data.get("runs") else None


def _run_meta(run: dict[str, Any] | None) -> str:
    if run is None:
        return "No benchmark runs recorded."
    parts = [
        f"Run <code>{html.escape(str(run.get('run_id')))}</code>",
        html.escape(str(run.get("run_at", "unknown time"))),
        f"branch <code>{html.escape(str(run.get('branch', 'unknown')))}</code>",
        f"testbed <code>{html.escape(str(run.get('testbed', 'unknown')))}</code>",
    ]
    sha = str(run.get("sha", ""))
    if sha and sha != "unknown":
        parts.append(f"sha <code>{html.escape(sha[:12])}</code>")
    source_url = run.get("source_url")
    if source_url:
        parts.append(f'<a href="{html.escape(str(source_url))}">source run</a>')
    return " | ".join(parts)


def _summary_cards(summary: dict[str, Any], data: dict[str, Any]) -> str:
    comparison_counts = summary.get("comparisons", {})
    coverage_counts = Counter(item.get("state") for item in data.get("coverage", []))
    thresholds = data.get("thresholds", threshold_policy())
    freshness = data.get("freshness", freshness_status(data))
    items = [
        ("Runs", len(data.get("runs", []))),
        ("Measurements", summary.get("measurements", 0)),
        ("Freshness", freshness.get("state", "unknown")),
        ("Surfaces", len(summary.get("surfaces", {}))),
        ("Live Surfaces", coverage_counts.get("live", 0)),
        ("Queued Surfaces", coverage_counts.get("queued", 0)),
        ("Families", len(summary.get("families", {}))),
        ("Lead", comparison_counts.get("lead", 0)),
        ("Behind", comparison_counts.get("behind", 0)),
        ("Native Only", comparison_counts.get("native-only", 0)),
        (
            "Change Threshold",
            f"{thresholds.get('primary_change_percent', _CHANGE_THRESHOLD_PERCENT):g}%",
        ),
    ]
    return "\n".join(
        f'<div class="card"><div class="label">{html.escape(label)}</div>'
        f'<div class="value">{html.escape(str(value))}</div></div>'
        for label, value in items
    )


def _coverage_row(item: dict[str, Any]) -> str:
    paths = ", ".join(item.get("benchmark_paths", []))
    latest_families = item.get("latest_families") or []
    family_text = ", ".join(latest_families) if latest_families else "-"
    return (
        "      <tr>"
        f'<td class="trend-{html.escape(str(item["state"]))}">{html.escape(str(item["state"]))}</td>'
        f"<td>{html.escape(str(item['label']))}<br><code>{html.escape(paths)}</code></td>"
        f"<td>{html.escape(str(item.get('latest_measurements', 0)))}"
        f"<br>{html.escape(family_text)}</td>"
        f"<td>{html.escape(str(item.get('history_runs', 0)))}</td>"
        f"<td><code>{html.escape(str(item['entrypoint']))}</code></td>"
        f"<td>{html.escape(str(item['comparison_scope']))}</td>"
        f"<td>{html.escape(str(item['next_step']))}</td>"
        "</tr>"
    )


def _comparison_coverage_row(item: dict[str, Any]) -> str:
    primary = ", ".join(item.get("primary_backends", [])) or "-"
    references = ", ".join(item.get("reference_backends", [])) or "-"
    candidates = ", ".join(item.get("external_competitor_candidates", [])) or "-"
    external_backends = ", ".join(item.get("external_competitor_backends", [])) or "-"
    comparison_scope = str(item.get("comparison_scope", "-"))
    entrypoint = str(item.get("entrypoint", "-"))
    next_step = str(item.get("next_step", "-"))
    sample_groups = item.get("sample_groups", [])
    group_text = ", ".join(sample_groups) if sample_groups else "-"
    if item.get("comparison_group_count", 0) > len(sample_groups):
        group_text += f", +{item['comparison_group_count'] - len(sample_groups)} more"
    return (
        "      <tr>"
        f'<td class="trend-{html.escape(str(item["state"]))}">{html.escape(str(item["state"]))}</td>'
        f"<td>{html.escape(str(item['label']))}</td>"
        f"<td><code>{html.escape(primary)}</code></td>"
        f"<td><code>{html.escape(references)}</code>"
        f"<br>{html.escape(str(item.get('reference_row_count', 0)))} rows</td>"
        f"<td>{html.escape(str(item.get('comparison_group_count', 0)))}"
        f"<br><code>{html.escape(group_text)}</code></td>"
        f"<td>{html.escape(str(item['scope']))}</td>"
        f"<td>{html.escape(comparison_scope)}<br><code>{html.escape(entrypoint)}</code></td>"
        f"<td>{html.escape(str(item['external_competitor_state']))}"
        f"<br><code>{html.escape(external_backends)}</code>"
        f"<br>{html.escape(str(item.get('external_competitor_row_count', 0)))} rows"
        f"<br><code>{html.escape(candidates)}</code></td>"
        f"<td>{html.escape(next_step)}</td>"
        "</tr>"
    )


def _service_decision_row(item: dict[str, Any]) -> str:
    evidence_urls = item.get("evidence_urls", [])
    evidence = ", ".join(str(url) for url in evidence_urls if url)
    if evidence:
        evidence = f" Sources: {evidence}"
    return (
        "      <tr>"
        f"<td><code>{html.escape(str(item.get('role', '-')))}</code></td>"
        f"<td>{html.escape(str(item.get('service', '-')))}</td>"
        f"<td>{html.escape(str(item.get('state', '-')))}</td>"
        f"<td>{html.escape(str(item.get('fit', '-')))}</td>"
        f"<td>{html.escape(str(item.get('cost_model', '-')))}</td>"
        f"<td>{html.escape(str(item.get('data_owner', '-')))}</td>"
        f"<td>{html.escape(str(item.get('ci_integration', '-')))}</td>"
        f"<td>{html.escape(str(item.get('approval_gate', '-')))}</td>"
        f"<td>{html.escape(str(item.get('rationale', '-')) + evidence)}</td>"
        f"<td>{html.escape(str(item.get('next_step', '-')))}</td>"
        "</tr>"
    )


def _contract_value_html(value: Any) -> str:
    if isinstance(value, list):
        return "<br>".join(f"<code>{html.escape(str(item))}</code>" for item in value)
    if isinstance(value, dict):
        return "<br>".join(
            f"<code>{html.escape(str(key))}</code>: {html.escape(str(item))}"
            for key, item in value.items()
        )
    return html.escape(str(value))


def _contract_row(key: str, value: Any) -> str:
    formatted = _contract_value_html(value)
    return (
        "      <tr>"
        f"<td><code>{html.escape(str(key))}</code></td>"
        f"<td>{formatted}</td>"
        "</tr>"
    )


def _publication_contract_row(key: str, value: Any) -> str:
    return _contract_row(key, value)


def _trend_row(item: dict[str, Any]) -> str:
    status = str(item["status"])
    previous_run = item.get("previous_run_id") or "-"
    latest_run = item.get("latest_run_id") or "-"
    ratio = item.get("latest_ratio")
    return (
        "      <tr>"
        f'<td class="trend-{html.escape(status)}">{html.escape(status)}</td>'
        f"<td>{html.escape(str(item['family']))}</td>"
        f"<td><code>{html.escape(str(item['comparable_group']))}</code></td>"
        f"<td>{html.escape(_format_time(item.get('previous_native_ns')))}"
        f"<br><code>{html.escape(str(previous_run))}</code></td>"
        f"<td>{html.escape(_format_time(item.get('latest_native_ns')))}"
        f"<br><code>{html.escape(str(latest_run))}</code></td>"
        f"<td>{html.escape(_format_change_label(item.get('native_change_percent')))}</td>"
        f"<td>{html.escape(_format_threshold_label(item))}</td>"
        f"<td>{html.escape(_format_ratio(ratio))}</td>"
        f"<td>{html.escape(_format_signed_percent(item.get('ratio_change_percent')))}</td>"
        f"<td>{_sparkline(item.get('series', []))}</td>"
        "</tr>"
    )


def _reference_trend_row(item: dict[str, Any]) -> str:
    status = str(item["status"])
    previous_run = item.get("previous_run_id") or "-"
    latest_run = item.get("latest_run_id") or "-"
    ratio = item.get("latest_backend_vs_primary_ratio")
    return (
        "      <tr>"
        f'<td class="trend-{html.escape(status)}">{html.escape(status)}</td>'
        f"<td>{html.escape(str(item['family']))}</td>"
        f"<td><code>{html.escape(str(item['comparable_group']))}</code></td>"
        f"<td><code>{html.escape(str(item['backend']))}</code>"
        f"<br>primary <code>{html.escape(str(item['primary_backend']))}</code></td>"
        f"<td>{html.escape(_format_time(item.get('previous_backend_ns')))}"
        f"<br><code>{html.escape(str(previous_run))}</code></td>"
        f"<td>{html.escape(_format_time(item.get('latest_backend_ns')))}"
        f"<br><code>{html.escape(str(latest_run))}</code></td>"
        f"<td>{html.escape(_format_change_label(item.get('backend_change_percent')))}</td>"
        f"<td>{html.escape(_format_threshold_label(item))}</td>"
        f"<td>{html.escape(_format_ratio(ratio))}</td>"
        f"<td>{html.escape(_format_signed_percent(item.get('ratio_change_percent')))}</td>"
        f"<td>{_sparkline(item.get('series', []), value_key='backend_ns')}</td>"
        "</tr>"
    )


def _comparison_row(item: dict[str, Any]) -> str:
    status = str(item["status"])
    status_class = "status-" + status
    primary = item.get("primary_backend") or "Native"
    reference = item.get("best_reference_backend") or "-"
    reference_time = _format_time(item.get("best_reference_ns"))
    source = html.escape(str(item.get("source_file") or "-"))
    return (
        "      <tr>"
        f'<td class="{status_class}">{html.escape(status)}</td>'
        f"<td>{html.escape(str(item['family']))}</td>"
        f"<td><code>{html.escape(str(item['comparable_group']))}</code></td>"
        f"<td>{html.escape(_format_time(item.get('native_ns')))}"
        f"<br><code>{html.escape(str(primary))}</code></td>"
        f"<td>{html.escape(str(reference))}: {html.escape(reference_time)}</td>"
        f"<td>{html.escape(_format_ratio(item.get('ratio')))}</td>"
        f"<td><code>{source}</code></td>"
        "</tr>"
    )


def _backend_summary_row(item: dict[str, Any]) -> str:
    status = str(item["status"])
    status_class = "status-" + status
    ratio_range = (
        f"{_format_ratio(item.get('min_backend_vs_primary_ratio'))} - "
        f"{_format_ratio(item.get('max_backend_vs_primary_ratio'))}"
    )
    return (
        "      <tr>"
        f'<td class="{status_class}">{html.escape(status)}</td>'
        f"<td>{html.escape(str(item['surface']))}</td>"
        f"<td><code>{html.escape(str(item['backend']))}</code></td>"
        f"<td>{html.escape(str(item['groups']))}</td>"
        f"<td>{html.escape(str(item['primary_faster']))}</td>"
        f"<td>{html.escape(str(item['backend_faster']))}</td>"
        f"<td>{html.escape(_format_ratio(item.get('geomean_backend_vs_primary_ratio')))}</td>"
        f"<td>{html.escape(ratio_range)}</td>"
        f"<td><code>{html.escape(str(item['strongest_primary_group']))}</code>"
        f"<br>{html.escape(_format_ratio(item.get('strongest_primary_ratio')))}</td>"
        f"<td><code>{html.escape(str(item['strongest_backend_group']))}</code>"
        f"<br>{html.escape(_format_ratio(item.get('strongest_backend_ratio')))}</td>"
        "</tr>"
    )


def _backend_matrix_row(item: dict[str, Any]) -> str:
    status = str(item["status"])
    status_class = "status-" + status
    source = html.escape(str(item.get("source_file") or "-"))
    return (
        "      <tr>"
        f'<td class="{status_class}">{html.escape(status)}</td>'
        f"<td>{html.escape(str(item['family']))}</td>"
        f"<td><code>{html.escape(str(item['comparable_group']))}</code></td>"
        f"<td><code>{html.escape(str(item['backend']))}</code></td>"
        f"<td><code>{html.escape(str(item['primary_backend']))}</code></td>"
        f"<td>{html.escape(_format_time(item.get('backend_ns')))}</td>"
        f"<td>{html.escape(_format_ratio(item.get('backend_vs_primary_ratio')))}</td>"
        f"<td>{html.escape(_format_mean_stddev(item))}</td>"
        f"<td><code>{source}</code></td>"
        "</tr>"
    )


def _run_history_row(run: dict[str, Any], data: dict[str, Any]) -> str:
    run_id = str(run.get("run_id", ""))
    sha = str(run.get("sha", ""))
    inputs = run.get("input_files", [])
    input_count = len(inputs) if isinstance(inputs, list) else 0
    marker = " latest" if run_id == str(data.get("latest_run_id")) else ""
    return (
        "      <tr>"
        f"<td><code>{html.escape(run_id)}</code>{html.escape(marker)}</td>"
        f"<td>{html.escape(str(run.get('run_at', '-')))}</td>"
        f"<td><code>{html.escape(str(run.get('branch', '-')))}</code></td>"
        f"<td><code>{html.escape(sha[:12] if sha else '-')}</code></td>"
        f"<td><code>{html.escape(str(run.get('testbed', '-')))}</code></td>"
        f"<td>{input_count}</td>"
        "</tr>"
    )


def _format_change_label(value: Any) -> str:
    if value is None:
        return "-"
    numeric = float(value)
    if abs(numeric) <= 1.0:
        return "flat"
    direction = "slower" if numeric > 0 else "faster"
    return f"{abs(numeric):.1f}% {direction}"


def _format_signed_percent(value: Any) -> str:
    if value is None:
        return "-"
    return f"{float(value):+.1f}%"


def _format_threshold_label(item: dict[str, Any]) -> str:
    threshold = item.get("threshold_percent")
    threshold_text = "-" if threshold is None else f"{float(threshold):g}%"
    return f"{item.get('threshold_status', '-')}: {threshold_text}"


def _format_mean_stddev(item: dict[str, Any]) -> str:
    mean = _format_time(item.get("mean_ns"))
    stddev = _format_time(item.get("stddev_ns"))
    stddev_percent = item.get("stddev_percent")
    if stddev_percent is None:
        return f"{mean} / {stddev}"
    return f"{mean} / {stddev} ({float(stddev_percent):.1f}%)"


def _sparkline(series: list[dict[str, Any]], value_key: str = "native_ns") -> str:
    points = [
        (str(item.get("run_id", "")), item.get(value_key))
        for item in series
        if item.get(value_key) is not None
    ]
    if len(points) < 2:
        return '<span class="spark-empty">-</span>'

    values = [float(value) for _, value in points]
    minimum = min(values)
    maximum = max(values)
    width = 120.0
    height = 28.0
    padding = 2.0
    usable_width = width - (padding * 2)
    usable_height = height - (padding * 2)

    coordinates = []
    for index, value in enumerate(values):
        x = padding + (usable_width * index / max(1, len(values) - 1))
        if maximum == minimum:
            y = height / 2
        else:
            y = padding + usable_height * ((value - minimum) / (maximum - minimum))
        coordinates.append(f"{x:.1f},{y:.1f}")

    title = ", ".join(
        f"{run_id}: {_format_time(value)}" for run_id, value in points[-8:]
    )
    return (
        '<svg class="spark" viewBox="0 0 120 28" role="img" '
        f'aria-label="{html.escape(title)}">'
        f"<title>{html.escape(title)}</title>"
        '<polyline fill="none" stroke="currentColor" stroke-width="2" '
        f'points="{" ".join(coordinates)}" />'
        "</svg>"
    )


def _summary_value_markdown(value: Any) -> str:
    if value is None:
        return "-"
    if isinstance(value, dict):
        if not value:
            return "-"
        return "<br>".join(
            f"`{key}`: `{item}`" for key, item in value.items() if item is not None
        )
    if isinstance(value, list):
        if not value:
            return "-"
        return "<br>".join(f"`{item}`" for item in value)
    return f"`{value}`"


def render_summary_md(data: dict[str, Any]) -> str:
    latest_run = _latest_run(data)
    summary = data.get("latest_summary", {})
    comparisons = data.get("latest_comparisons", [])
    backend_summary = data.get("latest_backend_summary", [])
    backend_matrix = data.get("latest_backend_matrix", [])
    trends = data.get("latest_trends", [])
    reference_trends = data.get("latest_reference_trends", [])
    summary_trends = data.get("trend_summary", trend_summary(data))
    testbed = data.get("testbed_summary", testbed_summary(data))
    coverage = data.get("coverage", [])
    comparison_coverage = data.get("comparison_coverage", [])
    service_rows = data.get("service_decision", [])
    service_summary = data.get(
        "service_decision_summary",
        service_decision_summary(service_rows or service_decision()),
    )
    publication = data.get("publication_contract", publication_contract())
    contract = data.get("comparison_input_contract", comparison_input_contract())
    metric_contract = data.get(
        "comparison_metric_contract", comparison_metric_contract()
    )
    external_status = data.get(
        "external_competitor_status", external_competitor_status(data)
    )
    lines = [
        "# DART Performance Dashboard Summary",
        "",
        f"- Schema version: `{data.get('schema_version')}`",
        f"- Generated: `{data.get('generated_at')}`",
    ]
    thresholds = data.get("thresholds", threshold_policy())
    freshness = data.get("freshness", freshness_status(data))
    if latest_run is not None:
        lines.extend(
            [
                f"- Latest run: `{latest_run.get('run_id')}`",
                f"- Latest run time: `{latest_run.get('run_at')}`",
                f"- Branch: `{latest_run.get('branch')}`",
                f"- SHA: `{latest_run.get('sha')}`",
                f"- Testbed: `{latest_run.get('testbed')}`",
            ]
        )
    lines.extend(
        [
            "",
            "## Latest Counts",
            "",
            f"- Measurements: `{summary.get('measurements', 0)}`",
            f"- Runs: `{len(data.get('runs', []))}`",
            f"- Surfaces: `{len(summary.get('surfaces', {}))}`",
            f"- Families: `{len(summary.get('families', {}))}`",
            f"- Comparisons: `{summary.get('comparisons', {})}`",
            f"- Freshness: `{freshness.get('state')}`",
            f"- Latest run age hours: `{freshness.get('latest_run_age_hours')}`",
            f"- Threshold policy: `{thresholds.get('mode')}`",
            f"- Informational change threshold: `{thresholds.get('primary_change_percent')}%`",
            "",
            "## Testbed Summary",
            "",
            "| Field | Value |",
            "| ----- | ----- |",
        ]
    )
    for key, value in testbed.items():
        lines.append(f"| `{key}` | {_summary_value_markdown(value)} |")
    lines.extend(
        [
            "",
            "## Service Decision Summary",
            "",
            "| Field | Value |",
            "| ----- | ----- |",
        ]
    )
    for key, value in service_summary.items():
        lines.append(f"| `{key}` | {_summary_value_markdown(value)} |")
    lines.extend(
        [
            "",
            "## Dashboard Service Decision",
            "",
            "| Role | Service | State | Fit | Cost / OSS fit | Data owner | CI integration | Approval gate | Next step |",
            "| ---- | ------- | ----- | --- | -------------- | ---------- | -------------- | ------------- | --------- |",
        ]
    )
    for item in service_rows:
        lines.append(
            "| {role} | {service} | {state} | {fit} | {cost} | {owner} | {ci} | {approval} | {next_step} |".format(
                role=item.get("role", "-"),
                service=item.get("service", "-"),
                state=item.get("state", "-"),
                fit=item.get("fit", "-"),
                cost=item.get("cost_model", "-"),
                owner=item.get("data_owner", "-"),
                ci=item.get("ci_integration", "-"),
                approval=item.get("approval_gate", "-"),
                next_step=item.get("next_step", "-"),
            )
        )
    lines.extend(
        [
            "",
            "## Publication Contract",
            "",
            f"- Host: `{publication.get('host')}`",
            f"- Site URL: `{publication.get('site_url')}`",
            f"- Pages source: `{publication.get('source_branch')}` branch, "
            f"`{publication.get('source_path')}` path",
            f"- Dashboard path: `{publication.get('dashboard_path')}`",
            f"- Build type: `{publication.get('build_type')}`",
            f"- Publisher workflow: `{publication.get('publisher_workflow')}`",
            f"- Workflow name: `{publication.get('workflow_name')}`",
            f"- Publish ref: `{publication.get('publish_ref')}`",
            "- schedule_crons: `"
            + "`, `".join(publication.get("schedule_crons", []))
            + "`",
            "- expected_update_interval_hours: "
            f"`{publication.get('expected_update_interval_hours')}`",
            f"- stale_after_hours: `{publication.get('stale_after_hours')}`",
            f"- Requires Pages build request: "
            f"`{publication.get('requires_pages_build_request')}`",
            "- Required permissions: `"
            + "`, `".join(publication.get("required_permissions", []))
            + "`",
            "- Required endpoints: `"
            + "`, `".join(publication.get("required_endpoints", []))
            + "`",
            "- Required website links: `"
            + "`, `".join(publication.get("required_website_links", []))
            + "`",
            "- Required workflow artifacts: `"
            + "`, `".join(publication.get("required_workflow_artifacts", []))
            + "`",
            "- Launch checks: `"
            + "`, `".join(publication.get("launch_checks", []))
            + "`",
            f"- Approval boundary: {publication.get('approval_boundary')}",
        ]
    )
    lines.extend(
        [
            "",
            "## Comparison Input Contract",
            "",
            f"- Format: `{contract.get('format')}`",
            f"- Backend suffix rule: {contract.get('backend_suffix_rule')}",
            f"- Primary backend rule: {contract.get('primary_backend_rule')}",
            f"- Live claim rule: {contract.get('claim_rule')}",
            "- Supported external backends: `"
            + "`, `".join(contract.get("supported_external_backends", []))
            + "`",
            "- Example rows: `" + "`, `".join(contract.get("example_rows", [])) + "`",
            "",
            "## External Competitor Status",
            "",
            "| Field | Value |",
            "| ----- | ----- |",
        ]
    )
    for key, value in external_status.items():
        lines.append(f"| `{key}` | {_summary_value_markdown(value)} |")
    lines.extend(
        [
            "",
            "## Comparison Metric Contract",
            "",
            f"- Time unit: `{metric_contract.get('time_unit')}`",
            f"- Backend matrix ratio: "
            f"`{metric_contract.get('ratio_field')}` = "
            f"`{metric_contract.get('ratio_formula')}`",
            f"- Latest comparison ratio: "
            f"`{metric_contract.get('latest_comparison_ratio_field')}` = "
            f"`{metric_contract.get('latest_comparison_ratio_formula')}`",
            f"- Backend matrix interpretation: "
            f"{metric_contract.get('interpretation')}",
            f"- Latest comparison interpretation: "
            f"{metric_contract.get('latest_comparison_interpretation')}",
            "",
            "## Benchmark Surface Coverage",
            "",
            "| State | Surface | Latest rows | History runs | Entrypoint | Next step |",
            "| ----- | ------- | ----------- | ------------ | ---------- | --------- |",
        ]
    )
    for item in coverage:
        lines.append(
            "| {state} | {label} | {latest} | {runs} | `{entrypoint}` | {next_step} |".format(
                state=item["state"],
                label=item["label"],
                latest=item.get("latest_measurements", 0),
                runs=item.get("history_runs", 0),
                entrypoint=item["entrypoint"],
                next_step=item["next_step"],
            )
        )
    lines.extend(
        [
            "",
            "## Reference And Competitor Coverage",
            "",
            "| State | Surface | Primary backends | Reference backends | Compared groups | Live scope | Comparison target | External competitors | Next step |",
            "| ----- | ------- | ---------------- | ------------------ | --------------- | ---------- | ----------------- | -------------------- | --------- |",
        ]
    )
    for item in comparison_coverage:
        external = item["external_competitor_state"]
        live_external = ", ".join(item.get("external_competitor_backends", []))
        if live_external:
            external = (
                f"{external}; live: {live_external} "
                f"({item.get('external_competitor_row_count', 0)} rows)"
            )
        candidates = ", ".join(item.get("external_competitor_candidates", []))
        if candidates:
            external = f"{external}; candidates: {candidates}"
        lines.append(
            "| {state} | {surface} | `{primary}` | `{references}` | {groups} | {scope} | {target} | {external} | {next_step} |".format(
                state=item["state"],
                surface=item["label"],
                primary=", ".join(item.get("primary_backends", [])) or "-",
                references=", ".join(item.get("reference_backends", [])) or "-",
                groups=item.get("comparison_group_count", 0),
                scope=item["scope"],
                target=item.get("comparison_scope", "-"),
                external=external,
                next_step=item.get("next_step", "-"),
            )
        )
    lines.extend(
        [
            "",
            "## Trend Summary",
            "",
            "| Field | Value |",
            "| ----- | ----- |",
        ]
    )
    for key, value in summary_trends.items():
        lines.append(f"| `{key}` | {_summary_value_markdown(value)} |")
    lines.extend(
        [
            "",
            "## Latest Changes vs Previous Run",
            "",
            "| Status | Family | Group | Previous primary | Latest primary | Primary change | Threshold | Latest ratio | Ratio change |",
            "| ------ | ------ | ----- | ---------------- | -------------- | -------------- | --------- | ------------ | ------------ |",
        ]
    )
    for item in trends:
        lines.append(
            "| {status} | {family} | `{group}` | {previous} | {latest} | {change} | {threshold} | {ratio} | {ratio_change} |".format(
                status=item["status"],
                family=item["family"],
                group=item["comparable_group"],
                previous=_format_time(item.get("previous_native_ns")),
                latest=_format_time(item.get("latest_native_ns")),
                change=_format_change_label(item.get("native_change_percent")),
                threshold=_format_threshold_label(item),
                ratio=_format_ratio(item.get("latest_ratio")),
                ratio_change=_format_signed_percent(item.get("ratio_change_percent")),
            )
        )
    lines.extend(
        [
            "",
            "## Latest Reference Backend Changes vs Previous Run",
            "",
            "| Status | Family | Group | Backend | Previous reference | Latest reference | Reference change | Threshold | Latest backend/primary | Ratio change |",
            "| ------ | ------ | ----- | ------- | ------------------ | ---------------- | ---------------- | --------- | ---------------------- | ------------ |",
        ]
    )
    for item in reference_trends:
        lines.append(
            "| {status} | {family} | `{group}` | `{backend}` | {previous} | {latest} | {change} | {threshold} | {ratio} | {ratio_change} |".format(
                status=item["status"],
                family=item["family"],
                group=item["comparable_group"],
                backend=item["backend"],
                previous=_format_time(item.get("previous_backend_ns")),
                latest=_format_time(item.get("latest_backend_ns")),
                change=_format_change_label(item.get("backend_change_percent")),
                threshold=_format_threshold_label(item),
                ratio=_format_ratio(item.get("latest_backend_vs_primary_ratio")),
                ratio_change=_format_signed_percent(item.get("ratio_change_percent")),
            )
        )
    lines.extend(
        [
            "",
            "## Latest Comparison Rows",
            "",
            "| Status | Family | Group | Native/primary | Best reference | Ratio |",
            "| ------ | ------ | ----- | -------------- | -------------- | ----- |",
        ]
    )
    for item in comparisons:
        primary = item.get("primary_backend") or "Native"
        reference = item.get("best_reference_backend") or "-"
        reference_time = _format_time(item.get("best_reference_ns"))
        lines.append(
            "| {status} | {family} | `{group}` | {native} | {reference}: {ref_time} | {ratio} |".format(
                status=item["status"],
                family=item["family"],
                group=item["comparable_group"],
                native=f"{_format_time(item.get('native_ns'))} ({primary})",
                reference=reference,
                ref_time=reference_time,
                ratio=_format_ratio(item.get("ratio")),
            )
        )
    lines.extend(
        [
            "",
            "## Latest Reference Backend Summary",
            "",
            "| Status | Surface | Backend | Groups | Primary faster | Reference faster | Geomean backend/primary | Range |",
            "| ------ | ------- | ------- | ------ | -------------- | ---------------- | ----------------------- | ----- |",
        ]
    )
    for item in backend_summary:
        ratio_range = (
            f"{_format_ratio(item.get('min_backend_vs_primary_ratio'))} - "
            f"{_format_ratio(item.get('max_backend_vs_primary_ratio'))}"
        )
        lines.append(
            "| {status} | {surface} | `{backend}` | {groups} | {primary_faster} | {backend_faster} | {geomean} | {ratio_range} |".format(
                status=item["status"],
                surface=item["surface"],
                backend=item["backend"],
                groups=item["groups"],
                primary_faster=item["primary_faster"],
                backend_faster=item["backend_faster"],
                geomean=_format_ratio(item.get("geomean_backend_vs_primary_ratio")),
                ratio_range=ratio_range,
            )
        )
    lines.extend(
        [
            "",
            "## Latest Backend Matrix",
            "",
            "| Status | Family | Group | Backend | Primary | Time | Backend/primary | Mean/stddev |",
            "| ------ | ------ | ----- | ------- | ------- | ---- | --------------- | ----------- |",
        ]
    )
    for item in backend_matrix:
        lines.append(
            "| {status} | {family} | `{group}` | `{backend}` | `{primary}` | {time} | {ratio} | {mean_stddev} |".format(
                status=item["status"],
                family=item["family"],
                group=item["comparable_group"],
                backend=item["backend"],
                primary=item["primary_backend"],
                time=_format_time(item.get("backend_ns")),
                ratio=_format_ratio(item.get("backend_vs_primary_ratio")),
                mean_stddev=_format_mean_stddev(item),
            )
        )
    lines.append("")
    return "\n".join(lines)


def external_competitor_status(data: dict[str, Any]) -> dict[str, Any]:
    comparison_coverage = data.get("comparison_coverage", [])
    external_competitor_rows = [
        item
        for item in comparison_coverage
        if int(item.get("external_competitor_row_count") or 0) > 0
    ]
    row_count = sum(
        int(item.get("external_competitor_row_count") or 0)
        for item in comparison_coverage
    )
    backends = sorted(
        {
            str(backend)
            for item in comparison_coverage
            for backend in item.get("external_competitor_backends", [])
        }
    )
    surfaces = sorted(str(item.get("surface")) for item in external_competitor_rows)
    sample_groups = sorted(
        {
            str(group)
            for item in comparison_coverage
            for group in item.get("external_competitor_sample_groups", [])
        }
    )
    contract = data.get("comparison_input_contract", comparison_input_contract())
    return {
        "state": "live" if row_count else "queued",
        "row_count": row_count,
        "backends": backends,
        "surfaces": surfaces,
        "sample_groups": sample_groups,
        "supported_external_backends": contract.get(
            "supported_external_backends",
            comparison_input_contract()["supported_external_backends"],
        ),
        "claim_rule": contract.get(
            "claim_rule", comparison_input_contract()["claim_rule"]
        ),
        "data_source": "comparison_coverage",
    }


def status_manifest(data: dict[str, Any]) -> dict[str, Any]:
    latest_run = _latest_run(data) or {}
    summary = data.get("latest_summary", {})
    coverage = data.get("coverage", [])
    coverage_counts = Counter(item.get("state") for item in coverage)
    external_status = data.get(
        "external_competitor_status", external_competitor_status(data)
    )
    return {
        "schema_version": data.get("schema_version"),
        "generated_at": data.get("generated_at"),
        "latest_run_id": data.get("latest_run_id"),
        "latest_run_at": latest_run.get("run_at"),
        "branch": latest_run.get("branch"),
        "sha": latest_run.get("sha"),
        "testbed": latest_run.get("testbed"),
        "source_url": latest_run.get("source_url"),
        "measurements": summary.get("measurements", 0),
        "runs": len(data.get("runs", [])),
        "live_surfaces": coverage_counts.get("live", 0),
        "queued_surfaces": coverage_counts.get("queued", 0),
        "comparison_states": summary.get("comparisons", {}),
        "thresholds": data.get("thresholds", threshold_policy()),
        "freshness": data.get("freshness", freshness_status(data)),
        "links": data.get("links", dashboard_links()),
        "service_decision": data.get("service_decision", service_decision()),
        "service_decision_summary": data.get(
            "service_decision_summary",
            service_decision_summary(data.get("service_decision", service_decision())),
        ),
        "publication_contract": data.get(
            "publication_contract", publication_contract()
        ),
        "comparison_input_contract": data.get(
            "comparison_input_contract", comparison_input_contract()
        ),
        "comparison_metric_contract": data.get(
            "comparison_metric_contract", comparison_metric_contract()
        ),
        "backend_summary_rows": len(data.get("latest_backend_summary", [])),
        "backend_matrix_rows": len(data.get("latest_backend_matrix", [])),
        "reference_trend_rows": len(data.get("latest_reference_trends", [])),
        "trend_summary": data.get("trend_summary", trend_summary(data)),
        "testbed_summary": data.get("testbed_summary", testbed_summary(data)),
        "comparison_coverage_rows": len(data.get("comparison_coverage", [])),
        "external_competitor_status": external_status,
        "external_competitor_rows": external_status.get("row_count", 0),
        "external_competitor_backends": external_status.get("backends", []),
        "external_competitor_surfaces": external_status.get("surfaces", []),
        "external_competitor_sample_groups": external_status.get("sample_groups", []),
        "dashboard_files": [
            "index.html",
            "data.json",
            "summary.md",
            "status.json",
        ],
    }


def _clean_output_dir(output_dir: Path) -> None:
    if not output_dir.exists():
        return
    if not output_dir.is_dir():
        raise RuntimeError(f"Dashboard output path is not a directory: {output_dir}")

    resolved = output_dir.resolve()
    unsafe_roots = {Path.cwd().resolve(), Path(resolved.anchor)}
    if resolved in unsafe_roots:
        raise RuntimeError(
            f"Refusing to clean unsafe dashboard output path: {output_dir}"
        )

    for child in output_dir.iterdir():
        if child.is_dir() and not child.is_symlink():
            shutil.rmtree(child)
        else:
            child.unlink()


def write_dashboard(data: dict[str, Any], output_dir: Path, title: str) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "data.json").write_text(
        json.dumps(data, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (output_dir / "index.html").write_text(
        render_html(data, title),
        encoding="utf-8",
    )
    (output_dir / "summary.md").write_text(
        render_summary_md(data),
        encoding="utf-8",
    )
    (output_dir / "status.json").write_text(
        json.dumps(status_manifest(data), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    args = parse_args()
    paths = _collect_inputs(args.input, args.allow_empty)
    measurements, contexts = load_measurements(paths)
    run = _new_run(args, paths, contexts)
    history_path = args.history
    if history_path is None:
        candidate = args.output_dir / "data.json"
        history_path = candidate if candidate.exists() else None
    data = _load_history(history_path)
    data = _merge_seed_history(data, args.seed_input)
    if _can_preserve_history_on_empty_run(data, measurements, args.allow_empty):
        data["generated_at"] = _iso_now()
        _refresh_derived_fields(data)
    else:
        data = _merge_history(data, run, measurements)
    if args.clean_output:
        _clean_output_dir(args.output_dir)
    write_dashboard(data, args.output_dir, args.title)

    summary = data["latest_summary"]
    print(
        "generated dashboard: {output_dir} "
        "({measurements} measurements, {comparisons} comparison states)".format(
            output_dir=args.output_dir,
            measurements=summary["measurements"],
            comparisons=summary["comparisons"],
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
