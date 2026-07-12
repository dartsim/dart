#!/usr/bin/env python3
"""Compare soft-body benchmark matrices across git revisions."""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

SCENES = {
    0: "adaptive_deformable",
    1: "soft_cubes",
    2: "soft_bodies",
    3: "soft_open_chain",
}
TIME_UNIT_TO_MS = {
    "ns": 1.0e-6,
    "us": 1.0e-3,
    "ms": 1.0,
    "s": 1.0e3,
}
HARNESS_FILES = [
    Path("tests/benchmark/integration/bm_soft_body.cpp"),
    Path("tests/benchmark/integration/soft_body_headless.cpp"),
]
REFERENCE_DETECTOR = "dart"
UNSUPPORTED_SHAPE_WARNING_FRAGMENTS = [
    "not supported by dartcollisiondetector",
    "not supported by nativecollisiondetector",
    "shape will be skipped by the native adapter",
    "unsupported shape type",
    "always get penetrated",
]
CHECKSUM_METRICS = [
    "skelPosL1",
    "skelPosSq",
    "skelVelL1",
    "skelVelSq",
    "pointPosL1",
    "pointPosSq",
    "pointVelL1",
    "pointVelSq",
    "pointWorldPosL1",
    "pointWorldPosSq",
]
HARNESS_CMAKE_BLOCK = """

if(TARGET dart-utils)
  dart_benchmarks(
    TYPE BM_INTEGRATION
    SOURCES
      bm_soft_body.cpp
    LINK_LIBRARIES
      dart-utils
  )

  add_executable(soft_body_headless soft_body_headless.cpp)
  target_link_libraries(soft_body_headless PRIVATE dart-utils)
endif()
"""
HARNESS_COLLISION_LINK_BLOCK = """

if(TARGET BM_INTEGRATION_soft_body)
  foreach(soft_body_collision_library dart-collision-bullet dart-collision-ode)
    if(TARGET ${soft_body_collision_library})
      target_link_libraries(
        BM_INTEGRATION_soft_body
        PRIVATE ${soft_body_collision_library}
      )
      target_link_libraries(
        soft_body_headless
        PRIVATE ${soft_body_collision_library}
      )
    endif()
  endforeach()
endif()
"""


@dataclass(frozen=True)
class Revision:
    label: str
    rev: str
    sha: str
    source_dir: Path
    build_dir: Path


@dataclass(frozen=True)
class BenchmarkRow:
    revision: str
    detector: str
    scene: str
    threads: int
    cpu_ms: float
    real_ms: float
    sim_s_per_s: float | None
    name: str
    cpu_median_ms: float | None = None
    cpu_stddev_ms: float | None = None
    sample_count: int = 1


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--current", default="HEAD")
    parser.add_argument("--parent", default="HEAD^1")
    parser.add_argument("--base")
    parser.add_argument("--detectors", default="fcl,dart,native,bullet,ode")
    parser.add_argument(
        "--expected-fastest-detector",
        default="native",
        help=(
            "Detector expected to be the fastest eligible current-row backend. "
            f"The checksum reference remains {REFERENCE_DETECTOR}."
        ),
    )
    parser.add_argument(
        "--expected-fastest-tie-tolerance",
        type=float,
        default=0.02,
        help=(
            "Relative CPU-time margin within which the expected fastest "
            "detector counts as matching the winning row. The acceptance "
            "wording is 'matches or beats'; near-tied backends sharing the "
            "same narrow-phase kernels would otherwise fail on scheduler "
            "noise. Set to 0 to require strict wins."
        ),
    )
    parser.add_argument("--threads", default="1,16")
    parser.add_argument("--benchmark-min-time", default="0.05s")
    parser.add_argument("--benchmark-repetitions", default="3")
    parser.add_argument(
        "--benchmark-cycles",
        type=int,
        default=1,
        help=(
            "Number of balanced benchmark cycles to run. Cycles alternate "
            "revision order so parent/current timing drift is less likely to "
            "look like a detector regression."
        ),
    )
    parser.add_argument(
        "--benchmark-run-order",
        choices=("detector", "revision"),
        default="detector",
        help=(
            "Run order for benchmark binaries. 'detector' interleaves "
            "revisions for each collision detector to reduce parent/current "
            "order drift; 'revision' preserves the older all-detectors-per-"
            "revision order."
        ),
    )
    parser.add_argument("--correctness-scenes", default="soft_cubes,soft_bodies")
    parser.add_argument("--correctness-steps", default="200")
    parser.add_argument("--correctness-tolerance", type=float, default=0.05)
    parser.add_argument(
        "--wait-for-local-dart-builds",
        action="store_true",
        help=(
            "Before each benchmark run, wait until local DART build/test "
            "workloads from sibling worktrees are idle."
        ),
    )
    parser.add_argument(
        "--idle-timeout",
        type=float,
        default=1800.0,
        help="Maximum seconds to wait for local build idleness.",
    )
    parser.add_argument(
        "--idle-poll-interval",
        type=float,
        default=5.0,
        help="Seconds between local build idleness checks.",
    )
    parser.add_argument(
        "--idle-cooldown",
        type=float,
        default=0.0,
        help=(
            "Seconds to wait after local DART workloads become idle before "
            "starting each benchmark run."
        ),
    )
    parser.add_argument(
        "--idle-max-load-1m",
        type=float,
        default=None,
        help=(
            "Optional maximum 1-minute system load average allowed before "
            "starting each benchmark run."
        ),
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--keep-going",
        action="store_true",
        help="Continue after an individual detector run fails.",
    )
    return parser.parse_args(argv)


def run(
    command: list[str],
    *,
    cwd: Path,
    env: dict[str, str] | None = None,
    capture_path: Path | None = None,
) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        command,
        cwd=cwd,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    if capture_path is not None:
        capture_path.parent.mkdir(parents=True, exist_ok=True)
        capture_path.write_text(result.stdout, encoding="utf-8")
    if result.returncode != 0:
        raise subprocess.CalledProcessError(
            result.returncode, command, output=result.stdout
        )
    return result


def git(root: Path, *args: str) -> str:
    result = run(["git", *args], cwd=root)
    return result.stdout.strip()


def repo_root() -> Path:
    return Path(git(Path.cwd(), "rev-parse", "--show-toplevel"))


def local_dart_workspace_root(root: Path) -> Path:
    for path in (root, *root.parents):
        if path.name == "dart":
            return path
    return root


def count_local_dart_workloads(root: Path) -> int:
    workspace = str(local_dart_workspace_root(root))
    build_tools = {"cc1plus", "clang-format", "cmake", "ninja"}
    python_tools = {"python", "python3", "python3.14", "pytest"}
    benchmark_markers = ("/BM_", "BM_INTEGRATION_", "/soft_body_headless")
    try:
        result = subprocess.run(
            ["ps", "-eo", "comm=,args="],
            cwd=root,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    except OSError:
        return 0

    if result.returncode != 0:
        return 0

    count = 0
    for line in result.stdout.splitlines():
        columns = line.split(maxsplit=1)
        if not columns:
            continue

        command = Path(columns[0]).name
        arguments = columns[1] if len(columns) > 1 else ""
        if workspace not in arguments:
            continue

        if command in build_tools:
            count += 1
        elif command in python_tools and (
            "cmake_build.py" in arguments or "pytest" in arguments
        ):
            count += 1
        elif any(marker in arguments for marker in benchmark_markers):
            count += 1
    return count


def wait_for_local_dart_idleness(
    root: Path,
    timeout: float,
    poll_interval: float,
    cooldown: float,
    max_load_1m: float | None,
) -> None:
    deadline = time.monotonic() + timeout
    last_count: int | None = None
    cooldown_complete = cooldown <= 0.0

    while True:
        count = count_local_dart_workloads(root)
        if count != 0:
            cooldown_complete = cooldown <= 0.0

        if count == 0:
            if max_load_1m is not None:
                load_1m = os.getloadavg()[0]
                if load_1m > max_load_1m:
                    print(
                        "waiting for 1-minute load average to drop below "
                        f"{max_load_1m:g}: {load_1m:.2f}",
                        file=sys.stderr,
                    )
                    time.sleep(max(0.1, poll_interval))
                    continue

            if not cooldown_complete:
                print(
                    f"cooling down for {cooldown:g}s before benchmark timing",
                    file=sys.stderr,
                )
                time.sleep(max(0.0, cooldown))
                cooldown_complete = True
                continue

            if last_count:
                print("local DART workloads are idle", file=sys.stderr)
            return

        if count != last_count:
            print(
                f"waiting for local DART workloads to become idle: {count}",
                file=sys.stderr,
            )
            last_count = count

        if time.monotonic() >= deadline:
            raise TimeoutError(
                "timed out waiting for local DART workloads to become idle"
            )

        time.sleep(max(0.1, poll_interval))


def parse_csv(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def parse_threads(value: str) -> set[int]:
    return {int(item) for item in parse_csv(value)}


def ensure_harness(source_dir: Path, harness_source: Path) -> bool:
    patched = False
    for relative in HARNESS_FILES:
        source = harness_source / relative
        destination = source_dir / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        if not destination.exists() or destination.read_bytes() != source.read_bytes():
            shutil.copy2(source, destination)
            patched = True

    cmake_path = source_dir / "tests/benchmark/integration/CMakeLists.txt"
    cmake_text = cmake_path.read_text(encoding="utf-8")
    if "bm_soft_body.cpp" not in cmake_text:
        cmake_path.write_text(cmake_text.rstrip() + HARNESS_CMAKE_BLOCK + "\n")
        patched = True
        cmake_text = cmake_path.read_text(encoding="utf-8")
    if "soft_body_collision_library" not in cmake_text:
        cmake_path.write_text(
            cmake_text.rstrip() + HARNESS_COLLISION_LINK_BLOCK + "\n",
            encoding="utf-8",
        )
        patched = True
    return patched


def prepare_revision(
    root: Path,
    output_dir: Path,
    label: str,
    rev: str,
    harness_source: Path | None,
) -> Revision:
    sha = git(root, "rev-parse", rev)
    source_dir = output_dir / "worktrees" / f"{label}-{sha[:12]}"
    if not source_dir.exists():
        source_dir.parent.mkdir(parents=True, exist_ok=True)
        run(["git", "worktree", "add", "--detach", str(source_dir), sha], cwd=root)
    else:
        existing = git(source_dir, "rev-parse", "HEAD")
        if existing != sha:
            raise SystemExit(
                f"Existing worktree {source_dir} is at {existing}, expected {sha}."
            )

    patched = False
    if harness_source is not None:
        patched = ensure_harness(source_dir, harness_source)
    else:
        missing = [
            str(relative)
            for relative in HARNESS_FILES
            if not (source_dir / relative).is_file()
        ]
        if missing:
            raise SystemExit(
                f"Revision {sha} does not own the soft-body benchmark harness: "
                + ", ".join(missing)
            )
    build_dir = output_dir / "build" / f"{label}-{sha[:12]}"
    metadata = {
        "label": label,
        "rev": rev,
        "sha": sha,
        "source_dir": str(source_dir),
        "build_dir": str(build_dir),
        "benchmark_harness_patched": patched,
    }
    (output_dir / f"{label}.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )
    return Revision(label, rev, sha, source_dir, build_dir)


def configure_and_build(revision: Revision) -> tuple[Path, Path]:
    conda_prefix = os.environ.get("CONDA_PREFIX", "")
    cmake_command = [
        "cmake",
        "-G",
        "Ninja",
        "-S",
        str(revision.source_dir),
        "-B",
        str(revision.build_dir),
        f"-DCMAKE_INSTALL_PREFIX={conda_prefix}",
        "-DCMAKE_BUILD_TYPE=Release",
        f"-DCMAKE_PREFIX_PATH={conda_prefix}",
        "-DCMAKE_C_COMPILER_LAUNCHER=",
        "-DCMAKE_CXX_COMPILER_LAUNCHER=",
        "-DDART_BUILD_DARTPY=OFF",
        "-DDART_BUILD_PROFILE=ON",
        "-DDART_USE_SYSTEM_GOOGLEBENCHMARK=ON",
        "-DDART_USE_SYSTEM_GOOGLETEST=ON",
        "-DDART_USE_SYSTEM_IMGUI=ON",
        "-DDART_USE_SYSTEM_TRACY=ON",
    ]
    run(cmake_command, cwd=revision.source_dir)
    run(
        [
            "cmake",
            "--build",
            str(revision.build_dir),
            "--target",
            "BM_INTEGRATION_soft_body",
            "soft_body_headless",
            "--parallel",
        ],
        cwd=revision.source_dir,
    )

    benchmark_candidates = [
        revision.build_dir / "bin/BM_INTEGRATION_soft_body",
        revision.build_dir / "tests/benchmark/integration/BM_INTEGRATION_soft_body",
    ]
    benchmark_binary = None
    for candidate in benchmark_candidates:
        if candidate.exists():
            benchmark_binary = candidate
            break
    if benchmark_binary is None:
        raise SystemExit(
            f"BM_INTEGRATION_soft_body binary not found for {revision.label}"
        )

    headless_candidates = [
        revision.build_dir / "bin/soft_body_headless",
        revision.build_dir / "tests/benchmark/integration/soft_body_headless",
    ]
    headless_binary = None
    for candidate in headless_candidates:
        if candidate.exists():
            headless_binary = candidate
            break
    if headless_binary is None:
        raise SystemExit(f"soft_body_headless binary not found for {revision.label}")

    return benchmark_binary, headless_binary


def run_detector(
    revision: Revision,
    binary: Path,
    detector: str,
    output_dir: Path,
    min_time: str,
    repetitions: str,
    cycle_index: int | None,
) -> tuple[Path | None, str | None]:
    suffix = "" if cycle_index is None else f"-cycle{cycle_index + 1}"
    json_path = output_dir / "raw" / f"{revision.label}-{detector}{suffix}.json"
    log_path = output_dir / "logs" / f"{revision.label}-{detector}{suffix}.log"
    json_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["COLLISION_DETECTOR"] = detector
    command = [
        str(binary),
        "--benchmark_filter=BM_SoftBodyStep/.*",
        f"--benchmark_min_time={min_time}",
        f"--benchmark_repetitions={repetitions}",
        "--benchmark_format=json",
        f"--benchmark_out={json_path}",
        "--benchmark_out_format=json",
    ]
    try:
        run(command, cwd=revision.source_dir, env=env, capture_path=log_path)
    except subprocess.CalledProcessError as exc:
        return None, exc.output
    if not json_path.exists():
        return None, f"missing benchmark JSON: {json_path}"
    return json_path, None


def parse_label(label: str) -> tuple[str | None, str | None, int | None]:
    scene_match = re.search(r"scene=([^ ]+)", label)
    detector_match = re.search(r"detector=([^ ]+)", label)
    threads_match = re.search(r"threads=([0-9]+)", label)
    return (
        scene_match.group(1) if scene_match else None,
        detector_match.group(1) if detector_match else None,
        int(threads_match.group(1)) if threads_match else None,
    )


def parse_name(name: str) -> tuple[str | None, int | None]:
    trimmed = re.sub(r"_(mean|median|stddev|cv)$", "", name)
    parts = trimmed.split("/")
    if len(parts) < 4:
        return None, None
    try:
        scene = SCENES[int(parts[1])]
        threads = int(parts[2])
    except (KeyError, ValueError):
        return None, None
    return scene, threads


def load_rows(
    revision: Revision,
    detector: str,
    json_path: Path,
    requested_threads: set[int],
) -> list[BenchmarkRow]:
    data = json.loads(json_path.read_text(encoding="utf-8"))
    iteration_rows: list[BenchmarkRow] = []
    aggregate_rows: dict[tuple[str, str, int], dict[str, BenchmarkRow]] = {}
    for benchmark in data.get("benchmarks", []):
        run_type = benchmark.get("run_type")
        if run_type not in {"iteration", "aggregate"}:
            continue
        if benchmark.get("error_occurred"):
            continue

        label_scene, label_detector, label_threads = parse_label(
            benchmark.get("label", "")
        )
        name_scene, name_threads = parse_name(benchmark.get("name", ""))
        scene = label_scene or name_scene
        threads = label_threads if label_threads is not None else name_threads
        if scene is None or threads is None or threads not in requested_threads:
            continue

        unit = benchmark.get("time_unit", "ns")
        multiplier = TIME_UNIT_TO_MS.get(unit)
        if multiplier is None:
            raise SystemExit(f"Unsupported benchmark time unit: {unit}")

        row = BenchmarkRow(
            revision=revision.label,
            detector=label_detector or detector,
            scene=scene,
            threads=threads,
            cpu_ms=float(benchmark["cpu_time"]) * multiplier,
            real_ms=float(benchmark["real_time"]) * multiplier,
            sim_s_per_s=(
                float(benchmark["sim_s/s"]) if "sim_s/s" in benchmark else None
            ),
            name=benchmark.get("name", ""),
        )
        if run_type == "iteration":
            iteration_rows.append(row)
        else:
            aggregate_name = str(benchmark.get("aggregate_name", ""))
            if aggregate_name:
                aggregate_rows.setdefault(row_key(row), {})[aggregate_name] = row

    if iteration_rows:
        return iteration_rows

    rows = []
    for aggregates in aggregate_rows.values():
        mean_row = aggregates.get("mean")
        if mean_row is None:
            continue
        median_row = aggregates.get("median")
        stddev_row = aggregates.get("stddev")
        rows.append(
            BenchmarkRow(
                revision=mean_row.revision,
                detector=mean_row.detector,
                scene=mean_row.scene,
                threads=mean_row.threads,
                cpu_ms=mean_row.cpu_ms,
                real_ms=mean_row.real_ms,
                sim_s_per_s=mean_row.sim_s_per_s,
                name=mean_row.name,
                cpu_median_ms=median_row.cpu_ms if median_row else None,
                cpu_stddev_ms=stddev_row.cpu_ms if stddev_row else None,
                sample_count=1,
            )
        )
    return rows


def aggregate_sample_rows(rows: list[BenchmarkRow]) -> BenchmarkRow:
    if not rows:
        raise ValueError("Cannot aggregate an empty benchmark row list")

    cpu_values = [row.cpu_ms for row in rows]
    real_values = [row.real_ms for row in rows]
    sim_values = [row.sim_s_per_s for row in rows if row.sim_s_per_s is not None]
    sample_count = len(rows)
    return BenchmarkRow(
        revision=rows[0].revision,
        detector=rows[0].detector,
        scene=rows[0].scene,
        threads=rows[0].threads,
        cpu_ms=statistics.fmean(cpu_values),
        real_ms=statistics.fmean(real_values),
        sim_s_per_s=statistics.fmean(sim_values) if sim_values else None,
        name=rows[0].name,
        cpu_median_ms=statistics.median(cpu_values),
        cpu_stddev_ms=statistics.stdev(cpu_values) if sample_count > 1 else 0.0,
        sample_count=sample_count,
    )


def benchmark_error_message(json_path: Path) -> str | None:
    data = json.loads(json_path.read_text(encoding="utf-8"))
    messages = {
        str(benchmark.get("error_message", "benchmark error"))
        for benchmark in data.get("benchmarks", [])
        if benchmark.get("error_occurred")
    }
    return "; ".join(sorted(messages)) if messages else None


def parse_checksum_line(output: str, expected_step: str) -> dict[str, float] | None:
    for line in output.splitlines():
        tokens = line.split()
        if len(tokens) < 4 or tokens[0] != "step" or tokens[1] != expected_step:
            continue

        values: dict[str, float] = {}
        index = 2
        while index + 1 < len(tokens):
            key = tokens[index]
            value = tokens[index + 1]
            try:
                values[key] = float(value)
            except ValueError:
                pass
            index += 2
        return values
    return None


def checksums_match(
    reference: dict[str, float],
    candidate: dict[str, float],
    tolerance: float,
) -> tuple[bool, list[str]]:
    mismatches = []
    for metric in CHECKSUM_METRICS:
        ref_value = reference.get(metric)
        candidate_value = candidate.get(metric)
        if ref_value is None or candidate_value is None:
            mismatches.append(f"{metric}=missing")
            continue

        allowed = 1.0e-8 + tolerance * max(1.0, abs(ref_value))
        delta = abs(candidate_value - ref_value)
        if delta > allowed:
            mismatches.append(f"{metric} delta {delta:.6g} > {allowed:.6g}")
    return not mismatches, mismatches


def has_unsupported_shape_warning(output: str) -> bool:
    lowered = output.lower()
    return any(fragment in lowered for fragment in UNSUPPORTED_SHAPE_WARNING_FRAGMENTS)


def run_headless_checksum(
    revision: Revision,
    headless_binary: Path,
    detector: str,
    scene: str,
    steps: str,
    output_dir: Path,
    thread_count: int = 1,
) -> tuple[dict[str, float] | None, str]:
    log_path = (
        output_dir
        / "logs"
        / f"{revision.label}-{detector}-{scene}-t{thread_count}-headless.log"
    )
    env = os.environ.copy()
    env["COLLISION_DETECTOR"] = detector
    env["THREADS"] = str(thread_count)
    command = [str(headless_binary), scene, steps, steps]
    try:
        result = run(command, cwd=revision.source_dir, env=env, capture_path=log_path)
    except subprocess.CalledProcessError as exc:
        return None, exc.output

    checksum = parse_checksum_line(result.stdout, steps)
    return checksum, result.stdout


def evaluate_detector_equivalence(
    current: Revision,
    headless_binary: Path,
    detectors: list[str],
    scenes: list[str],
    steps: str,
    tolerance: float,
    output_dir: Path,
    thread_count: int = 1,
) -> tuple[dict[str, dict[str, object]], list[str]]:
    detector_results: dict[str, dict[str, object]] = {}
    eligible = []
    reference_by_scene: dict[str, dict[str, float]] = {}

    for scene in scenes:
        checksum, output = run_headless_checksum(
            current,
            headless_binary,
            REFERENCE_DETECTOR,
            scene,
            steps,
            output_dir,
            thread_count,
        )
        if checksum is None:
            detector_results[REFERENCE_DETECTOR] = {
                "eligible": False,
                "reason": f"reference detector failed for {scene}",
                "details": output.splitlines()[-1] if output else "failed",
            }
            return detector_results, eligible
        reference_by_scene[scene] = checksum

    for detector in detectors:
        if detector == REFERENCE_DETECTOR:
            detector_results[detector] = {
                "eligible": True,
                "reason": "reference detector",
                "scenes": scenes,
                "threads": thread_count,
            }
            eligible.append(detector)
            continue

        detector_ok = True
        reasons = []
        for scene in scenes:
            checksum, output = run_headless_checksum(
                current,
                headless_binary,
                detector,
                scene,
                steps,
                output_dir,
                thread_count,
            )
            if checksum is None:
                detector_ok = False
                reasons.append(f"{scene}: failed")
                continue
            if has_unsupported_shape_warning(output):
                detector_ok = False
                reasons.append(f"{scene}: unsupported soft shape fallback")
                continue

            matches, mismatches = checksums_match(
                reference_by_scene[scene], checksum, tolerance
            )
            if not matches:
                detector_ok = False
                reasons.append(f"{scene}: {'; '.join(mismatches[:3])}")

        detector_results[detector] = {
            "eligible": detector_ok,
            "reason": "checksum-equivalent" if detector_ok else "; ".join(reasons),
            "scenes": scenes,
            "threads": thread_count,
        }
        if detector_ok:
            eligible.append(detector)

    return detector_results, eligible


def row_key(row: BenchmarkRow) -> tuple[str, str, int]:
    return (row.detector, row.scene, row.threads)


def pct_change(new: float, old: float) -> float:
    return 100.0 * (new - old) / old if old != 0.0 else float("inf")


def fmt_float(value: float | None, digits: int = 3) -> str:
    if value is None:
        return "n/a"
    return f"{value:.{digits}f}"


def fmt_cpu_with_std(mean: float, stddev: float | None, samples: int | None) -> str:
    if samples and samples > 1 and stddev is not None:
        return f"{mean:.3f} +/- {stddev:.3f}"
    return f"{mean:.3f}"


def write_cpu_change_graph(
    lines: list[str], comparison_items: list[dict[str, object]]
) -> None:
    valid_items = [item for item in comparison_items if "missing" not in item]
    if not valid_items:
        return

    label_width = max(
        len(f"{item['detector']}/{item['scene']}/{item['threads']}")
        for item in valid_items
    )
    bar_width = 16
    max_abs_change = max(
        1.0, *(abs(float(item["cpu_change_pct"])) for item in valid_items)
    )
    lines.extend(
        [
            "```text",
            f"{'detector/scene/threads':<{label_width}}  "
            f"{'faster':>{bar_width}}|{'slower':<{bar_width}}  change  scope",
        ]
    )
    for item in valid_items:
        change = float(item["cpu_change_pct"])
        units = int(round(abs(change) / max_abs_change * bar_width))
        units = min(bar_width, max(0, units))
        if change < 0.0:
            left = "." * (bar_width - units) + "#" * units
            right = "." * bar_width
        else:
            left = "." * bar_width
            right = "!" * units + "." * (bar_width - units)
        label = f"{item['detector']}/{item['scene']}/{item['threads']}"
        scope = "apples" if item.get("apples_to_apples", True) else "diagnostic"
        lines.append(f"{label:<{label_width}}  {left}|{right}  {change:+.1f}%  {scope}")
    lines.append("```")


def write_detector_winner_graph(
    lines: list[str], fastest: list[dict[str, object]]
) -> None:
    if not fastest:
        return

    bar_width = 24
    lines.extend(
        [
            "Detector CPU graph, lower is better. `*` marks the winning row.",
            "",
            "```text",
        ]
    )
    for item in fastest:
        detectors = {
            str(detector): float(cpu_ms)
            for detector, cpu_ms in item["detectors"].items()
        }
        if not detectors:
            continue

        max_cpu = max(detectors.values())
        if max_cpu <= 0.0:
            max_cpu = 1.0

        parts = []
        for detector, cpu_ms in sorted(detectors.items()):
            units = int(round(cpu_ms / max_cpu * bar_width))
            units = min(bar_width, max(1, units))
            marker = "*" if detector == item["winner"] else " "
            parts.append(
                f"{marker}{detector}:{'#' * units:<{bar_width}} "
                f"{cpu_ms:.3f}ms"
            )

        lines.append(
            "{scene}/{threads:<2} {parts}".format(
                scene=item["scene"],
                threads=item["threads"],
                parts=" | ".join(parts),
            )
        )
    lines.append("```")


def build_comparison(
    current_rows: dict[tuple[str, str, int], BenchmarkRow],
    other_rows: dict[tuple[str, str, int], BenchmarkRow],
    other_label: str,
    detector_equivalence: dict[str, dict[str, object]],
    unsupported_detector_runs: dict[tuple[str, str], bool],
) -> list[dict[str, object]]:
    comparison = []
    for key in sorted(current_rows):
        current = current_rows[key]
        current_supported = not unsupported_detector_runs.get(
            ("current", current.detector), False
        )
        current_equivalent = bool(
            detector_equivalence.get(current.detector, {}).get("eligible", False)
        )
        other_supported = not unsupported_detector_runs.get(
            (other_label, current.detector), False
        )
        apples_to_apples = current_supported and current_equivalent and other_supported
        diagnostic_reasons = []
        if not current_supported:
            diagnostic_reasons.append("current emitted unsupported-shape warnings")
        if not current_equivalent:
            diagnostic_reasons.append(
                str(
                    detector_equivalence.get(current.detector, {}).get(
                        "reason", "not checksum-equivalent to reference detector"
                    )
                )
            )
        if not other_supported:
            diagnostic_reasons.append(
                f"{other_label} emitted unsupported-shape warnings"
            )

        other = other_rows.get(key)
        if other is None:
            diagnostic_reasons.append(f"missing {other_label} row")
            comparison.append(
                {
                    "comparison": f"current_vs_{other_label}",
                    "detector": key[0],
                    "scene": key[1],
                    "threads": key[2],
                    "missing": other_label,
                    "apples_to_apples": apples_to_apples,
                    "diagnostic_reason": "; ".join(diagnostic_reasons),
                }
            )
            continue
        comparison.append(
            {
                "comparison": f"current_vs_{other_label}",
                "detector": key[0],
                "scene": key[1],
                "threads": key[2],
                "current_cpu_ms": current.cpu_ms,
                "current_cpu_median_ms": current.cpu_median_ms,
                "current_cpu_stddev_ms": current.cpu_stddev_ms,
                "current_sample_count": current.sample_count,
                f"{other_label}_cpu_ms": other.cpu_ms,
                f"{other_label}_cpu_median_ms": other.cpu_median_ms,
                f"{other_label}_cpu_stddev_ms": other.cpu_stddev_ms,
                f"{other_label}_sample_count": other.sample_count,
                "cpu_change_pct": pct_change(current.cpu_ms, other.cpu_ms),
                "cpu_median_change_pct": (
                    pct_change(current.cpu_median_ms, other.cpu_median_ms)
                    if current.cpu_median_ms is not None
                    and other.cpu_median_ms is not None
                    else None
                ),
                "current_sim_s_per_s": current.sim_s_per_s,
                f"{other_label}_sim_s_per_s": other.sim_s_per_s,
                "apples_to_apples": apples_to_apples,
                "diagnostic_reason": (
                    "; ".join(diagnostic_reasons) if diagnostic_reasons else ""
                ),
            }
        )
    return comparison


def fastest_rows(
    current_rows: dict[tuple[str, str, int], BenchmarkRow],
    detectors: list[str],
    eligible_detectors: list[str],
    expected_fastest_detector: str,
    tie_tolerance: float,
) -> tuple[list[dict[str, object]], list[str]]:
    by_scene_thread: dict[tuple[str, int], list[BenchmarkRow]] = {}
    for row in current_rows.values():
        by_scene_thread.setdefault((row.scene, row.threads), []).append(row)

    rows = []
    failures = []
    for (scene, threads), candidates in sorted(by_scene_thread.items()):
        by_detector = {row.detector: row for row in candidates}
        eligible_candidates = [
            row for row in candidates if row.detector in eligible_detectors
        ]
        if not eligible_candidates:
            failures.append(f"{scene}/{threads}: no equivalent detector rows")
            continue

        winner = min(eligible_candidates, key=lambda row: row.cpu_ms)
        reference_row = by_detector.get(REFERENCE_DETECTOR)
        expected_row = by_detector.get(expected_fastest_detector)
        detector_times = {
            detector: by_detector[detector].cpu_ms
            for detector in detectors
            if detector in by_detector
        }
        row = {
            "scene": scene,
            "threads": threads,
            "winner": winner.detector,
            "winner_cpu_ms": winner.cpu_ms,
            "reference_detector": REFERENCE_DETECTOR,
            "reference_cpu_ms": reference_row.cpu_ms if reference_row else None,
            "expected_fastest_detector": expected_fastest_detector,
            "expected_fastest_cpu_ms": (
                expected_row.cpu_ms if expected_row is not None else None
            ),
            "detectors": detector_times,
            "eligible_detectors": sorted(
                detector
                for detector in detector_times
                if detector in eligible_detectors
            ),
        }
        rows.append(row)
        if reference_row is None:
            failures.append(
                f"{scene}/{threads}: {REFERENCE_DETECTOR} reference detector " "missing"
            )
        if expected_row is None:
            failures.append(
                f"{scene}/{threads}: {expected_fastest_detector} expected "
                "fastest detector missing"
            )
        elif expected_fastest_detector not in eligible_detectors:
            failures.append(
                f"{scene}/{threads}: {expected_fastest_detector} is not "
                "checksum-equivalent to reference"
            )
        elif winner.detector != expected_fastest_detector:
            margin = (expected_row.cpu_ms - winner.cpu_ms) / winner.cpu_ms
            row["expected_fastest_margin"] = margin
            if margin > tie_tolerance:
                failures.append(
                    f"{scene}/{threads}: expected {expected_fastest_detector} "
                    f"{expected_row.cpu_ms:.3f} ms, "
                    f"winner {winner.detector} {winner.cpu_ms:.3f} ms "
                    f"(+{margin:.1%} exceeds tie tolerance {tie_tolerance:.1%})"
                )
            else:
                row["expected_fastest_within_tolerance"] = True
    return rows, failures


def write_markdown(
    output_dir: Path,
    revisions: list[Revision],
    comparisons: list[dict[str, object]],
    fastest: list[dict[str, object]],
    detector_equivalence: dict[str, dict[str, object]],
    failures: list[str],
    run_errors: list[dict[str, str]],
    benchmark_run_order: str,
    benchmark_cycles: int,
    expected_fastest_detector: str,
) -> None:
    lines = [
        "# Soft-Body Performance Comparison",
        "",
        "## Revisions",
        "",
        "| Label | Revision | Commit |",
        "| --- | --- | --- |",
    ]
    for revision in revisions:
        lines.append(f"| `{revision.label}` | `{revision.rev}` | `{revision.sha}` |")

    lines.extend(
        [
            "",
            "## Benchmark Settings",
            "",
            f"- Run order: `{benchmark_run_order}`",
            f"- Balanced cycles: `{benchmark_cycles}`",
        ]
    )

    lines.extend(
        [
            "",
            "## Current Detector Equivalence",
            "",
            "| Detector | Apples-to-apples eligible | Reason |",
            "| --- | --- | --- |",
        ]
    )
    for detector, result in sorted(detector_equivalence.items()):
        lines.append(
            "| `{detector}` | {eligible} | {reason} |".format(
                detector=detector,
                eligible="yes" if result.get("eligible") else "no",
                reason=str(result.get("reason", "")),
            )
        )

    for comparison_name in sorted({str(item["comparison"]) for item in comparisons}):
        comparison_items = [
            item for item in comparisons if item["comparison"] == comparison_name
        ]
        lines.extend(
            [
                "",
                f"## {comparison_name.replace('_', ' ')}",
                "",
            ]
        )
        write_cpu_change_graph(lines, comparison_items)
        lines.extend(
            [
                "",
                "| Detector | Scene | Threads | Scope | Other CPU ms | Current CPU ms | Mean CPU change | Median CPU change | Samples | Other sim_s/s | Current sim_s/s |",
                "| --- | --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
            ]
        )
        for item in comparison_items:
            if "missing" in item:
                continue
            other_key = "parent_cpu_ms" if "parent_cpu_ms" in item else "base_cpu_ms"
            other_sim_key = (
                "parent_sim_s_per_s"
                if "parent_sim_s_per_s" in item
                else "base_sim_s_per_s"
            )
            other_stddev_key = (
                "parent_cpu_stddev_ms"
                if "parent_cpu_stddev_ms" in item
                else "base_cpu_stddev_ms"
            )
            other_samples_key = (
                "parent_sample_count"
                if "parent_sample_count" in item
                else "base_sample_count"
            )
            median_change = item.get("cpu_median_change_pct")
            current_samples = int(item.get("current_sample_count", 1))
            other_samples = int(item.get(other_samples_key, 1))
            lines.append(
                "| `{detector}` | `{scene}` | {threads} | {scope} | {other_cpu} | "
                "{current_cpu} | {change:+.1f}% | {median_change} | "
                "{samples} | {other_sim} | {current_sim} |".format(
                    detector=item["detector"],
                    scene=item["scene"],
                    threads=item["threads"],
                    scope=(
                        "apples-to-apples"
                        if item.get("apples_to_apples", True)
                        else "diagnostic"
                    ),
                    other_cpu=fmt_cpu_with_std(
                        float(item[other_key]),
                        item.get(other_stddev_key),
                        other_samples,
                    ),
                    current_cpu=fmt_cpu_with_std(
                        float(item["current_cpu_ms"]),
                        item.get("current_cpu_stddev_ms"),
                        current_samples,
                    ),
                    change=float(item["cpu_change_pct"]),
                    median_change=(
                        f"{float(median_change):+.1f}%"
                        if median_change is not None
                        else "n/a"
                    ),
                    samples=f"{current_samples}/{other_samples}",
                    other_sim=fmt_float(item.get(other_sim_key)),
                    current_sim=fmt_float(item.get("current_sim_s_per_s")),
                )
            )

        diagnostic_items = [
            item
            for item in comparison_items
            if "missing" not in item and not item.get("apples_to_apples", True)
        ]
        if diagnostic_items:
            lines.extend(["", "Diagnostic rows:", ""])
            for item in diagnostic_items:
                lines.append(
                    "- `{detector}` `{scene}` threads `{threads}`: {reason}".format(
                        detector=item["detector"],
                        scene=item["scene"],
                        threads=item["threads"],
                        reason=item.get("diagnostic_reason") or "not apples-to-apples",
                    )
                )

    lines.extend(
        [
            "",
            "## Current Detector Winners",
            "",
        ]
    )
    write_detector_winner_graph(lines, fastest)
    tolerance_items = [
        item for item in fastest if item.get("expected_fastest_within_tolerance")
    ]
    if tolerance_items:
        lines.extend(["", "Within-tolerance matches (not strict wins):", ""])
        for item in tolerance_items:
            lines.append(
                "- `{scene}`/{threads}: expected `{expected}` trailed winner "
                "`{winner}` by {margin:.1%} (counted as a match).".format(
                    scene=item["scene"],
                    threads=item["threads"],
                    expected=item["expected_fastest_detector"],
                    winner=item["winner"],
                    margin=float(item["expected_fastest_margin"]),
                )
            )
    lines.extend(
        [
            "",
            f"| Scene | Threads | Winner | Winner CPU ms | Expected (`{expected_fastest_detector}`) CPU ms | Reference (`{REFERENCE_DETECTOR}`) CPU ms | Eligible detectors | Detector CPU ms |",
            "| --- | ---: | --- | ---: | ---: | ---: | --- | --- |",
        ]
    )
    for item in fastest:
        detector_summary = ", ".join(
            f"{detector}={cpu_ms:.3f}"
            for detector, cpu_ms in sorted(item["detectors"].items())
        )
        lines.append(
            "| `{scene}` | {threads} | `{winner}` | {winner_cpu} | "
            "{expected_cpu} | {reference_cpu} | {eligible} | {detectors} |".format(
                scene=item["scene"],
                threads=item["threads"],
                winner=item["winner"],
                winner_cpu=fmt_float(float(item["winner_cpu_ms"])),
                expected_cpu=fmt_float(item["expected_fastest_cpu_ms"]),
                reference_cpu=fmt_float(item["reference_cpu_ms"]),
                eligible=", ".join(
                    f"`{detector}`" for detector in item["eligible_detectors"]
                ),
                detectors=detector_summary,
            )
        )

    if run_errors:
        lines.extend(["", "## Run Errors", ""])
        for error in run_errors:
            lines.append(
                f"- `{error['revision']}` `{error['detector']}`: {error['message']}"
            )

    lines.extend(["", "## Evaluator Verdict", ""])
    if failures:
        lines.append("FAIL")
        for failure in failures:
            lines.append(f"- {failure}")
    else:
        lines.append("PASS")

    (output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    if args.benchmark_cycles < 1:
        raise SystemExit("--benchmark-cycles must be at least 1")

    root = repo_root()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    detectors = parse_csv(args.detectors)
    requested_threads = parse_threads(args.threads)
    revision_specs = [("current", args.current), ("parent", args.parent)]
    if args.base:
        revision_specs.append(("base", args.base))

    revisions = [
        prepare_revision(root, output_dir, label, rev, root)
        for label, rev in revision_specs
    ]

    revision_binaries: dict[str, tuple[Path, Path]] = {}
    current_headless_binary: Path | None = None
    for revision in revisions:
        binary, headless_binary = configure_and_build(revision)
        revision_binaries[revision.label] = (binary, headless_binary)
        if revision.label == "current":
            current_headless_binary = headless_binary

    all_row_samples: dict[str, dict[tuple[str, str, int], list[BenchmarkRow]]] = {
        revision.label: {} for revision in revisions
    }
    run_errors: list[dict[str, str]] = []
    unsupported_detector_runs: dict[tuple[str, str], bool] = {}

    def run_one(revision: Revision, detector: str, cycle_index: int | None) -> bool:
        binary, _ = revision_binaries[revision.label]
        if args.wait_for_local_dart_builds:
            wait_for_local_dart_idleness(
                root,
                args.idle_timeout,
                args.idle_poll_interval,
                args.idle_cooldown,
                args.idle_max_load_1m,
            )
        json_path, error = run_detector(
            revision=revision,
            binary=binary,
            detector=detector,
            output_dir=output_dir,
            min_time=args.benchmark_min_time,
            repetitions=args.benchmark_repetitions,
            cycle_index=cycle_index,
        )
        if error is not None:
            unsupported_detector_runs[(revision.label, detector)] = (
                has_unsupported_shape_warning(error)
            )
            run_errors.append(
                {
                    "revision": revision.label,
                    "detector": detector,
                    "message": error.splitlines()[-1] if error else "failed",
                }
            )
            return False

        suffix = "" if cycle_index is None else f"-cycle{cycle_index + 1}"
        log_path = output_dir / "logs" / f"{revision.label}-{detector}{suffix}.log"
        unsupported_detector_runs[
            (revision.label, detector)
        ] = unsupported_detector_runs.get((revision.label, detector), False) or (
            log_path.exists()
            and has_unsupported_shape_warning(log_path.read_text(encoding="utf-8"))
        )
        parsed_rows = load_rows(revision, detector, json_path, requested_threads)
        if not parsed_rows:
            error_message = benchmark_error_message(json_path)
            if error_message:
                run_errors.append(
                    {
                        "revision": revision.label,
                        "detector": detector,
                        "message": error_message,
                    }
                )
                return False

        for row in parsed_rows:
            all_row_samples[revision.label].setdefault(row_key(row), []).append(row)
        return True

    stop_requested = False
    for cycle in range(args.benchmark_cycles):
        cycle_index = None if args.benchmark_cycles == 1 else cycle
        cycle_revisions = list(revisions)
        if cycle % 2 == 1:
            cycle_revisions.reverse()

        if args.benchmark_run_order == "revision":
            run_sequence = [
                (revision, detector)
                for revision in cycle_revisions
                for detector in detectors
            ]
        else:
            run_sequence = [
                (revision, detector)
                for detector in detectors
                for revision in cycle_revisions
            ]

        for revision, detector in run_sequence:
            if not run_one(revision, detector, cycle_index) and not args.keep_going:
                stop_requested = True
                break
        if stop_requested:
            break

    if current_headless_binary is None:
        raise SystemExit("current soft_body_headless binary was not built")

    detector_equivalence, eligible_detectors = evaluate_detector_equivalence(
        current=revisions[0],
        headless_binary=current_headless_binary,
        detectors=detectors,
        scenes=parse_csv(args.correctness_scenes),
        steps=args.correctness_steps,
        tolerance=args.correctness_tolerance,
        output_dir=output_dir,
    )

    all_rows: dict[str, dict[tuple[str, str, int], BenchmarkRow]] = {
        label: {key: aggregate_sample_rows(rows) for key, rows in row_samples.items()}
        for label, row_samples in all_row_samples.items()
    }

    current_rows = all_rows.get("current", {})
    comparisons: list[dict[str, object]] = []
    for other_label in ("parent", "base"):
        if other_label in all_rows:
            comparisons.extend(
                build_comparison(
                    current_rows,
                    all_rows[other_label],
                    other_label,
                    detector_equivalence,
                    unsupported_detector_runs,
                )
            )

    fastest, detector_winner_failures = fastest_rows(
        current_rows,
        detectors,
        eligible_detectors,
        args.expected_fastest_detector,
        args.expected_fastest_tie_tolerance,
    )
    run_error_keys = {(error["revision"], error["detector"]) for error in run_errors}
    coverage_failures = []
    for label, rows in all_rows.items():
        expected_count = 0
        for detector in detectors:
            if (label, detector) in run_error_keys:
                continue
            expected_count += len(SCENES) * len(requested_threads)
        if len(rows) < expected_count:
            coverage_failures.append(
                f"{label} rows {len(rows)} < expected {expected_count}"
            )

    failures = [*coverage_failures, *detector_winner_failures]
    summary = {
        "revisions": [
            {
                "label": revision.label,
                "rev": revision.rev,
                "sha": revision.sha,
                "source_dir": str(revision.source_dir),
                "build_dir": str(revision.build_dir),
            }
            for revision in revisions
        ],
        "detectors": detectors,
        "threads": sorted(requested_threads),
        "benchmark_run_order": args.benchmark_run_order,
        "benchmark_cycles": args.benchmark_cycles,
        "expected_fastest_detector": args.expected_fastest_detector,
        "rows": {
            label: [
                {
                    "detector": row.detector,
                    "scene": row.scene,
                    "threads": row.threads,
                    "cpu_ms": row.cpu_ms,
                    "real_ms": row.real_ms,
                    "sim_s_per_s": row.sim_s_per_s,
                    "name": row.name,
                    "cpu_median_ms": row.cpu_median_ms,
                    "cpu_stddev_ms": row.cpu_stddev_ms,
                    "sample_count": row.sample_count,
                }
                for row in sorted(rows.values(), key=lambda row: row_key(row))
            ]
            for label, rows in all_rows.items()
        },
        "comparisons": comparisons,
        "current_detector_winners": fastest,
        "detector_equivalence": detector_equivalence,
        "eligible_detectors": eligible_detectors,
        "unsupported_detector_runs": [
            {
                "revision": revision,
                "detector": detector,
                "unsupported_shape_warning": unsupported,
            }
            for (revision, detector), unsupported in sorted(
                unsupported_detector_runs.items()
            )
        ],
        "run_errors": run_errors,
        "failures": failures,
        "verdict": "pass" if not failures else "fail",
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    write_markdown(
        output_dir,
        revisions,
        comparisons,
        fastest,
        detector_equivalence,
        failures,
        run_errors,
        args.benchmark_run_order,
        args.benchmark_cycles,
        args.expected_fastest_detector,
    )

    print(output_dir / "summary.md")
    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
