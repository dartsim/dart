#!/usr/bin/env python3
"""Capture balanced DART-vs-native soft-body benchmark pairs."""

from __future__ import annotations

import argparse
import fcntl
import json
import math
import os
import platform
import re
import shlex
import signal
import statistics
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import compare_soft_body_performance as matrix

SCHEMA_VERSION = "dart.soft_body_detector_pairs/v1"
REFERENCE_DETECTOR = "dart"
CANDIDATE_DETECTOR = "native"
SCENE_TO_INDEX = {name: index for index, name in matrix.SCENES.items()}
REQUIRED_SCENES = tuple(matrix.SCENES[index] for index in sorted(matrix.SCENES))
REQUIRED_THREADS = (1, 16)
REQUIRED_PAIRS = 20
REQUIRED_MIN_TIME = "0.5s"
REQUIRED_TOLERANCE = 0.02
REQUIRED_COOLDOWN_SECONDS = 10.0
REQUIRED_INITIAL_IDLE_SECONDS = 600.0
REQUIRED_MAX_LOAD_1M = 1.0
REQUIRED_CORRECTNESS_STEPS = "200"
REQUIRED_CORRECTNESS_TOLERANCE = 0.05
REQUIRED_THERMAL_MAX_CELSIUS = 80.0
REQUIRED_THERMAL_MAX_RISE_CELSIUS = 15.0
REQUIRED_RUN_TIMEOUT_SECONDS = 300.0
REQUIRED_PAIR_MAX_LOAD_RISE_1M = 0.5


@dataclass(frozen=True)
class PairSample:
    scene: str
    threads: int
    pair: int
    order: tuple[str, str]
    reference_cpu_ms: float
    candidate_cpu_ms: float
    reference_real_ms: float
    candidate_real_ms: float

    @property
    def cpu_ratio(self) -> float:
        return self.candidate_cpu_ms / self.reference_cpu_ms


@dataclass(frozen=True)
class ScheduledPair:
    phase: str
    scene: str
    threads: int
    pair: int
    order: tuple[str, str]


class RunnerInterrupted(RuntimeError):
    """Raised when the runner receives an external termination signal."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--revision", default="HEAD")
    parser.add_argument("--scenes", default=",".join(REQUIRED_SCENES))
    parser.add_argument("--threads", default="1,16")
    parser.add_argument("--pairs", type=int, default=REQUIRED_PAIRS)
    parser.add_argument("--warmup-pairs", type=int, default=1)
    parser.add_argument("--benchmark-min-time", default=REQUIRED_MIN_TIME)
    parser.add_argument("--tie-tolerance", type=float, default=REQUIRED_TOLERANCE)
    parser.add_argument("--cooldown", type=float, default=REQUIRED_COOLDOWN_SECONDS)
    parser.add_argument(
        "--initial-idle-seconds",
        type=float,
        default=REQUIRED_INITIAL_IDLE_SECONDS,
        help="Continuous idle interval required after the revision build.",
    )
    parser.add_argument("--idle-timeout", type=float, default=172800.0)
    parser.add_argument("--idle-poll-interval", type=float, default=10.0)
    parser.add_argument("--idle-max-load-1m", type=float, default=1.0)
    parser.add_argument(
        "--pair-max-load-rise-1m",
        type=float,
        default=REQUIRED_PAIR_MAX_LOAD_RISE_1M,
        help="Maximum 1-minute load increase across one detector pair.",
    )
    parser.add_argument(
        "--cpu-list",
        help="Optional taskset CPU list, for example 0-15.",
    )
    parser.add_argument("--correctness-steps", default="200")
    parser.add_argument("--correctness-tolerance", type=float, default=0.05)
    parser.add_argument(
        "--thermal-max-celsius",
        type=float,
        default=REQUIRED_THERMAL_MAX_CELSIUS,
    )
    parser.add_argument(
        "--thermal-max-rise-celsius",
        type=float,
        default=REQUIRED_THERMAL_MAX_RISE_CELSIUS,
    )
    parser.add_argument(
        "--run-timeout",
        type=float,
        default=REQUIRED_RUN_TIMEOUT_SECONDS,
        help="Maximum wall time for one benchmark invocation.",
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--diagnostic",
        action="store_true",
        help="Allow a reduced protocol. Diagnostic artifacts never report PASS.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the resolved schedule without creating an artifact or build.",
    )
    return parser.parse_args(argv)


def parse_scenes(value: str) -> list[str]:
    scenes = matrix.parse_csv(value)
    unknown = sorted(set(scenes) - set(REQUIRED_SCENES))
    if unknown:
        raise ValueError(f"Unknown soft-body scenes: {', '.join(unknown)}")
    if not scenes:
        raise ValueError("At least one soft-body scene is required")
    if len(set(scenes)) != len(scenes):
        raise ValueError("Soft-body scenes must not be repeated")
    return scenes


def parse_thread_list(value: str) -> list[int]:
    try:
        threads = [int(item) for item in matrix.parse_csv(value)]
    except ValueError as exc:
        raise ValueError("Threads must be a comma-separated integer list") from exc
    unsupported = sorted(set(threads) - set(REQUIRED_THREADS))
    if unsupported:
        raise ValueError(
            "BM_SoftBodyStep only registers threads 1 and 16; unsupported: "
            + ", ".join(str(item) for item in unsupported)
        )
    if not threads:
        raise ValueError("At least one thread setting is required")
    if len(set(threads)) != len(threads):
        raise ValueError("Thread settings must not be repeated")
    return threads


def build_schedule(
    pair_count: int,
    reference: str = REFERENCE_DETECTOR,
    candidate: str = CANDIDATE_DETECTOR,
) -> list[tuple[str, str]]:
    if pair_count <= 0:
        raise ValueError("Pair count must be positive")
    return [
        (reference, candidate) if index % 2 == 0 else (candidate, reference)
        for index in range(pair_count)
    ]


def build_protocol_schedule(
    scenes: list[str] | tuple[str, ...],
    threads: list[int] | tuple[int, ...],
    warmup_pairs: int,
    measured_pairs: int,
) -> list[ScheduledPair]:
    schedule = []
    for scene, thread_count in expected_row_keys(scenes, threads):
        for pair, order in enumerate(build_schedule(warmup_pairs), start=1):
            schedule.append(ScheduledPair("warmup", scene, thread_count, pair, order))
        for pair, order in enumerate(build_schedule(measured_pairs), start=1):
            schedule.append(ScheduledPair("measured", scene, thread_count, pair, order))
    return schedule


def validate_protocol(
    args: argparse.Namespace,
    scenes: list[str],
    threads: list[int],
) -> None:
    numeric_options = {
        "--tie-tolerance": args.tie_tolerance,
        "--cooldown": args.cooldown,
        "--initial-idle-seconds": args.initial_idle_seconds,
        "--idle-timeout": args.idle_timeout,
        "--idle-poll-interval": args.idle_poll_interval,
        "--idle-max-load-1m": args.idle_max_load_1m,
        "--pair-max-load-rise-1m": args.pair_max_load_rise_1m,
        "--correctness-tolerance": args.correctness_tolerance,
        "--thermal-max-celsius": args.thermal_max_celsius,
        "--thermal-max-rise-celsius": args.thermal_max_rise_celsius,
        "--run-timeout": args.run_timeout,
    }
    for option, value in numeric_options.items():
        if not math.isfinite(value):
            raise ValueError(f"{option} must be finite")
    if args.pairs <= 0 or args.pairs % 2 != 0:
        raise ValueError("--pairs must be a positive even number")
    if args.warmup_pairs < 1:
        raise ValueError("--warmup-pairs must be at least 1")
    if args.tie_tolerance < 0.0:
        raise ValueError("--tie-tolerance must be non-negative")
    if args.cooldown < 0.0:
        raise ValueError("--cooldown must be non-negative")
    if args.initial_idle_seconds < 0.0:
        raise ValueError("--initial-idle-seconds must be non-negative")
    if args.idle_timeout <= 0.0 or args.idle_poll_interval <= 0.0:
        raise ValueError("Idle timeout and poll interval must be positive")
    if args.idle_max_load_1m < 0.0:
        raise ValueError("--idle-max-load-1m must be non-negative")
    if args.pair_max_load_rise_1m < 0.0:
        raise ValueError("--pair-max-load-rise-1m must be non-negative")
    if args.thermal_max_celsius <= 0.0:
        raise ValueError("--thermal-max-celsius must be positive")
    if args.thermal_max_rise_celsius < 0.0:
        raise ValueError("--thermal-max-rise-celsius must be non-negative")
    if args.run_timeout <= 0.0:
        raise ValueError("--run-timeout must be positive")
    if args.cpu_list is not None and not re.fullmatch(r"[0-9,-]+", args.cpu_list):
        raise ValueError("--cpu-list must contain only digits, commas, and hyphens")

    if args.diagnostic:
        return

    failures = []
    if tuple(scenes) != REQUIRED_SCENES:
        failures.append(f"scenes must be {','.join(REQUIRED_SCENES)}")
    if tuple(threads) != REQUIRED_THREADS:
        failures.append("threads must be 1,16")
    if args.pairs != REQUIRED_PAIRS:
        failures.append(f"pairs must be {REQUIRED_PAIRS}")
    if args.warmup_pairs != 1:
        failures.append("warmup pairs must be 1")
    if args.benchmark_min_time != REQUIRED_MIN_TIME:
        failures.append(f"benchmark min time must be {REQUIRED_MIN_TIME}")
    if not math.isclose(args.tie_tolerance, REQUIRED_TOLERANCE):
        failures.append(f"tie tolerance must be {REQUIRED_TOLERANCE:g}")
    if args.cooldown < REQUIRED_COOLDOWN_SECONDS:
        failures.append(f"cooldown must be at least {REQUIRED_COOLDOWN_SECONDS:g}s")
    if args.initial_idle_seconds < REQUIRED_INITIAL_IDLE_SECONDS:
        failures.append(
            "initial idle interval must be at least "
            f"{REQUIRED_INITIAL_IDLE_SECONDS:g}s"
        )
    if args.idle_max_load_1m > REQUIRED_MAX_LOAD_1M:
        failures.append(f"idle 1-minute load must be at most {REQUIRED_MAX_LOAD_1M:g}")
    if args.pair_max_load_rise_1m > REQUIRED_PAIR_MAX_LOAD_RISE_1M:
        failures.append(
            "pair 1-minute load rise must be at most "
            f"{REQUIRED_PAIR_MAX_LOAD_RISE_1M:g}"
        )
    if args.correctness_steps != REQUIRED_CORRECTNESS_STEPS:
        failures.append(f"correctness steps must be {REQUIRED_CORRECTNESS_STEPS}")
    if not math.isclose(args.correctness_tolerance, REQUIRED_CORRECTNESS_TOLERANCE):
        failures.append(
            "correctness tolerance must be " f"{REQUIRED_CORRECTNESS_TOLERANCE:g}"
        )
    if args.thermal_max_celsius > REQUIRED_THERMAL_MAX_CELSIUS:
        failures.append(
            "thermal maximum must be at most " f"{REQUIRED_THERMAL_MAX_CELSIUS:g} C"
        )
    if args.thermal_max_rise_celsius > REQUIRED_THERMAL_MAX_RISE_CELSIUS:
        failures.append(
            "thermal rise must be at most " f"{REQUIRED_THERMAL_MAX_RISE_CELSIUS:g} C"
        )
    if args.run_timeout > REQUIRED_RUN_TIMEOUT_SECONDS:
        failures.append(
            f"run timeout must be at most {REQUIRED_RUN_TIMEOUT_SECONDS:g}s"
        )
    if failures:
        raise ValueError("Full protocol requirements: " + "; ".join(failures))


def expected_row_keys(
    scenes: list[str] | tuple[str, ...] = REQUIRED_SCENES,
    threads: list[int] | tuple[int, ...] = REQUIRED_THREADS,
) -> list[tuple[str, int]]:
    return [(scene, thread_count) for scene in scenes for thread_count in threads]


def benchmark_filter(scene: str, threads: int) -> str:
    return f"^BM_SoftBodyStep/{SCENE_TO_INDEX[scene]}/{threads}/200$"


def build_benchmark_command(
    binary: Path,
    json_path: Path,
    scene: str,
    threads: int,
    min_time: str,
    cpu_list: str | None,
) -> list[str]:
    command = [
        str(binary),
        f"--benchmark_filter={benchmark_filter(scene, threads)}",
        f"--benchmark_min_time={min_time}",
        "--benchmark_repetitions=1",
        "--benchmark_format=json",
        f"--benchmark_out={json_path}",
        "--benchmark_out_format=json",
    ]
    if cpu_list:
        return ["taskset", "-c", cpu_list, *command]
    return command


def _parse_label(label: str) -> dict[str, str]:
    fields = {}
    for item in label.split():
        if "=" in item:
            key, value = item.split("=", 1)
            fields[key] = value
    return fields


def load_run_cpu_time(
    path: Path,
    expected_detector: str,
    expected_scene: str,
    expected_threads: int,
    expected_executable: Path | None = None,
) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    context = data.get("context")
    if not isinstance(context, dict) or not context.get("executable"):
        raise ValueError(f"{path}: missing Google Benchmark executable context")
    executable = Path(str(context["executable"]))
    if (
        expected_executable is not None
        and executable.resolve() != expected_executable.resolve()
    ):
        raise ValueError(
            f"{path}: benchmark executable {executable}, "
            f"expected {expected_executable}"
        )
    expected_name = (
        f"BM_SoftBodyStep/{SCENE_TO_INDEX[expected_scene]}/" f"{expected_threads}/200"
    )
    rows = []
    errors = []
    for row in data.get("benchmarks", []):
        run_name = str(row.get("run_name") or row.get("name") or "")
        if re.sub(r"_(mean|median|stddev|cv)$", "", run_name) != expected_name:
            continue
        if row.get("error_occurred"):
            errors.append(str(row.get("error_message", "benchmark error")))
            continue
        if row.get("run_type") != "iteration":
            continue
        fields = _parse_label(str(row.get("label", "")))
        if fields.get("detector") != expected_detector:
            raise ValueError(
                f"{path}: detector label {fields.get('detector')!r}, "
                f"expected {expected_detector!r}"
            )
        if fields.get("scene") != expected_scene:
            raise ValueError(
                f"{path}: scene label {fields.get('scene')!r}, "
                f"expected {expected_scene!r}"
            )
        if fields.get("threads") != str(expected_threads):
            raise ValueError(
                f"{path}: threads label {fields.get('threads')!r}, "
                f"expected {expected_threads}"
            )
        unit = str(row.get("time_unit", "ns"))
        multiplier = matrix.TIME_UNIT_TO_MS.get(unit)
        if multiplier is None:
            raise ValueError(f"{path}: unsupported benchmark time unit {unit!r}")
        cpu_ms = float(row["cpu_time"]) * multiplier
        real_ms = float(row["real_time"]) * multiplier
        if not math.isfinite(cpu_ms) or cpu_ms <= 0.0:
            raise ValueError(f"{path}: CPU time must be finite and positive")
        if not math.isfinite(real_ms) or real_ms <= 0.0:
            raise ValueError(f"{path}: real time must be finite and positive")
        rows.append(
            {
                "name": expected_name,
                "label": str(row.get("label", "")),
                "cpu_ms": cpu_ms,
                "real_ms": real_ms,
                "time_unit": unit,
                "benchmark_executable": str(executable),
            }
        )

    if errors:
        raise ValueError(f"{path}: {'; '.join(sorted(errors))}")
    if len(rows) != 1:
        raise ValueError(f"{path}: expected one iteration row, found {len(rows)}")
    return rows[0]


def evaluate_pairs(
    samples: list[PairSample],
    scenes: list[str] | tuple[str, ...],
    threads: list[int] | tuple[int, ...],
    pair_count: int,
    tolerance: float,
) -> tuple[list[dict[str, Any]], list[str]]:
    grouped: dict[tuple[str, int], dict[int, PairSample]] = {}
    failures = []
    for sample in samples:
        if sample.pair < 1 or sample.pair > pair_count:
            failures.append(
                f"{sample.scene}/{sample.threads}: pair {sample.pair} is outside "
                f"1..{pair_count}"
            )
            continue
        key = (sample.scene, sample.threads)
        pairs = grouped.setdefault(key, {})
        if sample.pair in pairs:
            failures.append(
                f"{sample.scene}/{sample.threads}: duplicate pair {sample.pair}"
            )
        else:
            pairs[sample.pair] = sample

    expected_schedule = build_schedule(pair_count)
    results = []
    for scene, thread_count in expected_row_keys(scenes, threads):
        row_pairs = grouped.get((scene, thread_count), {})
        missing = [
            index for index in range(1, pair_count + 1) if index not in row_pairs
        ]
        row_failures = []
        if missing:
            row_failures.append(
                "missing pairs " + ",".join(str(index) for index in missing)
            )

        ratios = []
        reference_cpu_times = []
        candidate_cpu_times = []
        candidate_wins = 0
        reference_wins = 0
        ties = 0
        candidate_first = 0
        reference_first = 0
        for pair_index in range(1, pair_count + 1):
            sample = row_pairs.get(pair_index)
            if sample is None:
                continue
            expected_order = expected_schedule[pair_index - 1]
            if sample.order != expected_order:
                row_failures.append(
                    f"pair {pair_index} order {sample.order!r}, "
                    f"expected {expected_order!r}"
                )
            if sample.order[0] == CANDIDATE_DETECTOR:
                candidate_first += 1
            elif sample.order[0] == REFERENCE_DETECTOR:
                reference_first += 1
            if sample.reference_cpu_ms <= 0.0:
                row_failures.append(
                    f"pair {pair_index} reference CPU time is not positive"
                )
                continue
            if sample.candidate_cpu_ms <= 0.0:
                row_failures.append(
                    f"pair {pair_index} candidate CPU time is not positive"
                )
                continue
            ratio = sample.cpu_ratio
            if not math.isfinite(ratio):
                row_failures.append(f"pair {pair_index} CPU ratio is not finite")
                continue
            ratios.append(ratio)
            reference_cpu_times.append(sample.reference_cpu_ms)
            candidate_cpu_times.append(sample.candidate_cpu_ms)
            if sample.candidate_cpu_ms < sample.reference_cpu_ms:
                candidate_wins += 1
            elif sample.candidate_cpu_ms > sample.reference_cpu_ms:
                reference_wins += 1
            else:
                ties += 1

        median_ratio = statistics.median(ratios) if len(ratios) == pair_count else None
        passed = (
            not row_failures
            and median_ratio is not None
            and median_ratio <= 1.0 + tolerance
        )
        if not passed:
            if median_ratio is not None and median_ratio > 1.0 + tolerance:
                row_failures.append(
                    f"median native/dart ratio {median_ratio:.9f} exceeds "
                    f"{1.0 + tolerance:.9f}"
                )
            failures.extend(
                f"{scene}/{thread_count}: {failure}" for failure in row_failures
            )

        results.append(
            {
                "scene": scene,
                "threads": thread_count,
                "pairs": len(ratios),
                "reference_detector": REFERENCE_DETECTOR,
                "candidate_detector": CANDIDATE_DETECTOR,
                "median_reference_cpu_ms": (
                    statistics.median(reference_cpu_times)
                    if reference_cpu_times
                    else None
                ),
                "median_candidate_cpu_ms": (
                    statistics.median(candidate_cpu_times)
                    if candidate_cpu_times
                    else None
                ),
                "median_cpu_ratio": median_ratio,
                "median_cpu_delta": (
                    median_ratio - 1.0 if median_ratio is not None else None
                ),
                "min_cpu_ratio": min(ratios) if ratios else None,
                "max_cpu_ratio": max(ratios) if ratios else None,
                "candidate_wins": candidate_wins,
                "reference_wins": reference_wins,
                "ties": ties,
                "candidate_first": candidate_first,
                "reference_first": reference_first,
                "pass": passed,
                "failures": row_failures,
            }
        )

    unexpected = sorted(set(grouped) - set(expected_row_keys(scenes, threads)))
    for scene, thread_count in unexpected:
        failures.append(f"unexpected row {scene}/{thread_count}")
    return results, failures


def _read_text(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8").strip()
    except OSError:
        return None


def thermal_snapshot() -> dict[str, float]:
    temperatures = {}
    for path in sorted(Path("/sys/class/thermal").glob("thermal_zone*/temp")):
        raw = _read_text(path)
        if raw is None:
            continue
        try:
            value = float(raw) / 1000.0
        except ValueError:
            continue
        kind = _read_text(path.parent / "type") or path.parent.name
        temperatures[f"{path.parent.name}:{kind}"] = value
    for path in sorted(Path("/sys/class/hwmon").glob("hwmon*/temp*_input")):
        raw = _read_text(path)
        if raw is None:
            continue
        try:
            value = float(raw) / 1000.0
        except ValueError:
            continue
        device = _read_text(path.parent / "name") or path.parent.name
        label_path = path.with_name(path.name.replace("_input", "_label"))
        label = _read_text(label_path) or path.stem.removesuffix("_input")
        temperatures[f"{path.parent.name}:{device}:{label}"] = value
    return temperatures


def cpu_governor_snapshot() -> list[str]:
    governors = {
        value
        for path in Path("/sys/devices/system/cpu").glob(
            "cpu[0-9]*/cpufreq/scaling_governor"
        )
        if (value := _read_text(path))
    }
    return sorted(governors)


def current_affinity() -> list[int] | None:
    try:
        return sorted(os.sched_getaffinity(0))
    except AttributeError:
        return None


def cpu_model() -> str | None:
    text = _read_text(Path("/proc/cpuinfo"))
    if text is None:
        return platform.processor() or None
    for line in text.splitlines():
        if line.startswith("model name") and ":" in line:
            return line.split(":", 1)[1].strip()
    return platform.processor() or None


def environment_snapshot(root: Path) -> dict[str, Any]:
    load = os.getloadavg()
    temperatures = thermal_snapshot()
    return {
        "captured_at": utc_now(),
        "load_average": [load[0], load[1], load[2]],
        "local_dart_workloads": matrix.count_local_dart_workloads(root),
        "affinity": current_affinity(),
        "cpu_governors": cpu_governor_snapshot(),
        "thermal_available": bool(temperatures),
        "thermal_celsius": temperatures,
    }


def thermal_environment_failures(
    temperatures: dict[str, float],
    thermal_baseline: dict[str, float] | None,
    thermal_max_celsius: float,
    thermal_max_rise_celsius: float,
) -> list[str]:
    failures = []
    if thermal_baseline and not temperatures:
        failures.append("thermal sensors became unavailable")
        return failures
    if thermal_baseline:
        missing_sensors = sorted(set(thermal_baseline) - set(temperatures))
        if missing_sensors:
            failures.append("thermal sensors missing: " + ", ".join(missing_sensors))
    for sensor, temperature in temperatures.items():
        if not math.isfinite(temperature):
            failures.append(f"{sensor} is not finite")
            continue
        if temperature > thermal_max_celsius:
            failures.append(f"{sensor}={temperature:.2f} C > {thermal_max_celsius:g} C")
        if thermal_baseline and sensor in thermal_baseline:
            allowed = thermal_baseline[sensor] + thermal_max_rise_celsius
            if temperature > allowed:
                failures.append(
                    f"{sensor}={temperature:.2f} C > idle baseline "
                    f"{thermal_baseline[sensor]:.2f} C + "
                    f"{thermal_max_rise_celsius:g} C"
                )
    return failures


def idle_environment_failures(
    snapshot: dict[str, Any],
    max_load_1m: float,
    thermal_baseline: dict[str, float] | None,
    thermal_max_celsius: float,
    thermal_max_rise_celsius: float,
) -> list[str]:
    failures = []
    if snapshot["local_dart_workloads"] != 0:
        failures.append(f"local DART workloads={snapshot['local_dart_workloads']}")
    if snapshot["load_average"][0] > max_load_1m:
        failures.append(
            f"1-minute load={snapshot['load_average'][0]:.2f} > {max_load_1m:g}"
        )
    failures.extend(
        thermal_environment_failures(
            snapshot["thermal_celsius"],
            thermal_baseline,
            thermal_max_celsius,
            thermal_max_rise_celsius,
        )
    )
    return failures


def wait_for_stable_idle(
    root: Path,
    timeout: float,
    poll_interval: float,
    stable_seconds: float,
    max_load_1m: float,
    thermal_baseline: dict[str, float] | None,
    thermal_max_celsius: float,
    thermal_max_rise_celsius: float,
    history_path: Path | None = None,
    history_context: dict[str, Any] | None = None,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout
    stable_since: float | None = None
    last_report = 0.0
    while True:
        snapshot = environment_snapshot(root)
        failures = idle_environment_failures(
            snapshot,
            max_load_1m,
            thermal_baseline,
            thermal_max_celsius,
            thermal_max_rise_celsius,
        )
        if history_path is not None:
            append_jsonl(
                history_path,
                {
                    **(history_context or {}),
                    "snapshot": snapshot,
                    "failures": failures,
                },
            )
        idle = not failures
        now = time.monotonic()
        if idle:
            if stable_since is None:
                stable_since = now
            stable_for = now - stable_since
            if stable_for >= stable_seconds:
                return snapshot
        else:
            stable_since = None
            stable_for = 0.0

        if now >= deadline:
            raise TimeoutError("timed out waiting for a stable idle benchmark host")
        if now - last_report >= max(10.0, poll_interval):
            print(
                "waiting for stable benchmark host: "
                f"local_dart_workloads={snapshot['local_dart_workloads']} "
                f"load_1m={snapshot['load_average'][0]:.2f} "
                f"environment_failures={len(failures)} "
                f"stable={stable_for:.0f}/{stable_seconds:.0f}s",
                file=sys.stderr,
            )
            last_report = now
        time.sleep(poll_interval)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def append_jsonl(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(payload) + "\n")


def replace_cli_option(argv: list[str], option: str, value: str) -> list[str]:
    result = []
    index = 0
    while index < len(argv):
        item = argv[index]
        if item == option:
            index += 2
            continue
        if item.startswith(option + "="):
            index += 1
            continue
        result.append(item)
        index += 1
    return [*result, option, value]


def build_metadata(
    argv: list[str],
    revision_spec: str,
    revision_sha: str,
    dirty_paths: list[str],
    scenes: list[str],
    threads: list[int],
    args: argparse.Namespace,
) -> dict[str, Any]:
    command = [sys.executable, str(Path(__file__).resolve()), *argv]
    reproduce_argv = [item for item in argv if item != "--dry-run"]
    reproduce_argv = replace_cli_option(reproduce_argv, "--revision", revision_sha)
    reproduce_argv = replace_cli_option(reproduce_argv, "--output-dir", "OUTPUT_DIR")
    reproduce_command = [
        sys.executable,
        str(Path(__file__).resolve()),
        *reproduce_argv,
    ]
    temperatures = thermal_snapshot()
    return {
        "schema_version": SCHEMA_VERSION,
        "status": "preparing",
        "created_at": utc_now(),
        "invocation_cwd": str(Path.cwd()),
        "command": command,
        "command_shell": shlex.join(command),
        "reproduce_command": reproduce_command,
        "reproduce_command_shell": shlex.join(reproduce_command),
        "revision": {"spec": revision_spec, "sha": revision_sha},
        "source_dirty_paths": dirty_paths,
        "detectors": {
            "reference": REFERENCE_DETECTOR,
            "candidate": CANDIDATE_DETECTOR,
        },
        "protocol": {
            "scenes": scenes,
            "threads": threads,
            "pairs": args.pairs,
            "warmup_pairs": args.warmup_pairs,
            "benchmark_min_time": args.benchmark_min_time,
            "benchmark_repetitions": 1,
            "tie_tolerance": args.tie_tolerance,
            "cooldown_seconds": args.cooldown,
            "initial_idle_seconds": args.initial_idle_seconds,
            "idle_max_load_1m": args.idle_max_load_1m,
            "pair_max_load_rise_1m": args.pair_max_load_rise_1m,
            "correctness_steps": args.correctness_steps,
            "correctness_tolerance": args.correctness_tolerance,
            "thermal_max_celsius": args.thermal_max_celsius,
            "thermal_max_rise_celsius": args.thermal_max_rise_celsius,
            "thermal_required_when_available": True,
            "thermal_gate_scope": "pair-start",
            "post_run_thermal_observational": True,
            "run_timeout_seconds": args.run_timeout,
            "cpu_list": args.cpu_list,
            "diagnostic": args.diagnostic,
            "conformant": not args.diagnostic,
            "schedule": [list(order) for order in build_schedule(args.pairs)],
            "row_phase_order": [
                asdict(item)
                for item in build_protocol_schedule(
                    scenes, threads, args.warmup_pairs, args.pairs
                )
            ],
        },
        "host": {
            "platform": platform.platform(),
            "machine": platform.machine(),
            "python": platform.python_version(),
            "logical_cpus": os.cpu_count(),
            "cpu_model": cpu_model(),
            "initial_affinity": current_affinity(),
            "thermal_available": bool(temperatures),
            "initial_thermal_celsius": temperatures,
        },
    }


def _run_name(
    phase: str,
    scene: str,
    threads: int,
    pair: int,
    position: int,
    detector: str,
) -> str:
    return f"{phase}-{scene}-t{threads}-pair{pair:02d}-" f"pos{position}-{detector}"


def _terminate_process_group(process: subprocess.Popen[str]) -> str:
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        output, _ = process.communicate(timeout=5.0)
        return output or ""
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        output, _ = process.communicate()
        return output or ""


def run_captured_command(
    command: list[str],
    cwd: Path,
    env: dict[str, str],
    capture_path: Path,
    timeout: float,
) -> subprocess.CompletedProcess[str]:
    process = subprocess.Popen(
        command,
        cwd=cwd,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    try:
        output, _ = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired as exc:
        output = _terminate_process_group(process)
        capture_path.parent.mkdir(parents=True, exist_ok=True)
        capture_path.write_text(output, encoding="utf-8")
        raise subprocess.TimeoutExpired(command, timeout, output=output) from exc
    except BaseException:
        output = _terminate_process_group(process)
        capture_path.parent.mkdir(parents=True, exist_ok=True)
        capture_path.write_text(output, encoding="utf-8")
        raise

    capture_path.parent.mkdir(parents=True, exist_ok=True)
    capture_path.write_text(output, encoding="utf-8")
    result = subprocess.CompletedProcess(command, process.returncode, output)
    if process.returncode != 0:
        raise subprocess.CalledProcessError(process.returncode, command, output=output)
    return result


def acquire_benchmark_lock(root: Path, revision_sha: str):
    common_dir = Path(matrix.git(root, "rev-parse", "--git-common-dir"))
    if not common_dir.is_absolute():
        common_dir = (root / common_dir).resolve()
    lock_path = common_dir / "dart-soft-body-benchmark.lock"
    stream = lock_path.open("a+", encoding="utf-8")
    try:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        stream.close()
        raise RuntimeError(
            f"Another DART soft-body evidence runner holds {lock_path}"
        ) from None
    stream.seek(0)
    stream.truncate()
    stream.write(
        json.dumps(
            {"pid": os.getpid(), "revision": revision_sha, "acquired_at": utc_now()}
        )
        + "\n"
    )
    stream.flush()
    return stream, lock_path


def _raise_interrupted(signum: int, _frame: Any) -> None:
    raise RunnerInterrupted(f"received {signal.Signals(signum).name}")


def run_benchmark(
    root: Path,
    revision: matrix.Revision,
    binary: Path,
    output_dir: Path,
    phase: str,
    scene: str,
    threads: int,
    pair: int,
    position: int,
    detector: str,
    min_time: str,
    cpu_list: str | None,
    timeout: float,
) -> dict[str, Any]:
    name = _run_name(phase, scene, threads, pair, position, detector)
    raw_path = output_dir / "raw" / f"{name}.json"
    log_path = output_dir / "logs" / f"{name}.log"
    command = build_benchmark_command(
        binary, raw_path, scene, threads, min_time, cpu_list
    )
    env = os.environ.copy()
    env["COLLISION_DETECTOR"] = detector
    before = environment_snapshot(root)
    started_at = utc_now()
    record = {
        "phase": phase,
        "scene": scene,
        "threads": threads,
        "pair": pair,
        "position": position,
        "detector": detector,
        "command": command,
        "command_shell": shlex.join(command),
        "cwd": str(revision.source_dir),
        "environment": {"COLLISION_DETECTOR": detector},
        "raw_path": str(raw_path.relative_to(output_dir)),
        "log_path": str(log_path.relative_to(output_dir)),
        "started_at": started_at,
        "environment_before": before,
    }
    try:
        result = run_captured_command(
            command,
            cwd=revision.source_dir,
            env=env,
            capture_path=log_path,
            timeout=timeout,
        )
        if matrix.has_unsupported_shape_warning(result.stdout):
            raise RuntimeError(
                f"{name}: unsupported-shape fallback warning in benchmark log"
            )
        parsed = load_run_cpu_time(
            raw_path,
            detector,
            scene,
            threads,
            expected_executable=binary,
        )
    except BaseException as exc:
        record["status"] = "failed"
        record["error"] = f"{type(exc).__name__}: {exc}"
        record["finished_at"] = utc_now()
        record["environment_after"] = environment_snapshot(root)
        append_jsonl(output_dir / "runs.jsonl", record)
        raise

    record.update(parsed)
    record["status"] = "complete"
    record["finished_at"] = utc_now()
    record["environment_after"] = environment_snapshot(root)
    append_jsonl(output_dir / "runs.jsonl", record)
    return record


def pair_environment_failures(
    records: dict[str, dict[str, Any]],
    order: tuple[str, str],
    args: argparse.Namespace,
    thermal_baseline: dict[str, float] | None,
) -> list[str]:
    first_snapshot = records[order[0]]["environment_before"]
    failures = idle_environment_failures(
        first_snapshot,
        args.idle_max_load_1m,
        thermal_baseline,
        args.thermal_max_celsius,
        args.thermal_max_rise_celsius,
    )
    pair_load_limit = first_snapshot["load_average"][0] + args.pair_max_load_rise_1m
    for detector in order:
        for position in ("before", "after"):
            snapshot = records[detector][f"environment_{position}"]
            if snapshot["local_dart_workloads"] != 0:
                failures.append(
                    f"{detector} {position}: local DART workloads="
                    f"{snapshot['local_dart_workloads']}"
                )
            if snapshot["load_average"][0] > pair_load_limit:
                failures.append(
                    f"{detector} {position}: 1-minute load="
                    f"{snapshot['load_average'][0]:.2f} > pair start "
                    f"{first_snapshot['load_average'][0]:.2f} + "
                    f"{args.pair_max_load_rise_1m:g}"
                )
    return failures


def run_pair(
    root: Path,
    revision: matrix.Revision,
    binary: Path,
    output_dir: Path,
    phase: str,
    scene: str,
    threads: int,
    pair: int,
    order: tuple[str, str],
    args: argparse.Namespace,
    thermal_baseline: dict[str, float] | None,
) -> PairSample:
    records = {}
    for position, detector in enumerate(order, start=1):
        records[detector] = run_benchmark(
            root=root,
            revision=revision,
            binary=binary,
            output_dir=output_dir,
            phase=phase,
            scene=scene,
            threads=threads,
            pair=pair,
            position=position,
            detector=detector,
            min_time=args.benchmark_min_time,
            cpu_list=args.cpu_list,
            timeout=args.run_timeout,
        )
    environment_failures = pair_environment_failures(
        records, order, args, thermal_baseline
    )
    if environment_failures:
        raise RuntimeError(
            f"{phase} {scene}/{threads} pair {pair} environment invalid: "
            + "; ".join(environment_failures)
        )
    reference = records[REFERENCE_DETECTOR]
    candidate = records[CANDIDATE_DETECTOR]
    return PairSample(
        scene=scene,
        threads=threads,
        pair=pair,
        order=order,
        reference_cpu_ms=float(reference["cpu_ms"]),
        candidate_cpu_ms=float(candidate["cpu_ms"]),
        reference_real_ms=float(reference["real_ms"]),
        candidate_real_ms=float(candidate["real_ms"]),
    )


def evaluate_detector_equivalence_all_threads(
    revision: matrix.Revision,
    headless_binary: Path,
    scenes: list[str],
    threads: list[int],
    steps: str,
    tolerance: float,
    output_dir: Path,
) -> tuple[dict[str, dict[str, Any]], list[str]]:
    by_thread = {}
    for thread_count in threads:
        results, _eligible = matrix.evaluate_detector_equivalence(
            current=revision,
            headless_binary=headless_binary,
            detectors=[REFERENCE_DETECTOR, CANDIDATE_DETECTOR],
            scenes=scenes,
            steps=steps,
            tolerance=tolerance,
            output_dir=output_dir,
            thread_count=thread_count,
        )
        by_thread[thread_count] = results

    aggregate = {}
    eligible = []
    for detector in (REFERENCE_DETECTOR, CANDIDATE_DETECTOR):
        detector_by_thread = {
            str(thread_count): by_thread[thread_count].get(
                detector,
                {
                    "eligible": False,
                    "reason": "detector result missing",
                    "threads": thread_count,
                },
            )
            for thread_count in threads
        }
        detector_ok = all(
            result.get("eligible") for result in detector_by_thread.values()
        )
        reasons = [
            f"threads {thread_count}: {result.get('reason', 'failed')}"
            for thread_count, result in detector_by_thread.items()
            if not result.get("eligible")
        ]
        if detector == REFERENCE_DETECTOR and detector_ok:
            reason = "reference detector"
        elif detector_ok:
            reason = "checksum-equivalent for every scene/thread row"
        else:
            reason = "; ".join(reasons)
        aggregate[detector] = {
            "eligible": detector_ok,
            "reason": reason,
            "scenes": scenes,
            "threads": threads,
            "by_thread": detector_by_thread,
        }
        if detector_ok:
            eligible.append(detector)
    return aggregate, eligible


def write_markdown(
    output_dir: Path,
    metadata: dict[str, Any],
    summary: dict[str, Any],
) -> None:
    protocol = metadata["protocol"]
    lines = [
        "# Balanced soft-body detector pairs",
        "",
        f"- Revision: `{metadata['revision']['sha']}`",
        "- Authoritative completion marker: `COMPLETE.json`",
        f"- Invocation directory: `{metadata['invocation_cwd']}`",
        f"- Reference detector: `{REFERENCE_DETECTOR}`",
        f"- Candidate detector: `{CANDIDATE_DETECTOR}`",
        f"- Pairs per row: `{protocol['pairs']}`",
        f"- CPU match tolerance: `{protocol['tie_tolerance']:.1%}`",
        f"- Thermal sensors available: `{'yes' if metadata['host']['thermal_available'] else 'no'}`",
        f"- Thermal start ceiling/rise: `{protocol['thermal_max_celsius']:g} C` / `{protocol['thermal_max_rise_celsius']:g} C`",
        f"- Diagnostic protocol: `{'yes' if protocol['diagnostic'] else 'no'}`",
        "",
        "## Original invocation",
        "",
        "```bash",
        metadata["command_shell"],
        "```",
        "",
        "## Reproduce with a new empty output directory",
        "",
        "```bash",
        metadata["reproduce_command_shell"],
        "```",
        "",
        "## Detector equivalence",
        "",
        "| Detector | Eligible | Reason |",
        "| --- | --- | --- |",
    ]
    for detector, result in sorted(summary["detector_equivalence"].items()):
        lines.append(
            f"| `{detector}` | {'yes' if result.get('eligible') else 'no'} | "
            f"{result.get('reason', '')} |"
        )

    lines.extend(
        [
            "",
            "## Paired CPU verdict",
            "",
            "| Scene | Threads | Pairs | First D/N | Median DART ms | Median native ms | Median native/dart | Delta | Wins N/D | Verdict |",
            "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
        ]
    )
    for row in summary["rows"]:
        ratio = row["median_cpu_ratio"]
        delta = row["median_cpu_delta"]
        reference_ms = row["median_reference_cpu_ms"]
        candidate_ms = row["median_candidate_cpu_ms"]
        lines.append(
            "| `{scene}` | {threads} | {pairs} | {reference_first}/{candidate_first} | "
            "{reference_ms} | {candidate_ms} | {ratio} | {delta} | "
            "{candidate_wins}/{reference_wins} | {verdict} |".format(
                scene=row["scene"],
                threads=row["threads"],
                pairs=row["pairs"],
                reference_first=row["reference_first"],
                candidate_first=row["candidate_first"],
                reference_ms=(
                    f"{reference_ms:.6f}" if reference_ms is not None else "n/a"
                ),
                candidate_ms=(
                    f"{candidate_ms:.6f}" if candidate_ms is not None else "n/a"
                ),
                ratio=f"{ratio:.6f}" if ratio is not None else "n/a",
                delta=f"{delta:+.2%}" if delta is not None else "n/a",
                candidate_wins=row["candidate_wins"],
                reference_wins=row["reference_wins"],
                verdict="PASS" if row["pass"] else "FAIL",
            )
        )

    lines.extend(["", "## Evaluator verdict", "", summary["verdict"]])
    for failure in summary["failures"]:
        lines.append(f"- {failure}")
    (output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def finalize_artifact(
    output_dir: Path,
    manifest_path: Path,
    metadata: dict[str, Any],
    summary: dict[str, Any],
) -> None:
    metadata["status"] = "complete"
    metadata["verdict"] = summary["verdict"]
    metadata["finished_at"] = utc_now()
    write_json(manifest_path, metadata)
    write_markdown(output_dir, metadata, summary)
    write_json(output_dir / "summary.json", summary)
    write_json(
        output_dir / "COMPLETE.json",
        {
            "schema_version": SCHEMA_VERSION,
            "status": "complete",
            "verdict": summary["verdict"],
            "revision_sha": metadata["revision"]["sha"],
            "finished_at": metadata["finished_at"],
        },
    )


def remove_completion_files(output_dir: Path) -> None:
    for name in ("COMPLETE.json", "summary.json", "summary.md"):
        try:
            (output_dir / name).unlink()
        except OSError:
            pass


def _ensure_empty_output(output_dir: Path) -> None:
    if output_dir.exists() and any(output_dir.iterdir()):
        raise ValueError(f"Output directory is not empty: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        scenes = parse_scenes(args.scenes)
        threads = parse_thread_list(args.threads)
        validate_protocol(args, scenes, threads)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    root = matrix.repo_root()
    revision_sha = matrix.git(root, "rev-parse", args.revision)
    dirty_paths = [
        line for line in matrix.git(root, "status", "--porcelain").splitlines() if line
    ]
    metadata = build_metadata(
        argv, args.revision, revision_sha, dirty_paths, scenes, threads, args
    )

    if args.dry_run:
        plan = {
            "metadata": metadata,
            "row_keys": [list(key) for key in expected_row_keys(scenes, threads)],
        }
        print(json.dumps(plan, indent=2))
        return 0

    root_head = matrix.git(root, "rev-parse", "HEAD")
    if not args.diagnostic and dirty_paths:
        raise SystemExit(
            "The full paired protocol requires a clean source checkout; "
            "use --diagnostic for an explicitly non-passing smoke artifact."
        )
    if not args.diagnostic and root_head != revision_sha:
        raise SystemExit(
            "The full paired protocol requires --revision to resolve to the "
            "current HEAD so the runner and benchmark harness are pinned together."
        )

    output_dir = args.output_dir.resolve()
    try:
        _ensure_empty_output(output_dir)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    manifest_path = output_dir / "metadata.json"
    write_json(manifest_path, metadata)

    lock_stream = None
    previous_signal_handlers = {}
    try:
        for handled_signal in (signal.SIGTERM, signal.SIGHUP):
            previous_signal_handlers[handled_signal] = signal.getsignal(handled_signal)
            signal.signal(handled_signal, _raise_interrupted)

        lock_stream, lock_path = acquire_benchmark_lock(root, revision_sha)
        metadata["exclusive_lock"] = str(lock_path)
        revision = matrix.prepare_revision(
            root, output_dir, "current", revision_sha, None
        )
        if revision.sha != revision_sha:
            raise RuntimeError(
                f"Prepared revision {revision.sha}, expected {revision_sha}"
            )
        metadata["revision"] = {
            "requested_spec": args.revision,
            "sha": revision.sha,
            "source_dir": str(revision.source_dir),
            "build_dir": str(revision.build_dir),
        }
        metadata["status"] = "building"
        write_json(manifest_path, metadata)
        binary, headless_binary = matrix.configure_and_build(revision)
        metadata["benchmark_binary"] = str(binary)
        metadata["headless_binary"] = str(headless_binary)

        metadata["status"] = "checking-detector-equivalence"
        write_json(manifest_path, metadata)
        equivalence, eligible = evaluate_detector_equivalence_all_threads(
            revision=revision,
            headless_binary=headless_binary,
            scenes=scenes,
            threads=threads,
            steps=args.correctness_steps,
            tolerance=args.correctness_tolerance,
            output_dir=output_dir,
        )
        ineligible = [
            detector
            for detector in (REFERENCE_DETECTOR, CANDIDATE_DETECTOR)
            if detector not in eligible
        ]
        if ineligible:
            raise RuntimeError(
                "Detector checksum equivalence failed: " + ", ".join(ineligible)
            )

        metadata["detector_equivalence"] = equivalence
        metadata["status"] = "waiting-for-stable-idle"
        metadata["idle_history_path"] = "idle_checks.jsonl"
        write_json(manifest_path, metadata)
        metadata["stable_idle_snapshot"] = wait_for_stable_idle(
            root=root,
            timeout=args.idle_timeout,
            poll_interval=args.idle_poll_interval,
            stable_seconds=args.initial_idle_seconds,
            max_load_1m=args.idle_max_load_1m,
            thermal_baseline=None,
            thermal_max_celsius=args.thermal_max_celsius,
            thermal_max_rise_celsius=args.thermal_max_rise_celsius,
            history_path=output_dir / metadata["idle_history_path"],
            history_context={"phase": "preflight"},
        )
        thermal_baseline = metadata["stable_idle_snapshot"]["thermal_celsius"]
        metadata["thermal_baseline_celsius"] = thermal_baseline
        metadata["host"]["thermal_available_at_stable_idle"] = bool(thermal_baseline)
        write_json(manifest_path, metadata)

        samples = []
        current_phase_row = None
        for scheduled in build_protocol_schedule(
            scenes, threads, args.warmup_pairs, args.pairs
        ):
            phase_row = (scheduled.phase, scheduled.scene, scheduled.threads)
            if phase_row != current_phase_row:
                metadata["status"] = (
                    "warming-up" if scheduled.phase == "warmup" else "measuring"
                )
                metadata["current_row"] = {
                    "scene": scheduled.scene,
                    "threads": scheduled.threads,
                    "phase": scheduled.phase,
                }
                write_json(manifest_path, metadata)
                current_phase_row = phase_row

            wait_for_stable_idle(
                root=root,
                timeout=args.idle_timeout,
                poll_interval=args.idle_poll_interval,
                stable_seconds=0.0,
                max_load_1m=args.idle_max_load_1m,
                thermal_baseline=thermal_baseline or None,
                thermal_max_celsius=args.thermal_max_celsius,
                thermal_max_rise_celsius=args.thermal_max_rise_celsius,
                history_path=output_dir / metadata["idle_history_path"],
                history_context={
                    "phase": scheduled.phase,
                    "scene": scheduled.scene,
                    "threads": scheduled.threads,
                    "pair": scheduled.pair,
                },
            )
            sample = run_pair(
                root=root,
                revision=revision,
                binary=binary,
                output_dir=output_dir,
                phase=scheduled.phase,
                scene=scheduled.scene,
                threads=scheduled.threads,
                pair=scheduled.pair,
                order=scheduled.order,
                args=args,
                thermal_baseline=thermal_baseline or None,
            )
            if scheduled.phase == "measured":
                samples.append(sample)
                pair_record = asdict(sample)
                pair_record["cpu_ratio"] = sample.cpu_ratio
                pair_record["cpu_delta"] = sample.cpu_ratio - 1.0
                pair_record["environment_validated"] = True
                append_jsonl(output_dir / "pairs.jsonl", pair_record)
            if args.cooldown:
                time.sleep(args.cooldown)

        rows, failures = evaluate_pairs(
            samples, scenes, threads, args.pairs, args.tie_tolerance
        )
        if args.diagnostic:
            verdict = "DIAGNOSTIC"
        else:
            verdict = "PASS" if not failures else "FAIL"
        summary = {
            "schema_version": SCHEMA_VERSION,
            "revision": metadata["revision"],
            "detector_equivalence": equivalence,
            "rows": rows,
            "failures": failures,
            "verdict": verdict,
            "completion_marker_required": "COMPLETE.json",
        }
        metadata["final_environment"] = environment_snapshot(root)
        for handled_signal, previous in previous_signal_handlers.items():
            signal.signal(handled_signal, previous)
        previous_signal_handlers.clear()
        lock_stream.close()
        lock_stream = None
        finalize_artifact(output_dir, manifest_path, metadata, summary)
        return 0 if verdict in {"PASS", "DIAGNOSTIC"} else 1
    except BaseException as exc:
        remove_completion_files(output_dir)
        metadata["status"] = (
            "interrupted"
            if isinstance(exc, (KeyboardInterrupt, RunnerInterrupted))
            else "failed"
        )
        metadata["error"] = f"{type(exc).__name__}: {exc}"
        metadata["finished_at"] = utc_now()
        write_json(manifest_path, metadata)
        raise
    finally:
        for handled_signal, previous in previous_signal_handlers.items():
            signal.signal(handled_signal, previous)
        if lock_stream is not None:
            lock_stream.close()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
