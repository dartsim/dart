#!/usr/bin/env python3
"""Measure deterministic DART performance counts and compare revision records.

``run`` measures an installed arm; ``compare`` judges saved records; ``local``
prepares revisions and does both. ``backfill`` resumes a local revision history;
``ledger`` reports its comparison policy. ``publish`` writes trusted records to
gh-pages, including release records. Wall time and RSS are advisory. Measurement
requires Linux, the system Valgrind, and the active Pixi build environment.
"""

from __future__ import annotations

import argparse
import contextlib
import fcntl
import gzip
import hashlib
import io
import json
import math
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import tarfile
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VALGRIND = "/usr/bin/valgrind"
WORLD_SHA = "ad94d44b90f3023765e1b2a4d2ecc7390f5539fa761d6019e08b388ce5c5ff33"
MEASURED_PATHS = (
    "dart",
    "examples/contact_benchmark",
    "tests/benchmark",
    "tests/unit/lcpsolver/DantzigProblemCases.hpp",
    "data",
    "CMakeLists.txt",
    "cmake",
)
RELEASE_TAG = re.compile(r"v6\.\d+\.\d+")
LOCAL_PATH = re.compile(r"""file:///[^\s'"]+|(?<![\w.~:/-])/[^\s/'"]+/""")
# Only these comparison-policy failures can be acknowledged by a rationale.
WAIVABLE_FAILURES = (
    ("ir", "Perf-Regression-Rationale", r"[^:]+: Ir \+\d+\.\d+% \(limit \+1\.00%\)"),
    (
        "geomean",
        "Perf-Regression-Rationale",
        r"Ir geomean \+\d+\.\d+% \(limit \+0\.50%\); rationale required for .+",
    ),
    ("allocs", "Perf-Regression-Rationale", r"[^:]+: allocations \+\S+/step"),
    ("bytes", "Perf-Regression-Rationale", r"[^:]+: requested bytes \+\S+/step"),
    (
        "guards",
        "Rebaseline-Rationale",
        r"[^:]+: guards changed; Rebaseline-Rationale required",
    ),
    (
        "input",
        "Rebaseline-Rationale",
        r"[^:]+: input_sha changed; Rebaseline-Rationale required",
    ),
    (
        "percent",
        "Rebaseline-Rationale",
        r"[^:]+: Rebaseline-Rationale must state a signed percentage \(measured Ir \+\d+\.\d+%\)",
    ),
)
PERTURBATIONS = (
    "start4k",
    "start100k",
    "size16",
    "size48",
    "random1",
    "random2",
    "tcache0",
)


@dataclass(frozen=True)
class Row:
    row: str
    det: str
    driver: str
    args: tuple[str, ...]
    warmup: int
    steps: int
    ir: bool = True
    threads: int = 1
    parity: str = ""
    version: int = 1
    perturb: bool = True
    checkpoint: int = 0

    @property
    def key(self) -> str:
        return f"{self.row}/{self.det}" if self.det else self.row


CB = "contact_benchmark"
PB = "portable_step_bench"
# Workload files from the CMake targets, their build definitions and their
# local includes. DART library sources are measured code, not inputs. PB uses
# this checkout for both arms.
WORKLOAD_SOURCES = {
    PB: (
        "tools/perf/portable_step_bench.cpp",
        "tools/perf/CMakeLists.txt",
    ),
    CB: (
        "examples/contact_benchmark/main.cpp",
        "examples/contact_benchmark/ContactContainerScene.hpp",
        "examples/contact_benchmark/GazeboPreset.hpp",
        "examples/contact_benchmark/CMakeLists.txt",
    ),
    "BM_INTEGRATION_kinematics": (
        "tests/benchmark/integration/bm_kinematics.cpp",
        "tests/benchmark/PerfGuard.hpp",
        "tests/benchmark/CMakeLists.txt",
        "tests/benchmark/integration/CMakeLists.txt",
    ),
    "BM_UNIT_dantzig_lcp": (
        "tests/benchmark/unit/bm_dantzig_lcp.cpp",
        "tests/benchmark/PerfGuard.hpp",
        "tests/unit/lcpsolver/DantzigProblemCases.hpp",
        "tests/benchmark/CMakeLists.txt",
        "tests/benchmark/unit/CMakeLists.txt",
    ),
}
WORKLOAD_DATA = {
    "pend": ("data/sdf/benchmark.world",),
    "robot": (
        "data/sdf/atlas/ground.urdf",
        "data/sdf/atlas/atlas_v3_no_head.sdf",
    ),
}
BUILD_ERROR_MARKERS = (
    ": error:",
    ": fatal error:",
    "undefined reference",
    "cmake error",
)
RUNNER_ERROR_MARKERS = (
    "no space left on device",
    "killed signal terminated program",
    "virtual memory exhausted",
    "cannot allocate memory",
    "signal 9",
)
# Exact micro wrappers exclude timing/report formatting from the slope.
COLLECTION_SIGNATURES = {
    CB: "dart::simulation::World::step(bool)",
    PB: "stepAndRead(dart::simulation::World*)",
    "BM_INTEGRATION_kinematics": "BM_Dynamics(benchmark::State&)",
    "BM_UNIT_dantzig_lcp": "(anonymous namespace)::solveNative(benchmark::State&, int)",
}
CAP = "--max-contacts 20000 --max-contacts-per-pair 4 --disable-deactivation".split()
WORLD_ARGS = (
    "{world} --sdf-plane-shapes --max-contacts 12000 --max-contacts-per-pair 4".split()
)
SCENES = {
    "s3w": ((*WORLD_ARGS, "--disable-deactivation"), 5, 5, ("dart", "ode")),
    "s2r": (tuple(WORLD_ARGS), 1000, 200, ("dart", "ode")),
    "s1p": (("--generate-container", "60", *CAP), 100, 50, ("dart", "ode")),
    "s5a": (
        ("--generate-objects", "90", *CAP),
        100,
        100,
        ("dart", "fcl", "bullet", "ode"),
    ),
    "pend": (("{data}/sdf/benchmark.world",), 100, 1000, ("dart",)),
}
ROWS = [
    Row(name, det, CB, args, w, n, ir=(name, det) != ("s2r", "ode"))
    for name, (args, w, n, detectors) in SCENES.items()
    for det in detectors
]
GZ_ARGS = tuple("{world} --ground gzbox --max-contacts 10000".split())
# BM_Dynamics resets positions and velocities and draws no random values (only
# BM_Kinematics does), so its checksum is the same in every process.
DYN_ARGS = ("--benchmark_filter=BM_Dynamics/10$",)
LCP_ARGS = ("--benchmark_filter=solveNative/(boxed_coupled_96|friction_32)$",)
MICRO_CASES = {
    "dyn": ["BM_Dynamics/10"],
    "lcp": ["solveNative/boxed_coupled_96", "solveNative/friction_32"],
}
ROWS += [
    Row("gzb", "ode", PB, GZ_ARGS, 2, 3),
    Row("robot", "dart", PB, ("--robot", "atlas"), 300, 100),
    Row("dyn", "", "BM_INTEGRATION_kinematics", DYN_ARGS, 20, 20),
    Row("lcp", "", "BM_UNIT_dantzig_lcp", LCP_ARGS, 100, 100),
]
ROWS += [
    Row(
        f"mt4-{name}",
        "dart",
        CB,
        SCENES[name][0],
        SCENES[name][1],
        SCENES[name][2],
        ir=False,
        threads=4,
        parity=f"{name}/dart",
    )
    for name in ("s3w", "s1p")
]

# Canonical guard windows from the generalization baseline, measured natively.
# S3/S4 retain their historical 16-thread cells and add serial/4-thread cells.
DETECTORS = ("dart", "fcl", "bullet", "ode")
NIGHTLY_ROWS = [
    Row(
        f"S1-{objects}-t{threads}",
        det,
        CB,
        ("--generate-container", str(objects), *CAP),
        0,
        200,
        ir=False,
        threads=threads,
        perturb=False,
    )
    for objects in (60, 120)
    for det in ("dart", "ode")
    for threads in (1, 16)
]
NIGHTLY_ROWS += [
    Row("S2", det, CB, tuple(WORLD_ARGS), 0, 3000, ir=False, perturb=False)
    for det in DETECTORS
]
NIGHTLY_ROWS += [
    Row(
        f"{scene}-t{threads}",
        det,
        CB,
        args,
        0,
        300,
        ir=False,
        threads=threads,
        perturb=False,
    )
    for scene, args in (
        ("S3", (*WORLD_ARGS, "--disable-deactivation")),
        (
            "S4",
            (
                "--generate-objects",
                "900",
                "--max-contacts",
                "20000",
                "--max-contacts-per-pair",
                "4",
            ),
        ),
    )
    for det in DETECTORS
    for threads in (1, 4, 16)
]
NIGHTLY_ROWS += [
    Row(
        "S5",
        det,
        CB,
        (
            "--generate-objects",
            "90",
            "--max-contacts",
            "20000",
            "--max-contacts-per-pair",
            "4",
        ),
        0,
        300,
        ir=False,
        perturb=False,
    )
    for det in DETECTORS
]
NIGHTLY_ROWS += [
    Row(
        "S6",
        "dart",
        CB,
        ("--generate-container", "71"),
        0,
        20000,
        ir=False,
        perturb=False,
        checkpoint=5000,
    ),
    Row(
        "mf",
        "dart",
        CB,
        ("--generate-container", "120", "--matrix-free-contact-lcp", *CAP),
        50,
        50,
    ),
]


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def command_output(command: list[str]) -> str:
    return subprocess.check_output(command, cwd=ROOT, text=True).strip()


def field(text: str, name: str) -> str:
    match = re.search(rf"^{re.escape(name)}:\s*(.+)$", text, re.MULTILINE)
    if not match:
        raise ValueError(f"missing {name}")
    return match[1].strip()


def failed_guards(text: str) -> bool:
    """True when complete guards describe a correctness failure."""
    try:
        finite = field(text, "Final State Finite")
        return bool(guards(text)) and (
            finite == "false"
            or finite == "true"
            and field(text, "Time Advanced") == "false"
        )
    except ValueError:
        return False  # incomplete guards: the run itself failed


def guards(text: str) -> dict:
    result = {
        "hash": field(text, "Final State Hash"),
        "finite": field(text, "Final State Finite") == "true",
        "contacts": int(field(text, "Final Contacts")),
        "cap_hit": field(text, "Final Contact Cap Hit") == "true",
        "resting": field(text, "Final Resting").split(" mobile")[0].replace(" ", ""),
    }
    if re.search(r"^Final Contact Pairs:", text, re.MULTILINE):
        result["pairs"] = int(field(text, "Final Contact Pairs"))
    return result


def micro_guards(text: str) -> dict | None:
    if not re.search(r"^PERFGUARD\b", text, re.MULTILINE):
        return None
    samples = re.findall(
        r"^PERFGUARD case=(\S+) hash=(0x[0-9a-f]{16}) finite=(true|false)$",
        text,
        re.MULTILINE,
    )
    hashes = {name.split("/iterations:")[0]: value for name, value, _ in samples}
    if not samples or len(hashes) != len(samples):
        raise ValueError("missing or duplicate micro checksum")
    return {
        "hash": hashes,
        "finite": all(finite == "true" for _, _, finite in samples),
    }


def guard_evidence(metrics: dict) -> tuple:
    # Native penetration and canonical checkpoints live outside the guard map.
    return tuple(
        metrics.get(key) for key in ("guards", "max_penetration", "checkpoints")
    )


def identity(path: str, prefix: Path) -> None:
    if not Path(path).resolve().is_relative_to(prefix.resolve()):
        raise ValueError(
            f"arm identity: {Path(path).name} is outside the install prefix"
        )


def environment(prefix: Path) -> dict[str, str]:
    env = os.environ.copy()
    for key in ("LD_PRELOAD", "HEAPPAD", "PERF_WARMUP", "PERF_WINDOW", "PERF_MICRO"):
        env.pop(key, None)
    env.update(LC_ALL="C", GLIBC_TUNABLES="glibc.cpu.hwcaps=-FMA")
    # Only the arm and the active Pixi environment supply libraries; an inherited
    # path could load other dependencies that the fingerprint does not record.
    paths = [str(prefix / "lib")]
    if env.get("CONDA_PREFIX"):
        paths.append(str(Path(env["CONDA_PREFIX"]) / "lib"))
    env["LD_LIBRARY_PATH"] = ":".join(paths)
    return env


class UnsupportedRow(ValueError):
    pass


class UnknownOption(ValueError):
    def __init__(self, flag: str, message: str):
        super().__init__(message)
        self.flag = flag


class BuildFailure(ValueError):
    """A configure/build/install log identifies a source or configuration error."""


# Benchmarks run in their own sessions, so a terminal's Ctrl-C does not reach
# them; whoever catches the interruption stops every running group.
RUNNING: dict[int, subprocess.Popen] = {}
RUNNING_LOCK = threading.Lock()


def kill_group(process: subprocess.Popen) -> None:
    with contextlib.suppress(ProcessLookupError):  # it has already exited
        os.killpg(process.pid, signal.SIGKILL)
    process.wait()


def kill_running() -> None:
    with RUNNING_LOCK:
        running = list(RUNNING.values())
    for process in running:
        kill_group(process)


def execute(
    command: list[str], env: dict, log: Path, timeout: int, *, build: bool = False
) -> str:
    with log.open("w", encoding="utf-8") as output:
        process = subprocess.Popen(
            command,
            cwd=ROOT,
            env=env,
            stdout=output,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        with RUNNING_LOCK:
            RUNNING[id(process)] = process
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired as error:
            kill_group(process)
            raise ValueError(f"timeout: see {log.name}") from error
        except BaseException:
            kill_group(process)
            raise
        finally:
            with RUNNING_LOCK:
                RUNNING.pop(id(process), None)
    text = log.read_text(encoding="utf-8", errors="replace")
    if build and process.returncode:
        lower = text.lower()
        source_error = (
            process.returncode > 0
            and any(marker in lower for marker in BUILD_ERROR_MARKERS)
            and not any(marker in lower for marker in RUNNER_ERROR_MARKERS)
        )
        error = BuildFailure if source_error else ValueError
        raise error(f"exit {process.returncode}: see {log.name}")
    unsupported = re.search(r"^UNSUPPORTED: (.+)$", text, re.MULTILINE)
    if process.returncode == 3 and unsupported:
        raise UnsupportedRow(unsupported[1])
    # Both drivers print complete guards before exiting nonzero on a non-finite
    # state or failed time/frame advancement. These are
    # measured correctness failures, not infrastructure ones.
    if process.returncode and failed_guards(text):
        return text
    unknown = re.search(r"^Unknown option: (\S+)$", text, re.MULTILINE)
    if process.returncode == 1 and unknown:
        raise UnknownOption(unknown[1], f"exit 1: see {log.name}")
    if process.returncode:
        raise ValueError(f"exit {process.returncode}: see {log.name}")
    return text


def row_command(row: Row, args, world: Path, warmup: int, steps: int) -> list[str]:
    command = [
        str(args.bin_dir / row.driver),
        *(
            (
                str(world)
                if item == "{world}"
                else item.replace("{data}", str(args.source_dir / "data"))
            )
            for item in row.args
        ),
    ]
    if row.row == "robot":
        command += ["--data-dir", str(args.source_dir / "data")]
    if not row.det:
        return [*command, f"--benchmark_min_time={steps}x"]
    command += [
        "--warmup",
        str(warmup),
        "--steps",
        str(steps),
        "--world-threads",
        str(row.threads),
    ]
    if row.driver == "contact_benchmark":
        command += ["--collision", row.det, "--checkpoint", str(row.checkpoint)]
        if not row.checkpoint:
            command += ["--quiet"]
    else:
        command += ["--detector", row.det]
    return command


def perturb_environment(env: dict, args, config: str) -> dict:
    env.update(LD_PRELOAD=str(args.shim))
    if config == "tcache0":
        env["GLIBC_TUNABLES"] += ":glibc.malloc.tcache_count=0"
    elif config:
        env.update(LD_PRELOAD=f"{args.shim}:{args.heappad}", HEAPPAD=config)
    return env


def native(row: Row, args, world: Path, config: str = "") -> dict:
    env = perturb_environment(environment(args.prefix), args, config)
    env["PERF_WARMUP"] = str(row.warmup)
    if row.driver == PB:
        env["PERF_WINDOW"] = "stepAndRead"
    tag = row.key.replace("/", ".") + ".native" + (f".{config}" if config else "")
    try:
        text = execute(
            [
                "/usr/bin/time",
                "-f",
                "PERFTIME maxrss_kb=%M",
                *row_command(row, args, world, row.warmup, row.steps),
            ],
            env,
            args.output_dir / f"{tag}.log",
            args.timeout,
        )
    except UnknownOption as error:
        if error.flag in row.args:
            raise UnsupportedRow(f"{row.driver} lacks {error.flag}") from error
        raise
    match = re.search(
        r"^STEPALLOC steps=(\d+) measured=(\d+) allocs=(\d+) bytes=(\d+) libdart=(.+)$",
        text,
        re.MULTILINE,
    )
    if not match:
        raise ValueError(f"missing STEPALLOC: {tag}")
    steps, measured, allocs, size = map(int, match.groups()[:4])
    if (steps, measured) != (row.warmup + row.steps, row.steps):
        raise ValueError(
            f"interposition counted {steps}/{measured} steps, expected {row.warmup + row.steps}/{row.steps}"
        )
    identity(match[5], args.prefix)
    rss = re.search(r"^PERFTIME maxrss_kb=(\d+)$", text, re.MULTILINE)
    if not rss:
        raise ValueError(f"missing RSS: {tag}")
    metrics = {
        "guards": guards(text),
        "time_advanced": not bool(
            re.search(r"^Time Advanced:\s*false\s*$", text, re.MULTILINE)
        ),
        "allocs_per_step": allocs / measured,
        "bytes_per_step": size / measured,
        "allocs": allocs,
        "bytes": size,
        "libdart": Path(match[5]).name,
        "max_rss_kb": int(rss[1]),
        "wall_ms_per_step": float(field(text, "Avg Step Time").split()[0]),
    }
    if re.search(r"^Final Max Penetration:", text, re.MULTILINE):
        metrics["max_penetration"] = finite_or_none(
            float(field(text, "Final Max Penetration"))
        )
        # A finite final state can still report a non-finite penetration.
        if metrics["max_penetration"] is None:
            raise BenchmarkCaseError(f"non-finite final penetration: {row.key}")
    if row.checkpoint:
        metrics["checkpoints"] = [
            {
                "step": int(step),
                "max_penetration": finite_or_none(float(pen)),
                "resting": int(resting),
            }
            for step, pen, resting in re.findall(
                r"^step (\d+) .*? max_penetration (\S+) .*? resting (\d+)\b",
                text,
                re.MULTILINE,
            )
        ]
        expected = list(range(row.checkpoint, row.steps + 1, row.checkpoint))
        if [item["step"] for item in metrics["checkpoints"]] != expected:
            raise ValueError(f"missing canonical checkpoints: {row.key}")
        # The final finite flag covers only the last step.
        if any(item["max_penetration"] is None for item in metrics["checkpoints"]):
            raise BenchmarkCaseError(f"non-finite checkpoint penetration: {row.key}")
    return metrics


def callgrind(row: Row, args, world: Path, steps: int) -> dict[str, int]:
    tag = row.key.replace("/", ".") + f".{steps}"
    for previous in args.output_dir.glob(f"{tag}.*.cg"):
        previous.unlink()
    command = [
        VALGRIND,
        "--tool=callgrind",
        "--trace-children=yes",
        f"--callgrind-out-file={args.output_dir}/{tag}.%p.cg",
    ]
    command += [
        "--collect-atstart=no",
        f"--toggle-collect={COLLECTION_SIGNATURES[row.driver]}",
    ]
    if args.cache_sim:
        command += [
            "--cache-sim=yes",
            "--I1=32768,8,64",
            "--D1=32768,8,64",
            "--LL=16777216,16,64",
        ]
    env = environment(args.prefix)
    if not row.det:
        env["PERF_MICRO"] = "1"
    text = execute(
        [*command, *row_command(row, args, world, 0, steps)],
        env,
        args.output_dir / f"{tag}.log",
        args.timeout,
    )
    samples = []
    for path in args.output_dir.glob(f"{tag}.*.cg"):
        data = path.read_text(encoding="utf-8", errors="replace")
        events = re.search(r"^events:\s+(.+)$", data, re.MULTILINE)
        summary = re.search(r"^summary:\s+(.+)$", data, re.MULTILINE)
        objects = re.findall(
            r"^(?:c?ob)=.*?(/[^\n]*?/libdart\.so[^\n]*)$", data, re.MULTILINE
        )
        if not events or not summary or not objects:
            continue
        for obj in objects:
            identity(obj, args.prefix)
            native_lib = re.search(
                r"^STEPALLOC .* libdart=(.+)$",
                native_log(args, row).read_text(),
                re.MULTILINE,
            )
            if not native_lib or Path(obj).resolve() != Path(native_lib[1]).resolve():
                raise ValueError(
                    f"callgrind/native libdart identity differs: {Path(obj).name}"
                )
        samples.append(dict(zip(events[1].split(), map(int, summary[1].split()))))
    if not samples:
        raise ValueError(f"missing callgrind counts or libdart identity: {tag}")
    if steps == row.warmup + row.steps:
        observed = guards(text) if row.det else micro_guards(text)
        if observed != native_guards(args, row):
            raise ValueError(f"native/valgrind guards differ: {row.key}")
    return max(samples, key=lambda sample: sample["Ir"])


def native_log(args, row: Row) -> Path:
    tag = row.key.replace("/", ".") + ".native"
    if not row.det:
        tag += f".{row.warmup + row.steps}"
    return args.output_dir / f"{tag}.log"


def native_guards(args, row: Row) -> dict | None:
    text = native_log(args, row).read_text(encoding="utf-8", errors="replace")
    return guards(text) if row.det else micro_guards(text)


def robot_data_paths(source_dir: Path) -> list[Path]:
    """The loaded models and their mesh/URI references in this arm's checkout."""
    source_dir = source_dir.resolve()
    models = [source_dir / name for name in WORKLOAD_DATA["robot"]]
    paths = set(models)
    for model in models:
        root = ET.parse(model).getroot()
        references = [node.text.strip() for node in root.iter("uri") if node.text]
        references += [
            node.attrib["filename"]
            for node in root.iter("mesh")
            if "filename" in node.attrib
        ]
        for reference in references:
            path = (model.parent / reference.removeprefix("file://")).resolve()
            # Never hash a resource from outside the selected revision.
            if not path.is_relative_to(source_dir):
                display = reference
                referenced = Path(reference.removeprefix("file://"))
                if referenced.is_absolute():
                    display = (
                        referenced.relative_to(source_dir).as_posix()
                        if referenced.is_relative_to(source_dir)
                        else referenced.name
                    )
                raise ValueError(f"{model.name}: {display} is outside the revision")
            paths.add(path)
    return sorted(paths)


def row_result(row: Row, args=None) -> dict:
    result = {
        "row": row.row,
        "det": row.det,
        "version": row.version,
        "gated": False,
        "status": "ok",
        "threads": row.threads,
        "window": {"warmup": row.warmup, "steps": row.steps},
        "method": (
            "slope" if row.ir and not getattr(args, "native_only", False) else "native"
        ),
        "expected_ir": row.ir and not getattr(args, "native_only", False),
        "collection_signature": (
            COLLECTION_SIGNATURES[row.driver]
            if row.ir and not getattr(args, "native_only", False)
            else None
        ),
        "parity": row.parity,
        "qualification_required": row.perturb,
    }
    return result


def measure(row: Row, args, world: Path) -> dict:
    result = row_result(row, args)
    result["input_sha"] = None
    try:
        try:
            if "{world}" in row.args:
                result["input_sha"] = WORLD_SHA
            elif row.row == "pend":
                result["input_sha"] = sha(
                    (args.source_dir / WORKLOAD_DATA["pend"][0]).read_bytes()
                )
            elif row.row == "robot":
                result["input_sha"] = sha(
                    b"".join(
                        path.relative_to(args.source_dir.resolve()).as_posix().encode()
                        + path.read_bytes()
                        for path in robot_data_paths(args.source_dir)
                    )
                )
            else:
                result["input_sha"] = sha(
                    json.dumps([row.row, row.args], sort_keys=True).encode()
                )
        except (OSError, ValueError, ET.ParseError) as error:
            kind = (
                ValueError if getattr(args, "base_arm", False) else BenchmarkCaseError
            )
            text = str(error)
            for source in (args.source_dir, args.source_dir.resolve()):
                text = text.replace(f"{source}/", "")
            raise kind(f"{row.key}: failed to load revision inputs: {text}") from error
        metrics = (
            native(row, args, world) if row.det else micro_perturb(row, args, world, "")
        )
        result["head"] = metrics
        if (metrics.get("guards") or {}).get("finite") is False:
            result.update(status="broken", error="non-finite state")
            return result
        if metrics.get("time_advanced") is False:
            result.update(status="broken", error="simulation time did not advance")
            return result
        if args.perturb and row.perturb:
            result["perturbations"] = {}
            for config in PERTURBATIONS:
                altered = (
                    native(row, args, world, config)
                    if row.det
                    else micro_perturb(row, args, world, config)
                )
                # Requested bytes gate too, so they must not depend on layout,
                # and every perturbed run must also advance time correctly.
                stable = (
                    altered.get("time_advanced") is not False
                    and guard_evidence(altered) == guard_evidence(metrics)
                    and all(
                        altered.get(key) == metrics.get(key)
                        for key in ("allocs", "bytes")
                    )
                )
                result["perturbations"][config] = {
                    "stable": stable,
                    "guards": altered["guards"],
                    "max_penetration": altered.get("max_penetration"),
                    "checkpoints": altered.get("checkpoints"),
                    "allocs": altered["allocs"],
                    "bytes": altered.get("bytes"),
                    "time_advanced": altered.get("time_advanced"),
                }
            result["gated"] = all(
                item["stable"] for item in result["perturbations"].values()
            )
        if row.ir and not args.native_only:
            before = callgrind(row, args, world, row.warmup)
            after = callgrind(row, args, world, row.warmup + row.steps)
            counts = {key: after[key] - value for key, value in before.items()}
            if counts["Ir"] <= 0:
                raise ValueError(f"nonpositive slope: {row.key}")
            metrics["ir_per_step"] = counts["Ir"] / row.steps
            if args.cache_sim:
                l1 = sum(counts[key] for key in ("I1mr", "D1mr", "D1mw"))
                ll = sum(counts[key] for key in ("ILmr", "DLmr", "DLmw"))
                metrics["est_cycles_per_step"] = (
                    sum(counts[key] for key in ("Ir", "Dr", "Dw")) + 4 * l1 + 30 * ll
                ) / row.steps
    except BenchmarkCaseError as error:
        result.update(status="broken", gated=False, error=str(error))
        result.setdefault("head", {})
    except UnsupportedRow as error:
        result.update(
            status="unsupported",
            gated=False,
            error=str(error),
            head={},
            perturbations={},
        )
    except (OSError, ValueError, KeyError) as error:
        result.update(
            status="broken", gated=False, error=str(error), error_kind="infrastructure"
        )
    return result


class BenchmarkCaseError(ValueError):
    """A completed benchmark case reported a correctness failure."""


def micro_perturb(row: Row, args, world: Path, config: str) -> dict:
    counts = []
    for steps in (row.warmup, row.warmup + row.steps):
        tag = f"{row.row}.native.{steps}" + (f".{config}" if config else "")
        path = args.output_dir / f"{tag}.json"
        command = [
            *row_command(row, args, world, 0, steps),
            "--benchmark_out_format=json",
            f"--benchmark_out={path}",
        ]
        env = perturb_environment(environment(args.prefix), args, config)
        env.update(PERF_MICRO="1", PERF_WINDOW="micro", PERF_WARMUP="0")
        text = execute(
            command,
            env,
            path.with_suffix(".log"),
            args.timeout,
        )
        match = re.search(
            r"^STEPALLOC steps=(\d+) measured=(\d+) allocs=(\d+) bytes=(\d+) libdart=(.+)$",
            text,
            re.MULTILINE,
        )
        if not match:
            raise ValueError(f"missing micro native identity: {row.key}")
        identity(match[5], args.prefix)
        data = json.loads(path.read_text(encoding="utf-8"))
        cases = data.get("benchmarks", [])
        for item in cases:
            if item.get("error_occurred"):
                raise BenchmarkCaseError(f"{item['name']}: {item['error_message']}")
        if not cases or any(item["iterations"] != steps for item in cases):
            raise ValueError(f"micro iteration count differs: {row.key}")
        names = [item["name"].split("/iterations:")[0] for item in cases]
        observed = micro_guards(text)
        if names != MICRO_CASES[row.row] or (
            observed is not None and list(observed["hash"]) != names
        ):
            raise ValueError(f"micro cases or checksums differ: {row.key}")
        instrumented = observed is not None
        # Older archives have neither checksums nor explicit allocation hooks.
        # Partial instrumentation still fails rather than hiding a broken hook.
        expected = (len(names), len(names)) if instrumented else (0, 0)
        if tuple(map(int, match.groups()[:2])) != expected or (
            not instrumented and any(int(match[index]) for index in (3, 4))
        ):
            raise ValueError(f"micro allocation window count differs: {row.key}")
        counts.append(
            {
                "cases": names,
                "micro_instrumented": instrumented,
                "guards": observed,
                "allocs": int(match[3]) if instrumented else None,
                "bytes": int(match[4]) if instrumented else None,
            }
        )
    result = counts[1]
    if counts[0]["micro_instrumented"] != result["micro_instrumented"]:
        raise ValueError(f"micro instrumentation differs between runs: {row.key}")
    if not result["micro_instrumented"]:
        result.update(allocs_per_step=None, bytes_per_step=None)
        return result
    for key in ("allocs", "bytes"):
        result[key] -= counts[0][key]
        if result[key] < 0:
            raise ValueError(f"negative micro {key} slope: {row.key}")
    result["allocs_per_step"] = result["allocs"] / row.steps
    result["bytes_per_step"] = result["bytes"] / row.steps
    return result


def select_rows(names: str) -> list[Row]:
    if not names:
        return ROWS
    if names == "nightly":
        return [*ROWS, *NIGHTLY_ROWS]
    selected = []
    for name in names.split(","):
        matches = [
            row
            for row in [*ROWS, *NIGHTLY_ROWS]
            if name in (row.key, row.row) or name == "mt4" and row.parity
        ]
        if not matches:
            raise ValueError(f"unknown row {name}")
        selected += [row for row in matches if row not in selected]
    return selected


def cmake_compiler(build: Path) -> dict[str, str]:
    files = list((build / "CMakeFiles").glob("*/CMakeCXXCompiler.cmake"))
    if len(files) != 1:
        raise ValueError(f"missing or ambiguous CMake compiler provenance: {build}")
    text = files[0].read_text(encoding="utf-8")
    parts = []
    for key in ("ID", "VERSION"):
        match = re.search(
            rf'^set\(CMAKE_CXX_COMPILER_{key} "([^"]+)"\)$', text, re.MULTILINE
        )
        if not match:
            raise ValueError(f"missing CMake compiler {key}: {build}")
        parts.append(match[1])
    match = re.search(r'^set\(CMAKE_CXX_COMPILER "([^"]+)"\)$', text, re.MULTILINE)
    if not match:
        raise ValueError(f"missing CMake compiler executable: {build}")
    return {
        "compiler": " ".join(parts),
        "compiler_sha": sha(Path(match[1]).resolve(strict=True).read_bytes()),
    }


def valgrind_hashes() -> dict[str, str]:
    launcher = Path(VALGRIND).resolve(strict=True)
    prefix = launcher.parent.parent
    tools = {
        path.relative_to(prefix).as_posix(): sha(path.resolve(strict=True).read_bytes())
        for directory in ("libexec/valgrind", "lib/valgrind")
        for path in sorted((prefix / directory).glob("callgrind-*-linux"))
    }
    if not tools:
        raise ValueError(f"missing Valgrind Callgrind tool provenance: {prefix}")
    return {
        "valgrind_sha": sha(launcher.read_bytes()),
        "callgrind_sha": sha(json.dumps(tools, sort_keys=True).encode()),
    }


def library_hashes(prefix: Path) -> dict[str, str]:
    return {
        path.relative_to(prefix).as_posix(): sha(path.read_bytes())
        for path in sorted((prefix / "lib").rglob("libdart*.so*"))
        if path.is_file()
    }


def workload_hashes(
    source: Path, drivers, build: Path | None = None, prefix: Path | None = None
) -> dict[str, str]:
    commands = (
        json.loads((build / "compile_commands.json").read_text(encoding="utf-8"))
        if build is not None
        else []
    )

    def normalize(command):
        roots = [(source, "<SOURCE>"), (build, "<BUILD>"), (prefix, "<PREFIX>")]
        for path, label in sorted(
            ((str(path.resolve()), label) for path, label in roots if path is not None),
            key=lambda item: len(item[0]),
            reverse=True,
        ):
            command = command.replace(path, label)
        return command

    hashes = {}
    for driver in sorted(set(drivers) & WORKLOAD_SOURCES.keys()):
        paths = WORKLOAD_SOURCES[driver]
        if driver == CB:
            # Match the target's glob, so added or renamed files count too.
            paths = sorted(
                path.relative_to(source).as_posix()
                for pattern in ("*.cpp", "*.hpp", "CMakeLists.txt")
                for path in (source / "examples/contact_benchmark").glob(pattern)
            )
            if not any(path.endswith(".cpp") for path in paths):
                raise ValueError("missing workload source: examples/contact_benchmark")
        elif not (source / paths[0]).is_file():
            raise ValueError(f"missing workload source: {paths[0]}")
        # Scenes the sources load from the revision's own data directory
        # (dart://sample/... resolves to data/...) are inputs too.
        paths = sorted(
            {*paths}
            | {
                "data/" + uri
                for name in paths
                if (source / name).is_file()
                for uri in re.findall(
                    r'"dart://sample/([^"]+)"',
                    (source / name).read_text(encoding="utf-8", errors="replace"),
                )
            }
        )
        manifest = {
            name: (
                sha((source / name).read_bytes()) if (source / name).is_file() else None
            )
            for name in paths
        }
        if build is not None:
            compiled = {}
            for name in paths:
                if not name.endswith(".cpp"):
                    continue
                entries = [
                    (
                        normalize(entry["directory"]),
                        normalize(
                            entry["command"]
                            if "command" in entry
                            else shlex.join(entry["arguments"])
                        ),
                    )
                    for entry in commands
                    if (Path(entry["directory"]) / entry["file"]).resolve()
                    == (source / name).resolve()
                ]
                if not entries:
                    raise ValueError(f"missing workload compile command: {name}")
                compiled[name] = sorted(entries)
            manifest["compile_commands"] = compiled
        # Older archives may lack instrumentation/case headers; their absence
        # must differ from adding them, without reading this checkout's files.
        hashes[driver] = sha(json.dumps(manifest, sort_keys=True).encode())
    return hashes


def installed_provenance(args) -> dict:
    path = args.prefix / "share/dart/perf-build.json"
    if not path.is_file():
        raise ValueError(
            f"missing installed compiler provenance: {path}; use local to build and stamp the install"
        )
    stamp = json.loads(path.read_text(encoding="utf-8"))
    if (
        not isinstance(stamp, dict)
        or stamp.get("schema") != "dart-perf-build/1"
        or not isinstance(stamp.get("binaries"), dict)
        or not isinstance(stamp.get("libraries"), dict)
        or not isinstance(stamp.get("workload_sources"), dict)
        or any(
            not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{64}", value)
            for value in stamp.get("workload_sources", {}).values()
        )
        or any(
            driver not in stamp["workload_sources"]
            for driver in stamp["binaries"].keys() & WORKLOAD_SOURCES.keys()
        )
        or any(
            not isinstance(stamp.get(key), str) or not stamp[key].strip()
            for key in ("compiler", "pixi_lock_sha", "preset")
        )
        or not isinstance(stamp.get("compiler_sha"), str)
        or not re.fullmatch(r"[0-9a-f]{64}", stamp["compiler_sha"])
    ):
        raise ValueError(f"invalid installed build provenance: {path}")
    if stamp.get("commit") != args.commit:
        raise ValueError("installed build provenance commit differs from --commit")
    if stamp.get("libdart_sha") != sha((args.prefix / "lib/libdart.so").read_bytes()):
        raise ValueError("installed build provenance libdart hash differs")
    if stamp["libraries"] != library_hashes(args.prefix):
        raise ValueError("installed build provenance DART library hashes differ")
    return stamp


def fingerprint(args, provenance: dict | None = None) -> dict:
    if provenance is None:
        provenance = installed_provenance(args)
    for name in {PB, *(row.driver for row in select_rows(args.rows))}:
        if provenance.get("binaries", {}).get(name) != sha(
            (args.bin_dir / name).read_bytes()
        ):
            raise ValueError(f"installed build provenance driver hash differs: {name}")
    env = environment(args.prefix)
    driver = str(args.bin_dir / "portable_step_bench")
    guest = execute(
        [VALGRIND, "--tool=none", driver, "--cpu-only"],
        env,
        args.output_dir / "cpu.log",
        args.timeout,
    )
    cpu = field(guest, "Guest CPU")
    version = command_output([VALGRIND, "--version"]).removeprefix("valgrind-")
    harness = sha(
        Path(__file__).read_bytes()
        + b"".join(
            path.read_bytes()
            for path in sorted((ROOT / "tools/perf").rglob("*"))
            if path.is_file()
        )
    )
    values = {
        "valgrind": version,
        **valgrind_hashes(),
        "valgrind_guest_cpu": cpu,
        "compiler": provenance["compiler"],
        "compiler_sha": provenance["compiler_sha"],
        "compiler_provenance": provenance["schema"],
        "glibc": command_output(["getconf", "GNU_LIBC_VERSION"]).split()[-1],
        "pixi_lock_sha": provenance["pixi_lock_sha"],
        "runtime_pixi_lock_sha": sha(
            (Path(env.get("PIXI_PROJECT_ROOT", ROOT)) / "pixi.lock").read_bytes()
        ),
        "runtime_environment": env.get("PIXI_ENVIRONMENT_NAME")
        or Path(env.get("CONDA_PREFIX", "")).name,
        "preset": provenance["preset"],
        "harness_sha": harness,
        "allocshim_sha": sha(args.shim.read_bytes()),
        "heappad_sha": sha(args.heappad.read_bytes()) if args.perturb else None,
    }
    values["runner"] = {
        "environment": os.environ.get("RUNNER_ENVIRONMENT", "local"),
        "name": os.environ.get("RUNNER_NAME", "local"),
        "image": (
            f"{os.environ['ImageOS']}/{os.environ['ImageVersion']}"
            if os.environ.get("ImageOS") and os.environ.get("ImageVersion")
            else ""
        ),
    }
    values["host_cpu"] = next(
        (
            line.split(":", 1)[1].strip()
            for line in Path("/proc/cpuinfo").read_text().splitlines()
            if line.startswith("model name")
        ),
        "unknown",
    )
    values["fingerprint"] = sha(
        json.dumps(
            {
                key: value
                for key, value in values.items()
                if key not in ("runner", "host_cpu")
            },
            sort_keys=True,
        ).encode()
    )
    return values


def planned_rows(args) -> list[Row]:
    rows = select_rows(args.rows)
    for row in list(rows):
        if row.parity and all(other.key != row.parity for other in rows):
            rows.append(
                next(
                    other for other in [*ROWS, *NIGHTLY_ROWS] if other.key == row.parity
                )
            )
    return rows


def run_arm(args) -> dict:
    if getattr(args, "nightly", False):
        if args.rows:
            raise ValueError("--nightly uses the complete nightly row set; omit --rows")
        args.rows = "nightly"
    commit = command_output(
        ["git", "rev-parse", "--verify", f"{args.commit}^{{commit}}"]
    )
    args.prefix = args.prefix.resolve()
    args.bin_dir = (args.bin_dir or args.prefix / "bin").resolve()
    args.output_dir = args.output_dir.resolve()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for option, name in (("shim", "allocshim"), ("heappad", "heappad")):
        path = getattr(args, option)
        if path is None:
            path = args.prefix.parent / "shims" / f"{name}.so"
            if not path.is_file():
                path = ROOT / "build/perf" / f"lib{name}.so"
        path = path.resolve()
        if (option == "shim" or args.perturb) and not path.is_file():
            raise ValueError(f"--{option} file is missing: {path}")
        setattr(args, option, path)
    args.commit = commit
    provenance = installed_provenance(args)
    env_fingerprint = fingerprint(args, provenance)
    inputs = args.output_dir.parent / "inputs"
    inputs.mkdir(exist_ok=True)
    world = inputs / "3k_shapes.sdf"
    data = gzip.decompress(
        (ROOT / "tests/benchmark/worlds/3k_shapes.sdf.gz").read_bytes()
    )
    if sha(data) != WORLD_SHA:
        raise ValueError("3k_shapes input hash mismatch")
    if not world.exists() or sha(world.read_bytes()) != WORLD_SHA:
        with tempfile.NamedTemporaryFile(dir=inputs, delete=False) as temporary:
            temporary.write(data)
        os.replace(temporary.name, world)
    rows = planned_rows(args)
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        try:
            results = list(pool.map(lambda row: measure(row, args, world), rows))
        except BaseException:
            # Workers wait on their benchmarks; stop those before the pool's
            # shutdown waits for the workers.
            kill_running()
            raise
    for row, result in zip(rows, results):
        if (
            args.rows == "nightly"
            and row.driver == CB
            and result["status"] == "ok"
            and "pairs" not in (result.get("head", {}).get("guards") or {})
        ):
            result.update(
                status="broken", gated=False, error="missing contact pair count"
            )
        if row.driver in WORKLOAD_SOURCES:
            result["workload_sha"] = provenance["workload_sources"][row.driver]
            if result["input_sha"] is not None:
                result["input_sha"] = sha(
                    json.dumps([result["input_sha"], result["workload_sha"]]).encode()
                )
    by_key = {row_key(result): result for result in results}
    for result in results:
        if result["parity"] and result["status"] == "ok":
            serial = by_key.get(result["parity"])
            # Compare every guard, including contacts, cap hit, resting and finite.
            if (
                not serial
                or serial["status"] != "ok"
                or guard_evidence(serial["head"]) != guard_evidence(result["head"])
            ):
                result.update(
                    status="broken", error="mt4 guard parity missing or unequal"
                )
    record = {
        "schema": "dart-perf/1",
        "run": {
            "tier": "local",
            "harness_commit": command_output(["git", "rev-parse", "HEAD"]),
            "commit": commit,
            "parent": "",
            "branch": command_output(["git", "branch", "--show-current"]),
            "pr": None,
            "describe": command_output(
                ["git", "describe", "--tags", "--match", "v6*", "--always", commit]
            ),
            "time": datetime.now(timezone.utc).isoformat(),
            "env": env_fingerprint,
            "accepted": [],
        },
        "results": results,
    }
    write_json(args.output_dir / "record.json", record)
    return record


def row_key(row: dict) -> str:
    return row["row"] + ("/" + row["det"] if row.get("det") else "")


def rationales(body: str) -> list[dict]:
    accepted = []
    for line in body.splitlines():
        match = re.fullmatch(
            r"\s*(Perf-Regression|Rebaseline)-Rationale:\s*([^:]+):\s*(\S.*)", line
        )
        if match:
            accepted.append(
                {
                    "kind": (
                        "regression" if match[1] == "Perf-Regression" else "rebaseline"
                    ),
                    "rows": [row.strip() for row in match[2].split(",")],
                    "rationale": line.strip(),
                }
            )
    return accepted


def delta(base, head):
    return (head - base) / base if base else (0 if not head else None)


def at_least(value: float, threshold: float) -> bool:
    return value >= threshold or math.isclose(value, threshold, abs_tol=1e-12)


def percent(value: float | None) -> str:
    if value is None:
        return "—"
    # Never round a small nonzero delta to zero.
    return f"{100 * value:+.6g}%" if 0 < abs(value) < 0.00001 else f"{value:+.3%}"


def complete(row: dict, metrics: dict) -> bool:
    if row.get("expected_ir", row.get("method") == "slope"):
        value = metrics.get("ir_per_step")
        if (
            not isinstance(value, (float, int))
            or not math.isfinite(value)
            or value <= 0
        ):
            return False
    if row.get("det") or row.get("row") in ("dyn", "lcp"):
        if not row.get("det") and metrics.get("micro_instrumented") is False:
            return metrics.get("cases") == MICRO_CASES[row["row"]] and all(
                key in metrics and metrics[key] is None
                for key in (
                    "guards",
                    "allocs",
                    "bytes",
                    "allocs_per_step",
                    "bytes_per_step",
                )
            )
        for key in ("allocs_per_step", "bytes_per_step"):
            value = metrics.get(key)
            if (
                not isinstance(value, (float, int))
                or not math.isfinite(value)
                or value < 0
            ):
                return False
        guard = metrics.get("guards")
        if not isinstance(guard, dict) or not guard.get("hash"):
            return False
        if guard.get("finite") is not True:
            return False
        if row.get("det"):
            if not all(key in guard for key in ("contacts", "cap_hit", "resting")):
                return False
        elif (
            not isinstance(guard["hash"], dict)
            or metrics.get("cases") != MICRO_CASES[row["row"]]
            or set(guard["hash"]) != set(metrics["cases"])
            or any(
                not isinstance(value, str) or not re.fullmatch(r"0x[0-9a-f]{16}", value)
                for value in guard["hash"].values()
            )
        ):
            return False
    return bool(metrics)


def compare(base: dict, head: dict, body: str = "") -> dict:
    accepted = rationales(body)
    base_env, head_env = base["run"]["env"], head["run"]["env"]
    differing = [
        key
        for key in sorted(base_env.keys() | head_env.keys())
        if key not in ("fingerprint", "runner")
        and base_env.get(key) != head_env.get(key)
    ]
    if any(
        env.get("compiler_provenance") != "dart-perf-build/1"
        or not isinstance(env.get("compiler"), str)
        or not env["compiler"].strip()
        for env in (base_env, head_env)
    ):
        differing.append("compiler provenance (missing or invalid)")
    if any(
        not isinstance(env.get(key), str) or not re.fullmatch(r"[0-9a-f]{64}", env[key])
        for env in (base_env, head_env)
        for key in ("compiler_sha", "valgrind_sha", "callgrind_sha")
    ):
        differing.append("tool executable provenance (missing or invalid)")
    failures = []
    if (
        not base_env.get("fingerprint")
        or not head_env.get("fingerprint")
        or base_env["fingerprint"] != head_env["fingerprint"]
        or differing
    ):
        failures.append(
            "incompatible environment fingerprints: "
            + ", ".join(differing or ["fingerprint (missing or unequal)"])
        )
    else:
        parents = {row_key(row): row for row in base["results"]}
        children = {row_key(row): row for row in head["results"]}
        for key in sorted(parents.keys() & children.keys()):
            methods = [
                (
                    row.get("method"),
                    row.get("expected_ir", row.get("method") == "slope"),
                )
                for row in (parents[key], children[key])
            ]
            if methods[0] != methods[1]:
                failures.append(f"{key}: incompatible measurement method/expected_ir")
    if failures:
        return {
            "schema": "dart-perf/1",
            "run": {
                **head["run"],
                "parent": base["run"]["commit"],
                "accepted": accepted,
            },
            "results": [],
            "verdict": {
                "status": "ERROR",
                "failures": failures,
                "warnings": [],
                "ir_geomean": None,
            },
        }

    def acknowledgment(kind, key):
        return next(
            (
                item["rationale"]
                for item in accepted
                if item["kind"] == kind and key in item["rows"]
            ),
            "",
        )

    results, ratios, failures, warnings = [], [], [], []
    infrastructure = False
    if not parents and not children:
        failures.append("measurement record has no rows")
    for key in dict.fromkeys([*parents, *children]):
        parent, child = parents.get(key), children.get(key)
        bm = parent.get("head", {}) if parent else {}
        hm = child.get("head", {}) if child else {}
        result = {
            **(child or parent),
            "parent": bm,
            "parent_status": parent.get("status") if parent else None,
            "head": hm,
        }
        input_changed = bool(
            parent and child and parent.get("input_sha") != child.get("input_sha")
        )
        for arm, row in (("base", parent), ("head", child)):
            if row and row.get("error_kind") == "infrastructure":
                infrastructure = True
                failures.append(f"{key}: {arm} infrastructure error: {row['error']}")
            elif row and row.get("perturbations") and not row.get("gated"):
                failures.append(f"{key}: {arm} perturbation check failed")
        valid_measurements = bool(
            parent and child and complete(parent, bm) and complete(child, hm)
        )
        missing_micro = [
            arm
            for arm, metrics in (("base", bm), ("head", hm))
            if metrics.get("micro_instrumented") is False
        ]
        ir = (
            delta(bm["ir_per_step"], hm["ir_per_step"])
            if valid_measurements
            and not input_changed
            and "ir_per_step" in bm
            and "ir_per_step" in hm
            else None
        )
        allocs = (
            hm["allocs_per_step"] - bm["allocs_per_step"]
            if valid_measurements
            and not input_changed
            and not missing_micro
            and "allocs_per_step" in bm
            and "allocs_per_step" in hm
            else None
        )
        size = (
            hm["bytes_per_step"] - bm["bytes_per_step"]
            if valid_measurements
            and not input_changed
            and not missing_micro
            and "bytes_per_step" in bm
            and "bytes_per_step" in hm
            else None
        )
        equal = (
            None
            if missing_micro
            else bool(bm.get("guards") and guard_evidence(bm) == guard_evidence(hm))
        )
        reasons = []
        required_missing = bool(
            parent
            and child
            and parent.get("status") == "ok"
            and parent.get("version") == child.get("version")
            and not complete(parent, hm)
        )
        if (
            parent
            and child
            and parent.get("status") == child.get("status") == "unsupported"
        ):
            classification = "unsupported"
            result["gated"] = False
        elif (
            not child
            or child.get("status") != "ok"
            or not complete(child, hm)
            or required_missing
        ):
            classification = "broken"
            reasons.append(result.get("error", "missing or failed head measurement"))
        elif parent and (bm.get("guards") or {}).get("finite") is False:
            classification = "broken"
            reasons.append("base state is non-finite")
        elif parent and "head" in missing_micro and bm.get("guards"):
            # Losing an established guard is a policy failure, like a missing
            # required head measurement; a rationale cannot waive it.
            classification = "broken"
            reasons.append("head lacks micro instrumentation present in base")
        elif (
            not parent
            or parent.get("status") == "unsupported"
            or (
                parent.get("version") != child.get("version")
                and not input_changed
                and not missing_micro
            )
        ):
            classification = "new"
        elif parent.get("status") != "ok" or not complete(parent, bm):
            classification = "broken"
            reasons.append("missing or failed base measurement")
        elif missing_micro and input_changed:
            # A changed workload still needs a rationale; with an uninstrumented
            # side there is no delta to report.
            classification = "behaviour-change"
            result["gated"] = False
            if not acknowledgment("rebaseline", key):
                reasons.append("input_sha changed; Rebaseline-Rationale required")
        elif missing_micro:
            classification = "diagnostic"
            result["gated"] = False
        elif input_changed or not equal:
            classification = "behaviour-change"
            rationale = acknowledgment("rebaseline", key)
            if not rationale:
                changed = "input_sha" if input_changed else "guards"
                reasons.append(f"{changed} changed; Rebaseline-Rationale required")
            elif (
                ir is not None
                and ir > 0.01
                and not math.isclose(ir, 0.01, abs_tol=1e-12)
                and not re.search(r"[+-]\d+(?:\.\d+)?\s*%", rationale)
            ):
                reasons.append(
                    f"Rebaseline-Rationale must state a signed percentage (measured Ir {ir:+.2%})"
                )
        elif not parent.get("gated", False):
            classification = "diagnostic"
        else:
            classification = "gated"
            result["gated"] = True
            if not child.get("gated", False):
                reasons.append(
                    "head failed perturbation eligibility; base gate remains active"
                )
            if (
                parent.get("expected_ir", parent.get("method") == "slope")
                and ir is None
            ):
                reasons.append("missing or invalid Ir measurement on base or head")
            if "allocs_per_step" in bm and "allocs_per_step" not in hm:
                reasons.append("missing head allocation measurement")
            if "bytes_per_step" in bm and "bytes_per_step" not in hm:
                reasons.append("missing head requested-byte measurement")
            if (
                allocs is not None
                and allocs > 0
                and not acknowledgment("regression", key)
            ):
                reasons.append(f"allocations +{allocs:g}/step")
            if size is not None and size > 0 and not acknowledgment("regression", key):
                reasons.append(f"requested bytes +{size:g}/step")
            if ir is not None:
                ratios.append((key, hm["ir_per_step"] / bm["ir_per_step"]))
                if at_least(ir, 0.01) and not acknowledgment("regression", key):
                    reasons.append(f"Ir {ir:+.2%} (limit +1.00%)")
                elif at_least(ir, 0.003):
                    warnings.append(f"{key}: Ir {ir:+.2%}")
        if (
            not input_changed
            and bm.get("max_rss_kb")
            and hm.get("max_rss_kb")
            and at_least(delta(bm["max_rss_kb"], hm["max_rss_kb"]), 0.05)
        ):
            warnings.append(
                f"{key}: RSS {delta(bm['max_rss_kb'], hm['max_rss_kb']):+.2%} (advisory)"
            )
        result["delta"] = {
            "ir": ir,
            "allocs": allocs,
            "bytes": size,
            "guards_equal": equal,
            "class": classification,
        }
        result["wall_ms_per_step"] = {
            "parent": bm.get("wall_ms_per_step"),
            "head": hm.get("wall_ms_per_step"),
            "advisory": True,
        }
        result["failures"] = reasons
        result["gate_reason"] = (
            "base perturbation check passed; base gate remains active"
            if classification == "gated"
            else (
                "perturbation check passed; row is " + classification
                if result.get("gated")
                else (
                    "perturbation check failed"
                    if result.get("perturbations")
                    else "perturbation check not run"
                )
            )
        )
        if input_changed:
            result["gate_reason"] += "; input_sha changed"
        if missing_micro:
            result["gate_reason"] = "; ".join(
                f"{arm} lacks micro instrumentation" for arm in missing_micro
            )
        failures += [f"{key}: {reason}" for reason in reasons]
        results.append(result)
    geomean = (
        math.expm1(math.fsum(math.log(ratio) for _, ratio in ratios) / len(ratios))
        if ratios
        else None
    )
    if geomean is not None and at_least(geomean, 0.005):
        uncovered = [
            key
            for key, ratio in ratios
            if ratio > 1 and not acknowledgment("regression", key)
        ]
        if uncovered:
            failures.append(
                f"Ir geomean {geomean:+.2%} (limit +0.50%); rationale required for {', '.join(uncovered)}"
            )
    record = {
        "schema": "dart-perf/1",
        "run": {**head["run"], "parent": base["run"]["commit"], "accepted": accepted},
        "results": results,
        "verdict": {
            "status": (
                "ERROR"
                if infrastructure
                else "FAIL" if failures else "WARN" if warnings else "PASS"
            ),
            "failures": failures,
            "warnings": warnings,
            "ir_geomean": geomean,
        },
    }
    return record


def markdown_cell(value) -> str:
    return (
        str(value)
        .replace("|", "\\|")
        .replace("\r\n", "\n")
        .replace("\r", "\n")
        .replace("\n", "<br>")
    )


def row_change(row: dict) -> str:
    classification = row["delta"]["class"]
    if classification == "gated":
        change = row["delta"]
        # Match the gate: Ir increases below its +0.30% warning threshold are
        # neutral; allocation and byte increases regress even when acknowledged.
        classification = (
            "regressed"
            if row["failures"]
            or any((change[key] or 0) > 0 for key in ("allocs", "bytes"))
            or (change["ir"] is not None and at_least(change["ir"], 0.003))
            else (
                "improved"
                if any((change[key] or 0) < 0 for key in ("allocs", "bytes"))
                or (change["ir"] is not None and at_least(-change["ir"], 0.01))
                else "neutral"
            )
        )
    return classification


def markdown(record: dict) -> str:
    verdict = record["verdict"]
    env = record["run"]["env"]
    method = (
        "Slope method"
        if any(row["method"] == "slope" for row in record["results"])
        else "Native only (no Ir gate)"
    )
    threads = sorted({row.get("threads", 1) for row in record["results"]})
    thread_text = (
        "no measured rows"
        if not threads
        else "/".join(map(str, threads)) + (" thread" if threads == [1] else " threads")
    )
    lines = [
        (
            f"Perf smoke: {record['run']['commit']} — {verdict['status']}"
            if record["run"].get("mode") == "smoke"
            else f"Perf A/B: {record['run']['parent']} → {record['run']['commit']} — {verdict['status']}"
        ),
        f"{method}; {thread_text}; Valgrind {env['valgrind']}; {env.get('compiler') or 'compiler unavailable'}; glibc {env['glibc']}; {env['preset']}",
        "",
    ]
    counts = {}
    for row in record["results"]:
        classification = row_change(row)
        counts[classification] = counts.get(classification, 0) + 1
    lines += [", ".join(f"{count} {kind}" for kind, count in counts.items()) + ".", ""]
    lines += [f"- {item}" for item in [*verdict["failures"], *verdict["warnings"]]]
    if verdict["ir_geomean"] is not None:
        lines += [f"Ir geomean: {percent(verdict['ir_geomean'])}."]
    lines += [
        "",
        "| Row | Threads | Base Ir/step | Head Ir/step | Delta | Allocs/step | Bytes/step delta | Guards | Class | Gate qualification |",
        "|---|---:|---:|---:|---:|---:|---:|---|---|---|",
    ]

    def number(value):
        return f"{value:,.2f}".rstrip("0").rstrip(".") if value is not None else "—"

    for row in record["results"]:
        bm, hm, change = row["parent"], row["head"], row["delta"]
        change_text = percent(change["ir"])
        bytes_text = f"{change['bytes']:+g}" if change["bytes"] is not None else "—"
        guard_text = (
            "unavailable"
            if change["guards_equal"] is None
            else "same" if change["guards_equal"] else "changed"
        )
        if row.get("det"):
            for label, metrics in (("base", bm), ("head", hm)):
                guard = metrics.get("guards") or {}
                values = ", ".join(
                    f"{key}={str(guard[key]).lower()}"
                    for key in ("contacts", "pairs", "resting", "cap_hit")
                    if key in guard
                )
                guard_text += f"; {label}: {values or 'unavailable'}"
        values = [
            row_key(row),
            row.get("threads", 1),
            number(bm.get("ir_per_step")),
            number(hm.get("ir_per_step")),
            change_text,
            f"{number(bm.get('allocs_per_step'))} → {number(hm.get('allocs_per_step'))}",
            bytes_text,
            guard_text,
            change["class"],
            row["gate_reason"],
        ]
        lines.append("| " + " | ".join(map(markdown_cell, values)) + " |")
    lines += [
        "",
        "Wall time is recorded as advisory; RSS warns at +5%. Diagnostic and behaviour-change deltas do not enter the Ir gate.",
    ]
    return "\n".join(lines) + "\n"


def finite_or_none(value: float) -> float | None:
    # Records reject NaN and infinity; a non-finite state already breaks the row.
    return value if math.isfinite(value) else None


def write_json(path: Path, value: dict) -> None:
    text = json.dumps(value, indent=2, allow_nan=False) + "\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def read_record(path: Path, *, comparison: bool = False) -> dict:
    if path.stat().st_size > 16 * 1024 * 1024:
        raise ValueError("record exceeds 16 MiB")
    record = json.loads(
        path.read_text(encoding="utf-8"),
        parse_constant=lambda value: (_ for _ in ()).throw(
            ValueError(f"invalid number {value}")
        ),
    )
    if (
        not isinstance(record, dict)
        or record.get("schema") != "dart-perf/1"
        or not record.get("results")
        or not isinstance(record["results"], list)
        or not all(
            isinstance(row, dict)
            and isinstance(row.get("row"), str)
            and isinstance(row.get("det") or "", str)
            and isinstance(row.get("head", {}), dict)
            for row in record["results"]
        )
    ):
        raise ValueError("missing or unsupported measurement record")
    keys = [row_key(row) for row in record["results"]]
    if len(keys) != len(set(keys)):
        raise ValueError("measurement record repeats a row")
    # A comparison report has the same schema, but its rows carry the base's
    # qualification and per-arm deltas, not one revision's measurements.
    if comparison:
        json.dumps(record, allow_nan=False)
        if (
            not isinstance(record.get("run"), dict)
            or not isinstance(record.get("verdict"), dict)
            or record["verdict"].get("status") not in ("PASS", "WARN", "FAIL")
            or not isinstance(record["verdict"].get("failures"), list)
            or not isinstance(record["verdict"].get("warnings"), list)
            or "ir_geomean" not in record["verdict"]
            or not all(
                isinstance(item, str)
                for kind in ("failures", "warnings")
                for item in record["verdict"][kind]
            )
            or not (
                record["verdict"]["ir_geomean"] is None
                or type(record["verdict"]["ir_geomean"]) in (int, float)
            )
            or not all(
                isinstance(row.get("parent"), dict)
                and row.get("parent_status") in (None, "ok", "broken", "unsupported")
                and isinstance(row.get("head"), dict)
                and isinstance(row.get("delta"), dict)
                and all(
                    key in row["delta"]
                    for key in ("class", "ir", "allocs", "bytes", "guards_equal")
                )
                and row["delta"]["class"]
                in (
                    "gated",
                    "new",
                    "broken",
                    "unsupported",
                    "diagnostic",
                    "behaviour-change",
                )
                and all(
                    value is None or type(value) in (int, float)
                    for value in (
                        row["delta"][key] for key in ("ir", "allocs", "bytes")
                    )
                )
                and (
                    row["delta"]["guards_equal"] is None
                    or isinstance(row["delta"]["guards_equal"], bool)
                )
                and isinstance(row.get("failures"), list)
                and all(isinstance(item, str) for item in row["failures"])
                and all(
                    metrics.get("guards") is None or isinstance(metrics["guards"], dict)
                    for metrics in (row["parent"], row["head"])
                )
                for row in record["results"]
            )
            or not isinstance(record["run"].get("accepted", []), list)
            or not all(
                isinstance(item, dict) and isinstance(item.get("rationale"), str)
                for item in record["run"].get("accepted", [])
            )
        ):
            raise ValueError("missing or unsupported comparison record")
    elif "verdict" in record:
        raise ValueError("comparison report given where a measurement record belongs")
    return record


def validate_publication_environment(env: dict, *, local: bool = False) -> None:
    if (
        not isinstance(env, dict)
        or not isinstance(env.get("runner"), dict)
        or env["runner"].get("environment") != ("local" if local else "github-hosted")
    ):
        raise ValueError("refusing publication from an unexpected measurement runner")
    if not re.fullmatch(r"[0-9a-f]{64}", env.get("fingerprint", "")):
        raise ValueError("publication environment fingerprint must be a SHA256")


def find_local_path(value, location: str = "") -> str | None:
    """Return the first JSON location containing a local path."""
    if location == "run.accepted":
        return None
    if isinstance(value, str):
        return location if LOCAL_PATH.search(value) else None
    if isinstance(value, dict):
        if any(isinstance(key, str) and LOCAL_PATH.search(key) for key in value):
            return f"{location}.<key>" if location else "<key>"
        children = ((str(key), item) for key, item in value.items())
    elif isinstance(value, list):
        children = ((str(index), item) for index, item in enumerate(value))
    else:
        return None
    for key, item in children:
        found = find_local_path(item, f"{location}.{key}" if location else key)
        if found:
            return found
    return None


def publication_record(
    path: Path,
    tier: str,
    pr: int | None = None,
    tag: str | None = None,
    base_tag: str | None = None,
) -> dict:
    """Normalize trusted measurements into a durable performance record."""
    if tier not in ("merge", "nightly", "release", "backfill"):
        raise ValueError("unsupported publication tier")
    if tier == "release":
        if not tag or not base_tag:
            raise ValueError("release publication requires tag and base-tag")
    elif tag is not None or base_tag is not None:
        raise ValueError("tag and base-tag require release publication")
    if path.stat().st_size > 16 * 1024 * 1024:
        raise ValueError("record exceeds 16 MiB")
    record = json.loads(path.read_text(encoding="utf-8"))
    # Reject non-finite JSON before writing either JSON or the chart script.
    json.dumps(record, allow_nan=False)
    if (
        not isinstance(record, dict)
        or record.get("schema") != "dart-perf/1"
        or not isinstance(record.get("run"), dict)
        or not isinstance(record.get("results"), list)
        or not record["results"]
    ):
        raise ValueError("missing or unsupported publication record")
    run = record["run"]
    local = tier == "backfill"
    validate_publication_environment(run.get("env", {}), local=local)
    if not re.fullmatch(r"[0-9a-f]{40}", run.get("commit", "")):
        raise ValueError("publication commit must be a full SHA")
    measured = datetime.fromisoformat(run["time"].replace("Z", "+00:00"))
    if measured.tzinfo is None:
        raise ValueError("publication time must include its timezone")
    run["time"] = measured.astimezone(timezone.utc).isoformat()
    if pr is not None and (not isinstance(pr, int) or pr <= 0):
        raise ValueError("publication PR number must be positive")
    release = tier == "release" or (local and run.get("tier") == "release")
    if tier != "nightly":
        if not re.fullmatch(r"[0-9a-f]{40}", run.get("parent", "")):
            raise ValueError("publication requires the comparison-base SHA")
        if record.get("verdict", {}).get("status") not in ("PASS", "WARN", "FAIL"):
            raise ValueError("publication requires a completed comparison")
    else:
        run["parent"] = None
        record.pop("verdict", None)
    if local:
        if not re.fullmatch(r"[0-9a-f]{40}", run.get("harness_commit", "")):
            raise ValueError("backfill publication requires the harness commit")
    branch = "main"
    if release:
        tag = tag if tier == "release" else run.get("tag")
        base_tag = base_tag if tier == "release" else run.get("base_tag")
        if not tag or not base_tag:
            raise ValueError("release record requires tag and base_tag")
        scope = release_scope(run["commit"], tag, base_tag)
        if (
            scope["head"] != run["commit"]
            or scope["base"] != run["parent"]
            or (run.get("tier") == "release" and run.get("branch") != scope["branch"])
        ):
            raise ValueError("release publication scope differs from measurement")
        branch = scope["branch"]
        run.update(tag=scope["tag"], base_tag=scope["base_tag"])
    run.update(
        tier="release" if release else tier,
        branch=branch,
        pr=None if release else pr if pr is not None else run.get("pr"),
        accepted=run.get("accepted", []),
    )
    keys = []
    for row in record["results"]:
        if (
            not isinstance(row, dict)
            or not isinstance(row.get("row"), str)
            or not isinstance(row.get("det", ""), str)
            or not isinstance(row.get("head"), dict)
            or row.get("error_kind") == "infrastructure"
        ):
            # Correctness failures (BenchmarkCaseError, missing pair counts) keep
            # an empty head in measure(), so they publish; only infrastructure
            # errors and malformed rows are rejected here.
            raise ValueError("invalid or incomplete publication row")
        keys.append(row_key(row))
        if "head_env" in row:
            validate_publication_environment(row["head_env"], local=local)
        if tier == "nightly":
            row.pop("parent", None)
            row.pop("delta", None)
        for arm in ("parent", "head"):
            metrics = row.get(arm, {})
            if "libdart" in metrics:
                metrics["libdart"] = Path(metrics["libdart"]).name
        row["wall_ms_per_step"] = {
            "parent": row.get("parent", {}).get("wall_ms_per_step"),
            "head": row["head"].get("wall_ms_per_step"),
            "advisory": True,
        }
    if len(keys) != len(set(keys)):
        raise ValueError("publication record repeats a row")
    location = find_local_path(record)
    if location:
        raise ValueError(f"publication record contains a local path at {location}")
    if local:
        if not is_ancestor(run["harness_commit"], "origin/main"):
            raise ValueError("backfill harness commit must be on main")
        if not is_ancestor(run["commit"], "origin/main"):
            raise ValueError("backfill commit must be on main")
    return record


def publication_guard(tier: str) -> None:
    """The record's runner claim cannot replace checking the actual CI context."""
    if tier == "backfill":
        if "GITHUB_ACTIONS" in os.environ:
            raise ValueError("backfill publication is forbidden in GitHub Actions")
        return
    if tier not in ("merge", "nightly", "release"):
        raise ValueError("unsupported publication tier")
    if os.environ.get("RUNNER_ENVIRONMENT") != "github-hosted":
        raise ValueError("refusing to publish from a non-hosted runner")
    events = (
        ("push", "workflow_dispatch")
        if tier == "merge"
        else (
            ("workflow_dispatch",)
            if tier == "release"
            else ("schedule", "workflow_dispatch")
        )
    )
    if (
        os.environ.get("GITHUB_EVENT_NAME") not in events
        or os.environ.get("GITHUB_REF") != "refs/heads/main"
    ):
        raise ValueError("publication is restricted to trusted main events")


def smoke_build_failure(directory: Path, head: str, status: int) -> dict | None:
    """Accept exit 2 only when smoke saved a build failure for the expected head."""
    if status not in (0, 1, 2):
        raise ValueError("head smoke infrastructure failure")
    path = directory / "build-failure.json"
    if path.exists():
        broken = json.loads(path.read_text(encoding="utf-8"))
        if (
            not isinstance(broken, dict)
            or broken.get("error_kind") != "build"
            or broken.get("commit") != head
            or not isinstance(broken.get("error"), str)
            or not broken["error"]
        ):
            raise ValueError("head smoke infrastructure failure")
        return broken
    if status == 2:
        raise ValueError("head smoke infrastructure failure")
    return None


def merge_comment(record: dict, report: str, previous: str = "") -> str | None:
    """Refresh a marked verdict comment without rolling back newer evidence."""
    run = record["run"]
    status = record["verdict"]["status"]
    if status not in ("PASS", "WARN", "FAIL"):
        raise ValueError("merge comment requires a completed comparison")
    if not previous and status != "FAIL":
        return None
    measured = datetime.fromisoformat(run["time"].replace("Z", "+00:00"))
    identity = re.search(r"<!-- dart-perf-verdict:(\S+) (PASS|WARN|FAIL) -->", previous)
    if identity:
        previous_time = datetime.fromisoformat(identity[1])
        if previous_time > measured or (
            previous_time == measured and identity[2] != "FAIL" and status == "FAIL"
        ):
            return None
    message = "Post-merge performance check now passes."
    if status == "FAIL":
        message = "Post-merge performance check failed."
        rationales, fixes = set(), []
        for failure in record["verdict"].get("failures", []):
            kind = next(
                (
                    kind
                    for _, kind, pattern in WAIVABLE_FAILURES
                    if re.fullmatch(pattern, failure)
                ),
                None,
            )
            if kind:
                rationales.add(kind)
            else:
                fixes.append(failure)
        if fixes or not rationales:
            message += " Fix the non-waivable failures listed below"
            if fixes:
                message += ":\n\n" + "\n".join(
                    f"- {markdown_cell(failure)}" for failure in fixes
                )
            else:
                message += "."
        if rationales:
            message += (
                "\n\nAdd the applicable "
                + " / ".join(sorted(rationales))
                + " to the merged PR body for the acknowledged comparison failures."
            )
        message += "\n\nRerun the full workflow (including measurement)."
    body = (
        f"<!-- dart-perf-merge:{run['commit']} -->\n"
        f"<!-- dart-perf-verdict:{measured.isoformat()} {status} -->\n"
        f"{message}\n\n{report}"
    )
    return None if body == previous else body


def is_ancestor(base: str, head: str) -> bool:
    def check():
        return subprocess.run(
            [
                "git",
                "-C",
                str(ROOT),
                "merge-base",
                "--is-ancestor",
                base,
                head,
            ],
            text=True,
            capture_output=True,
        )

    ancestry = check()
    if ancestry.returncode not in (0, 1):
        # A concurrent writer may name a main commit newer than this checkout.
        subprocess.run(
            ["git", "-C", str(ROOT), "fetch", "--quiet", "origin", "main"],
            capture_output=True,
        )
        ancestry = check()
    # A commit still unknown keeps the published table; the record is written.
    return ancestry.returncode == 0


def nightly_table_can_advance(record: dict, previous: str) -> bool:
    """The table's full SHA and measurement time identify its published nightly."""
    if not previous:
        return True
    identity = re.search(
        r"^Generated at (\S+) for `([0-9a-f]{40})`\.$", previous, re.MULTILINE
    )
    if not identity:
        raise ValueError("nightly guard table is missing its run identity")
    measured, commit = identity.groups()
    run = record["run"]
    if commit == run["commit"]:
        return datetime.fromisoformat(run["time"]) > datetime.fromisoformat(measured)
    return is_ancestor(commit, run["commit"])


def release_scope(head: str | None, tag: str | None, base: str | None) -> dict:
    """Resolve a release candidate and previous tag before configuring a build."""

    def git(*arguments):
        return command_output(["git", "-C", str(ROOT), *arguments])

    def resolve(revision):
        try:
            return git(
                "rev-parse", "--verify", "--end-of-options", f"{revision}^{{commit}}"
            )
        except (subprocess.CalledProcessError, ValueError) as error:
            raise ValueError(f"unknown release revision: {revision}") from error

    if tag is not None and not RELEASE_TAG.fullmatch(tag):
        raise ValueError("release tag must have the form v6.x.y")
    tags = set(git("tag", "--list").splitlines())
    if head is None:
        if tag not in tags:
            raise ValueError("release head is required when the tag does not exist")
        head = tag
    head = resolve(head)
    try:
        version = ET.fromstring(git("show", f"{head}:package.xml")).findtext("version")
    except (subprocess.CalledProcessError, ValueError, ET.ParseError) as error:
        raise ValueError(
            "release candidate has no valid package.xml version"
        ) from error
    derived = f"v{version}"
    if not RELEASE_TAG.fullmatch(derived) or (tag and tag != derived):
        raise ValueError("release tag differs from the package.xml version")
    tag = tag or derived
    if tag in tags and resolve(tag) != head:
        raise ValueError("release tag does not name the candidate commit")
    branches = git(
        "for-each-ref", "--format=%(refname:short)", "refs/remotes/origin"
    ).splitlines()
    candidates = [
        "origin/main",
        *sorted(
            branch
            for branch in branches
            if re.fullmatch(r"origin/release-6\.\d+", branch)
        ),
    ]
    branch = next(
        (
            candidate.removeprefix("origin/")
            for candidate in candidates
            if candidate in branches
            and head in git("rev-list", "--first-parent", candidate).splitlines()
        ),
        None,
    )
    if branch is None:
        raise ValueError("release candidate is not on main or a release branch")
    try:
        previous = [
            name
            for name in git("tag", "--merged", f"{head}^").splitlines()
            if RELEASE_TAG.fullmatch(name) and name != tag
        ]
    except subprocess.CalledProcessError as error:
        raise ValueError("release candidate has no previous release") from error
    if not previous or (base is not None and base not in previous):
        raise ValueError("release base must be an earlier merged v6 tag")
    base_tag = base or max(
        previous, key=lambda name: tuple(map(int, name[1:].split(".")))
    )
    return {
        "head": head,
        "tag": tag,
        "base_tag": base_tag,
        "base": resolve(base_tag),
        "branch": branch,
    }


def guard_table(record: dict, previous: str = "") -> str:
    run = record["run"]
    lines = [
        "# DART main canonical behaviour guards",
        "",
        f"Generated at {run['time']} for `{run['commit']}`.",
        f"Environment fingerprint: `{run['env']['fingerprint']}`.",
        "",
        "Commands and scene definitions: [baseline evidence](https://github.com/dartsim/dart/blob/main/docs/dev_tasks/dart6_performance_generalization/01-baseline-evidence.md).",
        "This table is generated evidence, not a fixed reference. Wall time is advisory.",
        "S3 and S6 drift belongs to the #3056 / D7 owners.",
        "",
        "| Row | Detector | Threads | Warm-up / steps | Status | Hash | Contacts | Pairs | Resting | Finite | Cap hit | Max penetration | Allocs / step |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    drift = []
    diagnostics = []
    previous_rows = {
        tuple(cell.strip() for cell in line.strip("|").split("|")[:2]): line
        for line in previous.splitlines()
        if line.startswith("| S")
    }
    for row in record["results"]:
        if not re.match(r"^S[1-6](?:-|$)", row["row"]):
            continue
        head = row["head"]
        guards = head.get("guards", {})
        window = row.get("window", {})
        penetration = guards.get("max_penetration", head.get("max_penetration", "—"))
        values = [
            row["row"],
            row.get("det", ""),
            row.get("threads", 1),
            f"{window.get('warmup', '—')} / {window.get('steps', '—')}",
            row.get("status", "—"),
            *(
                guards.get(key, "—")
                for key in ("hash", "contacts", "pairs", "resting", "finite", "cap_hit")
            ),
            "non-finite" if penetration is None else penetration,
            head.get("allocs_per_step", "—"),
        ]
        table_row = "| " + " | ".join(map(markdown_cell, values)) + " |"
        lines.append(table_row)
        if re.match(r"^S[36](?:-|$)", row["row"]):
            old_row = previous_rows.get((row["row"], row.get("det", "")))
            if old_row and old_row != table_row:
                drift.append(row_key(row))
        if row.get("error"):
            diagnostics.extend(["", f"`{row_key(row)}`: {row['error']}", ""])
        if head.get("checkpoints"):
            checkpoints = f"`{row_key(row)}` checkpoints: `{json.dumps(head['checkpoints'], sort_keys=True)}`"
            diagnostics.extend(
                [
                    "",
                    checkpoints,
                    "",
                ]
            )
            if previous and any(
                line.startswith(f"`{row_key(row)}` checkpoints:")
                and line != checkpoints
                for line in previous.splitlines()
            ):
                drift.append(row_key(row) + " checkpoints")
    lines.extend(diagnostics)
    if drift:
        lines.extend(
            [
                "",
                "S3 / S6 drift since the prior nightly (informational; #3056 / D7 owners): "
                + ", ".join(drift)
                + ".",
            ]
        )
    return "\n".join(lines) + "\n"


def deterministic_measurements(record: dict, *, include_parent: bool = False) -> dict:
    """Reruns must preserve inputs, deterministic counts and correctness evidence."""
    return {
        row_key(row): {
            **{
                key: row.get(key)
                for key in (
                    "version",
                    "input_sha",
                    "threads",
                    "window",
                    "method",
                    "collection_signature",
                    "status",
                    "gated",
                    "qualification_required",
                    "perturbations",
                )
            },
            **{
                arm: {
                    key: row.get(arm, {}).get(key)
                    for key in (
                        "ir_per_step",
                        "allocs_per_step",
                        "bytes_per_step",
                        "guards",
                        "max_penetration",
                        "checkpoints",
                        "time_advanced",
                        "allocs",
                        "bytes",
                        "cases",
                        "micro_instrumented",
                        "est_cycles_per_step",
                    )
                }
                for arm in (("parent", "head") if include_parent else ("head",))
            },
        }
        for row in record["results"]
    }


def head_fingerprints(record: dict) -> dict:
    """Smoke-derived rows can use a different harness from the A/B run."""
    return {
        row_key(row): row["head_env"]["fingerprint"]
        for row in record["results"]
        if "head_env" in row
    }


def chart_data(pages: Path, record: dict) -> None:
    """Use the existing stock github-action-benchmark page and data format."""
    directory = pages / "performance/dart6-ir"
    directory.mkdir(parents=True, exist_ok=True)
    index = directory / "index.html"
    if not index.exists():
        shutil.copyfile(pages / "performance/dart6/index.html", index)
    prefix = "window.BENCHMARK_DATA = "
    data_path = directory / "data.js"
    if data_path.exists():
        script = data_path.read_text(encoding="utf-8")
        if not script.startswith(prefix):
            raise ValueError("unrecognized deterministic chart data")
        data = json.loads(script[len(prefix) :])
    else:
        data = {
            "lastUpdate": 0,
            "repoUrl": "https://github.com/dartsim/dart",
            "entries": {},
        }
    series = data["entries"].setdefault("DART 6 deterministic counts", [])
    run = record["run"]
    fingerprint = run["env"]["fingerprint"]
    row_fingerprints = head_fingerprints(record)
    benches = []
    for row in record["results"]:
        if row.get("status") != "ok":
            continue
        head_fingerprint = row.get("head_env", run["env"])["fingerprint"]
        extra = f"fingerprint: {head_fingerprint}"
        if run.get("pr"):
            extra += f"\nPR #{run['pr']}"
        for metric, label, unit in (
            ("ir_per_step", "Ir", "instructions / step"),
            ("allocs_per_step", "allocations", "allocations / step"),
        ):
            value = row["head"].get(metric)
            if value is not None:
                # Inputs split series; other continuity changes are annotated below.
                benches.append(
                    {
                        "name": f"{row_key(row)}@{row['version']}:{(row.get('input_sha') or 'unknown')[:8]} {label}",
                        "value": value,
                        "unit": unit,
                        "extra": extra,
                        "fingerprint": head_fingerprint,
                        "micro_instrumented": row["head"].get("micro_instrumented"),
                        **{
                            key: row.get(key)
                            for key in (
                                "input_sha",
                                "threads",
                                "window",
                                "method",
                                "collection_signature",
                            )
                        },
                    }
                )
    measurement = deterministic_measurements(record)
    repeated = False
    for point in series:
        if (
            point["commit"]["id"] != run["commit"]
            or point.get("fingerprint") != fingerprint
            or point.get("head_fingerprints", {}) != row_fingerprints
        ):
            continue
        if "measurement" in point:
            changed = point["measurement"] != measurement
        else:
            # Older stock data has only plotted counts; validate what it retained.
            changed = {
                bench["name"]: (bench["value"], bench["unit"])
                for bench in point["benches"]
            } != {bench["name"]: (bench["value"], bench["unit"]) for bench in benches}
        if changed:
            raise ValueError(
                "repeated merge changed deterministic counts or guards/inputs under the same environment fingerprint"
            )
        repeated = True
    if repeated or not benches:
        return
    timestamp = int(datetime.fromisoformat(run["time"]).timestamp() * 1000)
    series.append(
        {
            "commit": {
                "id": run["commit"],
                "message": run.get("describe", run["commit"]),
                "timestamp": run["time"],
                "committer": {"username": "github-actions[bot]"},
                "url": f"{data['repoUrl']}/commit/{run['commit']}",
            },
            "date": timestamp,
            "tool": "customSmallerIsBetter",
            "fingerprint": fingerprint,
            "head_fingerprints": row_fingerprints,
            "measurement": measurement,
            "benches": benches,
        }
    )
    series.sort(
        key=lambda point: (
            datetime.fromisoformat(point["commit"]["timestamp"]).timestamp(),
            point["commit"]["id"],
            point.get("fingerprint", ""),
            tuple(sorted(point.get("head_fingerprints", {}).items())),
        )
    )
    previous_benches = {}
    continuity_fields = (
        "threads",
        "window",
        "method",
        "collection_signature",
        "micro_instrumented",
    )
    for point in series:
        for bench in point["benches"]:
            bench["extra"] = re.sub(
                r"\n(?:fingerprint|input_sha|threads|window|method|collection_signature|micro_instrumented) changed: [^\n]*",
                "",
                bench.get("extra", ""),
            )
            previous = previous_benches.get(bench["name"])
            if previous:
                old_point, old_bench = previous
                for key in ("fingerprint", *continuity_fields):
                    old, new = old_bench, bench
                    if key == "fingerprint":
                        old = old_bench if key in old_bench else old_point
                        new = bench if key in bench else point
                    if old.get(key) != new.get(key):
                        bench["extra"] += (
                            f"\n{key} changed: {old.get(key, 'unknown')}"
                            f" -> {new.get(key, 'unknown')}"
                        )
            previous_benches[bench["name"]] = (point, bench)
    data["entries"]["DART 6 deterministic counts"] = series[-250:]
    data["lastUpdate"] = series[-1]["date"]
    # The stock page reads this as executable JS; JSON escaping closes script literals.
    data_path.write_text(
        prefix
        + json.dumps(data, indent=2, allow_nan=False).replace("<", "\\u003c")
        + "\n",
        encoding="utf-8",
    )


def load_records(paths) -> dict:
    """Read a comparison history, detecting conflicts before choosing duplicates."""
    records, histories = {}, {}
    if isinstance(paths, Path):
        paths = [paths]
    files = sorted(
        {
            file
            for path in paths
            for file in (path.rglob("*.json") if path.is_dir() else [path])
        }
    )
    for path in files:
        try:
            if path.stat().st_size > 16 * 1024 * 1024:
                raise ValueError("record exceeds 16 MiB")
            raw = json.loads(path.read_text(encoding="utf-8"))
            comparison = isinstance(raw, dict) and "verdict" in raw
            record = read_record(path, comparison=comparison)
            run = record["run"]
            if run.get("tier") not in ("merge", "backfill"):
                continue
            if not comparison:
                raise ValueError("history requires a completed comparison")
            commit = run["commit"]
            fingerprint = run["env"]["fingerprint"]
            if not re.fullmatch(r"[0-9a-f]{40}", commit) or not re.fullmatch(
                r"[0-9a-f]{64}", fingerprint
            ):
                raise ValueError("invalid history commit or fingerprint")
            if not re.fullmatch(r"[0-9a-f]{40}", run.get("parent", "")):
                raise ValueError("invalid history parent commit")
            measured = datetime.fromisoformat(run["time"].replace("Z", "+00:00"))
            if measured.tzinfo is None:
                raise ValueError("history time must include its timezone")
            source = run["env"]["runner"]["environment"]
            if source not in ("local", "github-hosted"):
                raise ValueError("invalid history measurement runner")
            histories.setdefault(commit, []).append((path, record, source, measured))
        except (OSError, ValueError, KeyError, TypeError) as error:
            raise ValueError(f"{path.name}: {error}") from error
    for commit, history in histories.items():
        hosted = any(source != "local" for _, _, source, _ in history)
        identities = {}
        ranked = []
        for path, record, source, measured in history:
            if hosted and source == "local":
                continue
            run = record["run"]
            identity = (run["env"]["fingerprint"], run["tier"], source)
            counts = (
                run["parent"],
                deterministic_measurements(record, include_parent=True),
            )
            if identity in identities:
                previous_path, previous_counts = identities[identity]
                if previous_counts != counts:
                    raise ValueError(
                        "conflicting deterministic measurements in "
                        f"{previous_path.name} and {path.name}"
                    )
            else:
                identities[identity] = (path, counts)
            ranked.append((run["tier"] == "merge", measured, record))
        records[commit] = max(ranked, key=lambda candidate: candidate[:2])[2]
    return records


def ledger_entries(
    records: dict, since: str, until: str, intent: Path | None = None
) -> dict:
    """Attribute comparison policy to commits in first-parent order."""

    def git(*arguments):
        return command_output(["git", "-C", str(ROOT), *arguments])

    commits = git(
        "rev-list", "--first-parent", "--reverse", until, f"^{since}"
    ).splitlines()
    measured = set(
        git(
            "rev-list", "--first-parent", until, f"^{since}", "--", *MEASURED_PATHS
        ).splitlines()
    )
    intentions = {}
    if intent is not None:
        for line in intent.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            columns = line.split("\t")
            if (
                len(columns) != 3
                or columns[1] not in ("perf", "behaviour", "unrelated")
                or not columns[2].strip()
            ):
                raise ValueError(f"{intent.name}: invalid intent row")
            key, value, reason = columns
            if re.fullmatch(r"#[1-9]\d*", key):
                matches = [
                    commit
                    for commit in commits
                    if records.get(commit, {}).get("run", {}).get("pr") == int(key[1:])
                ]
            elif re.fullmatch(r"[0-9a-f]{7,40}", key):
                matches = [commit for commit in commits if commit.startswith(key)]
                if len(matches) != 1:
                    matches = []
            else:
                matches = []
            if not matches:
                raise ValueError(
                    f"{intent.name}: unknown or ambiguous intent key {key}"
                )
            for commit in matches:
                intentions[commit] = (value, reason)

    entries = []
    for commit in commits:
        if commit not in records:
            continue
        record = records[commit]
        run, verdict = record["run"], record["verdict"]
        rows = {row_key(row): row for row in record["results"]}
        attributable, inherited = [], []
        for failure in verdict["failures"]:
            key, separator, reason = failure.partition(": ")
            row = rows.get(key)
            parent = row.get("parent", {}) if row else None
            base_reason = separator and reason in (
                "missing or failed base measurement",
                "base state is non-finite",
                "base perturbation check failed",
            )
            same_defect = False
            if row is not None and row.get("parent_status") != "unsupported":
                head = row.get("head", {})
                if reason == "head perturbation check failed":
                    same_defect = (
                        f"{key}: base perturbation check failed" in verdict["failures"]
                    )
                elif row["delta"]["class"] == "broken" and reason in row["failures"]:
                    same_defect = (
                        (
                            not parent
                            and not head
                            # shortcut: legacy empty bases lack status, rerun to distinguish unsupported rows.
                            and row.get("parent_status", "broken") == "broken"
                        )
                        or (
                            reason
                            in (
                                "non-finite state",
                                "missing or failed head measurement",
                            )
                            and (parent.get("guards") or {}).get("finite") is False
                            and (
                                (head.get("guards") or {}).get("finite") is False
                                or reason == "non-finite state"
                            )
                        )
                        or (
                            reason == "simulation time did not advance"
                            and parent.get("time_advanced") is False
                            and (parent.get("guards") or {}).get("finite") is not False
                        )
                    )
            (inherited if base_reason or same_defect else attributable).append(failure)
        rules, nonwaivable = set(), []
        for failure in attributable:
            rule = next(
                (
                    rule
                    for rule, _, pattern in WAIVABLE_FAILURES
                    if re.fullmatch(pattern, failure)
                ),
                None,
            )
            if rule is None:
                nonwaivable.append(failure)
                continue
            rules.add(rule)
        classification = (
            "NO-BASE"
            if verdict["failures"] and not attributable
            else (
                "BROKEN"
                if nonwaivable
                else (
                    "NEEDS-RATIONALE"
                    if attributable
                    else (
                        "WARN"
                        if any(
                            not warning.endswith("(advisory)")
                            for warning in verdict["warnings"]
                        )
                        else "PASS"
                    )
                )
            )
        )
        listed = []
        for key, row in rows.items():
            change = row_change(row)
            if change in ("regressed", "improved", "behaviour-change", "broken", "new"):
                listed.append(
                    {
                        "row": key,
                        "class": row["delta"]["class"],
                        "change": change,
                        **{
                            name: row["delta"].get(name)
                            for name in ("ir", "allocs", "bytes", "guards_equal")
                        },
                    }
                )
        groups = set()
        for path in git(
            "diff", "--name-only", run["parent"], commit, "--", *MEASURED_PATHS
        ).splitlines():
            parts = path.split("/")
            if parts[-1] == "CMakeLists.txt" or path.startswith("cmake/"):
                groups.add("cmake")
            elif path.startswith("dart/collision/"):
                groups.add(
                    "collision/"
                    + (
                        parts[2]
                        if len(parts) > 3 and parts[2] in DETECTORS
                        else "other"
                    )
                )
            elif path.startswith("dart/"):
                groups.add(parts[1] if len(parts) > 2 else "dart/other")
            else:
                groups.add("workload")
        value, reason = intentions.get(commit, (None, ""))
        entries.append(
            {
                "commit": commit,
                "pr": run.get("pr"),
                "tier": run["tier"],
                "parent": run["parent"],
                "class": classification,
                "rules": sorted(rules),
                "rows": listed,
                "groups": sorted(groups),
                "ir_geomean": verdict["ir_geomean"],
                "failures": nonwaivable,
                "inherited": inherited,
                "accepted": run.get("accepted", []),
                "intent": value,
                "reason": reason,
            }
        )
    denominator = [
        entry
        for entry in entries
        if entry["pr"] and entry["intent"] not in ("perf", "behaviour")
    ]
    headline = {
        "k": sum(
            entry["intent"] == "unrelated"
            and entry["class"] != "BROKEN"
            and (entry["class"] == "NEEDS-RATIONALE" or bool(entry["accepted"]))
            for entry in denominator
        ),
        "n": len(denominator),
        "broken": sum(entry["class"] == "BROKEN" for entry in entries),
        "rules": {
            rule: sum(rule in entry["rules"] for entry in entries)
            for rule in (
                "ir",
                "geomean",
                "allocs",
                "bytes",
                "guards",
                "input",
                "percent",
            )
        },
        "needing_intent": sum(
            not entry["intent"]
            and (
                entry["class"] != "PASS"
                or bool(entry["rows"])
                or bool(entry["accepted"])
            )
            for entry in entries
        ),
    }
    return {
        "since": since,
        "until": until,
        "missing": [
            commit for commit in commits if commit in measured and commit not in records
        ],
        "entries": entries,
        "headline": headline,
    }


def ledger_markdown(report: dict) -> str:
    headline = report["headline"]
    entries = report["entries"]
    lines = [
        "# DART performance ledger",
        "",
        f"Range: `{report['since']}` → `{report['until']}`; {len(entries)} records "
        f"({sum(entry['tier'] == 'merge' for entry in entries)} merge, "
        f"{sum(entry['tier'] == 'backfill' for entry in entries)} backfill).",
        "Missing: "
        + (", ".join(f"`{commit[:12]}`" for commit in report["missing"]) or "none")
        + ".",
        "",
        f"Unrelated merges needing a rationale: {headline['k']}/{headline['n']} "
        f"(target: at most 1 in 10). Broken: {headline['broken']}. Rules: "
        + ", ".join(f"{rule} {count}" for rule, count in headline["rules"].items())
        + f". Needing an intent: {headline['needing_intent']}.",
        "",
        "| Path group | PASS | WARN | NEEDS-RATIONALE | BROKEN | NO-BASE |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for group in sorted({group for entry in entries for group in entry["groups"]}):
        values = [
            group,
            *(
                sum(
                    group in entry["groups"] and entry["class"] == kind
                    for entry in entries
                )
                for kind in ("PASS", "WARN", "NEEDS-RATIONALE", "BROKEN", "NO-BASE")
            ),
        ]
        lines.append("| " + " | ".join(map(markdown_cell, values)) + " |")
    for title, intended in (("Unrelated or unlabelled", False), ("Intended", True)):
        lines += [
            "",
            f"## {title}",
            "",
            "| Commit | PR | Class | Rules | Rows | Ir geomean | Paths | Accepted | Intent |",
            "|---|---|---|---|---|---|---|---|---|",
        ]
        for entry in entries:
            if (entry["intent"] in ("perf", "behaviour")) != intended:
                continue
            values = [
                f"`{entry['commit'][:12]}`",
                f"#{entry['pr']}" if entry["pr"] else "—",
                entry["class"],
                ", ".join(entry["rules"]) or "—",
                ", ".join(
                    f"{row['row']}: {row['change']} ({percent(row['ir'])})"
                    for row in entry["rows"]
                )
                or "—",
                percent(entry["ir_geomean"]),
                ", ".join(entry["groups"]) or "—",
                "; ".join(item["rationale"] for item in entry["accepted"]) or "—",
                f"{entry['intent']}: {entry['reason']}" if entry["intent"] else "—",
            ]
            lines.append("| " + " | ".join(map(markdown_cell, values)) + " |")
    return "\n".join(lines) + "\n"


def release_markdown(record: dict) -> str:
    run = record["run"]
    source = "local" if run["env"]["runner"]["environment"] == "local" else "hosted"
    lines = [
        f"# Perf release `{run['base_tag']}` → `{run['tag']}` (`{run['branch']}`)",
        "",
        f"`{run['parent'][:12]}` → `{run['commit'][:12]}`; source: {source}.",
        "",
        markdown(record).rstrip(),
        "",
        "## Ledger",
        "",
    ]
    if run["branch"].startswith("release-6."):
        lines.append(f"{run['branch']} is tracked by tags only.")
    else:
        ledger = record["ledger"]
        lines += [
            "Missing: "
            + (", ".join(f"`{commit[:12]}`" for commit in ledger["missing"]) or "none")
            + ".",
            "",
            "| Commit | PR | Class | Rules | Rows | Ir geomean | Paths | Accepted |",
            "|---|---|---|---|---|---|---|---|",
        ]
        for entry in ledger["entries"]:
            values = [
                f"`{entry['commit'][:12]}`",
                f"#{entry['pr']}" if entry["pr"] else "—",
                entry["class"],
                ", ".join(entry["rules"]) or "—",
                ", ".join(
                    f"{row['row']}: {row['change']} ({percent(row['ir'])})"
                    for row in entry["rows"]
                )
                or "—",
                percent(entry["ir_geomean"]),
                ", ".join(entry["groups"]) or "—",
                "; ".join(item["rationale"] for item in entry["accepted"]) or "—",
            ]
            lines.append("| " + " | ".join(map(markdown_cell, values)) + " |")
    lines += ["", f"[JSON record]({run['tag']}.json)"]
    return "\n".join(lines) + "\n"


def release_index(pages: Path) -> str:
    records = [
        read_record(path, comparison=True)
        for path in (pages / "performance/releases").glob("*.json")
    ]
    records.sort(
        key=lambda record: tuple(map(int, record["run"]["tag"][1:].split("."))),
        reverse=True,
    )
    lines = [
        "# DART release performance",
        "",
        "| Tag | Base | Branch | Verdict | Row | Ir/step | ΔIr | Allocs/step | Guards | Fingerprint | Source | Measured (UTC) |",
        "|---|---|---|---|---|---:|---:|---:|---|---|---|---|",
    ]
    for record in records:
        run = record["run"]
        source = "local" if run["env"]["runner"]["environment"] == "local" else "hosted"
        for row in record["results"]:
            head, change = row["head"], row["delta"]
            equal = change.get("guards_equal")
            guard = "unavailable" if equal is None else "same" if equal else "changed"
            guard_hash = (head.get("guards") or {}).get("hash")
            if guard_hash:
                if not isinstance(guard_hash, str):
                    guard_hash = json.dumps(guard_hash, sort_keys=True)
                guard += f", {guard_hash[:12]}"
            values = [
                f"[{run['tag']}]({run['tag']}.md)",
                run["base_tag"],
                run["branch"],
                record["verdict"]["status"],
                row_key(row),
                head.get("ir_per_step", "—"),
                percent(change.get("ir")),
                head.get("allocs_per_step", "—"),
                guard,
                run["env"]["fingerprint"][:8],
                source,
                datetime.fromisoformat(run["time"].replace("Z", "+00:00"))
                .astimezone(timezone.utc)
                .isoformat(),
            ]
            lines.append("| " + " | ".join(map(markdown_cell, values)) + " |")
    return "\n".join(lines) + "\n"


def write_release(pages: Path, record: dict) -> list[str]:
    run = record["run"]
    directory = pages / "performance/releases"
    path = directory / f"{run['tag']}.json"
    tagged = subprocess.run(
        [
            "git",
            "-C",
            str(ROOT),
            "rev-parse",
            "--verify",
            "--end-of-options",
            f"{run['tag']}^{{commit}}",
        ],
        text=True,
        capture_output=True,
    )
    tagged_commit = tagged.stdout.strip() if tagged.returncode == 0 else None
    saved = read_record(path, comparison=True) if path.exists() else None
    if tagged_commit is not None and tagged_commit != run["commit"]:
        if saved is not None and saved["run"]["commit"] == tagged_commit:
            print(f"Keep {run['tag']}: tagged commit is final")
        else:
            print(f"Skip {run['tag']}: tag does not name the candidate commit")
        return []
    if saved is not None:
        previous = saved["run"]
        previous_source = previous["env"]["runner"]["environment"]
        incoming_source = run["env"]["runner"]["environment"]
        previous_local = previous_source == "local"
        incoming_local = incoming_source == "local"
        if not previous_local and incoming_local:
            print(f"Keep {run['tag']}: hosted measurements take precedence")
            return []
        identity = ("commit", "parent")
        same = (
            all(previous[key] == run[key] for key in identity)
            and previous["env"]["fingerprint"] == run["env"]["fingerprint"]
            and previous_source == incoming_source
        )
        if same:
            if deterministic_measurements(
                saved, include_parent=True
            ) != deterministic_measurements(record, include_parent=True):
                raise ValueError(
                    "repeated release changed deterministic counts or guards/inputs under the same environment fingerprint"
                )
            print(f"Keep {run['tag']}: identical deterministic measurements")
            return []
        if not (previous_local and not incoming_local):
            if tagged_commit == previous["commit"]:
                print(f"Keep {run['tag']}: tagged commit is final")
                return []
            if tagged_commit != run["commit"]:
                descendant = previous["commit"] != run["commit"] and is_ancestor(
                    previous["commit"], run["commit"]
                )
                later = previous["commit"] == run["commit"] and datetime.fromisoformat(
                    run["time"].replace("Z", "+00:00")
                ) > datetime.fromisoformat(previous["time"].replace("Z", "+00:00"))
                if not descendant and not later:
                    print(f"Keep {run['tag']}: older or diverged candidate")
                    return []
    if run["branch"].startswith("release-6."):
        ledger = {"entries": [], "missing": []}
    else:
        history = pages / "performance/records/main"
        report = ledger_entries(
            load_records([history]) if history.exists() else {},
            run["parent"],
            run["commit"],
        )
        ledger = {
            "entries": [
                entry
                for entry in report["entries"]
                if entry["accepted"] or entry["class"] != "PASS" or entry["rows"]
            ],
            "missing": report["missing"],
        }
    record = {**record, "ledger": ledger}
    directory.mkdir(parents=True, exist_ok=True)
    write_json(path, record)
    markdown_path = path.with_suffix(".md")
    markdown_path.write_text(release_markdown(record), encoding="utf-8")
    index = directory / "index.md"
    index.write_text(release_index(pages), encoding="utf-8")
    return [str(file.relative_to(pages)) for file in (path, markdown_path, index)]


def write_publication(pages: Path, record: dict) -> list[str]:
    run = record["run"]
    if run["tier"] == "release":
        return write_release(pages, record)
    changed = []
    records = pages / "performance/records/main"
    repeated = False
    chart_repeated = False
    tiers = (
        ("merge", "backfill")
        if run["tier"] in ("merge", "backfill")
        else (run["tier"],)
    )
    history = [
        json.loads(path.read_text(encoding="utf-8"))
        for path in sorted(
            {path for tier in tiers for path in records.glob(f"*/*-{tier}.json")},
            reverse=True,
        )
    ]
    incoming_source = run["env"]["runner"]["environment"]
    if incoming_source == "local" and any(
        saved["run"]["commit"] == run["commit"]
        and saved["run"]["env"]["runner"]["environment"] != "local"
        for saved in history
    ):
        print(f"Keep {run['commit'][:12]}: hosted measurements take precedence")
        return []
    for index, saved in enumerate(history):
        previous = saved["run"]
        if (
            previous["tier"] != run["tier"]
            or previous["env"]["runner"]["environment"] != incoming_source
        ):
            continue
        if (
            previous["commit"],
            previous["env"]["fingerprint"],
        ) == (
            run["commit"],
            run["env"]["fingerprint"],
        ) and head_fingerprints(saved) == head_fingerprints(record):
            if run["tier"] in ("merge", "backfill"):
                if previous.get("parent") != run.get("parent") or (
                    deterministic_measurements(saved, include_parent=True)
                    != deterministic_measurements(record, include_parent=True)
                ):
                    raise ValueError(
                        "repeated merge changed deterministic counts or guards/inputs under the same "
                        "environment fingerprint"
                    )
                chart_repeated = True
                repeated = repeated or (
                    saved["run"].get("accepted", []) == run.get("accepted", [])
                    and saved.get("verdict") == record.get("verdict")
                )
            else:
                repeated = saved["results"] == record["results"] and (
                    previous["time"] == run["time"] or index == 0
                )
                if repeated:
                    break
    if run["tier"] == "nightly":
        table = pages / "performance/guards/main.md"
        table.parent.mkdir(parents=True, exist_ok=True)
        previous_table = table.read_text(encoding="utf-8") if table.exists() else ""
        if nightly_table_can_advance(record, previous_table):
            table.write_text(guard_table(record, previous_table), encoding="utf-8")
            changed.append(str(table.relative_to(pages)))
    date = datetime.fromisoformat(run["time"])
    path = (
        records
        / f"{date.year:04d}"
        / f"{date.strftime('%Y-%m-%dT%H%M%S%fZ')}-{run['commit'][:12]}-{run['tier']}.json"
    )
    if not repeated:
        # A live rationale may change while the measurement's timestamp stays fixed.
        while path.exists():
            date += timedelta(microseconds=1)
            path = (
                records
                / f"{date.year:04d}"
                / f"{date.strftime('%Y-%m-%dT%H%M%S%fZ')}-{run['commit'][:12]}-{run['tier']}.json"
            )
        path.parent.mkdir(parents=True, exist_ok=True)
        write_json(path, record)
        changed.append(str(path.relative_to(pages)))
        if run["tier"] == "merge" and incoming_source != "local" and not chart_repeated:
            chart_data(pages, record)
            changed.append("performance/dart6-ir")
    return changed


def publish(args) -> bool:
    publication_guard(args.tier)
    requested = [args.record] if isinstance(args.record, Path) else args.record
    files = sorted(
        {
            file
            for path in requested
            for file in (path.rglob("*.json") if path.is_dir() else [path])
        }
    )
    if not files:
        raise ValueError("publication requires at least one record")
    if len(files) != 1 and args.tier != "backfill":
        raise ValueError("only backfill publication accepts multiple records")
    records = [
        publication_record(
            path,
            args.tier,
            args.pr,
            getattr(args, "tag", None),
            getattr(args, "base_tag", None),
        )
        for path in files
    ]
    if (
        args.tier == "backfill"
        and len({record["run"]["env"]["fingerprint"] for record in records}) != 1
    ):
        raise ValueError("backfill publication requires one environment fingerprint")
    records.sort(key=lambda record: record["run"]["tier"] == "release")
    release_refs = sorted(
        {
            f"refs/tags/{record['run']['tag']}"
            for record in records
            if record["run"]["tier"] == "release"
        }
    )
    pages = args.pages_dir.resolve()

    def git(*arguments: str, check: bool = True):
        return subprocess.run(
            ["git", "-C", str(pages), *arguments],
            check=check,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    if git("status", "--porcelain").stdout:
        raise ValueError("publication requires a clean dedicated gh-pages checkout")
    if git("branch", "--show-current").stdout.strip() != "gh-pages":
        raise ValueError("publication requires a dedicated gh-pages checkout")
    if args.tier != "backfill":
        git("config", "user.name", "github-actions[bot]")
        git(
            "config",
            "user.email",
            "41898282+github-actions[bot]@users.noreply.github.com",
        )
    for attempt in range(5):
        git("fetch", "origin", "gh-pages")
        if release_refs:
            remote_tags = command_output(
                ["git", "ls-remote", "--refs", "origin", *release_refs]
            )
            present = [line.split()[1] for line in remote_tags.splitlines()]
            if present:
                subprocess.run(
                    [
                        "git",
                        "-C",
                        str(ROOT),
                        "fetch",
                        "--no-tags",
                        "origin",
                        *(f"{ref}:{ref}" for ref in present),
                    ],
                    check=True,
                    text=True,
                    capture_output=True,
                )
        if (
            attempt == 0
            and git(
                "merge-base", "--is-ancestor", "HEAD", "origin/gh-pages", check=False
            ).returncode
        ):
            raise ValueError("gh-pages checkout has unpublished commits")
        # Regenerate even when Git could replay our commit without a conflict:
        # concurrent records can change deduplication and derived table/chart state.
        git("checkout", "-B", "gh-pages", "origin/gh-pages")
        with tempfile.TemporaryDirectory() as temporary:
            staged = Path(temporary) / "pages"
            shutil.copytree(pages, staged, ignore=shutil.ignore_patterns(".git"))
            paths = list(
                dict.fromkeys(
                    path
                    for record in records
                    for path in write_publication(staged, record)
                )
            )
            for path in paths:
                source, destination = staged / path, pages / path
                if source.is_dir():
                    shutil.copytree(source, destination, dirs_exist_ok=True)
                else:
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(source, destination)
        for path in paths:
            print(path)
        if paths:
            git("add", "--", *paths)
        if git("diff", "--cached", "--quiet", check=False).returncode:
            subject = (
                f"Record DART backfill performance for "
                f"{sum(record['run']['tier'] == 'backfill' for record in records)} commits and "
                f"{sum(record['run']['tier'] == 'release' for record in records)} tags"
                if args.tier == "backfill"
                else f"Record DART {args.tier} performance for {records[0]['run']['commit'][:12]}"
            )
            git("commit", "-m", subject)
        if (
            git("rev-parse", "HEAD").stdout
            == git("rev-parse", "origin/gh-pages").stdout
        ):
            return False
        pushed = git("push", "origin", "HEAD:refs/heads/gh-pages", check=False)
        if pushed.returncode == 0:
            return True
    raise RuntimeError(
        f"gh-pages push rejected after 5 fetch/regenerate attempts: {pushed.stderr.strip()}"
    )


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    sub = result.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run", help="measure an installed arm")
    run.add_argument("--prefix", type=Path, required=True)
    run.add_argument("--bin-dir", type=Path)
    run.add_argument("--commit", required=True, help="revision installed in --prefix")
    run.add_argument("--source-dir", type=Path, default=ROOT)
    local = sub.add_parser("local", help="build revisions and compare them")
    local.add_argument("--base", default="origin/main")
    local.add_argument("--head", default="HEAD")
    local.add_argument(
        "--smoke", action="store_true", help="build and measure only --head"
    )
    for item in (run, local):
        item.add_argument(
            "--output-dir",
            type=Path,
            required=item is run,
            default=ROOT / "build/perf-compare",
        )
        item.add_argument("--rows", default="")
        item.add_argument("--jobs", type=int, choices=range(1, 9), default=4)
        item.add_argument("--timeout", type=int, default=900)
        item.add_argument("--native-only", action="store_true")
        # Rows gate only on this run's heap-layout checks; --no-perturb skips
        # them and leaves every row diagnostic.
        item.add_argument(
            "--perturb", action=argparse.BooleanOptionalAction, default=True
        )
        item.add_argument("--cache-sim", action="store_true")
        item.add_argument(
            "--nightly",
            action="store_true",
            help="include canonical S1-S6 guards and mf",
        )
        item.add_argument("--shim", type=Path)
        item.add_argument("--heappad", type=Path)
    for item in (sub.add_parser("compare", help="judge saved measurements"), local):
        if item is not local:
            item.add_argument("--base", type=Path, required=True)
            item.add_argument("--head", type=Path, required=True)
        item.add_argument("--body-file", type=Path)
        item.add_argument("--json", type=Path)
        item.add_argument("--markdown", type=Path)
    publication = sub.add_parser(
        "publish", help="publish trusted performance records to gh-pages"
    )
    publication.add_argument("--record", type=Path, nargs="+", required=True)
    publication.add_argument(
        "--tier", choices=("merge", "nightly", "release", "backfill"), required=True
    )
    publication.add_argument("--pages-dir", type=Path, required=True)
    publication.add_argument("--pr", type=int)
    publication.add_argument("--tag")
    publication.add_argument("--base-tag")
    backfill_cli = sub.add_parser("backfill", help="measure and resume a revision list")
    backfill_cli.add_argument("--revs", type=Path, required=True)
    backfill_cli.add_argument(
        "--output-dir", type=Path, default=ROOT / "build/perf-backfill"
    )
    backfill_cli.add_argument(
        "--rows", default="s3w,s2r,s1p,s5a,pend,gzb,robot,dyn,lcp,mt4-s3w,mt4-s1p,S6"
    )
    backfill_cli.add_argument("--jobs", type=int, default=os.cpu_count() or 1)
    backfill_cli.add_argument("--timeout", type=int, default=900)
    backfill_cli.add_argument(
        "--plan-only", action="store_true", help="list revisions without building"
    )
    backfill_cli.set_defaults(
        nightly=False,
        cache_sim=False,
        native_only=False,
        perturb=True,
        shim=None,
        heappad=None,
    )
    ledger_cli = sub.add_parser("ledger", help="report changes and rationale friction")
    ledger_cli.add_argument("--records", type=Path, nargs="+", required=True)
    ledger_cli.add_argument("--since", required=True)
    ledger_cli.add_argument("--until", default="origin/main")
    ledger_cli.add_argument("--intent", type=Path)
    ledger_cli.add_argument("--json", type=Path)
    ledger_cli.add_argument("--markdown", type=Path)
    return result


def has_contact_driver(revision: str) -> bool:
    return (
        subprocess.run(
            [
                "git",
                "cat-file",
                "-e",
                f"{revision}:examples/contact_benchmark/CMakeLists.txt",
            ],
            cwd=ROOT,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        ).returncode
        == 0
    )


def build_shims(args, shims: Path) -> None:
    shims.mkdir(parents=True, exist_ok=True)
    compiler = "/usr/bin/cc"
    compiler_sha = sha(Path(compiler).resolve(strict=True).read_bytes())
    options = ["-O2", "-shared", "-fPIC"]
    for name in ("allocshim", "heappad"):
        source = ROOT / f"tools/perf/{name}.c"
        binary, stamp = shims / f"{name}.so", shims / f"{name}.sha256"
        identity = sha(
            json.dumps(
                [sha(source.read_bytes()), compiler_sha, options, "-ldl"]
            ).encode()
        )
        if (
            binary.is_file()
            and stamp.is_file()
            and stamp.read_text(encoding="utf-8", errors="replace").strip() == identity
        ):
            continue
        stamp.unlink(missing_ok=True)
        binary.unlink(missing_ok=True)
        execute(
            [
                compiler,
                *options,
                "-o",
                str(binary),
                str(source),
                "-ldl",
            ],
            os.environ.copy(),
            shims / f"{name}.log",
            args.timeout,
            build=True,
        )
        stamp.write_text(identity + "\n", encoding="utf-8")


def install_targets(build: Path) -> list[str]:
    """Find configured targets required by install without building the ALL graph."""
    reply = build / ".cmake/api/v1/reply"
    try:
        index_path = max(reply.glob("index-*.json"))
        index = json.loads(index_path.read_text())
        model = json.loads(
            (reply / index["reply"]["codemodel-v2"]["jsonFile"]).read_text()
        )
        targets = [
            json.loads((reply / target["jsonFile"]).read_text())
            for target in model["configurations"][0]["targets"]
        ]
        return sorted(target["name"] for target in targets if "install" in target)
    except (OSError, ValueError, KeyError, IndexError, TypeError) as error:
        raise ValueError(
            "cannot read install targets from the CMake File API"
        ) from error


def build_arm(args, revision, source, build, driver_build, prefix, drivers, log_prefix):
    dependency = os.environ.get("CONDA_PREFIX")
    if not dependency:
        raise ValueError("building requires the active Pixi environment (CONDA_PREFIX)")
    options = [
        "-DCMAKE_BUILD_TYPE=Release",
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
        f"-DCMAKE_INSTALL_PREFIX={prefix}",
        f"-DCMAKE_PREFIX_PATH={dependency}",
        "-DCMAKE_CXX_COMPILER=/usr/bin/c++",
        "-DCMAKE_C_COMPILER=/usr/bin/cc",
        "-DBUILD_TESTING=ON",
        "-DDART_BUILD_DARTPY=OFF",
        "-DDART_BUILD_PROFILE=OFF",
        "-DDART_ENABLE_SIMD=OFF",
        "-DDART_BUILD_GUI_OSG=ON",
        "-DDART_USE_SYSTEM_GOOGLEBENCHMARK=ON",
        "-DDART_USE_SYSTEM_GOOGLETEST=ON",
        "-DDART_USE_SYSTEM_IMGUI=ON",
        "-DDART_USE_SYSTEM_TRACY=ON",
        "-DDART_TREAT_WARNINGS_AS_ERRORS=OFF",
        "-DDART_DISABLE_COMPILER_CACHE=ON",
    ]
    query = build / ".cmake/api/v1/query/codemodel-v2"
    query.parent.mkdir(parents=True, exist_ok=True)
    query.touch()
    execute(
        [
            "cmake",
            "-G",
            "Ninja",
            "--fresh",
            "-S",
            str(source),
            "-B",
            str(build),
            *options,
        ],
        os.environ.copy(),
        Path(f"{log_prefix}.configure.log"),
        args.timeout,
        build=True,
    )
    workload_sources = workload_hashes(source, drivers, build, prefix)
    targets = ["dart-utils-urdf", *drivers]
    if CB not in drivers:
        targets += [
            "dart-collision-ode",
            "dart-collision-bullet",
            "dart-gui-osg",
        ]
    # Historical libraries share one install component, including optional ones.
    targets = sorted(set(targets) | set(install_targets(build)))
    execute(
        [
            "cmake",
            "--build",
            str(build),
            "--parallel",
            str(args.jobs),
            "--target",
            *targets,
        ],
        os.environ.copy(),
        Path(f"{log_prefix}.build.log"),
        max(args.timeout, 3600),
        build=True,
    )
    if prefix.exists():
        shutil.rmtree(prefix)
    execute(
        ["cmake", "--install", str(build), "--prefix", str(prefix)],
        os.environ.copy(),
        Path(f"{log_prefix}.install.log"),
        args.timeout,
        build=True,
    )
    binary = prefix / "bin"
    binary.mkdir(exist_ok=True)
    for target in targets:
        if (build / "bin" / target).is_file():
            shutil.copy2(build / "bin" / target, binary / target)
    execute(
        [
            "cmake",
            "-G",
            "Ninja",
            "--fresh",
            "-S",
            str(ROOT / "tools/perf"),
            "-B",
            str(driver_build),
            "-DCMAKE_BUILD_TYPE=Release",
            "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
            f"-DCMAKE_PREFIX_PATH={prefix};{dependency}",
            "-DCMAKE_CXX_COMPILER=/usr/bin/c++",
        ],
        os.environ.copy(),
        Path(f"{log_prefix}.driver-configure.log"),
        args.timeout,
        build=True,
    )
    workload_sources |= workload_hashes(ROOT, [PB], driver_build, prefix)
    execute(
        ["cmake", "--build", str(driver_build), "--parallel", str(args.jobs)],
        os.environ.copy(),
        Path(f"{log_prefix}.driver-build.log"),
        max(args.timeout, 3600),
        build=True,
    )
    shutil.copy2(driver_build / "portable_step_bench", binary / "portable_step_bench")
    compiler = cmake_compiler(build)
    if cmake_compiler(driver_build) != compiler:
        raise ValueError("DART and portable driver compiler provenance differs")
    write_json(
        prefix / "share/dart/perf-build.json",
        {
            "schema": "dart-perf-build/1",
            "commit": revision,
            **compiler,
            "pixi_lock_sha": sha((ROOT / "pixi.lock").read_bytes()),
            "preset": "perf-1",
            "libdart_sha": sha((prefix / "lib/libdart.so").read_bytes()),
            "libraries": library_hashes(prefix),
            "workload_sources": workload_sources,
            "binaries": {
                path.name: sha(path.read_bytes())
                for path in binary.iterdir()
                if path.is_file()
            },
        },
    )


def broken_arm(rows, args, run, marker) -> dict:
    return {
        "schema": "dart-perf/1",
        "run": {
            **run,
            "commit": marker["commit"],
            "describe": command_output(
                [
                    "git",
                    "describe",
                    "--tags",
                    "--match",
                    "v6*",
                    "--always",
                    marker["commit"],
                ]
            ),
            "time": marker["time"],
        },
        "results": [
            {
                **row_result(row, args),
                "status": "broken",
                "gated": False,
                "perturbations": {},
                "error": f"build failed: {marker['error']}",
                "error_kind": "build",
                "head": {},
            }
            for row in rows
        ],
    }


def arm_namespace(args, revision, source, prefix, output, shims, base_arm=False):
    arm = argparse.Namespace(**vars(args))
    arm.nightly = False  # The caller has already selected its row set.
    arm.prefix, arm.bin_dir, arm.commit = prefix, prefix / "bin", revision
    arm.source_dir = source
    arm.base_arm = base_arm
    arm.output_dir = output
    arm.shim, arm.heappad = shims / "allocshim.so", shims / "heappad.so"
    return arm


def host_identity(args) -> dict:
    libraries = sorted(
        {
            Path(line.split()[-1]).resolve()
            for line in Path("/proc/self/maps").read_text().splitlines()
            if line.split()
            and Path(line.split()[-1]).name in {"libc.so.6", "libm.so.6"}
        }
    )
    if {path.name for path in libraries} != {"libc.so.6", "libm.so.6"}:
        raise ValueError("cannot identify the mapped glibc libraries")
    return {
        "harness_commit": command_output(["git", "rev-parse", "HEAD"]),
        "rows": args.rows,
        "valgrind": command_output([VALGRIND, "--version"]).removeprefix("valgrind-"),
        **valgrind_hashes(),
        "compiler_sha": sha(Path("/usr/bin/c++").resolve().read_bytes()),
        "glibc": command_output(["getconf", "GNU_LIBC_VERSION"]).split()[-1],
        "glibc_sha": sha(b"".join(path.read_bytes() for path in libraries)),
    }


def hosted_reference() -> dict:
    try:
        paths = command_output(
            [
                "git",
                "ls-tree",
                "-r",
                "--name-only",
                "origin/gh-pages",
                "--",
                "performance/records/main",
            ]
        ).splitlines()
        records = [
            json.loads(command_output(["git", "show", f"origin/gh-pages:{path}"]))
            for path in paths
            if path.endswith("-merge.json")
        ]
        return max(records, key=lambda record: record["run"]["time"])["run"]["env"]
    except (subprocess.CalledProcessError, ValueError, KeyError) as error:
        raise ValueError(
            "no hosted merge reference is available; run git fetch origin gh-pages"
        ) from error


def backfill_plan(args) -> dict:
    revisions, pairs, skipped = [], [], []
    for line in args.revs.read_text(encoding="utf-8").splitlines():
        revision = line.split("#", 1)[0].strip()
        if not revision:
            continue
        commit = command_output(
            ["git", "rev-parse", "--verify", f"{revision}^{{commit}}"]
        )
        tag_exists = (
            re.fullmatch(RELEASE_TAG, revision)
            and subprocess.run(
                ["git", "show-ref", "--verify", "--quiet", f"refs/tags/{revision}"],
                cwd=ROOT,
                check=False,
            ).returncode
            == 0
        )
        if tag_exists:
            scope = release_scope(None, revision, None)
            pair = {
                "commit": scope["head"],
                "parent": scope["base"],
                "branch": scope["branch"],
                "tag": scope["tag"],
                "base_tag": scope["base_tag"],
            }
        else:
            if not has_contact_driver(commit):
                raise ValueError(f"{commit[:12]} lacks examples/contact_benchmark")
            changed = command_output(
                [
                    "git",
                    "diff",
                    "--name-only",
                    f"{commit}^1",
                    commit,
                    "--",
                    *MEASURED_PATHS,
                ]
            )
            if not changed:
                skipped.append(commit)
                continue
            base = command_output(
                [
                    "git",
                    "rev-list",
                    "--first-parent",
                    "-n1",
                    f"{commit}^1",
                    "--",
                    *MEASURED_PATHS,
                ]
            )
            if not base:
                raise ValueError(f"{commit[:12]} has no measured-path base")
            pair = {
                "commit": commit,
                "parent": base,
                "branch": "main",
                "tag": None,
                "base_tag": None,
            }
        pairs.append(pair)
        for arm in (pair["parent"], pair["commit"]):
            if arm not in revisions:
                revisions.append(arm)
    return {"revisions": revisions, "pairs": pairs, "skipped": skipped}


def backfill_records(args, plan: dict, run: dict) -> list[dict]:
    output = args.output_dir.resolve()
    arms, markers = {}, {}
    for revision in plan["revisions"]:
        directory = output / "runs" / revision
        if (directory / "record.json").is_file():
            arms[revision] = read_record(directory / "record.json")
        else:
            markers[revision] = json.loads(
                (directory / "build-failure.json").read_text()
            )
    reference = next(iter(arms.values()), None)
    if reference is None and run.get("fingerprint"):
        for path in sorted((output / "runs").glob("*/record.json")):
            with contextlib.suppress(ValueError, KeyError, OSError, TypeError):
                previous = read_record(path)
                if (
                    previous["run"]["commit"] == path.parent.name
                    and previous["run"]["env"]["fingerprint"] == run["fingerprint"]
                ) and not any(
                    row.get("error_kind") == "infrastructure"
                    for row in previous["results"]
                ):
                    reference = previous
                    break
    if reference is None and plan["pairs"]:
        raise ValueError(
            "no measured arm is available to identify the build-failure records"
        )
    for revision, marker in markers.items():
        arm = argparse.Namespace(**vars(args))
        if not has_contact_driver(revision):
            arm.rows = "gzb,robot"
        arms[revision] = broken_arm(planned_rows(arm), arm, reference["run"], marker)
    records_dir = output / "records"
    if records_dir.exists():
        shutil.rmtree(records_dir)
    quick = {row.key for row in select_rows("")}
    records = []
    for pair in plan["pairs"]:
        trimmed = []
        for revision in (pair["parent"], pair["commit"]):
            arm = json.loads(json.dumps(arms[revision]))
            arm["results"] = [row for row in arm["results"] if row_key(row) in quick]
            for row in arm["results"]:
                for key in ("wall_ms_per_step", "max_rss_kb"):
                    row.get("head", {}).pop(key, None)
            trimmed.append(arm)
        record = compare(*trimmed, "")
        subject = command_output(["git", "show", "-s", "--format=%s", pair["commit"]])
        pr = re.search(r"\(#(\d+)\)$", subject)
        record["run"].update(
            tier="release" if pair["tag"] else "backfill",
            branch=pair["branch"],
            parent=pair["parent"],
            pr=None if pair["tag"] or not pr else int(pr[1]),
            harness_commit=run["identity"]["harness_commit"],
            accepted=[],
        )
        if pair["tag"]:
            record["run"].update(tag=pair["tag"], base_tag=pair["base_tag"])
            path = records_dir / "releases" / f"{pair['tag']}.json"
        else:
            path = records_dir / "main" / f"{pair['commit']}.json"
        write_json(path, record)
        records.append(record)
    return records


def backfill_git(command: list[str]) -> None:
    process = subprocess.Popen(command, cwd=ROOT)
    try:
        returncode = process.wait()
    except BaseException:
        process.terminate()
        process.wait()
        raise
    if returncode:
        raise subprocess.CalledProcessError(returncode, command)


def backfill(args) -> list[dict]:
    plan = backfill_plan(args)
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    with (output / ".lock").open("a", encoding="utf-8") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise ValueError("another backfill holds the output lock") from error
        if command_output(
            [
                "git",
                "status",
                "--porcelain",
                "--ignored",
                "--",
                "scripts/perf_regression.py",
                "tools/perf",
                "pixi.lock",
            ]
        ):
            raise ValueError("the harness checkout is dirty")
        identity = host_identity(args)
        run_path = output / "run.json"
        if run_path.exists():
            run = json.loads(run_path.read_text(encoding="utf-8"))
            if identity != run["identity"]:
                raise ValueError("the backfill run identity has changed")
        else:
            reference = hosted_reference()
            for key in (
                "valgrind",
                "valgrind_sha",
                "callgrind_sha",
                "compiler_sha",
                "glibc",
            ):
                if identity.get(key) != reference.get(key):
                    raise ValueError(f"the hosted reference differs: {key}")
            run = {
                "identity": identity,
                "reference": {
                    key: reference[key]
                    for key in ("valgrind_guest_cpu", "compiler", "preset")
                },
            }
            write_json(run_path, run)
        if (output / "records").exists():
            shutil.rmtree(output / "records")
        shims = output / "shims"
        build_shims(args, shims)
        source, build, driver, prefix = (
            output / name for name in ("src", "build", "driver", "prefix")
        )
        if source.is_symlink():
            raise ValueError("backfill source must be a dedicated linked worktree")
        if not source.exists() and plan["revisions"]:
            backfill_git(
                [
                    "git",
                    "worktree",
                    "add",
                    "--detach",
                    str(source),
                    plan["revisions"][0],
                ]
            )
        if plan["revisions"]:
            git_file = source / ".git"
            if git_file.is_symlink() or not git_file.is_file():
                raise ValueError("backfill source must be a dedicated linked worktree")
            top = command_output(
                ["git", "-C", str(source), "rev-parse", "--show-toplevel"]
            )
            common = command_output(
                ["git", "-C", str(source), "rev-parse", "--git-common-dir"]
            )
            harness_common = command_output(["git", "rev-parse", "--git-common-dir"])
            git_dir = command_output(
                ["git", "-C", str(source), "rev-parse", "--absolute-git-dir"]
            )
            backlink = Path(git_dir) / "gitdir"
            if (
                Path(top).resolve() != source.resolve()
                or source.resolve() == ROOT.resolve()
                or (source / common).resolve() != (ROOT / harness_common).resolve()
                or not backlink.is_file()
                or Path(backlink.read_text().strip()).resolve() != git_file.resolve()
            ):
                raise ValueError("backfill source must be a dedicated linked worktree")
        for revision in plan["revisions"]:
            if host_identity(args) != run["identity"]:
                raise ValueError("the backfill run identity has changed")
            directory = output / "runs" / revision
            directory.mkdir(parents=True, exist_ok=True)
            record_path, failure_path = (
                directory / "record.json",
                directory / "build-failure.json",
            )
            done = False
            if record_path.is_file():
                with contextlib.suppress(ValueError, KeyError, OSError, TypeError):
                    previous = read_record(record_path)
                    done = (
                        previous["run"]["commit"] == revision
                        and run.get("fingerprint") is not None
                        and previous["run"]["env"]["fingerprint"] == run["fingerprint"]
                        and not any(
                            row.get("error_kind") == "infrastructure"
                            for row in previous["results"]
                        )
                    )
            if not done and failure_path.is_file():
                with contextlib.suppress(ValueError, KeyError, OSError, TypeError):
                    previous = json.loads(failure_path.read_text())
                    done = (
                        previous["commit"] == revision
                        and previous["identity"] == run["identity"]
                    )
            if done:
                print(f"skip {revision[:12]}")
                continue
            record_path.unlink(missing_ok=True)
            failure_path.unlink(missing_ok=True)
            backfill_git(
                ["git", "-C", str(source), "checkout", "--detach", "--force", revision]
            )
            backfill_git(["git", "-C", str(source), "clean", "-ffdx"])
            arm = arm_namespace(args, revision, source, prefix, directory, shims)
            if not has_contact_driver(revision):
                arm.rows = "gzb,robot"
            drivers = sorted({row.driver for row in planned_rows(arm)} - {PB})
            clean_retry = False
            for attempt in (1, 2):
                started = time.monotonic()
                build_error = None
                try:
                    build_arm(
                        arm,
                        revision,
                        source,
                        build,
                        driver,
                        prefix,
                        drivers,
                        directory / "arm",
                    )
                except ValueError as error:
                    build_error = error
                finally:
                    head = subprocess.check_output(
                        ["git", "-C", str(source), "rev-parse", "HEAD"], text=True
                    ).strip()
                    dirty = subprocess.check_output(
                        ["git", "-C", str(source), "status", "--porcelain"], text=True
                    ).strip()
                    if head != revision or dirty:
                        raise ValueError("the source tree moved during the build")
                if isinstance(build_error, BuildFailure):
                    if attempt == 1:
                        for tree in (build, driver):
                            if tree.exists():
                                shutil.rmtree(tree)
                        clean_retry = True
                        continue
                    if not clean_retry:
                        raise ValueError(
                            "build failed after an infrastructure retry"
                        ) from build_error
                    write_json(
                        failure_path,
                        {
                            "commit": revision,
                            "identity": run["identity"],
                            "error": str(build_error),
                            "error_kind": "build",
                            "time": datetime.now(timezone.utc).isoformat(),
                        },
                    )
                    record_path.unlink(missing_ok=True)
                    print(f"{revision[:12]} build failed (recorded)")
                    break
                if build_error is not None:
                    if attempt == 1:
                        continue
                    raise build_error
                built = time.monotonic() - started
                measured = time.monotonic()
                try:
                    record = run_arm(arm)
                except ValueError:
                    if attempt == 1:
                        continue
                    raise
                if host_identity(args) != run["identity"]:
                    os.replace(record_path, directory / "record.drift.json")
                    raise ValueError("the backfill run identity has changed")
                env = record["run"]["env"]
                if "fingerprint" not in run:
                    for key, value in run["reference"].items():
                        if env.get(key) != value:
                            os.replace(record_path, directory / "record.drift.json")
                            raise ValueError(f"the hosted reference differs: {key}")
                    run["fingerprint"] = env["fingerprint"]
                    write_json(run_path, run)
                if env["fingerprint"] != run["fingerprint"]:
                    os.replace(record_path, directory / "record.drift.json")
                    raise ValueError("a second backfill fingerprint appeared")
                if any(
                    row.get("error_kind") == "infrastructure"
                    for row in record["results"]
                ):
                    if attempt == 1:
                        continue
                    os.replace(record_path, directory / "record.infrastructure.json")
                    raise ValueError(
                        f"infrastructure errors persist at {revision[:12]}"
                    )
                print(
                    f"{revision[:12]} built {built:.0f}s measured {time.monotonic() - measured:.0f}s"
                )
                break
        records = backfill_records(args, plan, run)
        print(
            f"assembled {len(records)} records ({sum(bool(pair['tag']) for pair in plan['pairs'])} tags)"
        )
        return records


def local_arms(args) -> tuple[dict, dict]:
    if getattr(args, "nightly", False):
        if not args.smoke or args.rows:
            raise ValueError(
                "--nightly requires --smoke and the complete nightly row set"
            )
        args.rows = "nightly"
    output = args.output_dir.resolve()
    default_output = ROOT / "build/perf-compare"
    marker = output / ".perf-compare-owned"
    if (
        output == default_output
        and not args.output_dir.is_symlink()
        and marker.is_file()
        and not marker.is_symlink()
    ):
        shutil.rmtree(output)
    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()):
        raise ValueError(
            "local output directory must be empty (choose a fresh scratch directory)"
        )
    if output == default_output:
        marker.write_text("dart-perf/1\n", encoding="utf-8")
    dependency = os.environ.get("CONDA_PREFIX")
    if not dependency:
        raise ValueError("local requires the active Pixi environment (CONDA_PREFIX)")
    shims = output / "shims"
    revisions = [
        command_output(["git", "rev-parse", "--verify", f"{rev}^{{commit}}"])
        for rev in ((args.head,) if args.smoke else (args.base, args.head))
    ]
    contact_drivers = [has_contact_driver(revision) for revision in revisions]
    # Only a base that predates the contact driver narrows the default rows; a
    # head that drops it must not hide the rows it no longer measures.
    if not args.smoke and contact_drivers[0] and not contact_drivers[1]:
        raise ValueError("the head revision lacks examples/contact_benchmark")
    if not contact_drivers[0] and not args.rows:
        args.rows = "gzb,robot"
    drivers = sorted({row.driver for row in planned_rows(args)} - {PB})
    records = []
    for label, revision in zip(("a", "b"), revisions):
        source, build = output / f"src-{label}", output / f"build-{label}"
        source.mkdir()
        archive = subprocess.check_output(["git", "archive", revision], cwd=ROOT)
        with tarfile.open(fileobj=io.BytesIO(archive)) as contents:
            contents.extractall(source, filter="data")
        prefix = output / label
        try:
            if label == "a":
                build_shims(args, shims)
            build_arm(
                args,
                revision,
                source,
                build,
                output / f"driver-{label}",
                prefix,
                drivers,
                output / label,
            )
        except BuildFailure as error:
            marker = {
                "commit": revision,
                "error": str(error),
                "error_kind": "build",
                "time": datetime.now(timezone.utc).isoformat(),
            }
            if args.smoke:
                write_json(output / "build-failure.json", marker)
            if label == "a":
                raise
            records.append(
                broken_arm(planned_rows(args), args, records[0]["run"], marker)
            )
            break
        arm = arm_namespace(
            args,
            revision,
            source,
            prefix,
            output / f"{label}-run",
            shims,
            base_arm=label == "a" and not args.smoke,
        )
        records.append(run_arm(arm))
    if args.smoke:
        records[0]["run"]["mode"] = "smoke"
        write_json(output / "a-run/record.json", records[0])
        records.append(records[0])
    if args.json is None:
        args.json = output / "perf.json"
    if args.markdown is None:
        args.markdown = output / "perf.md"
    return tuple(records)


def main(argv: list[str] | None = None) -> int:
    def interrupt(signum, frame):
        raise KeyboardInterrupt

    for signum in (signal.SIGTERM, signal.SIGHUP):
        if signal.getsignal(signum) is not signal.SIG_IGN:
            signal.signal(signum, interrupt)
    args = parser().parse_args(argv)
    try:
        if args.command == "backfill":
            if args.jobs < 1:
                raise ValueError("--jobs must be a positive integer")
            select_rows(args.rows)
            if args.plan_only:
                plan = backfill_plan(args)
                for revision in plan["revisions"]:
                    print(revision)
                tags = sum(bool(pair["tag"]) for pair in plan["pairs"])
                print(
                    f"planned {len(plan['revisions'])} revisions for "
                    f"{len(plan['pairs']) - tags} commits and {tags} tags "
                    f"({len(plan['skipped'])} skipped)"
                )
            else:
                backfill(args)
            return 0
        if args.command == "ledger":
            record = ledger_entries(
                load_records(args.records), args.since, args.until, args.intent
            )
            report = ledger_markdown(record)
            print(report, end="")
            if args.json:
                write_json(args.json, record)
            if args.markdown:
                args.markdown.write_text(report, encoding="utf-8")
            return 0
        if args.command == "publish":
            print(
                "Performance records published"
                if publish(args)
                else "Performance records already current"
            )
            return 0
        if args.command == "run":
            record = run_arm(args)
            for row in record["results"]:
                print(
                    f"{row_key(row)}: {row['status']}"
                    + (f" — {row['error']}" if row.get("error") else "")
                )
            if any(
                row.get("error_kind") == "infrastructure" for row in record["results"]
            ):
                return 2
            return int(
                any(
                    row["status"] not in ("ok", "unsupported")
                    or args.nightly
                    and row["status"] != "ok"
                    or row["status"] == "ok"
                    and args.perturb
                    and row.get("qualification_required", True)
                    and not row["gated"]
                    for row in record["results"]
                )
            )
        if args.command == "local":
            base, head = local_arms(args)
        else:
            base, head = read_record(args.base), read_record(args.head)
        try:
            record = compare(
                base,
                head,
                args.body_file.read_text(encoding="utf-8") if args.body_file else "",
            )
        except (AttributeError, TypeError) as error:
            # A value of the wrong type deeper in a record than read_record()
            # checks is still a malformed record, not a policy failure.
            raise ValueError(f"malformed measurement record: {error}") from error
        if args.command == "local" and args.smoke and args.perturb:
            if any(
                row["status"] == "ok"
                and row.get("qualification_required", True)
                and not row.get("gated")
                for row in head["results"]
            ):
                record["verdict"]["failures"].append(
                    "smoke perturbation qualification failed or missing"
                )
                if record["verdict"]["status"] != "ERROR":
                    record["verdict"]["status"] = "FAIL"
        # measure() marks a detector row broken when its contact-pair count is
        # missing, so such rows fail the nightly here.
        if getattr(args, "nightly", False) and any(
            row["status"] != "ok" for row in head["results"]
        ):
            record["verdict"]["failures"].append("nightly row failed or unsupported")
            if record["verdict"]["status"] != "ERROR":
                record["verdict"]["status"] = "FAIL"
        report = markdown(record)
        print(report, end="")
        if args.json:
            write_json(args.json, record)
        if args.markdown:
            args.markdown.write_text(report, encoding="utf-8")
        return (
            2
            if record["verdict"]["status"] == "ERROR"
            else int(record["verdict"]["status"] == "FAIL")
        )
    except Exception as error:
        print(f"perf: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
