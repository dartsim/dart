#!/usr/bin/env python3
"""Measure deterministic DART performance counts and compare revision records.

``run`` measures an installed arm; ``compare`` judges saved records; ``local``
prepares revisions and does both. Wall time and RSS are advisory. Requires Linux,
the system Valgrind, and the active Pixi build environment.
"""

from __future__ import annotations

import argparse
import contextlib
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
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VALGRIND = "/usr/bin/valgrind"
WORLD_SHA = "ad94d44b90f3023765e1b2a4d2ecc7390f5539fa761d6019e08b388ce5c5ff33"
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
    "pend": ((str(ROOT / "data/sdf/benchmark.world"),), 100, 1000, ("dart",)),
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


def identity(path: str, prefix: Path) -> None:
    if not Path(path).resolve().is_relative_to(prefix.resolve()):
        raise ValueError(f"arm identity: {path} is outside {prefix}")


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


def execute(command: list[str], env: dict, log: Path, timeout: int) -> str:
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
            raise ValueError(f"timeout: see {log}") from error
        except BaseException:
            kill_group(process)
            raise
        finally:
            with RUNNING_LOCK:
                RUNNING.pop(id(process), None)
    text = log.read_text(encoding="utf-8", errors="replace")
    unsupported = re.search(r"^UNSUPPORTED: (.+)$", text, re.MULTILINE)
    if process.returncode == 3 and unsupported:
        raise UnsupportedRow(unsupported[1])
    # Both drivers print complete guards before exiting nonzero on a non-finite
    # state or failed time/frame advancement. These are
    # measured correctness failures, not infrastructure ones.
    if process.returncode and failed_guards(text):
        return text
    if process.returncode:
        raise ValueError(f"exit {process.returncode}: see {log}")
    return text


def row_command(row: Row, args, world: Path, warmup: int, steps: int) -> list[str]:
    command = [
        str(args.bin_dir / row.driver),
        *(str(world) if item == "{world}" else item for item in row.args),
    ]
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
        command += ["--collision", row.det, "--quiet", "--checkpoint", "0"]
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
    return {
        "guards": guards(text),
        "time_advanced": not bool(
            re.search(r"^Time Advanced:\s*false\s*$", text, re.MULTILINE)
        ),
        "allocs_per_step": allocs / measured,
        "bytes_per_step": size / measured,
        "allocs": allocs,
        "bytes": size,
        "libdart": match[5],
        "max_rss_kb": int(rss[1]),
        "wall_ms_per_step": float(field(text, "Avg Step Time").split()[0]),
    }


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
                raise ValueError(f"callgrind/native libdart identity differs: {obj}")
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


def measure(row: Row, args, world: Path) -> dict:
    result = {
        "row": row.row,
        "det": row.det,
        "version": row.version,
        "gated": False,
        "status": "ok",
        "threads": row.threads,
        "window": {"warmup": row.warmup, "steps": row.steps},
        "method": "slope" if row.ir and not args.native_only else "native",
        "expected_ir": row.ir and not args.native_only,
        "collection_signature": (
            COLLECTION_SIGNATURES[row.driver]
            if row.ir and not args.native_only
            else None
        ),
        "parity": row.parity,
    }
    if "{world}" in row.args:
        result["input_sha"] = WORLD_SHA
    elif row.row == "pend":
        result["input_sha"] = sha((ROOT / "data/sdf/benchmark.world").read_bytes())
    elif row.row == "robot":
        result["input_sha"] = sha(
            b"".join(
                path.relative_to(ROOT).as_posix().encode() + path.read_bytes()
                for path in sorted((ROOT / "data/sdf/atlas").glob("*"))
                if path.is_file()
            )
        )
    else:
        result["input_sha"] = sha(
            json.dumps([row.row, row.args], sort_keys=True).encode()
        )
    try:
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
        if args.perturb:
            result["perturbations"] = {}
            for config in PERTURBATIONS:
                altered = (
                    native(row, args, world, config)
                    if row.det
                    else micro_perturb(row, args, world, config)
                )
                # Requested bytes gate too, so they must not depend on layout,
                # and every perturbed run must also advance time correctly.
                stable = altered.get("time_advanced") is not False and all(
                    altered.get(key) == metrics.get(key)
                    for key in ("guards", "allocs", "bytes")
                )
                result["perturbations"][config] = {
                    "stable": stable,
                    "guards": altered["guards"],
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
    selected = []
    for name in names.split(","):
        matches = [
            row
            for row in ROWS
            if name in (row.key, row.row) or name == "mt4" and row.parity
        ]
        if not matches:
            raise ValueError(f"unknown row {name}")
        selected += [row for row in matches if row not in selected]
    return selected


def cmake_compiler(build: Path) -> str:
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
    return " ".join(parts)


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
            for path in sorted((ROOT / "tools/perf").glob("*"))
            if path.is_file()
        )
    )
    values = {
        "valgrind": version,
        "valgrind_guest_cpu": cpu,
        "compiler": provenance["compiler"],
        "compiler_provenance": provenance["schema"],
        "glibc": command_output(["getconf", "GNU_LIBC_VERSION"]).split()[-1],
        "pixi_lock_sha": provenance["pixi_lock_sha"],
        "runtime_pixi_lock_sha": sha(
            (Path(env.get("PIXI_PROJECT_ROOT", ROOT)) / "pixi.lock").read_bytes()
        ),
        "runtime_environment": env.get(
            "PIXI_ENVIRONMENT_NAME", env.get("CONDA_PREFIX")
        ),
        "preset": provenance["preset"],
        "harness_sha": harness,
        "allocshim_sha": sha(args.shim.read_bytes()),
        "heappad_sha": sha(args.heappad.read_bytes()) if args.perturb else None,
    }
    values["runner"] = {
        "environment": os.environ.get("RUNNER_ENVIRONMENT", "local"),
        "name": os.environ.get("RUNNER_NAME", os.uname().nodename),
        "image": "/".join(
            os.environ.get(key, "") for key in ("ImageOS", "ImageVersion")
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
            {key: value for key, value in values.items() if key != "runner"},
            sort_keys=True,
        ).encode()
    )
    return values


def run_arm(args) -> dict:
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
    rows = select_rows(args.rows)
    for row in list(rows):
        if row.parity and all(other.key != row.parity for other in rows):
            rows.append(next(other for other in ROWS if other.key == row.parity))
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        try:
            results = list(pool.map(lambda row: measure(row, args, world), rows))
        except BaseException:
            # Workers wait on their benchmarks; stop those before the pool's
            # shutdown waits for the workers.
            kill_running()
            raise
    for row, result in zip(rows, results):
        if row.driver in WORKLOAD_SOURCES:
            result["workload_sha"] = provenance["workload_sources"][row.driver]
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
                or serial["head"]["guards"] != result["head"]["guards"]
            ):
                result.update(
                    status="broken", error="mt4 guard parity missing or unequal"
                )
    record = {
        "schema": "dart-perf/1",
        "run": {
            "tier": "local",
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
        result = {**(child or parent), "parent": bm, "head": hm}
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
            else bool(bm.get("guards") and bm.get("guards") == hm.get("guards"))
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
        f"Perf A/B: {record['run']['parent']} → {record['run']['commit']} — {verdict['status']}",
        f"{method}; {thread_text}; Valgrind {env['valgrind']}; {env.get('compiler') or 'compiler unavailable'}; glibc {env['glibc']}; {env['preset']}",
        "",
    ]
    counts = {}
    for row in record["results"]:
        classification = row["delta"]["class"]
        if classification == "gated":
            classification = (
                "regressed"
                if row["failures"]
                or any(
                    row["delta"][key] is not None and row["delta"][key] > 0
                    for key in ("ir", "allocs", "bytes")
                )
                else (
                    "improved"
                    if row["delta"]["ir"] is not None
                    and at_least(-row["delta"]["ir"], 0.01)
                    else "neutral"
                )
            )
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
        lines.append(
            f"| {row_key(row)} | {row.get('threads', 1)} | {number(bm.get('ir_per_step'))} | {number(hm.get('ir_per_step'))} | {change_text} | {number(bm.get('allocs_per_step'))} → {number(hm.get('allocs_per_step'))} | {bytes_text} | {guard_text} | {change['class']} | {row['gate_reason']} |"
        )
    lines += [
        "",
        "Wall time is recorded as advisory; RSS warns at +5%. Diagnostic and behaviour-change deltas do not enter the Ir gate.",
    ]
    return "\n".join(lines) + "\n"


def write_json(path: Path, value: dict) -> None:
    path.write_text(
        json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )


def read_record(path: Path) -> dict:
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
    # A comparison report has the same schema, but its rows carry the base's
    # qualification and per-arm deltas, not one revision's measurements.
    if "verdict" in record:
        raise ValueError("comparison report given where a measurement record belongs")
    return record


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    sub = result.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run", help="measure an installed arm")
    run.add_argument("--prefix", type=Path, required=True)
    run.add_argument("--bin-dir", type=Path)
    run.add_argument("--commit", required=True, help="revision installed in --prefix")
    local = sub.add_parser("local", help="build revisions and compare them")
    local.add_argument("--base", default="origin/main")
    local.add_argument("--head", default="HEAD")
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
        item.add_argument("--shim", type=Path)
        item.add_argument("--heappad", type=Path)
    for item in (sub.add_parser("compare", help="judge saved measurements"), local):
        if item is not local:
            item.add_argument("--base", type=Path, required=True)
            item.add_argument("--head", type=Path, required=True)
        item.add_argument("--body-file", type=Path)
        item.add_argument("--json", type=Path)
        item.add_argument("--markdown", type=Path)
    return result


def local_arms(args) -> tuple[dict, dict]:
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
    shims.mkdir()
    for name in ("allocshim", "heappad"):
        execute(
            [
                "/usr/bin/cc",
                "-O2",
                "-shared",
                "-fPIC",
                "-o",
                str(shims / f"{name}.so"),
                str(ROOT / f"tools/perf/{name}.c"),
                "-ldl",
            ],
            os.environ.copy(),
            shims / f"{name}.log",
            args.timeout,
        )
    revisions = [
        command_output(["git", "rev-parse", "--verify", f"{rev}^{{commit}}"])
        for rev in (args.base, args.head)
    ]
    portable_only = any(
        not subprocess.run(
            [
                "git",
                "cat-file",
                "-e",
                f"{rev}:examples/contact_benchmark/CMakeLists.txt",
            ],
            cwd=ROOT,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        ).returncode
        == 0
        for rev in revisions
    )
    if portable_only and not args.rows:
        args.rows = "gzb,robot"
    drivers = sorted({row.driver for row in select_rows(args.rows)} - {PB})
    records = []
    for label, revision in zip(("a", "b"), revisions):
        source, build = output / f"src-{label}", output / f"build-{label}"
        source.mkdir()
        archive = subprocess.check_output(["git", "archive", revision], cwd=ROOT)
        with tarfile.open(fileobj=io.BytesIO(archive)) as contents:
            contents.extractall(source, filter="data")
        prefix = output / label
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
        try:
            execute(
                ["cmake", "-G", "Ninja", "-S", str(source), "-B", str(build), *options],
                os.environ.copy(),
                output / f"{label}.configure.log",
                args.timeout,
            )
            workload_sources = workload_hashes(source, drivers, build, prefix)
            targets = ["dart-utils-urdf", *drivers]
            if CB not in drivers:
                targets += [
                    "dart-collision-ode",
                    "dart-collision-bullet",
                    "dart-gui-osg",
                ]
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
                output / f"{label}.build.log",
                max(args.timeout, 3600),
            )
            execute(
                ["cmake", "--install", str(build), "--prefix", str(prefix)],
                os.environ.copy(),
                output / f"{label}.install.log",
                args.timeout,
            )
            binary = prefix / "bin"
            binary.mkdir(exist_ok=True)
            for target in targets:
                if (build / "bin" / target).is_file():
                    shutil.copy2(build / "bin" / target, binary / target)
            driver_build = output / f"driver-{label}"
            execute(
                [
                    "cmake",
                    "-G",
                    "Ninja",
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
                output / f"{label}.driver-configure.log",
                args.timeout,
            )
            workload_sources |= workload_hashes(ROOT, [PB], driver_build, prefix)
            execute(
                ["cmake", "--build", str(driver_build), "--parallel", str(args.jobs)],
                os.environ.copy(),
                output / f"{label}.driver-build.log",
                args.timeout,
            )
            shutil.copy2(
                driver_build / "portable_step_bench", binary / "portable_step_bench"
            )
            compiler = cmake_compiler(build)
            if cmake_compiler(driver_build) != compiler:
                raise ValueError("DART and portable driver compiler provenance differs")
            write_json(
                prefix / "share/dart/perf-build.json",
                {
                    "schema": "dart-perf-build/1",
                    "commit": revision,
                    "compiler": compiler,
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
        except (OSError, ValueError) as error:
            if label == "a":
                raise
            records.append(
                {
                    "schema": "dart-perf/1",
                    "run": {**records[0]["run"], "commit": revision},
                    "results": [
                        {
                            **row,
                            "status": "broken",
                            "gated": False,
                            "perturbations": {},
                            "error": f"head build failed: {error}",
                            "error_kind": "infrastructure",
                            "head": {},
                        }
                        for row in records[0]["results"]
                    ],
                }
            )
            break
        arm = argparse.Namespace(**vars(args))
        arm.prefix, arm.bin_dir, arm.commit = prefix, binary, revision
        arm.output_dir = output / f"{label}-run"
        arm.shim, arm.heappad = shims / "allocshim.so", shims / "heappad.so"
        records.append(run_arm(arm))
    if args.json is None:
        args.json = output / "perf.json"
    if args.markdown is None:
        args.markdown = output / "perf.md"
    return tuple(records)


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
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
                    or row["status"] == "ok"
                    and args.perturb
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
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(f"perf: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
