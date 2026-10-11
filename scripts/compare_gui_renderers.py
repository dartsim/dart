"""Compare completed DART renderer frames on Linux against measured OSG noise.

Run inside the selected Pixi environment with a current Filament binary and a
separately preserved OSG reference binary. Both arms retain their build/source
provenance. Matching behavior guards do not imply byte-identical core builds.
The script alternates fresh processes and never treats submission rate as
completed rendering.
"""

import argparse
import hashlib
import json
import math
import os
import platform
import re
import shutil
import statistics
import subprocess
import time
from pathlib import Path


def core_id(cpu):
    topology = Path(f"/sys/devices/system/cpu/cpu{cpu}/topology")
    return tuple(
        (topology / name).read_text().strip()
        for name in ("physical_package_id", "core_id")
    )


def physical_cpus():
    """Use one allowed hardware thread per physical core."""
    selected = {}
    for cpu in sorted(os.sched_getaffinity(0)):
        selected.setdefault(core_id(cpu), cpu)
    return list(selected.values())


def validate_cpus(cpus):
    if not cpus or not set(cpus).issubset(os.sched_getaffinity(0)):
        raise ValueError("CPU affinity must use allowed hardware threads")
    if len({core_id(cpu) for cpu in cpus}) != len(cpus):
        raise ValueError("CPU affinity must use one hardware thread per physical core")


def file_hash(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def resolved_libraries(binary, environment=None):
    output = subprocess.run(
        ["ldd", str(binary)],
        capture_output=True,
        text=True,
        check=True,
        **({"env": environment} if environment is not None else {}),
    ).stdout
    if "not found" in output:
        raise RuntimeError("A benchmark dependency could not be resolved by ldd")
    paths = re.findall(r"(?:=>\s+|^\s*)(/\S+)\s+\(", output, re.MULTILINE)
    if not paths:
        raise RuntimeError("ldd did not resolve any benchmark libraries")
    return {
        path: file_hash(Path(path).resolve(strict=True)) for path in sorted(set(paths))
    }


def source_snapshot(source):
    paths = [
        "dart",
        "cmake",
        "tests/benchmark",
        "scripts/compare_gui_renderers.py",
        "tests/test_gui_renderer_measurement.py",
        "CMakeLists.txt",
        "pixi.toml",
        "pixi.lock",
    ]
    files = (
        subprocess.run(
            [
                "git",
                "ls-files",
                "-z",
                "--cached",
                "--others",
                "--exclude-standard",
                "--",
                *paths,
            ],
            cwd=source,
            capture_output=True,
            check=True,
        )
        .stdout.decode()
        .split("\0")
    )
    return {
        "revision": subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=source,
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip(),
        "status": subprocess.run(
            ["git", "status", "--porcelain", "--untracked-files=all", "--", *paths],
            cwd=source,
            capture_output=True,
            text=True,
            check=True,
        ).stdout,
        "files": {
            name: file_hash(source / name) if (source / name).exists() else None
            for name in sorted(set(files) - {""})
        },
    }


def provenance(binary):
    build = next(
        (parent for parent in binary.parents if (parent / "CMakeCache.txt").is_file()),
        None,
    )
    if build is None:
        raise RuntimeError("Benchmark has no parent CMakeCache.txt")
    caches = {}
    compilers = {}
    for cache in sorted(build.rglob("CMakeCache.txt")):
        values = {}
        for line in cache.read_text().splitlines():
            if (
                line
                and not line.startswith(("#", "//"))
                and ":" in line
                and "=" in line
            ):
                key, value = line.split("=", 1)
                values[key.split(":", 1)[0]] = value
        caches[str(cache)] = {"sha256": file_hash(cache), "values": values}
        for key in (
            "CMAKE_C_COMPILER",
            "CMAKE_CXX_COMPILER",
            "DART_FILAMENT_SOURCE_CXX_COMPILER",
        ):
            if key in values:
                compiler = Path(values[key]).resolve(strict=True)
                compilers[str(compiler)] = {
                    "sha256": file_hash(compiler),
                    "version": subprocess.run(
                        [str(compiler), "--version"],
                        capture_output=True,
                        text=True,
                        check=True,
                    ).stdout,
                }
    source = Path(
        caches[str(build / "CMakeCache.txt")]["values"]["CMAKE_HOME_DIRECTORY"]
    ).resolve(strict=True)
    return {
        "binary": {"path": str(binary), "sha256": file_hash(binary)},
        "libraries": resolved_libraries(binary),
        "source": source_snapshot(source),
        "dependency_lock": {
            "path": str(source / "pixi.lock"),
            "sha256": file_hash(source / "pixi.lock"),
        },
        "cmake": caches,
        "compilers": compilers,
        "build_files": {
            str(path): file_hash(path)
            for name in (
                "build.ninja",
                "CMakeFiles/rules.ninja",
                "compile_commands.json",
            )
            if (path := build / name).is_file()
        },
    }


def verify_provenance(binary, expected):
    if provenance(binary) != expected:
        raise RuntimeError(
            "Benchmark binary, libraries, source, or build provenance changed during measurements"
        )


def reference_environment(library_directory):
    """Confine the preserved reference's library prefix to its own processes."""
    if library_directory is None:
        return None
    directory = library_directory.resolve(strict=True)
    if not directory.is_dir():
        raise ValueError("Reference library prefix must be a directory")
    environment = os.environ.copy()
    inherited = environment.get("LD_LIBRARY_PATH", "")
    environment["LD_LIBRARY_PATH"] = str(directory) + (
        os.pathsep + inherited if inherited else ""
    )
    return environment


def reference_provenance(binary, manifest_path, environment=None):
    """Verify an immutable old build without resnapshotting its changed checkout.

    The manifest records binary_sha256, libraries (name -> sha256/original),
    revision, source_freeze (path/sha256), and build_files (relative path -> hash).
    The source freeze contains the original benchmark_source_snapshot. All
    paths relative to the manifest are resolved alongside that manifest.
    """
    manifest = json.loads(manifest_path.read_text())
    if file_hash(binary) != manifest["binary_sha256"]:
        raise RuntimeError("Reference binary differs from its frozen provenance")
    freeze = manifest["source_freeze"]
    freeze_path = (manifest_path.parent / freeze["path"]).resolve(strict=True)
    if file_hash(freeze_path) != freeze["sha256"]:
        raise RuntimeError("Reference source freeze changed")
    snapshot = json.loads(freeze_path.read_text())["benchmark_source_snapshot"]
    if (
        not snapshot["files"]
        or not snapshot["revision"]
        or snapshot["revision"] != manifest["revision"]
    ):
        raise RuntimeError("Reference source revision does not match its build")
    build_files = manifest["build_files"]
    if "CMakeCache.txt" not in build_files or (
        "source/tests/benchmark/integration/gui_render_benchmark.cpp" not in build_files
    ):
        raise RuntimeError("Reference requires its build cache and benchmark source")
    for name, expected in build_files.items():
        path = manifest_path.parent / name
        if file_hash(path) != expected:
            raise RuntimeError(f"Reference build file changed: {name}")
        if name.startswith("source/") and expected != snapshot["files"].get(
            name.removeprefix("source/")
        ):
            raise RuntimeError(f"Reference source does not match its freeze: {name}")
    libraries = resolved_libraries(binary, environment)
    expected_libraries = {
        name: value["sha256"] for name, value in manifest["libraries"].items()
    }
    if {
        Path(path).name: value for path, value in libraries.items()
    } != expected_libraries:
        raise RuntimeError("Reference resolved libraries differ from its frozen build")
    return {
        "binary": {"path": str(binary), "sha256": file_hash(binary)},
        "libraries": libraries,
        "source": snapshot,
        "manifest": {"path": str(manifest_path), "sha256": file_hash(manifest_path)},
        "source_freeze": {"path": str(freeze_path), "sha256": freeze["sha256"]},
        "declared_build": manifest,
        "library_path": (environment or os.environ).get("LD_LIBRARY_PATH"),
    }


def verify_reference_provenance(binary, manifest_path, environment, expected):
    if reference_provenance(binary, manifest_path, environment) != expected:
        raise RuntimeError("Reference build provenance changed during measurements")


def measurement_phase(output):
    phase = 0
    for line in output.splitlines():
        if line == "DART_GUI_BENCHMARK_PHASE measured_begin":
            if phase != 0:
                raise RuntimeError("Duplicate or misplaced measured_begin marker")
            phase = 1
        elif line == "DART_GUI_BENCHMARK_PHASE measured_end":
            if phase != 1:
                raise RuntimeError("Duplicate or misplaced measured_end marker")
            phase = 2
    return phase


def gpu_memory_metrics(samples):
    measured = [sample["mib"] for sample in samples if sample["measured"]]
    if not measured:
        raise RuntimeError(
            "No GPU memory samples wholly within the measured frame window"
        )
    return {
        "gpu_memory_peak_mib": max(sample["mib"] for sample in samples),
        "gpu_memory_steady_mib": statistics.median(measured[len(measured) // 2 :]),
        "gpu_memory_samples": samples,
    }


def graphics_memory(output, pid, gpu_index):
    """Read this graphics process's framebuffer MB from nvidia-smi pmon."""
    for line in output.splitlines():
        columns = line.split()
        if len(columns) >= 5 and columns[0] == str(gpu_index):
            if columns[1] == str(pid) and columns[3].isdigit():
                return int(columns[3])
    return None


def compare_samples(noise_pairs, candidate_pairs):
    """Bound each lower-is-better metric by its observed baseline pair noise."""
    metrics = sorted(set.intersection(*(set(a) & set(b) for a, b in noise_pairs)))
    results = {}
    for metric in metrics:
        noise = max(abs(b[metric] / a[metric] - 1) for a, b in noise_pairs)
        ratios = [
            candidate[metric] / baseline[metric] - 1
            for baseline, candidate in candidate_pairs
        ]
        delta = statistics.median(ratios)
        results[metric] = {
            "baseline_pair_noise": noise,
            "paired_relative_change": delta,
            "paired_change_range": [min(ratios), max(ratios)],
            "passed": delta <= noise,
        }
    return results


def measure(binary, renderer, scene, args, directory, index, environment=None):
    stem = directory / f"{index:03d}-{renderer}"
    command = [
        "taskset",
        "-c",
        ",".join(map(str, args.cpus)),
        str(binary),
        "--renderer",
        renderer,
        "--scene",
        scene,
        "--width",
        str(args.width),
        "--height",
        str(args.height),
        "--warmup",
        str(args.warmup),
        "--seconds",
        str(args.seconds),
    ]
    if args.shadows:
        command.append("--shadows")
    memory = []
    peak_rss = 0
    with stem.with_suffix(".log").open("w") as output:
        process = subprocess.Popen(
            command,
            stdout=output,
            stderr=subprocess.STDOUT,
            **({"env": environment} if environment is not None else {}),
        )
        started = time.monotonic()
        deadline = started + args.seconds + 180
        try:
            while process.poll() is None:
                if time.monotonic() > deadline:
                    raise RuntimeError(f"Benchmark timed out: {stem}")
                try:
                    status = Path(f"/proc/{process.pid}/status").read_text()
                    for line in status.splitlines():
                        if line.startswith("VmHWM:"):
                            peak_rss = max(peak_rss, int(line.split()[1]))
                except FileNotFoundError:
                    pass
                if args.gpu_index is not None:
                    phase_before = measurement_phase(
                        stem.with_suffix(".log").read_text()
                    )
                    sample_start = time.monotonic() - started
                    sample = subprocess.run(
                        [
                            "nvidia-smi",
                            "pmon",
                            "-s",
                            "m",
                            "-c",
                            "1",
                            "-i",
                            str(args.gpu_index),
                        ],
                        capture_output=True,
                        text=True,
                        check=True,
                        timeout=5,
                    )
                    sample_end = time.monotonic() - started
                    phase_after = measurement_phase(
                        stem.with_suffix(".log").read_text()
                    )
                    value = graphics_memory(sample.stdout, process.pid, args.gpu_index)
                    if value is not None:
                        memory.append(
                            {
                                "start_seconds": sample_start,
                                "end_seconds": sample_end,
                                "mib": value,
                                "measured": phase_before == phase_after == 1,
                            }
                        )
                else:
                    time.sleep(0.05)
        finally:
            if process.poll() is None:
                process.kill()
            process.wait()
    if process.returncode:
        raise RuntimeError(
            f"Benchmark failed ({process.returncode}): {stem.with_suffix('.log')}"
        )
    log = stem.with_suffix(".log").read_text()
    if measurement_phase(log) != 2:
        raise RuntimeError("Benchmark did not delimit its measured frame window")
    rows = [line for line in log.splitlines() if line.startswith("{")]
    if not rows:
        raise RuntimeError(f"No benchmark record: {stem}")
    record = json.loads(rows[-1])
    if record["renderer"] != renderer or record["scene"] != scene:
        raise RuntimeError("Benchmark ran a different renderer or scene")
    if args.expect_device.casefold() not in record["device"].casefold():
        raise RuntimeError(f"Unexpected render device: {record['device']}")
    if args.gpu_index is None and "llvmpipe" not in record["device"].casefold():
        raise RuntimeError("Hardware GPU resource checks require --gpu-index")
    if (record["width"], record["height"], record["shadows"]) != (
        args.width,
        args.height,
        args.shadows,
    ):
        raise RuntimeError("Benchmark used different dimensions or shadows")
    if record["frames"] <= 0 or not record["guard_pixels"]:
        raise RuntimeError("Completed-frame or pixel guard failed")
    record["command"] = command
    record["peak_rss_kib"] = max(peak_rss, record.get("peak_rss_kib", 0))
    if args.gpu_index is not None:
        if not memory:
            raise RuntimeError(
                "No per-process GPU memory samples; resource gate cannot pass"
            )
        record.update(gpu_memory_metrics(memory))
    for metric in ("render_ms", "setup_ms", "readback_full_frame_ms", "peak_rss_kib"):
        if (
            not isinstance(record[metric], (float, int))
            or not math.isfinite(record[metric])
            or record[metric] <= 0
        ):
            raise RuntimeError(f"Invalid benchmark metric: {metric}")
    stem.with_suffix(".json").write_text(json.dumps(record, indent=2) + "\n")
    return record


def metric_values(record):
    names = (
        "render_ms",
        "setup_ms",
        "readback_full_frame_ms",
        "peak_rss_kib",
        "gpu_memory_peak_mib",
        "gpu_memory_steady_mib",
    )
    return {name: record[name] for name in names if name in record}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path, help="Current Filament benchmark binary")
    parser.add_argument("--reference-binary", type=Path, required=True)
    parser.add_argument("--reference-provenance", type=Path, required=True)
    parser.add_argument(
        "--reference-library-dir",
        type=Path,
        help="Prepend this preserved library directory only for the OSG arm",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--scenes",
        nargs="+",
        choices=["boxes", "mesh", "pointline", "soft"],
        default=["boxes"],
    )
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--warmup", type=int, default=60)
    parser.add_argument("--repeats", type=int, default=7)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--gpu-index", type=int)
    parser.add_argument(
        "--expect-device",
        required=True,
        help="Require this hardware or software device substring in both graphics device names",
    )
    parser.add_argument("--shadows", action="store_true")
    parser.add_argument("--cpus", type=int, nargs="+")
    args = parser.parse_args()
    if platform.system() != "Linux":
        parser.error("Linux reference measurements only")
    if (
        not math.isfinite(args.seconds)
        or args.seconds < 30
        or args.repeats < 7
        or args.warmup < 1
    ):
        parser.error("Require at least 30 measured seconds, 7 pairs, and warm-up")
    if args.width <= 0 or args.height <= 0:
        parser.error("Framebuffer dimensions must be positive")
    if not args.expect_device.strip():
        parser.error("Expected render device must be explicit")
    args.cpus = args.cpus or physical_cpus()
    try:
        validate_cpus(args.cpus)
    except ValueError as error:
        parser.error(str(error))
    if args.gpu_index is not None and args.gpu_index < 0:
        parser.error("GPU index must be nonnegative")
    if args.output.exists() and (
        not args.output.is_dir() or any(args.output.iterdir())
    ):
        parser.error("Output directory must be empty")
    binary = args.binary.resolve(strict=True)
    reference = args.reference_binary.resolve(strict=True)
    if binary == reference:
        parser.error("Filament and OSG reference must be separate binaries")
    reference_manifest = args.reference_provenance.resolve(strict=True)
    reference_env = reference_environment(args.reference_library_dir)
    initial_provenance = {
        "filament": provenance(binary),
        "osg": reference_provenance(reference, reference_manifest, reference_env),
    }

    def verify_builds():
        verify_provenance(binary, initial_provenance["filament"])
        verify_reference_provenance(
            reference, reference_manifest, reference_env, initial_provenance["osg"]
        )

    binaries = {"filament": binary, "osg": reference}
    args.output.mkdir(parents=True, exist_ok=True)
    report = {
        "host": platform.node(),
        "platform": platform.platform(),
        "cpu_models": sorted(
            {
                line.split(":", 1)[1].strip()
                for line in Path("/proc/cpuinfo").read_text().splitlines()
                if line.startswith("model name")
            }
        ),
        "cpus": args.cpus,
        "provenance": initial_provenance,
        "comparison_scope": "Separate preserved OSG and current Filament builds; matching recorded scene/state guards do not establish byte-identical core libraries or source builds.",
        "expected_device": args.expect_device,
        "gpu_index": args.gpu_index,
        "graphics_driver": {
            name: subprocess.run(
                command, capture_output=True, text=True, check=True
            ).stdout
            for name, command in {
                "nvidia": [
                    "nvidia-smi",
                    "--query-gpu=index,name,driver_version,uuid,pci.bus_id",
                    "--format=csv,noheader",
                ],
                "opengl": ["glxinfo", "-B"],
            }.items()
            if shutil.which(command[0])
        },
        "load_average_before": os.getloadavg(),
        "cpu_governors": {
            str(cpu): (
                Path(f"/sys/devices/system/cpu/cpu{cpu}/cpufreq/scaling_governor")
                .read_text()
                .strip()
                if Path(
                    f"/sys/devices/system/cpu/cpu{cpu}/cpufreq/scaling_governor"
                ).exists()
                else None
            )
            for cpu in args.cpus
        },
        "seconds_per_process": args.seconds,
        "repeats": args.repeats,
        "graphics_environment": {
            name: os.environ.get(name)
            for name in (
                "DISPLAY",
                "WAYLAND_DISPLAY",
                "VK_DRIVER_FILES",
                "MESA_VK_DEVICE_SELECT",
                "LIBGL_ALWAYS_SOFTWARE",
                "__GLX_VENDOR_LIBRARY_NAME",
                "__GL_SYNC_TO_VBLANK",
                "LD_LIBRARY_PATH",
            )
        },
        "gpu_sampling": "Per-process nvidia-smi pmon framebuffer MB; maximum observed one-second sample, not an instantaneous peak. Steady memory uses the latter half of samples wholly inside the measured frame window.",
        "complete": False,
        "scenes": {},
    }
    report_path = args.output / "report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    for scene in args.scenes:
        directory = args.output / scene
        directory.mkdir(exist_ok=True)
        records = []
        pairs = {"noise": [], "candidate": []}
        for phase in pairs:
            for repeat in range(args.repeats):
                renderers = ["osg", "osg"] if phase == "noise" else ["osg", "filament"]
                if phase == "candidate" and repeat % 2:
                    renderers.reverse()
                pair = {}
                for renderer in renderers:
                    verify_builds()
                    print(
                        f"{scene}: {phase} pair {repeat + 1}/{args.repeats}, {renderer}",
                        flush=True,
                    )
                    record = measure(
                        binaries[renderer],
                        renderer,
                        scene,
                        args,
                        directory,
                        len(records),
                        reference_env if renderer == "osg" else None,
                    )
                    verify_builds()
                    if records and (record["state_hash"], record["world_time"]) != (
                        records[0]["state_hash"],
                        records[0]["world_time"],
                    ):
                        raise RuntimeError(
                            "World behavior differs between benchmark runs"
                        )
                    records.append(record)
                    pair.setdefault(renderer, []).append(record)
                baseline = pair["osg"][0]
                other = pair["osg"][1] if phase == "noise" else pair["filament"][0]
                pairs[phase].append((metric_values(baseline), metric_values(other)))
        report["scenes"][scene] = {
            "records": records,
            "metrics": compare_samples(pairs["noise"], pairs["candidate"]),
        }
        report["load_average_after"] = os.getloadavg()
        report_path.write_text(json.dumps(report, indent=2) + "\n")
    verify_builds()
    report["complete"] = True
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    if not all(
        metric["passed"]
        for scene in report["scenes"].values()
        for metric in scene["metrics"].values()
    ):
        raise SystemExit(
            "Renderer regression exceeds measured baseline noise; see report.json"
        )


if __name__ == "__main__":
    main()
