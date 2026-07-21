#!/usr/bin/env python3
"""Generate current-source visual evidence for the literal Native25 arch.

This driver runs the dedicated off-screen capture executable, independently
runs ``fbf_paper_trace`` with the matching literal-wedge/colored-FBF contract,
checks their per-step physical outputs, validates every rendered frame, and
encodes a short stability clip. It deliberately labels the result as a DART
reconstruction rather than paper parity.
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any, Iterable, Sequence

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from _image_tools import read_image  # noqa: E402
from image_verdict import (  # noqa: E402
    analyze_contrast,
    analyze_non_blank,
    build_verdict,
)

SCHEMA_VERSION = "dart.fbf_literal_wedge_visual_evidence/v1"
DEFAULT_CAPTURE = (
    ROOT / "build/default/cpp/Release/tests/benchmark/integration/"
    "fbf_literal_wedge_visual_capture"
)
DEFAULT_TRACE = (
    ROOT / "build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace"
)
DEFAULT_FFMPEG = ROOT / ".pixi/envs/gazebo/bin/ffmpeg"
DEFAULT_FFPROBE = ROOT / ".pixi/envs/gazebo/bin/ffprobe"
DEFAULT_OUTPUT = (
    ROOT / "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence/"
    "fig07_arch25_literal"
)

CAPTURE_SOURCE_RELATIVE = (
    "tests/benchmark/integration/fbf_literal_wedge_visual_capture.cpp"
)
LITERAL_ARCH_SPEC_SOURCE_RELATIVE = (
    "examples/demos/scenes/FbfLiteralMasonryArchSpec.hpp"
)
TRACE_SOURCE_RELATIVE = "tests/benchmark/integration/fbf_paper_trace.cpp"
SOURCE_FILES = (
    "tests/benchmark/integration/CMakeLists.txt",
    CAPTURE_SOURCE_RELATIVE,
    TRACE_SOURCE_RELATIVE,
    LITERAL_ARCH_SPEC_SOURCE_RELATIVE,
    "dart/math/detail/MasonryArchGeometry.hpp",
    "dart/collision/native/NativeCollisionDetector.cpp",
    "dart/collision/native/NativeCollisionDetector.hpp",
    "dart/constraint/ExactCoulombFbfConstraintSolver.cpp",
    "dart/constraint/ExactCoulombFbfConstraintSolver.hpp",
    "dart/constraint/detail/ExactCoulombContactRowOperator.hpp",
    "scripts/_image_tools.py",
    "scripts/image_compose.py",
    "scripts/image_verdict.py",
    "scripts/run_fbf_literal_wedge_visual_capture.py",
)

BUILD_DIRECTORY = ROOT / "build/default/cpp/Release"
CMAKE_CACHE = BUILD_DIRECTORY / "CMakeCache.txt"
PENDING_STATUS = "pending_manual_inspection_nonpaper_reconstruction"
FINAL_STATUS = "valid_current_source_nonpaper_reconstruction"
MANUAL_SCHEMA_VERSION = "dart.fbf_literal_wedge_manual_inspection/v2"
INDEX_SCHEMA_VERSION = "dart.fbf_literal_wedge_artifact_index/v2"

TIMELINE_STILL_STEPS = (0, 150, 300, 450, 600)
MANUAL_STILL_STEPS = (0, 300, 600)
STILL_PATH_BY_STEP = {
    step: f"stills/step_{step:06d}.png" for step in TIMELINE_STILL_STEPS
}

INDEX_EXCLUSIONS_PENDING = {
    "artifact-index.json",
    "metadata.json",
    "pending-metadata.json",
    "manual-inspection.json",
}
INDEX_EXCLUSIONS_FINAL = {"artifact-index.json", "metadata.json"}

DURABLE_PATHS = {
    "artifact-index.json",
    "capture-runtime.json",
    "decoded/video_midpoint_t3.0.png",
    "fig07_literal_wedge_stability.mp4",
    "fig07_literal_wedge_timeline.compose.json",
    "fig07_literal_wedge_timeline.png",
    "fig07_literal_wedge_timeline.verdict.json",
    "frame-validation.json",
    "metadata.json",
    "pending-metadata.json",
    "provenance.json",
    "reference-fbf-paper-trace.csv",
    "trace-equivalence.json",
    "trajectory.csv",
    "video-probe.json",
    *STILL_PATH_BY_STEP.values(),
}
FINAL_DURABLE_PATHS = DURABLE_PATHS | {"manual-inspection.json"}
ALLOWED_DURABLE_DIRECTORIES = {"decoded", "stills"}

STAGING_LOG_PATHS = {
    "capture.stderr.txt",
    "capture.stdout.txt",
    "ffmpeg-midpoint.stderr.txt",
    "ffmpeg-midpoint.stdout.txt",
    "ffmpeg.stderr.txt",
    "ffmpeg.stdout.txt",
    "image-compose.stderr.txt",
    "image-compose.stdout.txt",
    "reference-fbf-paper-trace.stderr.txt",
    "verification.stderr.txt",
}
STAGING_DIRECTORIES = {"frames", "panel_frames"}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def _is_sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _payload_sha256(payload: Any) -> str:
    encoded = json.dumps(
        payload,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _require_bundle_root(path: Path, *, create: bool) -> Path:
    original = Path(path)
    absolute = Path(os.path.abspath(original))
    if original.is_symlink():
        raise ValueError(f"literal-wedge bundle root is a symlink: {original}")
    for ancestor in absolute.parents:
        if ancestor.is_symlink():
            raise ValueError(
                f"literal-wedge bundle root passes through a symlink: {original}"
            )
    if original.exists():
        if not original.is_dir():
            raise ValueError(
                f"literal-wedge bundle is not a regular directory: {original}"
            )
    elif create:
        original.mkdir(parents=True)
    else:
        raise ValueError(f"literal-wedge bundle directory does not exist: {original}")
    if original.is_symlink():
        raise ValueError(f"literal-wedge bundle root became a symlink: {original}")
    resolved = original.resolve(strict=True)
    if resolved != absolute:
        raise ValueError(
            f"literal-wedge bundle root passes through a symlink: {original}"
        )
    return resolved


def _bundle_entries(output: Path) -> tuple[set[str], set[str]]:
    output = _require_bundle_root(output, create=False)
    files: set[str] = set()
    directories: set[str] = set()
    for path in output.rglob("*"):
        relative = path.relative_to(output).as_posix()
        if path.is_symlink():
            raise ValueError(f"literal-wedge bundle contains a symlink: {relative}")
        if path.is_dir():
            directories.add(relative)
        elif path.is_file():
            files.add(relative)
        else:
            raise ValueError(
                f"literal-wedge bundle contains a non-regular entry: {relative}"
            )
    return files, directories


def _is_staging_file(relative: str) -> bool:
    path = Path(relative)
    return (
        relative in STAGING_LOG_PATHS
        or (len(path.parts) == 2 and path.parts[0] in STAGING_DIRECTORIES)
        or path.suffix == ".ffconcat"
    )


def _validate_bundle_paths(
    output: Path,
    *,
    complete: bool,
    final: bool,
    allow_staging: bool = False,
    allow_manual: bool = False,
) -> None:
    files, directories = _bundle_entries(output)
    expected = FINAL_DURABLE_PATHS if final else DURABLE_PATHS
    allowed_files = set(expected)
    if allow_manual:
        allowed_files.add("manual-inspection.json")
    unexpected_files = {
        relative
        for relative in files
        if relative not in allowed_files
        and not (allow_staging and _is_staging_file(relative))
    }
    allowed_directories = set(ALLOWED_DURABLE_DIRECTORIES)
    if allow_staging:
        allowed_directories.update(STAGING_DIRECTORIES)
    unexpected_directories = directories - allowed_directories
    if unexpected_files or unexpected_directories:
        raise ValueError(
            "literal-wedge bundle membership has unexpected entries: "
            f"files={sorted(unexpected_files)}, "
            f"directories={sorted(unexpected_directories)}"
        )
    if complete:
        missing = expected - files
        missing_directories = ALLOWED_DURABLE_DIRECTORIES - directories
        if missing or missing_directories:
            raise ValueError(
                "literal-wedge sealed bundle is incomplete: "
                f"files={sorted(missing)}, "
                f"directories={sorted(missing_directories)}"
            )
        surviving_staging = sorted(
            relative for relative in files if _is_staging_file(relative)
        )
        surviving_staging_directories = sorted(directories & STAGING_DIRECTORIES)
        if surviving_staging or surviving_staging_directories:
            raise ValueError(
                "literal-wedge ignored staging survived sealing: "
                f"files={surviving_staging}, "
                f"directories={surviving_staging_directories}"
            )


def _record_staging(output: Path) -> dict[str, Any]:
    files, directories = _bundle_entries(output)
    staging_files = sorted(relative for relative in files if _is_staging_file(relative))
    records = [
        {
            "path": relative,
            "bytes": (output / relative).stat().st_size,
            "sha256": sha256(output / relative),
        }
        for relative in staging_files
    ]
    return {
        "staging_pruned": False,
        "retained_after_seal": False,
        "files": records,
        "file_count": len(records),
        "record_sha256": _payload_sha256(records),
        "directories": sorted(directories & STAGING_DIRECTORIES),
    }


def _validate_staging_provenance(
    staging: dict[str, Any], frame_validation: dict[str, Any]
) -> None:
    records = staging.get("files")
    if (
        staging.get("staging_pruned") is not True
        or staging.get("retained_after_seal") is not False
        or not isinstance(records, list)
        or staging.get("file_count") != len(records)
        or staging.get("record_sha256") != _payload_sha256(records)
    ):
        raise ValueError("capture staging provenance is incomplete")
    paths: list[str] = []
    by_path: dict[str, dict[str, Any]] = {}
    for item in records:
        if (
            not isinstance(item, dict)
            or set(item) != {"path", "bytes", "sha256"}
            or not isinstance(item["path"], str)
            or not _is_staging_file(item["path"])
            or not isinstance(item["bytes"], int)
            or item["bytes"] < 0
            or not _is_sha256(item["sha256"])
        ):
            raise ValueError("malformed capture staging provenance record")
        paths.append(item["path"])
        by_path[item["path"]] = item
    if paths != sorted(set(paths)):
        raise ValueError("capture staging provenance paths are not unique and sorted")
    for frame in frame_validation.get("frames", []):
        record = by_path.get(frame.get("path"))
        if record != {
            "path": frame.get("path"),
            "bytes": frame.get("bytes"),
            "sha256": frame.get("sha256"),
        }:
            raise ValueError(
                f"raw frame staging provenance changed: {frame.get('path')}"
            )


def _prune_staging(output: Path) -> None:
    files, _ = _bundle_entries(output)
    for relative in files:
        if _is_staging_file(relative):
            (output / relative).unlink()
    for relative in STAGING_DIRECTORIES:
        path = output / relative
        if path.is_dir() and not path.is_symlink():
            shutil.rmtree(path)
    files, directories = _bundle_entries(output)
    surviving = sorted(relative for relative in files if _is_staging_file(relative))
    if surviving or directories & STAGING_DIRECTORIES:
        raise ValueError(
            "literal-wedge ignored staging survived pruning: "
            f"files={surviving}, "
            f"directories={sorted(directories & STAGING_DIRECTORIES)}"
        )


@contextlib.contextmanager
def _bundle_transaction(output: Path):
    output = _require_bundle_root(output, create=False)
    with tempfile.TemporaryDirectory(
        prefix=f".{output.name}.backup-", dir=output.parent
    ) as temporary:
        snapshot = Path(temporary) / "bundle"
        shutil.copytree(output, snapshot)
        try:
            yield
        except BaseException:
            if output.exists():
                shutil.rmtree(output)
            shutil.copytree(snapshot, output)
            raise


def run(
    command: Sequence[str | Path],
    *,
    cwd: Path = ROOT,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(item) for item in command],
        cwd=cwd,
        text=True,
        capture_output=True,
        check=False,
    )


def git_output(*args: str) -> str:
    result = run(("git", *args))
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or "git command failed")
    return result.stdout.strip()


def command_record(command: Sequence[str | Path]) -> dict[str, Any]:
    executable = str(command[0])
    resolved = shutil.which(executable) if "/" not in executable else executable
    if resolved is None or not Path(resolved).exists():
        return {
            "command": [str(item) for item in command],
            "available": False,
            "returncode": None,
            "stdout": "",
            "stderr": "executable not found",
        }
    result = run(command)
    return {
        "command": [str(item) for item in command],
        "available": True,
        "executable": str(Path(resolved).resolve()),
        "executable_sha256": sha256(Path(resolved).resolve()),
        "returncode": result.returncode,
        "stdout": result.stdout,
        "stderr": result.stderr,
    }


def read_cmake_cache(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if not line or line.startswith(("#", "//")) or "=" not in line:
            continue
        key_and_type, value = line.split("=", 1)
        key = key_and_type.split(":", 1)[0]
        values[key] = value
    return values


def collect_environment_provenance(
    *, capture: Path, ffmpeg: Path, ffprobe: Path
) -> dict[str, Any]:
    cache = read_cmake_cache(CMAKE_CACHE)
    selected_cache_keys = (
        "CMAKE_BUILD_TYPE",
        "CMAKE_CXX_COMPILER",
        "CMAKE_CXX_FLAGS",
        "CMAKE_CXX_FLAGS_RELEASE",
        "CMAKE_GENERATOR",
        "CMAKE_MAKE_PROGRAM",
        "DART_BUILD_GUI",
        "DART_BUILD_GUI_OSG",
        "DART_DISABLE_COMPILER_CACHE",
        "DART_ENABLE_GUI_OSG_SMOKE_TESTS",
    )
    compiler = Path(cache["CMAKE_CXX_COMPILER"])
    display_environment = {
        key: os.environ.get(key)
        for key in (
            "DISPLAY",
            "WAYLAND_DISPLAY",
            "XDG_SESSION_TYPE",
            "XDG_CURRENT_DESKTOP",
            "LIBGL_ALWAYS_SOFTWARE",
            "MESA_LOADER_DRIVER_OVERRIDE",
        )
    }
    osg_cache = {
        key: value
        for key, value in sorted(cache.items())
        if "OSG" in key.upper() or "OPENSCENEGRAPH" in key.upper()
    }
    return {
        "build": {
            "build_directory": str(BUILD_DIRECTORY),
            "cmake_cache_path": str(CMAKE_CACHE),
            "cmake_cache_sha256": sha256(CMAKE_CACHE),
            "cmake_cache": {key: cache.get(key) for key in selected_cache_keys},
            "cmake": command_record((shutil.which("cmake") or "cmake", "--version")),
            "compiler": command_record((compiler, "--version")),
            "ninja": command_record(
                (cache.get("CMAKE_MAKE_PROGRAM", "ninja"), "--version")
            ),
        },
        "render": {
            "display_environment": display_environment,
            "osg_cmake_cache": osg_cache,
            "osgversion": command_record(("osgversion",)),
            "osg_pkg_config": command_record(
                ("pkg-config", "--modversion", "openscenegraph-osg")
            ),
            "capture_ldd": command_record(("ldd", capture)),
            "xdpyinfo": command_record(("xdpyinfo",)),
            "xrandr": command_record(("xrandr", "--current")),
            "glxinfo": command_record(("glxinfo", "-B")),
        },
        "media_tools": {
            "ffmpeg": command_record((ffmpeg, "-version")),
            "ffprobe": command_record((ffprobe, "-version")),
        },
        "system": {
            "uname": command_record(("uname", "-a")),
            "libc": command_record(("getconf", "GNU_LIBC_VERSION")),
        },
    }


def ensure_empty_output(output: Path) -> Path:
    output = _require_bundle_root(output, create=True)
    if any(output.iterdir()):
        raise ValueError(f"output directory is not empty: {output}")
    return output


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def require_keys(record: dict[str, Any], required: Iterable[str], label: str) -> None:
    missing = sorted(set(required) - set(record))
    if missing:
        raise ValueError(f"{label} is missing fields: {', '.join(missing)}")


def validate_runtime(
    runtime: dict[str, Any],
    *,
    steps: int,
    stride: int,
    width: int,
    height: int,
    threads: int,
) -> None:
    require_keys(
        runtime,
        (
            "schema_version",
            "claim_scope",
            "scenario",
            "scene_contract",
            "solver",
            "solver_contract",
            "collision_frontend",
            "contact_manifold_mode",
            "dt",
            "friction",
            "density",
            "stone_count",
            "pinned_springers",
            "end_face_expansion_m",
            "downward_shift_m",
            "initial_physical_geometry_fingerprint",
            "exact_options_summary",
            "steps_completed",
            "frame_stride",
            "simulation_threads",
            "width",
            "height",
            "outcome",
            "frames",
        ),
        "capture runtime",
    )
    expected = {
        "schema_version": "dart.fbf_literal_wedge_visual_runtime/v2",
        "scenario": "masonry_arch_25_literal_wedge",
        "scene_contract": (
            "reconstructed_literal_wedge_arch_nonpaper_native_collision_frontend"
        ),
        "solver": "exact_fbf",
        "solver_contract": "dart_best_nonpaper_colored_inner_bgs",
        "collision_frontend": "native",
        "contact_manifold_mode": "four_point_planar",
        "solver_contract_argument": "dart_best_colored_bgs",
        "steps_completed": steps,
        "frame_stride": stride,
        "simulation_threads": threads,
        "width": width,
        "height": height,
        "stone_count": 25,
        "pinned_springers": [0, 24],
    }
    mismatches = {
        key: {"expected": value, "actual": runtime.get(key)}
        for key, value in expected.items()
        if runtime.get(key) != value
    }
    if mismatches:
        raise ValueError(f"capture runtime contract mismatch: {mismatches}")

    expected_scalars = {
        "dt": 1.0 / 60.0,
        "friction": 0.8,
        "density": 1000.0,
        "end_face_expansion_m": 1e-6,
        "downward_shift_m": 0.001001,
    }
    for key, expected_value in expected_scalars.items():
        actual_value = float(runtime[key])
        if not math.isclose(actual_value, expected_value, rel_tol=0.0, abs_tol=1e-15):
            raise ValueError(
                f"capture runtime {key}={actual_value} != {expected_value}"
            )

    expected_exact_options = {
        "max_outer_iterations": 5000,
        "tolerance": 1e-6,
        "inner_max_sweeps": 30,
        "run_fixed_inner_sweeps": True,
        "inner_local_solver": "exact_metric_projection",
        "inner_local_iterations": 1,
        "step_size_scale": 35.0,
        "outer_relaxation": 1.1,
        "enable_warm_start": True,
        "enable_step_size_persistence": False,
        "fallback_to_boxed_lcp": False,
    }
    fingerprint = runtime["initial_physical_geometry_fingerprint"]
    if fingerprint != {
        "algorithm": "fnv1a64_q1e-10_le_v1",
        "value": "1ff65f2a99ec96d1",
    }:
        raise ValueError(
            f"capture runtime initial physical geometry changed: {fingerprint}"
        )
    exact_options = runtime["exact_options_summary"]
    expected_scope = (
        "selected evidence controls only; the shared "
        "FbfLiteralMasonryArchSpec.hpp contract is authoritative"
    )
    if exact_options.get("scope") != expected_scope:
        raise ValueError("capture exact-option summary scope changed")
    option_mismatches = {
        key: {"expected": value, "actual": exact_options.get(key)}
        for key, value in expected_exact_options.items()
        if exact_options.get(key) != value
    }
    if option_mismatches:
        raise ValueError(f"capture exact-option mismatch: {option_mismatches}")

    outcome = runtime["outcome"]
    if outcome.get("exact_attempts") != steps or outcome.get("exact_solves") != steps:
        raise ValueError(
            f"capture runtime exact attempt/solve totals drifted: {outcome}"
        )
    if outcome.get("exact_failures") != 0 or outcome.get("boxed_lcp_fallbacks") != 0:
        raise ValueError(f"capture runtime used a failed/fallback solve: {outcome}")
    residual = float(outcome["worst_exact_residual"])
    if not math.isfinite(residual) or residual > 1e-6:
        raise ValueError(f"capture runtime residual gate failed: {residual}")
    displacement = float(outcome["max_arch_body_displacement_from_initial"])
    alignment = float(outcome["min_arch_body_orientation_alignment_from_initial"])
    if not math.isfinite(displacement) or displacement > 1e-3:
        raise ValueError(f"capture runtime displacement gate failed: {displacement}")
    if not math.isfinite(alignment) or alignment < 0.999:
        raise ValueError(f"capture runtime orientation gate failed: {alignment}")


def validate_frames(
    output: Path, runtime: dict[str, Any], width: int, height: int
) -> dict[str, Any]:
    frames: list[dict[str, Any]] = []
    pixel_hashes: set[str] = set()
    for item in runtime["frames"]:
        relative = Path(item["path"])
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError(f"unsafe frame path in runtime sidecar: {relative}")
        path = output / relative
        image = read_image(path)
        if image.width != width or image.height != height:
            raise ValueError(
                f"{path}: expected {width}x{height}, got {image.width}x{image.height}"
            )
        non_blank = analyze_non_blank(image)
        if not non_blank["pass"]:
            raise ValueError(f"{path}: blank-frame gate failed: {non_blank}")
        pixel_sha = hashlib.sha256(image.pixels).hexdigest()
        pixel_hashes.add(pixel_sha)
        frames.append(
            {
                "step": int(item["step"]),
                "path": str(relative),
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
                "pixel_sha256": pixel_sha,
                "non_blank": non_blank,
                "contrast": analyze_contrast(image),
            }
        )

    expected_steps = list(
        range(0, int(runtime["steps_completed"]) + 1, int(runtime["frame_stride"]))
    )
    if expected_steps[-1] != int(runtime["steps_completed"]):
        expected_steps.append(int(runtime["steps_completed"]))
    actual_steps = [item["step"] for item in frames]
    if actual_steps != expected_steps:
        raise ValueError(
            f"frame timeline mismatch: expected {expected_steps}, got {actual_steps}"
        )
    return {
        "schema_version": "dart.fbf_literal_wedge_frame_validation/v1",
        "pass": True,
        "frame_count": len(frames),
        "unique_pixel_frames": len(pixel_hashes),
        "motion_required": False,
        "motion_note": (
            "This no-projectile trajectory is a stability capture; visible motion is "
            "not required and may be below pixel resolution."
        ),
        "frames": frames,
    }


def validate_frame_validation_contract(
    runtime: dict[str, Any], frame_validation: dict[str, Any]
) -> dict[int, dict[str, Any]]:
    if (
        frame_validation.get("schema_version")
        != "dart.fbf_literal_wedge_frame_validation/v1"
        or frame_validation.get("pass") is not True
        or frame_validation.get("motion_required") is not False
    ):
        raise ValueError("stored frame-validation contract is not passing")
    frames = frame_validation.get("frames")
    runtime_frames = runtime.get("frames")
    if not isinstance(frames, list) or not isinstance(runtime_frames, list):
        raise ValueError("frame-validation/runtime frame records are missing")
    if frame_validation.get("frame_count") != len(frames) or len(frames) != len(
        runtime_frames
    ):
        raise ValueError("frame-validation/runtime frame count mismatch")

    expected_steps = list(
        range(
            0,
            int(runtime["steps_completed"]) + 1,
            int(runtime["frame_stride"]),
        )
    )
    if expected_steps[-1] != int(runtime["steps_completed"]):
        expected_steps.append(int(runtime["steps_completed"]))
    by_step: dict[int, dict[str, Any]] = {}
    for expected_step, runtime_frame, frame in zip(
        expected_steps, runtime_frames, frames
    ):
        expected_path = f"frames/step_{expected_step:06d}.png"
        if (
            runtime_frame != {"step": expected_step, "path": expected_path}
            or frame.get("step") != expected_step
            or frame.get("path") != expected_path
            or not _is_sha256(frame.get("sha256"))
            or not _is_sha256(frame.get("pixel_sha256"))
            or not isinstance(frame.get("bytes"), int)
            or frame["bytes"] <= 0
            or frame.get("non_blank", {}).get("pass") is not True
        ):
            raise ValueError(f"frame-validation record changed at step {expected_step}")
        by_step[expected_step] = frame
    if len(by_step) != len(expected_steps):
        raise ValueError("frame-validation steps are duplicated or incomplete")
    unique_pixels = len({frame["pixel_sha256"] for frame in frames})
    if frame_validation.get("unique_pixel_frames") != unique_pixels:
        raise ValueError("frame-validation unique-pixel count changed")
    return by_step


def promote_durable_stills(
    output: Path,
    runtime: dict[str, Any],
    frame_validation: dict[str, Any],
) -> list[dict[str, Any]]:
    by_step = validate_frame_validation_contract(runtime, frame_validation)
    bindings: list[dict[str, Any]] = []
    for step in TIMELINE_STILL_STEPS:
        frame = by_step.get(step)
        if frame is None:
            raise ValueError(f"capture lacks required durable still step {step}")
        source = output / frame["path"]
        if (
            not source.is_file()
            or source.is_symlink()
            or source.stat().st_size != frame["bytes"]
            or sha256(source) != frame["sha256"]
        ):
            raise ValueError(f"staging frame changed before promotion at step {step}")
        destination = output / STILL_PATH_BY_STEP[step]
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.is_symlink():
            raise ValueError(f"durable still is a symlink: {destination}")
        shutil.copyfile(source, destination)
        bindings.append(
            {
                "step": step,
                "path": STILL_PATH_BY_STEP[step],
                "source_frame_path": frame["path"],
                "bytes": frame["bytes"],
                "sha256": frame["sha256"],
                "pixel_sha256": frame["pixel_sha256"],
            }
        )
    validate_durable_stills(output, runtime, frame_validation, bindings)
    return bindings


def validate_durable_stills(
    output: Path,
    runtime: dict[str, Any],
    frame_validation: dict[str, Any],
    expected_bindings: Any,
) -> list[dict[str, Any]]:
    by_step = validate_frame_validation_contract(runtime, frame_validation)
    bindings: list[dict[str, Any]] = []
    for step in TIMELINE_STILL_STEPS:
        frame = by_step.get(step)
        if frame is None:
            raise ValueError(f"frame-validation lacks durable still step {step}")
        relative = STILL_PATH_BY_STEP[step]
        path = output / relative
        if (
            not path.is_file()
            or path.is_symlink()
            or path.stat().st_size != frame["bytes"]
            or sha256(path) != frame["sha256"]
        ):
            raise ValueError(f"durable still differs from frame-validation step {step}")
        image = read_image(path)
        if (
            image.width != int(runtime["width"])
            or image.height != int(runtime["height"])
            or hashlib.sha256(image.pixels).hexdigest() != frame["pixel_sha256"]
        ):
            raise ValueError(f"durable still pixel binding changed at step {step}")
        bindings.append(
            {
                "step": step,
                "path": relative,
                "source_frame_path": frame["path"],
                "bytes": frame["bytes"],
                "sha256": frame["sha256"],
                "pixel_sha256": frame["pixel_sha256"],
            }
        )
    if expected_bindings != bindings:
        raise ValueError("durable still provenance bindings changed")
    return bindings


def trace_command(trace: Path, steps: int, threads: int) -> list[str]:
    return [
        str(trace),
        "masonry_arch_25_literal_wedge",
        "exact_fbf",
        "1",
        str(steps),
        "nan",
        "performance",
        "default",
        "default",
        str(threads),
        "dart_best_colored_bgs",
        "native",
        "default",
    ]


def validate_capture_source_contract() -> dict[str, Any]:
    source_path = ROOT / CAPTURE_SOURCE_RELATIVE
    shared_header_path = ROOT / LITERAL_ARCH_SPEC_SOURCE_RELATIVE
    trace_path = ROOT / TRACE_SOURCE_RELATIVE
    source = " ".join(source_path.read_text(encoding="utf-8").split())
    shared_header = " ".join(shared_header_path.read_text(encoding="utf-8").split())
    trace = " ".join(trace_path.read_text(encoding="utf-8").split())

    consumer_required_fragments = {
        "shared_header_include": (
            '#include "../../../examples/demos/scenes/' 'FbfLiteralMasonryArchSpec.hpp"'
        ),
        "shared_namespace": "namespace literalArch = fbf_literal_masonry_arch;",
        "shared_dt": "constexpr double kDt = literalArch::kTimeStep;",
        "shared_friction": "constexpr double kFriction = literalArch::kFriction;",
        "shared_density": "constexpr double kDensity = literalArch::kDensity;",
        "shared_stone_count": (
            "constexpr std::size_t kStoneCount = literalArch::kStoneCount;"
        ),
        "shared_end_face_expansion": (
            "constexpr double kEndFaceExpansion = literalArch::kEndFaceExpansion;"
        ),
        "shared_downward_shift": (
            "constexpr double kDownwardShift = literalArch::kDownwardShift;"
        ),
        "shared_tolerance": (
            "constexpr double kTolerance = literalArch::kResidualTolerance;"
        ),
        "exact_fbf_demo_palette_world": (
            "literalArch::createWorld( literalArch::SolverLane::ExactFbf, "
            "literalArch::VisualMode::DemoPalette, simulationThreads);"
        ),
        "legacy_world_name": ('world->setName("fbf_literal_wedge_visual_capture");'),
        "scoped_erp_owns_run": (
            "literalArch::ScopedContactErrorReductionParameter scopedContactErp; "
            "auto world = createCaptureWorld(config.simulationThreads);"
        ),
        "observed_initial_fingerprint": (
            "literalArch::physicalGeometryFingerprint( "
            "literalArch::inspectPhysicsContract(world));"
        ),
        "exact_options_are_labeled_summary": '\\"exact_options_summary\\"',
    }
    consumer_forbidden_fragments = {
        "duplicate_contact_erp_mutation": (
            "dart::constraint::ContactConstraint::setErrorReductionParameter("
        ),
        "duplicate_exact_options": "ExactCoulombFbfConstraintSolverOptions options;",
        "duplicate_literal_wedge_generator": "generateMasonryArchStoneWedges(",
        "duplicate_native_detector": "NativeCollisionDetector::create()",
        "duplicate_ground_builder": "SkeletonPtr createGround(",
        "duplicate_stone_builder": "SkeletonPtr createStone(",
    }
    shared_required_fragments = {
        "dt": "inline constexpr double kTimeStep = 1.0 / 60.0;",
        "gravity": "inline constexpr double kGravity = 9.81;",
        "friction": "inline constexpr double kFriction = 0.8;",
        "density": "inline constexpr double kDensity = 1000.0;",
        "stone_count": "inline constexpr std::size_t kStoneCount = 25u;",
        "max_contacts": "inline constexpr std::size_t kMaxContacts = 400u;",
        "max_contacts_per_pair": (
            "inline constexpr std::size_t kMaxContactsPerPair = 8u;"
        ),
        "end_face_expansion": ("inline constexpr double kEndFaceExpansion = 1e-6;"),
        "downward_shift": "inline constexpr double kDownwardShift = 0.001001;",
        "erp_zero": (
            "inline constexpr double kDesiredContactErrorReductionParameter = 0.0;"
        ),
        "barrier_offsets_omitted": ("MasonryArchBarrierGapPolicy::OmitSourceOffsets;"),
        "scoped_erp_type": "class ScopedContactErrorReductionParameter",
        "scoped_erp_capture": (
            "mPreviousValue( dart::constraint::ContactConstraint::"
            "getErrorReductionParameter())"
        ),
        "scoped_erp_apply": (
            "dart::constraint::ContactConstraint::setErrorReductionParameter( "
            "kDesiredContactErrorReductionParameter);"
        ),
        "scoped_erp_restore": (
            "~ScopedContactErrorReductionParameter() { "
            "dart::constraint::ContactConstraint::setErrorReductionParameter( "
            "mPreviousValue); }"
        ),
        "scoped_erp_noncopyable": (
            "ScopedContactErrorReductionParameter( const "
            "ScopedContactErrorReductionParameter&) = delete;"
        ),
        "max_outer_iterations": ("options.maxOuterIterations = kMaxOuterIterations;"),
        "accept_outer_cap": "options.acceptOuterMaxIterations = true;",
        "tolerance": "options.tolerance = kResidualTolerance;",
        "inner_sweeps": "options.innerMaxSweeps = kInnerMaxSweeps;",
        "fixed_inner_sweeps": "options.runFixedInnerSweeps = true;",
        "exact_metric_local_solver": (
            "ExactCoulombFbfLocalBlockSolver:: ExactMetricProjection;"
        ),
        "one_local_iteration": (
            "options.innerLocalIterations = kInnerLocalIterations;"
        ),
        "step_size_scale": "options.stepSizeScale = kStepSizeScale;",
        "adaptive_step_size": "options.enableAdaptiveStepSize = true;",
        "outer_relaxation": "options.outerRelaxation = kOuterRelaxation;",
        "warm_start": "options.enableWarmStart = true;",
        "no_projected_gradient_retry": (
            "options.enableProjectedGradientRetry = false;"
        ),
        "no_dense_polish": "options.enableDenseResidualPolish = false;",
        "no_boxed_fallback": "options.fallbackToBoxedLcp = false;",
        "zero_normal_seed": "options.seedNormalImpulseFromDiagonal = false;",
        "contact_row_operator": "options.useContactRowDelassusOperator = true;",
        "zero_matrix_free_operator": ("options.useMatrixFreeDelassusOperator = false;"),
        "zero_matrix_free_seed": "options.useMatrixFreeDelassusSeed = false;",
        "no_dense_snapshot": "options.assembleDenseContactRowSnapshot = false;",
        "fresh_step_size": "options.enableStepSizePersistence = false;",
        "cross_step_policy_installed": (
            "installed->setExactCoulombCrossStepPolicyOptions( "
            "makeExactCrossStepPolicyOptions());"
        ),
        "contact_dynamics_explicit": (
            "dynamics->setFrictionCoeff(kFriction); "
            "dynamics->setRestitutionCoeff(0.0); "
            "dynamics->setPrimarySlipCompliance(-1.0); "
            "dynamics->setSecondarySlipCompliance(-1.0); "
            "dynamics->setFirstFrictionDirection(Eigen::Vector3d::Zero());"
        ),
        "physical_geometry_fingerprint": (
            "inline std::string physicalGeometryFingerprint("
            "const PhysicsContract& contract)"
        ),
        "literal_wedges": (
            "generateMasonryArchStoneWedges( kStoneCount, {}, kBarrierGapPolicy, "
            "kEndFaceExpansion);"
        ),
        "centroid_relative_mesh": "vertices.push_back(vertex - geometry.centroid);",
        "visual_only_ground": (
            "body->createShapeNodeWith<dart::dynamics::VisualAspect>( "
            "std::make_shared<dart::dynamics::BoxShape>("
        ),
        "palette_collision_mesh": (
            "dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, "
            "dart::dynamics::DynamicsAspect>(shape);"
        ),
        "exact_prism_mass": "const double mass = kDensity * geometry.volume;",
        "exact_prism_inertia": (
            "inertia.setMoment(mass * geometry.momentPerUnitMass);"
        ),
        "initial_transform": (
            "geometry.centroid - kDownwardShift * Eigen::Vector3d::UnitZ();"
        ),
        "pinned_springers": (
            "if (index == 0u || index + 1u == kStoneCount) "
            "skeleton->setMobile(false);"
        ),
        "native_frontend": "NativeCollisionDetector::create();",
        "four_point_planar": "ContactManifoldMode::FourPointPlanar",
        "colored_bgs_enabled": (
            "installed->setExactCoulombColoredBlockGaussSeidelEnabled(true);"
        ),
        "participant_affinity_enabled": (
            "installed->setExactCoulombColoredBlockGaussSeidelParticipantAffinityEnabled( "
            "true);"
        ),
        "split_impulse": "installed->setSplitImpulseEnabled(true);",
        "installed_max_contacts": ("collisionOption.maxNumContacts = kMaxContacts;"),
        "installed_max_contacts_per_pair": (
            "collisionOption.maxNumContactsPerPair = kMaxContactsPerPair;"
        ),
        "world_dt": "world->setTimeStep(kTimeStep);",
        "world_gravity": ("world->setGravity(Eigen::Vector3d(0.0, 0.0, -kGravity));"),
        "world_threads": "world->setNumSimulationThreads(simulationThreads);",
        "deactivation_disabled": "deactivation.mEnabled = false;",
        "world_solver": "installSolver(world, lane, simulationThreads);",
        "world_ground": "world->addSkeleton(createGround(visualMode));",
        "world_stones": (
            "world->addSkeleton(createStone(index, geometries[index], visualMode));"
        ),
    }
    trace_required_fragments = {
        "shared_header_include": (
            '#include "../../../examples/demos/scenes/' 'FbfLiteralMasonryArchSpec.hpp"'
        ),
        "exact_25_native_colored_predicate": (
            "scenario == Scenario::MasonryArch25LiteralWedge && solverMode == "
            "SolverMode::ExactFbf && contract == "
            "SolverContract::DartBestColoredBgs && collisionFrontend == "
            "CollisionFrontend::Native && gNativeManifoldSensitivitySelector "
            "== NativeManifoldSensitivitySelector::Default"
        ),
        "shared_physical_world": (
            "auto world = shared::createWorld( shared::SolverLane::ExactFbf, "
            "shared::VisualMode::None, simulationThreads);"
        ),
        "preserved_trace_identity": (
            'std::string("exact_coulomb_trace_") + '
            "scenarioName(Scenario::MasonryArch25LiteralWedge)"
        ),
        "shared_path_dispatch": (
            "return createSharedLiteralMasonryArch25StandingWorld( traceScope, "
            "initialStepSize, simulationThreads);"
        ),
        "process_owned_erp": (
            "dart::constraint::ContactConstraint::setErrorReductionParameter( "
            "shared::kDesiredContactErrorReductionParameter);"
        ),
    }
    trace_forbidden_fragments = {
        "duplicate_literal_builder": "addLiteralMasonryArch25(",
        "duplicate_wedge_generator": "generateMasonryArchStoneWedges(",
        "duplicate_exact_solver": (
            "std::make_unique<dart::constraint::" "ExactCoulombFbfConstraintSolver>"
        ),
        "duplicate_native_detector": "NativeCollisionDetector::create()",
    }

    consumer_missing = [
        name
        for name, fragment in consumer_required_fragments.items()
        if fragment not in source
    ]
    shared_missing = [
        name
        for name, fragment in shared_required_fragments.items()
        if fragment not in shared_header
    ]
    trace_missing = [
        name
        for name, fragment in trace_required_fragments.items()
        if fragment not in trace
    ]
    consumer_forbidden = [
        name
        for name, fragment in consumer_forbidden_fragments.items()
        if fragment in source
    ]
    world_builder = shared_header.split(
        "inline dart::simulation::WorldPtr createWorld(", 1
    )[-1].split("struct StonePhysicsContract", 1)[0]
    shared_forbidden = []
    if "ContactConstraint::setErrorReductionParameter(" in world_builder:
        shared_forbidden.append("world_builder_mutates_process_erp")
    trace_shared_builder = trace.split(
        "createSharedLiteralMasonryArch25StandingWorld(", 1
    )[-1].split("std::shared_ptr<dart::simulation::World> createTraceWorld(", 1)[0]
    trace_forbidden = [
        name
        for name, fragment in trace_forbidden_fragments.items()
        if fragment in trace_shared_builder
    ]
    missing = [
        *(f"capture_consumer:{name}" for name in consumer_missing),
        *(f"shared_header:{name}" for name in shared_missing),
        *(f"trace_consumer:{name}" for name in trace_missing),
    ]
    forbidden = [
        *(f"capture_consumer:{name}" for name in consumer_forbidden),
        *(f"shared_header:{name}" for name in shared_forbidden),
        *(f"trace_consumer:{name}" for name in trace_forbidden),
    ]
    return {
        "pass": not missing and not forbidden,
        "source_path": str(source_path.relative_to(ROOT)),
        "source_sha256": sha256(source_path),
        "shared_header_path": str(shared_header_path.relative_to(ROOT)),
        "shared_header_sha256": sha256(shared_header_path),
        "trace_path": str(trace_path.relative_to(ROOT)),
        "trace_sha256": sha256(trace_path),
        "required_fragments": {
            "capture_consumer": consumer_required_fragments,
            "shared_header": shared_required_fragments,
            "trace_consumer": trace_required_fragments,
        },
        "forbidden_fragments": {
            "capture_consumer": consumer_forbidden_fragments,
            "shared_header": {
                "world_builder_mutates_process_erp": (
                    "ContactConstraint::setErrorReductionParameter("
                )
            },
            "trace_consumer": trace_forbidden_fragments,
        },
        "missing": missing,
        "forbidden": forbidden,
        "capture_consumer_binding": {
            "pass": not consumer_missing and not consumer_forbidden,
            "missing": consumer_missing,
            "forbidden": consumer_forbidden,
        },
        "shared_header_contract": {
            "pass": not shared_missing and not shared_forbidden,
            "missing": shared_missing,
            "forbidden": shared_forbidden,
        },
        "trace_consumer_binding": {
            "pass": not trace_missing and not trace_forbidden,
            "missing": trace_missing,
            "forbidden": trace_forbidden,
        },
        "asserted_contract": {
            "dt": 1.0 / 60.0,
            "gravity": [0.0, 0.0, -9.81],
            "deactivation_enabled": False,
            "friction": 0.8,
            "primary_friction": 0.8,
            "secondary_friction": 0.8,
            "restitution": 0.0,
            "slip_compliance": [-1.0, -1.0],
            "first_friction_direction": [0.0, 0.0, 0.0],
            "density": 1000.0,
            "stone_count": 25,
            "pinned_springers": [0, 24],
            "barrier_gap_policy": "omit_source_offsets",
            "end_face_expansion_m": 1e-6,
            "downward_shift_m": 0.001001,
            "max_contacts": 400,
            "max_contacts_per_pair": 8,
            "split_impulse": True,
            "contact_erp": 0.0,
            "physical_geometry_fingerprint": "1ff65f2a99ec96d1",
            "colored_bgs": True,
            "participant_affinity": True,
            "max_outer_iterations": 5000,
            "accept_outer_max_iterations": True,
            "tolerance": 1e-6,
            "inner_max_sweeps": 30,
            "fixed_inner_sweeps": True,
            "inner_local_solver": "exact_metric_projection",
            "inner_local_iterations": 1,
            "step_size_scale": 35.0,
            "adaptive_step_size": True,
            "outer_relaxation": 1.1,
            "warm_start": True,
            "projected_gradient_retry": False,
            "dense_residual_polish": False,
            "boxed_lcp_fallback": False,
            "diagonal_seed": False,
            "matrix_free_seed": False,
            "dense_contact_snapshot": False,
            "step_size_persistence": False,
        },
    }


def compare_trace(
    capture_rows: list[dict[str, str]], reference_rows: list[dict[str, str]]
) -> dict[str, Any]:
    if not capture_rows or len(capture_rows) != len(reference_rows) + 1:
        raise ValueError(
            "capture/reference row count mismatch: "
            f"capture={len(capture_rows)}, reference={len(reference_rows)}"
        )
    if capture_rows[0].get("step") != "0":
        raise ValueError("capture trajectory is missing its constructed-t0 row")
    capture_rows = capture_rows[1:]

    integer_fields = {
        "contacts": "contacts",
        "exact_solves_delta": "step_exact_solves",
        "exact_failures_delta": "step_exact_failures",
        "boxed_lcp_fallbacks_delta": "step_fallbacks",
        "iterations": "step_fbf_iterations",
        "colored_bgs_used": "last_exact_colored_bgs_used",
        "colored_bgs_manifolds": "last_exact_colored_bgs_manifolds",
        "colored_bgs_colors": "last_exact_colored_bgs_colors",
        "colored_bgs_max_manifolds_per_color": (
            "last_exact_colored_bgs_max_manifolds_per_color"
        ),
    }
    float_fields = {
        "sim_time": "time",
        "residual": "residual",
        "max_arch_body_displacement_from_initial": (
            "max_arch_body_displacement_from_initial"
        ),
        "min_arch_body_orientation_alignment_from_initial": (
            "min_arch_body_orientation_alignment_from_initial"
        ),
        "crown_z": "z",
    }
    max_abs_difference = {field: 0.0 for field in float_fields}
    mismatches: list[str] = []
    zero_iteration_steps: list[int] = []
    positive_colored_solve_counts: list[int] = []
    tolerance = 2e-14
    expected_reference_text = {
        "scenario": "masonry_arch_25_literal_wedge",
        "solver": "exact_fbf",
        "solver_contract": "dart_best_nonpaper_colored_inner_bgs",
        "precision_contract": "float64",
        "scene_contract": (
            "reconstructed_literal_wedge_arch_nonpaper_native_collision_frontend"
        ),
        "baumgarte_contract": ("split_impulse_no_velocity_baumgarte_erp_zero_vs_paper"),
        "collision_frontend": "native",
        "inner_local_solver": "exact_metric",
        "row_operator_request": "contact_row_no_dense_snapshot",
        "row_operator_mode": "contact_row_no_dense_snapshot",
        "initial_gamma_contract": "automatic_safe_bound",
        "inner_bgs_schedule_contract": (
            "dart_deterministic_manifold_colored_bgs_nonpaper"
        ),
        "exact_contact_row_logical_cpus_to_date": "none",
        "max_phase_contact_row_logical_cpus_to_date": "none",
    }
    expected_reference_int = {
        "inner_sweeps_requested": 30,
        "fixed_inner_sweeps_requested": 1,
        "step_size_persistence_enabled": 0,
        "requested_threads": 1,
        "actual_threads": 1,
        "contacts": 96,
        "unique_colliding_body_pairs": 24,
        "max_outer_iterations": 5000,
        "accept_outer_max_iterations": 1,
        "inner_local_iterations": 1,
        "adaptive_step_size_enabled": 1,
        "warm_start_enabled": 1,
        "projected_gradient_retry_enabled": 0,
        "dense_residual_polish_enabled": 0,
        "fallback_to_boxed_lcp_enabled": 0,
        "diagonal_seed_enabled": 0,
        "matrix_free_seed_enabled": 0,
        "split_impulse_enabled": 1,
        "step_exact_solves": 1,
        "step_exact_failures": 0,
        "step_fallbacks": 0,
        "step_parallel_contact_row_delassus_products": 0,
        "max_contact_row_participants_to_date": 1,
        "last_exact_colored_bgs_manifolds": 24,
        "last_exact_colored_bgs_colors": 3,
        "last_exact_colored_bgs_max_manifolds_per_color": 8,
    }
    expected_reference_float = {
        "tolerance": 1e-6,
        "step_size_scale": 35.0,
        "outer_relaxation": 1.1,
    }
    for index, (capture, reference) in enumerate(
        zip(capture_rows, reference_rows, strict=True), start=1
    ):
        if int(capture["step"]) != index or int(reference["step"]) != index:
            mismatches.append(f"step {index}: row step mismatch")
            continue
        for field, expected in expected_reference_text.items():
            if reference.get(field) != expected:
                mismatches.append(
                    f"step {index}: {field}={reference.get(field)!r} != {expected!r}"
                )
        for field, expected in expected_reference_int.items():
            if int(reference[field]) != expected:
                mismatches.append(
                    f"step {index}: {field}={reference[field]} != {expected}"
                )
        for field, expected in expected_reference_float.items():
            if not math.isclose(
                float(reference[field]), expected, rel_tol=0.0, abs_tol=1e-15
            ):
                mismatches.append(
                    f"step {index}: {field}={reference[field]} != {expected}"
                )
        if (
            capture.get("exact_status") != "success"
            or reference.get("status") != "success"
        ):
            mismatches.append(
                f"step {index}: non-success status capture={capture.get('exact_status')} "
                f"reference={reference.get('status')}"
            )
        if capture.get("fbf_status") != "success":
            mismatches.append(
                f"step {index}: capture fbf_status={capture.get('fbf_status')}"
            )
        if int(capture["exact_attempts_delta"]) != 1:
            mismatches.append(
                f"step {index}: exact_attempts_delta={capture['exact_attempts_delta']} != 1"
            )
        for capture_field, reference_field in integer_fields.items():
            if int(capture[capture_field]) != int(reference[reference_field]):
                mismatches.append(
                    f"step {index}: {capture_field}={capture[capture_field]} != "
                    f"{reference_field}={reference[reference_field]}"
                )
        for capture_field, reference_field in float_fields.items():
            capture_value = float(capture[capture_field])
            reference_value = float(reference[reference_field])
            difference = abs(capture_value - reference_value)
            max_abs_difference[capture_field] = max(
                max_abs_difference[capture_field], difference
            )
            scale = max(1.0, abs(capture_value), abs(reference_value))
            if not math.isfinite(difference) or difference > tolerance * scale:
                mismatches.append(
                    f"step {index}: {capture_field} differs by {difference:.17g}"
                )
        residual = float(reference["residual"])
        if not math.isfinite(residual) or residual > 1e-6:
            mismatches.append(f"step {index}: invalid exact residual {residual}")

        iterations = int(reference["step_fbf_iterations"])
        colored_used = int(reference["last_exact_colored_bgs_used"])
        colored_solves = int(reference["last_exact_colored_bgs_solves"])
        colored_dispatches = int(reference["last_exact_colored_bgs_dispatches"])
        colored_participants = int(reference["last_exact_colored_bgs_max_participants"])
        if iterations == 0:
            zero_iteration_steps.append(index)
            if (
                int(reference["step_warm_starts"]) != 1
                or colored_used != 0
                or colored_solves != 0
                or colored_dispatches != 0
                or colored_participants != 0
                or int(reference["last_exact_colored_bgs_manifolds"]) != 24
                or int(reference["last_exact_colored_bgs_colors"]) != 3
                or int(reference["last_exact_colored_bgs_max_manifolds_per_color"]) != 8
            ):
                mismatches.append(
                    f"step {index}: zero-iteration warm-start schedule/dispatch "
                    "contract is not populated and idle"
                )
        elif (
            colored_used != 1
            or colored_solves < iterations
            or colored_solves == 0
            or colored_dispatches != 0
            or colored_participants != 1
        ):
            mismatches.append(
                f"step {index}: positive-work colored contract used={colored_used}, "
                f"solves={colored_solves}, iterations={iterations}, "
                f"dispatches={colored_dispatches}, participants={colored_participants}"
            )
        else:
            positive_colored_solve_counts.append(colored_solves)
    expected_zero_iteration_steps = [2, 3, 4, 6, 363]
    if zero_iteration_steps != expected_zero_iteration_steps:
        mismatches.append(
            "zero-iteration warm-start steps drifted: "
            f"expected {expected_zero_iteration_steps}, got {zero_iteration_steps}"
        )
    source_contract = validate_capture_source_contract()
    if not source_contract["pass"]:
        mismatches.append(
            f"capture source contract fragments missing: {source_contract['missing']}"
        )

    return {
        "schema_version": "dart.fbf_literal_wedge_trace_equivalence/v2",
        "pass": not mismatches,
        "rows_compared": len(reference_rows),
        "numeric_relative_tolerance": tolerance,
        "max_abs_difference": max_abs_difference,
        "integer_fields": integer_fields,
        "float_fields": float_fields,
        "expected_reference_text": expected_reference_text,
        "expected_reference_int": expected_reference_int,
        "expected_reference_float": expected_reference_float,
        "defining_source_contract": source_contract,
        "zero_iteration_warm_start_steps": zero_iteration_steps,
        "positive_work_colored_solves": {
            "rows": len(positive_colored_solve_counts),
            "minimum": min(positive_colored_solve_counts),
            "maximum": max(positive_colored_solve_counts),
            "total": sum(positive_colored_solve_counts),
            "contract": "positive and at least the reported FBF iteration count",
        },
        "zero_iteration_contract": {
            "expected_count": 5,
            "contacts": 96,
            "exact_solves": 1,
            "warm_starts": 1,
            "colored_used": 0,
            "colored_solves": 0,
            "colored_dispatches": 0,
            "colored_participants": 0,
            "schedule": {"manifolds": 24, "colors": 3, "max_width": 8},
        },
        "mismatches": mismatches,
        "meaning": (
            "The off-screen app and current fbf_paper_trace independently stepped "
            "the same literal Native/FourPointPlanar colored-FBF contract; rendering "
            "adds only VisualAspect state and a nonphysical floor proxy."
        ),
    }


def probe_video(
    ffprobe: Path,
    video: Path,
    *,
    expected_frames: int,
    expected_width: int,
    expected_height: int,
    expected_fps: int,
    simulation_seconds: float,
) -> dict[str, Any]:
    probed = run(
        (
            ffprobe,
            "-v",
            "error",
            "-count_frames",
            "-show_entries",
            "stream=codec_name,width,height,r_frame_rate,avg_frame_rate,nb_read_frames:format=duration,size",
            "-of",
            "json",
            video,
        )
    )
    if probed.returncode != 0:
        raise RuntimeError(f"ffprobe failed ({probed.returncode}): {probed.stderr}")
    payload = json.loads(probed.stdout)
    streams = payload.get("streams", [])
    if len(streams) != 1 or int(streams[0].get("nb_read_frames", 0)) != expected_frames:
        raise ValueError(
            f"video frame count mismatch: expected {expected_frames}, probe={streams}"
        )
    if streams[0].get("codec_name") != "h264":
        raise ValueError(f"unexpected video codec: {streams[0].get('codec_name')}")
    if (
        int(streams[0].get("width", 0)) != expected_width
        or int(streams[0].get("height", 0)) != expected_height
    ):
        raise ValueError(f"video dimensions mismatch: {streams[0]}")
    if streams[0].get("avg_frame_rate") != f"{expected_fps}/1":
        raise ValueError(f"video frame rate mismatch: {streams[0]}")
    playback_seconds = float(payload["format"]["duration"])
    payload.update(
        {
            "schema_version": "dart.fbf_literal_wedge_video_probe/v2",
            "path": video.name,
            "sha256": sha256(video),
            "pass": True,
            "timing": {
                "simulation_duration_seconds": simulation_seconds,
                "captured_frames": expected_frames,
                "playback_fps": expected_fps,
                "playback_duration_seconds": playback_seconds,
                "playback_acceleration_factor": (simulation_seconds / playback_seconds),
                "meaning": (
                    "The 10-second simulation is sampled into 61 frames and "
                    "played in 6.1 seconds; this is a time-lapse, not real time."
                ),
            },
        }
    )
    return payload


def encode_video(
    ffmpeg: Path,
    ffprobe: Path,
    output: Path,
    *,
    fps: int,
    expected_frames: int,
    expected_width: int,
    expected_height: int,
    expected_fps: int,
    simulation_seconds: float,
) -> dict[str, Any]:
    video = output / "fig07_literal_wedge_stability.mp4"
    command = [
        str(ffmpeg),
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-framerate",
        str(fps),
        "-pattern_type",
        "glob",
        "-i",
        str(output / "frames/step_*.png"),
        "-c:v",
        "libx264",
        "-crf",
        "18",
        "-preset",
        "slow",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(video),
    ]
    encoded = run(command)
    (output / "ffmpeg.stdout.txt").write_text(encoded.stdout, encoding="utf-8")
    (output / "ffmpeg.stderr.txt").write_text(encoded.stderr, encoding="utf-8")
    if encoded.returncode != 0 or not video.is_file():
        raise RuntimeError(f"ffmpeg failed ({encoded.returncode}): {encoded.stderr}")
    return probe_video(
        ffprobe,
        video,
        expected_frames=expected_frames,
        expected_width=expected_width,
        expected_height=expected_height,
        expected_fps=expected_fps,
        simulation_seconds=simulation_seconds,
    )


def decode_midpoint(
    ffmpeg: Path, output: Path, *, seconds: float, width: int, height: int
) -> dict[str, Any]:
    video = output / "fig07_literal_wedge_stability.mp4"
    decoded = output / "decoded" / "video_midpoint_t3.0.png"
    decoded.parent.mkdir(parents=True, exist_ok=True)
    command: list[str | Path] = [
        ffmpeg,
        "-hide_banner",
        "-loglevel",
        "error",
        "-ss",
        str(seconds),
        "-i",
        video,
        "-frames:v",
        "1",
        "-y",
        decoded,
    ]
    result = run(command)
    (output / "ffmpeg-midpoint.stdout.txt").write_text(result.stdout, encoding="utf-8")
    (output / "ffmpeg-midpoint.stderr.txt").write_text(result.stderr, encoding="utf-8")
    if result.returncode != 0 or not decoded.is_file():
        raise RuntimeError(
            f"midpoint decode failed ({result.returncode}): {result.stderr}"
        )
    image = read_image(decoded)
    non_blank = analyze_non_blank(image)
    if image.width != width or image.height != height or not non_blank["pass"]:
        raise ValueError(
            f"decoded midpoint failed image gate: {image.width}x{image.height}, "
            f"non_blank={non_blank}"
        )
    return {
        "path": str(decoded.relative_to(output)),
        "sha256": sha256(decoded),
        "seconds": seconds,
        "command": [str(item) for item in command],
        "width": image.width,
        "height": image.height,
        "non_blank": non_blank,
    }


def compose_timeline(output: Path, steps: int) -> dict[str, Any]:
    selected = TIMELINE_STILL_STEPS
    if steps != selected[-1]:
        raise ValueError(
            f"literal-wedge timeline requires {selected[-1]} steps, got {steps}"
        )
    inputs = [output / STILL_PATH_BY_STEP[step] for step in selected]
    panel = output / "fig07_literal_wedge_timeline.png"
    manifest = output / "fig07_literal_wedge_timeline.compose.json"
    command: list[str | Path] = [
        sys.executable,
        ROOT / "scripts/image_compose.py",
        "side-by-side",
        *inputs,
        "--out",
        panel,
        "--labels",
        *(f"t={step / 60.0:.1f}s" for step in selected),
        "--gap",
        "8",
        "--manifest",
        manifest,
    ]
    composed = run(command)
    (output / "image-compose.stdout.txt").write_text(composed.stdout, encoding="utf-8")
    (output / "image-compose.stderr.txt").write_text(composed.stderr, encoding="utf-8")
    if composed.returncode != 0:
        raise RuntimeError(
            f"image compositor failed ({composed.returncode}): {composed.stderr}"
        )
    compose_payload = json.loads(manifest.read_text(encoding="utf-8"))
    compose_payload["inputs"] = [STILL_PATH_BY_STEP[step] for step in selected]
    compose_payload["out"] = panel.name
    write_json(manifest, compose_payload)
    verdict = build_verdict(
        panel,
        metadata={
            "scenario": "masonry_arch_25_literal_wedge",
            "claim_scope": "current-source DART reconstruction; not paper parity",
        },
        require_contrast=False,
    )
    if not verdict["pass"]:
        raise ValueError(f"timeline panel image gate failed: {verdict['reasons']}")
    write_json(output / "fig07_literal_wedge_timeline.verdict.json", verdict)
    return {
        "path": panel.name,
        "sha256": sha256(panel),
        "selected_steps": list(selected),
        "compose_manifest": manifest.name,
        "compose_manifest_sha256": sha256(manifest),
    }


def artifact_index(output: Path, excluded: set[str]) -> dict[str, Any]:
    artifacts = []
    for path in sorted(item for item in output.rglob("*") if item.is_file()):
        relative = str(path.relative_to(output))
        if relative in excluded:
            continue
        artifacts.append(
            {
                "path": relative,
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
        )
    return {
        "schema_version": INDEX_SCHEMA_VERSION,
        "artifact_count": len(artifacts),
        "excluded": sorted(excluded),
        "artifacts": artifacts,
    }


def validate_artifact_index(
    output: Path, payload: dict[str, Any], *, excluded: set[str]
) -> None:
    if payload.get("schema_version") != INDEX_SCHEMA_VERSION:
        raise ValueError("unexpected literal-wedge artifact-index schema")
    if payload.get("excluded") != sorted(excluded):
        raise ValueError("literal-wedge artifact-index exclusions changed")
    artifacts = payload.get("artifacts")
    if not isinstance(artifacts, list) or payload.get("artifact_count") != len(
        artifacts
    ):
        raise ValueError("literal-wedge artifact-index count changed")
    listed: list[str] = []
    for item in artifacts:
        if not isinstance(item, dict) or set(item) != {"path", "bytes", "sha256"}:
            raise ValueError("malformed literal-wedge artifact-index entry")
        relative = item["path"]
        path = Path(relative) if isinstance(relative, str) else Path("/")
        if not isinstance(relative, str) or path.is_absolute() or ".." in path.parts:
            raise ValueError(f"unsafe literal-wedge artifact-index path: {relative!r}")
        artifact = output / relative
        if (
            not artifact.is_file()
            or artifact.is_symlink()
            or artifact.stat().st_size != item["bytes"]
            or sha256(artifact) != item["sha256"]
        ):
            raise ValueError(f"literal-wedge artifact changed: {relative}")
        listed.append(relative)
    if listed != sorted(set(listed)):
        raise ValueError("literal-wedge artifact-index paths are not unique and sorted")
    actual = {
        path.relative_to(output).as_posix()
        for path in output.rglob("*")
        if path.is_file()
        and not path.is_symlink()
        and path.relative_to(output).as_posix() not in excluded
    }
    if actual != set(listed):
        raise ValueError(
            "literal-wedge artifact-index membership mismatch: "
            f"missing={sorted(actual - set(listed))}, "
            f"extra={sorted(set(listed) - actual)}"
        )


def existing_panel_record(output: Path) -> dict[str, Any]:
    panel = output / "fig07_literal_wedge_timeline.png"
    manifest = output / "fig07_literal_wedge_timeline.compose.json"
    compose = json.loads(manifest.read_text(encoding="utf-8"))
    verdict = build_verdict(
        panel,
        metadata={
            "scenario": "masonry_arch_25_literal_wedge",
            "claim_scope": "current-source DART reconstruction; not paper parity",
        },
        require_contrast=False,
    )
    if not verdict["pass"]:
        raise ValueError(f"timeline panel image gate failed: {verdict['reasons']}")
    write_json(output / "fig07_literal_wedge_timeline.verdict.json", verdict)
    return {
        "path": panel.name,
        "sha256": sha256(panel),
        "selected_steps": [
            int(Path(path).stem.removeprefix("step_")) for path in compose["inputs"]
        ],
        "compose_manifest": manifest.name,
        "compose_manifest_sha256": sha256(manifest),
    }


def validate_timeline_bundle(
    output: Path, panel_record: dict[str, Any]
) -> dict[str, Any]:
    panel = output / "fig07_literal_wedge_timeline.png"
    manifest = output / "fig07_literal_wedge_timeline.compose.json"
    verdict_path = output / "fig07_literal_wedge_timeline.verdict.json"
    if (
        panel_record.get("path") != panel.name
        or panel_record.get("sha256") != sha256(panel)
        or panel_record.get("selected_steps") != list(TIMELINE_STILL_STEPS)
        or panel_record.get("compose_manifest") != manifest.name
        or panel_record.get("compose_manifest_sha256") != sha256(manifest)
    ):
        raise ValueError("timeline panel provenance binding changed")
    compose = json.loads(manifest.read_text(encoding="utf-8"))
    if (
        compose.get("schema_version") != "dart.image_compose/v1"
        or compose.get("pass") is not True
        or compose.get("mode") != "side-by-side"
        or compose.get("inputs")
        != [STILL_PATH_BY_STEP[step] for step in TIMELINE_STILL_STEPS]
        or compose.get("out") != panel.name
        or compose.get("labels")
        != [f"t={step / 60.0:.1f}s" for step in TIMELINE_STILL_STEPS]
    ):
        raise ValueError("timeline compose manifest changed")
    stored_verdict = json.loads(verdict_path.read_text(encoding="utf-8"))
    if (
        stored_verdict.get("schema_version") != "dart.image_verdict/v1"
        or stored_verdict.get("pass") is not True
        or stored_verdict.get("metadata")
        != {
            "scenario": "masonry_arch_25_literal_wedge",
            "claim_scope": "current-source DART reconstruction; not paper parity",
        }
    ):
        raise ValueError("timeline image verdict changed")
    current_verdict = build_verdict(
        panel,
        metadata={
            "scenario": "masonry_arch_25_literal_wedge",
            "claim_scope": "current-source DART reconstruction; not paper parity",
        },
        require_contrast=False,
    )
    if (
        not current_verdict["pass"]
        or current_verdict.get("checks") != stored_verdict.get("checks")
        or current_verdict.get("thresholds_used")
        != stored_verdict.get("thresholds_used")
        or current_verdict.get("image", {}).get("width")
        != stored_verdict.get("image", {}).get("width")
        or current_verdict.get("image", {}).get("height")
        != stored_verdict.get("image", {}).get("height")
    ):
        raise ValueError("timeline image verdict no longer reproduces")
    return panel_record


def prior_source_hash(provenance: dict[str, Any], relative: str) -> str | None:
    for item in provenance.get("source", {}).get("files", []):
        if item.get("path") == relative:
            return item.get("sha256")
    return None


def trace_revalidation_contract(
    trace_equivalence: dict[str, Any], *, expected_rows: int
) -> dict[str, Any]:
    max_abs_difference = trace_equivalence.get("max_abs_difference", {})
    zero_numeric_difference = bool(max_abs_difference) and all(
        float(value) == 0.0 for value in max_abs_difference.values()
    )
    complete_contract_sections = all(
        isinstance(trace_equivalence.get(key), dict) and trace_equivalence[key]
        for key in (
            "expected_reference_text",
            "expected_reference_int",
            "expected_reference_float",
        )
    )
    defining_source_contract_pass = (
        trace_equivalence.get("defining_source_contract", {}).get("pass") is True
    )
    rows_compared = trace_equivalence.get("rows_compared")
    result = {
        "reference_trace_regenerated_from_current_executable": True,
        "current_trace_executable_and_source_bound_in_provenance": True,
        "expected_rows": expected_rows,
        "rows_compared": rows_compared,
        "row_count_matches": rows_compared == expected_rows,
        "comparison_pass": trace_equivalence.get("pass") is True,
        "zero_numeric_difference": zero_numeric_difference,
        "no_mismatches": trace_equivalence.get("mismatches") == [],
        "complete_contract_sections_present": complete_contract_sections,
        "defining_source_contract_pass": defining_source_contract_pass,
    }
    result["pass"] = all(
        result[key]
        for key in (
            "row_count_matches",
            "comparison_pass",
            "zero_numeric_difference",
            "no_mismatches",
            "complete_contract_sections_present",
            "defining_source_contract_pass",
        )
    )
    return result


def build_provenance(
    args: argparse.Namespace,
    *,
    runtime_path: Path,
    reference_path: Path,
    capture_command: Sequence[str | Path],
    frame_validation: dict[str, Any],
    trace_equivalence: dict[str, Any],
    video_probe: dict[str, Any],
    panel: dict[str, Any],
    midpoint: dict[str, Any],
    durable_stills: list[dict[str, Any]],
    staging: dict[str, Any],
    generation_mode: str,
    prior_provenance: dict[str, Any] | None,
) -> dict[str, Any]:
    runtime_config = json.loads(runtime_path.read_text(encoding="utf-8"))
    git_status = git_output("status", "--porcelain=v1", "--untracked-files=all")
    capture_hash = sha256(args.capture)
    trace_hash = sha256(args.trace)
    continuity: dict[str, Any] = {
        "current_capture_executable_sha256": capture_hash,
        "current_trace_executable_sha256": trace_hash,
        "capture_executable_matches_original_raw": True,
        "trace_executable_matches_original_raw": True,
        "capture_source_matches_original_raw": True,
        "trace_source_matches_original_raw": True,
    }
    if prior_provenance is not None:
        prior_source = prior_provenance.get("source", {})
        prior_continuity = prior_source.get("binary_and_raw_source_continuity", {})
        prior_capture_hash = prior_continuity.get(
            "original_capture_executable_sha256",
            prior_source.get("capture_executable_sha256"),
        )
        prior_trace_hash = prior_continuity.get(
            "original_trace_executable_sha256",
            prior_source.get("trace_executable_sha256"),
        )
        prior_capture_source = prior_continuity.get(
            "original_capture_source_sha256",
            prior_source_hash(
                prior_provenance,
                "tests/benchmark/integration/fbf_literal_wedge_visual_capture.cpp",
            ),
        )
        prior_trace_source = prior_continuity.get(
            "original_trace_source_sha256",
            prior_source_hash(
                prior_provenance,
                "tests/benchmark/integration/fbf_paper_trace.cpp",
            ),
        )
        trace_revalidation = trace_revalidation_contract(
            trace_equivalence,
            expected_rows=int(runtime_config["steps_completed"]),
        )
        continuity.update(
            {
                "original_capture_executable_sha256": prior_capture_hash,
                "original_trace_executable_sha256": prior_trace_hash,
                "original_capture_source_sha256": prior_capture_source,
                "original_trace_source_sha256": prior_trace_source,
                "capture_executable_matches_original_raw": (
                    prior_capture_hash == capture_hash
                ),
                "trace_executable_matches_original_raw": (
                    prior_trace_hash == trace_hash
                ),
                "capture_source_matches_original_raw": (
                    prior_capture_source
                    == sha256(
                        ROOT / "tests/benchmark/integration/"
                        "fbf_literal_wedge_visual_capture.cpp"
                    )
                ),
                "trace_source_matches_original_raw": (
                    prior_trace_source
                    == sha256(ROOT / "tests/benchmark/integration/fbf_paper_trace.cpp")
                ),
                "current_trace_revalidation": trace_revalidation,
            }
        )
        if not (
            continuity["capture_executable_matches_original_raw"]
            and continuity["capture_source_matches_original_raw"]
        ):
            raise ValueError(
                "existing raw capture binary/source continuity failed: " f"{continuity}"
            )
        if generation_mode != "revalidated_existing_raw_assets" or not (
            trace_revalidation["pass"]
        ):
            raise ValueError(
                "current trace revalidation did not prove a fresh complete-contract "
                f"zero-difference comparison: {trace_revalidation}"
            )

    return {
        "schema_version": "dart.fbf_literal_wedge_visual_evidence/v2",
        "generation_mode": generation_mode,
        "evidence_state": PENDING_STATUS,
        "claim_scope": (
            "Current-source DART evidence for the reconstructed literal 25-wedge "
            "Native/FourPointPlanar exact-FBF scene; not paper parity."
        ),
        "limitations": [
            "The paper's author scene, camera, source code, and golden frames are unavailable.",
            "The clip contains the stable no-projectile 600-step scenario, not the paper video's crown impact.",
            "The 10-second simulation is sampled into 61 frames and played in 6.1 seconds, approximately 1.639x faster than real time.",
            "The capture app independently mirrors fbf_paper_trace; trace-equivalence.json binds their per-step outcomes.",
            "VisualAspect state and a finite floor proxy with no CollisionAspect or DynamicsAspect are the only rendering additions.",
        ],
        "commands": {
            "capture": [str(item) for item in capture_command],
            "reference_trace": trace_command(
                args.trace,
                int(runtime_config["steps_completed"]),
                int(runtime_config["simulation_threads"]),
            ),
            "automation": [str(item) for item in sys.argv],
            "midpoint_decode": midpoint["command"],
        },
        "source": {
            "git_head": git_output("rev-parse", "HEAD"),
            "worktree_dirty": bool(git_status),
            "git_status_porcelain_v1": git_status,
            "git_status_sha256": hashlib.sha256(git_status.encode()).hexdigest(),
            "capture_executable": str(args.capture),
            "capture_executable_sha256": capture_hash,
            "trace_executable": str(args.trace),
            "trace_executable_sha256": trace_hash,
            "binary_and_raw_source_continuity": continuity,
            "files": [
                {"path": relative, "sha256": sha256(ROOT / relative)}
                for relative in SOURCE_FILES
            ],
        },
        "environment": collect_environment_provenance(
            capture=args.capture, ffmpeg=args.ffmpeg, ffprobe=args.ffprobe
        ),
        "runtime": {
            "path": runtime_path.name,
            "sha256": sha256(runtime_path),
            "trajectory_path": "trajectory.csv",
            "trajectory_sha256": sha256(args.output / "trajectory.csv"),
            "reference_trace_path": reference_path.name,
            "reference_trace_sha256": sha256(reference_path),
        },
        "validation": {
            "frame_validation_path": "frame-validation.json",
            "frame_validation_sha256": sha256(args.output / "frame-validation.json"),
            "trace_equivalence_path": "trace-equivalence.json",
            "trace_equivalence_sha256": sha256(args.output / "trace-equivalence.json"),
            "video_probe_path": "video-probe.json",
            "video_probe_sha256": sha256(args.output / "video-probe.json"),
            "automated_validation_pass": (
                frame_validation["pass"]
                and trace_equivalence["pass"]
                and video_probe["pass"]
            ),
            "manual_inspection_bound": False,
            "trace_timeline_binding": {
                "capture_runtime_sha256": sha256(runtime_path),
                "frame_validation_sha256": sha256(
                    args.output / "frame-validation.json"
                ),
                "trajectory_sha256": sha256(args.output / "trajectory.csv"),
                "reference_trace_sha256": sha256(reference_path),
                "trace_equivalence_sha256": sha256(
                    args.output / "trace-equivalence.json"
                ),
                "timeline_compose_manifest_sha256": panel["compose_manifest_sha256"],
                "durable_stills": durable_stills,
            },
        },
        "media": {
            "video": {
                "path": video_probe["path"],
                "sha256": video_probe["sha256"],
                "timing": video_probe["timing"],
            },
            "decoded_midpoint": midpoint,
            "timeline_panel": panel,
            "durable_stills": durable_stills,
        },
        "capture_staging": staging,
    }


def write_pending_bundle(
    args: argparse.Namespace,
    *,
    runtime_path: Path,
    provenance: dict[str, Any],
    trace_equivalence: dict[str, Any],
    frame_validation: dict[str, Any],
    video_probe: dict[str, Any],
    panel: dict[str, Any],
    midpoint: dict[str, Any],
) -> dict[str, Any]:
    provenance_path = args.output / "provenance.json"
    write_json(provenance_path, provenance)
    manual_path = args.output / "manual-inspection.json"
    if manual_path.is_file() or manual_path.is_symlink():
        manual_path.unlink()
    index_path = args.output / "artifact-index.json"
    write_json(
        index_path,
        artifact_index(args.output, INDEX_EXCLUSIONS_PENDING),
    )
    metadata = {
        "schema_version": "dart.fbf_literal_wedge_visual_bundle/v2",
        "status": PENDING_STATUS,
        "claim_valid": False,
        "paper_parity": False,
        "manual_inspection_required": True,
        "manual_inspected": False,
        "manual_inspection_bound": False,
        "runtime_sha256": sha256(runtime_path),
        "provenance_sha256": sha256(provenance_path),
        "artifact_index_sha256": sha256(index_path),
        "trace_equivalence_sha256": sha256(args.output / "trace-equivalence.json"),
        "frame_validation_sha256": sha256(args.output / "frame-validation.json"),
        "video_probe_sha256": sha256(args.output / "video-probe.json"),
        "video_sha256": video_probe["sha256"],
        "timeline_panel_sha256": panel["sha256"],
        "decoded_midpoint_path": midpoint["path"],
        "decoded_midpoint_sha256": midpoint["sha256"],
        "durable_stills": provenance["media"]["durable_stills"],
        "staging_pruned": True,
        "raw_frame_staging_required": False,
        "timing": video_probe["timing"],
    }
    write_json(args.output / "metadata.json", metadata)
    write_json(args.output / "pending-metadata.json", metadata)
    _validate_bundle_paths(
        args.output,
        complete=True,
        final=False,
        allow_manual=False,
    )
    validate_artifact_index(
        args.output,
        json.loads(index_path.read_text(encoding="utf-8")),
        excluded=INDEX_EXCLUSIONS_PENDING,
    )
    return metadata


def verify_pending_hash_dag(
    output: Path, pending_metadata: dict[str, Any]
) -> dict[str, Any]:
    errors: list[str] = []

    try:
        _validate_bundle_paths(
            output,
            complete=True,
            final=False,
            allow_manual=True,
        )
    except ValueError as error:
        errors.append(str(error))

    def check_hash(label: str, expected: str | None, path: Path) -> None:
        if not path.is_file():
            errors.append(f"{label}: missing {path}")
        elif expected != sha256(path):
            errors.append(f"{label}: hash mismatch for {path}")

    pending_path = output / "pending-metadata.json"
    metadata_path = output / "metadata.json"
    if not pending_path.is_file():
        errors.append("immutable pending-metadata.json is missing")
    elif sha256(pending_path) != sha256(metadata_path):
        errors.append("metadata.json differs from immutable pending-metadata.json")

    metadata_hash_fields = {
        "artifact_index_sha256": "artifact-index.json",
        "provenance_sha256": "provenance.json",
        "runtime_sha256": "capture-runtime.json",
        "trace_equivalence_sha256": "trace-equivalence.json",
        "frame_validation_sha256": "frame-validation.json",
        "video_probe_sha256": "video-probe.json",
        "video_sha256": "fig07_literal_wedge_stability.mp4",
        "timeline_panel_sha256": "fig07_literal_wedge_timeline.png",
        "decoded_midpoint_sha256": "decoded/video_midpoint_t3.0.png",
    }
    for field, relative in metadata_hash_fields.items():
        check_hash(field, pending_metadata.get(field), output / relative)

    index_path = output / "artifact-index.json"
    index = json.loads(index_path.read_text(encoding="utf-8"))
    listed = {item["path"] for item in index["artifacts"]}
    excluded = set(index.get("excluded", []))
    required_excluded = INDEX_EXCLUSIONS_PENDING
    if excluded != required_excluded:
        errors.append(
            f"pending artifact-index exclusions {sorted(excluded)} != "
            f"{sorted(required_excluded)}"
        )
    for item in index["artifacts"]:
        path = output / item["path"]
        if not path.is_file():
            errors.append(f"artifact-index missing {item['path']}")
            continue
        if path.stat().st_size != item["bytes"]:
            errors.append(f"artifact-index byte mismatch {item['path']}")
        if sha256(path) != item["sha256"]:
            errors.append(f"artifact-index hash mismatch {item['path']}")
    actual = {
        str(path.relative_to(output)) for path in output.rglob("*") if path.is_file()
    } - excluded
    if actual != listed:
        errors.append(
            f"artifact-index membership mismatch missing={sorted(actual - listed)} "
            f"extra={sorted(listed - actual)}"
        )
    try:
        validate_artifact_index(output, index, excluded=INDEX_EXCLUSIONS_PENDING)
    except ValueError as error:
        errors.append(str(error))

    provenance_path = output / "provenance.json"
    provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
    if provenance.get("evidence_state") != PENDING_STATUS:
        errors.append("provenance is not in pending evidence state")
    if not provenance.get("validation", {}).get("automated_validation_pass"):
        errors.append("provenance automated validation is not passing")
    if provenance.get("validation", {}).get("manual_inspection_bound") is not False:
        errors.append("pending provenance already binds manual inspection")
    capture_staging = provenance.get("capture_staging", {})

    source = provenance.get("source", {})
    continuity = source.get("binary_and_raw_source_continuity", {})
    if provenance.get("generation_mode") == "revalidated_existing_raw_assets" and not (
        continuity.get("capture_executable_matches_original_raw") is True
        and continuity.get("capture_source_matches_original_raw") is True
    ):
        errors.append("revalidated provenance does not retain original raw capture")
    check_hash(
        "capture executable",
        source.get("capture_executable_sha256"),
        Path(source.get("capture_executable", "")),
    )
    check_hash(
        "trace executable",
        source.get("trace_executable_sha256"),
        Path(source.get("trace_executable", "")),
    )
    for item in source.get("files", []):
        check_hash(
            f"source {item.get('path')}", item.get("sha256"), ROOT / item["path"]
        )

    environment = provenance.get("environment", {})
    build = environment.get("build", {})
    check_hash(
        "cmake cache",
        build.get("cmake_cache_sha256"),
        Path(build.get("cmake_cache_path", "")),
    )
    if build.get("cmake_cache", {}).get("DART_DISABLE_COMPILER_CACHE") != "ON":
        errors.append("pending provenance does not bind no-cache Release config")

    def check_recorded_executables(value: Any, label: str) -> None:
        if isinstance(value, dict):
            if value.get("available") is True and "executable_sha256" in value:
                check_hash(
                    f"environment tool {label}",
                    value.get("executable_sha256"),
                    Path(value.get("executable", "")),
                )
            for key, child in value.items():
                check_recorded_executables(child, f"{label}.{key}")
        elif isinstance(value, list):
            for index_value, child in enumerate(value):
                check_recorded_executables(child, f"{label}[{index_value}]")

    check_recorded_executables(environment, "environment")

    runtime_info = provenance.get("runtime", {})
    check_hash(
        "provenance runtime",
        runtime_info.get("sha256"),
        output / runtime_info.get("path", ""),
    )
    check_hash(
        "provenance trajectory",
        runtime_info.get("trajectory_sha256"),
        output / runtime_info.get("trajectory_path", ""),
    )
    check_hash(
        "provenance reference trace",
        runtime_info.get("reference_trace_sha256"),
        output / runtime_info.get("reference_trace_path", ""),
    )
    validation = provenance.get("validation", {})
    for prefix in ("frame_validation", "trace_equivalence", "video_probe"):
        check_hash(
            f"provenance {prefix}",
            validation.get(f"{prefix}_sha256"),
            output / validation.get(f"{prefix}_path", ""),
        )
    media = provenance.get("media", {})
    check_hash(
        "provenance video",
        media.get("video", {}).get("sha256"),
        output / media.get("video", {}).get("path", ""),
    )
    check_hash(
        "provenance midpoint",
        media.get("decoded_midpoint", {}).get("sha256"),
        output / media.get("decoded_midpoint", {}).get("path", ""),
    )
    check_hash(
        "provenance timeline",
        media.get("timeline_panel", {}).get("sha256"),
        output / media.get("timeline_panel", {}).get("path", ""),
    )

    runtime = json.loads((output / "capture-runtime.json").read_text(encoding="utf-8"))
    validate_runtime(
        runtime,
        steps=int(runtime["steps_completed"]),
        stride=int(runtime["frame_stride"]),
        width=int(runtime["width"]),
        height=int(runtime["height"]),
        threads=int(runtime["simulation_threads"]),
    )
    stored_frames = json.loads((output / "frame-validation.json").read_text())
    try:
        validate_frame_validation_contract(runtime, stored_frames)
        _validate_staging_provenance(capture_staging, stored_frames)
        durable_stills = validate_durable_stills(
            output,
            runtime,
            stored_frames,
            media.get("durable_stills"),
        )
        if pending_metadata.get("durable_stills") != durable_stills:
            errors.append("pending metadata durable still bindings changed")
        binding = provenance.get("validation", {}).get("trace_timeline_binding", {})
        expected_binding = {
            "capture_runtime_sha256": sha256(output / "capture-runtime.json"),
            "frame_validation_sha256": sha256(output / "frame-validation.json"),
            "trajectory_sha256": sha256(output / "trajectory.csv"),
            "reference_trace_sha256": sha256(output / "reference-fbf-paper-trace.csv"),
            "trace_equivalence_sha256": sha256(output / "trace-equivalence.json"),
            "timeline_compose_manifest_sha256": sha256(
                output / "fig07_literal_wedge_timeline.compose.json"
            ),
            "durable_stills": durable_stills,
        }
        if binding != expected_binding:
            errors.append("trace/timeline/durable-still binding changed")
        validate_timeline_bundle(output, media.get("timeline_panel", {}))
    except (OSError, ValueError) as error:
        errors.append(str(error))
    stored_trace = json.loads((output / "trace-equivalence.json").read_text())
    if not stored_trace.get("pass"):
        errors.append("stored trace equivalence is not passing")
    recomputed_trace = compare_trace(
        read_csv(output / "trajectory.csv"),
        read_csv(output / "reference-fbf-paper-trace.csv"),
    )
    if recomputed_trace != stored_trace:
        errors.append("trace equivalence changed after pending state")
    if provenance.get("generation_mode") == "revalidated_existing_raw_assets":
        recomputed_trace_revalidation = trace_revalidation_contract(
            recomputed_trace,
            expected_rows=int(runtime["steps_completed"]),
        )
        if (
            continuity.get("current_trace_revalidation")
            != recomputed_trace_revalidation
        ):
            errors.append(
                "current trace revalidation contract changed after pending state"
            )
        if not recomputed_trace_revalidation["pass"]:
            errors.append("current trace revalidation contract is not passing")
    stored_video_probe = json.loads((output / "video-probe.json").read_text())
    if not stored_video_probe.get("pass"):
        errors.append("stored video probe is not passing")

    if errors:
        raise ValueError("pending hash DAG verification failed: " + "; ".join(errors))
    return {
        "pass": True,
        "artifact_count": len(index["artifacts"]),
        "source_count": len(source.get("files", [])),
        "frame_count": stored_frames["frame_count"],
        "trace_rows": stored_trace["rows_compared"],
        "pending_metadata_sha256": sha256(pending_path),
    }


def verify_manual_inspection(
    output: Path, record_path: Path, pending_metadata: dict[str, Any]
) -> dict[str, Any]:
    record = json.loads(record_path.read_text(encoding="utf-8"))
    require_keys(
        record,
        (
            "schema_version",
            "manual_inspected",
            "pass",
            "claim_scope",
            "inspection_session",
            "inspection_tool",
            "pending_metadata_sha256",
            "representative_images",
            "video",
            "timing_disclosure",
        ),
        "manual-inspection record",
    )
    if record["schema_version"] != MANUAL_SCHEMA_VERSION:
        raise ValueError(
            f"unexpected manual-inspection schema: {record['schema_version']}"
        )
    if record["manual_inspected"] is not True or record["pass"] is not True:
        raise ValueError("manual inspection did not explicitly pass")
    if "not paper parity" not in record["claim_scope"].lower():
        raise ValueError("manual inspection must preserve the non-paper claim scope")
    if record["pending_metadata_sha256"] != sha256(output / "pending-metadata.json"):
        raise ValueError("manual inspection is not bound to the pending metadata")
    session = record["inspection_session"]
    tool = record["inspection_tool"]
    for field in ("label", "inspector_label"):
        if not isinstance(session.get(field), str) or not session[field].strip():
            raise ValueError(f"manual inspection session lacks {field}")
    if not isinstance(tool.get("name"), str) or not tool["name"].strip():
        raise ValueError("manual inspection tool name is missing")
    if not isinstance(tool.get("version"), str) or not tool["version"].strip():
        raise ValueError("manual inspection tool version is missing")
    if not isinstance(tool.get("commands"), list) or not tool["commands"]:
        raise ValueError("manual inspection commands are missing")

    required_paths = {
        *(STILL_PATH_BY_STEP[step] for step in MANUAL_STILL_STEPS),
        "fig07_literal_wedge_timeline.png",
    }
    inspected_paths: set[str] = set()
    for item in record["representative_images"]:
        relative = item.get("path")
        path = output / relative
        if not path.is_file() or item.get("sha256") != sha256(path):
            raise ValueError(f"manual inspection image binding failed: {relative}")
        if not isinstance(item.get("observation"), str) or not item["observation"]:
            raise ValueError(f"manual inspection image lacks observation: {relative}")
        inspected_paths.add(relative)
    if inspected_paths != required_paths:
        raise ValueError(
            f"manual inspection paths {sorted(inspected_paths)} != {sorted(required_paths)}"
        )

    video = record["video"]
    video_path = output / video["path"]
    if video.get("sha256") != sha256(video_path):
        raise ValueError("manual video hash binding failed")
    midpoint = video.get("decoded_midpoint", {})
    midpoint_path = output / midpoint.get("path", "")
    if (
        midpoint.get("seconds") != 3.0
        or not midpoint_path.is_file()
        or midpoint.get("sha256") != sha256(midpoint_path)
        or not isinstance(midpoint.get("decode_command"), list)
        or not midpoint["decode_command"]
        or not isinstance(midpoint.get("inspection_command"), str)
        or not midpoint["inspection_command"]
    ):
        raise ValueError("manual decoded-midpoint binding is incomplete")

    timing = record["timing_disclosure"]
    expected_timing = pending_metadata["timing"]
    for key in (
        "simulation_duration_seconds",
        "captured_frames",
        "playback_duration_seconds",
        "playback_acceleration_factor",
    ):
        if not math.isclose(
            float(timing[key]), float(expected_timing[key]), rel_tol=0.0, abs_tol=1e-12
        ):
            raise ValueError(f"manual timing disclosure mismatch for {key}")
    return record


def _finalize_existing_in_place(args: argparse.Namespace) -> dict[str, Any]:
    if args.manual_inspection_record is None:
        raise ValueError("--finalize-existing requires --manual-inspection-record")
    metadata_path = args.output / "metadata.json"
    pending_metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if (
        pending_metadata.get("status") != PENDING_STATUS
        or pending_metadata.get("claim_valid") is not False
        or pending_metadata.get("manual_inspection_bound") is not False
    ):
        raise ValueError("bundle is not in the required pending-manual state")
    record_path = args.manual_inspection_record.resolve()
    if record_path.parent != args.output.resolve():
        raise ValueError("manual-inspection record must live in the bundle root")
    dag_verification = verify_pending_hash_dag(args.output, pending_metadata)
    record = verify_manual_inspection(args.output, record_path, pending_metadata)
    manual_hash = sha256(record_path)

    provenance_path = args.output / "provenance.json"
    provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
    if not provenance.get("validation", {}).get("automated_validation_pass"):
        raise ValueError("automated provenance validation is not passing")
    provenance["evidence_state"] = FINAL_STATUS
    provenance["validation"].update(
        {
            "manual_inspection_bound": True,
            "manual_inspection_path": record_path.name,
            "manual_inspection_sha256": manual_hash,
        }
    )
    provenance["finalization"] = {
        "mode": "explicit_manual_inspection_finalization",
        "command": [str(item) for item in sys.argv],
        "pending_metadata_sha256": record["pending_metadata_sha256"],
        "pending_hash_dag_verification": dag_verification,
        "manual_inspection_sha256": manual_hash,
        "claim_valid_after_finalization": True,
        "paper_parity": False,
    }
    write_json(provenance_path, provenance)

    index_path = args.output / "artifact-index.json"
    write_json(
        index_path,
        artifact_index(args.output, INDEX_EXCLUSIONS_FINAL),
    )
    finalized = dict(pending_metadata)
    finalized.update(
        {
            "status": FINAL_STATUS,
            "claim_valid": True,
            "manual_inspected": True,
            "manual_inspection_bound": True,
            "manual_inspection_path": record_path.name,
            "manual_inspection_sha256": manual_hash,
            "pending_metadata_sha256": record["pending_metadata_sha256"],
            "pending_metadata_path": "pending-metadata.json",
            "provenance_sha256": sha256(provenance_path),
            "artifact_index_sha256": sha256(index_path),
        }
    )
    write_json(metadata_path, finalized)
    _validate_bundle_paths(args.output, complete=True, final=True)
    validate_artifact_index(
        args.output,
        json.loads(index_path.read_text(encoding="utf-8")),
        excluded=INDEX_EXCLUSIONS_FINAL,
    )
    return finalized


def finalize_existing(args: argparse.Namespace) -> dict[str, Any]:
    args.output = _require_bundle_root(args.output, create=False)
    with _bundle_transaction(args.output):
        return _finalize_existing_in_place(args)


def _validate_live_provenance_identity(provenance: dict[str, Any]) -> None:
    errors: list[str] = []

    def check_hash(label: str, expected: Any, path: Path) -> None:
        if not path.is_file() or path.is_symlink():
            errors.append(f"{label}: missing {path}")
        elif expected != sha256(path):
            errors.append(f"{label}: hash mismatch for {path}")

    source = provenance.get("source", {})
    check_hash(
        "capture executable",
        source.get("capture_executable_sha256"),
        Path(source.get("capture_executable", "")),
    )
    check_hash(
        "trace executable",
        source.get("trace_executable_sha256"),
        Path(source.get("trace_executable", "")),
    )
    for item in source.get("files", []):
        relative = item.get("path")
        if not isinstance(relative, str):
            errors.append("source provenance path is missing")
            continue
        check_hash(f"source {relative}", item.get("sha256"), ROOT / relative)

    environment = provenance.get("environment", {})
    build = environment.get("build", {})
    check_hash(
        "cmake cache",
        build.get("cmake_cache_sha256"),
        Path(build.get("cmake_cache_path", "")),
    )
    if build.get("cmake_cache", {}).get("DART_DISABLE_COMPILER_CACHE") != "ON":
        errors.append("provenance does not bind no-cache Release config")

    def check_recorded_executables(value: Any, label: str) -> None:
        if isinstance(value, dict):
            if value.get("available") is True and "executable_sha256" in value:
                check_hash(
                    f"environment tool {label}",
                    value.get("executable_sha256"),
                    Path(value.get("executable", "")),
                )
            for key, child in value.items():
                check_recorded_executables(child, f"{label}.{key}")
        elif isinstance(value, list):
            for index, child in enumerate(value):
                check_recorded_executables(child, f"{label}[{index}]")

    check_recorded_executables(environment, "environment")
    if errors:
        raise ValueError("live provenance identity failed: " + "; ".join(errors))


def verify_finalized_bundle(output: Path, *, live_identity: bool) -> dict[str, Any]:
    output = _require_bundle_root(output, create=False)
    _validate_bundle_paths(output, complete=True, final=True)
    metadata = json.loads((output / "metadata.json").read_text(encoding="utf-8"))
    if (
        metadata.get("schema_version") != "dart.fbf_literal_wedge_visual_bundle/v2"
        or metadata.get("status") != FINAL_STATUS
        or metadata.get("claim_valid") is not True
        or metadata.get("paper_parity") is not False
        or metadata.get("manual_inspected") is not True
        or metadata.get("manual_inspection_bound") is not True
        or metadata.get("staging_pruned") is not True
        or metadata.get("raw_frame_staging_required") is not False
    ):
        raise ValueError("final literal-wedge metadata claim boundary changed")

    pending_path = output / "pending-metadata.json"
    pending = json.loads(pending_path.read_text(encoding="utf-8"))
    if (
        pending.get("status") != PENDING_STATUS
        or pending.get("claim_valid") is not False
        or pending.get("manual_inspection_bound") is not False
        or metadata.get("pending_metadata_sha256") != sha256(pending_path)
    ):
        raise ValueError("immutable pending literal-wedge state changed")

    hash_fields = {
        "artifact_index_sha256": "artifact-index.json",
        "provenance_sha256": "provenance.json",
        "runtime_sha256": "capture-runtime.json",
        "trace_equivalence_sha256": "trace-equivalence.json",
        "frame_validation_sha256": "frame-validation.json",
        "video_probe_sha256": "video-probe.json",
        "video_sha256": "fig07_literal_wedge_stability.mp4",
        "timeline_panel_sha256": "fig07_literal_wedge_timeline.png",
        "decoded_midpoint_sha256": "decoded/video_midpoint_t3.0.png",
        "manual_inspection_sha256": "manual-inspection.json",
    }
    for field, relative in hash_fields.items():
        path = output / relative
        if metadata.get(field) != sha256(path):
            raise ValueError(f"final metadata hash binding changed: {field}")

    index = json.loads((output / "artifact-index.json").read_text(encoding="utf-8"))
    validate_artifact_index(output, index, excluded=INDEX_EXCLUSIONS_FINAL)
    provenance = json.loads((output / "provenance.json").read_text(encoding="utf-8"))
    if (
        provenance.get("evidence_state") != FINAL_STATUS
        or "not paper parity" not in provenance.get("claim_scope", "").lower()
        or provenance.get("validation", {}).get("automated_validation_pass") is not True
        or provenance.get("validation", {}).get("manual_inspection_bound") is not True
        or provenance.get("validation", {}).get("manual_inspection_sha256")
        != sha256(output / "manual-inspection.json")
        or provenance.get("finalization", {}).get("paper_parity") is not False
        or provenance.get("finalization", {}).get("claim_valid_after_finalization")
        is not True
    ):
        raise ValueError("final provenance claim/finalization boundary changed")
    if live_identity:
        _validate_live_provenance_identity(provenance)

    runtime = json.loads((output / "capture-runtime.json").read_text(encoding="utf-8"))
    validate_runtime(
        runtime,
        steps=int(runtime["steps_completed"]),
        stride=int(runtime["frame_stride"]),
        width=int(runtime["width"]),
        height=int(runtime["height"]),
        threads=int(runtime["simulation_threads"]),
    )
    frames = json.loads((output / "frame-validation.json").read_text(encoding="utf-8"))
    validate_frame_validation_contract(runtime, frames)
    _validate_staging_provenance(provenance.get("capture_staging", {}), frames)
    durable_stills = validate_durable_stills(
        output,
        runtime,
        frames,
        provenance.get("media", {}).get("durable_stills"),
    )
    if metadata.get("durable_stills") != durable_stills:
        raise ValueError("final metadata durable still bindings changed")
    binding = provenance.get("validation", {}).get("trace_timeline_binding")
    expected_binding = {
        "capture_runtime_sha256": sha256(output / "capture-runtime.json"),
        "frame_validation_sha256": sha256(output / "frame-validation.json"),
        "trajectory_sha256": sha256(output / "trajectory.csv"),
        "reference_trace_sha256": sha256(output / "reference-fbf-paper-trace.csv"),
        "trace_equivalence_sha256": sha256(output / "trace-equivalence.json"),
        "timeline_compose_manifest_sha256": sha256(
            output / "fig07_literal_wedge_timeline.compose.json"
        ),
        "durable_stills": durable_stills,
    }
    if binding != expected_binding:
        raise ValueError("final trace/timeline/durable-still binding changed")
    validate_timeline_bundle(
        output, provenance.get("media", {}).get("timeline_panel", {})
    )

    stored_trace = json.loads((output / "trace-equivalence.json").read_text())
    recomputed_trace = compare_trace(
        read_csv(output / "trajectory.csv"),
        read_csv(output / "reference-fbf-paper-trace.csv"),
    )
    if stored_trace.get("pass") is not True or recomputed_trace != stored_trace:
        raise ValueError("sealed trace equivalence changed")
    stored_probe = json.loads((output / "video-probe.json").read_text())
    if stored_probe.get("pass") is not True or stored_probe.get("sha256") != sha256(
        output / "fig07_literal_wedge_stability.mp4"
    ):
        raise ValueError("sealed video probe changed")
    manual = verify_manual_inspection(
        output,
        output / "manual-inspection.json",
        pending,
    )
    return {
        "schema_version": metadata["schema_version"],
        "status": metadata["status"],
        "claim_valid": True,
        "paper_parity": False,
        "artifact_count": index["artifact_count"],
        "frame_count": frames["frame_count"],
        "durable_still_count": len(durable_stills),
        "trace_rows": stored_trace["rows_compared"],
        "manual_inspection_sha256": sha256(output / "manual-inspection.json"),
        "metadata_sha256": sha256(output / "metadata.json"),
    }


def verify_only(args: argparse.Namespace) -> dict[str, Any]:
    args.output = _require_bundle_root(args.output, create=False)
    result = verify_finalized_bundle(args.output, live_identity=True)
    runtime = json.loads((args.output / "capture-runtime.json").read_text())
    expected_frames = len(runtime["frames"])
    simulation_seconds = int(runtime["steps_completed"]) * float(runtime["dt"])
    live_probe = probe_video(
        args.ffprobe,
        args.output / "fig07_literal_wedge_stability.mp4",
        expected_frames=expected_frames,
        expected_width=int(runtime["width"]),
        expected_height=int(runtime["height"]),
        expected_fps=args.fps,
        simulation_seconds=simulation_seconds,
    )
    stored_probe = json.loads((args.output / "video-probe.json").read_text())
    if live_probe != stored_probe:
        raise ValueError("live ffprobe result changed")
    traced = run(
        trace_command(
            args.trace,
            int(runtime["steps_completed"]),
            int(runtime["simulation_threads"]),
        )
    )
    live_reference_rows = list(csv.DictReader(traced.stdout.splitlines()))
    live_comparison = compare_trace(
        read_csv(args.output / "trajectory.csv"),
        live_reference_rows,
    )
    stored_comparison = json.loads(
        (args.output / "trace-equivalence.json").read_text(encoding="utf-8")
    )
    if traced.returncode != 0 or live_comparison != stored_comparison:
        raise ValueError("live literal-wedge trace replay changed")
    return {
        **result,
        "live_trace_replay": True,
        "live_video_probe": True,
        "ignored_staging_required": False,
    }


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--revalidate-existing",
        action="store_true",
        help="revalidate existing raw media without rerunning simulation",
    )
    mode.add_argument(
        "--finalize-existing",
        action="store_true",
        help="bind a verified manual-inspection record to a pending bundle",
    )
    mode.add_argument(
        "--verify-only",
        action="store_true",
        help="verify the sealed bundle without ignored raw-frame staging",
    )
    parser.add_argument(
        "--manual-inspection-record",
        type=Path,
        help="manual record supplied only to --finalize-existing",
    )
    parser.add_argument("--capture", type=Path, default=DEFAULT_CAPTURE)
    parser.add_argument("--trace", type=Path, default=DEFAULT_TRACE)
    parser.add_argument("--ffmpeg", type=Path, default=DEFAULT_FFMPEG)
    parser.add_argument("--ffprobe", type=Path, default=DEFAULT_FFPROBE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--steps", type=int, default=600)
    parser.add_argument("--frame-stride", type=int, default=10)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--fps", type=int, default=10)
    args = parser.parse_args(argv)
    for name in ("steps", "frame_stride", "width", "height", "threads", "fps"):
        if getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    return args


def _generate_pending_in_place(args: argparse.Namespace) -> dict[str, Any]:
    prior_provenance = None
    if args.revalidate_existing:
        prior_path = args.output / "provenance.json"
        if prior_path.is_file():
            prior_provenance = json.loads(prior_path.read_text(encoding="utf-8"))
        if prior_provenance is None:
            raise ValueError("existing bundle lacks capture provenance")
        if (
            prior_provenance.get("capture_staging", {}).get("staging_pruned") is True
            and not (args.output / "frames").is_dir()
        ):
            raise ValueError(
                "sealed literal-wedge evidence cannot be revalidated without a "
                "fresh capture; use --verify-only"
            )
        capture_command = prior_provenance.get("commands", {}).get("capture", [])
        if not capture_command:
            raise ValueError("existing bundle lacks its original capture command")
    else:
        capture_command = [
            args.capture,
            args.output,
            str(args.steps),
            str(args.frame_stride),
            str(args.width),
            str(args.height),
            str(args.threads),
        ]
        captured = run(capture_command)
        (args.output / "capture.stdout.txt").write_text(
            captured.stdout, encoding="utf-8"
        )
        (args.output / "capture.stderr.txt").write_text(
            captured.stderr, encoding="utf-8"
        )
        if captured.returncode != 0:
            raise RuntimeError(
                f"capture failed ({captured.returncode}): {captured.stderr.strip()}"
            )

    runtime_path = args.output / "capture-runtime.json"
    runtime = json.loads(runtime_path.read_text(encoding="utf-8"))
    steps = int(runtime["steps_completed"])
    stride = int(runtime["frame_stride"])
    width = int(runtime["width"])
    height = int(runtime["height"])
    threads = int(runtime["simulation_threads"])
    validate_runtime(
        runtime,
        steps=steps,
        stride=stride,
        width=width,
        height=height,
        threads=threads,
    )
    frame_validation = validate_frames(args.output, runtime, width, height)
    write_json(args.output / "frame-validation.json", frame_validation)
    durable_stills = promote_durable_stills(
        args.output,
        runtime,
        frame_validation,
    )

    reference_path = args.output / "reference-fbf-paper-trace.csv"
    traced = run(trace_command(args.trace, steps, threads))
    reference_path.write_text(traced.stdout, encoding="utf-8")
    (args.output / "reference-fbf-paper-trace.stderr.txt").write_text(
        traced.stderr, encoding="utf-8"
    )
    if traced.returncode != 0:
        raise RuntimeError(
            f"reference trace failed ({traced.returncode}): {traced.stderr.strip()}"
        )
    trace_equivalence = compare_trace(
        read_csv(args.output / "trajectory.csv"), read_csv(reference_path)
    )
    write_json(args.output / "trace-equivalence.json", trace_equivalence)
    if not trace_equivalence["pass"]:
        raise ValueError(
            "capture/reference trace mismatch: "
            + "; ".join(trace_equivalence["mismatches"][:5])
        )

    expected_frames = frame_validation["frame_count"]
    simulation_seconds = steps * float(runtime["dt"])
    if args.revalidate_existing:
        video_probe = probe_video(
            args.ffprobe,
            args.output / "fig07_literal_wedge_stability.mp4",
            expected_frames=expected_frames,
            expected_width=width,
            expected_height=height,
            expected_fps=args.fps,
            simulation_seconds=simulation_seconds,
        )
    else:
        video_probe = encode_video(
            args.ffmpeg,
            args.ffprobe,
            args.output,
            fps=args.fps,
            expected_frames=expected_frames,
            expected_width=width,
            expected_height=height,
            expected_fps=args.fps,
            simulation_seconds=simulation_seconds,
        )
    write_json(args.output / "video-probe.json", video_probe)

    panel = compose_timeline(args.output, steps)
    midpoint = decode_midpoint(
        args.ffmpeg,
        args.output,
        seconds=3.0,
        width=width,
        height=height,
    )
    staging = _record_staging(args.output)
    provenance = build_provenance(
        args,
        runtime_path=runtime_path,
        reference_path=reference_path,
        capture_command=capture_command,
        frame_validation=frame_validation,
        trace_equivalence=trace_equivalence,
        video_probe=video_probe,
        panel=panel,
        midpoint=midpoint,
        durable_stills=durable_stills,
        staging=staging,
        generation_mode=(
            "revalidated_existing_raw_assets"
            if args.revalidate_existing
            else "fresh_capture"
        ),
        prior_provenance=prior_provenance,
    )
    _prune_staging(args.output)
    provenance["capture_staging"]["staging_pruned"] = True
    metadata = write_pending_bundle(
        args,
        runtime_path=runtime_path,
        provenance=provenance,
        trace_equivalence=trace_equivalence,
        frame_validation=frame_validation,
        video_probe=video_probe,
        panel=panel,
        midpoint=midpoint,
    )
    return {
        "metadata": metadata,
        "frame_validation": frame_validation,
        "trace_equivalence": trace_equivalence,
        "video_probe": video_probe,
    }


def generate_pending(args: argparse.Namespace) -> dict[str, Any]:
    if args.revalidate_existing:
        args.output = _require_bundle_root(args.output, create=False)
        _validate_bundle_paths(
            args.output,
            complete=False,
            final=False,
            allow_staging=True,
            allow_manual=True,
        )
    else:
        args.output = ensure_empty_output(args.output)
    with _bundle_transaction(args.output):
        return _generate_pending_in_place(args)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if args.finalize_existing:
            finalized = finalize_existing(args)
            result = {
                "output": str(args.output),
                "status": finalized["status"],
                "claim_valid": finalized["claim_valid"],
                "manual_inspection_bound": finalized["manual_inspection_bound"],
            }
        elif args.verify_only:
            if args.manual_inspection_record is not None:
                raise ValueError(
                    "--manual-inspection-record is only valid with "
                    "--finalize-existing"
                )
            for executable in (args.trace, args.ffprobe):
                if not executable.is_file():
                    raise FileNotFoundError(executable)
            result = {"output": str(args.output), **verify_only(args)}
        else:
            if args.manual_inspection_record is not None:
                raise ValueError(
                    "--manual-inspection-record is only valid with "
                    "--finalize-existing"
                )
            for executable in (args.capture, args.trace, args.ffmpeg, args.ffprobe):
                if not executable.is_file():
                    raise FileNotFoundError(executable)
            generated = generate_pending(args)
            metadata = generated["metadata"]
            frame_validation = generated["frame_validation"]
            trace_equivalence = generated["trace_equivalence"]
            video_probe = generated["video_probe"]
            result = {
                "output": str(args.output),
                "status": metadata["status"],
                "claim_valid": metadata["claim_valid"],
                "frames": frame_validation["frame_count"],
                "unique_pixel_frames": frame_validation["unique_pixel_frames"],
                "video": video_probe["path"],
                "trace_equivalent": trace_equivalence["pass"],
                "staging_pruned": True,
            }
    except (
        FileNotFoundError,
        OSError,
        RuntimeError,
        ValueError,
        json.JSONDecodeError,
    ) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1

    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
