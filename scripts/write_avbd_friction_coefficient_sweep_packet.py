#!/usr/bin/env python3
"""Write a validated AVBD friction-coefficient sweep benchmark packet."""

from __future__ import annotations

import argparse
import json
import math
import re
import struct
import sys
from hashlib import sha256
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from avbd_packet_schema import AVBD_PACKET_SCHEMA_VERSION  # noqa: E402

DEFAULT_OUTPUT = Path(
    "docs/plans/104-vertex-block-descent-solver/"
    "avbd-friction-coefficient-sweep-packet.json"
)
SCENE_ID = "avbd_demo2d_dynamic_friction"
BENCHMARK_NAME = "BM_AvbdDemo2dFrictionCoefficientSweep"
EXPECTED_MAX_FRICTIONS = (0.0, 0.5, 1.0, 2.5, 5.0)
BOX_COUNT = 11
RIGID_BODIES = 12
COLLISION_SHAPES = 12
SOURCE_SCENE_INDEX = 2
TIME_STEP = 1.0 / 60.0
RESOLVED_SOLVER_IDENTITY = {
    "avbd_rigid_contact_config_emplaced": False,
    "recorded_from": "friction coefficient sweep benchmark scene counters",
    "rigid_contact_solver": "sequential_impulse",
    "rigid_point_joint_solver": "none",
}
_AGGREGATE_SUFFIX_RE = re.compile(r"_(?:mean|median|stddev|cv)$")
_REPEATS_SUFFIX_RE = re.compile(r"/repeats:\d+")
_SVG_SIZE_RE = re.compile(
    r"<svg\b[^>]*\bwidth=\"([0-9.]+)\"[^>]*\bheight=\"([0-9.]+)\""
)


class AvbdFrictionCoefficientSweepPacketError(RuntimeError):
    pass


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--benchmark-json", type=Path, required=True)
    parser.add_argument(
        "--reference-timing-json",
        type=Path,
        action="append",
        default=[],
        help=(
            "JSON from scripts/run_avbd_demo2d_reference_timing.py. Pass one "
            "file per expected friction value to embed a native source sweep."
        ),
    )
    parser.add_argument(
        "--capture-manifest",
        type=Path,
        action="append",
        default=[],
        help=(
            "manifest.json from scripts/capture_py_demo.py. Pass one "
            "metadata-tagged manifest per expected friction value."
        ),
    )
    parser.add_argument("--plot-svg", type=Path)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _load_json(path: Path) -> dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise AvbdFrictionCoefficientSweepPacketError(
            f"{path}: file not found"
        ) from exc
    except json.JSONDecodeError as exc:
        raise AvbdFrictionCoefficientSweepPacketError(
            f"{path}: invalid JSON: {exc}"
        ) from exc
    if not isinstance(data, dict):
        raise AvbdFrictionCoefficientSweepPacketError(
            f"{path}: top-level JSON value must be an object"
        )
    return data


def _sha256(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _validate_plot(plot_svg: Path) -> dict[str, Any]:
    try:
        text = plot_svg.read_text(encoding="utf-8")
    except FileNotFoundError as exc:
        raise AvbdFrictionCoefficientSweepPacketError(
            f"{plot_svg}: plot SVG not found"
        ) from exc
    if "<svg" not in text:
        raise AvbdFrictionCoefficientSweepPacketError(
            f"{plot_svg}: plot artifact must be SVG"
        )
    match = _SVG_SIZE_RE.search(text)
    if match is None:
        raise AvbdFrictionCoefficientSweepPacketError(
            f"{plot_svg}: plot SVG missing width/height"
        )
    return {
        "file": str(plot_svg),
        "sha256": _sha256(plot_svg),
        "width": float(match.group(1)),
        "height": float(match.group(2)),
    }


def _png_dimensions(path: Path) -> tuple[int, int]:
    with path.open("rb") as file:
        header = file.read(24)
    if (
        len(header) != 24
        or header[:8] != b"\x89PNG\r\n\x1a\n"
        or header[12:16] != b"IHDR"
    ):
        raise AvbdFrictionCoefficientSweepPacketError(f"{path}: expected a PNG image")
    width, height = struct.unpack(">II", header[16:24])
    if width < 1 or height < 1:
        raise AvbdFrictionCoefficientSweepPacketError(f"{path}: invalid PNG dimensions")
    return width, height


def _artifact_path(manifest_dir: Path, value: object, key: str) -> Path:
    if not isinstance(value, str) or not value:
        raise AvbdFrictionCoefficientSweepPacketError(f"capture manifest missing {key}")
    path = Path(value)
    if not path.is_absolute():
        path = manifest_dir / path
    return path


def _artifact_label(manifest_dir: Path, path: Path) -> str:
    try:
        return path.resolve().relative_to(manifest_dir.resolve()).as_posix()
    except ValueError:
        return path.name


def _row_name(row: dict[str, Any]) -> str:
    name = row.get("run_name", row.get("name", ""))
    return name if isinstance(name, str) else ""


def _canonical_name(name: str) -> str:
    return _AGGREGATE_SUFFIX_RE.sub("", _REPEATS_SUFFIX_RE.sub("", name))


def _benchmark_family(name: str) -> str:
    return _canonical_name(name).split("/", 1)[0]


def _finite_counter(row: dict[str, Any], key: str) -> float:
    value = row.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise AvbdFrictionCoefficientSweepPacketError(
            f"{BENCHMARK_NAME}: missing {key}"
        )
    value = float(value)
    if not math.isfinite(value):
        raise AvbdFrictionCoefficientSweepPacketError(
            f"{BENCHMARK_NAME}: non-finite {key}"
        )
    return value


def _require_counter(row: dict[str, Any], key: str, expected: float) -> None:
    value = _finite_counter(row, key)
    if not math.isclose(value, expected, rel_tol=0.0, abs_tol=1e-15):
        raise AvbdFrictionCoefficientSweepPacketError(
            f"{BENCHMARK_NAME}: expected {key}={expected:g}, got {value:g}"
        )


def _is_representative(row: dict[str, Any]) -> bool:
    run_type = row.get("run_type", "iteration")
    aggregate_name = row.get("aggregate_name")
    return run_type == "iteration" or aggregate_name in ("mean", "median")


def _timing_row(rows: list[dict[str, Any]]) -> dict[str, Any]:
    median_rows = [row for row in rows if row.get("aggregate_name") == "median"]
    if median_rows:
        return median_rows[0]
    mean_rows = [row for row in rows if row.get("aggregate_name") == "mean"]
    if mean_rows:
        return mean_rows[0]
    return rows[0]


def _friction_key(value: float) -> int:
    return int(round(value * 10.0))


def _expected_friction_by_key() -> dict[int, float]:
    return {_friction_key(value): value for value in EXPECTED_MAX_FRICTIONS}


def _validate_representative_row(row: dict[str, Any], max_friction: float) -> None:
    _require_counter(row, "rigid_bodies", float(RIGID_BODIES))
    _require_counter(row, "rigid_body_joints", 0.0)
    _require_counter(row, "collision_shapes", float(COLLISION_SHAPES))
    _require_counter(row, "source_scene_index", float(SOURCE_SCENE_INDEX))
    _require_counter(row, "max_friction", max_friction)
    _require_counter(row, "min_friction", 0.0)
    _require_counter(row, "friction_samples", float(BOX_COUNT))
    for key in ("real_time", "cpu_time"):
        _finite_counter(row, key)


def _validate_benchmark(benchmark_json: Path) -> dict[str, Any]:
    data = _load_json(benchmark_json)
    rows = data.get("benchmarks")
    if not isinstance(rows, list):
        raise AvbdFrictionCoefficientSweepPacketError(
            "benchmark JSON missing benchmarks list"
        )

    matching_rows = [
        row
        for row in rows
        if isinstance(row, dict) and _benchmark_family(_row_name(row)) == BENCHMARK_NAME
    ]
    if not matching_rows:
        raise AvbdFrictionCoefficientSweepPacketError(
            f"benchmark JSON missing {BENCHMARK_NAME}"
        )

    expected_by_key = _expected_friction_by_key()
    representative_by_key: dict[int, list[dict[str, Any]]] = {
        key: [] for key in expected_by_key
    }
    packet_rows: list[dict[str, Any]] = []
    for row in matching_rows:
        for key in ("real_time", "cpu_time"):
            _finite_counter(row, key)
        packet_rows.append(row)

        if not _is_representative(row):
            continue
        max_friction = _finite_counter(row, "max_friction")
        friction_key = _friction_key(max_friction)
        if friction_key not in representative_by_key:
            raise AvbdFrictionCoefficientSweepPacketError(
                f"{BENCHMARK_NAME}: unexpected max_friction={max_friction:g}"
            )
        expected = expected_by_key[friction_key]
        _validate_representative_row(row, expected)
        representative_by_key[friction_key].append(row)

    missing = [
        expected_by_key[key]
        for key, rows_for_key in representative_by_key.items()
        if not rows_for_key
    ]
    if missing:
        missing_text = ", ".join(f"{value:g}" for value in missing)
        raise AvbdFrictionCoefficientSweepPacketError(
            f"{BENCHMARK_NAME}: missing max_friction values: {missing_text}"
        )

    plot_data = []
    for friction_key in sorted(representative_by_key):
        timing_row = _timing_row(representative_by_key[friction_key])
        max_friction = expected_by_key[friction_key]
        plot_data.append(
            {
                "max_friction": max_friction,
                "cpu_time_per_step_ns": _finite_counter(timing_row, "cpu_time"),
                "real_time_per_step_ns": _finite_counter(timing_row, "real_time"),
                "friction_samples": _finite_counter(timing_row, "friction_samples"),
                "time_unit": timing_row.get("time_unit", "ns"),
            }
        )

    context = data.get("context", {})
    if not isinstance(context, dict):
        context = {}
    return {
        "json_sha256": _sha256(benchmark_json),
        "benchmark": BENCHMARK_NAME,
        "context": {
            key: context[key]
            for key in (
                "executable",
                "num_cpus",
                "mhz_per_cpu",
                "library_version",
                "library_build_type",
                "json_schema_version",
            )
            if key in context
        },
        "rows": packet_rows,
        "plot_data": plot_data,
        "invariants": {
            "source_demo": "avbd-demo2d",
            "source_scene": "Dynamic Friction",
            "source_scene_index": SOURCE_SCENE_INDEX,
            "time_step": TIME_STEP,
            "rigid_bodies": RIGID_BODIES,
            "collision_shapes": COLLISION_SHAPES,
            "friction_samples": BOX_COUNT,
            "max_friction": list(EXPECTED_MAX_FRICTIONS),
            "min_friction": 0.0,
        },
    }


def _finite_field(data: dict[str, Any], key: str) -> float:
    value = data.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise AvbdFrictionCoefficientSweepPacketError(f"reference timing missing {key}")
    value = float(value)
    if not math.isfinite(value):
        raise AvbdFrictionCoefficientSweepPacketError(
            f"reference timing has non-finite {key}"
        )
    return value


def _int_field(data: dict[str, Any], key: str) -> int:
    value = data.get(key)
    if not isinstance(value, int) or isinstance(value, bool):
        raise AvbdFrictionCoefficientSweepPacketError(
            f"reference timing missing integer {key}"
        )
    return value


def _validate_reference_timing(
    reference_json: Path,
    expected_max_friction: float,
) -> dict[str, Any]:
    data = _load_json(reference_json)
    expected = {
        "schema_version": 1,
        "source_demo": "avbd-demo2d",
        "source_revision": "74699a11f858",
        "scene_index": SOURCE_SCENE_INDEX,
        "scene_name": "Dynamic Friction",
        "scene_builder": "sceneDynamicFriction",
    }
    for key, value in expected.items():
        if data.get(key) != value:
            raise AvbdFrictionCoefficientSweepPacketError(
                f"reference timing expected {key}={value!r}, " f"got {data.get(key)!r}"
            )
    if _int_field(data, "rigid_bodies") != RIGID_BODIES:
        raise AvbdFrictionCoefficientSweepPacketError(
            f"reference timing expected {RIGID_BODIES} rigid bodies"
        )
    if _int_field(data, "dynamic_bodies") != BOX_COUNT:
        raise AvbdFrictionCoefficientSweepPacketError(
            f"reference timing expected {BOX_COUNT} dynamic bodies"
        )
    if _int_field(data, "static_bodies") != 1:
        raise AvbdFrictionCoefficientSweepPacketError(
            "reference timing expected 1 static body"
        )
    if _int_field(data, "joints") != 0:
        raise AvbdFrictionCoefficientSweepPacketError(
            "reference timing expected 0 joints"
        )
    if _int_field(data, "collision_shapes") != COLLISION_SHAPES:
        raise AvbdFrictionCoefficientSweepPacketError(
            f"reference timing expected {COLLISION_SHAPES} collision shapes"
        )
    if _int_field(data, "box_count") != BOX_COUNT:
        raise AvbdFrictionCoefficientSweepPacketError(
            f"reference timing expected box_count={BOX_COUNT}"
        )
    if _int_field(data, "friction_samples") != BOX_COUNT:
        raise AvbdFrictionCoefficientSweepPacketError(
            f"reference timing expected friction_samples={BOX_COUNT}"
        )
    if _int_field(data, "steps") < 1:
        raise AvbdFrictionCoefficientSweepPacketError(
            "reference timing steps must be positive"
        )
    if _int_field(data, "warmup_steps") < 0:
        raise AvbdFrictionCoefficientSweepPacketError(
            "reference timing warmup_steps must be non-negative"
        )
    if not math.isclose(
        _finite_field(data, "initial_speed"),
        10.0,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise AvbdFrictionCoefficientSweepPacketError(
            "reference timing expected initial_speed=10"
        )
    for key in ("requested_max_friction", "dynamic_max_friction"):
        if not math.isclose(
            _finite_field(data, key),
            expected_max_friction,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise AvbdFrictionCoefficientSweepPacketError(
                f"reference timing expected {key}={expected_max_friction:g}"
            )
    if not math.isclose(
        _finite_field(data, "dynamic_min_friction"),
        0.0,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise AvbdFrictionCoefficientSweepPacketError(
            "reference timing expected dynamic_min_friction=0"
        )
    for key in (
        "elapsed_ns",
        "cpu_time_per_step_ns",
        "final_time",
        "min_friction",
        "max_friction",
    ):
        _finite_field(data, key)

    return {
        "json_sha256": _sha256(reference_json),
        "source_demo": data["source_demo"],
        "repository": data.get(
            "repository", "https://github.com/savant117/avbd-demo2d"
        ),
        "source_revision": data["source_revision"],
        "scene_index": data["scene_index"],
        "scene_name": data["scene_name"],
        "scene_builder": data["scene_builder"],
        "compiler": data.get("compiler"),
        "compile_flags": data.get("compile_flags"),
        "compile_command": data.get("compile_command"),
        "warmup_steps": data["warmup_steps"],
        "steps": data["steps"],
        "elapsed_ns": data["elapsed_ns"],
        "cpu_time_per_step_ns": data["cpu_time_per_step_ns"],
        "max_friction": expected_max_friction,
        "invariants": {
            "rigid_bodies": data["rigid_bodies"],
            "dynamic_bodies": data["dynamic_bodies"],
            "static_bodies": data["static_bodies"],
            "joints": data["joints"],
            "collision_shapes": data["collision_shapes"],
            "box_count": data["box_count"],
            "friction_samples": data["friction_samples"],
            "initial_speed": data["initial_speed"],
            "requested_max_friction": data["requested_max_friction"],
            "dynamic_min_friction": data["dynamic_min_friction"],
            "dynamic_max_friction": data["dynamic_max_friction"],
            "overall_min_friction": data["min_friction"],
            "overall_max_friction": data["max_friction"],
            "final_time": data["final_time"],
        },
    }


def _validate_reference_sweep(
    reference_jsons: list[Path],
    benchmark: dict[str, Any],
) -> dict[str, Any] | None:
    if not reference_jsons:
        return None

    expected_by_key = _expected_friction_by_key()
    rows_by_key: dict[int, dict[str, Any]] = {}
    for reference_json in reference_jsons:
        data = _load_json(reference_json)
        max_friction = _finite_field(data, "requested_max_friction")
        friction_key = _friction_key(max_friction)
        if friction_key not in expected_by_key:
            raise AvbdFrictionCoefficientSweepPacketError(
                f"reference timing unexpected max_friction={max_friction:g}"
            )
        if friction_key in rows_by_key:
            raise AvbdFrictionCoefficientSweepPacketError(
                f"reference timing duplicate max_friction={max_friction:g}"
            )
        expected = expected_by_key[friction_key]
        rows_by_key[friction_key] = _validate_reference_timing(reference_json, expected)

    missing = [
        expected_by_key[key] for key in expected_by_key if key not in rows_by_key
    ]
    if missing:
        missing_text = ", ".join(f"{value:g}" for value in missing)
        raise AvbdFrictionCoefficientSweepPacketError(
            f"reference timing missing max_friction values: {missing_text}"
        )

    benchmark_by_key = {
        _friction_key(row["max_friction"]): row for row in benchmark["plot_data"]
    }
    plot_data = []
    comparison = []
    for friction_key in sorted(rows_by_key):
        reference_row = rows_by_key[friction_key]
        max_friction = expected_by_key[friction_key]
        reference_ns = float(reference_row["cpu_time_per_step_ns"])
        dart_ns = float(benchmark_by_key[friction_key]["cpu_time_per_step_ns"])
        plot_data.append(
            {
                "max_friction": max_friction,
                "cpu_time_per_step_ns": reference_ns,
                "time_unit": "ns",
            }
        )
        ratio = dart_ns / reference_ns
        comparison.append(
            {
                "max_friction": max_friction,
                "dart_cpu_time_per_step_ns": dart_ns,
                "reference_cpu_time_per_step_ns": reference_ns,
                "dart_to_reference_cpu_time_ratio": ratio,
                "dart_faster_than_reference": ratio < 1.0,
            }
        )

    return {
        "source_demo": "avbd-demo2d",
        "repository": "https://github.com/savant117/avbd-demo2d",
        "source_revision": "74699a11f858",
        "scene_index": SOURCE_SCENE_INDEX,
        "scene_name": "Dynamic Friction",
        "scene_builder": "sceneDynamicFriction",
        "rows": [rows_by_key[key] for key in sorted(rows_by_key)],
        "plot_data": plot_data,
        "comparison": comparison,
        "all_dart_faster_than_reference": all(
            row["dart_faster_than_reference"] for row in comparison
        ),
    }


def _manifest_max_friction(manifest: dict[str, Any]) -> float:
    metadata = manifest.get("metadata")
    if not isinstance(metadata, dict):
        raise AvbdFrictionCoefficientSweepPacketError(
            "capture manifest missing metadata"
        )
    value = metadata.get("max_friction")
    if not isinstance(value, str):
        raise AvbdFrictionCoefficientSweepPacketError(
            "capture manifest metadata missing max_friction"
        )
    try:
        max_friction = float(value)
    except ValueError as exc:
        raise AvbdFrictionCoefficientSweepPacketError(
            "capture manifest max_friction must be numeric"
        ) from exc
    if not math.isfinite(max_friction):
        raise AvbdFrictionCoefficientSweepPacketError(
            "capture manifest max_friction must be finite"
        )
    return max_friction


def _validate_capture_manifest(
    manifest_path: Path,
    expected_max_friction: float,
) -> dict[str, Any]:
    manifest = _load_json(manifest_path)
    if manifest.get("schema_version") != 1:
        raise AvbdFrictionCoefficientSweepPacketError(
            "capture manifest schema_version must be 1"
        )
    if manifest.get("scene") != SCENE_ID:
        raise AvbdFrictionCoefficientSweepPacketError(
            f"capture scene must be {SCENE_ID}"
        )
    if manifest.get("switch_scene") is not None:
        raise AvbdFrictionCoefficientSweepPacketError(
            "friction sweep capture must not switch scenes"
        )
    if manifest.get("force_drag") is not None:
        raise AvbdFrictionCoefficientSweepPacketError(
            "friction sweep capture must not force-drag"
        )
    max_friction = _manifest_max_friction(manifest)
    if not math.isclose(
        max_friction,
        expected_max_friction,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise AvbdFrictionCoefficientSweepPacketError(
            "capture manifest max_friction does not match expected value"
        )

    scene_env = manifest.get("scene_environment")
    if not isinstance(scene_env, dict):
        raise AvbdFrictionCoefficientSweepPacketError(
            "capture manifest missing scene_environment"
        )
    env_value = scene_env.get("DART_AVBD_DEMO2D_DYNAMIC_FRICTION_MAX_FRICTION")
    try:
        env_max_friction = float(env_value)
    except (TypeError, ValueError) as exc:
        raise AvbdFrictionCoefficientSweepPacketError(
            "capture manifest missing matching Dynamic Friction scene env"
        ) from exc
    if not math.isfinite(env_max_friction) or not math.isclose(
        env_max_friction,
        expected_max_friction,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise AvbdFrictionCoefficientSweepPacketError(
            "capture manifest missing matching Dynamic Friction scene env"
        )

    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise AvbdFrictionCoefficientSweepPacketError(
            "capture manifest missing artifacts"
        )
    manifest_dir = manifest_path.parent
    screenshot = _artifact_path(manifest_dir, artifacts.get("screenshot"), "screenshot")
    frames_dir = _artifact_path(manifest_dir, artifacts.get("frames"), "frames")
    if not screenshot.is_file():
        raise AvbdFrictionCoefficientSweepPacketError(
            f"{screenshot}: screenshot not found"
        )
    if not frames_dir.is_dir():
        raise AvbdFrictionCoefficientSweepPacketError(
            f"{frames_dir}: frame directory not found"
        )
    frame_paths = sorted(frames_dir.glob("frame_*.png"))
    if not frame_paths:
        raise AvbdFrictionCoefficientSweepPacketError(
            f"{frames_dir}: no PNG frames found"
        )
    width, height = _png_dimensions(screenshot)
    if _png_dimensions(frame_paths[0]) != (width, height):
        raise AvbdFrictionCoefficientSweepPacketError(
            "first frame dimensions do not match screenshot"
        )
    if _png_dimensions(frame_paths[-1]) != (width, height):
        raise AvbdFrictionCoefficientSweepPacketError(
            "last frame dimensions do not match screenshot"
        )
    ui_ready = manifest.get("ui_ready")
    if not isinstance(ui_ready, dict):
        raise AvbdFrictionCoefficientSweepPacketError(
            "capture manifest missing ui_ready"
        )

    return {
        "manifest_sha256": _sha256(manifest_path),
        "max_friction": expected_max_friction,
        "scene": SCENE_ID,
        "show_ui": bool(manifest.get("show_ui")),
        "ui_ready": ui_ready,
        "screenshot": {
            "file": _artifact_label(manifest_dir, screenshot),
            "sha256": _sha256(screenshot),
            "width": width,
            "height": height,
        },
        "frames": {
            "directory": _artifact_label(manifest_dir, frames_dir),
            "count": len(frame_paths),
            "first_frame": {
                "file": _artifact_label(manifest_dir, frame_paths[0]),
                "sha256": _sha256(frame_paths[0]),
            },
            "last_frame": {
                "file": _artifact_label(manifest_dir, frame_paths[-1]),
                "sha256": _sha256(frame_paths[-1]),
            },
        },
    }


def _validate_visual_sweep(capture_manifests: list[Path]) -> dict[str, Any] | None:
    if not capture_manifests:
        return None

    expected_by_key = _expected_friction_by_key()
    captures_by_key: dict[int, dict[str, Any]] = {}
    for manifest_path in capture_manifests:
        manifest = _load_json(manifest_path)
        max_friction = _manifest_max_friction(manifest)
        friction_key = _friction_key(max_friction)
        if friction_key not in expected_by_key:
            raise AvbdFrictionCoefficientSweepPacketError(
                f"capture manifest unexpected max_friction={max_friction:g}"
            )
        if friction_key in captures_by_key:
            raise AvbdFrictionCoefficientSweepPacketError(
                f"capture manifest duplicate max_friction={max_friction:g}"
            )
        captures_by_key[friction_key] = _validate_capture_manifest(
            manifest_path,
            expected_by_key[friction_key],
        )

    missing = [
        expected_by_key[key] for key in expected_by_key if key not in captures_by_key
    ]
    if missing:
        missing_text = ", ".join(f"{value:g}" for value in missing)
        raise AvbdFrictionCoefficientSweepPacketError(
            f"capture manifests missing max_friction values: {missing_text}"
        )
    return {
        "scene": SCENE_ID,
        "captures": [captures_by_key[key] for key in sorted(captures_by_key)],
    }


def make_packet(
    benchmark_json: Path,
    reference_timing_jsons: list[Path] | None = None,
    capture_manifests: list[Path] | None = None,
    plot_svg: Path | None = None,
) -> dict[str, Any]:
    benchmark = _validate_benchmark(benchmark_json)
    reference_sweep = _validate_reference_sweep(
        reference_timing_jsons or [],
        benchmark,
    )
    visual_sweep = _validate_visual_sweep(capture_manifests or [])
    packet = {
        "schema_version": AVBD_PACKET_SCHEMA_VERSION,
        "resolved_solver_identity": RESOLVED_SOLVER_IDENTITY,
        "packet": "avbd_friction_coefficient_sweep",
        "scene": SCENE_ID,
        "target": {
            "paper_gap": "friction coefficient comparison",
            "scope": (
                "benchmark-selected sweep over the source-shaped "
                "avbd-demo2d Dynamic Friction scene"
            ),
            "complete_paper_reproduction": False,
        },
        "scene_invariants": {
            "source_demo": "avbd-demo2d",
            "scene_name": "Dynamic Friction",
            "scene_index": SOURCE_SCENE_INDEX,
            "box_count": BOX_COUNT,
            "time_step": TIME_STEP,
            "initial_velocity": [10.0, 0.0, 0.0],
            "max_friction": list(EXPECTED_MAX_FRICTIONS),
            "min_friction": 0.0,
        },
        "visual_anchor": {
            "scene": SCENE_ID,
            "existing_packet": "avbd-demo2d-dynamic-friction-packet.json",
            "note": (
                "This sweep reuses the source-shaped Dynamic Friction visual scene. "
                "When visual_sweep is present, each coefficient has its own capture "
                "manifest hash."
            ),
        },
        "benchmark": benchmark,
        "reproduction": {
            "benchmark_command": (
                "build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint "
                "--benchmark_filter=BM_AvbdDemo2dFrictionCoefficientSweep "
                "--benchmark_min_time=0.5s --benchmark_repetitions=3 "
                "--benchmark_out=<benchmark-json> --benchmark_out_format=json"
            ),
            "reference_timing_command": (
                "python scripts/run_avbd_demo2d_reference_timing.py "
                "--source-dir <avbd-demo2d-source-dir> --scene dynamic_friction "
                "--dynamic-friction-max-friction <max-friction> "
                "--output <reference-timing-json>"
            ),
            "visual_capture_command": (
                "DART_AVBD_DEMO2D_DYNAMIC_FRICTION_MAX_FRICTION=<max-friction> "
                "python scripts/capture_py_demo.py "
                "--scene avbd_demo2d_dynamic_friction "
                "--metadata max_friction=<max-friction> "
                "--env DART_AVBD_DEMO2D_DYNAMIC_FRICTION_MAX_FRICTION=<max-friction> "
                "--frames 24 --width 640 --height 360 --output-dir <capture-dir>"
            ),
        },
    }
    if reference_sweep is not None:
        packet["reference_sweep"] = reference_sweep
    if visual_sweep is not None:
        packet["visual_sweep"] = visual_sweep
    if plot_svg is not None:
        packet["rendered_plot"] = _validate_plot(plot_svg)
    remaining_gates = [
        "same-hardware paper/site friction comparison",
        "GPU AVBD row parity and benchmark packets",
    ]
    if reference_sweep is None:
        remaining_gates.insert(0, "source/reference friction-sweep timing comparison")
    elif not reference_sweep["all_dart_faster_than_reference"]:
        remaining_gates.insert(
            0,
            "DART CPU performance must beat the native source sweep before a CPU win is claimed for every friction coefficient",
        )
    if visual_sweep is None:
        remaining_gates.insert(0, "per-coefficient visual capture or video evidence")
    if plot_svg is None:
        remaining_gates.insert(0, "rendered friction-sweep plot")
    packet["remaining_gates"] = remaining_gates
    return packet


def write_packet(path: Path, packet: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(packet, indent=2, sort_keys=True) + "\n")


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        packet = make_packet(
            args.benchmark_json,
            args.reference_timing_json,
            args.capture_manifest,
            args.plot_svg,
        )
    except AvbdFrictionCoefficientSweepPacketError as exc:
        raise SystemExit(str(exc)) from exc
    write_packet(args.output, packet)
    print(f"Wrote AVBD friction-coefficient sweep packet: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
