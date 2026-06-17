#!/usr/bin/env python3
"""Compare DART 6 and DART 7 Atlas SIMBICON pose evidence over selected steps."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import struct
from dataclasses import dataclass
from typing import Any

import numpy as np

_ROOT = pathlib.Path(__file__).resolve().parents[1]
_DEFAULT_TRACE_DIR = _ROOT / "build" / "default" / "simbicon_traces"
_DEFAULT_DART6_JSON = (
    _DEFAULT_TRACE_DIR / "dart6_atlas_simbicon_body_contact_summary_v3_0p40s.json"
)
_DEFAULT_DART7_JSON = (
    _DEFAULT_TRACE_DIR
    / "dart7_atlas_simbicon_body_contact_summary_explicit_dart6_gains_v3_model_0p40s.json"
)
_DEFAULT_MESH_DIR = _ROOT.parent / "task_6" / "data" / "sdf" / "atlas"
_DEFAULT_STEPS = (302, 311, 312, 338, 353, 354)
_JOINT_NAMES = (
    "l_leg_hpy",
    "l_leg_kny",
    "l_leg_aky",
    "r_leg_hpy",
    "r_leg_kny",
    "r_leg_aky",
)
_TRACE_METADATA_KEYS = (
    "steps_completed",
    "atlas_model_uri",
    "atlas_model_format",
    "atlas_initial_pose",
    "atlas_transition_mode",
    "atlas_sagittal_feedback_scale",
    "atlas_coronal_feedback_scale",
    "atlas_body_contact_min_depth",
    "atlas_ground_contact_min_depth",
    "atlas_joint_kp",
    "atlas_joint_kd",
    "native_contact_surface_tolerance",
    "native_contact_surface_seed_depth",
)


@dataclass(frozen=True)
class TraceSpec:
    label: str
    vertical_axis: int
    left_foot_mesh: pathlib.Path
    right_foot_mesh: pathlib.Path


def _load_stl_vertices(path: pathlib.Path) -> np.ndarray:
    data = path.read_bytes()
    if len(data) >= 84:
        triangle_count = struct.unpack_from("<I", data, 80)[0]
        expected_size = 84 + triangle_count * 50
        if triangle_count > 0 and expected_size == len(data):
            vertices = np.empty((triangle_count * 3, 3), dtype=float)
            offset = 84
            for triangle in range(triangle_count):
                values = struct.unpack_from("<12fH", data, offset)
                vertices[triangle * 3 + 0, :] = values[3:6]
                vertices[triangle * 3 + 1, :] = values[6:9]
                vertices[triangle * 3 + 2, :] = values[9:12]
                offset += 50
            return vertices

    vertices: list[tuple[float, float, float]] = []
    for raw_line in data.decode("utf-8", errors="ignore").splitlines():
        fields = raw_line.strip().split()
        if len(fields) == 4 and fields[0].lower() == "vertex":
            vertices.append((float(fields[1]), float(fields[2]), float(fields[3])))
    if not vertices:
        raise ValueError(f"could not read STL vertices from {path}")
    return np.asarray(vertices, dtype=float)


def _matrix4(value: object, *, name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} must be a finite 4x4 matrix")
    return matrix


def _vec3(value: object, *, name: str) -> tuple[float, float, float] | None:
    if value is None:
        return None
    vector = np.asarray(value, dtype=float).reshape(-1)
    if vector.shape[0] < 3 or not np.all(np.isfinite(vector[:3])):
        raise ValueError(f"{name} must expose at least three finite components")
    return (float(vector[0]), float(vector[1]), float(vector[2]))


def _vertical(value: object, axis: int, *, name: str) -> float | None:
    vector = _vec3(value, name=name)
    return None if vector is None else vector[axis]


def _finite_float(value: object) -> float | None:
    if value is None:
        return None
    try:
        parsed = float(value)
    except TypeError, ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def _transform_vertices(vertices: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return vertices @ transform[:3, :3].T + transform[:3, 3]


def _axis_unit(axis: int) -> np.ndarray:
    result = np.zeros(3, dtype=float)
    result[axis] = 1.0
    return result


def _local_axis_tilt_degrees(
    transform: np.ndarray, *, local_axis: int, vertical_axis: int
) -> float:
    axis = transform[:3, local_axis]
    norm = float(np.linalg.norm(axis))
    if norm <= 1e-12 or not math.isfinite(norm):
        raise ValueError("foot transform axis must have finite nonzero length")
    vertical = _axis_unit(vertical_axis)
    cosine = abs(float(axis @ vertical) / norm)
    cosine = min(1.0, max(-1.0, cosine))
    return math.degrees(math.acos(cosine))


def _foot_pose(
    sample: dict[str, Any], *, key: str, vertical_axis: int
) -> dict[str, Any]:
    transform = _matrix4(sample[key], name=key)
    return {
        "translation": [float(value) for value in transform[:3, 3]],
        "vertical": float(transform[vertical_axis, 3]),
        "local_z_axis_tilt_degrees": _local_axis_tilt_degrees(
            transform, local_axis=2, vertical_axis=vertical_axis
        ),
    }


def _mesh_bottom(
    *,
    sample: dict[str, Any],
    key: str,
    vertices: np.ndarray,
    vertical_axis: int,
) -> float:
    transform = _matrix4(sample[key], name=key)
    world_vertices = _transform_vertices(vertices, transform)
    return float(np.min(world_vertices[:, vertical_axis]))


def _sample_by_step(trace: dict[str, Any], step: int) -> dict[str, Any]:
    for sample in trace.get("samples", []):
        if sample.get("step") == step:
            return sample
    raise KeyError(f"trace has no sample at step {step}")


def _state(sample: dict[str, Any]) -> object:
    return sample.get("state_after_update", sample.get("controller_state"))


def _transitioned(sample: dict[str, Any]) -> bool:
    return bool(
        sample.get("state_transitioned_during_update", sample.get("state_transitioned"))
    )


def _side_from_foot_name(value: object) -> str | None:
    if value in ("l_foot", "left", "l"):
        return "left"
    if value in ("r_foot", "right", "r"):
        return "right"
    return None


def _limb_roles(sample: dict[str, Any]) -> dict[str, str]:
    stance = _side_from_foot_name(sample.get("stance_foot"))
    swing = _side_from_foot_name(sample.get("swing_foot"))
    if stance is None or swing is None:
        state = _finite_float(_state(sample))
        if state is not None:
            right_swing = int(state) in (0, 1)
            swing = "right" if right_swing else "left"
            stance = "left" if right_swing else "right"
    if stance is None or swing is None:
        return {}
    return {"stance": stance, "swing": swing}


def _contact_summary(sample: dict[str, Any], foot: str) -> dict[str, Any]:
    body = sample.get(f"{foot}_foot_body")
    if isinstance(body, dict):
        return {
            "count": body.get("count"),
            "mean_depth": _finite_float(body.get("mean_depth")),
            "max_depth": _finite_float(body.get("max_depth")),
        }
    return {
        "count": sample.get(f"{foot}_foot_body_contacts"),
        "mean_depth": _finite_float(sample.get(f"{foot}_foot_body_contact_mean_depth")),
        "max_depth": _finite_float(sample.get(f"{foot}_foot_body_contact_max_depth")),
    }


def _joint_values(sample: dict[str, Any], key: str) -> dict[str, float]:
    values = sample.get(key)
    if not isinstance(values, dict):
        return {}
    return {
        name: float(values[name])
        for name in _JOINT_NAMES
        if name in values and math.isfinite(float(values[name]))
    }


def _trace_metadata(trace: dict[str, Any]) -> dict[str, Any]:
    return {key: trace[key] for key in _TRACE_METADATA_KEYS if key in trace}


def _sample_summary(
    *,
    sample: dict[str, Any],
    spec: TraceSpec,
    left_vertices: np.ndarray,
    right_vertices: np.ndarray,
) -> dict[str, Any]:
    left_pose = _foot_pose(
        sample,
        key="left_foot_transform",
        vertical_axis=spec.vertical_axis,
    )
    right_pose = _foot_pose(
        sample,
        key="right_foot_transform",
        vertical_axis=spec.vertical_axis,
    )
    left_bottom = _mesh_bottom(
        sample=sample,
        key="left_foot_transform",
        vertices=left_vertices,
        vertical_axis=spec.vertical_axis,
    )
    right_bottom = _mesh_bottom(
        sample=sample,
        key="right_foot_transform",
        vertices=right_vertices,
        vertical_axis=spec.vertical_axis,
    )
    roles = _limb_roles(sample)
    pose_by_side = {"left": left_pose, "right": right_pose}
    return {
        "step": int(sample["step"]),
        "state": _state(sample),
        "transitioned": _transitioned(sample),
        "time": _finite_float(sample.get("time")),
        "left_mesh_bottom": left_bottom,
        "right_mesh_bottom": right_bottom,
        "left_minus_right_mesh_bottom": left_bottom - right_bottom,
        "left_ankle_vertical": _vertical(
            sample.get("left_ankle_position"),
            spec.vertical_axis,
            name="left_ankle_position",
        ),
        "right_ankle_vertical": _vertical(
            sample.get("right_ankle_position"),
            spec.vertical_axis,
            name="right_ankle_position",
        ),
        "pelvis_vertical": (
            _vertical(
                sample.get("pelvis", sample.get("pelvis_z")),
                spec.vertical_axis,
                name="pelvis",
            )
            if "pelvis" in sample
            else _finite_float(sample.get("pelvis_z"))
        ),
        "com_vertical": _vertical(
            sample.get("com"),
            spec.vertical_axis,
            name="com",
        ),
        "left_foot_pose": left_pose,
        "right_foot_pose": right_pose,
        "role_foot_pose": {
            role: pose_by_side[side]
            for role, side in roles.items()
            if side in pose_by_side
        },
        "left_foot_body": _contact_summary(sample, "left"),
        "right_foot_body": _contact_summary(sample, "right"),
        "joint_positions": _joint_values(sample, "joint_positions"),
        "joint_velocities": _joint_values(sample, "joint_velocities"),
        "requested_torques": _joint_values(sample, "requested_torques"),
    }


def _numeric_delta(
    dart7: dict[str, Any], dart6: dict[str, Any], keys: tuple[str, ...]
) -> dict[str, float]:
    result: dict[str, float] = {}
    for key in keys:
        lhs = dart7.get(key)
        rhs = dart6.get(key)
        if isinstance(lhs, (int, float)) and isinstance(rhs, (int, float)):
            result[key] = float(lhs) - float(rhs)
    return result


def _foot_pose_delta(dart7: dict[str, Any], dart6: dict[str, Any]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key in ("left_foot_pose", "right_foot_pose"):
        lhs = dart7.get(key)
        rhs = dart6.get(key)
        if not isinstance(lhs, dict) or not isinstance(rhs, dict):
            continue
        delta = _numeric_delta(
            lhs,
            rhs,
            ("vertical", "local_z_axis_tilt_degrees"),
        )
        lhs_translation = lhs.get("translation")
        rhs_translation = rhs.get("translation")
        if (
            isinstance(lhs_translation, list)
            and isinstance(rhs_translation, list)
            and len(lhs_translation) == 3
            and len(rhs_translation) == 3
        ):
            delta["translation_norm"] = float(
                np.linalg.norm(
                    np.asarray(lhs_translation) - np.asarray(rhs_translation)
                )
            )
        result[key] = delta

    role_result: dict[str, dict[str, float]] = {}
    dart7_roles = dart7.get("role_foot_pose")
    dart6_roles = dart6.get("role_foot_pose")
    if isinstance(dart7_roles, dict) and isinstance(dart6_roles, dict):
        for role in ("stance", "swing"):
            lhs = dart7_roles.get(role)
            rhs = dart6_roles.get(role)
            if isinstance(lhs, dict) and isinstance(rhs, dict):
                role_result[role] = _numeric_delta(
                    lhs,
                    rhs,
                    ("vertical", "local_z_axis_tilt_degrees"),
                )
    if role_result:
        result["role_foot_pose"] = role_result
    return result


def compare_traces(
    *,
    dart6_trace: dict[str, Any],
    dart7_trace: dict[str, Any],
    mesh_dir: pathlib.Path,
    steps: tuple[int, ...],
) -> dict[str, Any]:
    specs = {
        "dart6": TraceSpec(
            label="dart6",
            vertical_axis=1,
            left_foot_mesh=mesh_dir / "l_foot.stl",
            right_foot_mesh=mesh_dir / "r_foot.stl",
        ),
        "dart7": TraceSpec(
            label="dart7",
            vertical_axis=2,
            left_foot_mesh=mesh_dir / "l_foot.stl",
            right_foot_mesh=mesh_dir / "r_foot.stl",
        ),
    }
    left_vertices = _load_stl_vertices(specs["dart6"].left_foot_mesh)
    right_vertices = _load_stl_vertices(specs["dart6"].right_foot_mesh)
    samples: list[dict[str, Any]] = []
    for step in steps:
        dart6 = _sample_summary(
            sample=_sample_by_step(dart6_trace, step),
            spec=specs["dart6"],
            left_vertices=left_vertices,
            right_vertices=right_vertices,
        )
        dart7 = _sample_summary(
            sample=_sample_by_step(dart7_trace, step),
            spec=specs["dart7"],
            left_vertices=left_vertices,
            right_vertices=right_vertices,
        )
        samples.append(
            {
                "step": step,
                "dart6": dart6,
                "dart7": dart7,
                "dart7_minus_dart6": _numeric_delta(
                    dart7,
                    dart6,
                    (
                        "left_minus_right_mesh_bottom",
                        "left_ankle_vertical",
                        "right_ankle_vertical",
                        "pelvis_vertical",
                        "com_vertical",
                    ),
                ),
                "foot_pose_dart7_minus_dart6": _foot_pose_delta(dart7, dart6),
            }
        )

    return {
        "source": "compare_atlas_simbicon_pose_window",
        "dart6_vertical_axis": "y",
        "dart7_vertical_axis": "z",
        "dart6_trace": _trace_metadata(dart6_trace),
        "dart7_trace": _trace_metadata(dart7_trace),
        "mesh_dir": str(mesh_dir),
        "steps": list(steps),
        "samples": samples,
    }


def _parse_steps(value: str) -> tuple[int, ...]:
    steps = tuple(int(part) for part in value.split(",") if part.strip())
    if not steps:
        raise argparse.ArgumentTypeError("at least one step is required")
    return steps


def _format_mm(value: float | None) -> str:
    if value is None:
        return "n/a"
    return f"{value * 1000.0:.2f}"


def _format_float(value: object) -> str:
    parsed = _finite_float(value)
    if parsed is None:
        return "n/a"
    return f"{parsed:.2f}"


def _print_summary(result: dict[str, Any]) -> None:
    print("Atlas SIMBICON pose-window comparison")
    print(
        "step  d6_L-R_mm  d7_L-R_mm  delta_mm  "
        "d7_stance_tilt_deg  d7_left_contacts  d6_left_contacts"
    )
    for row in result["samples"]:
        dart6 = row["dart6"]
        dart7 = row["dart7"]
        delta = row["dart7_minus_dart6"].get("left_minus_right_mesh_bottom")
        stance_pose = dart7.get("role_foot_pose", {}).get("stance", {})
        print(
            f"{row['step']:>4}  "
            f"{_format_mm(dart6['left_minus_right_mesh_bottom']):>9}  "
            f"{_format_mm(dart7['left_minus_right_mesh_bottom']):>9}  "
            f"{_format_mm(delta):>8}  "
            f"{_format_float(stance_pose.get('local_z_axis_tilt_degrees')):>18}  "
            f"{dart7['left_foot_body']['count']!s:>16}  "
            f"{dart6['left_foot_body']['count']!s:>16}"
        )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Compare selected DART 6 and DART 7 Atlas SIMBICON trace samples "
            "using the shared Atlas foot meshes."
        )
    )
    parser.add_argument("--dart6-json", type=pathlib.Path, default=_DEFAULT_DART6_JSON)
    parser.add_argument("--dart7-json", type=pathlib.Path, default=_DEFAULT_DART7_JSON)
    parser.add_argument("--mesh-dir", type=pathlib.Path, default=_DEFAULT_MESH_DIR)
    parser.add_argument("--steps", type=_parse_steps, default=_DEFAULT_STEPS)
    parser.add_argument("--json-out", type=pathlib.Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    dart6_trace = json.loads(args.dart6_json.read_text(encoding="utf-8"))
    dart7_trace = json.loads(args.dart7_json.read_text(encoding="utf-8"))
    result = compare_traces(
        dart6_trace=dart6_trace,
        dart7_trace=dart7_trace,
        mesh_dir=args.mesh_dir,
        steps=args.steps,
    )
    _print_summary(result)
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(f"Wrote {args.json_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
