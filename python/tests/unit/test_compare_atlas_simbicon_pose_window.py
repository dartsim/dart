from __future__ import annotations

import importlib.util
import math
import pathlib
import struct
import sys

import pytest

_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SPEC = importlib.util.spec_from_file_location(
    "compare_atlas_simbicon_pose_window",
    _ROOT / "scripts" / "compare_atlas_simbicon_pose_window.py",
)
assert _SPEC is not None
assert _SPEC.loader is not None
compare_atlas_simbicon_pose_window = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = compare_atlas_simbicon_pose_window
_SPEC.loader.exec_module(compare_atlas_simbicon_pose_window)


def _write_binary_stl(path: pathlib.Path) -> None:
    vertices = (
        (0.0, 0.0, 0.0),
        (0.2, 0.0, 0.0),
        (0.0, 0.1, 0.3),
    )
    payload = bytearray(b"test".ljust(80, b"\0"))
    payload.extend(struct.pack("<I", 1))
    payload.extend(
        struct.pack(
            "<12fH",
            0.0,
            0.0,
            1.0,
            *vertices[0],
            *vertices[1],
            *vertices[2],
            0,
        )
    )
    path.write_bytes(bytes(payload))


def _transform(*, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> list[list[float]]:
    return [
        [1.0, 0.0, 0.0, x],
        [0.0, 1.0, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _dart6_y_up_foot_transform(
    *, x: float = 0.0, y: float = 0.0, z: float = 0.0
) -> list[list[float]]:
    return [
        [1.0, 0.0, 0.0, x],
        [0.0, 0.0, 1.0, y],
        [0.0, -1.0, 0.0, z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _dart7_z_up_foot_transform(
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    tilt_degrees: float = 0.0,
) -> list[list[float]]:
    angle = math.radians(tilt_degrees)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return [
        [1.0, 0.0, 0.0, x],
        [0.0, cosine, -sine, y],
        [0.0, sine, cosine, z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def test_compare_traces_uses_dart6_y_and_dart7_z_vertical_axes(
    tmp_path: pathlib.Path,
) -> None:
    mesh_dir = tmp_path / "atlas"
    mesh_dir.mkdir()
    _write_binary_stl(mesh_dir / "l_foot.stl")
    _write_binary_stl(mesh_dir / "r_foot.stl")
    dart6_trace = {
        "samples": [
            {
                "step": 302,
                "time": 0.302,
                "state_after_update": "0",
                "left_foot_transform": _dart6_y_up_foot_transform(y=1.0),
                "right_foot_transform": _dart6_y_up_foot_transform(y=0.7),
                "left_ankle_position": [0.0, 1.2, 0.0],
                "right_ankle_position": [0.0, 0.8, 0.0],
                "pelvis": [0.0, 2.0, 0.0],
                "com": [0.0, 2.2, 0.0],
                "left_foot_body": {"count": 0, "mean_depth": None, "max_depth": None},
                "right_foot_body": {"count": 4, "mean_depth": 0.1, "max_depth": 0.2},
                "joint_positions": {"l_leg_kny": 1.0, "r_leg_aky": -0.2},
            }
        ]
    }
    dart7_trace = {
        "atlas_joint_kp": 1000.0,
        "atlas_joint_kd": 1.0,
        "samples": [
            {
                "step": 302,
                "time": 0.302,
                "controller_state": 0,
                "left_foot_transform": _dart7_z_up_foot_transform(
                    z=2.0, tilt_degrees=30.0
                ),
                "right_foot_transform": _dart7_z_up_foot_transform(z=1.4),
                "left_ankle_position": [0.0, 0.0, 2.4],
                "right_ankle_position": [0.0, 0.0, 1.5],
                "pelvis_z": 3.0,
                "com": [0.0, 0.0, 3.2],
                "left_foot_body_contacts": 3,
                "left_foot_body_contact_mean_depth": 0.03,
                "left_foot_body_contact_max_depth": 0.04,
                "right_foot_body_contacts": 5,
                "right_foot_body_contact_mean_depth": 0.05,
                "right_foot_body_contact_max_depth": 0.06,
                "joint_positions": {"l_leg_kny": 0.5, "r_leg_aky": -0.1},
            }
        ]
    }

    result = compare_atlas_simbicon_pose_window.compare_traces(
        dart6_trace=dart6_trace,
        dart7_trace=dart7_trace,
        mesh_dir=mesh_dir,
        steps=(302,),
    )

    row = result["samples"][0]
    assert row["dart6"]["left_minus_right_mesh_bottom"] == pytest.approx(0.3)
    assert row["dart7"]["left_minus_right_mesh_bottom"] == pytest.approx(0.6)
    assert row["dart7_minus_dart6"]["left_minus_right_mesh_bottom"] == pytest.approx(
        0.3
    )
    assert row["dart6"]["left_foot_body"]["count"] == 0
    assert row["dart7"]["left_foot_body"]["count"] == 3
    assert row["dart7"]["joint_positions"]["l_leg_kny"] == 0.5
    assert row["dart6"]["left_foot_pose"]["local_z_axis_tilt_degrees"] == pytest.approx(
        0.0
    )
    assert row["dart7"]["left_foot_pose"]["local_z_axis_tilt_degrees"] == pytest.approx(
        30.0
    )
    assert row["dart7"]["role_foot_pose"]["stance"] == row["dart7"]["left_foot_pose"]
    assert row["foot_pose_dart7_minus_dart6"]["left_foot_pose"][
        "local_z_axis_tilt_degrees"
    ] == pytest.approx(30.0)
    assert row["foot_pose_dart7_minus_dart6"]["role_foot_pose"]["stance"][
        "local_z_axis_tilt_degrees"
    ] == pytest.approx(30.0)
    assert result["dart7_trace"]["atlas_joint_kp"] == 1000.0
    assert result["dart7_trace"]["atlas_joint_kd"] == 1.0
