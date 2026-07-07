from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import scene_diff


def _scene() -> dict:
    return {
        "schema": "dart.scene/v0",
        "world": {
            "time": 0.0,
            "frame": 0,
            "gravity": [0.0, 0.0, -9.81],
            "body_count": 1,
        },
        "bodies": [
            {
                "id": "body:box",
                "name": "box",
                "shapes": [
                    {
                        "id": "shape:body:box:0",
                        "type": "box",
                        "size": [1.0, 1.0, 1.0],
                    }
                ],
            }
        ],
        "joints": [],
        "flat_index": {
            "world": "world bodies=1 joints=0 time_step=0.001 gravity=[0, 0, -9.81]",
            "body:box": "rigid body box mass=1 position=[0, 0, 0] shapes=1",
            "shape:body:box:0": "box shape on box size=[1, 1, 1]",
        },
    }


def test_scene_diff_passes_with_numeric_tolerance_and_ignored_time() -> None:
    actual = _scene()
    expected = _scene()
    actual["world"]["time"] = 1.0
    actual["world"]["frame"] = 100
    actual["world"]["gravity"][2] = -9.8100000001

    verdict = scene_diff.compare_scene_json(actual, expected, abs_tol=1e-8)

    assert verdict["schema_version"] == "dart.scene_diff/v1"
    assert verdict["pass"] is True
    assert verdict["diffs"] == []
    assert verdict["thresholds_used"]["ignore_world_time"] is True


def test_scene_diff_reports_first_structural_differences() -> None:
    actual = _scene()
    expected = _scene()
    actual["bodies"][0]["shapes"][0]["size"][2] = 2.0
    del actual["flat_index"]["shape:body:box:0"]

    verdict = scene_diff.compare_scene_json(actual, expected, max_diffs=5)

    assert verdict["pass"] is False
    assert {
        ("$.bodies[0].shapes[0].size[2]", "numeric mismatch"),
        ("$.flat_index.shape:body:box:0", "missing key"),
    } <= {(diff["path"], diff["reason"]) for diff in verdict["diffs"]}
    assert verdict["reasons"]


def test_scene_diff_rejects_non_positive_max_diffs() -> None:
    try:
        scene_diff.compare_scene_json(_scene(), _scene(), max_diffs=0)
    except ValueError as exc:
        assert "max_diffs" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_scene_diff_cli_writes_verdict(tmp_path: Path) -> None:
    actual = tmp_path / "actual.json"
    expected = tmp_path / "expected.json"
    out = tmp_path / "verdict.json"
    actual.write_text(json.dumps(_scene()), encoding="utf-8")
    expected.write_text(json.dumps(_scene()), encoding="utf-8")

    assert scene_diff.main([str(actual), str(expected), "--out", str(out)]) == 0
    verdict = json.loads(out.read_text(encoding="utf-8"))

    assert verdict["pass"] is True
