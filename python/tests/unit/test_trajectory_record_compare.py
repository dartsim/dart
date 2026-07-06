"""Unit coverage for DART trajectory recorder and comparator tooling."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import trajectory_compare
import trajectory_record


def _perturb_first_box_pos_z(tsv: str, delta: float) -> str:
    lines = tsv.splitlines()
    columns = []
    for line in lines:
        if line.startswith("# columns:"):
            columns = line.split(":", 1)[1].strip().split()
            break
    assert columns
    body_index = columns.index("body")
    pos_z_index = columns.index("pos_z")
    for index, line in enumerate(lines):
        if line.startswith("#") or not line.strip():
            continue
        parts = line.split()
        if parts[body_index] != "box":
            continue
        parts[pos_z_index] = f"{float(parts[pos_z_index]) + delta:.17e}"
        lines[index] = "\t".join(parts)
        return "\n".join(lines) + "\n"
    raise AssertionError("box trajectory row not found")


def test_short_trajectory_records_bit_exact_on_repeat() -> None:
    first = trajectory_record.record_trajectory_for_scene(
        "box_on_ground", steps=8, bodies=("box",)
    )
    second = trajectory_record.record_trajectory_for_scene(
        "box_on_ground", steps=8, bodies=("box",)
    )

    assert first == second
    assert first.startswith("# dart.trajectory/v0 scene=box_on_ground")
    assert "# columns: frame time body pos_x pos_y pos_z" in first
    verdict = trajectory_compare.compare_trajectory_texts(first, second)
    assert verdict["pass"] is True
    assert verdict["matched"] is True
    assert verdict["first_divergence"] is None


def test_compare_reports_seeded_first_divergence() -> None:
    baseline = trajectory_record.record_trajectory_for_scene(
        "box_on_ground", steps=6, bodies=("box",)
    )
    repeat = trajectory_record.record_trajectory_for_scene(
        "box_on_ground", steps=6, bodies=("box",)
    )
    perturbed = _perturb_first_box_pos_z(baseline, 1.0e-3)

    exact = trajectory_compare.compare_trajectory_texts(baseline, perturbed)
    assert exact["pass"] is False
    assert exact["first_divergence"]["frame"] == 1
    assert exact["first_divergence"]["body"] == "box"
    assert exact["first_divergence"]["column"] == "pos_z"
    assert exact["first_divergence"]["a"] != exact["first_divergence"]["b"]

    tolerant = trajectory_compare.compare_trajectory_texts(
        baseline,
        perturbed,
        mode="tolerance",
        calibration_texts=[baseline, repeat],
        calibration_factor=2.0,
    )
    assert tolerant["pass"] is False
    assert tolerant["first_divergence"]["frame"] == 1
    assert tolerant["first_divergence"]["body"] == "box"
    assert tolerant["first_divergence"]["column"] == "pos_z"
    assert tolerant["max_abs_dev"]["pos_z"] >= 1.0e-3
    assert "same-scene DART trajectory calibration runs" in (
        tolerant["thresholds_used"]["tolerance_policy"]
    )


def test_contact_trace_reports_position_normal_and_depth() -> None:
    jsonl = trajectory_record.record_contact_events_for_scene(
        "two_body_contact", steps=6
    )
    records = [json.loads(line) for line in jsonl.splitlines() if line.strip()]
    assert records

    first_with_contact = next(record for record in records if record["contacts"])
    contact = first_with_contact["contacts"][0]
    assert first_with_contact["schema_version"] == "dart.contact_events/v0"
    assert first_with_contact["frame"] >= 1
    assert first_with_contact["count"] >= 1
    assert {contact["body_a"], contact["body_b"]} == {"box", "ground"}
    assert len(contact["position"]) == 3
    assert len(contact["normal"]) == 3
    assert all(math.isfinite(value) for value in contact["position"])
    # The contact normal is a unit vector; a wrong-but-finite payload
    # (e.g. [0,0,0]) would pass a bare finiteness check but fails this.
    normal_length = math.sqrt(sum(value * value for value in contact["normal"]))
    assert abs(normal_length - 1.0) < 1.0e-6
    # Small penetration for a resting box on the ground (the box is 0.1 m tall);
    # a bogus depth like -5.0 is rejected.
    assert abs(contact["depth"]) < 0.05
    # The contact lies near the box/ground interface (z ~ 0), not far away.
    assert abs(contact["position"][2]) < 0.2


def test_combined_trajectory_and_contact_recording_shares_runner() -> None:
    runner = trajectory_record.resolve_world_runner(scene="two_body_contact")

    trajectory_text, contact_text = trajectory_record.record_trajectory_and_contact_events(
        runner, steps=6, bodies=("box",)
    )

    trajectory_frames = {
        int(line.split()[0])
        for line in trajectory_text.splitlines()
        if line.strip() and not line.startswith("#")
    }
    contact_frames = {
        json.loads(line)["frame"] for line in contact_text.splitlines() if line.strip()
    }
    assert trajectory_frames
    assert contact_frames
    assert contact_frames <= trajectory_frames
