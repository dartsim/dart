from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import image_ab_round2_packet
from _image_tools import read_image, write_png


def test_judge_form_hides_case_variant_and_expected_labels() -> None:
    judge_form = {
        "schema_version": "dart.image_ab_round2_observations/v1",
        "study_id": "round2-test",
        "rows": [
            {
                "artifact_id": "r2_abc",
                "image": "images/r2_abc.png",
                "question": "Does this show a defect?",
                "choices": ["defect", "clean", "uncertain"],
            }
        ],
    }

    markdown = image_ab_round2_packet.render_judge_form_markdown(judge_form)

    assert "r2_abc" in markdown
    assert "images/r2_abc.png" in markdown
    assert "ground_penetration" not in markdown
    assert "multi_view" not in markdown
    assert "expected" not in markdown.lower()


def test_annotate_still_writes_overlay_pixels(tmp_path: Path) -> None:
    source = tmp_path / "source.png"
    target = tmp_path / "annotated.png"
    write_png(source, 96, 72, bytes((30, 30, 30)) * (96 * 72))

    image_ab_round2_packet.annotate_still(
        source,
        target,
        image_ab_round2_packet.CASES[0],
    )

    annotated = read_image(target)
    assert annotated.width == 96
    assert annotated.height == 72
    assert annotated.pixels != bytes((30, 30, 30)) * (96 * 72)
    assert b"\xff\xe6\x00" in annotated.pixels


def test_merge_observations_builds_reducer_manifest() -> None:
    answer_key = {
        "schema_version": "dart.image_ab_round2_packet/v1",
        "study_id": "round2-test",
        "rows": [
            {
                "artifact_id": "r2_a",
                "case": "clean",
                "variant": "single",
                "expected": "clean",
                "image": "images/r2_a.png",
            },
            {
                "artifact_id": "r2_b",
                "case": "penetration",
                "variant": "annotated",
                "expected": "defect",
                "image": "images/r2_b.png",
            },
        ],
    }
    observations = {
        "schema_version": "dart.image_ab_round2_observations/v1",
        "study_id": "round2-test",
        "rows": [
            {"artifact_id": "r2_a", "judge": "j1", "observed": "clean"},
            {"artifact_id": "r2_b", "judge": "j1", "observed": "defect"},
        ],
    }

    manifest = image_ab_round2_packet.merge_observations(answer_key, observations)

    assert manifest == {
        "schema_version": "dart.image_ab_study/v1",
        "study_id": "round2-test",
        "rows": [
            {
                "case": "clean",
                "variant": "single",
                "judge": "j1",
                "expected": "clean",
                "observed": "clean",
            },
            {
                "case": "penetration",
                "variant": "annotated",
                "judge": "j1",
                "expected": "defect",
                "observed": "defect",
            },
        ],
    }


def test_score_observations_writes_manifest_and_results(tmp_path: Path) -> None:
    answer_key = {
        "schema_version": "dart.image_ab_round2_packet/v1",
        "study_id": "round2-test",
        "rows": [
            {
                "artifact_id": "r2_a",
                "case": "penetration",
                "variant": "single",
                "expected": "defect",
                "image": "images/r2_a.png",
            },
            {
                "artifact_id": "r2_b",
                "case": "penetration",
                "variant": "annotated",
                "expected": "defect",
                "image": "images/r2_b.png",
            },
        ],
    }
    observations = {
        "schema_version": "dart.image_ab_round2_observations/v1",
        "study_id": "round2-test",
        "rows": [
            {"artifact_id": "r2_a", "judge": "j1", "observed": "clean"},
            {"artifact_id": "r2_b", "judge": "j1", "observed": "defect"},
        ],
    }
    answer_key_path = tmp_path / "answer_key.json"
    observations_path = tmp_path / "observations.json"
    manifest_path = tmp_path / "manifest.json"
    results_path = tmp_path / "results.json"
    markdown_path = tmp_path / "results.md"
    answer_key_path.write_text(json.dumps(answer_key), encoding="utf-8")
    observations_path.write_text(json.dumps(observations), encoding="utf-8")

    results = image_ab_round2_packet.score_observations(
        answer_key_path=answer_key_path,
        observations_path=observations_path,
        out_path=manifest_path,
        results_path=results_path,
        markdown_path=markdown_path,
        baseline="single",
    )

    assert results["variants"]["single"]["detection_rate"] == 0.0
    assert results["variants"]["annotated"]["detection_rate"] == 1.0
    assert json.loads(manifest_path.read_text(encoding="utf-8"))[
        "schema_version"
    ] == "dart.image_ab_study/v1"
    assert json.loads(results_path.read_text(encoding="utf-8")) == results
    assert "| `annotated` |" in markdown_path.read_text(encoding="utf-8")


def test_parse_size_rejects_invalid_shape() -> None:
    try:
        image_ab_round2_packet._parse_size("640-by-480")
    except ValueError as exc:
        assert "WIDTHxHEIGHT" in str(exc)
    else:
        raise AssertionError("expected ValueError")
