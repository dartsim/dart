from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import image_ab_study


def _manifest() -> dict:
    return {
        "schema_version": "dart.image_ab_study/v1",
        "study_id": "round2-test",
        "rows": [
            {
                "case": "penetration",
                "variant": "single",
                "judge": "j1",
                "expected": "defect",
                "observed": "clean",
            },
            {
                "case": "tunnel",
                "variant": "single",
                "judge": "j1",
                "expected": "defect",
                "observed": "defect",
            },
            {
                "case": "control",
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
            {
                "case": "tunnel",
                "variant": "annotated",
                "judge": "j1",
                "expected": "defect",
                "observed": "defect",
            },
            {
                "case": "control",
                "variant": "annotated",
                "judge": "j1",
                "expected": "clean",
                "observed": "defect",
            },
            {
                "case": "control2",
                "variant": "annotated",
                "judge": "j2",
                "expected": "control",
                "observed": "uncertain",
            },
        ],
    }


def test_image_ab_study_reports_detection_delta_and_false_positives() -> None:
    results = image_ab_study.reduce_study(_manifest(), baseline="single")

    single = results["variants"]["single"]
    annotated = results["variants"]["annotated"]

    assert single["detection_rate"] == 0.5
    assert single["false_positive_rate"] == 0.0
    assert annotated["detection_rate"] == 1.0
    assert annotated["detection_delta_vs_baseline"] == 0.5
    assert annotated["false_positive_rate"] == 0.5
    assert annotated["uncertain"] == 1
    assert annotated["judge_count"] == 2


def test_image_ab_study_markdown_formats_table() -> None:
    results = image_ab_study.reduce_study(_manifest(), baseline="single")
    markdown = image_ab_study.render_markdown(results)

    assert "| `annotated` | 2 | 100.0% | +50.0% | 2 | 50.0% | 1 |" in markdown


def test_image_ab_study_rejects_unknown_expected_label() -> None:
    manifest = _manifest()
    manifest["rows"][0]["expected"] = "typo"

    try:
        image_ab_study.reduce_study(manifest, baseline="single")
    except ValueError as exc:
        assert "expected must be one of" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_image_ab_study_cli_writes_json_and_markdown(tmp_path: Path) -> None:
    manifest = tmp_path / "manifest.json"
    out = tmp_path / "results.json"
    markdown = tmp_path / "results.md"
    manifest.write_text(json.dumps(_manifest()), encoding="utf-8")

    assert (
        image_ab_study.main(
            [str(manifest), "--out", str(out), "--markdown", str(markdown)]
        )
        == 0
    )
    results = json.loads(out.read_text(encoding="utf-8"))

    assert results["schema_version"] == "dart.image_ab_study_results/v1"
    assert markdown.read_text(encoding="utf-8").startswith(
        "# Image A/B Study Results"
    )
