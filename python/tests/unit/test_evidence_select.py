"""Unit coverage for claim-driven evidence selection."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import evidence_select


def _candidates(tmp_path: Path, artifacts: list[dict]) -> dict:
    for artifact in artifacts:
        target = tmp_path / artifact["path"]
        target.parent.mkdir(parents=True, exist_ok=True)
        if not target.exists():
            target.write_bytes(b"x" * artifact.pop("_bytes", 100))
        artifact.pop("_bytes", None)
    return {
        "schema_version": "dart.evidence_candidates/v1",
        "claims": [
            {"id": "C1", "text": "no penetration at rest"},
            {"id": "C2", "text": "stack stays upright"},
        ],
        "artifacts": artifacts,
    }


def test_selection_covers_claims_with_rationale(tmp_path: Path) -> None:
    manifest = _candidates(
        tmp_path,
        [
            {"path": "a.png", "kind": "still", "claims": ["C1"], "quality": 0.9},
            {"path": "b.png", "kind": "still", "claims": ["C2"], "quality": 0.8},
        ],
    )
    result = evidence_select.select_evidence(manifest, tmp_path)
    assert result["pass"] is True
    assert [entry["path"] for entry in result["selected"]] == ["a.png", "b.png"]
    assert all("covers claim(s)" in entry["rationale"] for entry in result["selected"])
    assert all(len(entry["sha256"]) == 64 for entry in result["selected"])


def test_redundant_and_no_new_coverage_artifacts_are_rejected(tmp_path: Path) -> None:
    manifest = _candidates(
        tmp_path,
        [
            {
                "path": "best.png",
                "kind": "still",
                "claims": ["C1", "C2"],
                "quality": 0.95,
                "azimuth": 0.4,
            },
            {
                "path": "similar.png",
                "kind": "still",
                "claims": ["C1", "C2"],
                "quality": 0.90,
                "azimuth": 0.45,
            },
            {
                "path": "other_angle.png",
                "kind": "still",
                "claims": ["C1"],
                "quality": 0.85,
                "azimuth": 3.5,
            },
        ],
    )
    result = evidence_select.select_evidence(manifest, tmp_path)
    assert [entry["path"] for entry in result["selected"]] == ["best.png"]
    reasons = {entry["path"]: entry["reason"] for entry in result["rejected"]}
    assert "redundant" in reasons["similar.png"]
    assert "no claim coverage" in reasons["other_angle.png"]


def test_coverage_beats_quality_under_tight_budget(tmp_path: Path) -> None:
    # With only one artifact allowed, a covering set exists (cover.png proves
    # both claims), so the selector must prefer it over the higher-quality
    # single-claim still instead of failing to cover C2.
    manifest = _candidates(
        tmp_path,
        [
            {"path": "high.png", "kind": "still", "claims": ["C1"], "quality": 0.99},
            {
                "path": "cover.png",
                "kind": "still",
                "claims": ["C1", "C2"],
                "quality": 0.90,
            },
        ],
    )
    result = evidence_select.select_evidence(manifest, tmp_path, max_artifacts=1)
    assert result["pass"] is True
    assert result["uncovered_claims"] == []
    assert [entry["path"] for entry in result["selected"]] == ["cover.png"]


def test_budgets_are_enforced(tmp_path: Path) -> None:
    manifest = _candidates(
        tmp_path,
        [
            {"path": "a.png", "kind": "still", "claims": ["C1"], "quality": 0.9},
            {
                "path": "big.mp4",
                "kind": "video",
                "claims": ["C2"],
                "quality": 0.8,
                "_bytes": 2000,
            },
        ],
    )
    result = evidence_select.select_evidence(
        manifest, tmp_path, max_total_bytes=500
    )
    assert result["pass"] is False
    assert result["uncovered_claims"] == ["C2"]
    assert any("size budget" in entry["reason"] for entry in result["rejected"])


def test_artifact_without_claims_is_invalid(tmp_path: Path) -> None:
    manifest = _candidates(
        tmp_path,
        [{"path": "a.png", "kind": "still", "claims": [], "quality": 0.9}],
    )
    with pytest.raises(ValueError, match="supports no claims"):
        evidence_select.select_evidence(manifest, tmp_path)


def test_cli_roundtrip(tmp_path: Path) -> None:
    manifest = _candidates(
        tmp_path,
        [
            {"path": "a.png", "kind": "still", "claims": ["C1", "C2"], "quality": 0.9}
        ],
    )
    candidates_path = tmp_path / "candidates.json"
    candidates_path.write_text(json.dumps(manifest), encoding="utf-8")
    out_path = tmp_path / "selection.json"
    code = evidence_select.main([str(candidates_path), "--out", str(out_path)])
    assert code == 0
    saved = json.loads(out_path.read_text(encoding="utf-8"))
    assert saved["schema_version"] == "dart.evidence_selection/v1"
    assert saved["pass"] is True
