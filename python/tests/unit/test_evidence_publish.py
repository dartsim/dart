"""Unit coverage for PR-body visual-verification section generation."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import evidence_publish


def _selection(tmp_path: Path) -> Path:
    (tmp_path / "shot.png").write_bytes(b"png")
    (tmp_path / "clip.mp4").write_bytes(b"mp4")
    manifest = {
        "schema_version": "dart.evidence_selection/v1",
        "claims": [
            {"id": "C1", "text": "no penetration at rest", "covered": True},
            {"id": "C2", "text": "stack stays upright", "covered": False},
        ],
        "selected": [
            {
                "path": "shot.png",
                "kind": "still",
                "claims": ["C1"],
                "caption": "contact markers at rest",
                "observe": "yellow cross at interface",
                "quality": 0.9,
                "bytes": 3,
                "sha256": "0" * 64,
                "rationale": "covers claim(s) C1; quality=0.900",
            },
            {
                "path": "clip.mp4",
                "kind": "video",
                "claims": ["C1"],
                "caption": "settling clip",
                "observe": "",
                "quality": 0.7,
                "bytes": 3,
                "sha256": "1" * 64,
                "rationale": "covers claim(s) C1; quality=0.700",
            },
        ],
        "rejected": [],
        "total_bytes": 6,
        "uncovered_claims": ["C2"],
        "pass": False,
    }
    path = tmp_path / "selection.json"
    path.write_text(json.dumps(manifest), encoding="utf-8")
    return path


def test_manual_backend_emits_placeholders_and_context(tmp_path: Path) -> None:
    selection = _selection(tmp_path)
    out = tmp_path / "section.md"
    code = evidence_publish.main(
        [
            str(selection),
            "--environment",
            "Linux x86_64, llvmpipe",
            "--configuration",
            "box_on_ground, 200 steps",
            "--limitation",
            "static frame only",
            "--not-proven",
            "penetration depth tolerance",
            "--reproduce",
            "pixi run agent-capture -- --scene box_on_ground --out demo",
            "--out",
            str(out),
        ]
    )
    assert code == 0
    text = out.read_text(encoding="utf-8")
    assert "## Visual verification" in text
    assert "UPLOAD-PLACEHOLDER" in text
    assert "**Environment**: Linux x86_64, llvmpipe" in text
    assert "What to look for: yellow cross at interface" in text
    assert "Why this artifact" in text
    assert "✘ (no evidence)" in text  # uncovered claim surfaces to the reviewer
    assert "static frame only" in text
    assert "penetration depth tolerance" in text
    assert "pixi run agent-capture" in text


def test_gh_release_dry_run_predicts_urls_without_uploading(tmp_path: Path) -> None:
    selection = _selection(tmp_path)
    out = tmp_path / "section.md"
    manifest_out = tmp_path / "publication.json"
    code = evidence_publish.main(
        [
            str(selection),
            "--backend",
            "gh-release",
            "--repo",
            "dartsim/dart",
            "--tag",
            "verification-media",
            "--environment",
            "Linux",
            "--out",
            str(out),
            "--manifest-out",
            str(manifest_out),
        ]
    )
    assert code == 0
    manifest = json.loads(manifest_out.read_text(encoding="utf-8"))
    assert manifest["uploaded"] is False
    assert "dry-run" in manifest["note"]
    text = out.read_text(encoding="utf-8")
    # Images embed inline; videos fall back to plain links.
    assert (
        "![contact markers at rest](https://github.com/dartsim/dart/releases/"
        "download/verification-media/shot.png)" in text
    )
    assert (
        "[settling clip](https://github.com/dartsim/dart/releases/download/"
        "verification-media/clip.mp4)" in text
    )
    assert "UPLOAD-PLACEHOLDER" not in text


def test_gh_release_requires_repo(tmp_path: Path) -> None:
    selection = _selection(tmp_path)
    code = evidence_publish.main(
        [
            str(selection),
            "--backend",
            "gh-release",
            "--environment",
            "Linux",
            "--out",
            str(tmp_path / "x.md"),
        ]
    )
    assert code == 2
