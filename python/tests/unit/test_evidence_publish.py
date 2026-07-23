"""Unit coverage for PR-body visual-verification section generation."""

from __future__ import annotations

import hashlib
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
                "sha256": hashlib.sha256(b"png").hexdigest(),
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
                "sha256": hashlib.sha256(b"mp4").hexdigest(),
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
    assert manifest["operation_succeeded"] is True
    assert manifest["publication_complete"] is False
    assert manifest["selection_pass"] is False
    assert manifest["pass"] is False
    assert "dry-run" in manifest["note"]
    expected_tag = evidence_publish._content_addressed_tag(
        "verification-media", manifest["artifact_set_sha256"]
    )
    assert manifest["release_tag"] == expected_tag
    assert len(manifest["source"]["revision"]) == 40
    assert isinstance(manifest["source"]["dirty"], bool)
    assert len(manifest["source"]["publisher_sha256"]) == 64
    assert len(manifest["source"]["dirty_content_sha256"]) == 64
    text = out.read_text(encoding="utf-8")
    # Images embed inline; videos fall back to plain links.
    assert (
        "![contact markers at rest](https://github.com/dartsim/dart/releases/"
        f"download/{expected_tag}/shot.png)" in text
    )
    assert (
        "[settling clip](https://github.com/dartsim/dart/releases/download/"
        f"{expected_tag}/clip.mp4)" in text
    )
    assert "UPLOAD-PLACEHOLDER" not in text


def test_gh_release_yes_uploads_each_artifact(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    selection = _selection(tmp_path)
    calls: list[list[str]] = []

    class _Completed:
        def __init__(self, returncode: int) -> None:
            self.returncode = returncode
            self.stdout = ""
            self.stderr = ""

    def fake_gh(args: list[str], *, check: bool = True) -> "_Completed":
        calls.append(list(args))
        # First call is `release view` for a tag that does not exist yet.
        if args[:2] == ["release", "view"]:
            return _Completed(1)
        return _Completed(0)

    monkeypatch.setattr(evidence_publish, "_gh", fake_gh)
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
            "--yes",
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
    assert manifest["uploaded"] is True
    assert manifest["publication_complete"] is True
    assert manifest["selection_pass"] is False
    assert manifest["pass"] is False
    assert "note" not in manifest
    verbs = [call[:2] for call in calls]
    assert verbs[0] == ["release", "view"]
    assert verbs[1] == ["release", "create"]
    uploads = [call for call in calls if call[:2] == ["release", "upload"]]
    assert len(uploads) == 2  # one per selected artifact
    # Re-upload is idempotent within a content-addressed release. Changed
    # bytes produce a new tag instead of mutating evidence embedded in a PR.
    assert all("--clobber" in call for call in uploads)
    expected_tag = evidence_publish._content_addressed_tag(
        "verification-media", manifest["artifact_set_sha256"]
    )
    assert all(expected_tag in call for call in calls[:2])
    assert manifest["urls"]["shot.png"] == (
        "https://github.com/dartsim/dart/releases/download/" f"{expected_tag}/shot.png"
    )
    assert manifest["artifacts"][0]["sha256"] == hashlib.sha256(b"png").hexdigest()


def test_upload_uses_verified_staged_bytes_when_source_changes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    selection = _selection(tmp_path)
    uploaded_bytes: list[bytes] = []

    class _Completed:
        def __init__(self, returncode: int) -> None:
            self.returncode = returncode
            self.stdout = ""
            self.stderr = ""

    def fake_gh(args: list[str], *, check: bool = True) -> "_Completed":
        if args[:2] == ["release", "view"]:
            # Mutating the live selection directory after staging must not alter
            # the bytes uploaded under the content-addressed tag.
            (tmp_path / "shot.png").write_bytes(b"changed after staging")
            return _Completed(1)
        if args[:2] == ["release", "upload"]:
            uploaded_bytes.append(Path(args[3]).read_bytes())
        return _Completed(0)

    monkeypatch.setattr(evidence_publish, "_gh", fake_gh)
    code = evidence_publish.main(
        [
            str(selection),
            "--backend",
            "gh-release",
            "--repo",
            "dartsim/dart",
            "--yes",
            "--environment",
            "Linux",
            "--out",
            str(tmp_path / "section.md"),
        ]
    )

    assert code == 0
    assert sorted(uploaded_bytes) == [b"mp4", b"png"]


def test_duplicate_basenames_are_rejected(tmp_path: Path) -> None:
    (tmp_path / "a").mkdir()
    (tmp_path / "b").mkdir()
    (tmp_path / "a" / "shot.png").write_bytes(b"x")
    (tmp_path / "b" / "shot.png").write_bytes(b"y")
    manifest = {
        "schema_version": "dart.evidence_selection/v1",
        "claims": [{"id": "C1", "text": "t", "covered": True}],
        "selected": [
            {
                "path": "a/shot.png",
                "kind": "still",
                "claims": ["C1"],
                "caption": "",
                "observe": "",
                "quality": 0.5,
                "bytes": 1,
                "sha256": hashlib.sha256(b"x").hexdigest(),
                "rationale": "r",
            },
            {
                "path": "b/shot.png",
                "kind": "still",
                "claims": ["C1"],
                "caption": "",
                "observe": "",
                "quality": 0.5,
                "bytes": 1,
                "sha256": hashlib.sha256(b"y").hexdigest(),
                "rationale": "r",
            },
        ],
        "rejected": [],
        "total_bytes": 2,
        "uncovered_claims": [],
        "pass": True,
    }
    selection = tmp_path / "selection.json"
    selection.write_text(json.dumps(manifest), encoding="utf-8")
    code = evidence_publish.main(
        [
            str(selection),
            "--environment",
            "Linux",
            "--out",
            str(tmp_path / "x.md"),
        ]
    )
    assert code == 2


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


def test_changed_artifact_is_rejected_before_render_or_upload(tmp_path: Path) -> None:
    selection = _selection(tmp_path)
    (tmp_path / "shot.png").write_bytes(b"changed")

    code = evidence_publish.main(
        [
            str(selection),
            "--environment",
            "Linux",
            "--out",
            str(tmp_path / "x.md"),
        ]
    )

    assert code == 2
    assert not (tmp_path / "x.md").exists()


def test_changed_selection_uses_a_new_release_tag(tmp_path: Path) -> None:
    first = _selection(tmp_path)
    first_manifest = evidence_publish._load_selection(first)
    first_tag = evidence_publish._content_addressed_tag(
        "verification-media",
        evidence_publish._artifact_set_digest(first_manifest),
    )

    (tmp_path / "shot.png").write_bytes(b"new png")
    data = json.loads(first.read_text(encoding="utf-8"))
    data["selected"][0]["bytes"] = len(b"new png")
    data["selected"][0]["sha256"] = hashlib.sha256(b"new png").hexdigest()
    first.write_text(json.dumps(data), encoding="utf-8")
    second_manifest = evidence_publish._load_selection(first)
    second_tag = evidence_publish._content_addressed_tag(
        "verification-media",
        evidence_publish._artifact_set_digest(second_manifest),
    )

    assert second_tag != first_tag
