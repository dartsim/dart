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


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _selection(tmp_path: Path, *, passing: bool = True) -> Path:
    shot = tmp_path / "shot.png"
    clip = tmp_path / "clip.mp4"
    shot.write_bytes(b"png")
    clip.write_bytes(b"mp4")
    manifest = {
        "schema_version": "dart.evidence_selection/v1",
        "claims": [
            {"id": "C1", "text": "no penetration at rest", "covered": True},
            {
                "id": "C2",
                "text": "stack stays upright",
                "covered": passing,
            },
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
                "sha256": _sha256(shot),
                "rationale": "covers claim(s) C1; quality=0.900",
            },
            {
                "path": "clip.mp4",
                "kind": "video",
                "claims": ["C2"] if passing else ["C1"],
                "caption": "settling clip",
                "observe": "",
                "quality": 0.7,
                "bytes": 3,
                "sha256": _sha256(clip),
                "rationale": (
                    "covers claim(s) C2; quality=0.700"
                    if passing
                    else "covers claim(s) C1; quality=0.700"
                ),
            },
        ],
        "rejected": [],
        "total_bytes": 6,
        "uncovered_claims": [] if passing else ["C2"],
        "pass": passing,
    }
    path = tmp_path / "selection.json"
    path.write_text(json.dumps(manifest), encoding="utf-8")
    return path


def _semantic_args(verdict: str = "pass") -> list[str]:
    return [
        "--not-proven",
        "penetration depth tolerance",
        "--text-oracle",
        "settled pose and contact-count assertions passed",
        "--visible-observation",
        "the box is upright and the contact marker is at the interface",
        "--reconciliation",
        "the visible settled pose agrees with the measured oracle",
        "--semantic-verdict",
        verdict,
    ]


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
            "--reproduce",
            "pixi run agent-capture -- --scene box_on_ground --out demo",
            *_semantic_args(),
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
    assert "stack stays upright ✔" in text
    assert "### Semantic review" in text
    assert "settled pose and contact-count assertions passed" in text
    assert "the visible settled pose agrees with the measured oracle" in text
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
            *_semantic_args(),
            "--out",
            str(out),
            "--manifest-out",
            str(manifest_out),
        ]
    )
    assert code == 0
    manifest = json.loads(manifest_out.read_text(encoding="utf-8"))
    assert manifest["schema_version"] == "dart.evidence_publication/v2"
    assert manifest["pass"] is True
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
            *_semantic_args(),
            "--out",
            str(out),
            "--manifest-out",
            str(manifest_out),
        ]
    )
    assert code == 0
    manifest = json.loads(manifest_out.read_text(encoding="utf-8"))
    assert manifest["uploaded"] is True
    assert manifest["pass"] is True
    assert "note" not in manifest
    verbs = [call[:2] for call in calls]
    assert verbs[0] == ["release", "view"]
    assert verbs[1] == ["release", "create"]
    uploads = [call for call in calls if call[:2] == ["release", "upload"]]
    assert len(uploads) == 2  # one per selected artifact
    # Uploads clobber so regenerated evidence keeps a stable download URL.
    assert all("--clobber" in call for call in uploads)
    assert manifest["urls"]["shot.png"] == (
        "https://github.com/dartsim/dart/releases/download/"
        "verification-media/shot.png"
    )


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
                "sha256": "0" * 64,
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
                "sha256": "1" * 64,
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
            *_semantic_args(),
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
            *_semantic_args(),
            "--out",
            str(tmp_path / "x.md"),
        ]
    )
    assert code == 2


def test_uncovered_selection_is_rendered_but_fails_closed(tmp_path: Path) -> None:
    selection = _selection(tmp_path, passing=False)
    out = tmp_path / "section.md"
    manifest_out = tmp_path / "publication.json"

    code = evidence_publish.main(
        [
            str(selection),
            "--environment",
            "Linux",
            *_semantic_args(),
            "--out",
            str(out),
            "--manifest-out",
            str(manifest_out),
        ]
    )

    assert code == 1
    assert "✘ (no evidence)" in out.read_text(encoding="utf-8")
    manifest = json.loads(manifest_out.read_text(encoding="utf-8"))
    assert manifest["pass"] is False
    assert "uncovered claims: C2" in manifest["note"]


def test_uncertain_semantic_review_fails_closed(tmp_path: Path) -> None:
    selection = _selection(tmp_path)
    manifest_out = tmp_path / "publication.json"

    code = evidence_publish.main(
        [
            str(selection),
            "--environment",
            "Linux",
            *_semantic_args("uncertain"),
            "--out",
            str(tmp_path / "section.md"),
            "--manifest-out",
            str(manifest_out),
        ]
    )

    assert code == 1
    manifest = json.loads(manifest_out.read_text(encoding="utf-8"))
    assert manifest["pass"] is False
    assert manifest["semantic_review"]["verdict"] == "uncertain"


def test_inconsistent_passing_selection_fails_closed(tmp_path: Path) -> None:
    selection = _selection(tmp_path, passing=False)
    manifest = json.loads(selection.read_text(encoding="utf-8"))
    manifest["pass"] = True
    selection.write_text(json.dumps(manifest), encoding="utf-8")

    code = evidence_publish.main(
        [
            str(selection),
            "--environment",
            "Linux",
            *_semantic_args(),
            "--out",
            str(tmp_path / "section.md"),
        ]
    )

    assert code == 2


def test_stored_coverage_without_selected_artifact_fails_closed(
    tmp_path: Path,
) -> None:
    selection = _selection(tmp_path)
    manifest = json.loads(selection.read_text(encoding="utf-8"))
    manifest["selected"][1]["claims"] = ["C1"]
    selection.write_text(json.dumps(manifest), encoding="utf-8")

    code = evidence_publish.main(
        [
            str(selection),
            "--environment",
            "Linux",
            *_semantic_args(),
            "--out",
            str(tmp_path / "section.md"),
        ]
    )

    assert code == 2


@pytest.mark.parametrize(
    ("mutation", "expected"),
    [
        ("hash", "changed content"),
        ("size", "changed size"),
        ("missing", "missing or not a regular file"),
        ("directory", "missing or not a regular file"),
        ("unknown-claim", "unknown claims"),
    ],
)
def test_gh_release_yes_revalidates_selected_artifacts_before_mutation(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    mutation: str,
    expected: str,
) -> None:
    selection = _selection(tmp_path)
    manifest = json.loads(selection.read_text(encoding="utf-8"))
    if mutation == "hash":
        (tmp_path / "shot.png").write_bytes(b"PNG")
    elif mutation == "size":
        (tmp_path / "shot.png").write_bytes(b"changed")
    elif mutation == "missing":
        (tmp_path / "shot.png").unlink()
    elif mutation == "directory":
        (tmp_path / "shot.png").unlink()
        (tmp_path / "shot.png").mkdir()
    else:
        manifest["selected"][0]["claims"] = ["C999"]
        selection.write_text(json.dumps(manifest), encoding="utf-8")

    calls = []

    def record_gh(*args, **kwargs):
        calls.append((args, kwargs))
        raise AssertionError("invalid evidence must not mutate GitHub")

    monkeypatch.setattr(evidence_publish, "_gh", record_gh)
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
            *_semantic_args(),
            "--out",
            str(tmp_path / "section.md"),
        ]
    )

    assert code == 2
    assert calls == []
    assert expected in capsys.readouterr().err


def test_gh_release_yes_does_not_upload_failing_evidence(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    selection = _selection(tmp_path, passing=False)
    calls = []

    def record_gh(*args, **kwargs):
        calls.append((args, kwargs))
        raise AssertionError("invalid evidence must not mutate GitHub")

    monkeypatch.setattr(evidence_publish, "_gh", record_gh)
    manifest_out = tmp_path / "publication.json"

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
            *_semantic_args(),
            "--out",
            str(tmp_path / "section.md"),
            "--manifest-out",
            str(manifest_out),
        ]
    )

    assert code == 1
    assert calls == []
    manifest = json.loads(manifest_out.read_text(encoding="utf-8"))
    assert manifest["uploaded"] is False
    assert manifest["urls"] == {}


def test_required_semantic_field_rejects_empty_value(tmp_path: Path) -> None:
    selection = _selection(tmp_path)
    args = [
        str(selection),
        "--environment",
        "Linux",
        *_semantic_args(),
        "--out",
        str(tmp_path / "section.md"),
    ]
    args[args.index("--visible-observation") + 1] = ""

    with pytest.raises(SystemExit) as caught:
        evidence_publish.main(args)

    assert caught.value.code == 2
