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


def _release_asset_name(path: Path) -> str:
    return f"sha256-{_sha256(path)}{path.suffix.lower()}"


class _Completed:
    def __init__(
        self, returncode: int = 0, *, stdout: str = "", stderr: str = ""
    ) -> None:
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


def _semantic_args(verdict: str = "pass") -> list[str]:
    return [
        "--text-oracle",
        "metrics report zero invalid contacts",
        "--visible-observation",
        "the stack is upright and contact markers align with interfaces",
        "--reconciliation",
        "the visible layout agrees with the measured contact state",
        "--semantic-verdict",
        verdict,
        "--not-proven",
        "long-horizon stability",
    ]


def _selection(tmp_path: Path, passing: bool = True) -> Path:
    shot = tmp_path / "shot.png"
    clip = tmp_path / "clip.mp4"
    shot.write_bytes(b"png")
    clip.write_bytes(b"mp4")
    # When passing, the clip covers C2 so every claim is covered; when not,
    # C2 stays uncovered to exercise the failing-publication path.
    manifest = {
        "schema_version": "dart.evidence_selection/v1",
        "claims": [
            {"id": "C1", "text": "no penetration at rest", "covered": True},
            {"id": "C2", "text": "stack stays upright", "covered": passing},
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
    assert "static frame only" in text
    assert "penetration depth tolerance" in text
    assert "pixi run agent-capture" in text
    assert "### Semantic review" in text
    assert "metrics report zero invalid contacts" in text
    assert "the stack is upright" in text
    assert "**Verdict**: pass" in text


def test_non_passing_selection_fails_publication(tmp_path: Path) -> None:
    # The section still renders honestly (uncovered claims surface to the
    # reviewer as gaps), but the publication manifest and exit code must not
    # report success, so automation cannot treat incomplete evidence as ready.
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
    manifest = json.loads(manifest_out.read_text(encoding="utf-8"))
    assert manifest["pass"] is False
    assert "uncovered claims: C2" in manifest["note"]
    text = out.read_text(encoding="utf-8")
    assert "✘ (no evidence)" in text  # uncovered claim surfaces to the reviewer


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
    assert manifest["schema_version"] == "dart.evidence_publication/v3"
    assert manifest["uploaded"] is False
    assert "dry-run" in manifest["note"]
    shot_name = _release_asset_name(tmp_path / "shot.png")
    clip_name = _release_asset_name(tmp_path / "clip.mp4")
    assert manifest["artifacts"][0] == {
        "path": "shot.png",
        "bytes": 3,
        "sha256": _sha256(tmp_path / "shot.png"),
        "release_asset": shot_name,
        "url": (
            "https://github.com/dartsim/dart/releases/download/"
            f"verification-media/{shot_name}"
        ),
    }
    text = out.read_text(encoding="utf-8")
    # Images embed inline; videos fall back to plain links.
    assert (
        "![contact markers at rest](https://github.com/dartsim/dart/releases/"
        f"download/verification-media/{shot_name})" in text
    )
    assert (
        "[settling clip](https://github.com/dartsim/dart/releases/download/"
        f"verification-media/{clip_name})" in text
    )
    assert "UPLOAD-PLACEHOLDER" not in text


def test_gh_release_yes_uploads_each_artifact(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    selection = _selection(tmp_path)
    calls: list[list[str]] = []

    def fake_gh(args: list[str], *, check: bool = True) -> "_Completed":
        calls.append(list(args))
        # First call is `release view` for a tag that does not exist yet.
        if args[:2] == ["release", "view"]:
            return _Completed(1, stderr="release not found")
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
    assert "note" not in manifest
    verbs = [call[:2] for call in calls]
    assert verbs[0] == ["release", "view"]
    assert verbs[1] == ["release", "create"]
    uploads = [call for call in calls if call[:2] == ["release", "upload"]]
    assert len(uploads) == 1
    assert "--clobber" not in uploads[0]
    uploaded_names = [
        Path(argument).name for argument in uploads[0][3 : uploads[0].index("--repo")]
    ]
    assert uploaded_names == [
        _release_asset_name(tmp_path / "shot.png"),
        _release_asset_name(tmp_path / "clip.mp4"),
    ]
    shot_name = _release_asset_name(tmp_path / "shot.png")
    assert manifest["urls"]["shot.png"] == (
        "https://github.com/dartsim/dart/releases/download/"
        f"verification-media/{shot_name}"
    )


def test_content_addressing_isolates_same_basename_across_publications(
    tmp_path: Path,
) -> None:
    urls: list[str] = []
    for index, content in enumerate((b"png", b"PNG")):
        root = tmp_path / str(index)
        root.mkdir()
        selection = _selection(root)
        shot = root / "shot.png"
        shot.write_bytes(content)
        selected = json.loads(selection.read_text(encoding="utf-8"))
        selected["selected"][0]["sha256"] = _sha256(shot)
        selection.write_text(json.dumps(selected), encoding="utf-8")
        manifest_out = root / "publication.json"

        code = evidence_publish.main(
            [
                str(selection),
                "--backend",
                "gh-release",
                "--repo",
                "dartsim/dart",
                "--environment",
                "Linux",
                *_semantic_args(),
                "--out",
                str(root / "section.md"),
                "--manifest-out",
                str(manifest_out),
            ]
        )

        assert code == 0
        manifest = json.loads(manifest_out.read_text(encoding="utf-8"))
        urls.append(manifest["urls"]["shot.png"])
        assert manifest["artifacts"][0]["sha256"] == _sha256(shot)

    assert urls[0] != urls[1]
    assert urls[0].endswith(f"/{_release_asset_name(tmp_path / '0' / 'shot.png')}")
    assert urls[1].endswith(f"/{_release_asset_name(tmp_path / '1' / 'shot.png')}")


def test_partial_content_addressed_upload_resumes_without_clobber(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    selection = _selection(tmp_path)
    shot = tmp_path / "shot.png"
    shot_name = _release_asset_name(shot)
    calls: list[list[str]] = []
    release = {
        "isImmutable": False,
        "assets": [
            {
                "name": shot_name,
                "size": shot.stat().st_size,
                "digest": f"sha256:{_sha256(shot)}",
                "state": "uploaded",
            }
        ],
    }

    def fake_gh(args: list[str], *, check: bool = True) -> "_Completed":
        calls.append(list(args))
        if args[:2] == ["release", "view"]:
            return _Completed(stdout=json.dumps(release))
        return _Completed()

    monkeypatch.setattr(evidence_publish, "_gh", fake_gh)
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

    assert code == 0
    assert not any(call[:2] == ["release", "create"] for call in calls)
    uploads = [call for call in calls if call[:2] == ["release", "upload"]]
    assert len(uploads) == 1
    assert "--clobber" not in uploads[0]
    uploaded_names = [
        Path(argument).name for argument in uploads[0][3 : uploads[0].index("--repo")]
    ]
    assert uploaded_names == [_release_asset_name(tmp_path / "clip.mp4")]
    manifest = json.loads(manifest_out.read_text(encoding="utf-8"))
    assert manifest["urls"]["shot.png"].endswith(f"/{shot_name}")


def test_existing_content_addressed_asset_mismatch_fails_before_mutation(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    selection = _selection(tmp_path)
    shot = tmp_path / "shot.png"
    calls: list[list[str]] = []
    release = {
        "isImmutable": False,
        "assets": [
            {
                "name": _release_asset_name(shot),
                "size": shot.stat().st_size,
                "digest": f"sha256:{'0' * 64}",
                "state": "uploaded",
            }
        ],
    }

    def fake_gh(args: list[str], *, check: bool = True) -> "_Completed":
        calls.append(list(args))
        return _Completed(stdout=json.dumps(release))

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
            *_semantic_args(),
            "--out",
            str(tmp_path / "section.md"),
        ]
    )

    assert code == 2
    assert len(calls) == 1
    assert calls[0][:2] == ["release", "view"]


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("digest", None),
        ("digest", ""),
        ("state", None),
        ("state", "new"),
    ],
)
def test_existing_asset_requires_verifiable_digest_and_uploaded_state(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    field: str,
    value: object,
) -> None:
    selection = _selection(tmp_path)
    shot = tmp_path / "shot.png"
    calls: list[list[str]] = []
    asset = {
        "name": _release_asset_name(shot),
        "size": shot.stat().st_size,
        "digest": f"sha256:{_sha256(shot)}",
        "state": "uploaded",
    }
    asset[field] = value
    release = {"isImmutable": False, "assets": [asset]}

    def fake_gh(args: list[str], *, check: bool = True) -> "_Completed":
        calls.append(list(args))
        return _Completed(stdout=json.dumps(release))

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
            *_semantic_args(),
            "--out",
            str(tmp_path / "section.md"),
        ]
    )

    assert code == 2
    assert len(calls) == 1
    assert calls[0][:2] == ["release", "view"]


def test_release_lookup_failure_is_not_treated_as_missing_release(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    selection = _selection(tmp_path)
    calls: list[list[str]] = []

    def fake_gh(args: list[str], *, check: bool = True) -> "_Completed":
        calls.append(list(args))
        return _Completed(1, stderr="authentication failed")

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
            *_semantic_args(),
            "--out",
            str(tmp_path / "section.md"),
        ]
    )

    assert code == 2
    assert len(calls) == 1
    assert calls[0][:2] == ["release", "view"]


def test_duplicate_basenames_are_rejected(tmp_path: Path) -> None:
    (tmp_path / "a").mkdir()
    (tmp_path / "b").mkdir()
    a_shot = tmp_path / "a" / "shot.png"
    b_shot = tmp_path / "b" / "shot.png"
    a_shot.write_bytes(b"x")
    b_shot.write_bytes(b"y")
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
                "sha256": _sha256(a_shot),
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
                "sha256": _sha256(b_shot),
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


def test_uncertain_semantic_review_fails_publication(tmp_path: Path) -> None:
    selection = _selection(tmp_path)
    out = tmp_path / "section.md"
    manifest_out = tmp_path / "publication.json"
    code = evidence_publish.main(
        [
            str(selection),
            "--environment",
            "Linux",
            *_semantic_args("uncertain"),
            "--out",
            str(out),
            "--manifest-out",
            str(manifest_out),
        ]
    )
    assert code == 1
    manifest = json.loads(manifest_out.read_text(encoding="utf-8"))
    assert manifest["pass"] is False
    assert manifest["semantic_review"]["verdict"] == "uncertain"
    assert "semantic review verdict is uncertain" in manifest["note"]


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
        ("kind", "invalid kind"),
        ("observe", "invalid observe"),
        ("quality", "invalid quality"),
        ("rationale", "empty rationale"),
        ("duplicate-claim", "duplicate claim IDs"),
        ("zero-bytes", "invalid byte count"),
        ("rejected", "rejected artifact entries are invalid"),
    ],
)
def test_gh_release_yes_validates_complete_selection_before_github_lookup(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    mutation: str,
    expected: str,
) -> None:
    selection = _selection(tmp_path)
    manifest = json.loads(selection.read_text(encoding="utf-8"))
    if mutation == "kind":
        manifest["selected"][0]["kind"] = "unknown"
    elif mutation == "observe":
        manifest["selected"][0]["observe"] = 1
    elif mutation == "quality":
        manifest["selected"][0]["quality"] = float("nan")
    elif mutation == "rationale":
        manifest["selected"][0]["rationale"] = " "
    elif mutation == "duplicate-claim":
        manifest["selected"][0]["claims"] = ["C1", "C1"]
    elif mutation == "zero-bytes":
        manifest["selected"][0]["bytes"] = 0
    else:
        manifest["rejected"] = [{"path": "other.png", "reason": ""}]
    selection.write_text(json.dumps(manifest), encoding="utf-8")

    calls = []

    def record_gh(*args, **kwargs):
        calls.append((args, kwargs))
        raise AssertionError("invalid selection must fail before a GitHub lookup")

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
        raise AssertionError("failing evidence must not mutate GitHub")

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


def test_gh_release_dry_run_does_not_render_unpublishable_urls(
    tmp_path: Path,
) -> None:
    selection = _selection(tmp_path, passing=False)
    out = tmp_path / "section.md"
    manifest_out = tmp_path / "publication.json"

    code = evidence_publish.main(
        [
            str(selection),
            "--backend",
            "gh-release",
            "--repo",
            "dartsim/dart",
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
    manifest = json.loads(manifest_out.read_text(encoding="utf-8"))
    assert manifest["urls"] == {}
    assert all(artifact["url"] is None for artifact in manifest["artifacts"])
    assert "UPLOAD-PLACEHOLDER" in out.read_text(encoding="utf-8")


def test_semantic_review_rejects_empty_fields(tmp_path: Path) -> None:
    selection = _selection(tmp_path)
    args = [
        str(selection),
        "--environment",
        "Linux",
        *_semantic_args(),
        "--out",
        str(tmp_path / "section.md"),
    ]
    args[args.index("metrics report zero invalid contacts")] = " "
    with pytest.raises(SystemExit):
        evidence_publish.main(args)


def test_semantic_review_requires_claim_boundary(tmp_path: Path) -> None:
    selection = _selection(tmp_path)
    args = [
        str(selection),
        "--environment",
        "Linux",
        *_semantic_args(),
        "--out",
        str(tmp_path / "section.md"),
    ]
    index = args.index("--not-proven")
    del args[index : index + 2]
    with pytest.raises(SystemExit):
        evidence_publish.main(args)
