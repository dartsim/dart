"""Unit coverage for PR-body visual-verification section generation."""

from __future__ import annotations

import hashlib
import json
import os
import sys
from http.client import IncompleteRead
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import evidence_publish


@pytest.fixture(autouse=True)
def _forbid_github_access(monkeypatch: pytest.MonkeyPatch) -> None:
    def forbidden(*args, **kwargs):
        raise AssertionError("tests must mock all gh and upload calls")

    monkeypatch.setattr(evidence_publish, "_gh", forbidden)
    monkeypatch.setattr(evidence_publish, "urlopen", forbidden)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class _Completed:
    def __init__(
        self, returncode: int = 0, *, stdout: str = "", stderr: str = ""
    ) -> None:
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


class _Response:
    def __init__(self, body: bytes, status: int = 201) -> None:
        self.status = status
        self.body = body

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False

    def read(self) -> bytes:
        return self.body


_IMAGE_URL = (
    "https://github.com/user-attachments/assets/" "12345678-1234-1234-1234-123456789abc"
)
_VIDEO_URL = (
    "https://github.com/user-attachments/assets/" "abcdef12-abcd-abcd-abcd-abcdef123456"
)


def _mock_gh(monkeypatch: pytest.MonkeyPatch) -> list[list[str]]:
    calls = []

    def fake_gh(args: list[str]) -> _Completed:
        calls.append(list(args))
        if args == ["auth", "token"]:
            return _Completed(stdout="test-token\n")
        assert args == ["api", "repos/dartsim/dart", "--jq", ".id"]
        return _Completed(stdout="1234\n")

    monkeypatch.setattr(evidence_publish, "_gh", fake_gh)
    return calls


def _attachments_args(
    selection: Path,
    out: Path,
    manifest_out: Path,
    *,
    yes: bool = True,
    verdict: str = "pass",
) -> list[str]:
    return [
        str(selection),
        "--backend",
        "attachments",
        "--repo",
        "dartsim/dart",
        *(["--yes"] if yes else []),
        "--environment",
        "Linux",
        *_semantic_args(verdict),
        "--out",
        str(out),
        "--manifest-out",
        str(manifest_out),
    ]


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
    assert "settled pose and contact-count assertions passed" in text
    assert "the box is upright" in text
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


@pytest.mark.parametrize(
    "alias",
    [
        "same-output",
        "selected-artifact",
        "case-output",
        "case-artifact",
        "case-selection",
    ],
)
def test_output_aliases_fail_before_github(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    alias: str,
) -> None:
    selection = _selection(tmp_path)
    calls: list[list[str]] = []

    def fake_gh(args: list[str], *, check: bool = True) -> "_Completed":
        calls.append(list(args))
        return _Completed()

    monkeypatch.setattr(evidence_publish, "_gh", fake_gh)
    out = tmp_path / ("shot.png" if alias == "selected-artifact" else "section.md")
    manifest_out = out if alias == "same-output" else tmp_path / "publication.json"
    if alias == "case-output":
        manifest_out = tmp_path / "Section.md"
        assert not out.exists() and not manifest_out.exists()
    elif alias == "case-artifact":
        out = tmp_path / "SHOT.PNG"
    elif alias == "case-selection":
        manifest_out = tmp_path / "SELECTION.JSON"
    code = evidence_publish.main(
        [
            str(selection),
            "--backend",
            "attachments",
            "--repo",
            "dartsim/dart",
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

    assert code == 2
    assert calls == []


def test_unwritable_output_shape_fails_before_github(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    selection = _selection(tmp_path)
    parent_file = tmp_path / "not-a-directory"
    parent_file.write_text("x", encoding="utf-8")
    calls: list[list[str]] = []

    def fake_gh(args: list[str], *, check: bool = True) -> "_Completed":
        calls.append(list(args))
        return _Completed()

    monkeypatch.setattr(evidence_publish, "_gh", fake_gh)
    code = evidence_publish.main(
        [
            str(selection),
            "--backend",
            "attachments",
            "--repo",
            "dartsim/dart",
            "--yes",
            "--environment",
            "Linux",
            *_semantic_args(),
            "--out",
            str(parent_file / "section.md"),
        ]
    )

    assert code == 2
    assert calls == []


def test_attachments_requires_repo(tmp_path: Path) -> None:
    selection = _selection(tmp_path)
    code = evidence_publish.main(
        [
            str(selection),
            "--backend",
            "attachments",
            "--environment",
            "Linux",
            *_semantic_args(),
            "--out",
            str(tmp_path / "x.md"),
        ]
    )
    assert code == 2


@pytest.mark.parametrize("repo", ["dartsim", "/dart", "dartsim/", "bad owner/dart"])
def test_attachments_rejects_invalid_repository(tmp_path: Path, repo: str) -> None:
    selection = _selection(tmp_path)
    code = evidence_publish.main(
        [
            str(selection),
            "--backend",
            "attachments",
            "--repo",
            repo,
            "--environment",
            "Linux",
            *_semantic_args(),
            "--out",
            str(tmp_path / "section.md"),
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
        ("claim-text", "invalid text"),
        ("kind", "invalid kind"),
        ("observe", "invalid observe"),
        ("quality", "invalid quality"),
        ("rationale", "empty rationale"),
        ("duplicate-claim", "duplicate claim IDs"),
        ("zero-bytes", "invalid byte count"),
        ("rejected", "rejected artifact entries are invalid"),
    ],
)
def test_attachments_yes_validates_complete_selection_before_github_lookup(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    mutation: str,
    expected: str,
) -> None:
    selection = _selection(tmp_path)
    manifest = json.loads(selection.read_text(encoding="utf-8"))
    if mutation == "claim-text":
        manifest["claims"][0]["text"] = " "
    elif mutation == "kind":
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
            "attachments",
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
def test_attachments_yes_revalidates_selected_artifacts_before_mutation(
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
            "attachments",
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
    args[args.index("settled pose and contact-count assertions passed")] = " "
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


@pytest.mark.parametrize("alias", ["dot", "symlink", "hardlink"])
def test_selection_rejects_aliases_of_one_artifact(tmp_path: Path, alias: str) -> None:
    selection = _selection(tmp_path)
    manifest = json.loads(selection.read_text(encoding="utf-8"))
    if alias == "dot":
        path = "./shot.png"
    else:
        path = "alias.png"
        link = tmp_path / path
        if alias == "symlink":
            link.symlink_to(tmp_path / "shot.png")
        else:
            os.link(tmp_path / "shot.png", link)
    manifest["selected"][1] = {
        **manifest["selected"][0],
        "path": path,
        "claims": ["C2"],
    }
    selection.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(ValueError, match="duplicated"):
        evidence_publish._load_selection(selection)


def test_attachments_success_embeds_upload_response_urls(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    selection = _selection(tmp_path)
    out = tmp_path / "section.md"
    manifest_out = tmp_path / "publication.json"
    gh_calls = _mock_gh(monkeypatch)
    requests = []
    urls = iter([_IMAGE_URL, _VIDEO_URL])

    def upload(request, *, timeout):
        requests.append(request)
        assert timeout > 0
        assert request.get_method() == "POST"
        assert request.get_header("Authorization") == "Bearer test-token"
        assert request.get_header("Accept") == "application/json"
        parsed = urlsplit(request.full_url)
        assert parsed.scheme == "https"
        assert parsed.netloc == "uploads.github.com"
        assert parsed.path == "/user-attachments/assets"
        query = parse_qs(parsed.query)
        name = "shot.png" if len(requests) == 1 else "clip.mp4"
        mime = "image/png" if len(requests) == 1 else "video/mp4"
        assert query == {
            "name": [name],
            "content_type": [mime],
            "repository_id": ["1234"],
        }
        assert request.get_header("Content-type") == mime
        assert request.data == (tmp_path / name).read_bytes()
        return _Response(json.dumps({"url": next(urls)}).encode())

    monkeypatch.setattr(evidence_publish, "urlopen", upload)
    assert evidence_publish.main(_attachments_args(selection, out, manifest_out)) == 0
    assert gh_calls == [
        ["auth", "token"],
        ["api", "repos/dartsim/dart", "--jq", ".id"],
    ]
    assert len(requests) == 2
    text = out.read_text(encoding="utf-8")
    assert f"![contact markers at rest]({_IMAGE_URL})" in text
    assert f"\n\n{_VIDEO_URL}\n\n" in text
    assert "UPLOAD-PLACEHOLDER" not in text
    manifest = json.loads(manifest_out.read_text(encoding="utf-8"))
    assert manifest["schema_version"] == "dart.evidence_publication/v5"
    assert manifest["status"] == "published"
    assert manifest["pass"] is True
    assert manifest["uploaded"] is True
    assert manifest["repository"] == "dartsim/dart"
    assert manifest["url_provenance"] == "upload_response"
    assert manifest["urls"] == {"shot.png": _IMAGE_URL, "clip.mp4": _VIDEO_URL}
    assert manifest["artifacts"] == [
        {
            "path": name,
            "bytes": 3,
            "sha256": _sha256(tmp_path / name),
            "url": url,
            "url_provenance": "upload_response",
        }
        for name, url in [("shot.png", _IMAGE_URL), ("clip.mp4", _VIDEO_URL)]
    ]


def test_attachments_dry_run_makes_no_github_calls(tmp_path: Path) -> None:
    selection = _selection(tmp_path)
    out = tmp_path / "section.md"
    manifest_out = tmp_path / "publication.json"
    assert (
        evidence_publish.main(
            _attachments_args(selection, out, manifest_out, yes=False)
        )
        == 0
    )
    manifest = json.loads(manifest_out.read_text(encoding="utf-8"))
    assert manifest["status"] == "dry_run"
    assert manifest["uploaded"] is False
    assert manifest["urls"] == {}
    assert manifest["url_provenance"] is None
    assert all(artifact["url"] is None for artifact in manifest["artifacts"])
    assert all(artifact["url_provenance"] is None for artifact in manifest["artifacts"])
    assert "UPLOAD-PLACEHOLDER" in out.read_text(encoding="utf-8")


@pytest.mark.parametrize(
    ("passing", "verdict", "yes"),
    [
        (False, "pass", True),
        (False, "pass", False),
        (True, "fail", True),
        (True, "uncertain", True),
    ],
)
def test_attachments_failing_gates_make_no_github_calls(
    tmp_path: Path, passing: bool, verdict: str, yes: bool
) -> None:
    selection = _selection(tmp_path, passing=passing)
    out = tmp_path / "section.md"
    manifest_out = tmp_path / "publication.json"
    assert (
        evidence_publish.main(
            _attachments_args(selection, out, manifest_out, yes=yes, verdict=verdict)
        )
        == 1
    )
    manifest = json.loads(manifest_out.read_text(encoding="utf-8"))
    assert manifest["status"] == "not_ready"
    assert manifest["pass"] is False
    assert manifest["uploaded"] is False
    assert manifest["urls"] == {}
    assert all(artifact["url"] is None for artifact in manifest["artifacts"])
    assert "UPLOAD-PLACEHOLDER" in out.read_text(encoding="utf-8")


@pytest.mark.parametrize(
    ("status", "body"),
    [
        (200, json.dumps({"url": _IMAGE_URL}).encode()),
        (202, json.dumps({"url": _IMAGE_URL}).encode()),
        (201, b"not JSON"),
        (201, b"[]"),
        (201, b"{}"),
        (201, b'{"url": null}'),
        (201, b'{"url": "https://example.com/image.png"}'),
        (201, b'{"url": "https://github.com/user-attachments/assets/not-a-uuid"}'),
        (201, json.dumps({"url": _IMAGE_URL.replace("https:", "http:")}).encode()),
        (201, json.dumps({"url": _IMAGE_URL + "/"}).encode()),
        (201, json.dumps({"url": _IMAGE_URL + "?download=1"}).encode()),
        (201, json.dumps({"url": _IMAGE_URL + "#image"}).encode()),
    ],
)
def test_invalid_upload_response_fails_closed(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    status: int,
    body: bytes,
) -> None:
    selection = _selection(tmp_path)
    out = tmp_path / "section.md"
    manifest_out = tmp_path / "publication.json"
    out.write_text("OLD SUCCESS URL\n", encoding="utf-8")
    manifest_out.write_text(json.dumps({"pass": True}), encoding="utf-8")
    _mock_gh(monkeypatch)
    uploads = []

    def upload(request, *, timeout):
        uploads.append(request)
        return _Response(body, status)

    monkeypatch.setattr(evidence_publish, "urlopen", upload)
    assert evidence_publish.main(_attachments_args(selection, out, manifest_out)) == 2
    assert len(uploads) == 1
    failed = json.loads(manifest_out.read_text(encoding="utf-8"))
    assert failed["status"] == "failed"
    assert failed["pass"] is False
    assert failed["uploaded"] is False
    assert failed["error"]
    assert failed["urls"] == {}
    assert all(artifact["url"] is None for artifact in failed["artifacts"])
    assert all(artifact["url_provenance"] is None for artifact in failed["artifacts"])
    section = out.read_text(encoding="utf-8")
    assert "OLD SUCCESS URL" not in section
    assert "**Publishable evidence**: no" in section
    assert "**Verdict**: pass" not in section


def test_partial_upload_invalidates_stale_success_outputs(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    selection = _selection(tmp_path)
    out = tmp_path / "section.md"
    manifest_out = tmp_path / "publication.json"
    out.write_text("OLD SUCCESS URL\n", encoding="utf-8")
    manifest_out.write_text(json.dumps({"pass": True}), encoding="utf-8")
    _mock_gh(monkeypatch)
    uploads = []

    def upload(request, *, timeout):
        uploads.append(request)
        state = json.loads(manifest_out.read_text(encoding="utf-8"))
        assert state["status"] == "publishing"
        assert state["pass"] is False
        assert state["urls"] == {}
        assert "OLD SUCCESS URL" not in out.read_text(encoding="utf-8")
        if len(uploads) == 1:
            return _Response(json.dumps({"url": _IMAGE_URL}).encode())
        raise OSError("simulated interrupted upload")

    monkeypatch.setattr(evidence_publish, "urlopen", upload)
    assert evidence_publish.main(_attachments_args(selection, out, manifest_out)) == 2
    assert len(uploads) == 2
    failed = json.loads(manifest_out.read_text(encoding="utf-8"))
    assert failed["status"] == "failed"
    assert failed["pass"] is False
    assert failed["uploaded"] is False
    assert failed["urls"] == {}
    assert "simulated interrupted upload" in failed["error"]
    assert _IMAGE_URL not in out.read_text(encoding="utf-8")
    assert "**Publishable evidence**: no" in out.read_text(encoding="utf-8")


def test_incomplete_upload_response_fails_closed(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    selection = _selection(tmp_path)
    out = tmp_path / "section.md"
    manifest_out = tmp_path / "publication.json"
    out.write_text("OLD SUCCESS URL\n", encoding="utf-8")
    manifest_out.write_text(json.dumps({"pass": True}), encoding="utf-8")
    _mock_gh(monkeypatch)
    response = _Response(b"")

    def incomplete_read():
        raise IncompleteRead(b'{"url":', 100)

    monkeypatch.setattr(response, "read", incomplete_read)
    monkeypatch.setattr(evidence_publish, "urlopen", lambda *args, **kwargs: response)
    assert evidence_publish.main(_attachments_args(selection, out, manifest_out)) == 2
    failed = json.loads(manifest_out.read_text(encoding="utf-8"))
    assert failed["status"] == "failed"
    assert failed["pass"] is False
    assert failed["uploaded"] is False
    assert "IncompleteRead" in failed["error"]
    assert failed["urls"] == {}
    assert all(artifact["url"] is None for artifact in failed["artifacts"])
    section = out.read_text(encoding="utf-8")
    assert "OLD SUCCESS URL" not in section
    assert "**Publishable evidence**: no" in section


def test_unsupported_suffix_fails_before_any_upload(tmp_path: Path) -> None:
    selection = _selection(tmp_path)
    manifest = json.loads(selection.read_text(encoding="utf-8"))
    (tmp_path / "clip.mp4").rename(tmp_path / "clip.avi")
    manifest["selected"][1]["path"] = "clip.avi"
    selection.write_text(json.dumps(manifest), encoding="utf-8")
    out = tmp_path / "section.md"
    manifest_out = tmp_path / "publication.json"
    assert evidence_publish.main(_attachments_args(selection, out, manifest_out)) == 2
    failed = json.loads(manifest_out.read_text(encoding="utf-8"))
    assert failed["pass"] is False
    assert failed["status"] == "failed"
    assert "unsupported" in failed["error"].lower()


@pytest.mark.parametrize(
    ("suffix", "mime", "kind"),
    [
        (".png", "image/png", "still"),
        (".jpg", "image/jpeg", "still"),
        (".jpeg", "image/jpeg", "still"),
        (".gif", "image/gif", "still"),
        (".mp4", "video/mp4", "video"),
        (".webm", "video/webm", "video"),
        (".mov", "video/quicktime", "video"),
    ],
)
def test_attachments_supported_suffixes_have_explicit_mime_types(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    suffix: str,
    mime: str,
    kind: str,
) -> None:
    selection = _selection(tmp_path)
    manifest = json.loads(selection.read_text(encoding="utf-8"))
    name = f"shot with spaces{suffix.upper()}"
    (tmp_path / "shot.png").rename(tmp_path / name)
    manifest["selected"][0].update(path=name, kind=kind)
    selection.write_text(json.dumps(manifest), encoding="utf-8")
    _mock_gh(monkeypatch)
    requests = []

    def upload(request, *, timeout):
        requests.append(request)
        if len(requests) == 1:
            assert parse_qs(urlsplit(request.full_url).query)["name"] == [name]
            assert parse_qs(urlsplit(request.full_url).query)["content_type"] == [mime]
            assert request.get_header("Content-type") == mime
        return _Response(json.dumps({"url": _IMAGE_URL}).encode())

    monkeypatch.setattr(evidence_publish, "urlopen", upload)
    assert (
        evidence_publish.main(
            _attachments_args(
                selection, tmp_path / "section.md", tmp_path / "publication.json"
            )
        )
        == 0
    )
    assert len(requests) == 2


@pytest.mark.parametrize("backend", ["manual", "dry-run", "attachments"])
@pytest.mark.parametrize("failure_at", ["section", "manifest"])
@pytest.mark.parametrize("error_type", [KeyboardInterrupt, OSError])
def test_passing_rewrite_invalidates_previous_manifest_before_section(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    backend: str,
    failure_at: str,
    error_type: type[BaseException],
) -> None:
    selection = _selection(tmp_path)
    out = tmp_path / "section.md"
    manifest_out = tmp_path / "publication.json"
    out.write_text("OLD SUCCESS URL\n", encoding="utf-8")
    manifest_out.write_text(json.dumps({"pass": True}), encoding="utf-8")
    atomic_write = evidence_publish._atomic_write_text
    interrupted = False

    def interrupt_write(path: Path, text: str) -> None:
        nonlocal interrupted
        if path == out and text.startswith("## Visual verification\n"):
            assert not manifest_out.exists()
            atomic_write(path, text)
            if failure_at == "section":
                interrupted = True
                raise error_type("simulated section write failure")
            return
        if path == manifest_out and json.loads(text)["pass"]:
            interrupted = True
            raise error_type("simulated final manifest write failure")
        atomic_write(path, text)

    _mock_gh(monkeypatch)
    monkeypatch.setattr(
        evidence_publish,
        "urlopen",
        lambda request, **kwargs: _Response(json.dumps({"url": _IMAGE_URL}).encode()),
    )
    monkeypatch.setattr(evidence_publish, "_atomic_write_text", interrupt_write)
    args = _attachments_args(selection, out, manifest_out, yes=backend == "attachments")
    if backend == "manual":
        args[args.index("--backend") + 1] = "manual"

    if error_type is KeyboardInterrupt:
        with pytest.raises(KeyboardInterrupt):
            evidence_publish.main(args)
    else:
        assert evidence_publish.main(args) == 2

    assert interrupted
    assert "OLD SUCCESS URL" not in out.read_text(encoding="utf-8")
    if manifest_out.exists():
        assert json.loads(manifest_out.read_text(encoding="utf-8"))["pass"] is False


@pytest.mark.parametrize("status", ["publishing", "failed", "not_ready"])
def test_interrupted_section_write_leaves_non_passing_manifest(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, status: str
) -> None:
    selection = _selection(tmp_path, passing=status != "not_ready")
    out = tmp_path / "section.md"
    manifest_out = tmp_path / "publication.json"
    out.write_text("OLD SUCCESS URL\n", encoding="utf-8")
    manifest_out.write_text(json.dumps({"pass": True}), encoding="utf-8")
    atomic_write = evidence_publish._atomic_write_text
    uploads = []

    def interrupt_section_write(path: Path, text: str) -> None:
        atomic_write(path, text)
        if path == out and (f"**Status**: `{status}`" in text or status == "not_ready"):
            raise KeyboardInterrupt("simulated interruption between output writes")

    def upload(request, *, timeout):
        uploads.append(request)
        raise OSError("simulated interrupted upload")

    _mock_gh(monkeypatch)
    monkeypatch.setattr(evidence_publish, "urlopen", upload)
    monkeypatch.setattr(evidence_publish, "_atomic_write_text", interrupt_section_write)
    with pytest.raises(KeyboardInterrupt):
        evidence_publish.main(_attachments_args(selection, out, manifest_out))

    manifest = json.loads(manifest_out.read_text(encoding="utf-8"))
    assert manifest["pass"] is False
    assert manifest["status"] == status
    assert manifest["urls"] == {}
    assert "OLD SUCCESS URL" not in out.read_text(encoding="utf-8")
    assert len(uploads) == (1 if status == "failed" else 0)
