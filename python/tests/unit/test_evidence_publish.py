"""Unit coverage for PR-body visual-verification section generation."""

from __future__ import annotations

import hashlib
import json
import os
import sys
import tempfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import evidence_publish


@pytest.fixture(autouse=True)
def isolate_tempdir(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    tempdir = tmp_path / "temp"
    tempdir.mkdir()
    monkeypatch.setattr(tempfile, "tempdir", str(tempdir))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _publication_args(
    selection: Path,
    out: Path,
    manifest_out: Path,
    *,
    verdict: str = "pass",
) -> list[str]:
    return [
        str(selection),
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


def test_section_emits_local_references_and_context(tmp_path: Path) -> None:
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
    assert (
        f"![contact markers at rest]({_sha256(tmp_path / 'shot.png')}.png)" in text
    )
    assert f"\n\n![]({_sha256(tmp_path / 'clip.mp4')}.mp4)\n\n" in text
    assert "![settling clip]" not in text
    assert "UPLOAD-PLACEHOLDER" not in text
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
    assert manifest["status"] == "not_ready"
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
def test_output_aliases_fail_before_writing(
    tmp_path: Path,
    alias: str,
) -> None:
    selection = _selection(tmp_path)
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


def test_unwritable_output_shape_fails_before_writing(
    tmp_path: Path,
) -> None:
    selection = _selection(tmp_path)
    parent_file = tmp_path / "not-a-directory"
    parent_file.write_text("x", encoding="utf-8")
    code = evidence_publish.main(
        [
            str(selection),
            "--environment",
            "Linux",
            *_semantic_args(),
            "--out",
            str(parent_file / "section.md"),
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
    assert manifest["status"] == "not_ready"
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
def test_validates_complete_selection_before_writing(
    tmp_path: Path,
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
def test_revalidates_selected_artifacts_before_writing(
    tmp_path: Path,
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


def test_publication_stages_verified_files_in_selection_order(tmp_path: Path) -> None:
    selection_dir = tmp_path / "selection"
    selection_dir.mkdir()
    selection = _selection(selection_dir)
    manifest = json.loads(selection.read_text(encoding="utf-8"))
    frames = selection_dir / "frames"
    frames.mkdir()
    (selection_dir / "shot.png").rename(frames / "shot.png")
    (selection_dir / "clip.mp4").rename(tmp_path / "clip.mp4")
    manifest["selected"][0]["path"] = "frames/./shot.png"
    manifest["selected"][1]["path"] = str(tmp_path / "clip.mp4")
    selection.write_text(json.dumps(manifest), encoding="utf-8")
    out = tmp_path / "section.md"
    manifest_out = tmp_path / "publication.json"

    assert evidence_publish.main(_publication_args(selection, out, manifest_out)) == 0

    names = [
        artifact["sha256"] + Path(artifact["path"]).suffix.lower()
        for artifact in manifest["selected"]
    ]
    text = out.read_text(encoding="utf-8")
    assert f"![contact markers at rest]({names[0]})" in text
    assert f"\n\n![]({names[1]})\n\n" in text
    publication = json.loads(manifest_out.read_text(encoding="utf-8"))
    assert publication["schema_version"] == "dart.evidence_publication/v5"
    assert publication["status"] == "ready"
    assert publication["pass"] is True
    assert publication["attach"] == names
    attach_dir = Path(publication["attach_dir"])
    assert attach_dir.is_absolute()
    assert attach_dir.parent == tmp_path / "temp"
    assert attach_dir.name.startswith("dart-evidence-attach-")
    assert {path.name for path in attach_dir.iterdir()} == set(names)
    for name, artifact in zip(names, manifest["selected"]):
        assert (attach_dir / name).is_file()
        assert _sha256(attach_dir / name) == artifact["sha256"]
    assert publication["artifacts"] == [
        {key: artifact[key] for key in ("path", "bytes", "sha256")}
        for artifact in manifest["selected"]
    ]
    assert (
        not {
            "backend",
            "repository",
            "urls",
            "url_provenance",
            "uploaded",
            "attempt_id",
        }
        & publication.keys()
    )


def test_caption_is_escaped_like_gh_alt_text(tmp_path: Path) -> None:
    selection = _selection(tmp_path)
    manifest = json.loads(selection.read_text(encoding="utf-8"))
    manifest["selected"][0]["caption"] = "a](https://example.com) [b]\nc\\d"
    selection.write_text(json.dumps(manifest), encoding="utf-8")
    out = tmp_path / "section.md"
    manifest_out = tmp_path / "publication.json"

    assert evidence_publish.main(_publication_args(selection, out, manifest_out)) == 0

    name = manifest["selected"][0]["sha256"] + ".png"
    assert f"![a\\](https://example.com) \\[b\\] c\\\\d]({name})" in out.read_text(
        encoding="utf-8"
    )


def test_changed_staged_copy_fails_before_writing(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    selection = _selection(tmp_path)
    out = tmp_path / "section.md"
    manifest_out = tmp_path / "publication.json"

    def corrupt_copy(source: Path, destination: Path) -> None:
        Path(destination).write_bytes(b"changed")

    monkeypatch.setattr(evidence_publish.shutil, "copyfile", corrupt_copy)

    assert evidence_publish.main(_publication_args(selection, out, manifest_out)) == 2
    assert not out.exists()
    assert not manifest_out.exists()
    assert not list((tmp_path / "temp").iterdir())


def test_failed_rerun_drops_previous_passing_manifest(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    selection = _selection(tmp_path)
    out = tmp_path / "section.md"
    manifest_out = tmp_path / "publication.json"
    assert evidence_publish.main(_publication_args(selection, out, manifest_out)) == 0
    assert json.loads(manifest_out.read_text(encoding="utf-8"))["pass"] is True
    staged_dirs = set((tmp_path / "temp").iterdir())

    def corrupt_copy(source: Path, destination: Path) -> None:
        Path(destination).write_bytes(b"changed")

    monkeypatch.setattr(evidence_publish.shutil, "copyfile", corrupt_copy)

    assert evidence_publish.main(_publication_args(selection, out, manifest_out)) == 2
    assert not manifest_out.exists()
    assert set((tmp_path / "temp").iterdir()) == staged_dirs


def test_identical_content_uses_one_staged_file(tmp_path: Path) -> None:
    selection = _selection(tmp_path)
    manifest = json.loads(selection.read_text(encoding="utf-8"))
    (tmp_path / "clip.mp4").write_bytes((tmp_path / "shot.png").read_bytes())
    manifest["selected"][1]["sha256"] = manifest["selected"][0]["sha256"]
    selection.write_text(json.dumps(manifest), encoding="utf-8")
    out = tmp_path / "section.md"
    manifest_out = tmp_path / "publication.json"

    assert evidence_publish.main(_publication_args(selection, out, manifest_out)) == 0

    name = manifest["selected"][0]["sha256"] + ".png"
    text = out.read_text(encoding="utf-8")
    assert f"![contact markers at rest]({name})" in text
    assert f"\n\n![]({name})\n\n" in text
    publication = json.loads(manifest_out.read_text(encoding="utf-8"))
    assert publication["attach"] == [name]
    attach_dir = Path(publication["attach_dir"])
    assert list(attach_dir.iterdir()) == [attach_dir / name]


def test_more_than_50_artifacts_fail_before_writing(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    selection = _selection(tmp_path)
    manifest = json.loads(selection.read_text(encoding="utf-8"))
    for index in range(49):
        name = f"extra-{index}.png"
        (tmp_path / name).write_bytes(b"png")
        manifest["selected"].append({**manifest["selected"][0], "path": name})
    manifest["total_bytes"] = sum(
        artifact["bytes"] for artifact in manifest["selected"]
    )
    selection.write_text(json.dumps(manifest), encoding="utf-8")
    out = tmp_path / "section.md"
    manifest_out = tmp_path / "publication.json"

    assert evidence_publish.main(_publication_args(selection, out, manifest_out)) == 2

    error = capsys.readouterr().err
    assert "gh" in error and "50" in error and "--attach" in error
    assert "per-command" in error
    assert not out.exists()
    assert not manifest_out.exists()


@pytest.mark.parametrize(
    ("passing", "verdict"),
    [(False, "pass"), (True, "fail"), (True, "uncertain")],
)
def test_failing_gates_yield_not_ready(
    tmp_path: Path, passing: bool, verdict: str
) -> None:
    selection = _selection(tmp_path, passing=passing)
    out = tmp_path / "section.md"
    manifest_out = tmp_path / "publication.json"

    assert (
        evidence_publish.main(
            _publication_args(selection, out, manifest_out, verdict=verdict)
        )
        == 1
    )

    manifest = json.loads(manifest_out.read_text(encoding="utf-8"))
    assert manifest["status"] == "not_ready"
    assert manifest["pass"] is False


@pytest.mark.parametrize(
    ("suffix", "kind"),
    [
        (".png", "still"),
        (".jpg", "still"),
        (".jpeg", "still"),
        (".gif", "still"),
        (".webp", "still"),
        (".svg", "still"),
        (".mp4", "video"),
        (".webm", "video"),
        (".mov", "video"),
    ],
)
def test_supported_attachment_suffixes(tmp_path: Path, suffix: str, kind: str) -> None:
    selection = _selection(tmp_path)
    manifest = json.loads(selection.read_text(encoding="utf-8"))
    name = f"shot{suffix.upper()}"
    (tmp_path / "shot.png").rename(tmp_path / name)
    manifest["selected"][0].update(path=name, kind=kind)
    selection.write_text(json.dumps(manifest), encoding="utf-8")

    out = tmp_path / "section.md"
    assert (
        evidence_publish.main(
            _publication_args(selection, out, tmp_path / "publication.json")
        )
        == 0
    )
    caption = "" if kind == "video" else "contact markers at rest"
    staged_name = manifest["selected"][0]["sha256"] + suffix
    assert f"![{caption}]({staged_name})" in out.read_text(encoding="utf-8")


@pytest.mark.parametrize("failure_at", ["section", "manifest"])
@pytest.mark.parametrize("error_type", [KeyboardInterrupt, OSError])
def test_passing_rewrite_invalidates_previous_manifest_before_section(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
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

    monkeypatch.setattr(evidence_publish, "_atomic_write_text", interrupt_write)
    args = _publication_args(selection, out, manifest_out)

    if error_type is KeyboardInterrupt:
        with pytest.raises(KeyboardInterrupt):
            evidence_publish.main(args)
    else:
        assert evidence_publish.main(args) == 2

    assert interrupted
    assert "OLD SUCCESS URL" not in out.read_text(encoding="utf-8")
    if manifest_out.exists():
        assert json.loads(manifest_out.read_text(encoding="utf-8"))["pass"] is False


def test_interrupted_section_write_leaves_non_passing_manifest(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    selection = _selection(tmp_path, passing=False)
    out = tmp_path / "section.md"
    manifest_out = tmp_path / "publication.json"
    out.write_text("OLD SUCCESS URL\n", encoding="utf-8")
    manifest_out.write_text(json.dumps({"pass": True}), encoding="utf-8")
    atomic_write = evidence_publish._atomic_write_text

    def interrupt_section_write(path: Path, text: str) -> None:
        atomic_write(path, text)
        if path == out:
            raise KeyboardInterrupt("simulated interruption between output writes")

    monkeypatch.setattr(evidence_publish, "_atomic_write_text", interrupt_section_write)
    with pytest.raises(KeyboardInterrupt):
        evidence_publish.main(_publication_args(selection, out, manifest_out))

    manifest = json.loads(manifest_out.read_text(encoding="utf-8"))
    assert manifest["pass"] is False
    assert manifest["status"] == "not_ready"
    assert "OLD SUCCESS URL" not in out.read_text(encoding="utf-8")
