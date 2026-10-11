"""Generate a PR-body visual-verification section with local media references.

Takes an evidence selection manifest (scripts/evidence_select.py) plus
context fields and produces the "Visual verification" markdown a reviewer
can read without reproducing the environment: what each artifact shows, the
text oracle, semantic image observation, reconciliation/verdict, explicit
claim boundary, environment/configuration, limitations, and reproduction
commands. This script never accesses the network. Insert the section into the
complete PR body because gh pr edit --body-file replaces the whole body, or
post it separately with gh pr comment. After explicit maintainer/user approval,
run GitHub CLI 2.99+ from the manifest attach_dir (<out>.attach, which each run
refills with hash-named copies) with an explicit PR URL (or a
number plus -R owner/repo); gh pr create also needs -R owner/repo and --head
<branch>. Pass --attach for each manifest attach name (at most 50 per command).
GitHub CLI uploads media and rewrites the local references in place. The
fallback is drag-and-drop in the web editor: replace each reference with the
URL the editor inserts. Media is GitHub-hosted, never committed.
"""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import math
import os
import re
import shutil
import sys
import tempfile
from pathlib import Path
from typing import Any

SCHEMA_VERSION = "dart.evidence_publication/v5"
SELECTION_SCHEMA_VERSION = "dart.evidence_selection/v1"

# gh --attach validates file types and sizes itself and renders these as players.
_VIDEO_SUFFIXES = {".mp4", ".mov", ".webm"}
_ARTIFACT_KINDS = {"still", "grid", "composite", "video"}
# Staged copies are named <sha256><suffix>; a rerun clears only these names.
_STAGED_NAME = re.compile(r"[0-9a-f]{64}(?:\.[^.]+)?")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_selection(path: Path) -> dict[str, Any]:
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict):
        raise ValueError("selection manifest must be a JSON object")
    if manifest.get("schema_version") != SELECTION_SCHEMA_VERSION:
        raise ValueError(
            f"selection manifest must declare schema_version="
            f"{SELECTION_SCHEMA_VERSION!r}"
        )
    claims = manifest.get("claims")
    if not isinstance(claims, list) or not claims:
        raise ValueError("selection manifest has no claims")
    claim_ids: set[str] = set()
    for claim in claims:
        if not isinstance(claim, dict):
            raise ValueError("selection claim entries must be objects")
        claim_id = claim.get("id")
        if not isinstance(claim_id, str) or not claim_id.strip():
            raise ValueError("selection claim IDs must be non-empty strings")
        if claim_id in claim_ids:
            raise ValueError(f"selection claim ID {claim_id!r} is duplicated")
        if not isinstance(claim.get("text"), str) or not claim["text"].strip():
            raise ValueError(f"selection claim {claim_id!r} has invalid text")
        if type(claim.get("covered")) is not bool:
            raise ValueError(
                f"selection claim {claim_id!r} must declare boolean covered"
            )
        claim_ids.add(claim_id)

    selected = manifest.get("selected")
    if not isinstance(selected, list) or not selected:
        raise ValueError("selection manifest has no selected artifacts")
    if len(selected) > 50:
        raise ValueError("selection exceeds gh's per-command --attach limit of 50")
    selected_files: set[tuple[int, int]] = set()
    covered_by_artifacts: set[str] = set()
    selected_bytes = 0
    for artifact in selected:
        if not isinstance(artifact, dict):
            raise ValueError("selected artifact entries must be objects")
        declared_path = artifact.get("path")
        if not isinstance(declared_path, str) or not declared_path.strip():
            raise ValueError("selected artifact paths must be non-empty strings")
        name = Path(declared_path).name
        if not name:
            raise ValueError(f"selected artifact has invalid path {declared_path!r}")

        kind = artifact.get("kind")
        if kind not in _ARTIFACT_KINDS:
            raise ValueError(
                f"selected artifact {declared_path!r} has invalid kind {kind!r}"
            )
        supported = artifact.get("claims")
        if (
            not isinstance(supported, list)
            or not supported
            or not all(isinstance(item, str) and item.strip() for item in supported)
        ):
            raise ValueError(f"selected artifact {declared_path!r} must support claims")
        if len(set(supported)) != len(supported):
            raise ValueError(
                f"selected artifact {declared_path!r} has duplicate claim IDs"
            )
        unknown = sorted(set(supported) - claim_ids)
        if unknown:
            raise ValueError(
                f"selected artifact {declared_path!r} references unknown "
                f"claims {unknown}"
            )
        covered_by_artifacts.update(supported)

        for field in ("caption", "observe", "rationale"):
            if not isinstance(artifact.get(field), str):
                raise ValueError(
                    f"selected artifact {declared_path!r} has invalid {field}"
                )
        if not artifact["rationale"].strip():
            raise ValueError(f"selected artifact {declared_path!r} has empty rationale")
        quality = artifact.get("quality")
        if (
            isinstance(quality, bool)
            or not isinstance(quality, (int, float))
            or not math.isfinite(quality)
            or not 0.0 <= quality <= 1.0
        ):
            raise ValueError(f"selected artifact {declared_path!r} has invalid quality")
        expected_bytes = artifact.get("bytes")
        if type(expected_bytes) is not int or expected_bytes <= 0:
            raise ValueError(
                f"selected artifact {declared_path!r} has invalid byte count"
            )
        expected_sha = artifact.get("sha256")
        if not isinstance(expected_sha, str) or not re.fullmatch(
            r"[0-9a-f]{64}", expected_sha
        ):
            raise ValueError(f"selected artifact {declared_path!r} has invalid sha256")
        artifact_path = Path(declared_path)
        if not artifact_path.is_absolute():
            artifact_path = path.parent / artifact_path
        artifact_path = artifact_path.resolve()
        if not artifact_path.is_file():
            raise ValueError(
                f"selected artifact is missing or not a regular file: {artifact_path}"
            )
        # Compare files, not strings, so ./a.png, symlinks and hardlinks to
        # one file cannot count as separate artifacts.
        stat = artifact_path.stat()
        if (stat.st_dev, stat.st_ino) in selected_files:
            raise ValueError(f"selected artifact path {declared_path!r} is duplicated")
        selected_files.add((stat.st_dev, stat.st_ino))
        actual_bytes = stat.st_size
        if actual_bytes != expected_bytes:
            raise ValueError(
                f"selected artifact {declared_path!r} changed size after "
                f"selection: expected {expected_bytes}, got {actual_bytes}"
            )
        actual_sha = _sha256(artifact_path)
        if actual_sha != expected_sha:
            raise ValueError(
                f"selected artifact {declared_path!r} changed content after selection"
            )
        selected_bytes += actual_bytes

    expected_uncovered = sorted(claim_ids - covered_by_artifacts)
    uncovered = manifest.get("uncovered_claims")
    if (
        not isinstance(uncovered, list)
        or not all(isinstance(item, str) for item in uncovered)
        or sorted(uncovered) != expected_uncovered
    ):
        raise ValueError(
            "selection uncovered_claims does not match selected artifact coverage"
        )
    for claim in claims:
        expected_covered = claim["id"] in covered_by_artifacts
        if claim["covered"] is not expected_covered:
            raise ValueError(
                f"selection claim {claim['id']!r} covered flag does not match "
                "selected artifact coverage"
            )
    if type(manifest.get("pass")) is not bool or manifest["pass"] is not (
        not expected_uncovered
    ):
        raise ValueError(
            "selection pass flag does not match selected artifact coverage"
        )
    if (
        type(manifest.get("total_bytes")) is not int
        or manifest["total_bytes"] != selected_bytes
    ):
        raise ValueError("selection total_bytes does not match selected artifact sizes")
    rejected = manifest.get("rejected")
    if not isinstance(rejected, list):
        raise ValueError("selection rejected artifacts must be a list")
    for artifact in rejected:
        if (
            not isinstance(artifact, dict)
            or not isinstance(artifact.get("path"), str)
            or not artifact["path"].strip()
            or not isinstance(artifact.get("reason"), str)
            or not artifact["reason"].strip()
        ):
            raise ValueError("selection rejected artifact entries are invalid")
    return manifest


def _artifact_reference(artifact: dict[str, Any], path: str) -> str:
    caption = (
        ""
        if Path(artifact["path"]).suffix.lower() in _VIDEO_SUFFIXES
        else artifact["caption"] or Path(artifact["path"]).name
    )
    # Escape alt text the way gh does, so a caption cannot close the image early.
    caption = caption.translate(
        str.maketrans({"\\": "\\\\", "[": "\\[", "]": "\\]", "\n": " ", "\r": " "})
    )
    return f"![{caption}]({path})"


def render_section(
    selection: dict[str, Any],
    attach: list[str],
    *,
    environment: str,
    configuration: str,
    limitations: list[str],
    not_proven: list[str],
    reproduce: list[str],
    text_oracle: str,
    visible_observation: str,
    reconciliation: str,
    semantic_verdict: str,
) -> str:
    lines: list[str] = ["## Visual verification", ""]
    lines.append(f"**Environment**: {environment}")
    if configuration:
        lines.append(f"**Configuration**: {configuration}")
    lines.append("")

    lines.append("### Claims and evidence")
    lines.append("")
    claim_map = {claim["id"]: claim for claim in selection["claims"]}
    for claim_id in sorted(claim_map):
        claim = claim_map[claim_id]
        status = "✔" if claim["covered"] else "✘ (no evidence)"
        lines.append(f"- **{claim_id}** — {claim['text']} {status}")
    lines.append("")

    for index, artifact in enumerate(selection["selected"], start=1):
        caption = artifact["caption"] or Path(artifact["path"]).name
        lines.append(f"#### {index}. {caption}")
        lines.append("")
        lines.append(_artifact_reference(artifact, attach[index - 1]))
        lines.append("")
        details = [f"Supports: {', '.join(artifact['claims'])}"]
        if artifact.get("observe"):
            details.append(f"What to look for: {artifact['observe']}")
        details.append(f"Why this artifact: {artifact['rationale']}")
        for detail in details:
            lines.append(f"- {detail}")
        lines.append("")

    lines.append("### Semantic review")
    lines.append("")
    lines.append(f"- **Text oracle**: {text_oracle}")
    lines.append(f"- **Visible observation**: {visible_observation}")
    lines.append(f"- **Reconciliation**: {reconciliation}")
    lines.append(f"- **Verdict**: {semantic_verdict}")
    lines.append("")

    if not_proven:
        lines.append("**What this evidence does not prove**:")
        for item in not_proven:
            lines.append(f"- {item}")
        lines.append("")
    if limitations:
        lines.append("**Limitations**:")
        for item in limitations:
            lines.append(f"- {item}")
        lines.append("")
    if reproduce:
        lines.append("<details><summary>Reproduce</summary>")
        lines.append("")
        lines.append("```bash")
        lines.extend(reproduce)
        lines.append("```")
        lines.append("")
        lines.append("</details>")
        lines.append("")
    return "\n".join(lines).rstrip() + "\n"


def _non_empty(value: str) -> str:
    stripped = value.strip()
    if not stripped:
        raise argparse.ArgumentTypeError("value must not be empty")
    return stripped


def _selection_passes(selection: dict[str, Any]) -> bool:
    claims = selection.get("claims")
    selected = selection.get("selected")
    uncovered = selection.get("uncovered_claims")
    covered_by_artifacts = (
        {
            claim_id
            for artifact in selected
            if isinstance(artifact, dict) and isinstance(artifact.get("claims"), list)
            for claim_id in artifact["claims"]
            if isinstance(claim_id, str)
        }
        if isinstance(selected, list)
        else set()
    )
    return (
        selection.get("pass") is True
        and isinstance(claims, list)
        and bool(claims)
        and all(
            isinstance(claim, dict)
            and claim.get("covered") is True
            and claim.get("id") in covered_by_artifacts
            for claim in claims
        )
        and isinstance(uncovered, list)
        and not uncovered
    )


def _paths_alias(left: Path, right: Path) -> bool:
    if (
        str(left.resolve(strict=False)).casefold()
        == str(right.resolve(strict=False)).casefold()
    ):
        return True
    if left.exists() and right.exists():
        return os.path.samefile(left, right)
    return False


def _inside(path: Path, directory: Path) -> bool:
    child = str(path.resolve(strict=False)).casefold()
    parent = str(directory.resolve(strict=False)).casefold()
    return child == parent or child.startswith(parent.rstrip("/\\") + os.sep)


def _preflight_outputs(
    selection_path: Path,
    selection: dict[str, Any],
    out: Path,
    manifest_out: Path | None,
    attach_dir: Path,
) -> None:
    """Prove output paths are distinct, non-input, and atomically writable."""
    outputs = [out] + ([manifest_out] if manifest_out is not None else [])
    protected = [selection_path]
    for artifact in selection["selected"]:
        artifact_path = Path(artifact["path"])
        if not artifact_path.is_absolute():
            artifact_path = selection_path.parent / artifact_path
        protected.append(artifact_path)

    if manifest_out is not None and _paths_alias(out, manifest_out):
        raise ValueError("--out and --manifest-out must be distinct files")
    if attach_dir.is_symlink() or (attach_dir.exists() and not attach_dir.is_dir()):
        raise ValueError(f"staging path is not a directory: {attach_dir}")
    for path in protected + outputs:
        if _inside(path, attach_dir):
            raise ValueError(
                f"inputs and outputs must stay outside the staging directory: {path}"
            )
    for output in outputs:
        if output.is_symlink():
            raise ValueError(f"output path must not be a symbolic link: {output}")
        if output.exists() and not output.is_file():
            raise ValueError(f"output path is not a regular file: {output}")
        for source in protected:
            if _paths_alias(output, source):
                raise ValueError(
                    f"output path aliases the selection or an artifact: {output}"
                )
        output.parent.mkdir(parents=True, exist_ok=True)
        probe: Path | None = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                prefix=f".{output.name}.probe-",
                dir=output.parent,
                delete=False,
            ) as stream:
                probe = Path(stream.name)
                stream.write("probe\n")
                stream.flush()
                os.fsync(stream.fileno())
        finally:
            if probe is not None:
                probe.unlink(missing_ok=True)


def _atomic_write_text(path: Path, text: str) -> None:
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            prefix=f".{path.name}.tmp-",
            dir=path.parent,
            delete=False,
        ) as stream:
            temporary = Path(stream.name)
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def _selection_digest(selection: dict[str, Any]) -> str:
    canonical = json.dumps(
        selection, ensure_ascii=False, separators=(",", ":"), sort_keys=True
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _build_manifest(
    selection: dict[str, Any],
    *,
    out: Path,
    attach_dir: Path | None,
    attach: list[str],
    publication_pass: bool,
    text_oracle: str,
    visible_observation: str,
    reconciliation: str,
    semantic_verdict: str,
    note: str | None = None,
) -> dict[str, Any]:
    manifest: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "status": "ready" if publication_pass else "not_ready",
        "attach_dir": None if attach_dir is None else str(attach_dir),
        "attach": list(dict.fromkeys(attach)),
        "selection_sha256": _selection_digest(selection),
        "artifacts": [
            {
                "path": artifact["path"],
                "bytes": artifact["bytes"],
                "sha256": artifact["sha256"],
            }
            for artifact in selection["selected"]
        ],
        "section": str(out.resolve()),
        "artifact_count": len(selection["selected"]),
        "semantic_review": {
            "text_oracle": text_oracle,
            "visible_observation": visible_observation,
            "reconciliation": reconciliation,
            "verdict": semantic_verdict,
        },
        "pass": publication_pass,
    }
    if note is not None:
        manifest["note"] = note
    return manifest


def _retire_previous_success(manifest_out: Path | None) -> None:
    """Delete a previous passing publication manifest.

    Only a regular file holding a publication manifest qualifies, so an
    aliased input or a symlink, which preflight rejects, is never touched.
    """
    if manifest_out is None or manifest_out.is_symlink():
        return
    try:
        previous = json.loads(manifest_out.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return
    if (
        isinstance(previous, dict)
        and str(previous.get("schema_version", "")).startswith(
            "dart.evidence_publication/"
        )
        and previous.get("pass") is True
    ):
        manifest_out.unlink(missing_ok=True)


def _clear_staging(attach_dir: Path) -> None:
    """Remove copies a run staged in attach_dir; leave any other file alone."""
    if attach_dir.is_dir():
        for child in attach_dir.iterdir():
            if _STAGED_NAME.fullmatch(child.name):
                child.unlink()


def _write_outputs(
    out: Path,
    manifest_out: Path | None,
    section: str,
    manifest: dict[str, Any],
) -> str:
    """Invalidate the manifest first; commit passing states after the section."""
    text = json.dumps(manifest, indent=2, sort_keys=True)
    if manifest_out is not None:
        if manifest["pass"]:
            manifest_out.unlink(missing_ok=True)
        else:
            _atomic_write_text(manifest_out, text + "\n")
    _atomic_write_text(out, section)
    if manifest_out is not None:
        _atomic_write_text(manifest_out, text + "\n")
    return text


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("selection", type=Path, help="evidence selection JSON")
    parser.add_argument("--environment", required=True, type=_non_empty)
    parser.add_argument("--configuration", default="")
    parser.add_argument("--limitation", action="append", default=[], type=_non_empty)
    parser.add_argument(
        "--not-proven",
        action="append",
        required=True,
        type=_non_empty,
        dest="not_proven",
        help="claim boundary the selected evidence does not establish",
    )
    parser.add_argument("--reproduce", action="append", default=[])
    parser.add_argument(
        "--text-oracle",
        required=True,
        type=_non_empty,
        help="measured state/test result that decides correctness",
    )
    parser.add_argument(
        "--visible-observation",
        required=True,
        type=_non_empty,
        help="facts actually observed after opening the selected images",
    )
    parser.add_argument(
        "--reconciliation",
        required=True,
        type=_non_empty,
        help="how the visible observation agrees or disagrees with the text oracle",
    )
    parser.add_argument(
        "--semantic-verdict",
        choices=("pass", "fail", "uncertain"),
        required=True,
        help="semantic text/image verdict; only pass is publication-ready",
    )
    parser.add_argument("--out", type=Path, required=True, help="markdown output")
    parser.add_argument("--manifest-out", type=Path, help="publication manifest JSON")
    args = parser.parse_args(argv)

    out = args.out.resolve()
    # The section owns its staging directory, so reruns replace its copies.
    staging = out.with_name(out.name + ".attach")
    attach_dir: Path | None = None
    try:
        # A failed rerun must not leave the previous passing manifest behind.
        _retire_previous_success(args.manifest_out)
        selection = _load_selection(args.selection)
        _preflight_outputs(
            args.selection, selection, args.out, args.manifest_out, staging
        )
        _clear_staging(staging)
        selection_pass = _selection_passes(selection)
        semantic_pass = args.semantic_verdict == "pass"
        publication_ready = selection_pass and semantic_pass
        # Identical content shares one name; the first suffix wins.
        names: dict[str, str] = {}
        attach = [
            names.setdefault(
                artifact["sha256"],
                artifact["sha256"] + Path(artifact["path"]).suffix.lower(),
            )
            for artifact in selection["selected"]
        ]
        # Only publishable evidence is staged for gh --attach.
        if publication_ready:
            staging.mkdir(exist_ok=True)
            attach_dir = staging
            for artifact, name in zip(selection["selected"], attach):
                staged_path = attach_dir / name
                shutil.copyfile(args.selection.parent / artifact["path"], staged_path)
                if _sha256(staged_path) != artifact["sha256"]:
                    raise ValueError(
                        f"selected artifact {artifact['path']!r} changed content "
                        "while staging"
                    )
        note: str | None = None

        if not selection_pass:
            uncovered = sorted(
                claim["id"]
                for claim in selection.get("claims", [])
                if not claim.get("covered", False)
            )
            note = (
                "selection is not passing"
                + (f" (uncovered claims: {', '.join(uncovered)})" if uncovered else "")
                + "; the section marks the gaps — do not treat this "
                "publication as complete evidence"
            )
        elif not semantic_pass:
            note = (
                f"semantic review verdict is {args.semantic_verdict}; "
                "do not treat this publication as complete evidence"
            )

        section = render_section(
            selection,
            attach,
            environment=args.environment,
            configuration=args.configuration,
            limitations=args.limitation,
            not_proven=args.not_proven,
            reproduce=args.reproduce,
            text_oracle=args.text_oracle,
            visible_observation=args.visible_observation,
            reconciliation=args.reconciliation,
            semantic_verdict=args.semantic_verdict,
        )
        manifest = _build_manifest(
            selection,
            out=args.out,
            attach_dir=attach_dir,
            attach=attach,
            publication_pass=publication_ready,
            text_oracle=args.text_oracle,
            visible_observation=args.visible_observation,
            reconciliation=args.reconciliation,
            semantic_verdict=args.semantic_verdict,
            note=note,
        )
        text = _write_outputs(args.out, args.manifest_out, section, manifest)
    except (OSError, ValueError, KeyError) as error:
        if attach_dir is not None:
            with contextlib.suppress(OSError):
                _clear_staging(attach_dir)
        print(f"error: {error}", file=sys.stderr)
        return 2
    print(text)
    return 0 if manifest["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
