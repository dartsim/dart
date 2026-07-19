"""Generate a PR-body visual-verification section and publish media to GitHub.

Takes an evidence selection manifest (scripts/evidence_select.py) plus
context fields and produces the "Visual verification" markdown a reviewer
can read without reproducing the environment: what each artifact shows, the
environment/configuration, what to observe, limitations, and reproduction
commands. Media is GitHub-hosted, never committed to the repository.

Backends:
- manual (default): emits the section with UPLOAD-PLACEHOLDER markers plus
  step-by-step instructions for the documented PR web-editor attachment flow
  (the only official way to mint user-attachments URLs; see
  .claude/commands/dart-pr.md). No network access, no approval needed.
- gh-release: uploads assets to a dedicated release tag via the gh CLI and
  embeds the release-asset URLs directly. Mutates shared GitHub state, so it
  requires both --yes and maintainer/user approval per repo policy
  (docs/ai/principles.md, "Shared state needs approval"). Release-asset
  images render inline in PR bodies; videos appear as links (only
  user-attachments get the inline player).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

SCHEMA_VERSION = "dart.evidence_publication/v1"
SELECTION_SCHEMA_VERSION = "dart.evidence_selection/v1"

_IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".gif"}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _source_state() -> dict[str, Any]:
    """Identify the exact publisher source tree used for the publication."""

    root = Path(__file__).resolve().parents[1]
    try:
        revision = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=root,
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
        status = subprocess.run(
            ["git", "status", "--porcelain=v1", "-z", "--untracked-files=all"],
            cwd=root,
            capture_output=True,
            check=True,
        ).stdout
        diff = subprocess.run(
            ["git", "diff", "--binary", "HEAD", "--"],
            cwd=root,
            capture_output=True,
            check=True,
        ).stdout
        untracked = subprocess.run(
            ["git", "ls-files", "--others", "--exclude-standard", "-z"],
            cwd=root,
            capture_output=True,
            check=True,
        ).stdout.split(b"\0")
    except (OSError, subprocess.CalledProcessError):
        return {"revision": None, "dirty": None}
    return {
        "revision": revision,
        "dirty": bool(status),
        "status_sha256": hashlib.sha256(status).hexdigest(),
        "dirty_content_sha256": _dirty_content_digest(root, status, diff, untracked),
        "publisher_sha256": _sha256(Path(__file__).resolve()),
    }


def _dirty_content_digest(
    root: Path, status: bytes, diff: bytes, untracked: list[bytes]
) -> str:
    """Bind the dirty source identity to bytes, not just porcelain path names."""

    digest = hashlib.sha256()
    digest.update(b"status\0" + status)
    digest.update(b"diff\0" + diff)
    for raw_path in sorted(path for path in untracked if path):
        relative = Path(raw_path.decode("utf-8", errors="surrogateescape"))
        path = (root / relative).resolve()
        if root != path and root not in path.parents:
            raise ValueError(f"untracked source path escapes repository: {relative}")
        digest.update(b"untracked\0" + raw_path + b"\0")
        if path.is_file():
            digest.update(bytes.fromhex(_sha256(path)))
        else:
            digest.update(b"not-a-regular-file")
    return digest.hexdigest()


def _resolve_artifact_path(base_dir: Path, value: str) -> Path:
    relative = Path(value)
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError(
            f"artifact path must stay below the selection directory: {value!r}"
        )
    root = base_dir.resolve()
    path = (root / relative).resolve()
    if root not in path.parents:
        raise ValueError(f"artifact path escapes the selection directory: {value!r}")
    if not path.is_file():
        raise ValueError(f"selected artifact does not exist as a file: {value!r}")
    return path


def _artifact_set_digest(selection: dict[str, Any]) -> str:
    """Return a stable digest of the verified path/size/content set."""

    digest = hashlib.sha256()
    for artifact in sorted(selection["selected"], key=lambda item: item["path"]):
        record = f"{artifact['path']}\0{artifact['bytes']}\0{artifact['sha256']}\n"
        digest.update(record.encode("utf-8"))
    return digest.hexdigest()


def _content_addressed_tag(base_tag: str, artifact_digest: str) -> str:
    return f"{base_tag}-{artifact_digest[:16]}"


def _stage_artifacts(
    selection: dict[str, Any], base_dir: Path, stage_dir: Path
) -> dict[str, Path]:
    """Copy verified bytes to an immutable upload set and verify the copies."""

    staged: dict[str, Path] = {}
    stage_dir.mkdir(parents=True, exist_ok=True)
    for artifact in selection["selected"]:
        source = _resolve_artifact_path(base_dir, artifact["path"])
        destination = stage_dir / source.name
        shutil.copyfile(source, destination, follow_symlinks=False)
        if destination.stat().st_size != artifact["bytes"]:
            raise ValueError(f"staged artifact changed size: {artifact['path']!r}")
        digest = _sha256(destination)
        if digest != artifact["sha256"].lower():
            raise ValueError(
                f"staged artifact changed content: {artifact['path']!r}; "
                f"manifest={artifact['sha256']}, staged={digest}"
            )
        destination.chmod(0o444)
        staged[artifact["path"]] = destination
    return staged


def _load_selection(path: Path) -> dict[str, Any]:
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if manifest.get("schema_version") != SELECTION_SCHEMA_VERSION:
        raise ValueError(
            f"selection manifest must declare schema_version="
            f"{SELECTION_SCHEMA_VERSION!r}"
        )
    if not manifest.get("selected"):
        raise ValueError("selection manifest has no selected artifacts")
    # Verify the bytes immediately before publication. The selector's digest is
    # an input claim, not proof that the file has not changed since selection.
    base_dir = path.parent
    basenames: dict[str, str] = {}
    for artifact in manifest["selected"]:
        artifact_path = _resolve_artifact_path(base_dir, artifact["path"])
        expected_size = artifact.get("bytes")
        if isinstance(expected_size, bool) or not isinstance(expected_size, int):
            raise ValueError(
                f"selected artifact {artifact['path']!r} has no integer byte count"
            )
        actual_size = artifact_path.stat().st_size
        if actual_size != expected_size:
            raise ValueError(
                f"selected artifact {artifact['path']!r} changed size: "
                f"manifest={expected_size}, actual={actual_size}"
            )
        expected_sha256 = artifact.get("sha256")
        if not isinstance(expected_sha256, str) or len(expected_sha256) != 64:
            raise ValueError(
                f"selected artifact {artifact['path']!r} has no SHA-256 digest"
            )
        actual_sha256 = _sha256(artifact_path)
        if actual_sha256 != expected_sha256.lower():
            raise ValueError(
                f"selected artifact {artifact['path']!r} changed content: "
                f"manifest={expected_sha256}, actual={actual_sha256}"
            )

        # Release assets are keyed by basename. Duplicates inside one
        # content-addressed release would still collide.
        name = Path(artifact["path"]).name
        if name in basenames:
            raise ValueError(
                f"selected artifacts {basenames[name]!r} and "
                f"{artifact['path']!r} share the basename {name!r}; rename "
                "one before publishing"
            )
        basenames[name] = artifact["path"]
    return manifest


def _gh(args: list[str], *, check: bool = True) -> subprocess.CompletedProcess:
    return subprocess.run(["gh", *args], capture_output=True, text=True, check=check)


def _publish_gh_release(
    selection: dict[str, Any],
    staged_paths: dict[str, Path],
    repo: str,
    tag: str,
) -> dict[str, str]:
    """Upload selected artifacts as release assets; return path->URL map."""
    view = _gh(["release", "view", tag, "--repo", repo], check=False)
    if view.returncode != 0:
        _gh(
            [
                "release",
                "create",
                tag,
                "--repo",
                repo,
                "--title",
                f"Verification media ({tag})",
                "--notes",
                "GitHub-hosted media for PR visual-verification sections. "
                + "Not a software release.",
                "--prerelease",
            ]
        )
    urls: dict[str, str] = {}
    for artifact in selection["selected"]:
        path = staged_paths[artifact["path"]]
        if path.stat().st_size != artifact["bytes"]:
            raise ValueError(f"staged artifact changed size: {artifact['path']!r}")
        digest = _sha256(path)
        if digest != artifact["sha256"].lower():
            raise ValueError(
                f"staged artifact changed before upload: {artifact['path']!r}"
            )
        # The release tag is content-addressed over all verified artifact
        # digests. Clobber is therefore idempotent: the same URL can only be
        # rewritten with the same selected bytes, while changed evidence gets
        # a new tag and cannot mutate media embedded in an older PR.
        _gh(
            [
                "release",
                "upload",
                tag,
                str(path),
                "--repo",
                repo,
                "--clobber",
            ]
        )
        urls[artifact["path"]] = (
            f"https://github.com/{repo}/releases/download/{tag}/{path.name}"
        )
    return urls


def _artifact_reference(artifact: dict[str, Any], url: str | None) -> str:
    name = Path(artifact["path"]).name
    if url is None:
        return (
            f"<!-- UPLOAD-PLACEHOLDER: drag-drop {artifact['path']} into the PR "
            f"web editor here and delete this comment -->\n"
            f"`{name}` (pending upload)"
        )
    suffix = Path(artifact["path"]).suffix.lower()
    if suffix in _IMAGE_SUFFIXES:
        return f"![{artifact['caption'] or name}]({url})"
    return f"[{artifact['caption'] or name}]({url})"


def render_section(
    selection: dict[str, Any],
    urls: dict[str, str],
    *,
    environment: str,
    configuration: str,
    limitations: list[str],
    not_proven: list[str],
    reproduce: list[str],
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
        lines.append(_artifact_reference(artifact, urls.get(artifact["path"])))
        lines.append("")
        details = [f"Supports: {', '.join(artifact['claims'])}"]
        if artifact.get("observe"):
            details.append(f"What to look for: {artifact['observe']}")
        details.append(f"Why this artifact: {artifact['rationale']}")
        for detail in details:
            lines.append(f"- {detail}")
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


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("selection", type=Path, help="evidence selection JSON")
    parser.add_argument("--backend", choices=("manual", "gh-release"), default="manual")
    parser.add_argument("--repo", default="", help="owner/repo for gh-release")
    parser.add_argument(
        "--tag", default="verification-media", help="release tag for gh-release"
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="actually upload (gh-release backend); otherwise dry-run",
    )
    parser.add_argument("--environment", required=True)
    parser.add_argument("--configuration", default="")
    parser.add_argument("--limitation", action="append", default=[])
    parser.add_argument("--not-proven", action="append", default=[], dest="not_proven")
    parser.add_argument("--reproduce", action="append", default=[])
    parser.add_argument("--out", type=Path, required=True, help="markdown output")
    parser.add_argument("--manifest-out", type=Path, help="publication manifest JSON")
    args = parser.parse_args(argv)

    try:
        selection = _load_selection(args.selection)
        artifact_set_sha256 = _artifact_set_digest(selection)
        release_tag = _content_addressed_tag(args.tag, artifact_set_sha256)
        urls: dict[str, str] = {}
        uploaded = False
        if args.backend == "gh-release":
            if not args.repo:
                raise ValueError("--backend gh-release requires --repo owner/repo")
            if args.yes:
                with tempfile.TemporaryDirectory(
                    prefix="dart-evidence-publish-"
                ) as temp:
                    staged_paths = _stage_artifacts(
                        selection, args.selection.parent, Path(temp)
                    )
                    urls = _publish_gh_release(
                        selection, staged_paths, args.repo, release_tag
                    )
                uploaded = True
            else:
                # Dry-run: emit the URLs the upload would produce.
                for artifact in selection["selected"]:
                    name = Path(artifact["path"]).name
                    urls[artifact["path"]] = (
                        f"https://github.com/{args.repo}/releases/download/"
                        f"{release_tag}/{name}"
                    )
        section = render_section(
            selection,
            urls,
            environment=args.environment,
            configuration=args.configuration,
            limitations=args.limitation,
            not_proven=args.not_proven,
            reproduce=args.reproduce,
        )
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(section, encoding="utf-8")
        manifest = {
            "schema_version": SCHEMA_VERSION,
            "backend": args.backend,
            "uploaded": uploaded,
            "base_tag": args.tag if args.backend == "gh-release" else None,
            "release_tag": release_tag if args.backend == "gh-release" else None,
            "selection_manifest_sha256": _sha256(args.selection),
            "source": _source_state(),
            "artifact_set_sha256": artifact_set_sha256,
            "artifacts": [
                {
                    "path": artifact["path"],
                    "bytes": artifact["bytes"],
                    "sha256": artifact["sha256"],
                    "url": urls.get(artifact["path"]),
                }
                for artifact in selection["selected"]
            ],
            "urls": urls,
            "section": str(args.out),
            "artifact_count": len(selection["selected"]),
            "operation_succeeded": True,
            "selection_pass": selection.get("pass") is True,
            "publication_complete": uploaded,
            "pass": uploaded and selection.get("pass") is True,
        }
        if args.backend == "gh-release" and not args.yes:
            manifest["note"] = (
                "dry-run: URLs are predicted, nothing was uploaded; re-run with "
                "--yes after maintainer approval"
            )
    except (
        OSError,
        ValueError,
        KeyError,
        subprocess.CalledProcessError,
    ) as error:
        detail = ""
        if isinstance(error, subprocess.CalledProcessError):
            detail = f": {error.stderr.strip() if error.stderr else error}"
        print(f"error: {error}{detail}", file=sys.stderr)
        return 2
    text = json.dumps(manifest, indent=2, sort_keys=True)
    if args.manifest_out is not None:
        args.manifest_out.parent.mkdir(parents=True, exist_ok=True)
        args.manifest_out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
