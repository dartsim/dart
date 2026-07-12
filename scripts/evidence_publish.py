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
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

SCHEMA_VERSION = "dart.evidence_publication/v1"
SELECTION_SCHEMA_VERSION = "dart.evidence_selection/v1"

_IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".gif"}


def _load_selection(path: Path) -> dict[str, Any]:
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if manifest.get("schema_version") != SELECTION_SCHEMA_VERSION:
        raise ValueError(
            f"selection manifest must declare schema_version="
            f"{SELECTION_SCHEMA_VERSION!r}"
        )
    if not manifest.get("selected"):
        raise ValueError("selection manifest has no selected artifacts")
    # Release assets are keyed by basename, so duplicates would silently
    # clobber each other under the shared tag (and collide in placeholders).
    basenames: dict[str, str] = {}
    for artifact in manifest["selected"]:
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
    base_dir: Path,
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
        path = Path(artifact["path"])
        if not path.is_absolute():
            path = base_dir / path
        _gh(
            [
                "release",
                "upload",
                tag,
                str(path),
                "--repo",
                repo,
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
        urls: dict[str, str] = {}
        uploaded = False
        if args.backend == "gh-release":
            if not args.repo:
                raise ValueError("--backend gh-release requires --repo owner/repo")
            if args.yes:
                urls = _publish_gh_release(
                    selection, args.selection.parent, args.repo, args.tag
                )
                uploaded = True
            else:
                # Dry-run: emit the URLs the upload would produce.
                for artifact in selection["selected"]:
                    name = Path(artifact["path"]).name
                    urls[artifact["path"]] = (
                        f"https://github.com/{args.repo}/releases/download/"
                        f"{args.tag}/{name}"
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
        # The publication inherits the selection's verdict: a non-passing
        # selection still renders its section honestly (uncovered claims show
        # as "✘ (no evidence)"), but automation must not read the publication
        # as ready — the manifest fails and the CLI exits non-zero.
        selection_pass = bool(selection.get("pass", False))
        manifest = {
            "schema_version": SCHEMA_VERSION,
            "backend": args.backend,
            "uploaded": uploaded,
            "urls": urls,
            "section": str(args.out),
            "artifact_count": len(selection["selected"]),
            "pass": selection_pass,
        }
        if not selection_pass:
            uncovered = sorted(
                claim["id"]
                for claim in selection.get("claims", [])
                if not claim.get("covered", False)
            )
            manifest["note"] = (
                "selection is not passing"
                + (f" (uncovered claims: {', '.join(uncovered)})" if uncovered else "")
                + "; the section marks the gaps — do not treat this "
                "publication as complete evidence"
            )
        elif args.backend == "gh-release" and not args.yes:
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
    # Non-zero for a rendered-but-failing publication (uncovered claims) so
    # automation cannot mistake incomplete evidence for success; hard errors
    # above return 2.
    return 0 if manifest["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
