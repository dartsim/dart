"""Generate a PR-body visual-verification section and publish media to GitHub.

Takes an evidence selection manifest (scripts/evidence_select.py) plus
context fields and produces the "Visual verification" markdown a reviewer
can read without reproducing the environment: what each artifact shows, the
text oracle, semantic image observation, reconciliation/verdict, explicit
claim boundary, environment/configuration, limitations, and reproduction
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
import math
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any
from urllib.parse import quote

SCHEMA_VERSION = "dart.evidence_publication/v3"
SELECTION_SCHEMA_VERSION = "dart.evidence_selection/v1"

_IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".gif"}
_ARTIFACT_KINDS = {"still", "grid", "composite", "video"}
_GITHUB_REPOSITORY = re.compile(
    r"[A-Za-z0-9](?:[A-Za-z0-9-]*[A-Za-z0-9])?/[A-Za-z0-9_.-]+"
)


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
    selected_paths: set[str] = set()
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
        if declared_path in selected_paths:
            raise ValueError(f"selected artifact path {declared_path!r} is duplicated")
        selected_paths.add(declared_path)

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
        if not artifact_path.is_file():
            raise ValueError(
                f"selected artifact is missing or not a regular file: {artifact_path}"
            )
        actual_bytes = artifact_path.stat().st_size
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


def _gh(args: list[str], *, check: bool = True) -> subprocess.CompletedProcess:
    return subprocess.run(["gh", *args], capture_output=True, text=True, check=check)


def _release_asset_name(artifact: dict[str, Any]) -> str:
    """Return an immutable, content-addressed release-asset name."""
    suffix = Path(artifact["path"]).suffix.lower()
    return f"sha256-{artifact['sha256']}{suffix}"


def _release_asset_url(repo: str, tag: str, artifact: dict[str, Any]) -> str:
    tag_path = quote(tag, safe="")
    asset_path = quote(_release_asset_name(artifact), safe="")
    return f"https://github.com/{repo}/releases/download/{tag_path}/{asset_path}"


def _parse_release_assets(
    view: subprocess.CompletedProcess,
) -> tuple[bool, dict[str, dict[str, Any]]]:
    payload = json.loads(view.stdout)
    if not isinstance(payload, dict):
        raise ValueError("gh release view returned a non-object JSON value")
    if type(payload.get("isImmutable")) is not bool:
        raise ValueError("gh release view omitted boolean isImmutable")
    assets = payload.get("assets")
    if not isinstance(assets, list):
        raise ValueError("gh release view omitted its assets list")
    by_name: dict[str, dict[str, Any]] = {}
    for asset in assets:
        if not isinstance(asset, dict):
            raise ValueError("gh release view returned a non-object asset")
        name = asset.get("name")
        if not isinstance(name, str) or not name:
            raise ValueError("gh release view returned an asset without a name")
        if name in by_name:
            raise ValueError(f"release contains duplicate asset name {name!r}")
        by_name[name] = asset
    return payload["isImmutable"], by_name


def _validate_existing_asset(asset: dict[str, Any], artifact: dict[str, Any]) -> None:
    name = _release_asset_name(artifact)
    if type(asset.get("size")) is not int or asset["size"] != artifact["bytes"]:
        raise ValueError(
            f"existing release asset {name!r} does not match the selected byte count"
        )
    digest = asset.get("digest")
    expected_digest = f"sha256:{artifact['sha256']}"
    if digest != expected_digest:
        raise ValueError(
            f"existing release asset {name!r} does not match the selected digest"
        )
    if asset.get("state") != "uploaded":
        raise ValueError(f"existing release asset {name!r} is not fully uploaded")


def _stage_release_assets(
    artifacts: list[dict[str, Any]],
    base_dir: Path,
    stage_dir: Path,
) -> dict[str, Path]:
    """Freeze and revalidate every selected asset before any GitHub action."""
    staged: dict[str, Path] = {}
    for index, artifact in enumerate(artifacts):
        name = _release_asset_name(artifact)
        source = Path(artifact["path"])
        if not source.is_absolute():
            source = base_dir / source
        target = (
            stage_dir / name
            if name not in staged
            else stage_dir / f"duplicate-{index:04d}-{name}"
        )
        shutil.copyfile(source, target)
        if (
            target.stat().st_size != artifact["bytes"]
            or _sha256(target) != artifact["sha256"]
        ):
            raise ValueError(
                f"selected artifact {artifact['path']!r} changed while staging"
            )
        staged.setdefault(name, target)
    return staged


def _publish_gh_release(
    selection: dict[str, Any],
    base_dir: Path,
    repo: str,
    tag: str,
) -> dict[str, str]:
    """Upload selected artifacts as release assets; return path->URL map."""
    urls = {
        artifact["path"]: _release_asset_url(repo, tag, artifact)
        for artifact in selection["selected"]
    }
    selected_assets: dict[str, dict[str, Any]] = {}
    for artifact in selection["selected"]:
        name = _release_asset_name(artifact)
        previous = selected_assets.setdefault(name, artifact)
        if (
            previous["sha256"] != artifact["sha256"]
            or previous["bytes"] != artifact["bytes"]
        ):
            raise ValueError(
                f"selected artifacts collide on release asset name {name!r}"
            )

    with tempfile.TemporaryDirectory(prefix="dart-evidence-publish-") as temp_dir:
        staged = _stage_release_assets(
            selection["selected"],
            base_dir,
            Path(temp_dir),
        )
        view_arguments = [
            "release",
            "view",
            tag,
            "--repo",
            repo,
            "--json",
            "assets,isImmutable",
        ]
        view = _gh(view_arguments, check=False)
        release_exists = view.returncode == 0
        immutable = False
        existing: dict[str, dict[str, Any]] = {}
        if release_exists:
            immutable, existing = _parse_release_assets(view)
        elif "release not found" not in f"{view.stdout}\n{view.stderr}".lower():
            raise subprocess.CalledProcessError(
                view.returncode,
                ["gh", *view_arguments],
                output=view.stdout,
                stderr=view.stderr,
            )

        missing: list[str] = []
        for name, artifact in selected_assets.items():
            if name in existing:
                _validate_existing_asset(existing[name], artifact)
            else:
                missing.append(name)
        if immutable and missing:
            raise ValueError(
                f"release tag {tag!r} is immutable and cannot accept "
                "missing evidence assets"
            )

        if not release_exists:
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
        if missing:
            # Content-addressed names make retries idempotent and prevent a
            # later publication from replacing bytes behind an older PR URL.
            _gh(
                [
                    "release",
                    "upload",
                    tag,
                    *(str(staged[name]) for name in missing),
                    "--repo",
                    repo,
                ]
            )
        final_view = _gh(view_arguments)
        _, final_assets = _parse_release_assets(final_view)
        for name, artifact in selected_assets.items():
            if name not in final_assets:
                raise ValueError(
                    f"release verification did not find selected asset {name!r}"
                )
            _validate_existing_asset(final_assets[name], artifact)
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
        lines.append(_artifact_reference(artifact, urls.get(artifact["path"])))
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


def _release_tag(value: str) -> str:
    """Return a conservative GitHub release tag that cannot become a CLI option."""
    tag = _non_empty(value)
    forbidden = set(" ~^:?*[\\")
    components = tag.split("/")
    if (
        len(tag) > 200
        or tag.startswith("-")
        or tag == "@"
        or tag.endswith(("/", "."))
        or ".." in tag
        or "@{" in tag
        or "//" in tag
        or any(ord(character) < 32 or ord(character) == 127 for character in tag)
        or any(character in forbidden for character in tag)
        or any(
            not component or component.startswith(".") or component.endswith(".lock")
            for component in components
        )
    ):
        raise argparse.ArgumentTypeError("value must be a safe GitHub release tag")
    return tag


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


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("selection", type=Path, help="evidence selection JSON")
    parser.add_argument("--backend", choices=("manual", "gh-release"), default="manual")
    parser.add_argument("--repo", default="", help="owner/repo for gh-release")
    parser.add_argument(
        "--tag",
        default="verification-media",
        type=_release_tag,
        help="release tag for gh-release",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="actually upload (gh-release backend); otherwise dry-run",
    )
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

    try:
        selection = _load_selection(args.selection)
        selection_pass = _selection_passes(selection)
        semantic_pass = args.semantic_verdict == "pass"
        publication_ready = selection_pass and semantic_pass
        urls: dict[str, str] = {}
        uploaded = False
        repository: str | None = None
        release_tag: str | None = None
        if args.backend == "gh-release":
            repo = args.repo.strip()
            if not _GITHUB_REPOSITORY.fullmatch(repo):
                raise ValueError("--backend gh-release requires --repo owner/repo")
            repository = repo
            release_tag = args.tag
            if args.yes:
                if publication_ready:
                    urls = _publish_gh_release(
                        selection, args.selection.parent, repo, args.tag
                    )
                    uploaded = True
            elif publication_ready:
                # Dry-run: emit the URLs the upload would produce.
                for artifact in selection["selected"]:
                    urls[artifact["path"]] = _release_asset_url(
                        repo, args.tag, artifact
                    )
        section = render_section(
            selection,
            urls,
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
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(section, encoding="utf-8")
        manifest = {
            "schema_version": SCHEMA_VERSION,
            "backend": args.backend,
            "repository": repository,
            "release_tag": release_tag,
            "uploaded": uploaded,
            "urls": urls,
            "artifacts": [
                {
                    "path": artifact["path"],
                    "bytes": artifact["bytes"],
                    "sha256": artifact["sha256"],
                    "release_asset": (
                        _release_asset_name(artifact)
                        if args.backend == "gh-release"
                        else None
                    ),
                    "url": urls.get(artifact["path"]),
                }
                for artifact in selection["selected"]
            ],
            "section": str(args.out),
            "artifact_count": len(selection["selected"]),
            "semantic_review": {
                "text_oracle": args.text_oracle,
                "visible_observation": args.visible_observation,
                "reconciliation": args.reconciliation,
                "verdict": args.semantic_verdict,
            },
            "pass": publication_ready,
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
        elif not semantic_pass:
            manifest["note"] = (
                f"semantic review verdict is {args.semantic_verdict}; "
                "do not treat this publication as complete evidence"
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
    return 0 if manifest["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
