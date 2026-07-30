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
import os
import re
import shutil
import subprocess
import sys
import tempfile
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
from urllib.parse import quote, unquote, urlsplit

SCHEMA_VERSION = "dart.evidence_publication/v4"
SELECTION_SCHEMA_VERSION = "dart.evidence_selection/v1"

_IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".gif"}
_ARTIFACT_KINDS = {"still", "grid", "composite", "video"}
_GITHUB_REPOSITORY = re.compile(
    r"[A-Za-z0-9](?:[A-Za-z0-9-]*[A-Za-z0-9])?/[A-Za-z0-9_.-]+"
)


@dataclass
class _PublicationAttempt:
    attempt_id: str
    mutation_started: bool = False
    observed_assets: list[dict[str, Any]] = field(default_factory=list)


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


def _validated_remote_asset_url(
    asset: dict[str, Any],
    artifact: dict[str, Any],
    repo: str,
    tag: str,
) -> str:
    """Return a GitHub-attested release URL after validating its binding."""
    name = _release_asset_name(artifact)
    url = asset.get("url")
    if not isinstance(url, str) or not url:
        raise ValueError(
            f"verified release asset {name!r} has no GitHub-reported download URL"
        )
    parsed = urlsplit(url)
    expected_path = f"/{repo}/releases/download/{tag}/{name}"
    if (
        parsed.scheme != "https"
        or parsed.hostname != "github.com"
        or parsed.username is not None
        or parsed.password is not None
        or parsed.port is not None
        or parsed.query
        or parsed.fragment
        or unquote(parsed.path) != expected_path
    ):
        raise ValueError(
            f"verified release asset {name!r} has an unexpected GitHub-reported "
            "download URL"
        )
    return url


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


def _asset_recovery(name: str) -> str:
    return (
        f"do not clobber or delete {name!r} automatically; retry only after a "
        "maintainer explicitly approves deletion of this exact incomplete asset "
        "or chooses a new release tag"
    )


def _validate_existing_asset(asset: dict[str, Any], artifact: dict[str, Any]) -> None:
    name = _release_asset_name(artifact)
    if type(asset.get("size")) is not int or asset["size"] != artifact["bytes"]:
        raise ValueError(
            f"existing release asset {name!r} does not match the selected byte "
            f"count; {_asset_recovery(name)}"
        )
    digest = asset.get("digest")
    expected_digest = f"sha256:{artifact['sha256']}"
    if digest != expected_digest:
        raise ValueError(
            f"existing release asset {name!r} does not match the selected digest; "
            f"{_asset_recovery(name)}"
        )
    if asset.get("state") != "uploaded":
        raise ValueError(
            f"existing release asset {name!r} is not fully uploaded; "
            f"{_asset_recovery(name)}"
        )


def _observed_selected_assets(
    assets: dict[str, dict[str, Any]],
    selected_assets: dict[str, dict[str, Any]],
) -> list[dict[str, Any]]:
    observed: list[dict[str, Any]] = []
    for name in sorted(selected_assets):
        asset = assets.get(name)
        observed.append(
            {
                "name": name,
                "present": asset is not None,
                "size": asset.get("size") if asset is not None else None,
                "digest": asset.get("digest") if asset is not None else None,
                "state": asset.get("state") if asset is not None else None,
                "url": asset.get("url") if asset is not None else None,
            }
        )
    return observed


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


def _refresh_observed_assets_after_publication_failure(
    view_arguments: list[str],
    selected_assets: dict[str, dict[str, Any]],
    attempt: _PublicationAttempt,
) -> None:
    """Best-effort refresh remote state without masking publication failure."""
    attempt.observed_assets = []
    try:
        view = _gh(view_arguments, check=False)
        if view.returncode != 0:
            return
        _, assets = _parse_release_assets(view)
    except OSError, TypeError, ValueError, subprocess.CalledProcessError:
        return
    attempt.observed_assets = _observed_selected_assets(assets, selected_assets)


def _publish_gh_release(
    selection: dict[str, Any],
    base_dir: Path,
    repo: str,
    tag: str,
    attempt: _PublicationAttempt,
) -> dict[str, str]:
    """Upload selected artifacts and return GitHub-attested path->URL bindings."""
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
            attempt.observed_assets = _observed_selected_assets(
                existing, selected_assets
            )
        elif "release not found" not in f"{view.stdout}\n{view.stderr}".lower():
            raise subprocess.CalledProcessError(
                view.returncode,
                ["gh", *view_arguments],
                output=view.stdout,
                stderr=view.stderr,
            )
        else:
            attempt.observed_assets = _observed_selected_assets({}, selected_assets)

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
            attempt.mutation_started = True
            try:
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
            except OSError, subprocess.CalledProcessError:
                _refresh_observed_assets_after_publication_failure(
                    view_arguments,
                    selected_assets,
                    attempt,
                )
                raise
        if missing:
            # Content-addressed names prevent a later publication from replacing
            # bytes behind an older PR URL. Exact uploaded assets are reusable;
            # same-name incomplete or unverifiable assets deliberately block.
            attempt.mutation_started = True
            try:
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
            except OSError, subprocess.CalledProcessError:
                _refresh_observed_assets_after_publication_failure(
                    view_arguments,
                    selected_assets,
                    attempt,
                )
                raise
        try:
            final_view = _gh(view_arguments)
            _, final_assets = _parse_release_assets(final_view)
        except OSError, TypeError, ValueError, subprocess.CalledProcessError:
            _refresh_observed_assets_after_publication_failure(
                view_arguments,
                selected_assets,
                attempt,
            )
            raise
        attempt.observed_assets = _observed_selected_assets(
            final_assets, selected_assets
        )
        urls: dict[str, str] = {}
        for name, artifact in selected_assets.items():
            if name not in final_assets:
                raise ValueError(
                    f"release verification did not find selected asset {name!r}"
                )
            _validate_existing_asset(final_assets[name], artifact)
            url = _validated_remote_asset_url(final_assets[name], artifact, repo, tag)
            for selected in selection["selected"]:
                if _release_asset_name(selected) == name:
                    urls[selected["path"]] = url
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


def _paths_alias(left: Path, right: Path) -> bool:
    if left.resolve(strict=False) == right.resolve(strict=False):
        return True
    if left.exists() and right.exists():
        return os.path.samefile(left, right)
    return False


def _preflight_outputs(
    selection_path: Path,
    selection: dict[str, Any],
    out: Path,
    manifest_out: Path | None,
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
    backend: str,
    repository: str | None,
    release_tag: str | None,
    out: Path,
    urls: dict[str, str],
    uploaded: bool,
    status: str,
    url_provenance: str | None,
    publication_pass: bool,
    text_oracle: str,
    visible_observation: str,
    reconciliation: str,
    semantic_verdict: str,
    attempt: _PublicationAttempt | None = None,
    note: str | None = None,
    error: str | None = None,
    recovery: str | None = None,
) -> dict[str, Any]:
    manifest: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "backend": backend,
        "repository": repository,
        "release_tag": release_tag,
        "uploaded": uploaded,
        "urls": urls,
        "url_provenance": url_provenance,
        "selection_sha256": _selection_digest(selection),
        "artifacts": [
            {
                "path": artifact["path"],
                "bytes": artifact["bytes"],
                "sha256": artifact["sha256"],
                "release_asset": (
                    _release_asset_name(artifact) if backend == "gh-release" else None
                ),
                "url": urls.get(artifact["path"]),
            }
            for artifact in selection["selected"]
        ],
        "section": str(out),
        "artifact_count": len(selection["selected"]),
        "semantic_review": {
            "text_oracle": text_oracle,
            "visible_observation": visible_observation,
            "reconciliation": reconciliation,
            "verdict": semantic_verdict,
        },
        "attempt_id": attempt.attempt_id if attempt is not None else None,
        "remote_mutation_started": (
            attempt.mutation_started if attempt is not None else False
        ),
        "remote_mutation_possible": (
            status == "publishing"
            or (attempt.mutation_started if attempt is not None else False)
        ),
        "observed_release_assets": (
            attempt.observed_assets if attempt is not None else []
        ),
        "pass": publication_pass,
    }
    if note is not None:
        manifest["note"] = note
    if error is not None:
        manifest["error"] = error
    if recovery is not None:
        manifest["recovery"] = recovery
    return manifest


def _render_publication_state(
    status: str,
    *,
    attempt_id: str,
    error: str | None = None,
    recovery: str | None = None,
) -> str:
    lines = [
        "## Visual verification publication is not ready",
        "",
        f"- **Status**: `{status}`",
        f"- **Attempt**: `{attempt_id}`",
        "- **Publishable evidence**: no",
        "",
        "Do not copy URLs or treat this output as completed verification.",
    ]
    if error is not None:
        lines.extend(["", f"**Error**: {error}"])
    if recovery is not None:
        lines.extend(["", f"**Recovery**: {recovery}"])
    return "\n".join(lines) + "\n"


def _write_outputs(
    out: Path,
    manifest_out: Path | None,
    section: str,
    manifest: dict[str, Any],
) -> str:
    """Atomically replace the section first and the authoritative manifest last."""
    text = json.dumps(manifest, indent=2, sort_keys=True)
    _atomic_write_text(out, section)
    if manifest_out is not None:
        _atomic_write_text(manifest_out, text + "\n")
    return text


def _recovery(repo: str, tag: str) -> str:
    return (
        f"Re-query {repo} release tag {tag!r}. Exact assets in uploaded state with "
        "matching size and SHA-256 may be reused. If a same-name asset is "
        "incomplete, unverifiable, or mismatched, do not delete or clobber it: "
        "obtain explicit maintainer approval to delete only that exact asset or "
        "publish under a new tag."
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

    selection: dict[str, Any] | None = None
    attempt: _PublicationAttempt | None = None
    repository: str | None = None
    release_tag: str | None = None
    try:
        selection = _load_selection(args.selection)
        _preflight_outputs(args.selection, selection, args.out, args.manifest_out)
        selection_pass = _selection_passes(selection)
        semantic_pass = args.semantic_verdict == "pass"
        publication_ready = selection_pass and semantic_pass
        urls: dict[str, str] = {}
        uploaded = False
        status = "not_ready"
        url_provenance: str | None = None
        note: str | None = None

        if args.backend == "gh-release":
            repo = args.repo.strip()
            if not _GITHUB_REPOSITORY.fullmatch(repo):
                raise ValueError("--backend gh-release requires --repo owner/repo")
            repository = repo
            release_tag = args.tag
            if args.yes and publication_ready:
                attempt = _PublicationAttempt(attempt_id=str(uuid.uuid4()))
                recovery = _recovery(repo, args.tag)
                publishing = _build_manifest(
                    selection,
                    backend=args.backend,
                    repository=repository,
                    release_tag=release_tag,
                    out=args.out,
                    urls={},
                    uploaded=False,
                    status="publishing",
                    url_provenance=None,
                    publication_pass=False,
                    text_oracle=args.text_oracle,
                    visible_observation=args.visible_observation,
                    reconciliation=args.reconciliation,
                    semantic_verdict=args.semantic_verdict,
                    attempt=attempt,
                    note=(
                        "A GitHub publication attempt is in progress. This state "
                        "invalidates any prior success output."
                    ),
                    recovery=recovery,
                )
                _write_outputs(
                    args.out,
                    args.manifest_out,
                    _render_publication_state(
                        "publishing",
                        attempt_id=attempt.attempt_id,
                        recovery=recovery,
                    ),
                    publishing,
                )
                urls = _publish_gh_release(
                    selection,
                    args.selection.parent,
                    repo,
                    args.tag,
                    attempt,
                )
                uploaded = True
                status = "published_and_verified"
                url_provenance = "github_release_view"
            elif not args.yes and publication_ready:
                # Dry-run URLs are planning aids, never remote attestations.
                for artifact in selection["selected"]:
                    urls[artifact["path"]] = _release_asset_url(
                        repo, args.tag, artifact
                    )
                status = "dry_run"
                url_provenance = "predicted"
                note = (
                    "dry-run: URLs are predicted, nothing was uploaded; re-run "
                    "with --yes after maintainer approval"
                )
        elif publication_ready:
            status = "manual_ready"

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
        manifest = _build_manifest(
            selection,
            backend=args.backend,
            repository=repository,
            release_tag=release_tag,
            out=args.out,
            urls=urls,
            uploaded=uploaded,
            status=status,
            url_provenance=url_provenance,
            publication_pass=publication_ready,
            text_oracle=args.text_oracle,
            visible_observation=args.visible_observation,
            reconciliation=args.reconciliation,
            semantic_verdict=args.semantic_verdict,
            attempt=attempt,
            note=note,
        )
        text = _write_outputs(args.out, args.manifest_out, section, manifest)
    except (
        OSError,
        ValueError,
        KeyError,
        subprocess.CalledProcessError,
    ) as error:
        error_text = str(error)
        if (
            isinstance(error, subprocess.CalledProcessError)
            and error.stderr
            and error.stderr.strip()
        ):
            error_text = f"{error_text}: {error.stderr.strip()}"
        if selection is not None and attempt is not None:
            status = (
                "partial_or_unverified"
                if attempt.mutation_started
                else "failed_pre_mutation"
            )
            recovery = _recovery(
                repository or args.repo.strip(), release_tag or args.tag
            )
            failure = _build_manifest(
                selection,
                backend=args.backend,
                repository=repository,
                release_tag=release_tag,
                out=args.out,
                urls={},
                uploaded=False,
                status=status,
                url_provenance=None,
                publication_pass=False,
                text_oracle=args.text_oracle,
                visible_observation=args.visible_observation,
                reconciliation=args.reconciliation,
                semantic_verdict=args.semantic_verdict,
                attempt=attempt,
                note=(
                    "Publication did not reach a remotely verified success state. "
                    "Any prior success output remains invalidated."
                ),
                error=error_text,
                recovery=recovery,
            )
            try:
                _write_outputs(
                    args.out,
                    args.manifest_out,
                    _render_publication_state(
                        status,
                        attempt_id=attempt.attempt_id,
                        error=error_text,
                        recovery=recovery,
                    ),
                    failure,
                )
            except OSError as write_error:
                error_text = (
                    f"{error_text}; additionally failed to persist failure state: "
                    f"{write_error}"
                )
        print(f"error: {error_text}", file=sys.stderr)
        return 2
    print(text)
    return 0 if manifest["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
