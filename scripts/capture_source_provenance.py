#!/usr/bin/env python3
"""Deterministic source and artifact provenance for visual captures."""

from __future__ import annotations

import argparse
import json
import os
import struct
import subprocess
import sys
from hashlib import sha256
from pathlib import Path
from typing import Any

CAPTURE_SOURCE_PROVENANCE_ALGORITHM = "sha256-length-prefixed-capture-source-tree-v1"
CAPTURE_ARTIFACT_PROVENANCE_ALGORITHM = "sha256-v1"
CAPTURE_SOURCE_ROOTS = (
    "CMakeLists.txt",
    "cmake",
    "dart",
    "pixi.lock",
    "pixi.toml",
    "pyproject.toml",
    "python/CMakeLists.txt",
    "python/dartpy",
    "python/examples/demos",
    "scripts/capture_py_demo.py",
    "scripts/capture_source_provenance.py",
)


class CaptureSourceProvenanceError(RuntimeError):
    """Raised when the capture source snapshot cannot be resolved."""


def sha256_file(path: Path) -> str:
    digest = sha256()
    try:
        with path.open("rb") as file:
            for chunk in iter(lambda: file.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise CaptureSourceProvenanceError(
            f"cannot hash capture artifact {path}: {exc}"
        ) from exc
    return digest.hexdigest()


def _run_git(repo_root: Path, *args: str) -> bytes:
    try:
        result = subprocess.run(
            ["git", "-C", str(repo_root), *args],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        detail = ""
        if isinstance(exc, subprocess.CalledProcessError):
            detail = exc.stderr.decode("utf-8", errors="replace").strip()
        suffix = f": {detail}" if detail else ""
        raise CaptureSourceProvenanceError(
            f"cannot resolve capture source state with git{suffix}"
        ) from exc
    return result.stdout


def _capture_source_paths(repo_root: Path) -> list[str]:
    output = _run_git(
        repo_root,
        "ls-files",
        "--cached",
        "--others",
        "--exclude-standard",
        "-z",
        "--",
        *CAPTURE_SOURCE_ROOTS,
    )
    paths = sorted({os.fsdecode(encoded) for encoded in output.split(b"\0") if encoded})
    if not paths:
        raise CaptureSourceProvenanceError(
            "capture source state did not resolve any files"
        )
    return paths


def _path_payload(path: Path) -> tuple[bytes, bytes]:
    if path.is_symlink():
        try:
            return b"L", os.fsencode(os.readlink(path))
        except OSError as exc:
            raise CaptureSourceProvenanceError(
                f"cannot read capture source symlink {path}: {exc}"
            ) from exc
    if not path.exists():
        raise CaptureSourceProvenanceError(
            f"capture source path is listed by git but missing from the "
            f"working tree: {path}"
        )
    if not path.is_file():
        raise CaptureSourceProvenanceError(
            f"capture source path is not a regular file: {path}"
        )
    try:
        return b"F", path.read_bytes()
    except OSError as exc:
        raise CaptureSourceProvenanceError(
            f"cannot read capture source file {path}: {exc}"
        ) from exc


def compute_capture_source_provenance(repo_root: Path) -> dict[str, Any]:
    """Hash the source tree that can affect a Python demo capture."""

    repo_root = repo_root.resolve()
    digest = sha256()
    for root in CAPTURE_SOURCE_ROOTS:
        encoded_root = root.encode("utf-8")
        digest.update(struct.pack("<Q", len(encoded_root)))
        digest.update(encoded_root)

    paths = _capture_source_paths(repo_root)
    for relative in paths:
        encoded_path = relative.encode("utf-8")
        kind, payload = _path_payload(repo_root / relative)
        digest.update(struct.pack("<Q", len(encoded_path)))
        digest.update(encoded_path)
        digest.update(kind)
        digest.update(struct.pack("<Q", len(payload)))
        digest.update(payload)

    head = _run_git(repo_root, "rev-parse", "HEAD").decode("ascii").strip()
    if len(head) != 40 or any(
        character not in "0123456789abcdef" for character in head
    ):
        raise CaptureSourceProvenanceError(
            f"git returned an invalid capture source revision: {head!r}"
        )

    return {
        "algorithm": CAPTURE_SOURCE_PROVENANCE_ALGORITHM,
        "digest": digest.hexdigest(),
        "file_count": len(paths),
        "git_head": head,
        "roots": list(CAPTURE_SOURCE_ROOTS),
    }


def capture_artifact_provenance(
    *,
    scene_metrics_events: Path | None,
    screenshot: Path,
) -> dict[str, Any]:
    return {
        "algorithm": CAPTURE_ARTIFACT_PROVENANCE_ALGORITHM,
        "scene_metrics_events_sha256": (
            sha256_file(scene_metrics_events)
            if scene_metrics_events is not None and scene_metrics_events.is_file()
            else None
        ),
        "screenshot_sha256": sha256_file(screenshot),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
    )
    parser.add_argument(
        "--digest-only",
        action="store_true",
        help="print only the capture source digest",
    )
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)
    provenance = compute_capture_source_provenance(args.repo_root)
    if args.digest_only:
        print(provenance["digest"])
    else:
        print(json.dumps(provenance, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
