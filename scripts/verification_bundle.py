#!/usr/bin/env python3
"""Package DART scene/simulation evidence for agent or VLM review."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
from pathlib import Path
from typing import Any

SCHEMA_VERSION = "dart.verification_bundle/v1"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _copy_artifact(source: Path, destination_dir: Path, role: str) -> dict[str, Any]:
    if not source.is_file():
        raise ValueError(f"{source}: artifact does not exist")
    target = destination_dir / source.name
    if source.resolve() != target.resolve():
        shutil.copy2(source, target)
    return {
        "role": role,
        "path": target.name,
        "bytes": target.stat().st_size,
        "sha256": _sha256(target),
    }


def _prompt(question: str, artifacts: list[dict[str, Any]]) -> str:
    lines = [
        "# DART Scene Verification Bundle",
        "",
        "Use the text artifacts as the primary oracle. Use images only to "
        "corroborate scene layout, gross motion, and obvious rendering or "
        "contact failures. Do not decide static geometry correctness from an "
        "image alone.",
        "",
        f"Question: {question}",
        "",
        "Artifacts:",
    ]
    for artifact in artifacts:
        lines.append(
            f"- `{artifact['path']}` ({artifact['role']}, "
            f"{artifact['bytes']} bytes, sha256={artifact['sha256']})"
        )
    lines.extend(
        [
            "",
            "Review checklist:",
            "",
            "1. Read scene JSON/text for units, gravity, bodies, joints, shapes, "
            "poses, limits, and missing collision geometry.",
            "2. Read metrics and trajectories for energy, momentum, contacts, "
            "penetration, determinism, and first divergence.",
            "3. Inspect the still frame and optional grid for gross scene or "
            "motion contradictions with the text evidence.",
            "4. Report pass/fail/uncertain with the artifact and field that "
            "supports each claim.",
            "",
        ]
    )
    return "\n".join(lines)


def build_bundle(
    *,
    out_dir: Path,
    question: str,
    text: list[Path],
    image: Path,
    grid: Path | None = None,
    metadata: dict[str, str] | None = None,
) -> dict[str, Any]:
    out_dir.mkdir(parents=True, exist_ok=True)
    artifacts: list[dict[str, Any]] = []
    artifact_names: set[str] = set()
    for path in text:
        if path.name in artifact_names:
            raise ValueError(f"duplicate artifact filename {path.name!r}")
        artifact_names.add(path.name)
        artifacts.append(_copy_artifact(path, out_dir, "text-primary"))
    if image.name in artifact_names:
        raise ValueError(f"duplicate artifact filename {image.name!r}")
    artifact_names.add(image.name)
    artifacts.append(_copy_artifact(image, out_dir, "image-still"))
    if grid is not None:
        if grid.name in artifact_names:
            raise ValueError(f"duplicate artifact filename {grid.name!r}")
        artifact_names.add(grid.name)
        artifacts.append(_copy_artifact(grid, out_dir, "image-grid"))

    prompt = _prompt(question, artifacts)
    prompt_path = out_dir / "vlm_prompt.md"
    prompt_path.write_text(prompt, encoding="utf-8")
    artifacts.append(
        {
            "role": "review-prompt",
            "path": prompt_path.name,
            "bytes": prompt_path.stat().st_size,
            "sha256": _sha256(prompt_path),
        }
    )

    manifest = {
        "schema_version": SCHEMA_VERSION,
        "question": question,
        "metadata": metadata or {},
        "artifacts": artifacts,
    }
    (out_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return manifest


def _metadata(values: list[str]) -> dict[str, str]:
    metadata: dict[str, str] = {}
    for value in values:
        if "=" not in value:
            raise ValueError(f"metadata must be KEY=VALUE, got {value!r}")
        key, item = value.split("=", 1)
        if not key:
            raise ValueError("metadata keys must be non-empty")
        metadata[key] = item
    return metadata


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--question", required=True)
    parser.add_argument(
        "--text",
        type=Path,
        action="append",
        required=True,
        help="primary text artifact: scene JSON/text, metrics, trajectory TSV, or contacts JSONL",
    )
    parser.add_argument("--image", type=Path, required=True, help="single still frame")
    parser.add_argument("--grid", type=Path, help="optional multi-view or motion grid")
    parser.add_argument("--metadata", action="append", default=[], metavar="KEY=VALUE")
    args = parser.parse_args(argv)

    try:
        manifest = build_bundle(
            out_dir=args.out,
            question=args.question,
            text=args.text,
            image=args.image,
            grid=args.grid,
            metadata=_metadata(args.metadata),
        )
    except (OSError, ValueError) as exc:
        print(f"verification_bundle.py: {exc}", file=sys.stderr)
        return 2

    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
