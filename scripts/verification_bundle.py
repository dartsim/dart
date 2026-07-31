#!/usr/bin/env python3
"""Package DART scene/simulation evidence for agent or VLM review."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
import tempfile
from pathlib import Path
from typing import Any

SCHEMA_VERSION = "dart.verification_bundle/v1"
GENERATED_ARTIFACT_NAMES = frozenset({"manifest.json", "vlm_prompt.md"})


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


def _reserve_artifact_name(name: str, artifact_names: set[str]) -> None:
    if name in GENERATED_ARTIFACT_NAMES:
        raise ValueError(f"reserved artifact filename {name!r}")
    if name in artifact_names:
        raise ValueError(f"duplicate artifact filename {name!r}")
    artifact_names.add(name)


def _prompt(question: str, artifacts: list[dict[str, Any]]) -> str:
    lines = [
        "# DART Scene Verification Bundle",
        "",
        "Use the text artifacts as the primary oracle. Use images only to "
        "corroborate scene layout, gross motion, and obvious rendering or "
        "contact failures. Do not decide static geometry correctness from an "
        "image alone. A view report or passing image-verdict is a machine check, "
        "not semantic visual review.",
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
            + "poses, limits, and missing collision geometry.",
            "2. Read metrics and trajectories for energy, momentum, contacts, "
            + "penetration, determinism, and first divergence.",
            "3. State the visible observation expected if the claim is true, "
            + "then actually open each selected image. Use original/full detail "
            + "when contacts, labels, bounds, or axes are small.",
            "4. Describe only visible facts tied to the question. Treat a poor "
            + "or ambiguous view as a reason to reframe, not as negative physics "
            + "evidence.",
            "5. Reconcile visual observations with the text oracle. Any "
            + "unexplained disagreement is fail/uncertain, never an averaged pass.",
            "6. Report under: Text oracle; Visible observation; Reconciliation; "
            + "Verdict (pass/fail/uncertain); Not proven and limitations. Cite the "
            + "artifact and field supporting each claim.",
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
    inputs = [
        *((path, "text-primary") for path in text),
        (image, "image-still"),
    ]
    if grid is not None:
        inputs.append((grid, "image-grid"))
    artifact_names: set[str] = set()
    for path, _ in inputs:
        _reserve_artifact_name(path.name, artifact_names)
        if not path.is_file():
            raise ValueError(f"{path}: artifact does not exist")

    out_dir.parent.mkdir(parents=True, exist_ok=True)
    if out_dir.exists() and not out_dir.is_dir():
        raise ValueError(f"{out_dir}: bundle output is not a directory")

    with tempfile.TemporaryDirectory(
        prefix=f".{out_dir.name}.staging-",
        dir=out_dir.parent,
    ) as temp_dir:
        staging_dir = Path(temp_dir) / "bundle"
        staging_dir.mkdir()
        manifest = _build_staged_bundle(
            staging_dir=staging_dir,
            question=question,
            inputs=inputs,
            metadata=metadata,
        )
        previous_dir = Path(temp_dir) / "previous"
        if out_dir.exists():
            os.replace(out_dir, previous_dir)
        try:
            os.replace(staging_dir, out_dir)
        except OSError:
            if previous_dir.exists():
                os.replace(previous_dir, out_dir)
            raise
    return manifest


def _build_staged_bundle(
    *,
    staging_dir: Path,
    question: str,
    inputs: list[tuple[Path, str]],
    metadata: dict[str, str] | None,
) -> dict[str, Any]:
    artifacts: list[dict[str, Any]] = []
    for path, role in inputs:
        artifacts.append(_copy_artifact(path, staging_dir, role))

    prompt = _prompt(question, artifacts)
    prompt_path = staging_dir / "vlm_prompt.md"
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
    (staging_dir / "manifest.json").write_text(
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
