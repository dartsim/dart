#!/usr/bin/env python3
"""Validate PLAN-081's IPC upstream scene corpus manifest."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from collections import Counter
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = (
    REPO_ROOT
    / "docs"
    / "plans"
    / "081-deformable-implicit-barrier-solver"
    / "ipc_scene_corpus_manifest.json"
)
EXPECTED_COMMIT = "573d2c7e04104d3f9baf526bdaee7745891a571a"
EXPECTED_SCENE_COUNT = 154
EXPECTED_REGULAR_TXT_FILE_COUNT = 144
EXPECTED_SYMLINK_TXT_PATH_COUNT = 10
VALID_TARGET_TYPES = {"test", "benchmark", "example", "manual", "not-applicable"}
VALID_STATUSES = {"planned", "implemented", "manual", "not-applicable"}
EXPECTED_FAMILY_COUNTS = {
    "failure-case": 6,
    "other-bar-twist": 1,
    "other-ccd": 6,
    "other-codimensional-unit": 9,
    "other-friction": 7,
    "other-material-resolution-sweep": 5,
    "other-tunnel": 1,
    "other-typical": 5,
    "paper-erleben": 10,
    "paper-scalability": 9,
    "paper-timestep-variants": 10,
    "paper-top-level": 24,
    "paper-video-examples": 8,
    "supplementB-sqp-benchmark": 30,
    "supplementB-utopia": 1,
    "tutorial": 22,
}
REQUIRED_FIELDS = {
    "upstream_path",
    "upstream_commit",
    "alias_of",
    "family",
    "topic",
    "priority",
    "dart_target_type",
    "status",
    "dart_artifact",
    "required_assets_or_importer",
    "expected_invariant",
    "dart_command_or_ctest_or_benchmark",
    "visual_evidence_requirement",
    "benchmark_profile_artifact",
    "notes_or_gap",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument(
        "--upstream-dir",
        type=Path,
        help="Optional ipc-sim/IPC checkout for path parity validation.",
    )
    return parser.parse_args()


def load_manifest(path: Path) -> dict[str, object]:
    try:
        return json.loads(path.read_text())
    except json.JSONDecodeError as exc:
        raise SystemExit(f"{path}: invalid JSON: {exc}") from exc


def check_required_manifest_fields(
    manifest: dict[str, object], errors: list[str]
) -> None:
    if manifest.get("schema_version") != 1:
        errors.append("schema_version must be 1")

    source = manifest.get("source")
    if not isinstance(source, dict):
        errors.append("source must be an object")
        return

    if source.get("commit") != EXPECTED_COMMIT:
        errors.append(
            f"source.commit must be {EXPECTED_COMMIT}, got {source.get('commit')!r}"
        )
    if source.get("txt_scene_path_count") != EXPECTED_SCENE_COUNT:
        errors.append(
            "source.txt_scene_path_count must be "
            f"{EXPECTED_SCENE_COUNT}, got {source.get('txt_scene_path_count')!r}"
        )
    if source.get("regular_txt_file_count") != EXPECTED_REGULAR_TXT_FILE_COUNT:
        errors.append(
            "source.regular_txt_file_count must be "
            f"{EXPECTED_REGULAR_TXT_FILE_COUNT}, got "
            f"{source.get('regular_txt_file_count')!r}"
        )
    if source.get("symlink_txt_path_count") != EXPECTED_SYMLINK_TXT_PATH_COUNT:
        errors.append(
            "source.symlink_txt_path_count must be "
            f"{EXPECTED_SYMLINK_TXT_PATH_COUNT}, got "
            f"{source.get('symlink_txt_path_count')!r}"
        )


def upstream_head(upstream_dir: Path) -> str:
    result = subprocess.run(
        ["git", "-C", upstream_dir.as_posix(), "rev-parse", "HEAD"],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    return result.stdout.strip()


def tracked_txt_paths(upstream_dir: Path) -> list[str]:
    result = subprocess.run(
        [
            "git",
            "-C",
            upstream_dir.as_posix(),
            "ls-tree",
            "-r",
            "--name-only",
            "HEAD",
            "input",
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    return sorted(line for line in result.stdout.splitlines() if line.endswith(".txt"))


def check_scene_rows(
    manifest: dict[str, object], upstream_dir: Path | None, errors: list[str]
) -> None:
    scenes = manifest.get("scenes")
    if not isinstance(scenes, list):
        errors.append("scenes must be a list")
        return

    if len(scenes) != EXPECTED_SCENE_COUNT:
        errors.append(f"expected {EXPECTED_SCENE_COUNT} scenes, found {len(scenes)}")

    paths: list[str] = []
    family_counts: Counter[str] = Counter()
    target_counts: Counter[str] = Counter()

    for index, row in enumerate(scenes):
        if not isinstance(row, dict):
            errors.append(f"scene {index}: row must be an object")
            continue

        missing = REQUIRED_FIELDS - row.keys()
        if missing:
            errors.append(f"scene {index}: missing fields {sorted(missing)}")

        upstream_path = row.get("upstream_path")
        if not isinstance(upstream_path, str) or not upstream_path.endswith(".txt"):
            errors.append(f"scene {index}: invalid upstream_path {upstream_path!r}")
            continue
        paths.append(upstream_path)

        if row.get("upstream_commit") != EXPECTED_COMMIT:
            errors.append(f"{upstream_path}: unexpected upstream_commit")

        alias_of = row.get("alias_of")
        if not isinstance(alias_of, str):
            errors.append(f"{upstream_path}: alias_of must be a string")

        target_type = row.get("dart_target_type")
        if target_type in {None, "", "unclassified"}:
            errors.append(f"{upstream_path}: dart_target_type is unclassified")
        elif target_type not in VALID_TARGET_TYPES:
            errors.append(f"{upstream_path}: invalid dart_target_type {target_type!r}")

        if row.get("status") not in VALID_STATUSES:
            errors.append(f"{upstream_path}: invalid status {row.get('status')!r}")

        family = row.get("family")
        if not family or family == "unclassified":
            errors.append(f"{upstream_path}: missing family")
        elif family not in EXPECTED_FAMILY_COUNTS:
            errors.append(f"{upstream_path}: unexpected family {family!r}")
        if not row.get("dart_artifact"):
            errors.append(f"{upstream_path}: missing dart_artifact")
        if not row.get("expected_invariant"):
            errors.append(f"{upstream_path}: missing expected_invariant")
        if not row.get("dart_command_or_ctest_or_benchmark"):
            errors.append(f"{upstream_path}: missing command/test/benchmark field")
        if not isinstance(row.get("required_assets_or_importer"), list):
            errors.append(
                f"{upstream_path}: required_assets_or_importer must be a list"
            )
        if not isinstance(row.get("visual_evidence_requirement"), str):
            errors.append(
                f"{upstream_path}: visual_evidence_requirement must be a string"
            )
        if not isinstance(row.get("benchmark_profile_artifact"), str):
            errors.append(
                f"{upstream_path}: benchmark_profile_artifact must be a string"
            )
        if (
            target_type == "benchmark"
            and row.get("benchmark_profile_artifact") == "not-required"
        ):
            errors.append(f"{upstream_path}: benchmark rows need a profile artifact")

        family_counts[str(row.get("family"))] += 1
        target_counts[str(row.get("dart_target_type"))] += 1

    duplicate_paths = sorted(
        path for path, count in Counter(paths).items() if count > 1
    )
    if duplicate_paths:
        errors.append(f"duplicate upstream paths: {duplicate_paths}")

    if sorted(paths) != paths:
        errors.append("scenes must be sorted by upstream_path")

    summary = manifest.get("summary")
    if not isinstance(summary, dict):
        errors.append("summary must be an object")
    else:
        if dict(sorted(family_counts.items())) != EXPECTED_FAMILY_COUNTS:
            errors.append(
                "family counts do not match the audited IPC scene taxonomy: "
                f"{dict(sorted(family_counts.items()))}"
            )
        if summary.get("family_counts") != dict(sorted(family_counts.items())):
            errors.append("summary.family_counts does not match scene rows")
        if summary.get("target_type_counts") != dict(sorted(target_counts.items())):
            errors.append("summary.target_type_counts does not match scene rows")

    if upstream_dir is not None:
        upstream_dir = upstream_dir.resolve()
        commit = upstream_head(upstream_dir)
        if commit != EXPECTED_COMMIT:
            errors.append(f"upstream checkout must be {EXPECTED_COMMIT}, got {commit}")
        upstream_paths = tracked_txt_paths(upstream_dir)
        if paths != upstream_paths:
            missing = sorted(set(upstream_paths) - set(paths))
            extra = sorted(set(paths) - set(upstream_paths))
            errors.append(
                "manifest/upstream path mismatch: "
                f"missing={missing[:5]} extra={extra[:5]}"
            )
        for row in scenes:
            if not isinstance(row, dict):
                continue
            upstream_path = row.get("upstream_path")
            if not isinstance(upstream_path, str):
                continue
            path = upstream_dir / upstream_path
            expected_alias = (
                path.resolve().relative_to(upstream_dir).as_posix()
                if path.is_symlink()
                else ""
            )
            if row.get("alias_of") != expected_alias:
                errors.append(
                    f"{upstream_path}: alias_of must be {expected_alias!r}, "
                    f"got {row.get('alias_of')!r}"
                )


def main() -> int:
    args = parse_args()
    manifest = load_manifest(args.manifest)
    errors: list[str] = []
    check_required_manifest_fields(manifest, errors)
    check_scene_rows(manifest, args.upstream_dir, errors)

    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1

    scene_count = len(manifest.get("scenes", []))
    print(f"IPC scene manifest OK: {scene_count} scenes, zero unclassified rows")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
