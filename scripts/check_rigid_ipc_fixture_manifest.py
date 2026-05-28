#!/usr/bin/env python3
"""Validate PLAN-082's rigid-ipc upstream fixture manifest."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from collections import Counter
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = (
    REPO_ROOT
    / "docs"
    / "plans"
    / "082-rigid-implicit-barrier-contact"
    / "rigid_ipc_fixture_manifest.json"
)
EXPECTED_COMMIT = "23b6ba6fbf8434056444ae106356fd2209136988"
EXPECTED_SOURCE_COUNTS = {
    "fixture_json_path_count": 300,
    "regular_fixture_json_file_count": 270,
    "symlink_fixture_json_path_count": 30,
    "test_data_json_path_count": 405,
    "test_source_path_count": 8,
    "benchmark_script_path_count": 8,
    "comparison_path_count": 77,
}
EXPECTED_ENTRY_COUNT = 798
EXPECTED_SOURCE_KIND_COUNTS = {
    "benchmark-script": 8,
    "comparison": 77,
    "fixture": 300,
    "test-data": 405,
    "test-source": 8,
}
VALID_SOURCE_KINDS = set(EXPECTED_SOURCE_KIND_COUNTS)
VALID_TARGET_TYPES = {"test", "benchmark", "example", "manual"}
VALID_STATUSES = {"planned", "implemented", "manual", "not-applicable"}
REQUIRED_FIELDS = {
    "upstream_path",
    "upstream_commit",
    "source_kind",
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
        help="Optional ipc-sim/rigid-ipc checkout for path parity validation.",
    )
    return parser.parse_args()


def load_manifest(path: Path) -> dict[str, Any]:
    try:
        return json.loads(path.read_text())
    except json.JSONDecodeError as exc:
        raise SystemExit(f"{path}: invalid JSON: {exc}") from exc


def git_lines(upstream_dir: Path, args: list[str]) -> list[str]:
    result = subprocess.run(
        ["git", "-C", upstream_dir.as_posix(), *args],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    return [line for line in result.stdout.splitlines() if line]


def upstream_head(upstream_dir: Path) -> str:
    return git_lines(upstream_dir, ["rev-parse", "HEAD"])[0]


def tracked_paths(upstream_dir: Path, root: str) -> list[str]:
    return sorted(
        git_lines(upstream_dir, ["ls-tree", "-r", "--name-only", "HEAD", root])
    )


def expected_tracked_entries(upstream_dir: Path) -> dict[str, list[str]]:
    benchmark_paths = [
        "tools/benchmark.py",
        "tools/scalability.py",
        "tools/benchmarks/chains.py",
        "tools/benchmarks/codimensional.py",
        "tools/benchmarks/friction.py",
        "tools/benchmarks/mechanisms.py",
        "tools/benchmarks/print_fixtures.py",
        "tools/benchmarks/unit_tests.py",
    ]
    return {
        "fixture": [
            path
            for path in tracked_paths(upstream_dir, "fixtures")
            if path.endswith(".json")
        ],
        "test-data": [
            path
            for path in tracked_paths(upstream_dir, "tests/data")
            if path.endswith(".json")
        ],
        "test-source": [
            path
            for root in ["tests/barrier", "tests/ccd"]
            for path in tracked_paths(upstream_dir, root)
            if path.endswith((".cpp", ".hpp"))
        ],
        "benchmark-script": [
            path for path in benchmark_paths if (upstream_dir / path).exists()
        ],
        "comparison": [
            path
            for path in tracked_paths(upstream_dir, "comparisons")
            if path.endswith((".cpp", ".json", ".py", ".sh", ".xml"))
        ],
    }


def check_required_manifest_fields(
    manifest: dict[str, Any], errors: list[str], strict_counts: bool
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

    if strict_counts:
        for field, expected in EXPECTED_SOURCE_COUNTS.items():
            if source.get(field) != expected:
                errors.append(
                    f"source.{field} must be {expected}, got {source.get(field)!r}"
                )


def check_entries(
    manifest: dict[str, Any],
    upstream_dir: Path | None,
    errors: list[str],
    strict_counts: bool,
) -> None:
    entries = manifest.get("entries")
    if not isinstance(entries, list):
        errors.append("entries must be a list")
        return

    if strict_counts and len(entries) != EXPECTED_ENTRY_COUNT:
        errors.append(f"expected {EXPECTED_ENTRY_COUNT} entries, found {len(entries)}")

    keys: list[tuple[str, str]] = []
    source_kind_counts: Counter[str] = Counter()
    family_counts: Counter[str] = Counter()
    target_counts: Counter[str] = Counter()

    for index, row in enumerate(entries):
        if not isinstance(row, dict):
            errors.append(f"entry {index}: row must be an object")
            continue

        missing = REQUIRED_FIELDS - row.keys()
        if missing:
            errors.append(f"entry {index}: missing fields {sorted(missing)}")

        source_kind = row.get("source_kind")
        if source_kind not in VALID_SOURCE_KINDS:
            errors.append(f"entry {index}: invalid source_kind {source_kind!r}")
            continue

        upstream_path = row.get("upstream_path")
        if not isinstance(upstream_path, str) or not upstream_path:
            errors.append(f"entry {index}: invalid upstream_path {upstream_path!r}")
            continue

        keys.append((source_kind, upstream_path))
        if row.get("upstream_commit") != EXPECTED_COMMIT:
            errors.append(f"{upstream_path}: unexpected upstream_commit")
        if not isinstance(row.get("alias_of"), str):
            errors.append(f"{upstream_path}: alias_of must be a string")

        target_type = row.get("dart_target_type")
        if target_type in {None, "", "unclassified"}:
            errors.append(f"{upstream_path}: dart_target_type is unclassified")
        elif target_type not in VALID_TARGET_TYPES:
            errors.append(f"{upstream_path}: invalid dart_target_type {target_type!r}")

        if row.get("status") not in VALID_STATUSES:
            errors.append(f"{upstream_path}: invalid status {row.get('status')!r}")
        if not row.get("family") or row.get("family") == "unclassified":
            errors.append(f"{upstream_path}: missing family")
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

        source_kind_counts[str(source_kind)] += 1
        family_counts[str(row.get("family"))] += 1
        target_counts[str(target_type)] += 1

    duplicate_keys = sorted(key for key, count in Counter(keys).items() if count > 1)
    if duplicate_keys:
        errors.append(f"duplicate entries: {duplicate_keys[:5]}")
    if sorted(keys) != keys:
        errors.append("entries must be sorted by source_kind and upstream_path")

    if (
        strict_counts
        and dict(sorted(source_kind_counts.items())) != EXPECTED_SOURCE_KIND_COUNTS
    ):
        errors.append(
            "source kind counts do not match audited rigid-ipc taxonomy: "
            f"{dict(sorted(source_kind_counts.items()))}"
        )

    summary = manifest.get("summary")
    if not isinstance(summary, dict):
        errors.append("summary must be an object")
    else:
        if summary.get("source_kind_counts") != dict(
            sorted(source_kind_counts.items())
        ):
            errors.append("summary.source_kind_counts does not match entry rows")
        if summary.get("family_counts") != dict(sorted(family_counts.items())):
            errors.append("summary.family_counts does not match entry rows")
        if summary.get("target_type_counts") != dict(sorted(target_counts.items())):
            errors.append("summary.target_type_counts does not match entry rows")

    if upstream_dir is not None:
        upstream_dir = upstream_dir.resolve()
        commit = upstream_head(upstream_dir)
        if commit != EXPECTED_COMMIT:
            errors.append(f"upstream checkout must be {EXPECTED_COMMIT}, got {commit}")
        expected = expected_tracked_entries(upstream_dir)
        actual: dict[str, list[str]] = {kind: [] for kind in VALID_SOURCE_KINDS}
        for source_kind, upstream_path in keys:
            actual[source_kind].append(upstream_path)
        for source_kind, expected_paths in expected.items():
            actual_paths = sorted(actual[source_kind])
            if actual_paths != sorted(expected_paths):
                missing = sorted(set(expected_paths) - set(actual_paths))
                extra = sorted(set(actual_paths) - set(expected_paths))
                errors.append(
                    f"{source_kind}: manifest/upstream path mismatch: "
                    f"missing={missing[:5]} extra={extra[:5]}"
                )

        row_by_key = {
            (row.get("source_kind"), row.get("upstream_path")): row
            for row in entries
            if isinstance(row, dict)
        }
        for path in expected.get("fixture", []):
            row = row_by_key.get(("fixture", path))
            if not isinstance(row, dict):
                continue
            fixture_path = upstream_dir / path
            expected_alias = (
                fixture_path.resolve().relative_to(upstream_dir).as_posix()
                if fixture_path.is_symlink()
                else ""
            )
            if row.get("alias_of") != expected_alias:
                errors.append(
                    f"{path}: alias_of must be {expected_alias!r}, "
                    f"got {row.get('alias_of')!r}"
                )


def validate_manifest(
    manifest: dict[str, Any],
    upstream_dir: Path | None = None,
    *,
    strict_counts: bool = True,
) -> list[str]:
    errors: list[str] = []
    check_required_manifest_fields(manifest, errors, strict_counts)
    check_entries(manifest, upstream_dir, errors, strict_counts)
    return errors


def main() -> int:
    args = parse_args()
    manifest = load_manifest(args.manifest)
    errors = validate_manifest(manifest, args.upstream_dir)

    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1

    entry_count = len(manifest.get("entries", []))
    print(
        f"Rigid IPC fixture manifest OK: {entry_count} entries, "
        "zero unclassified rows"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
