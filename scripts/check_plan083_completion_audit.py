#!/usr/bin/env python3
"""Validate PLAN-083's completion audit sidecar."""

from __future__ import annotations

import argparse
import json
import re
import sys
from collections import Counter
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
PLAN_DIR = REPO_ROOT / "docs" / "plans" / "083-unified-newton-barrier-multibody"
DEFAULT_AUDIT = PLAN_DIR / "completion-audit.md"
DEFAULT_MANIFEST = PLAN_DIR / "paper-deck-manifest.md"
DEFAULT_CPU_CORPUS = PLAN_DIR / "cpu-scene-corpus.json"
DEFAULT_GPU_PACKET = PLAN_DIR / "gpu-parity-packet.json"

REQUIRED_AUDIT_PHRASES = {
    "Verdict: NOT COMPLETE.",
    "Temporary dev-task folder retired",
    "dev-task folder is retired",
    "Maintainer Decision Required",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit", type=Path, default=DEFAULT_AUDIT)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--cpu-corpus", type=Path, default=DEFAULT_CPU_CORPUS)
    parser.add_argument("--gpu-packet", type=Path, default=DEFAULT_GPU_PACKET)
    return parser.parse_args()


def _load_json(path: Path) -> dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise SystemExit(f"{path}: invalid JSON: {exc}") from exc


def _manifest_status_counts(path: Path) -> Counter[str]:
    counts: Counter[str] = Counter()
    row_pattern = re.compile(r"^\| `[^`]+`\s*\|")
    for line in path.read_text(encoding="utf-8").splitlines():
        if not row_pattern.match(line):
            continue
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        if len(cells) < 7:
            continue
        status = cells[-1].strip()
        if not status or set(status) == {"-"}:
            continue
        counts[status] += 1
    return counts


def _summary_status_counts(
    path: Path, row_key: str
) -> tuple[int, Counter[str], dict[str, Any]]:
    data = _load_json(path)
    rows = data.get(row_key)
    if not isinstance(rows, list):
        raise SystemExit(f"{path}: {row_key} must be a list")
    counts = Counter(str(row.get("status")) for row in rows if isinstance(row, dict))
    return len(rows), counts, data


def _format_counts(counts: Counter[str]) -> str:
    return ", ".join(f"`{key}`: {counts[key]}" for key in sorted(counts))


def _table_row(audit_text: str, first_cell: str) -> list[str] | None:
    for line in audit_text.splitlines():
        if not line.startswith("|"):
            continue
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        if not cells:
            continue
        if cells[0] == first_cell:
            return cells
    return None


def _table_status(audit_text: str, requirement: str) -> str | None:
    cells = _table_row(audit_text, requirement)
    if cells is None or len(cells) < 2:
        return None
    return cells[1]


def _validate_evidence_row(
    audit_text: str,
    artifact: str,
    expected_row_count: int,
    expected_counts: Counter[str],
) -> list[str]:
    cells = _table_row(audit_text, artifact)
    if cells is None or len(cells) < 3:
        return [f"audit evidence row missing for {artifact}"]

    errors: list[str] = []
    if cells[1] != str(expected_row_count):
        errors.append(f"audit {artifact} row count is stale")
    if cells[2] != _format_counts(expected_counts):
        errors.append(f"audit {artifact} status-count row is stale")
    return errors


def validate_audit(
    audit_text: str,
    manifest_counts: Counter[str],
    cpu_counts: Counter[str],
    gpu_counts: Counter[str],
    cpu_row_count: int,
    gpu_row_count: int,
) -> list[str]:
    errors: list[str] = []

    audit_text_lower = audit_text.lower()
    for phrase in sorted(REQUIRED_AUDIT_PHRASES):
        if phrase.lower() not in audit_text_lower:
            errors.append(f"audit missing required phrase: {phrase}")

    if (
        manifest_counts.get("planned", 0) > 0
        and "Verdict: NOT COMPLETE." not in audit_text
    ):
        errors.append("audit must stay NOT COMPLETE while manifest planned rows remain")
    if (
        cpu_counts.get("planned", 0) > 0
        and _table_status(audit_text, "CPU packets exist for performance rows")
        != "Blocked"
    ):
        errors.append(
            "audit must block CPU packet completion while CPU rows remain planned"
        )
    incomplete_gpu_count = gpu_counts.get("planned", 0) + gpu_counts.get(
        "in-progress",
        0,
    )
    if (
        incomplete_gpu_count > 0
        and _table_status(audit_text, "GPU packets exist for GPU claims") != "Blocked"
    ):
        errors.append(
            "audit must block GPU packet completion while GPU rows remain incomplete"
        )

    errors.extend(
        _validate_evidence_row(
            audit_text,
            "`paper-deck-manifest.md`",
            sum(manifest_counts.values()),
            manifest_counts,
        )
    )
    errors.extend(
        _validate_evidence_row(
            audit_text,
            "`cpu-scene-corpus.json`",
            cpu_row_count,
            cpu_counts,
        )
    )
    errors.extend(
        _validate_evidence_row(
            audit_text,
            "`gpu-parity-packet.json`",
            gpu_row_count,
            gpu_counts,
        )
    )

    return errors


def main() -> int:
    args = parse_args()
    audit_text = args.audit.read_text(encoding="utf-8")
    manifest_counts = _manifest_status_counts(args.manifest)
    cpu_row_count, cpu_counts, cpu_data = _summary_status_counts(
        args.cpu_corpus, "scenes"
    )
    gpu_row_count, gpu_counts, gpu_data = _summary_status_counts(
        args.gpu_packet, "rows"
    )

    errors = validate_audit(
        audit_text,
        manifest_counts,
        cpu_counts,
        gpu_counts,
        cpu_row_count,
        gpu_row_count,
    )

    if cpu_data.get("summary", {}).get("status_counts") != dict(
        sorted(cpu_counts.items())
    ):
        errors.append("CPU corpus summary.status_counts does not match scene rows")
    if gpu_data.get("summary", {}).get("status_counts") != dict(
        sorted(gpu_counts.items())
    ):
        errors.append("GPU packet summary.status_counts does not match rows")

    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        return 1

    print(
        "PLAN-083 completion audit OK: "
        f"{sum(manifest_counts.values())} manifest rows, "
        f"{cpu_row_count} CPU corpus rows, {gpu_row_count} GPU packet rows"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
