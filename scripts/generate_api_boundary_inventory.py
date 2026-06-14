#!/usr/bin/env python3
"""Generate the public API boundary inventory document."""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
OUTPUT_PATH = REPO_ROOT / "build" / "reports" / "api-boundary-inventory.md"
ALLOWLIST_PATH = REPO_ROOT / "scripts" / "check_api_boundaries_allowlist.txt"

CXX_API_ROOT = REPO_ROOT / "dart"
PYTHON_BINDING_ROOT = REPO_ROOT / "python" / "dartpy"
HEADER_SUFFIXES = {".h", ".hpp"}
SOURCE_SUFFIXES = {".cpp", ".h", ".hpp"}

PRIVATE_INCLUDE_PATTERN = re.compile(
    r"^\s*#\s*include\s*[<\"]dart/[^>\"]*/(?:detail|internal)/[^>\"]+[>\"]"
)
PRIVATE_REFERENCE_PATTERN = re.compile(r"\b(?:detail|internal)::")
COMPATIBILITY_PATTERN = re.compile(
    r"\b(?:DART_DEPRECATED|deprecated|compatibility|backward compatible|gz-physics)\b",
    re.IGNORECASE,
)
CLASS_BINDING_PATTERN = re.compile(r'nb::class_<[^>]+>\s*\([^,]+,\s*"([^"]+)"')
MODULE_FUNCTION_PATTERN = re.compile(r'\b(?:m|sm)\.def\(\s*"([^"]+)"')


@dataclass(frozen=True)
class CxxHeaderInventory:
    path: str
    module: str
    classification: str
    private_include_count: int
    private_reference_count: int
    compatibility_count: int
    samples: tuple[str, ...]


@dataclass(frozen=True)
class PythonInventory:
    path: str
    classification: str
    class_names: tuple[str, ...]
    module_function_count: int
    allowlist_entries: tuple[str, ...]


def rel_path(path: Path) -> str:
    return path.relative_to(REPO_ROOT).as_posix()


def module_name(path: Path) -> str:
    parts = path.relative_to(REPO_ROOT).parts
    if parts[0] == "dart":
        if len(parts) == 2:
            return "top-level"
        if parts[1] == "simulation" and len(parts) >= 3 and parts[2] == "experimental":
            return "simulation/experimental"
        return parts[1]
    return "unknown"


def is_public_header(path: Path) -> bool:
    rel_parts = path.relative_to(REPO_ROOT).parts
    if path.suffix not in HEADER_SUFFIXES:
        return False
    if "detail" in rel_parts or "internal" in rel_parts:
        return False
    if rel_parts[-1].endswith("-impl.hpp") or rel_parts[-1].endswith("_impl.hpp"):
        return False
    return rel_parts[:4] != ("dart", "simulation", "experimental", "comps")


def iter_public_headers() -> list[Path]:
    return sorted(
        path
        for path in CXX_API_ROOT.rglob("*")
        if path.is_file() and is_public_header(path)
    )


def summarize_samples(lines: list[str]) -> tuple[str, ...]:
    samples: list[str] = []
    for line in lines:
        stripped = line.strip()
        if not stripped:
            continue
        if PRIVATE_INCLUDE_PATTERN.search(line) or PRIVATE_REFERENCE_PATTERN.search(
            line
        ):
            samples.append(re.sub(r"\s+", " ", stripped))
        if len(samples) == 2:
            break
    return tuple(samples)


def format_sample_cell(samples: tuple[str, ...]) -> str:
    if not samples:
        return "n/a"

    def code_span(sample: str) -> str:
        escaped = sample.replace("|", "\\|").replace("`", "\\`")
        return f"`{escaped}`"

    return "<br>".join(code_span(sample) for sample in samples)


def classify_header(path: Path) -> CxxHeaderInventory:
    lines = path.read_text(errors="replace").splitlines()
    private_include_count = sum(
        1 for line in lines if PRIVATE_INCLUDE_PATTERN.search(line)
    )
    private_reference_count = sum(
        1 for line in lines if PRIVATE_REFERENCE_PATTERN.search(line)
    )
    compatibility_count = sum(1 for line in lines if COMPATIBILITY_PATTERN.search(line))

    if private_include_count or private_reference_count:
        classification = "exposed-implementation debt"
    elif compatibility_count:
        classification = "compatibility"
    elif module_name(path) == "simulation/experimental":
        classification = "experimental"
    else:
        classification = "supported public"

    return CxxHeaderInventory(
        path=rel_path(path),
        module=module_name(path),
        classification=classification,
        private_include_count=private_include_count,
        private_reference_count=private_reference_count,
        compatibility_count=compatibility_count,
        samples=summarize_samples(lines),
    )


def load_allowlist_by_path() -> dict[str, list[str]]:
    entries_by_path: dict[str, list[str]] = {}
    for raw_line in ALLOWLIST_PATH.read_text().splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split("|", 6)
        if len(parts) != 7:
            continue
        check_id, path, substring, replacement, remove_by, tracking, reason = (
            part.strip() for part in parts
        )
        entries_by_path.setdefault(path, []).append(
            f"{check_id}: {substring}; replacement={replacement}; remove_by={remove_by}; tracking={tracking}; reason={reason}"
        )
    return entries_by_path


def iter_python_sources() -> list[Path]:
    return sorted(
        path
        for path in PYTHON_BINDING_ROOT.rglob("*")
        if path.is_file() and path.suffix in SOURCE_SUFFIXES
    )


def classify_python_source(
    path: Path, allowlist_by_path: dict[str, list[str]]
) -> PythonInventory:
    text = path.read_text(errors="replace")
    path_text = rel_path(path)
    class_names = tuple(sorted(set(CLASS_BINDING_PATTERN.findall(text))))
    module_function_count = len(MODULE_FUNCTION_PATTERN.findall(text))
    allowlist_entries = tuple(allowlist_by_path.get(path_text, ()))
    classification = "compatibility/debt" if allowlist_entries else "supported public"
    return PythonInventory(
        path=path_text,
        classification=classification,
        class_names=class_names,
        module_function_count=module_function_count,
        allowlist_entries=allowlist_entries,
    )


def append_table(
    lines: list[str], headers: tuple[str, ...], rows: list[tuple[str, ...]]
) -> None:
    normalized_rows = [
        tuple(cell.replace("\n", "<br>").replace("__", "\\_\\_") for cell in row)
        for row in rows
    ]
    widths = [
        max(
            3,
            len(header),
            *(len(row[column_index]) for row in normalized_rows),
        )
        for column_index, header in enumerate(headers)
    ]

    def render_row(row: tuple[str, ...]) -> str:
        return (
            "| "
            + " | ".join(
                cell.ljust(widths[column_index])
                for column_index, cell in enumerate(row)
            )
            + " |"
        )

    lines.append(render_row(headers))
    lines.append(render_row(tuple("-" * width for width in widths)))
    for row in normalized_rows:
        lines.append(render_row(row))
    lines.append("")


def render_inventory() -> str:
    cxx_entries = [classify_header(path) for path in iter_public_headers()]
    allowlist_by_path = load_allowlist_by_path()
    python_entries = [
        classify_python_source(path, allowlist_by_path)
        for path in iter_python_sources()
    ]

    lines: list[str] = [
        "<!-- Generated by scripts/generate_api_boundary_inventory.py; do not edit by hand. -->",
        "",
        "# API Boundary Inventory",
        "",
        "This on-demand report summarizes the current public API boundary signals used",
        "by the policy in [api-boundaries.md](api-boundaries.md). It is generated on",
        "demand and not committed; produce it with `pixi run report-api-boundary-inventory`.",
        "The enforced source of truth is scripts/check_api_boundaries.py and",
        "scripts/check_api_boundaries_allowlist.txt (run `pixi run check-api-boundaries`).",
        "",
        "The inventory is intentionally signal-based. A header can be installed and still",
        "contain exposed implementation debt if public declarations mention `detail` or",
        "`internal` implementation types. Compatibility entries stay public until their",
        "documented downstream migration or removal condition is satisfied.",
        "",
        "## Summary",
        "",
    ]

    cxx_debt = [
        entry
        for entry in cxx_entries
        if entry.classification == "exposed-implementation debt"
    ]
    cxx_compat = [entry for entry in cxx_entries if entry.compatibility_count]
    python_debt = [entry for entry in python_entries if entry.allowlist_entries]
    lines.extend(
        [
            f"- C++ public headers scanned: {len(cxx_entries)}",
            f"- C++ headers with exposed implementation debt: {len(cxx_debt)}",
            f"- C++ headers with compatibility signals: {len(cxx_compat)}",
            f"- dartpy binding sources scanned: {len(python_entries)}",
            f"- dartpy binding sources with allowlisted compatibility debt: {len(python_debt)}",
            "",
            "## C++ Module Summary",
            "",
        ]
    )

    module_rows: list[tuple[str, ...]] = []
    for module in sorted({entry.module for entry in cxx_entries}):
        entries = [entry for entry in cxx_entries if entry.module == module]
        counts = {
            "supported public": 0,
            "compatibility": 0,
            "experimental": 0,
            "exposed-implementation debt": 0,
        }
        for entry in entries:
            counts[entry.classification] += 1
        module_rows.append(
            (
                module,
                str(len(entries)),
                str(counts["supported public"]),
                str(counts["compatibility"]),
                str(counts["experimental"]),
                str(counts["exposed-implementation debt"]),
            )
        )
    append_table(
        lines,
        (
            "Module",
            "Headers",
            "Supported",
            "Compatibility",
            "Experimental",
            "Exposed Debt",
        ),
        module_rows,
    )

    lines.extend(["## C++ Exposed Implementation Debt", ""])
    append_table(
        lines,
        ("Header", "Module", "Private Includes", "Private References", "Samples"),
        [
            (
                entry.path,
                entry.module,
                str(entry.private_include_count),
                str(entry.private_reference_count),
                format_sample_cell(entry.samples),
            )
            for entry in cxx_debt
        ],
    )

    lines.extend(["## C++ Compatibility Signals", ""])
    append_table(
        lines,
        ("Header", "Module", "Signals", "Classification"),
        [
            (
                entry.path,
                entry.module,
                str(entry.compatibility_count),
                entry.classification,
            )
            for entry in cxx_compat
        ],
    )

    lines.extend(["## dartpy Binding Surface", ""])
    append_table(
        lines,
        (
            "Binding Source",
            "Classification",
            "Classes",
            "Module Functions",
            "Debt Entries",
        ),
        [
            (
                entry.path,
                entry.classification,
                ", ".join(entry.class_names) if entry.class_names else "n/a",
                str(entry.module_function_count),
                (
                    "<br>".join(entry.allowlist_entries)
                    if entry.allowlist_entries
                    else "none"
                ),
            )
            for entry in python_entries
        ],
    )

    lines.extend(
        [
            "## Maintenance Notes",
            "",
            "- New supported public API should reduce or avoid exposed implementation debt.",
            "- A new `compatibility/debt` dartpy row must have an allowlist entry with a",
            "  replacement, removal condition, tracking field, and reason.",
            "- Gz-physics-sensitive C++ compatibility changes require the Gazebo integration",
            "  evidence described in [build-system.md](build-system.md#gazebo-integration-feature).",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=OUTPUT_PATH,
        help="inventory output path",
    )
    args = parser.parse_args()

    output_path = args.output
    expected = render_inventory()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(expected)
    print(f"Wrote {output_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
