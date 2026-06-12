#!/usr/bin/env python3
"""Check that the LCP solver roster stays synchronized across public surfaces."""

from __future__ import annotations

import ast
import csv
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
MANIFEST_PATH = ROOT / "tests/common/lcpsolver/lcp_solver_manifest.hpp"
SELECTION_GUIDE_PATH = ROOT / "docs/background/lcp/07_selection-guide.md"
LCP_DEMO_PATH = ROOT / "python/examples/demos/scenes/lcp_physics.py"
LCP_DOCS_DIR = ROOT / "docs/background/lcp"
LCP_PROFILE_CSV_PATHS = {
    "standard": ROOT / "docs/background/lcp/figures/performance_profile_standard.csv",
    "boxed": ROOT / "docs/background/lcp/figures/performance_profile_boxed.csv",
    "findex": ROOT
    / "docs/background/lcp/figures/performance_profile_frictionindex.csv",
}
LCP_PROFILE_EVIDENCE_CSV_PATH = (
    ROOT / "docs/background/lcp/figures/performance_profile_evidence.csv"
)
PROFILE_KEY_BY_CATEGORY = {
    "Standard": "standard",
    "Boxed": "boxed",
    "FrictionIndex": "findex",
}
PROBLEM_TYPE_COUNTER_BY_CATEGORY = {
    "Standard": "problem_type_standard",
    "Boxed": "problem_type_boxed",
    "FrictionIndex": "problem_type_friction_index",
}
SOLVER_IDENTITY_SCHEMA_VERSION = 1
REQUIRED_EVIDENCE_COLUMNS = (
    "category",
    "solver",
    "problem_size",
    "solver_identity_schema_version",
    "solver_manifest_index",
    "time_ns",
    "contract_ok",
    "iterations",
    "residual",
    "complementarity",
    "bound_violation",
    "solver_supports_standard",
    "solver_supports_boxed",
    "solver_supports_friction_index",
    "solver_supports_problem",
    "problem_type_standard",
    "problem_type_boxed",
    "problem_type_friction_index",
    "problem_type_invalid",
)
DARTPY_BINDING_PATH = ROOT / "python/dartpy/math/lcp.cpp"
DARTPY_MATH_STUB_PATH = ROOT / "python/stubs/dartpy/math.pyi"
DARTPY_INIT_STUB_PATH = ROOT / "python/stubs/dartpy/__init__.pyi"
DOCUMENTED_LCP_PATH_PATTERN = re.compile(
    r"(?P<path>dart/math/lcp/(?:[A-Za-z0-9_]+/)*[A-Za-z0-9_]+\.(?:hpp|cpp)"
    r"|(?:pivoting|projection|newton|other)/(?:[A-Za-z0-9_]+/)*"
    r"[A-Za-z0-9_]+\.(?:hpp|cpp))(?:/cpp)?"
)


@dataclass(frozen=True)
class SolverEntry:
    name: str
    family: str
    standard: bool
    boxed: bool
    findex: bool
    class_name: str


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _display_path(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _bool(value: str) -> bool:
    if value == "true":
        return True
    if value == "false":
        return False
    raise ValueError(f"unexpected C++ bool literal: {value}")


def parse_cpp_manifest() -> list[SolverEntry]:
    pattern = re.compile(
        r'\{\s*"(?P<name>[^"]+)"\s*,\s*'
        r'"(?P<family>[^"]+)"\s*,\s*'
        r"(?P<standard>true|false)\s*,\s*"
        r"(?P<boxed>true|false)\s*,\s*"
        r"(?P<findex>true|false)\s*,\s*"
        r"&createLcpSolver<dart::math::(?P<class_name>[A-Za-z0-9_]+)>\s*\}",
        re.DOTALL,
    )
    entries = [
        SolverEntry(
            name=match.group("name"),
            family=match.group("family"),
            standard=_bool(match.group("standard")),
            boxed=_bool(match.group("boxed")),
            findex=_bool(match.group("findex")),
            class_name=match.group("class_name"),
        )
        for match in pattern.finditer(_read(MANIFEST_PATH))
    ]
    if not entries:
        raise AssertionError(f"no solver entries parsed from {MANIFEST_PATH}")
    return entries


def parse_documented_manifest_names() -> list[str]:
    text = _read(SELECTION_GUIDE_PATH)
    begin = "<!-- dart-lcp-solver-manifest: begin -->"
    end = "<!-- dart-lcp-solver-manifest: end -->"
    try:
        block = text.split(begin, 1)[1].split(end, 1)[0]
    except IndexError as exc:
        raise AssertionError(
            f"{SELECTION_GUIDE_PATH} is missing the LCP solver manifest block"
        ) from exc

    names: list[str] = []
    for line in block.splitlines():
        if line.startswith("- "):
            match = re.search(r"`([^`]+)`", line)
            if match:
                names.append(match.group(1))
    return names


def _literal_assignment(module: ast.Module, name: str) -> Any:
    for node in module.body:
        if isinstance(node, ast.Assign):
            if any(
                isinstance(target, ast.Name) and target.id == name
                for target in node.targets
            ):
                return ast.literal_eval(node.value)
        if (
            isinstance(node, ast.AnnAssign)
            and isinstance(node.target, ast.Name)
            and node.target.id == name
            and node.value is not None
        ):
            return ast.literal_eval(node.value)
    raise AssertionError(f"{LCP_DEMO_PATH} is missing assignment {name}")


def parse_demo_roster() -> tuple[list[dict[str, Any]], dict[str, str]]:
    module = ast.parse(_read(LCP_DEMO_PATH), filename=str(LCP_DEMO_PATH))
    rows = list(_literal_assignment(module, "_SOLVER_SUPPORT_ROWS"))
    class_names = dict(_literal_assignment(module, "_SOLVER_CLASS_NAMES"))
    return rows, class_names


def parse_bound_solver_classes() -> dict[str, str]:
    pattern = re.compile(
        r"bind(?:Parameterized)?LcpSolverClass"
        r"<\s*(?P<class_name>[A-Za-z0-9_]+)\s*>\s*"
        r'\(\s*m\s*,\s*"(?P<python_name>[A-Za-z0-9_]+)"\s*\)',
        re.DOTALL,
    )
    return {
        match.group("class_name"): match.group("python_name")
        for match in pattern.finditer(_read(DARTPY_BINDING_PATH))
    }


def parse_math_stub_solver_classes() -> set[str]:
    pattern = re.compile(r"^class ([A-Za-z0-9_]+)\(LcpSolver\):", re.MULTILINE)
    return set(pattern.findall(_read(DARTPY_MATH_STUB_PATH)))


def assert_unique(values: list[str], label: str) -> None:
    seen: set[str] = set()
    duplicates: list[str] = []
    for value in values:
        if value in seen:
            duplicates.append(value)
        seen.add(value)
    if duplicates:
        raise AssertionError(f"{label} contains duplicate entries: {duplicates}")


def _documented_lcp_path_candidates(raw_path: str, token: str) -> list[Path]:
    paths = [raw_path]
    if token.endswith(".hpp/cpp"):
        paths.append(f"{raw_path.removesuffix('.hpp')}.cpp")
    if token.endswith(".cpp/hpp"):
        paths.append(f"{raw_path.removesuffix('.cpp')}.hpp")

    candidates: list[Path] = []
    for path in paths:
        if path.startswith("dart/math/lcp/"):
            candidates.append(ROOT / path)
        else:
            candidates.append(ROOT / "dart/math/lcp" / path)
    return candidates


def check_documented_lcp_paths() -> None:
    missing: list[str] = []
    for doc_path in sorted(LCP_DOCS_DIR.glob("*.md")):
        text = _read(doc_path)
        for match in DOCUMENTED_LCP_PATH_PATTERN.finditer(text):
            token = match.group(0)
            raw_path = match.group("path")
            for candidate in _documented_lcp_path_candidates(raw_path, token):
                if not candidate.is_file():
                    rel_doc = doc_path.relative_to(ROOT)
                    rel_candidate = candidate.relative_to(ROOT)
                    missing.append(f"{rel_doc}: {token} -> {rel_candidate}")

    if missing:
        formatted = "\n".join(f"  - {entry}" for entry in missing)
        raise AssertionError(
            "documented LCP header/source paths do not exist:\n" f"{formatted}"
        )


def check_performance_profile_headers(manifest: list[SolverEntry]) -> None:
    manifest_by_name = {entry.name: entry for entry in manifest}
    unsupported_by_profile = {
        "standard": {entry.name for entry in manifest if not entry.standard},
        "boxed": {entry.name for entry in manifest if not entry.boxed},
        "findex": {entry.name for entry in manifest if not entry.findex},
    }

    errors: list[str] = []
    for profile, path in LCP_PROFILE_CSV_PATHS.items():
        with path.open(newline="", encoding="utf-8") as f:
            header = next(csv.reader(f))
        if not header or header[0] != "tau":
            errors.append(f"{path.relative_to(ROOT)} has invalid header {header!r}")
            continue

        solvers = header[1:]
        unknown = sorted(set(solvers) - set(manifest_by_name))
        unsupported = sorted(set(solvers) & unsupported_by_profile[profile])
        if unknown:
            errors.append(
                f"{path.relative_to(ROOT)} contains unknown solvers: {unknown}"
            )
        if unsupported:
            errors.append(
                f"{path.relative_to(ROOT)} contains non-native {profile} solvers: "
                f"{unsupported}"
            )

    if errors:
        raise AssertionError(
            "LCP performance profile headers are out of sync with native "
            "solver support:\n  - " + "\n  - ".join(errors)
        )


def _csv_counter_as_int(row: dict[str, str], key: str) -> int | None:
    value = row.get(key, "")
    if value == "":
        return None
    try:
        numeric = float(value)
    except ValueError:
        return None
    rounded = int(round(numeric))
    if abs(numeric - rounded) > 1e-9:
        return None
    return rounded


def _csv_finite_float(row: dict[str, str], key: str) -> float | None:
    value = row.get(key, "")
    if value == "":
        return None
    try:
        numeric = float(value)
    except ValueError:
        return None
    if not math.isfinite(numeric):
        return None
    return numeric


def _solver_support(entry: SolverEntry, category: str) -> bool:
    if category == "Standard":
        return entry.standard
    if category == "Boxed":
        return entry.boxed
    if category == "FrictionIndex":
        return entry.findex
    raise AssertionError(f"unknown LCP profile category: {category}")


def check_performance_profile_evidence(
    manifest: list[SolverEntry],
    path: Path = LCP_PROFILE_EVIDENCE_CSV_PATH,
) -> None:
    manifest_by_name = {entry.name: entry for entry in manifest}
    manifest_index_by_name = {
        entry.name: index for index, entry in enumerate(manifest, start=1)
    }

    errors: list[str] = []
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        header = reader.fieldnames or []
        missing_columns = [
            column for column in REQUIRED_EVIDENCE_COLUMNS if column not in header
        ]
        if missing_columns:
            raise AssertionError(
                f"{_display_path(path)} is missing required columns: {missing_columns}"
            )

        row_count = 0
        for row_number, row in enumerate(reader, start=2):
            row_count += 1
            category = row["category"]
            solver_name = row["solver"]
            entry = manifest_by_name.get(solver_name)

            if category not in PROFILE_KEY_BY_CATEGORY:
                errors.append(f"row {row_number}: unknown category {category!r}")
                continue
            if entry is None:
                errors.append(f"row {row_number}: unknown solver {solver_name!r}")
                continue

            problem_size = _csv_counter_as_int(row, "problem_size")
            if problem_size is None or problem_size <= 0:
                errors.append(
                    f"row {row_number}: invalid problem_size {row['problem_size']!r}"
                )

            time_ns = _csv_finite_float(row, "time_ns")
            if time_ns is None or time_ns <= 0.0:
                errors.append(f"row {row_number}: invalid time_ns {row['time_ns']!r}")

            contract_ok = _csv_counter_as_int(row, "contract_ok")
            if contract_ok != 1:
                errors.append(
                    f"row {row_number}: contract_ok {row['contract_ok']!r} != 1"
                )

            iterations = _csv_counter_as_int(row, "iterations")
            if iterations is None or iterations < 0:
                errors.append(
                    f"row {row_number}: invalid iterations {row['iterations']!r}"
                )

            for metric in ("residual", "complementarity", "bound_violation"):
                value = _csv_finite_float(row, metric)
                if value is None or value < 0.0:
                    errors.append(f"row {row_number}: invalid {metric} {row[metric]!r}")

            identity_version = _csv_counter_as_int(
                row, "solver_identity_schema_version"
            )
            if identity_version != SOLVER_IDENTITY_SCHEMA_VERSION:
                errors.append(
                    f"row {row_number}: solver_identity_schema_version "
                    f"{row['solver_identity_schema_version']!r} != "
                    f"{SOLVER_IDENTITY_SCHEMA_VERSION}"
                )

            manifest_index = _csv_counter_as_int(row, "solver_manifest_index")
            expected_manifest_index = manifest_index_by_name[solver_name]
            if manifest_index != expected_manifest_index:
                errors.append(
                    f"row {row_number}: solver_manifest_index "
                    f"{row['solver_manifest_index']!r} does not match "
                    f"{solver_name} index {expected_manifest_index}"
                )

            expected_support = {
                "solver_supports_standard": entry.standard,
                "solver_supports_boxed": entry.boxed,
                "solver_supports_friction_index": entry.findex,
                "solver_supports_problem": _solver_support(entry, category),
            }
            for key, expected in expected_support.items():
                expected_value = 1 if expected else 0
                actual = _csv_counter_as_int(row, key)
                if actual != expected_value:
                    errors.append(
                        f"row {row_number}: {key} {row[key]!r} != "
                        f"{expected_value} for {solver_name}/{category}"
                    )

            expected_problem_type = {
                "problem_type_standard": 0,
                "problem_type_boxed": 0,
                "problem_type_friction_index": 0,
                "problem_type_invalid": 0,
            }
            expected_problem_type[PROBLEM_TYPE_COUNTER_BY_CATEGORY[category]] = 1
            for key, expected_value in expected_problem_type.items():
                actual = _csv_counter_as_int(row, key)
                if actual != expected_value:
                    errors.append(
                        f"row {row_number}: {key} {row[key]!r} != "
                        f"{expected_value} for {category}"
                    )

    if row_count == 0:
        errors.append(f"{_display_path(path)} has no evidence rows")

    if errors:
        raise AssertionError(
            "LCP performance profile evidence is out of sync:\n  - "
            + "\n  - ".join(errors)
        )


def check_roster() -> None:
    check_documented_lcp_paths()

    manifest = parse_cpp_manifest()
    check_performance_profile_headers(manifest)
    check_performance_profile_evidence(manifest)
    manifest_names = [entry.name for entry in manifest]
    manifest_classes = [entry.class_name for entry in manifest]
    manifest_by_name = {entry.name: entry for entry in manifest}

    assert len(manifest) == 24, f"expected 24 LCP solvers, found {len(manifest)}"
    assert_unique(manifest_names, "C++ LCP solver manifest")
    assert_unique(manifest_classes, "C++ LCP solver class names")

    standard_count = sum(1 for entry in manifest if entry.standard)
    boxed_count = sum(1 for entry in manifest if entry.boxed)
    findex_count = sum(1 for entry in manifest if entry.findex)
    if (standard_count, boxed_count, findex_count) != (23, 15, 16):
        raise AssertionError(
            "unexpected LCP support counts: "
            f"standard={standard_count}, boxed={boxed_count}, findex={findex_count}"
        )

    documented_names = parse_documented_manifest_names()
    if documented_names != manifest_names:
        raise AssertionError(
            "docs/background/lcp/07_selection-guide.md solver block does not "
            f"match the C++ manifest.\nmanifest={manifest_names}\ndocs={documented_names}"
        )

    demo_rows, demo_class_names = parse_demo_roster()
    demo_names = [row["name"] for row in demo_rows]
    if demo_names != manifest_names:
        raise AssertionError(
            f"lcp_physics solver rows do not match manifest.\n"
            f"manifest={manifest_names}\ndemo={demo_names}"
        )
    if list(demo_class_names) != manifest_names:
        raise AssertionError(
            f"lcp_physics class-name map keys do not match manifest.\n"
            f"manifest={manifest_names}\nmap={list(demo_class_names)}"
        )
    for row in demo_rows:
        entry = manifest_by_name[row["name"]]
        expected = {
            "family": entry.family,
            "standard": entry.standard,
            "boxed": entry.boxed,
            "findex": entry.findex,
        }
        actual = {key: row[key] for key in expected}
        if actual != expected:
            raise AssertionError(
                f"lcp_physics metadata mismatch for {entry.name}: "
                f"expected {expected}, got {actual}"
            )
        if demo_class_names[entry.name] != entry.class_name:
            raise AssertionError(
                f"lcp_physics class map mismatch for {entry.name}: "
                f"expected {entry.class_name}, got {demo_class_names[entry.name]}"
            )

    binding_names = parse_bound_solver_classes()
    for entry in manifest:
        bound_name = binding_names.get(entry.class_name)
        if bound_name != entry.class_name:
            raise AssertionError(
                f"dartpy binding mismatch for {entry.class_name}: {bound_name!r}"
            )

    math_stub_classes = parse_math_stub_solver_classes()
    missing_math_stub = sorted(set(manifest_classes) - math_stub_classes)
    if missing_math_stub:
        raise AssertionError(
            "python/stubs/dartpy/math.pyi is missing solver classes: "
            f"{missing_math_stub}"
        )

    init_stub_text = _read(DARTPY_INIT_STUB_PATH)
    missing_init_stub = [
        class_name
        for class_name in manifest_classes
        if class_name not in init_stub_text
    ]
    if missing_init_stub:
        raise AssertionError(
            "python/stubs/dartpy/__init__.pyi is missing solver classes: "
            f"{missing_init_stub}"
        )


def main() -> int:
    try:
        check_roster()
    except AssertionError as exc:
        print(f"LCP solver roster check failed: {exc}", file=sys.stderr)
        return 1

    print(
        "LCP solver roster check passed: 24 solvers, 23 standard, 15 boxed, 16 findex."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
