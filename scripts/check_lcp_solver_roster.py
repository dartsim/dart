#!/usr/bin/env python3
"""Check that the LCP solver roster stays synchronized across public surfaces."""

from __future__ import annotations

import ast
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


def check_roster() -> None:
    check_documented_lcp_paths()

    manifest = parse_cpp_manifest()
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
