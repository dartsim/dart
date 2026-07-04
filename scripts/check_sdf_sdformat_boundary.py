#!/usr/bin/env python3
"""Check that SDF IO stays on libsdformat instead of raw XML parsing."""

from __future__ import annotations

import re
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SDF_ROOT = REPO_ROOT / "dart" / "utils" / "sdf"
SOURCE_SUFFIXES = {".cpp", ".hpp", ".h"}

FORBIDDEN_PATTERNS: tuple[tuple[re.Pattern[str], str], ...] = (
    (
        re.compile(r"#\s*include\s*[<\"].*(?:tinyxml|xml_helpers)", re.IGNORECASE),
        "SDF production code must not include TinyXML or DART XML helpers.",
    ),
    (
        re.compile(
            r"\b(?:TiXml\w*|tinyxml2::|XMLDocument|XMLElement|XMLNode|XMLAttribute)\b"
        ),
        "SDF production code must not use raw XML parser types.",
    ),
    (
        re.compile(
            r"(?:->|\.)"
            r"(?:FirstChild(?:Element)?|NextSibling(?:Element)?|"
            r"GetText|Query[A-Za-z]*Attribute|Attribute)\s*\("
        ),
        "SDF production code must not use raw XML tree or attribute APIs.",
    ),
    (
        re.compile(r"\bstd::(?:sto[fdil]|strto[fdil]|istringstream)\b"),
        "SDF production code must not parse SDF element text itself.",
    ),
)

HELPER_HEADER = REPO_ROOT / "dart" / "utils" / "sdf" / "detail" / "sdf_helpers.hpp"
HELPER_IMPL = REPO_ROOT / "dart" / "utils" / "sdf" / "detail" / "sdf_helpers.cpp"
ALLOWED_HELPER_API = {"findAuthoredElement", "hasAuthoredElement"}
ALLOWED_HELPER_IMPL = ALLOWED_HELPER_API | {"findChildElement"}

HELPER_HEADER_API_PATTERN = re.compile(
    r"DART_UTILS_API\s+(?:ElementPtr|bool)\s+([A-Za-z_]\w*)\s*\("
)
HELPER_IMPL_FUNCTION_PATTERN = re.compile(
    r"^(?:sdf::ElementPtr|ElementPtr|bool)\s+([A-Za-z_]\w*)\s*\(",
    re.MULTILINE,
)


@dataclass(frozen=True)
class Violation:
    path: Path
    line: int
    message: str
    text: str

    def format(self) -> str:
        rel_path = self.path.relative_to(REPO_ROOT).as_posix()
        return f"{rel_path}:{self.line}: {self.message}\n  {self.text}"


def iter_sdf_sources() -> list[Path]:
    return sorted(
        path
        for path in SDF_ROOT.rglob("*")
        if path.is_file() and path.suffix in SOURCE_SUFFIXES
    )


def find_forbidden_xml_usage() -> list[Violation]:
    violations: list[Violation] = []
    for path in iter_sdf_sources():
        for line_number, line in enumerate(path.read_text().splitlines(), 1):
            for pattern, message in FORBIDDEN_PATTERNS:
                if pattern.search(line):
                    violations.append(
                        Violation(path, line_number, message, line.strip())
                    )
    return violations


def find_helper_surface_violations() -> list[Violation]:
    violations: list[Violation] = []

    header_text = HELPER_HEADER.read_text()
    for match in HELPER_HEADER_API_PATTERN.finditer(header_text):
        name = match.group(1)
        if name not in ALLOWED_HELPER_API:
            line_number = header_text.count("\n", 0, match.start()) + 1
            violations.append(
                Violation(
                    HELPER_HEADER,
                    line_number,
                    "SDF helper API must stay limited to authored sdformat element lookup.",
                    match.group(0).strip(),
                )
            )

    impl_text = HELPER_IMPL.read_text()
    for match in HELPER_IMPL_FUNCTION_PATTERN.finditer(impl_text):
        name = match.group(1)
        if name not in ALLOWED_HELPER_IMPL:
            line_number = impl_text.count("\n", 0, match.start()) + 1
            violations.append(
                Violation(
                    HELPER_IMPL,
                    line_number,
                    "SDF helper implementation must not grow generic XML parsing helpers.",
                    match.group(0).strip(),
                )
            )

    return violations


def main() -> int:
    violations = find_forbidden_xml_usage() + find_helper_surface_violations()
    if violations:
        print(
            "SDF sdformat boundary check failed. Use libsdformat typed DOM APIs "
            "for SDF semantics; keep direct sdf::Element access limited to the "
            "documented authored/default or serialization-gap bridge.",
            file=sys.stderr,
        )
        for violation in violations:
            print(violation.format(), file=sys.stderr)
        return 1

    print("SDF sdformat boundary check passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
