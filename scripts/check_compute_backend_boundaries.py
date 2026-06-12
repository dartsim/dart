#!/usr/bin/env python3
"""Check that scalable-compute backend details stay out of public APIs.

The simulation compute API may expose backend-neutral concepts such as graph
nodes, executor injection, profiles, stage metadata, and DOT visualization. It
must not expose CUDA/SYCL/device/stream/kernel/memory-pool concepts through
public C++ headers or the default dartpy simulation bindings before a later
promotion design and benchmark gate justify that API surface.
"""

from __future__ import annotations

import bisect
import re
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SIMULATION_HEADER_ROOT = REPO_ROOT / "dart" / "simulation"
SIMULATION_BINDING_ROOT = REPO_ROOT / "python" / "dartpy" / "simulation"

SOURCE_SUFFIXES = {".cpp", ".h", ".hpp"}
SKIPPED_HEADER_PARTS = {
    "comps",
    "detail",
    "internal",
}
SKIPPED_HEADER_SUFFIXES = {
    "-impl.hpp",
    "_impl.hpp",
}

PROHIBITED_TERMS = {
    "adaptivecpp",
    "cuda",
    "device",
    "gpu",
    "hip",
    "kernel",
    "kokkos",
    "memorypool",
    "metal",
    "rocm",
    "stream",
    "sycl",
    "transferqueue",
    "vulkan",
    "warp",
}

# A backend capability marker in stage metadata is intentionally backend-neutral:
# it describes acceleration intent for diagnostics/profiling, not a concrete
# device, stream, kernel, memory allocator, or package dependency.
ALLOWLIST = {
    (
        "dart/simulation/compute/compute_stage_metadata.hpp",
        "Gpu",
    ),
}


@dataclass(frozen=True)
class Violation:
    path: str
    line: int
    token: str
    text: str


def _is_scanned_header(path: Path) -> bool:
    rel = path.relative_to(SIMULATION_HEADER_ROOT)
    if any(part in SKIPPED_HEADER_PARTS for part in rel.parts):
        return False
    return not any(path.name.endswith(suffix) for suffix in SKIPPED_HEADER_SUFFIXES)


def iter_scanned_sources() -> list[Path]:
    sources = [
        path
        for path in SIMULATION_HEADER_ROOT.rglob("*")
        if path.is_file() and path.suffix in {".h", ".hpp"} and _is_scanned_header(path)
    ]
    if SIMULATION_BINDING_ROOT.exists():
        sources.extend(
            path
            for path in SIMULATION_BINDING_ROOT.rglob("*")
            if path.is_file() and path.suffix in SOURCE_SUFFIXES
        )
    return sorted(sources)


def _mask_comments_and_literals(text: str) -> str:
    """Replace comments and string/char literals with spaces, preserving lines."""
    result: list[str] = []
    i = 0
    state = "code"
    while i < len(text):
        ch = text[i]
        nxt = text[i + 1] if i + 1 < len(text) else ""

        if state == "code":
            if ch == "/" and nxt == "/":
                result.extend((" ", " "))
                i += 2
                state = "line_comment"
            elif ch == "/" and nxt == "*":
                result.extend((" ", " "))
                i += 2
                state = "block_comment"
            elif ch == '"':
                result.append(" ")
                i += 1
                state = "string"
            elif ch == "'":
                result.append(" ")
                i += 1
                state = "char"
            else:
                result.append(ch)
                i += 1
        elif state == "line_comment":
            result.append("\n" if ch == "\n" else " ")
            i += 1
            if ch == "\n":
                state = "code"
        elif state == "block_comment":
            if ch == "*" and nxt == "/":
                result.extend((" ", " "))
                i += 2
                state = "code"
            else:
                result.append("\n" if ch == "\n" else " ")
                i += 1
        elif state == "string":
            if ch == "\\" and nxt:
                result.extend((" ", " "))
                i += 2
            else:
                result.append("\n" if ch == "\n" else " ")
                i += 1
                if ch == '"':
                    state = "code"
        elif state == "char":
            if ch == "\\" and nxt:
                result.extend((" ", " "))
                i += 2
            else:
                result.append("\n" if ch == "\n" else " ")
                i += 1
                if ch == "'":
                    state = "code"

    return "".join(result)


def _line_starts(text: str) -> list[int]:
    starts = [0]
    starts.extend(match.end() for match in re.finditer("\n", text))
    return starts


def _line_number(starts: list[int], offset: int) -> int:
    return bisect.bisect_right(starts, offset)


def _source_line(text: str, line_number: int) -> str:
    return text.splitlines()[line_number - 1].strip()


def _identifier_terms(token: str) -> list[str]:
    terms: list[str] = []
    for piece in token.split("_"):
        if not piece:
            continue
        terms.extend(
            match.group(0).lower()
            for match in re.finditer(
                r"[A-Z]+(?=[A-Z][a-z]|\d|$)|[A-Z]?[a-z]+|\d+", piece
            )
        )
    return terms


def _is_backend_identifier(token: str) -> bool:
    terms = _identifier_terms(token)
    compact = "".join(terms)
    if compact in PROHIBITED_TERMS:
        return True
    if any(term in PROHIBITED_TERMS for term in terms):
        return True

    bigrams = {left + right for left, right in zip(terms, terms[1:])}
    return bool(bigrams & {"adaptivecpp", "memorypool", "transferqueue"})


def find_backend_boundary_violations_in_text(
    rel_path: str, text: str
) -> list[Violation]:
    masked = _mask_comments_and_literals(text)
    starts = _line_starts(masked)
    violations: list[Violation] = []

    for match in re.finditer(r"\b[A-Za-z_][A-Za-z0-9_]*\b", masked):
        token = match.group(0)
        if not _is_backend_identifier(token):
            continue
        if (rel_path, token) in ALLOWLIST:
            continue
        line_number = _line_number(starts, match.start())
        violations.append(
            Violation(
                rel_path,
                line_number,
                token,
                _source_line(text, line_number),
            )
        )

    return violations


def find_backend_boundary_violations() -> list[Violation]:
    violations: list[Violation] = []
    for path in iter_scanned_sources():
        rel_path = path.relative_to(REPO_ROOT).as_posix()
        violations.extend(
            find_backend_boundary_violations_in_text(
                rel_path, path.read_text(encoding="utf-8")
            )
        )
    return violations


def run_self_tests() -> None:
    allowed_path = "dart/simulation/compute/compute_stage_metadata.hpp"
    if find_backend_boundary_violations_in_text(allowed_path, "enum X { Gpu };"):
        raise AssertionError("Gpu metadata allowlist self-test failed")

    comment_fixture = """
    // CUDA and device text in a comment is fine.
    const char* name = "CudaDevice";
    struct BackendNeutral {};
    """
    if find_backend_boundary_violations_in_text(
        "dart/simulation/world.hpp", comment_fixture
    ):
        raise AssertionError("comment/string masking self-test failed")

    cases = (
        ("class CudaDevice {};", "CudaDevice"),
        ("void set_sycl_stream(int);", "set_sycl_stream"),
        ("struct MemoryPoolHandle {};", "MemoryPoolHandle"),
    )
    for source, token in cases:
        violations = find_backend_boundary_violations_in_text(
            "dart/simulation/world.hpp", source
        )
        if not any(violation.token == token for violation in violations):
            raise AssertionError(f"backend token self-test failed for {token}")


def main() -> int:
    run_self_tests()
    violations = find_backend_boundary_violations()
    if not violations:
        print("Compute backend boundary check passed.")
        return 0

    print("Compute backend boundary check failed:", file=sys.stderr)
    for violation in violations:
        print(
            f"{violation.path}:{violation.line}: backend-api-leakage: "
            f"public API token '{violation.token}' exposes backend plumbing\n"
            f"  {violation.text}",
            file=sys.stderr,
        )
    return 1


if __name__ == "__main__":
    sys.exit(main())
