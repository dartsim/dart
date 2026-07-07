#!/usr/bin/env python3
"""Check that DART 7 World promotion blockers stay explicitly classified.

PLAN-041/042 promote the ECS-backed World to the official
``dart::simulation::World`` and remove the DART 6 simulation pipeline from the
main branch. This gate keeps retired-path blockers visible: old DART 6 World
dependencies, experimental namespace tokens, and experimental package/build-
option names must live only in known transition buckets.

The default mode is a ratchet for lint jobs. It fails on new unclassified
blockers and on growth in the current code/build/test transition buckets, but
it does not fail merely because the current transition debt still exists. Pass
``--strict-final`` when doing the final promotion/removal slice; in that mode,
any remaining code/build/test blocker outside transition docs and historical
changelog entries fails.
"""

from __future__ import annotations

import argparse
import re
import subprocess
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

TEXT_SUFFIXES = {
    ".c",
    ".cc",
    ".cmake",
    ".cpp",
    ".cu",
    ".cuh",
    ".h",
    ".hpp",
    ".in",
    ".md",
    ".py",
    ".rst",
    ".toml",
    ".txt",
    ".yaml",
    ".yml",
}

SKIP_DIR_PARTS = {
    ".git",
    ".mypy_cache",
    ".pixi",
    ".pytest_cache",
    "__pycache__",
    "build",
    "external",
    "node_modules",
}

SKIP_PATHS = {
    "scripts/check_dart7_world_promotion_blockers.py",
    "python/tests/unit/test_check_dart7_world_promotion_blockers.py",
}


@dataclass(frozen=True)
class Marker:
    name: str
    kind: str
    pattern: re.Pattern[str]


@dataclass(frozen=True)
class Finding:
    path: str
    line: int
    marker: str
    kind: str
    category: str | None
    text: str


@dataclass(frozen=True)
class Violation:
    path: str
    line: int
    message: str


MARKERS = (
    Marker(
        "classic-world-shared-ptr",
        "classic-world",
        re.compile(r"\b(?:dart::)?simulation::WorldPtr\b|\bWorldPtr\b"),
    ),
    Marker(
        "classic-world-factory",
        "classic-world",
        re.compile(r"\b(?:dart::)?simulation::World::create\s*\(|\bWorld::create\s*\("),
    ),
    Marker(
        "classic-world-config",
        "classic-world",
        re.compile(r"\bWorldConfig\b|\bCollisionDetectorType\b|\bLcpSolverType\b"),
    ),
    Marker(
        "classic-render-world-binding",
        "classic-world",
        re.compile(r"\bRenderWorld\b"),
    ),
    Marker(
        "classic-world-method",
        "classic-world",
        re.compile(
            r"\b(?:addSimpleFrame|getNumSimpleFrames|getConstraintSolver|"
            r"checkCollision|getSimFrames|bake)\s*\("
        ),
    ),
    Marker(
        "public-classic-world-loader",
        "classic-world",
        re.compile(r"\b(?:dart::)?io::readWorld\b"),
    ),
    Marker(
        "temporary-world-facade",
        "temporary-world-facade",
        re.compile(
            r"\bWorld7\b|\bworld7\.hpp\b|" r"\b(?:dart::)?simulation::v7::World\b"
        ),
    ),
    Marker(
        "experimental-build-option",
        "experimental-staging",
        re.compile(r"\bDART_BUILD_SIMULATION_EXPERIMENTAL\b"),
    ),
    Marker(
        "experimental-cmake-target",
        "experimental-staging",
        re.compile(r"\bdart-simulation-experimental\b"),
    ),
    Marker(
        "experimental-component",
        "experimental-staging",
        re.compile(r"\bsimulation-experimental\b"),
    ),
    Marker(
        "experimental-namespace",
        "experimental-staging",
        re.compile(r"\bdart::simulation::experimental\b"),
    ),
    Marker(
        "experimental-include-path",
        "experimental-staging",
        re.compile(r"\bdart/simulation/experimental\b"),
    ),
)

FINAL_ALLOWED_CATEGORIES = {
    "historical_changelog",
    "transition_docs",
}

RELEASE6_BRANCH_REF = re.compile(
    r"^refs/(?:heads|remotes/[^/]+)/release-6(?:\.\d+(?:\.\d+)*)?$"
)

# Ratchet ceilings for blocker categories that represent code, build, tests, or
# user-facing examples. These numbers are the current staged-transition counts;
# future promotion work should drive them down. Increasing one means new DART 6
# World, experimental namespace/path, or staged package debt was introduced and
# needs an explicit audit update.
DEFAULT_CATEGORY_CEILINGS = {
    "ci_metadata": 5,
    "classic_loader_api": 40,
    "classic_loader_tests": 0,
    "classic_render_sensor_bridge": 30,
    "classic_simulation_owner": 6,
    "dartsim_app_transition": 29,
    "example_runner_tests": 3,
    "experimental_python_tests": 0,
    "experimental_staging_implementation": 1324,
    "experimental_staging_tests": 1360,
    "legacy_classic_tests": 284,
    "main_tree_parity_reference": 0,
    "python_classic_render_binding": 59,
    "python_integration_transition_tests": 0,
    "python_transition_tests": 0,
    "staged_build_pipeline": 55,
    "transition_examples": 53,
    "user_facing_classic_examples": 0,
}

DEFAULT_CATEGORY_ALLOWED_PATHS = {"main_tree_parity_reference": set()}


def _is_text_path(path: Path) -> bool:
    return path.name == "CMakeLists.txt" or path.suffix in TEXT_SUFFIXES


def _tracked_files(root: Path) -> list[Path]:
    try:
        result = subprocess.run(
            ["git", "-C", str(root), "ls-files", "-z"],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
    except OSError, subprocess.CalledProcessError:
        result = None

    if result is not None:
        rels = [Path(raw.decode("utf-8")) for raw in result.stdout.split(b"\0") if raw]
    else:
        rels = [
            path.relative_to(root)
            for path in root.rglob("*")
            if path.is_file()
            and not any(part in SKIP_DIR_PARTS for part in path.relative_to(root).parts)
        ]

    return [
        rel
        for rel in rels
        if (root / rel).is_file()
        and rel.as_posix() not in SKIP_PATHS
        and _is_text_path(rel)
        and not any(part in SKIP_DIR_PARTS for part in rel.parts)
    ]


def _git_refnames(root: Path) -> tuple[str, ...]:
    try:
        result = subprocess.run(
            [
                "git",
                "-C",
                str(root),
                "for-each-ref",
                "--format=%(refname)",
                "refs/heads",
                "refs/remotes",
            ],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
    except OSError, subprocess.CalledProcessError:
        return ()
    return tuple(line.strip() for line in result.stdout.splitlines() if line.strip())


def release6_branch_refs(refnames: tuple[str, ...]) -> tuple[str, ...]:
    return tuple(sorted(ref for ref in set(refnames) if RELEASE6_BRANCH_REF.match(ref)))


def release6_branch_violations(refnames: tuple[str, ...]) -> list[Violation]:
    if release6_branch_refs(refnames):
        return []
    return [
        Violation(
            "git refs",
            0,
            "final DART 7 promotion requires a local release-6.* branch ref "
            "(refs/heads/release-6.* or refs/remotes/*/release-6.*) so parity "
            "evidence comes from the DART 6 maintenance line rather than "
            "main-branch classic World tests",
        )
    ]


def _line_for_offset(text: str, offset: int) -> int:
    return text.count("\n", 0, offset) + 1


def _read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _classify(path: str, kind: str) -> str | None:
    if path.startswith(".claude/") or path.startswith(".codex/"):
        return "ai_workflow_metadata"
    if path == "CHANGELOG.md":
        return "historical_changelog"
    if path == "README.md":
        return "user_facing_classic_examples"
    if path.startswith("docs/"):
        return "transition_docs"
    if path.startswith("dart/simulation/experimental/"):
        return "experimental_staging_implementation"
    if path.startswith("tests/unit/simulation/experimental/") or path.startswith(
        "tests/benchmark/simulation/experimental/"
    ):
        if kind == "classic-world":
            return "main_tree_parity_reference"
        return "experimental_staging_tests"
    if path == "pixi.toml" or path == "CMakeLists.txt" or path.startswith("cmake/"):
        return "staged_build_pipeline"
    if path.startswith("scripts/") or path.startswith("python/tests/unit/test_check_"):
        return "promotion_guard_scripts"
    if path.startswith("dart/simulation/"):
        return "classic_simulation_owner"
    if path.startswith("dart/io/"):
        return "classic_loader_api"
    if path.startswith("dart/gui/") or path.startswith("dart/sensor/"):
        return "classic_render_sensor_bridge"
    if path.startswith("dartsim/"):
        return "dartsim_app_transition"
    if path.startswith("python/examples/demos/"):
        return "transition_examples"
    if path.startswith("python/tests/integration/"):
        return "python_integration_transition_tests"
    if path.startswith("python/tests/unit/simulation/test_experimental_"):
        return "experimental_python_tests"
    if path.startswith("python/tests/unit/test_run_cpp_example.py"):
        return "example_runner_tests"
    if path.startswith("python/tests/unit/io/"):
        return "classic_loader_tests"
    if path.startswith("python/tests/"):
        return "python_transition_tests"
    if path.startswith("python/dartpy/"):
        return "python_classic_render_binding"
    if path.startswith("examples/demos/") or path.startswith("examples/experimental"):
        return "transition_examples"
    if path.startswith("examples/"):
        return "user_facing_classic_examples"
    if path.startswith("tests/"):
        return "legacy_classic_tests"
    if path.startswith(".github/"):
        return "ci_metadata"
    return None


def collect_findings(root: Path = REPO_ROOT) -> list[Finding]:
    root = root.resolve()
    findings: list[Finding] = []
    for rel in _tracked_files(root):
        path = root / rel
        try:
            text = _read_text(path)
        except UnicodeDecodeError:
            continue
        relpath = rel.as_posix()
        for marker in MARKERS:
            for match in marker.pattern.finditer(text):
                findings.append(
                    Finding(
                        path=relpath,
                        line=_line_for_offset(text, match.start()),
                        marker=marker.name,
                        kind=marker.kind,
                        category=_classify(relpath, marker.kind),
                        text=text[
                            text.rfind("\n", 0, match.start())
                            + 1 : (
                                text.find("\n", match.end())
                                if text.find("\n", match.end()) != -1
                                else len(text)
                            )
                        ].strip(),
                    )
                )
    return findings


def _category_counts(findings: list[Finding]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for finding in findings:
        category = finding.category or "unclassified"
        counts[category] = counts.get(category, 0) + 1
    return counts


def _default_violations(
    findings: list[Finding],
    category_ceilings: Mapping[str, int] | None,
    category_allowed_paths: Mapping[str, set[str]] | None,
) -> list[Violation]:
    violations: list[Violation] = []
    for finding in findings:
        if finding.category is not None:
            continue
        violations.append(
            Violation(
                finding.path,
                finding.line,
                f"unclassified {finding.kind} blocker ({finding.marker}): "
                f"{finding.text}",
            )
        )

    if category_ceilings is None:
        return violations

    counts = _category_counts(findings)
    for category, ceiling in sorted(category_ceilings.items()):
        count = counts.get(category, 0)
        if count <= ceiling:
            continue
        violations.append(
            Violation(
                f"category:{category}",
                0,
                f"{category} blocker count grew from ratchet ceiling "
                f"{ceiling} to {count}; reduce the new debt or update the "
                "audit with an explicit promotion-plan rationale",
            )
        )

    if category_allowed_paths is None:
        return violations

    paths_by_category: dict[str, set[str]] = {}
    for finding in findings:
        if finding.category is None:
            continue
        paths_by_category.setdefault(finding.category, set()).add(finding.path)
    for category, allowed_paths in sorted(category_allowed_paths.items()):
        paths = paths_by_category.get(category, set())
        for path in sorted(paths - allowed_paths):
            violations.append(
                Violation(
                    path,
                    0,
                    f"{category} blocker moved into an unapproved file; "
                    "retire the main-tree DART 6 parity dependency or update "
                    "the audit with an explicit release-6.* parity migration "
                    "rationale",
                )
            )
    return violations


def find_violations(
    root: Path = REPO_ROOT,
    *,
    category_ceilings: Mapping[str, int] | None = DEFAULT_CATEGORY_CEILINGS,
    category_allowed_paths: Mapping[str, set[str]] | None = (
        DEFAULT_CATEGORY_ALLOWED_PATHS
    ),
) -> list[Violation]:
    return _default_violations(
        collect_findings(root),
        category_ceilings,
        category_allowed_paths,
    )


def _final_violations(findings: list[Finding]) -> list[Violation]:
    violations: list[Violation] = []
    for finding in findings:
        if finding.category in FINAL_ALLOWED_CATEGORIES:
            continue
        category = finding.category or "unclassified"
        violations.append(
            Violation(
                finding.path,
                finding.line,
                "final DART 7 promotion still has "
                f"{finding.kind} debt in {category} ({finding.marker})",
            )
        )
    return violations


def find_final_violations(root: Path = REPO_ROOT) -> list[Violation]:
    return _final_violations(collect_findings(root))


def _format_violation(violation: Violation) -> str:
    if violation.line > 0:
        return f"{violation.path}:{violation.line}: {violation.message}"
    return f"{violation.path}: {violation.message}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        type=Path,
        default=REPO_ROOT,
        help="Repository root to check.",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="Print every classified blocker finding.",
    )
    parser.add_argument(
        "--strict-final",
        action="store_true",
        help=(
            "Fail on any remaining code/build/test blocker. This is expected "
            "to fail until the final C++ World/package/removal transaction."
        ),
    )
    parser.add_argument(
        "--require-release6-branch",
        action="store_true",
        help=(
            "Also require a local release-6.* branch ref. Use with "
            "--strict-final before claiming final parity evidence."
        ),
    )
    args = parser.parse_args()

    root = args.root.resolve()
    findings = collect_findings(root)
    violations = (
        _final_violations(findings)
        if args.strict_final
        else _default_violations(
            findings,
            DEFAULT_CATEGORY_CEILINGS,
            DEFAULT_CATEGORY_ALLOWED_PATHS,
        )
    )
    release6_refs = release6_branch_refs(_git_refnames(root))
    if args.require_release6_branch:
        violations.extend(release6_branch_violations(release6_refs))

    if args.list:
        for finding in findings:
            category = finding.category or "unclassified"
            print(
                f"{finding.path}:{finding.line}: {category}: "
                f"{finding.marker}: {finding.text}"
            )
        if release6_refs:
            for ref in release6_refs:
                print(f"{ref}: release6-parity-ref")

    if violations:
        for violation in violations:
            print(_format_violation(violation))
        return 1

    counts = _category_counts(findings)
    if counts:
        summary = ", ".join(f"{name}={count}" for name, count in sorted(counts.items()))
        if args.require_release6_branch:
            summary = f"{summary}, release6_branch_refs={len(release6_refs)}"
        print(f"DART 7 World promotion blocker check passed ({summary})")
    else:
        print("DART 7 World promotion blocker check passed (no blockers found)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
