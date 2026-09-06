"""Guard the shared CI code-path filter (.github/filters/ci-code.yml).

Two things silently break the doc-only skip:

1. ``dorny/paths-filter`` OR-s the patterns of a filter under its default
   ``predicate-quantifier: some``.  A negated pattern such as ``!docs/**``
   then matches every file that is *not* under ``docs/``, so a filter written
   as ``**`` plus exclusions reports ``code=true`` for every change.  Every
   workflow that reads the shared filter must set
   ``predicate-quantifier: some-with-excludes`` on that step.
2. Literal exclusions drift when files move or are deleted; a stale literal is
   harmless to the workflow but means the intent is no longer enforced.

The test is intentionally dependency-free (no YAML parser) so it runs under
the guarded ``test-ai-infra`` runner on every platform.
"""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FILTER_FILE = ROOT / ".github" / "filters" / "ci-code.yml"
WORKFLOW_DIR = ROOT / ".github" / "workflows"
REQUIRED_QUANTIFIER = "some-with-excludes"
FILTER_REFERENCE = "filters: .github/filters/ci-code.yml"


def _filter_patterns(text: str = "") -> list[str]:
    content = text or FILTER_FILE.read_text(encoding="utf-8")
    patterns: list[str] = []
    for line in content.splitlines():
        stripped = line.strip()
        if not stripped.startswith("- "):
            continue
        value = stripped[2:].strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in {'"', "'"}:
            value = value[1:-1]
        patterns.append(value)
    return patterns


def _is_glob(pattern: str) -> bool:
    return any(char in pattern for char in "*?[")


def _workflows_using_shared_filter() -> list[Path]:
    return sorted(
        path
        for path in WORKFLOW_DIR.glob("*.yml")
        if FILTER_REFERENCE in path.read_text(encoding="utf-8")
    )


def _filter_steps(text: str) -> list[str]:
    """Return each `uses: dorny/paths-filter` step block that reads the shared file."""
    blocks: list[str] = []
    step_re = re.compile(
        r"(?ms)^\s*- uses: dorny/paths-filter@[^\n]+\n.*?(?=^\s*- (?:uses|name):|^\s{0,2}\S|\Z)"
    )
    for match in step_re.finditer(text):
        block = match.group(0)
        if FILTER_REFERENCE in block:
            blocks.append(block)
    return blocks


def test_filter_has_exactly_one_positive_pattern() -> None:
    patterns = _filter_patterns()
    positives = [pattern for pattern in patterns if not pattern.startswith("!")]
    assert positives == ["**"], positives


def test_filter_excludes_ai_tool_and_doc_surfaces() -> None:
    patterns = set(_filter_patterns())
    for required in (
        "!.agents/**",
        "!.claude/**",
        "!.codex/**",
        "!.opencode/**",
        "!docs/**",
        "!paper/**",
        "!CHANGELOG.md",
    ):
        assert required in patterns, required


def test_packaged_and_test_consumed_files_stay_code() -> None:
    """Files that builds, tests, or packaging consume must never be excluded.

    pyproject.toml embeds README.md as the wheel's long description and
    scikit-build-core packages LICENSE; python/tests parse
    python/examples/demos/README.md. Markdown is therefore excluded only by
    tree or by root name, never with a blanket ``*.md`` glob.
    """
    patterns = _filter_patterns()
    # tutorials/, data/, and examples/ are installed by the root CMakeLists.txt
    # and therefore packaged into the wheel.
    for forbidden in (
        "!README.md",
        "!LICENSE",
        "!package.xml",
        "!tutorials/**",
        "!data/**",
        "!examples/**",
    ):
        assert forbidden not in patterns, forbidden
    blanket = [pattern for pattern in patterns if pattern.endswith("*.md")]
    assert not blanket, f"blanket markdown exclusions: {blanket}"
    pyproject = (ROOT / "pyproject.toml").read_text(encoding="utf-8")
    assert 'readme = "README.md"' in pyproject


def test_literal_exclusions_exist() -> None:
    missing = [
        pattern
        for pattern in _filter_patterns()
        if pattern.startswith("!")
        and not _is_glob(pattern[1:])
        and not (ROOT / pattern[1:]).exists()
    ]
    assert not missing, f"stale literal exclusions in ci-code.yml: {missing}"


def test_every_shared_filter_step_sets_the_quantifier() -> None:
    workflows = _workflows_using_shared_filter()
    assert workflows, "no workflow reads the shared filter file"
    problems: list[str] = []
    for workflow in workflows:
        text = workflow.read_text(encoding="utf-8")
        steps = _filter_steps(text)
        if not steps:
            problems.append(f"{workflow.name}: could not locate the paths-filter step")
            continue
        for step in steps:
            if f"predicate-quantifier: {REQUIRED_QUANTIFIER}" not in step:
                problems.append(
                    f"{workflow.name}: paths-filter step must set "
                    f"predicate-quantifier: {REQUIRED_QUANTIFIER}"
                )
    assert not problems, "\n".join(problems)


def test_filter_steps_parser_detects_missing_quantifier() -> None:
    sample = (
        "      - uses: dorny/paths-filter@v4\n"
        "        id: filter\n"
        "        with:\n"
        "          filters: .github/filters/ci-code.yml\n"
        "      - name: next\n"
    )
    steps = _filter_steps(sample)
    assert len(steps) == 1
    assert REQUIRED_QUANTIFIER not in steps[0]
