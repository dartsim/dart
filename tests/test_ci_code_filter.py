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

import ast
import re
import tomllib
from fnmatch import fnmatchcase
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
        "!docs/ai/**",
        "!docs/onboarding/**",
        "!paper/**",
        "!CHANGELOG.md",
    ):
        assert required in patterns, required


def test_packaged_and_test_consumed_files_stay_code() -> None:
    """Files that builds, tests, or packaging consume must never be excluded.

    pyproject.toml embeds README.md as the wheel's long description and
    scikit-build-core packages LICENSE; the root CMakeLists.txt installs
    tutorials/, data/, and examples/ and configures docs/doxygen; python/tests
    parse python/examples/demos/README.md, docs/python_api, docs/readthedocs/
    conf.py, and evidence packets under docs/plans; a C++ test parses
    docs/background/lcp. Markdown is therefore excluded only by tree or by
    name, never with a blanket ``*.md`` glob.
    """
    patterns = _filter_patterns()
    for forbidden in (
        "!README.md",
        "!LICENSE",
        "!package.xml",
        "!tutorials/**",
        "!data/**",
        "!examples/**",
        "!docs/**",
        "!docs/plans/**",
        "!docs/python_api/**",
        "!docs/doxygen/**",
        "!docs/background/**",
        "!docs/background/lcp/**",
        "!docs/readthedocs/**",
        "!docs/readthedocs/conf.py",
    ):
        assert forbidden not in patterns, forbidden
    blanket = [pattern for pattern in patterns if pattern.endswith("**/*.md")]
    assert blanket == ["!docs/readthedocs/**/*.md"], blanket
    pyproject = (ROOT / "pyproject.toml").read_text(encoding="utf-8")
    assert 'readme = "README.md"' in pyproject


# --- Consumer scan: no excluded path may be read by the tier the skip disables.

_TREE_RE = re.compile(r"^(?P<dir>[^*?\[]+)/\*\*$")
_TREE_EXT_RE = re.compile(r"^(?P<dir>[^*?\[]+)/\*\*/\*\.(?P<ext>[A-Za-z0-9]+)$")


def _exclusion_matcher(pattern: str):
    """Return a predicate for the three pattern shapes the filter may use.

    ``dir/**`` (a tree), ``dir/**/*.ext`` (one extension anywhere under a
    tree, including its root), and a literal path. Anything else fails the
    suite so the matcher stays exact with respect to dorny/paths-filter.
    """
    body = pattern[1:]
    tree = _TREE_RE.match(body)
    if tree:
        prefix = tree.group("dir")
        return lambda path: path == prefix or path.startswith(prefix + "/")
    tree_ext = _TREE_EXT_RE.match(body)
    if tree_ext:
        prefix, ext = tree_ext.group("dir"), tree_ext.group("ext")
        return lambda path: path.startswith(prefix + "/") and path.endswith("." + ext)
    assert not any(
        char in body for char in "*?["
    ), f"unsupported pattern shape: {pattern}"
    return lambda path: path == body


def _exclusions() -> list[tuple[str, object]]:
    return [
        (pattern, _exclusion_matcher(pattern))
        for pattern in _filter_patterns()
        if pattern.startswith("!")
    ]


def _excluded_by(path: str, exclusions) -> str | None:
    for pattern, matches in exclusions:
        if matches(path):
            return pattern
    return None


def _python_path_candidates(source: str) -> set[str]:
    """Repository paths a Python file may open: literals and Path/os.path joins."""
    candidates: set[str] = set()
    try:
        tree = ast.parse(source)
    except SyntaxError:
        return candidates
    # Only the outermost `a / "b" / "c"` chain names the path being opened;
    # its inner sub-chains are directory prefixes, not separate reads.
    inner_chains = {
        id(node.left)
        for node in ast.walk(tree)
        if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Div)
    }
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            value = node.value.strip()
            if "/" in value and not value.startswith(("http", "/", "-", "$")):
                candidates.add(value.strip("/"))
        elif (
            isinstance(node, ast.BinOp)
            and isinstance(node.op, ast.Div)
            and id(node) not in inner_chains
        ):
            chain: list[ast.expr] = []
            current: ast.expr = node
            while isinstance(current, ast.BinOp) and isinstance(current.op, ast.Div):
                chain.append(current.right)
                current = current.left
            chain.append(current)
            segments = [
                str(item.value)
                for item in reversed(chain)
                if isinstance(item, ast.Constant) and isinstance(item.value, str)
            ]
            if len(segments) >= 2:
                candidates.add("/".join(segments).strip("/"))
        elif isinstance(node, ast.Call) and getattr(node.func, "attr", "") == "join":
            segments = [
                str(arg.value)
                for arg in node.args
                if isinstance(arg, ast.Constant) and isinstance(arg.value, str)
            ]
            if len(segments) >= 2:
                candidates.add("/".join(segments).strip("/"))
    return candidates


_QUOTED_RE = re.compile(r'"([^"\n]*)"')
_QUOTED_PATH_RE = re.compile(
    r"(?:^|/)((?:docs|data|examples|tutorials|python|dart|dartsim|tests|scripts)/[^\"]+)"
)


def _text_path_candidates(source: str) -> set[str]:
    """Repository paths inside double-quoted C++/CMake string literals.

    Only quoted strings count so that a comment pointing readers at a design
    doc is not mistaken for a file read; a `${VAR}/docs/...` CMake string and
    a `DART_SOURCE_DIR "/docs/..."` C++ concatenation both resolve to the
    repository-relative path.
    """
    candidates: set[str] = set()
    for quoted in _QUOTED_RE.finditer(source):
        for match in _QUOTED_PATH_RE.finditer(quoted.group(1)):
            candidates.add(match.group(1).rstrip("/ "))
    return candidates


def _skipped_tier_consumers() -> dict[str, set[str]]:
    """Map repository path -> consumers in the platform/wheel tier that reference it."""
    consumers: dict[str, set[str]] = {}

    def record(candidates: set[str], consumer: Path) -> None:
        for candidate in candidates:
            if (ROOT / candidate).exists():
                consumers.setdefault(candidate, set()).add(
                    str(consumer.relative_to(ROOT))
                )

    python_sources = [
        *sorted((ROOT / "python" / "tests").rglob("*.py")),
        *sorted((ROOT / "python" / "examples").rglob("*.py")),
        ROOT / "scripts" / "test_wheel.py",
    ]
    for path in python_sources:
        record(_python_path_candidates(path.read_text(encoding="utf-8")), path)
    cpp_sources = [
        path
        for suffix in ("*.cpp", "*.hpp")
        for path in sorted((ROOT / "tests").rglob(suffix))
    ]
    cmake_sources = [ROOT / "CMakeLists.txt", *sorted((ROOT / "cmake").glob("*.cmake"))]
    for path in cpp_sources + cmake_sources:
        record(
            _text_path_candidates(path.read_text(encoding="utf-8", errors="replace")),
            path,
        )
    return consumers


def test_exclusion_patterns_use_supported_shapes() -> None:
    for pattern, _ in _exclusions():
        assert pattern.startswith("!"), pattern


def test_consumer_scan_sees_known_readers() -> None:
    """Sensitivity check: the scan must keep finding the readers it exists for."""
    consumers = _skipped_tier_consumers()
    for path, reader in (
        ("docs/doxygen/Doxyfile.in", "CMakeLists.txt"),
        (
            "docs/background/lcp/07_selection-guide.md",
            "tests/unit/math/lcp/test_all_solvers_smoke.cpp",
        ),
        ("docs/readthedocs/conf.py", "python/tests/unit/gui/test_gui_scene.py"),
        ("docs/python_api/modules/gui.rst", "python/tests/unit/gui/test_gui_scene.py"),
        (
            "python/examples/demos/README.md",
            "python/tests/integration/test_demos_cycle.py",
        ),
    ):
        assert reader in consumers.get(path, set()), (path, reader, consumers.get(path))


# Excluded paths that a platform-tier test reads, allowed because the same
# validation runs in the docs-only tier (CI Lint) on every PR. Each entry names
# the `check-lint` sub-task that covers it; the test below verifies the task is
# still wired into `check-lint`, so the allowance cannot outlive its reason.
LINT_TIER_COVERED = {
    "docs/onboarding/release-roadmap.md": "check-dart7-clean-break-policy",
}


def _check_lint_tasks() -> set[str]:
    pixi = tomllib.loads((ROOT / "pixi.toml").read_text(encoding="utf-8"))
    tasks = pixi["tasks"]
    seen: set[str] = set()
    pending = ["check-lint"]
    while pending:
        name = pending.pop()
        if name in seen:
            continue
        seen.add(name)
        task = tasks.get(name, {})
        for dependency in task.get("depends-on", []) if isinstance(task, dict) else []:
            pending.append(
                dependency["task"] if isinstance(dependency, dict) else dependency
            )
    return seen


def test_lint_tier_allowances_are_still_covered() -> None:
    lint_tasks = _check_lint_tasks()
    for path, task in LINT_TIER_COVERED.items():
        assert (ROOT / path).exists(), path
        assert (
            task in lint_tasks
        ), f"{task} no longer runs under check-lint; drop the allowance for {path}"


def test_no_excluded_path_is_consumed_by_the_skipped_tier() -> None:
    """Every file the platform tests, demos, C++ tests, or CMake read stays code.

    A PR that only touches an excluded path skips the platform test jobs, the
    install steps, and the wheel builds; if one of those reads the path, a
    regression can merge green. The scan is deliberately conservative: a
    string that names an existing repository path counts as a read.
    """
    exclusions = _exclusions()
    offenders = {
        path: (pattern, sorted(consumers))
        for path, consumers in _skipped_tier_consumers().items()
        if path not in LINT_TIER_COVERED
        and (pattern := _excluded_by(path, exclusions)) is not None
    }
    assert not offenders, "\n".join(
        f"{path} is excluded by {pattern} but referenced by {consumers}"
        for path, (pattern, consumers) in sorted(offenders.items())
    )


def test_review_gate_changes_reach_native_windows_job() -> None:
    """The isolated native job must run when any review-gate input changes."""
    workflow = (WORKFLOW_DIR / "ci_windows.yml").read_text(encoding="utf-8")
    hook_filter = workflow.split("            hooks:\n", 1)[1].split(
        "\n  hook-smoke:", 1
    )[0]
    patterns = _filter_patterns(hook_filter)
    for consumer_input in (
        "scripts/review_gate.py",
        "scripts/install_git_hooks.py",
        "tests/test_review_gate.py",
        "scripts/run_pytest.py",
        "pixi.toml",
        "pixi.lock",
        "pyproject.toml",
        ".github/actions/setup-pixi-ci/action.yml",
        ".github/workflows/ci_windows.yml",
    ):
        assert any(fnmatchcase(consumer_input, pattern) for pattern in patterns), (
            consumer_input,
            patterns,
        )
    native_job = workflow.split("\n  hook-smoke:\n", 1)[1].split("\n  build:\n", 1)[0]
    assert "needs.changes.outputs.hooks == 'true'" in native_job
    assert (
        "pixi run python -I scripts/run_pytest.py tests/test_review_gate.py -q"
        in native_job
    )
