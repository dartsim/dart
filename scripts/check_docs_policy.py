#!/usr/bin/env python3
"""Enforce repository documentation content policies."""

from __future__ import annotations

import ast
import fnmatch
import functools
import json
import re
import subprocess
import sys
import unicodedata
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import unquote

try:
    from markdown_it import MarkdownIt
except ImportError as exc:  # pragma: no cover - environment guard
    raise SystemExit(
        "check_docs_policy.py needs markdown-it-py (run it through "
        "`pixi run check-docs-policy`)"
    ) from exc

SKIP_DIRS = {".deps", ".git", ".pixi", "build", "external", "node_modules"}
GIT_QUERY_ERRORS = (OSError, subprocess.CalledProcessError)
REQUIRED_DOCS_TOP_LEVEL_DIRS = (
    "ai",
    "assets",
    "background",
    "design",
    "dev_tasks",
    "doxygen",
    "onboarding",
    "plans",
    "python_api",
    "readthedocs",
)
DOCS_PLACEMENT_OWNER = "docs/README.md"
DOCS_PLACEMENT_HEADING = "## Where Docs Belong"
DEV_TASK_SIZE_ADVISORIES = {"RESUME.md": 200, "README.md": 400}

# These strings have shown up as "footer metadata" in Markdown docs and should
# not be committed to the repository.
FORBIDDEN_FOOTER_MARKERS = (
    "**Generated**:",
    "**Project Version**:",
    "**Maintainer**:",
    "**Document Generated:**",
    "**DART Version:**",
    "**Analysis Complete**",
)

PLAN_FILE_RE = re.compile(r"^\d{3}-.+\.md$")
PLAN_BLOCK_RE = re.compile(
    r"^### (?P<id>PLAN-\d{3}):.*?(?=^### PLAN-\d{3}:|\Z)",
    re.MULTILINE | re.DOTALL,
)
DASHBOARD_FIELD_RE = re.compile(
    r"^- (Status|Horizon|Dimension|Next step|Gate):", re.MULTILINE
)
DESIGN_DASHBOARD_FIELD_RE = re.compile(
    r"^- (Priority|Horizon|Next step|Gate):", re.MULTILINE
)
DASHBOARD_REQUIRED_FIELDS = (
    "owner",
    "status",
    "horizon",
    "dimension",
    "next_step",
    "gate",
)
DASHBOARD_STATUS_VALUES = {"Proposed", "Active", "Blocked", "Complete", "Parked"}
DASHBOARD_HORIZON_VALUES = {"Now", "Next", "Later", "Parked"}
DASHBOARD_ENTRY_MAX_LINES = 40
DASHBOARD_NEXT_STEP_MAX_LINES = 15
DOCS_AI_FRONTMATTER_FILES = {
    "README.md",
    "components.md",
    "north-star.md",
    "orchestration.md",
    "principles.md",
    "terminology.md",
    "verification.md",
    "workflows.md",
}
DOCS_AI_FRONTMATTER_KEYS = {"type", "owner"}
DOCS_AI_TYPE_LEGEND = {
    "ai-principles",
    "ai-entrypoint",
    "ai-component-policy",
    "ai-verification-policy",
    "ai-workflow-map",
    "ai-operating-model",
    "ai-north-star",
    "ai-terminology",
}
DISCOVERABILITY_INDEXES = {
    "docs/ai": "docs/ai/README.md",
    "docs/background": "docs/background/README.md",
    "docs/onboarding": "docs/onboarding/README.md",
}
MARKDOWN_LINK_PATTERNS = (":(glob)*.md", ":(glob)**/*.md")
# Docs that must be reachable from another tracked doc, script, test, or
# workflow; a doc nobody references is dead weight (docs/README.md).
ORPHAN_DOC_PATTERNS = (":(glob)docs/**/*.md",)
ORPHAN_SIDECAR_PATTERNS = (":(glob)docs/plans/*/*",)
ORPHAN_EXCLUDED_PREFIXES = ("docs/readthedocs/",)
# Root surfaces discovered by convention rather than by link: the docs map,
# one index/rule sheet per bucket, and dev-task entrypoints
# (docs/dev_tasks/README.md). Deeper README/AGENTS files must be reachable.
ORPHAN_INDEX_NAMES = {"README.md", "AGENTS.md"}
ORPHAN_DEV_TASK_ENTRYPOINTS = {"README.md", "RESUME.md"}
ORPHAN_REFERENCE_BOUNDARY_BEFORE = re.compile(r"[A-Za-z0-9_.\-]")
ORPHAN_REFERENCE_BOUNDARY_AFTER = re.compile(r"[A-Za-z0-9_\-]")
ORPHAN_CORPUS_PATTERNS = (
    ":(glob)*.md",
    ":(glob)docs/**/*.md",
    ":(glob)docs/**/*.rst",
    ":(glob)docs/**/*.py",
    ":(glob)**/AGENTS.md",
    ":(glob).claude/**/*.md",
    ":(glob).codex/**/*",
    ":(glob).github/**/*",
    ":(glob)scripts/**/*.py",
    ":(glob)tests/**/*.py",
    ":(glob)python/**/*.py",
    "pixi.toml",
)
NORTH_STAR_FRESHNESS_MARKER_RE = re.compile(
    r"<!--\s*docs-policy:\s*evidence-last-verified=(?P<date>\d{4}-\d{2}-\d{2})\s*-->"
)
PAPERS_CLOSED_VALUE_FIELDS = ("Type", "Status", "Priority", "Verdict")
# CommonMark plus GFM tables, so links inside table cells are seen and code
# blocks, inline code, and HTML are never mistaken for links or headings.
MARKDOWN_PARSER = MarkdownIt("commonmark").enable("table")


@functools.lru_cache(maxsize=None)
def _parse_markdown(text: str):
    """Parse once per distinct text; every pass over a page reuses the tokens."""
    return MARKDOWN_PARSER.parse(text)


@functools.lru_cache(maxsize=None)
def _read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


# MyST directives whose body is literal (code, raw markup, or Sphinx syntax),
# not Markdown; every other directive body is parsed for links.
MYST_LITERAL_DIRECTIVES = {
    "code",
    "code-block",
    "code-cell",
    "sourcecode",
    "literalinclude",
    "raw",
    "eval-rst",
    "math",
    "mermaid",
    "include",
    "toctree",
    "highlight",
    "parsed-literal",
    "glossary",
}
MYST_DIRECTIVE_RE = re.compile(r"^\{(?P<name>[A-Za-z][A-Za-z0-9_-]*)\}")


def iter_markdown_files(repo_root: Path) -> list[Path]:
    return iter_tracked_files(repo_root, ["*.md"])


def iter_tracked_files(repo_root: Path, patterns: list[str]) -> list[Path]:
    """Return git-tracked files for the requested pathspecs.

    The fallback keeps unit tests and source archives usable outside a git
    checkout, while the normal path avoids linting ignored local research notes.
    """
    try:
        result = subprocess.run(
            [
                "git",
                "ls-files",
                "-z",  # NUL-delimited: no C-style quoting of non-ASCII names
                "--cached",
                "--others",
                "--exclude-standard",
                *patterns,
            ],
            cwd=repo_root,
            check=True,
            capture_output=True,
            text=True,
        )
    except GIT_QUERY_ERRORS:
        files: list[Path] = []
        for pattern in patterns:
            fallback_pattern = pattern.removeprefix(":(glob)")
            for path in repo_root.glob(fallback_pattern):
                if any(part in SKIP_DIRS for part in path.parts):
                    continue
                files.append(path)
        return sorted(set(files))

    files = []
    for line in result.stdout.split("\0"):
        if not line:
            continue
        path = repo_root / line
        if path.exists():
            files.append(path)
    return sorted(files)


def _display_path(path: Path, repo_root: Path) -> str:
    """Repo-relative path in POSIX form on every platform, so prefix checks
    and corpus keys match the forward-slash paths docs and pathspecs use."""
    try:
        return path.relative_to(repo_root).as_posix()
    except ValueError:
        return str(path)


def check_file(path: Path, repo_root: Path) -> list[str]:
    rel_path = str(path.relative_to(repo_root))
    text = path.read_text(encoding="utf-8", errors="replace")
    lines = text.splitlines()

    failures: list[str] = []

    # Footer marker checks: only scan the tail to avoid false positives in body
    # text (e.g., "generated by X" as part of an explanation).
    tail_len = min(len(lines), 80)
    tail_offset = len(lines) - tail_len
    for index, line in enumerate(lines[tail_offset:], start=tail_offset + 1):
        for marker in FORBIDDEN_FOOTER_MARKERS:
            if marker in line:
                failures.append(
                    f"{rel_path}:{index}: disallowed footer marker {marker!r}"
                )
                break

    return failures


def check_docs_indexes(repo_root: Path) -> list[str]:
    """Ensure tracked top-level docs buckets stay visible from docs indexes."""
    failures: list[str] = []
    docs_readme = repo_root / "docs" / "README.md"
    docs_agents = repo_root / "docs" / "AGENTS.md"

    for path in (docs_readme, docs_agents):
        if not path.exists():
            failures.append(f"{path.relative_to(repo_root)}: missing docs index")
    if docs_readme.exists():
        text = docs_readme.read_text(encoding="utf-8", errors="replace")
        for directory in REQUIRED_DOCS_TOP_LEVEL_DIRS:
            marker = f"{directory}/"
            if marker not in text:
                failures.append(
                    f"{docs_readme.relative_to(repo_root)}: missing `{marker}` entry"
                )

    return failures


def check_docs_placement_owner(repo_root: Path) -> list[str]:
    """Ensure the docs placement owner exists and stays discoverable.

    ``docs/README.md`` owns the bucket map and placement matrix; the root and
    docs-local ``AGENTS.md`` entrypoints must point agents at it.
    """
    failures: list[str] = []
    owner = repo_root / DOCS_PLACEMENT_OWNER
    if not owner.exists():
        return [f"{DOCS_PLACEMENT_OWNER}: missing docs placement owner"]
    owner_text = owner.read_text(encoding="utf-8", errors="replace")
    if DOCS_PLACEMENT_HEADING not in owner_text:
        failures.append(
            f"{DOCS_PLACEMENT_OWNER}: missing `{DOCS_PLACEMENT_HEADING}` "
            "placement section"
        )

    for rel_path in ("AGENTS.md", "docs/AGENTS.md"):
        path = repo_root / rel_path
        if not path.exists():
            failures.append(f"{rel_path}: missing docs index")
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        if DOCS_PLACEMENT_OWNER not in text:
            failures.append(f"{rel_path}: missing `{DOCS_PLACEMENT_OWNER}` pointer")

    return failures


def check_dev_task_shape(repo_root: Path) -> list[str]:
    """Ensure active dev-task folders include status and resume context."""
    failures: list[str] = []
    dev_tasks_dir = repo_root / "docs" / "dev_tasks"
    if not dev_tasks_dir.exists():
        return failures

    for task_dir in sorted(path for path in dev_tasks_dir.iterdir() if path.is_dir()):
        for required in ("README.md", "RESUME.md"):
            required_path = task_dir / required
            if not required_path.exists():
                failures.append(
                    f"{task_dir.relative_to(repo_root)}: missing required {required}"
                )

    return failures


def check_dev_task_size(repo_root: Path) -> list[str]:
    """Report dev-task snapshot files that have grown into logs (advisory)."""
    warnings: list[str] = []
    dev_tasks_dir = repo_root / "docs" / "dev_tasks"
    if not dev_tasks_dir.exists():
        return warnings

    for task_dir in sorted(path for path in dev_tasks_dir.iterdir() if path.is_dir()):
        for name, budget in DEV_TASK_SIZE_ADVISORIES.items():
            path = task_dir / name
            if not path.exists():
                continue
            line_count = len(
                path.read_text(encoding="utf-8", errors="replace").splitlines()
            )
            if line_count > budget:
                warnings.append(
                    f"{path.relative_to(repo_root)}: {line_count} lines exceeds the "
                    f"{budget}-line snapshot budget; prune history or promote "
                    "durable content (docs/dev_tasks/README.md)"
                )
    return warnings


def _dashboard_entries(text: str) -> list[dict[str, str]]:
    entries: list[dict[str, str]] = []
    for match in PLAN_BLOCK_RE.finditer(text):
        block = match.group(0)
        status_match = re.search(r"^- Status:\s*(?P<status>.+)$", block, re.MULTILINE)
        horizon_match = re.search(
            r"^- Horizon:\s*(?P<horizon>.+)$", block, re.MULTILINE
        )
        dimension_match = re.search(
            r"^- Dimension:\s*(?P<dimension>.+)$", block, re.MULTILINE
        )
        next_step_match = re.search(
            r"^- Next step:\s*(?P<next_step>.+)$", block, re.MULTILINE
        )
        gate_match = re.search(r"^- Gate:\s*(?P<gate>.+)$", block, re.MULTILINE)
        owner_match = re.search(
            r"- Owner doc:.*?\]\((?P<owner>[^)]+)\)",
            block,
            re.DOTALL,
        )
        entries.append(
            {
                "id": match.group("id"),
                "status": status_match.group("status").strip() if status_match else "",
                "horizon": (
                    horizon_match.group("horizon").strip() if horizon_match else ""
                ),
                "dimension": (
                    dimension_match.group("dimension").strip()
                    if dimension_match
                    else ""
                ),
                "next_step": (
                    next_step_match.group("next_step").strip()
                    if next_step_match
                    else ""
                ),
                "gate": gate_match.group("gate").strip() if gate_match else "",
                "owner": owner_match.group("owner").strip() if owner_match else "",
            }
        )
    return entries


def _normalize_plan_owner(owner: str) -> str:
    return owner.split("#", maxsplit=1)[0].strip()


URI_SCHEME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9+.\-]*:")


def _is_external_link(link: str) -> bool:
    """Any RFC 3986 scheme (`https:`, `mailto:`, `urn:`, `ssh:`) or a
    scheme-relative `//host/path` destination is external."""
    return bool(URI_SCHEME_RE.match(link)) or link.startswith("//")


def _strip_markdown_link_target(link: str) -> str:
    """Return the destination of a parser-provided link (spaces are literal)."""
    target = link.strip()
    if target.startswith("<") and ">" in target:
        return target[1 : target.index(">")]
    return target


def _normalize_markdown_link(link: str) -> str:
    """Return the decoded filesystem path of a link destination.

    The query and fragment are split off first so a percent-encoded reserved
    character in the path (``guide%23v2.md``) decodes to the literal filename
    instead of being mistaken for a delimiter.
    """
    target = _strip_markdown_link_target(link)
    target = target.split("?", maxsplit=1)[0]
    return unquote(target.split("#", maxsplit=1)[0].strip())


def _resolve_dashboard_owner(
    owner: str, repo_root: Path, plans_dir: Path
) -> Path | None:
    target = _normalize_plan_owner(owner)
    if not target or _is_external_link(target):
        return None

    target_path = Path(target)
    if target_path.is_absolute():
        return target_path.resolve()

    if target_path.parts[:2] == ("docs", "plans"):
        return (repo_root / target_path).resolve()
    if target_path.parts[:1] == ("plans",):
        return (repo_root / "docs" / target_path).resolve()
    return (plans_dir / target_path).resolve()


def _direct_numbered_plan_file(owner_path: Path, plans_dir: Path) -> str | None:
    try:
        rel_path = owner_path.relative_to(plans_dir.resolve())
    except ValueError:
        return None

    if len(rel_path.parts) == 1 and PLAN_FILE_RE.match(rel_path.name):
        return rel_path.name
    return None


def _resolve_markdown_link(
    link: str,
    base_dir: Path,
    repo_root: Path | None = None,
    current_file: Path | None = None,
) -> Path | None:
    target = _normalize_markdown_link(link)
    raw_target = _strip_markdown_link_target(link)
    if raw_target.startswith("#"):
        return current_file.resolve() if current_file else base_dir.resolve()
    if not target or _is_external_link(target):
        return None

    if target.startswith("/"):
        if repo_root is None:
            return Path(target).resolve()
        return (repo_root / target.lstrip("/")).resolve()

    target_path = Path(target)
    if target_path.is_absolute():
        return target_path.resolve()
    # Markdown semantics only: a relative destination resolves against the
    # referring file's directory, exactly as GitHub renders it. A destination
    # that only exists from the repository root is a broken link.
    return (base_dir / target_path).resolve()


def _iter_block_tokens(text: str, offset: int = 0):
    """Yield (line, token) pairs for every block-level token, recursing into
    MyST directive fences whose body is Markdown (admonitions, grids, ...).
    Literal directives and ordinary code fences are yielded but not entered."""
    for token in _parse_markdown(text):
        line_number = offset + ((token.map[0] + 1) if token.map else 0)
        directive = (
            MYST_DIRECTIVE_RE.match(token.info.strip())
            if token.type == "fence"
            else None
        )
        if directive and directive.group("name") not in MYST_LITERAL_DIRECTIVES:
            yield from _iter_block_tokens(token.content, offset=line_number)
            continue
        yield line_number, token


def _iter_inline_tokens(text: str, offset: int = 0):
    """Yield (line, inline token) pairs, including those inside
    Markdown-bearing MyST directives."""
    for line_number, token in _iter_block_tokens(text, offset=offset):
        if token.type == "inline" and token.children:
            yield line_number, token


class _HrefCollector(HTMLParser):
    """Collect `href` attributes and `id`/`name` anchors (any tag, any
    quoting, any case)."""

    def __init__(self) -> None:
        super().__init__()
        self.hrefs: list[str] = []
        self.anchors: set[str] = set()

    def handle_starttag(self, tag, attrs):  # noqa: ARG002 - HTMLParser API
        for name, value in attrs:
            if not value:
                continue
            if name.lower() == "href":
                self.hrefs.append(value)
            elif name.lower() in {"id", "name"}:
                self.anchors.add(value)


def _html_hrefs(fragment: str) -> list[str]:
    collector = _HrefCollector()
    collector.feed(fragment)
    collector.close()
    return collector.hrefs


def _html_anchor_ids(text: str) -> set[str]:
    """Return `id`/`name` anchors declared in raw HTML within ``text``."""
    collector = _HrefCollector()
    for _, token in _iter_block_tokens(text):
        if token.type == "html_block":
            collector.feed(token.content)
        elif token.type == "inline" and token.children:
            for child in token.children:
                if child.type == "html_inline":
                    collector.feed(child.content)
    collector.close()
    return collector.anchors


@functools.lru_cache(maxsize=None)
def _page_anchors(page: Path) -> set[str]:
    """Anchors a Markdown page defines: heading ids plus HTML id/name.

    Pages under a Sphinx source root get docutils section ids (what the
    published site renders); repository-local Markdown gets GitHub slugs.
    """
    sphinx = _sphinx_source_root(page, None) is not None
    return _markdown_heading_anchors(page, docutils=sphinx) | _html_anchor_ids(
        _read_text(page)
    )


TOCTREE_ENTRY_RE = re.compile(r"^(?:.*<(?P<bracketed>[^>]+)>|(?P<plain>\S.*?))\s*$")


def _iter_html_hrefs(text: str) -> list[tuple[int, str]]:
    """Return (line, href) pairs from raw HTML blocks and inline HTML."""
    hrefs: list[tuple[int, str]] = []
    for line_number, token in _iter_block_tokens(text):
        fragments: list[str] = []
        if token.type == "html_block":
            fragments.append(token.content)
        elif token.type == "inline" and token.children:
            fragments.extend(
                child.content for child in token.children if child.type == "html_inline"
            )
        for fragment in fragments:
            hrefs.extend((line_number, href) for href in _html_hrefs(fragment))
    return hrefs


def _iter_toctree_entries(text: str) -> list[tuple[int, str, bool]]:
    """Return (line, target, is_glob) triples for MyST ``{toctree}`` entries.

    Option lines (``:maxdepth: 1``), blank lines, comments, ``self``, and
    external URLs are skipped; ``Title <target>`` yields the target; under
    ``:glob:`` a wildcard entry is flagged so the caller expands it.
    """
    entries: list[tuple[int, str, bool]] = []
    for line_number, token in _iter_block_tokens(text):
        if token.type != "fence":
            continue
        directive = MYST_DIRECTIVE_RE.match(token.info.strip())
        if not directive or directive.group("name") != "toctree":
            continue
        lines = token.content.splitlines()
        glob_mode = any(line.strip() == ":glob:" for line in lines)
        for index, raw in enumerate(lines, start=1):
            line = raw.strip()
            if not line or line.startswith((":", "%", "<!--")):
                continue
            match = TOCTREE_ENTRY_RE.match(line)
            target = (match.group("bracketed") or match.group("plain")).strip()
            if target == "self" or _is_external_link(target):
                continue  # `self` is Sphinx's special entry for the current page
            is_glob = glob_mode and any(character in target for character in "*?[")
            entries.append((line_number + index, target, is_glob))
    return entries


def _resolve_built_html_href(href: str, path: Path, source_root: Path) -> Path | None:
    """Resolve a raw HTML ``href`` on a Sphinx page to the source page it
    renders (``x.html`` -> ``x.md``/``x.rst``, ``x/`` -> ``x/index.*``) or to
    an existing file such as a static asset; None when nothing matches."""
    base = source_root if href.startswith("/") else path.parent
    target = href.lstrip("/")
    root = source_root.resolve()
    prefix = "/" if href.startswith("/") else ""
    if target.endswith(".html"):
        return _resolve_myst_doc_role(
            prefix + target[: -len(".html")], path, source_root
        )
    if target.endswith("/"):
        return _resolve_myst_doc_role(prefix + target + "index", path, source_root)
    resolved = (base / target).resolve()
    if resolved.exists() and resolved.is_relative_to(root):
        return resolved
    return None


def _iter_markdown_links(text: str) -> list[tuple[int, str]]:
    """Return (line, destination) pairs for every Markdown link in ``text``.

    The CommonMark parser resolves reference-style links, keeps balanced
    parentheses in destinations, and never yields link-shaped text inside
    fenced or indented code, inline code, or HTML. Images are not links.
    """
    links: list[tuple[int, str]] = []
    for line_number, token in _iter_inline_tokens(text):
        for child in token.children:
            if child.type == "link_open":
                href = child.attrGet("href")
                if href:
                    # Keep markdown-it's percent-encoded form; the resolver
                    # decodes path and fragment separately.
                    links.append((line_number, str(href)))
    return links


MYST_DOC_ROLE_SUFFIX = "{doc}"
SPHINX_SOURCE_SUFFIXES = (".md", ".rst")


def _iter_myst_doc_roles(text: str) -> list[tuple[int, str]]:
    """Return (line, target) pairs for MyST ``{doc}`` cross-references.

    markdown-it sees ``{doc}`` as text followed by an inline code span; the
    span holds either ``target`` or ``Label <target>``. Roles inside
    Markdown-bearing directives are included.
    """
    roles: list[tuple[int, str]] = []
    for line_number, token in _iter_inline_tokens(text):
        children = token.children
        for index, child in enumerate(children[:-1]):
            if child.type != "text" or not child.content.endswith(MYST_DOC_ROLE_SUFFIX):
                continue
            span = children[index + 1]
            if span.type != "code_inline":
                continue
            target = span.content.strip()
            if target.endswith(">") and "<" in target:
                target = target[target.rindex("<") + 1 : -1].strip()
            if target:
                roles.append((line_number, target))
    return roles


@functools.lru_cache(maxsize=None)
def _sphinx_source_root(path: Path, repo_root: Path | None) -> Path | None:
    """Return the nearest ancestor holding ``conf.py`` (the Sphinx srcdir)."""
    for ancestor in path.parents:
        if (ancestor / "conf.py").is_file():
            return ancestor
        if repo_root is not None and ancestor == repo_root:
            break
    return None


@functools.lru_cache(maxsize=None)
def _sphinx_exclude_patterns(source_root: Path) -> tuple[str, ...]:
    """Read ``exclude_patterns`` from ``conf.py`` without executing it."""
    try:
        tree = ast.parse(_read_text(source_root / "conf.py"))
    except OSError, SyntaxError, ValueError:
        return ()
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        if not any(
            isinstance(target, ast.Name) and target.id == "exclude_patterns"
            for target in node.targets
        ):
            continue
        try:
            value = ast.literal_eval(node.value)
        except ValueError:
            return ()
        if isinstance(value, (list, tuple)):
            return tuple(str(item) for item in value)
    return ()


def _is_sphinx_excluded(page: Path, source_root: Path) -> bool:
    """Whether Sphinx drops ``page`` from its document set via exclude_patterns."""
    try:
        rel = page.resolve().relative_to(source_root.resolve()).as_posix()
    except ValueError:
        return True
    for pattern in _sphinx_exclude_patterns(source_root):
        pattern = pattern.rstrip("/")
        if rel == pattern or rel.startswith(pattern + "/"):
            return True
        if fnmatch.fnmatch(rel, pattern) or fnmatch.fnmatch(rel, pattern + "/*"):
            return True
    return False


def _resolve_myst_doc_role(target: str, path: Path, source_root: Path) -> Path | None:
    """Resolve a ``{doc}`` target to an existing page, or None when missing."""
    base = source_root if target.startswith("/") else path.parent
    stem = base / target.lstrip("/")
    candidates = [stem] if stem.suffix in SPHINX_SOURCE_SUFFIXES else []
    candidates.extend(
        stem.with_name(stem.name + suffix) for suffix in SPHINX_SOURCE_SUFFIXES
    )
    root = source_root.resolve()
    for candidate in candidates:
        resolved = candidate.resolve()
        # Sphinx only registers documents under its source root; a target that
        # escapes it (`../../README`) or matches `exclude_patterns` is not a
        # page even if the file exists.
        if (
            resolved.is_file()
            and resolved.is_relative_to(root)
            and not _is_sphinx_excluded(resolved, source_root)
        ):
            return resolved
    return None


def _expand_toctree_glob(pattern: str, path: Path, source_root: Path) -> list[Path]:
    """Pages a ``:glob:`` toctree pattern matches, in Sphinx's document set."""
    base = source_root if pattern.startswith("/") else path.parent
    relative = pattern.lstrip("/")
    matches: list[Path] = []
    for suffix in ("", *SPHINX_SOURCE_SUFFIXES):
        for candidate in base.glob(relative + suffix):
            if (
                candidate.suffix not in SPHINX_SOURCE_SUFFIXES
                or not candidate.is_file()
            ):
                continue
            resolved = candidate.resolve()
            if resolved == path.resolve() or _is_sphinx_excluded(resolved, source_root):
                continue
            if resolved not in matches:
                matches.append(resolved)
    return matches


def _github_heading_slug(heading: str) -> str:
    """Apply GitHub's heading-to-anchor slug rules to rendered heading text."""
    text = heading.strip().lower()
    text = re.sub(r"[^\w\- ]", "", text)
    return text.replace(" ", "-")


def _inline_plain_text(token) -> str:
    """Return the text a reader sees for an inline token: text and code
    content, with line breaks as spaces and link/emphasis markup removed."""
    parts: list[str] = []
    for child in token.children or []:
        if child.type in {"text", "code_inline"}:
            parts.append(child.content)
        elif child.type in {"softbreak", "hardbreak"}:
            parts.append(" ")
        elif child.children:
            parts.append(_inline_plain_text(child))
    return "".join(parts)


def _docutils_id(text: str) -> str:
    """Approximate docutils ``make_id``: the section id Sphinx renders."""
    ascii_text = (
        unicodedata.normalize("NFKD", text).encode("ascii", "ignore").decode("ascii")
    )
    identifier = re.sub(r"[^a-z0-9]+", "-", ascii_text.lower())
    identifier = re.sub(r"^[-0-9]+|-+$", "", identifier)
    return identifier or "id"


def _markdown_heading_anchors(path: Path, docutils: bool = False) -> set[str]:
    """Return the anchors a markdown file's headings generate.

    ATX and Setext headings both count. GitHub (default) reserves taken slugs
    so `Foo`, `Foo-1`, `Foo` yields `foo`, `foo-1`, `foo-2`; docutils ids
    (``docutils=True``) follow the same reservation with docutils normalization.
    """
    anchors: set[str] = set()
    occurrences: dict[str, int] = {}
    tokens = _parse_markdown(_read_text(path))
    slugify = _docutils_id if docutils else _github_heading_slug
    for index, token in enumerate(tokens):
        if token.type != "heading_open" or index + 1 >= len(tokens):
            continue
        original = slugify(_inline_plain_text(tokens[index + 1]))
        slug = original
        while slug in anchors:
            occurrences[original] = occurrences.get(original, 0) + 1
            slug = f"{original}-{occurrences[original]}"
        anchors.add(slug)
    return anchors


def check_plan_id_uniqueness(entries: list[dict[str, str]]) -> list[str]:
    """PLAN-091 WP-091.5: each PLAN-ID identifies exactly one plan block.

    A colliding ID (two initiatives sharing one ``PLAN-NNN``) makes grep-by-ID
    ambiguous and breaks the packet-execution harness, so reject it. Entries
    carry a ``source`` file label so collisions across the dashboard and the
    archive (for example a stale dashboard copy left behind by a partial
    archive move) are rejected, not just same-file duplicates.
    """
    counts: dict[str, int] = {}
    sources: dict[str, list[str]] = {}
    order: list[str] = []
    for entry in entries:
        plan_id = entry["id"]
        if plan_id not in counts:
            order.append(plan_id)
            sources[plan_id] = []
        counts[plan_id] = counts.get(plan_id, 0) + 1
        sources[plan_id].append(entry.get("source", "docs/plans/dashboard.md"))
    failures: list[str] = []
    for plan_id in order:
        if counts[plan_id] > 1:
            where = ", ".join(sorted(set(sources[plan_id])))
            failures.append(
                f"{where}: "
                f"{plan_id} identifies {counts[plan_id]} plan blocks; each "
                "PLAN-ID must identify exactly one initiative (renumber the "
                "colliding entries onto fresh IDs, or remove the stale copy)"
            )
    return failures


def check_plan_lifecycle(repo_root: Path) -> list[str]:
    """Ensure living plans stay current instead of becoming archival state."""
    failures: list[str] = []
    plans_dir = repo_root / "docs" / "plans"
    dashboard = plans_dir / "dashboard.md"
    if not plans_dir.exists() or not dashboard.exists():
        return failures

    plan_files = {
        path.name for path in plans_dir.glob("*.md") if PLAN_FILE_RE.match(path.name)
    }
    dashboard_text = dashboard.read_text(encoding="utf-8", errors="replace")
    entries = _dashboard_entries(dashboard_text)
    unique_entries: list[dict[str, str]] = list(entries)
    archive = plans_dir / "archive.md"
    if archive.exists():
        archive_text = archive.read_text(encoding="utf-8", errors="replace")
        for match in PLAN_BLOCK_RE.finditer(archive_text):
            unique_entries.append(
                {"id": match.group("id"), "source": "docs/plans/archive.md"}
            )
    failures.extend(check_plan_id_uniqueness(unique_entries))

    referenced_plan_files: set[str] = set()
    for entry in entries:
        for field in DASHBOARD_REQUIRED_FIELDS:
            if not entry[field]:
                failures.append(
                    "docs/plans/dashboard.md: "
                    f"{entry['id']} is missing required `{field}` field"
                )
        if entry["status"] and entry["status"] not in DASHBOARD_STATUS_VALUES:
            failures.append(
                "docs/plans/dashboard.md: "
                f"{entry['id']} has unknown status `{entry['status']}`"
            )
        if entry["horizon"] and entry["horizon"] not in DASHBOARD_HORIZON_VALUES:
            failures.append(
                "docs/plans/dashboard.md: "
                f"{entry['id']} has unknown horizon `{entry['horizon']}`"
            )
        owner = _normalize_plan_owner(entry["owner"])
        owner_path = _resolve_dashboard_owner(owner, repo_root, plans_dir)
        if owner_path:
            try:
                owner_path.relative_to(repo_root.resolve())
            except ValueError:
                failures.append(
                    "docs/plans/dashboard.md: "
                    f"{entry['id']} owner doc escapes repository: `{owner}`"
                )
            else:
                if not owner_path.exists():
                    failures.append(
                        "docs/plans/dashboard.md: "
                        f"{entry['id']} owner doc does not exist: `{owner}`"
                    )
        owner_plan_file = (
            _direct_numbered_plan_file(owner_path, plans_dir) if owner_path else None
        )
        if owner_plan_file:
            referenced_plan_files.add(owner_plan_file)
            if entry["status"] == "Complete":
                failures.append(
                    "docs/plans/dashboard.md: completed "
                    f"{entry['id']} still points to numbered plan file "
                    f"`{owner}`; move durable output to its owner doc and "
                    "retarget or remove the plan entry"
                )

    for plan_file in sorted(plan_files - referenced_plan_files):
        failures.append(
            f"docs/plans/{plan_file}: numbered plan file is not referenced "
            "from docs/plans/dashboard.md"
        )

    for plan_file in sorted(plan_files):
        path = plans_dir / plan_file
        text = path.read_text(encoding="utf-8", errors="replace")
        repeated_field = DASHBOARD_FIELD_RE.search(text)
        if repeated_field:
            line = text.count("\n", 0, repeated_field.start()) + 1
            failures.append(
                f"docs/plans/{plan_file}:{line}: plan file repeats dashboard "
                f"field `{repeated_field.group(1)}`"
            )

    return failures


def _dashboard_next_step_line_count(block: str) -> int | None:
    """Count the lines in a dashboard entry's ``- Next step:`` field.

    The field spans from the ``- Next step:`` bullet to the next top-level
    ``- Field:`` bullet (or the end of the block), trailing blanks excluded.
    """
    lines = block.splitlines()
    start = None
    for index, line in enumerate(lines):
        if line.startswith("- Next step:"):
            start = index
            break
    if start is None:
        return None
    end = len(lines)
    for index in range(start + 1, len(lines)):
        if lines[index].startswith("- "):
            end = index
            break
    span = lines[start:end]
    while span and not span[-1].strip():
        span.pop()
    return len(span)


def check_dashboard_structure(repo_root: Path) -> list[str]:
    """Keep the plan dashboard a bounded operating view.

    Each ``### PLAN-`` entry stays within a line budget, its ``- Next step:``
    field stays short, and completed plans move to ``docs/plans/archive.md``
    instead of accumulating in the dashboard.
    """
    failures: list[str] = []
    dashboard = repo_root / "docs" / "plans" / "dashboard.md"
    if not dashboard.exists():
        return failures

    text = dashboard.read_text(encoding="utf-8", errors="replace")
    failures.extend(_malformed_plan_headings(text, "docs/plans/dashboard.md"))
    for match in PLAN_BLOCK_RE.finditer(text):
        plan_id = match.group("id")
        block = match.group(0)

        entry_lines = len(block.rstrip().splitlines())
        if entry_lines > DASHBOARD_ENTRY_MAX_LINES:
            failures.append(
                "docs/plans/dashboard.md: "
                f"{plan_id} entry is {entry_lines} lines; keep each dashboard "
                f"entry to at most {DASHBOARD_ENTRY_MAX_LINES} lines (move the "
                "history to the owner plan file's `## Progress log` section)"
            )

        status_match = re.search(r"^- Status:\s*(?P<status>.+)$", block, re.MULTILINE)
        if status_match and status_match.group("status").strip() == "Complete":
            failures.append(
                "docs/plans/dashboard.md: "
                f"{plan_id} has `Status: Complete`; move the entry to "
                "docs/plans/archive.md (the dashboard shows only operating plans)"
            )

        next_step_lines = _dashboard_next_step_line_count(block)
        if (
            next_step_lines is not None
            and next_step_lines > DASHBOARD_NEXT_STEP_MAX_LINES
        ):
            failures.append(
                "docs/plans/dashboard.md: "
                f"{plan_id} `Next step` field is {next_step_lines} lines; keep "
                f"it to at most {DASHBOARD_NEXT_STEP_MAX_LINES} lines (state only "
                "the current action and relocate history to the owner plan "
                "file's `## Progress log` section)"
            )

    return failures


def _malformed_plan_headings(text: str, rel_path: str) -> list[str]:
    """Reject plan-block headings that PLAN_BLOCK_RE cannot see.

    Every PLAN rule keys on the strict ``### PLAN-NNN: Title`` heading shape.
    A near-miss heading (wrong level, separator, digit count, or spacing)
    would otherwise make the whole entry invisible to those rules, so flag it
    instead of silently skipping it.
    """
    failures: list[str] = []
    strict = re.compile(r"^### PLAN-\d{3}: \S")
    loose = re.compile(r"^#{1,6}\s+PLAN-")
    for line_number, line in enumerate(text.splitlines(), start=1):
        if loose.match(line) and not strict.match(line):
            failures.append(
                f"{rel_path}:{line_number}: malformed plan heading "
                f"`{line.strip()}`; use `### PLAN-NNN: Title` so the plan "
                "shape checks can see the entry"
            )
    return failures


def check_plan_archive_shape(repo_root: Path) -> list[str]:
    """Ensure every archived plan records a completed final status."""
    failures: list[str] = []
    archive = repo_root / "docs" / "plans" / "archive.md"
    if not archive.exists():
        return failures

    text = archive.read_text(encoding="utf-8", errors="replace")
    failures.extend(_malformed_plan_headings(text, "docs/plans/archive.md"))
    for match in PLAN_BLOCK_RE.finditer(text):
        plan_id = match.group("id")
        block = match.group(0)
        if "**Final status:** Complete" not in block:
            failures.append(
                "docs/plans/archive.md: "
                f"{plan_id} is missing the `**Final status:** Complete` marker "
                "required for archived plans"
            )
    return failures


def check_design_docs_index(repo_root: Path) -> list[str]:
    """Ensure durable design docs stay discoverable and non-roadmap-shaped."""
    failures: list[str] = []
    design_dir = repo_root / "docs" / "design"
    if not design_dir.exists():
        return failures

    readme = design_dir / "README.md"
    agents = design_dir / "AGENTS.md"
    for required in (readme, agents):
        if not required.exists():
            failures.append(
                f"{required.relative_to(repo_root)}: missing design index/rules file"
            )

    readme_text = (
        readme.read_text(encoding="utf-8", errors="replace") if readme.exists() else ""
    )
    readme_links: set[str] = set()
    for _, raw_link in _iter_markdown_links(readme_text):
        linked_path = _resolve_markdown_link(raw_link, design_dir)
        if not linked_path:
            continue
        try:
            rel_path = linked_path.relative_to(design_dir.resolve())
        except ValueError:
            continue
        if len(rel_path.parts) == 1:
            readme_links.add(rel_path.name)
    for design_doc in sorted(design_dir.glob("*.md")):
        if design_doc.name in {"README.md", "AGENTS.md"}:
            continue
        rel_path = design_doc.relative_to(repo_root)
        if design_doc.name not in readme_links:
            failures.append(f"{rel_path}: missing from docs/design/README.md")

        text = design_doc.read_text(encoding="utf-8", errors="replace")
        repeated_field = DESIGN_DASHBOARD_FIELD_RE.search(text)
        if repeated_field:
            line = text.count("\n", 0, repeated_field.start()) + 1
            failures.append(
                f"{rel_path}:{line}: design doc repeats dashboard field "
                f"`{repeated_field.group(1)}`"
            )

    return failures


def check_markdown_internal_links(repo_root: Path) -> list[str]:
    """Reject broken internal links and dead heading anchors in every tracked
    markdown doc. Fenced code blocks are skipped; anchors are compared against
    GitHub-style heading slugs of the target file.
    """
    warnings: list[str] = []
    for path in iter_tracked_files(repo_root, list(MARKDOWN_LINK_PATTERNS)):
        text = path.read_text(encoding="utf-8", errors="replace")
        for line_number, raw_link in _iter_markdown_links(text):
            target = _strip_markdown_link_target(raw_link)
            if not target or _is_external_link(target):
                continue
            resolved = _resolve_markdown_link(
                raw_link,
                path.parent,
                repo_root=repo_root,
                current_file=path,
            )
            if resolved is None:
                continue
            try:
                resolved.relative_to(repo_root.resolve())
            except ValueError:
                warnings.append(
                    f"{path.relative_to(repo_root)}:{line_number}: internal link "
                    f"escapes repository: `{raw_link}`"
                )
                continue
            if not resolved.exists():
                warnings.append(
                    f"{path.relative_to(repo_root)}:{line_number}: broken internal "
                    f"link `{raw_link}` -> `{_display_path(resolved, repo_root)}`"
                )
                continue
            anchor = _markdown_link_anchor(raw_link)
            if anchor and resolved.is_dir():
                # `[Foo](foo/#section)` renders the directory's README.
                resolved = resolved / "README.md"
            if anchor and resolved.suffix == ".md" and resolved.is_file():
                if anchor not in _page_anchors(resolved):
                    warnings.append(
                        f"{path.relative_to(repo_root)}:{line_number}: link "
                        f"`{raw_link}` names heading anchor `#{anchor}` that "
                        f"`{_display_path(resolved, repo_root)}` does not define"
                    )
        source_root = _sphinx_source_root(path, repo_root)
        for line_number, href in _iter_html_hrefs(text):
            target = _strip_markdown_link_target(href)
            if not target or _is_external_link(target):
                continue
            fragment = unquote(target.split("#", 1)[1]) if "#" in target else ""
            target = unquote(target.split("?", 1)[0].split("#", 1)[0])
            if not target:
                page: Path | None = path  # same-page `#fragment`
            elif source_root is not None:
                page = _resolve_built_html_href(target, path, source_root)
            else:
                page = _resolve_markdown_link(target, path.parent, repo_root, path)
                if page is not None and not (
                    page.exists() and page.is_relative_to(repo_root.resolve())
                ):
                    page = None
            if page is None:
                warnings.append(
                    f"{path.relative_to(repo_root)}:{line_number}: raw HTML href "
                    f"`{href}` does not name an existing page or file"
                )
                continue
            if (
                fragment
                and page.suffix == ".md"
                and fragment not in _page_anchors(page)
            ):
                warnings.append(
                    f"{path.relative_to(repo_root)}:{line_number}: raw HTML href "
                    f"`{href}` names anchor `#{fragment}` that "
                    f"`{_display_path(page, repo_root)}` does not define"
                )
        if source_root is None:
            continue
        for line_number, target in _iter_myst_doc_roles(text):
            if _resolve_myst_doc_role(target, path, source_root) is None:
                warnings.append(
                    f"{path.relative_to(repo_root)}:{line_number}: MyST role "
                    f"`{{doc}}` targets `{target}`, which is not a page under "
                    f"`{_display_path(source_root, repo_root)}`"
                )
        for line_number, target, is_glob in _iter_toctree_entries(text):
            if is_glob:
                if not _expand_toctree_glob(target, path, source_root):
                    warnings.append(
                        f"{path.relative_to(repo_root)}:{line_number}: toctree glob "
                        f"`{target}` matches no page under "
                        f"`{_display_path(source_root, repo_root)}`"
                    )
                continue
            if _resolve_myst_doc_role(target, path, source_root) is None:
                warnings.append(
                    f"{path.relative_to(repo_root)}:{line_number}: toctree entry "
                    f"`{target}` is not a page under "
                    f"`{_display_path(source_root, repo_root)}`"
                )
    return warnings


def _markdown_link_anchor(link: str) -> str:
    target = _strip_markdown_link_target(link)
    if "#" not in target:
        return ""
    # Fragments are case-sensitive: `#Heading` does not reach `#heading`.
    return unquote(target.split("#", maxsplit=1)[1].strip())


def check_docs_discoverability(repo_root: Path) -> list[str]:
    """Reject owner-index discoverability gaps for conservative pilot buckets.

    Blocking since the PLAN-121 promotion: a direct pilot-bucket doc must be
    linked or mentioned from its owner index so agents can discover it.
    """
    warnings: list[str] = []
    for directory, index in DISCOVERABILITY_INDEXES.items():
        docs_dir = repo_root / directory
        index_path = repo_root / index
        if not docs_dir.exists() or not index_path.exists():
            continue

        index_text = index_path.read_text(encoding="utf-8", errors="replace")
        linked_targets: set[Path] = set()
        for _, raw_link in _iter_markdown_links(index_text):
            resolved = _resolve_markdown_link(
                raw_link,
                index_path.parent,
                repo_root=repo_root,
                current_file=index_path,
            )
            if resolved:
                linked_targets.add(resolved)

        for doc in sorted(docs_dir.glob("*.md")):
            if doc.name in {"README.md"}:
                continue
            repo_relative = doc.relative_to(repo_root).as_posix()
            text_mentions_doc = _mentions_reference(
                index_text, repo_relative
            ) or _mentions_reference(index_text, doc.name)
            if doc.resolve() not in linked_targets and not text_mentions_doc:
                warnings.append(
                    f"{doc.relative_to(repo_root)}: not linked from `{index}`"
                )
    return warnings


def check_docs_orphans(repo_root: Path) -> tuple[list[str], list[str]]:
    """Reject docs that are not reachable from a root surface.

    Roots are every tracked file in the reference corpus that is not itself
    subject to this check: bucket indexes and ``AGENTS.md`` files, root docs,
    the published site sources, scripts, tests, workflows, and ``pixi.toml``.
    A checked doc is reachable when a root, or another reachable checked doc,
    references it through a Markdown link that resolves to it, by its full
    repo-relative path as a whole token, or by a bare name that is unique
    among the checked files. Mutual references between unreachable docs do
    not keep each other alive. Markdown docs under ``docs/`` (outside the
    published site) fail; plan sidecar data files warn because scripts may
    address them through computed paths.
    """
    corpus: dict[str, str] = {}
    for path in iter_tracked_files(repo_root, list(ORPHAN_CORPUS_PATTERNS)):
        if path.is_dir():
            continue
        try:
            corpus[_display_path(path, repo_root)] = path.read_text(
                encoding="utf-8", errors="replace"
            )
        except OSError:
            continue

    checked: dict[str, bool] = {}  # rel_path -> blocking
    for pattern in ORPHAN_DOC_PATTERNS:
        for path in iter_tracked_files(repo_root, [pattern]):
            rel_path = _display_path(path, repo_root)
            if rel_path.startswith(ORPHAN_EXCLUDED_PREFIXES) or path.is_dir():
                continue
            if _is_orphan_root_by_convention(rel_path):
                continue
            checked[rel_path] = True
    for pattern in ORPHAN_SIDECAR_PATTERNS:
        for path in iter_tracked_files(repo_root, [pattern]):
            rel_path = _display_path(path, repo_root)
            if path.suffix == ".md" or path.is_dir() or rel_path in checked:
                continue
            checked[rel_path] = False

    # Bare-name matching is only unambiguous when no other tracked file in the
    # corpus or the checked set shares the basename (a published-site page
    # with the same name must not lend its links to a repo-local doc).
    name_counts: dict[str, int] = {}
    for rel_path in set(checked) | set(corpus):
        name = Path(rel_path).name
        name_counts[name] = name_counts.get(name, 0) + 1

    # Markdown referrers: every link resolved relative to the referring file.
    link_targets: dict[str, set[str]] = {}
    for other, text in corpus.items():
        if not other.endswith(".md"):
            continue
        other_path = repo_root / other
        targets: set[str] = set()
        for _, raw_link in _iter_markdown_links(text):
            resolved = _resolve_markdown_link(
                raw_link,
                other_path.parent,
                repo_root=repo_root,
                current_file=other_path,
            )
            if resolved is None:
                continue
            if resolved.is_dir() and (resolved / "README.md").is_file():
                # A directory link exposes that directory's README.
                resolved = resolved / "README.md"
            try:
                targets.add(resolved.relative_to(repo_root.resolve()).as_posix())
            except ValueError:
                continue
        link_targets[other] = targets

    def _referrers(rel_path: str) -> set[str]:
        """Files that reference ``rel_path``: a resolved Markdown link, the
        full repo-relative path as a whole token (scripts, tests, workflows,
        prose in backticks), or the bare name as a whole token when it is
        unique among the checked files."""
        name = Path(rel_path).name
        unique = name_counts.get(name, 0) <= 1
        found: set[str] = set()
        for other, text in corpus.items():
            if other == rel_path:
                continue
            if rel_path in link_targets.get(other, ()):
                found.add(other)  # resolved link (may be a directory link)
            elif name not in text:
                continue
            elif _mentions_reference(text, rel_path):
                found.add(other)
            elif unique and _mentions_reference(text, name):
                found.add(other)
        return found

    referrers = {rel_path: _referrers(rel_path) for rel_path in checked}

    def _propagate(roots: set[str], allowed_referrer) -> set[str]:
        reachable = set(roots)
        changed = True
        while changed:
            changed = False
            for rel_path, sources in referrers.items():
                if rel_path in reachable:
                    continue
                if any(s in reachable and allowed_referrer(s) for s in sources):
                    reachable.add(rel_path)
                    changed = True
        return reachable

    def _is_doc(rel_path: str) -> bool:
        return rel_path.endswith((".md", ".rst"))

    # Documentation must be discoverable through documentation: only bucket
    # indexes, other docs, and agent instruction files count as referrers.
    # A page named only by a script or test is still unindexed.
    doc_roots = {o for o in corpus if o not in checked and _is_doc(o)}
    doc_reachable = _propagate(doc_roots, _is_doc)
    # Sidecar data may legitimately be addressed by scripts and tests.
    all_reachable = _propagate({o for o in corpus if o not in checked}, lambda _s: True)

    failures: list[str] = []
    warnings: list[str] = []
    for rel_path, blocking in sorted(checked.items()):
        reachable = doc_reachable if blocking else all_reachable
        if rel_path in reachable:
            continue
        if blocking:
            failures.append(
                f"{rel_path}: not reachable from any index, owner doc, script, "
                "test, or workflow; link it from its owner index or delete it"
            )
        else:
            warnings.append(
                f"{rel_path}: plan sidecar not reachable from its owner plan, a "
                "sidecar doc, or a script"
            )
    return failures, warnings


def _is_orphan_root_by_convention(rel_path: str) -> bool:
    """Return whether a doc is discovered by convention instead of by link."""
    parts = Path(rel_path).parts
    name = parts[-1]
    if parts[:1] != ("docs",):
        return False
    if name in ORPHAN_INDEX_NAMES and len(parts) == 2:
        return True  # docs/README.md, docs/AGENTS.md
    if (
        name in ORPHAN_INDEX_NAMES
        and len(parts) == 3
        and parts[1] in REQUIRED_DOCS_TOP_LEVEL_DIRS
    ):
        return True  # docs/<registered bucket>/{README,AGENTS}.md
    return (
        parts[:2] == ("docs", "dev_tasks")
        and len(parts) == 4
        and name in ORPHAN_DEV_TASK_ENTRYPOINTS
    )


def _mentions_reference(text: str, candidate: str) -> bool:
    """Return whether ``candidate`` appears in ``text`` as a whole path token.

    ``api.md`` must not match inside ``old-api.md`` or ``api.mdx``; a longer
    path ending in the candidate (``../plans/api.md``) still counts.
    """
    start = 0
    while True:
        index = text.find(candidate, start)
        if index < 0:
            return False
        before = text[index - 1] if index > 0 else ""
        after_index = index + len(candidate)
        after = text[after_index] if after_index < len(text) else ""
        if not ORPHAN_REFERENCE_BOUNDARY_BEFORE.match(before or " ") and not (
            ORPHAN_REFERENCE_BOUNDARY_AFTER.match(after or " ")
        ):
            return True
        start = index + 1


def _parse_frontmatter(text: str) -> dict[str, str]:
    match = re.match(r"^---\s*\n(.*?)\n---\s*\n", text, re.DOTALL)
    if not match:
        return {}

    frontmatter: dict[str, str] = {}
    for line in match.group(1).splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        key_value = re.match(r"^([A-Za-z0-9_-]+):\s*(.+)$", line)
        if key_value:
            frontmatter[key_value.group(1)] = _normalize_frontmatter_value(
                key_value.group(2).strip()
            )
    return frontmatter


def _normalize_frontmatter_value(value: str) -> str:
    """Normalize the single-line YAML scalars used in DART frontmatter."""
    if len(value) < 2:
        return value
    if value[0] == '"' and value[-1] == '"':
        try:
            return str(json.loads(value))
        except json.JSONDecodeError:
            return value[1:-1]
    if value[0] == "'" and value[-1] == "'":
        return value[1:-1].replace("''", "'")
    return value


def check_ai_doc_frontmatter(repo_root: Path) -> list[str]:
    """Enforce the narrow docs/ai frontmatter pilot."""
    failures: list[str] = []
    ai_dir = repo_root / "docs" / "ai"
    actual_docs = (
        {path.name for path in ai_dir.glob("*.md")} if ai_dir.exists() else set()
    )
    for filename in sorted(actual_docs - DOCS_AI_FRONTMATTER_FILES):
        failures.append(
            f"docs/ai/{filename}: docs/ai frontmatter pilot roster is missing "
            "this Markdown file"
        )
    for filename in sorted(DOCS_AI_FRONTMATTER_FILES):
        path = ai_dir / filename
        rel_path = path.relative_to(repo_root)
        if not path.exists():
            failures.append(f"{rel_path}: missing required AI policy document")
            continue

        frontmatter = _parse_frontmatter(
            path.read_text(encoding="utf-8", errors="replace")
        )
        if not frontmatter:
            failures.append(f"{rel_path}: missing required docs/ai frontmatter")
            continue

        for key in DOCS_AI_FRONTMATTER_KEYS:
            if not frontmatter.get(key):
                failures.append(f"{rel_path}: missing frontmatter field `{key}`")

        doc_type = frontmatter.get("type", "")
        if doc_type and doc_type not in DOCS_AI_TYPE_LEGEND:
            failures.append(
                f"{rel_path}: frontmatter `type` not in legend: `{doc_type}`"
            )

        unknown_keys = set(frontmatter) - DOCS_AI_FRONTMATTER_KEYS
        if unknown_keys:
            failures.append(
                f"{rel_path}: unsupported docs/ai frontmatter field(s): "
                + ", ".join(f"`{key}`" for key in sorted(unknown_keys))
            )

        owner = frontmatter.get("owner", "")
        if owner and owner != "self":
            owner_path = _resolve_markdown_link(owner, path.parent, repo_root=repo_root)
            if owner_path is None or not owner_path.exists():
                failures.append(
                    f"{rel_path}: frontmatter owner does not resolve: `{owner}`"
                )

    return failures


def _line_number_for_offset(text: str, offset: int) -> int:
    return text.count("\n", 0, offset) + 1


def _parse_markdown_tables(text: str) -> list[tuple[int, list[str], list[list[str]]]]:
    lines = text.splitlines()
    tables: list[tuple[int, list[str], list[list[str]]]] = []
    index = 0
    while index + 1 < len(lines):
        header = lines[index]
        divider = lines[index + 1]
        if not (header.startswith("|") and divider.startswith("|")):
            index += 1
            continue
        if not re.match(r"^\|\s*:?-{3,}:?\s*(\|\s*:?-{3,}:?\s*)+\|?$", divider):
            index += 1
            continue

        headers = [cell.strip() for cell in header.strip("|").split("|")]
        rows: list[list[str]] = []
        row_index = index + 2
        while row_index < len(lines) and lines[row_index].startswith("|"):
            rows.append(
                [cell.strip() for cell in lines[row_index].strip("|").split("|")]
            )
            row_index += 1
        tables.append((index + 1, headers, rows))
        index = row_index
    return tables


def _strip_code_ticks(value: str) -> str:
    value = value.strip()
    if value.startswith("`") and value.endswith("`"):
        return value[1:-1]
    return value


def _strip_markdown_cell_markup(value: str) -> str:
    return re.sub(r"[*_`]", "", value).strip()


def _parse_papers_property_values(text: str) -> dict[str, set[str]]:
    values_by_property: dict[str, set[str]] = {}
    for _, headers, rows in _parse_markdown_tables(text):
        if "Property" not in headers or "Values" not in headers:
            continue
        for row in rows:
            if len(row) != len(headers):
                continue
            row_data = dict(zip(headers, row, strict=True))
            property_name = _strip_markdown_cell_markup(row_data["Property"])
            values = set(re.findall(r"`([^`]+)`", row_data.get("Values", "")))
            if values:
                values_by_property[property_name] = values
    return values_by_property


def check_papers_catalog(repo_root: Path) -> list[str]:
    """Validate the papers catalog shape without generating a second source."""
    failures: list[str] = []
    path = repo_root / "docs" / "readthedocs" / "papers.md"
    if not path.exists():
        return failures

    text = path.read_text(encoding="utf-8", errors="replace")
    allowed_values_by_field = _parse_papers_property_values(text)
    for field in PAPERS_CLOSED_VALUE_FIELDS:
        if not allowed_values_by_field.get(field):
            failures.append(
                "docs/readthedocs/papers.md: property legend is missing "
                f"closed values for `{field}`"
            )

    summary_entries: dict[str, dict[str, str]] = {}
    for table_line, headers, rows in _parse_markdown_tables(text):
        if "ID" not in headers:
            continue
        for row_offset, row in enumerate(rows, start=2):
            if len(row) != len(headers):
                failures.append(
                    f"docs/readthedocs/papers.md:{table_line + row_offset}: "
                    "summary table row has the wrong number of cells"
                )
                continue
            row_data = dict(zip(headers, row, strict=True))
            entry_id = _strip_code_ticks(row_data["ID"])
            if not entry_id:
                continue
            if entry_id in summary_entries:
                failures.append(
                    "docs/readthedocs/papers.md: duplicate summary entry "
                    f"`{entry_id}`"
                )
            else:
                summary_entries[entry_id] = {
                    "Status": _strip_code_ticks(row_data.get("Status", "")),
                    "Priority": _strip_code_ticks(row_data.get("Priority", "")),
                    "Verdict": _strip_code_ticks(row_data.get("Verdict", "")),
                }
            _validate_papers_table_value(
                failures,
                table_line + row_offset,
                entry_id,
                "Status",
                row_data.get("Status", ""),
                allowed_values_by_field.get("Status", set()),
            )
            _validate_papers_table_value(
                failures,
                table_line + row_offset,
                entry_id,
                "Priority",
                row_data.get("Priority", ""),
                allowed_values_by_field.get("Priority", set()),
            )
            _validate_papers_table_value(
                failures,
                table_line + row_offset,
                entry_id,
                "Verdict",
                row_data.get("Verdict", ""),
                allowed_values_by_field.get("Verdict", set()),
            )

    detail_matches = list(re.finditer(r"^### `(?P<id>[^`]+)`\s*$", text, re.MULTILINE))
    detail_ids: set[str] = set()
    for index, match in enumerate(detail_matches):
        entry_id = match.group("id")
        if entry_id in detail_ids:
            failures.append(
                f"docs/readthedocs/papers.md:{_line_number_for_offset(text, match.start())}: "
                f"duplicate detail entry `{entry_id}`"
            )
        detail_ids.add(entry_id)
        next_start = (
            detail_matches[index + 1].start()
            if index + 1 < len(detail_matches)
            else len(text)
        )
        next_section = re.search(r"^## ", text[match.end() :], re.MULTILINE)
        if next_section:
            next_start = min(next_start, match.end() + next_section.start())
        block = text[match.end() : next_start]
        line_number = _line_number_for_offset(text, match.start())
        detail_fields = _validate_papers_detail_block(
            failures,
            repo_root,
            path,
            text,
            block,
            entry_id,
            line_number,
            allowed_values_by_field,
        )
        if detail_fields and entry_id in summary_entries:
            _validate_papers_summary_detail_parity(
                failures,
                line_number,
                entry_id,
                summary_entries[entry_id],
                detail_fields,
            )

    summary_ids = set(summary_entries)
    for entry_id in sorted(summary_ids - detail_ids):
        failures.append(
            f"docs/readthedocs/papers.md: summary entry `{entry_id}` has no detail block"
        )
    for entry_id in sorted(detail_ids - summary_ids):
        failures.append(
            f"docs/readthedocs/papers.md: detail entry `{entry_id}` has no summary row"
        )

    return failures


def _validate_papers_table_value(
    failures: list[str],
    line_number: int,
    entry_id: str,
    field: str,
    value: str,
    allowed_values: set[str],
) -> None:
    if not value:
        return
    normalized = _strip_code_ticks(value)
    if allowed_values and normalized not in allowed_values:
        failures.append(
            f"docs/readthedocs/papers.md:{line_number}: `{entry_id}` has invalid "
            f"{field} `{normalized}`"
        )


def _validate_papers_detail_block(
    failures: list[str],
    repo_root: Path,
    path: Path,
    full_text: str,
    block: str,
    entry_id: str,
    heading_line: int,
    allowed_values_by_field: dict[str, set[str]],
) -> dict[str, str] | None:
    property_match = re.search(
        r"- \*\*Type:\*\*\s*(?P<type>[^·\n]+)"
        r"\s*·\s*\*\*Topic:\*\*\s*(?P<topic>[^·\n]+)"
        r"\s*·\s*\*\*Status:\*\*\s*(?P<status>[^·\n]+)"
        r"\s*·\s*\*\*Priority:\*\*\s*(?P<priority>[^·\n]+)"
        r"\s*·\s*\*\*Verdict:\*\*\s*(?P<verdict>[^\n]+)",
        block,
    )
    if not property_match:
        failures.append(
            f"docs/readthedocs/papers.md:{heading_line}: detail entry `{entry_id}` "
            "is missing the Type/Topic/Status/Priority/Verdict property line"
        )
        return None

    field_values = {
        "Type": (
            _strip_code_ticks(property_match.group("type").strip()),
            allowed_values_by_field.get("Type", set()),
        ),
        "Status": (
            _strip_code_ticks(property_match.group("status").strip()),
            allowed_values_by_field.get("Status", set()),
        ),
        "Priority": (
            _strip_code_ticks(property_match.group("priority").strip()),
            allowed_values_by_field.get("Priority", set()),
        ),
        "Verdict": (
            _strip_code_ticks(property_match.group("verdict").strip()),
            allowed_values_by_field.get("Verdict", set()),
        ),
    }
    property_line = heading_line + block[: property_match.start()].count("\n") + 1
    for field, (value, allowed_values) in field_values.items():
        if allowed_values and value not in allowed_values:
            failures.append(
                f"docs/readthedocs/papers.md:{property_line}: `{entry_id}` has "
                f"invalid {field} `{value}`"
            )

    block_offset = full_text.find(block)
    for relative_line, raw_link in _iter_markdown_links(block):
        target = _strip_markdown_link_target(raw_link)
        if not target or _is_external_link(target):
            continue
        resolved = _resolve_markdown_link(
            raw_link,
            path.parent,
            repo_root=repo_root,
            current_file=path,
        )
        if resolved is None:
            continue
        if not resolved.exists():
            line_number = (
                _line_number_for_offset(full_text, block_offset) + relative_line - 1
            )
            failures.append(
                f"docs/readthedocs/papers.md:{line_number}: `{entry_id}` has "
                f"broken local link `{raw_link}`"
            )
    return {field: value for field, (value, _) in field_values.items()}


def _validate_papers_summary_detail_parity(
    failures: list[str],
    line_number: int,
    entry_id: str,
    summary_fields: dict[str, str],
    detail_fields: dict[str, str],
) -> None:
    for field in ("Status", "Priority", "Verdict"):
        summary_value = summary_fields.get(field, "")
        detail_value = detail_fields.get(field, "")
        if summary_value and detail_value and summary_value != detail_value:
            failures.append(
                f"docs/readthedocs/papers.md:{line_number}: `{entry_id}` has "
                f"mismatched {field}: summary `{summary_value}`, detail "
                f"`{detail_value}`"
            )


def check_north_star_evidence_freshness(repo_root: Path) -> list[str]:
    """Report committed evidence paths newer than the verification marker."""
    path = repo_root / "docs" / "ai" / "north-star.md"
    if not path.exists():
        return []

    text = path.read_text(encoding="utf-8", errors="replace")
    marker = NORTH_STAR_FRESHNESS_MARKER_RE.search(text)
    if not marker:
        return [
            "docs/ai/north-star.md: missing advisory evidence freshness marker "
            "`<!-- docs-policy: evidence-last-verified=YYYY-MM-DD -->`"
        ]
    last_verified = marker.group("date")

    current_state = re.search(
        r"## Current State(?P<body>.*?)(?=^## What Is Missing|\Z)",
        text,
        re.MULTILINE | re.DOTALL,
    )
    if not current_state:
        return []

    warnings: list[str] = []
    candidates = _extract_repo_path_candidates(current_state.group("body"))
    for candidate in sorted(candidates):
        candidate_path = repo_root / candidate
        if not candidate_path.exists():
            continue
        last_commit_date = _git_last_commit_date(repo_root, candidate)
        if last_commit_date and last_commit_date > last_verified:
            warnings.append(
                "docs/ai/north-star.md: evidence path "
                f"`{candidate}` changed on {last_commit_date}, newer than "
                f"evidence-last-verified={last_verified}"
            )
    return warnings


def _extract_repo_path_candidates(text: str) -> set[str]:
    candidates: set[str] = set()
    path_re = re.compile(
        r"(?<![\w/.-])"
        r"(?P<path>"
        r"(?:docs|dart|python|scripts|tests|examples|tutorials|\.github)"
        r"/[A-Za-z0-9_./*-]+"
        r"|README\.md|CHANGELOG\.md|AGENTS\.md|pixi\.toml"
        r")"
    )
    for match in path_re.finditer(text):
        candidate = match.group("path").rstrip(".,;:)")
        if "*" not in candidate:
            candidates.add(candidate)
    return candidates


def _git_last_commit_date(repo_root: Path, repo_relative_path: str) -> str | None:
    try:
        result = subprocess.run(
            [
                "git",
                "log",
                "-1",
                "--date=short",
                "--format=%ad",
                "--",
                repo_relative_path,
            ],
            cwd=repo_root,
            check=True,
            capture_output=True,
            text=True,
        )
    except GIT_QUERY_ERRORS:
        return None
    return result.stdout.strip() or None


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    failures: list[str] = []
    warnings: list[str] = []
    for path in iter_markdown_files(repo_root):
        failures.extend(check_file(path, repo_root))
    failures.extend(check_docs_indexes(repo_root))
    failures.extend(check_docs_placement_owner(repo_root))
    failures.extend(check_dev_task_shape(repo_root))
    warnings.extend(check_dev_task_size(repo_root))
    failures.extend(check_plan_lifecycle(repo_root))
    failures.extend(check_dashboard_structure(repo_root))
    failures.extend(check_plan_archive_shape(repo_root))
    failures.extend(check_design_docs_index(repo_root))
    failures.extend(check_ai_doc_frontmatter(repo_root))
    failures.extend(check_papers_catalog(repo_root))
    failures.extend(check_markdown_internal_links(repo_root))
    failures.extend(check_docs_discoverability(repo_root))
    orphan_failures, orphan_warnings = check_docs_orphans(repo_root)
    failures.extend(orphan_failures)
    warnings.extend(orphan_warnings)
    warnings.extend(check_north_star_evidence_freshness(repo_root))

    if warnings:
        print("Documentation policy advisories:", file=sys.stderr)
        for warning in warnings:
            print(f"  - {warning}", file=sys.stderr)

    if not failures:
        return 0

    print("Documentation policy check failed:", file=sys.stderr)
    for failure in failures:
        print(f"  - {failure}", file=sys.stderr)
    print("\nFix the documentation policy failures listed above.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
