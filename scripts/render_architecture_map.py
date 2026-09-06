#!/usr/bin/env python3
"""Render the DART architecture map views with archify.

Source of truth: the typed JSON views under ``docs/assets/architecture/``
(``<view>.architecture.json`` or ``<view>.dataflow.json``). Output: one
self-contained HTML file per view under
``docs/readthedocs/_generated/architecture-map/`` (gitignored and copied into
the published site through ``html_extra_path``), plus the same set of files as
plain-text fallbacks when Node.js or the pinned archify checkout is unavailable.

The renderer is archify (MIT, https://github.com/tt-a1i/archify), pinned by
tag and fetched into ``.deps/archify`` on first use; it is never vendored. The
blocking structural checks live in ``scripts/check_architecture_map.py`` and do
not need Node.js; this driver only validates and renders.

Exit codes: 0 rendered; 2 toolchain unavailable (fallback written, or
``--strict`` turned that into an error); 1 a view failed validation or delivery.

PLAN-130 WP-130.1 owns this file.
"""

from __future__ import annotations

import argparse
import html
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_IR_DIR = REPO_ROOT / "docs" / "assets" / "architecture"
DEFAULT_OUTPUT_DIR = (
    REPO_ROOT / "docs" / "readthedocs" / "_generated" / "architecture-map"
)
DEFAULT_DEPS_DIR = REPO_ROOT / ".deps" / "archify"

ARCHIFY_REPOSITORY = "https://github.com/tt-a1i/archify.git"
ARCHIFY_TAG = "v2.16.0"
# Commit the tag resolved to when it was pinned (2026-09-05). Upgrades change
# both values together after the skill's audit procedure re-verifies rendering.
ARCHIFY_COMMIT = "c826e6c3a7abad19c0f3cd1ca57207d54b1ad8de"
ARCHIFY_CLI = Path("archify") / "bin" / "archify.mjs"
MIN_NODE_MAJOR = 18

DART_REPOSITORY_URL = "https://github.com/dartsim/dart"
VIEW_SUFFIXES = {
    ".architecture.json": "architecture",
    ".dataflow.json": "dataflow",
}
SCRIPT_RELPATH = "scripts/render_architecture_map.py"

EXIT_OK = 0
EXIT_FAILED = 1
EXIT_UNAVAILABLE = 2

_FONT_LINK_RE = re.compile(r"<link[^>]*fonts\.(?:googleapis|gstatic)\.com[^>]*>", re.S)
_EMPTY_NOSCRIPT_RE = re.compile(r"<noscript>\s*</noscript>", re.S)
_DOCTYPE_RE = re.compile(r"^\s*<!doctype html[^>]*>\s*", re.I)


@dataclass
class View:
    path: Path
    name: str
    diagram_type: str

    @property
    def relpath(self) -> str:
        try:
            return self.path.resolve().relative_to(REPO_ROOT).as_posix()
        except ValueError:
            return self.path.as_posix()


@dataclass
class Report:
    rendered: list[str] = field(default_factory=list)
    fallback: list[str] = field(default_factory=list)
    failed: list[str] = field(default_factory=list)
    messages: list[str] = field(default_factory=list)


def log(message: str) -> None:
    print(f"[architecture-map] {message}", file=sys.stderr)


def discover_views(ir_dir: Path) -> list[View]:
    views: list[View] = []
    if not ir_dir.is_dir():
        return views
    for path in sorted(ir_dir.iterdir()):
        for suffix, diagram_type in VIEW_SUFFIXES.items():
            if path.name.endswith(suffix):
                name = path.name[: -len(suffix)]
                views.append(View(path=path, name=name, diagram_type=diagram_type))
                break
    return views


def node_executable(min_major: int = MIN_NODE_MAJOR) -> Path | None:
    node = shutil.which("node")
    if not node:
        return None
    try:
        version = subprocess.run(
            [node, "--version"], check=True, capture_output=True, text=True
        ).stdout.strip()
    except OSError, subprocess.CalledProcessError:
        return None
    match = re.match(r"v(\d+)", version)
    if not match or int(match.group(1)) < min_major:
        log(f"Node.js {version} is older than the required v{min_major}.")
        return None
    return Path(node)


def _git_head(repo: Path) -> str | None:
    try:
        return subprocess.run(
            ["git", "-C", str(repo), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except OSError, subprocess.CalledProcessError:
        return None


def ensure_archify(
    deps_dir: Path,
    *,
    fetch: bool = True,
    tag: str = ARCHIFY_TAG,
    commit: str = ARCHIFY_COMMIT,
    repository: str = ARCHIFY_REPOSITORY,
) -> Path | None:
    """Return the archify checkout at the pinned commit, cloning it if allowed."""
    cli = deps_dir / ARCHIFY_CLI
    if cli.is_file() and _git_head(deps_dir) == commit:
        return deps_dir
    if deps_dir.exists():
        if _git_head(deps_dir) not in (None, commit):
            log(
                f"{deps_dir} is not at the pinned archify commit {commit[:12]}; "
                "re-fetching."
            )
        if not fetch:
            return None
        shutil.rmtree(deps_dir, ignore_errors=True)
    if not fetch:
        return None
    deps_dir.parent.mkdir(parents=True, exist_ok=True)
    try:
        subprocess.run(
            [
                "git",
                "clone",
                "--quiet",
                "--depth",
                "1",
                "--branch",
                tag,
                repository,
                str(deps_dir),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        detail = getattr(exc, "stderr", "") or str(exc)
        log(f"Could not fetch archify {tag}: {detail.strip()}")
        return None
    head = _git_head(deps_dir)
    if head != commit:
        log(
            f"archify {tag} resolved to {head}, expected {commit}; refusing to "
            "render with an unverified checkout."
        )
        return None
    if not cli.is_file():
        log(f"archify checkout at {deps_dir} lacks {ARCHIFY_CLI}.")
        return None
    return deps_dir


def stamp_repository(ir: dict, revision: str, url: str = DART_REPOSITORY_URL) -> dict:
    """Return a copy of an architecture IR with ``meta.repository`` set.

    Tracked views omit ``meta.repository`` because archify requires a full
    revision and verifies every cited path at that revision; stamping the
    current HEAD at render time keeps evidence links pointing at the built
    commit instead of a stale pin.
    """
    stamped = json.loads(json.dumps(ir))
    meta = stamped.setdefault("meta", {})
    meta["repository"] = {"url": url, "revision": revision}
    return stamped


def declares_sources(ir: dict) -> bool:
    return any(component.get("sources") for component in ir.get("components", []))


def strip_external_fonts(text: str) -> str:
    text = _FONT_LINK_RE.sub("", text)
    return _EMPTY_NOSCRIPT_RE.sub("", text)


def source_comment(view: View, revision: str | None) -> str:
    rev = revision or "unknown revision"
    return (
        f"<!-- Generated by {SCRIPT_RELPATH} from {view.relpath} "
        f"(archify {ARCHIFY_TAG}, DART {rev}). Do not edit; edit the JSON "
        "source. -->\n"
    )


def prepend_source_comment(text: str, comment: str) -> str:
    match = _DOCTYPE_RE.match(text)
    if match:
        return text[: match.end()] + comment + text[match.end() :]
    return comment + text


def run_archify(
    node: Path, archify_dir: Path, args: list[str], cwd: Path
) -> tuple[int, dict]:
    env = dict(os.environ)
    env["ARCHIFY_UPDATE_CHECK_DISABLED"] = "1"
    completed = subprocess.run(
        [str(node), str(archify_dir / ARCHIFY_CLI), *args, "--json"],
        cwd=cwd,
        env=env,
        capture_output=True,
        text=True,
    )
    payload: dict = {}
    if completed.stdout.strip():
        try:
            payload = json.loads(completed.stdout)
        except json.JSONDecodeError:
            payload = {"raw": completed.stdout}
    if completed.stderr.strip():
        payload.setdefault("stderr", completed.stderr.strip())
    return completed.returncode, payload


def _format_diagnostics(payload: dict) -> str:
    lines: list[str] = []
    for item in payload.get("diagnostics", []) or []:
        code = item.get("code", "?")
        severity = item.get("severity", "?")
        message = item.get("message", "")
        lines.append(f"    {severity} {code}: {message}")
        fixes = item.get("supportedFixes") or []
        if fixes:
            lines.append(f"      fixes: {'; '.join(fixes)}")
    if not lines and payload.get("error"):
        lines.append(f"    {payload['error']}")
    if not lines and payload.get("stderr"):
        lines.append(f"    {payload['stderr']}")
    return "\n".join(lines)


def render_view(
    view: View,
    *,
    node: Path,
    archify_dir: Path,
    output_dir: Path,
    revision: str | None,
    repo_root: Path = REPO_ROOT,
    strip_fonts: bool = True,
    check_only: bool = False,
) -> tuple[bool, str]:
    """Validate (and unless ``check_only``, deliver) one view. Returns (ok, log)."""
    try:
        ir = json.loads(view.path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return False, f"{view.relpath}: unreadable JSON: {exc}"

    repo_args: list[str] = []
    if view.diagram_type == "architecture" and declares_sources(ir):
        if not revision:
            return (
                False,
                f"{view.relpath}: declares source evidence but the DART "
                "revision is unknown (not a git checkout?).",
            )
        ir = stamp_repository(ir, revision)
        repo_args = ["--repo-root", str(repo_root)]

    with tempfile.TemporaryDirectory(prefix="architecture-map-") as tmp:
        candidate = Path(tmp) / view.path.name
        candidate.write_text(json.dumps(ir, indent=2) + "\n", encoding="utf-8")
        code, payload = run_archify(
            node,
            archify_dir,
            [
                "validate",
                view.diagram_type,
                str(candidate),
                "--quality",
                "showcase",
                *repo_args,
            ],
            cwd=repo_root,
        )
        if code != 0 or not payload.get("ok", False):
            return (
                False,
                f"{view.relpath}: archify validation failed\n"
                + _format_diagnostics(payload),
            )
        if check_only:
            return True, f"{view.relpath}: validated"

        output = Path(tmp) / f"{view.name}.html"
        code, payload = run_archify(
            node,
            archify_dir,
            [
                "deliver",
                view.diagram_type,
                str(candidate),
                str(output),
                "--quality",
                "showcase",
                *repo_args,
            ],
            cwd=repo_root,
        )
        if code != 0 or not payload.get("ok", False) or not output.is_file():
            return (
                False,
                f"{view.relpath}: archify delivery failed\n"
                + _format_diagnostics(payload),
            )
        text = output.read_text(encoding="utf-8")

    if strip_fonts:
        text = strip_external_fonts(text)
    text = prepend_source_comment(text, source_comment(view, revision))
    output_dir.mkdir(parents=True, exist_ok=True)
    target = output_dir / f"{view.name}.html"
    target.write_text(text, encoding="utf-8")
    return True, f"{view.relpath}: rendered to {target}"


def fallback_html(view: View, reason: str, revision: str | None) -> str:
    """A text rendering of the IR used when the interactive renderer is absent."""
    try:
        ir = json.loads(view.path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        ir = {"meta": {"title": view.name}, "_error": str(exc)}
    title = html.escape(str(ir.get("meta", {}).get("title", view.name)))
    parts = [
        "<!DOCTYPE html>\n",
        source_comment(view, revision),
        '<html lang="en"><head><meta charset="utf-8">',
        f"<title>{title}</title>",
        "<style>body{font:15px/1.5 system-ui,sans-serif;margin:1.5rem;"
        "color:#1f2933;background:#fff}h1{font-size:1.25rem}"
        "h2{font-size:1rem;margin-top:1.25rem}code{font-size:.9em}"
        "ul{padding-left:1.25rem}.note{color:#52606d}</style></head><body>",
        f"<h1>{title}</h1>",
        f'<p class="note">Text rendering. {html.escape(reason)} '
        f"Source: <code>{html.escape(view.relpath)}</code>.</p>",
    ]
    if "_error" in ir:
        parts.append(f"<p>Unreadable source: {html.escape(ir['_error'])}</p>")

    def item(node: dict) -> str:
        label = html.escape(str(node.get("label", node.get("id", ""))))
        sublabel = node.get("sublabel")
        tag = node.get("tag")
        text = f"<strong>{label}</strong>"
        if sublabel:
            text += f" &mdash; {html.escape(str(sublabel))}"
        if tag:
            text += f" <em>[{html.escape(str(tag))}]</em>"
        return f"<li>{text}</li>"

    if view.diagram_type == "dataflow":
        stages = [
            s.get("label", f"stage {i}") for i, s in enumerate(ir.get("stages", []))
        ]
        for index, stage in enumerate(stages):
            nodes = [n for n in ir.get("nodes", []) if n.get("stage") == index]
            parts.append(f"<h2>{html.escape(str(stage))}</h2><ul>")
            parts.extend(item(n) for n in nodes)
            parts.append("</ul>")
        edges = ir.get("flows", [])
    else:
        parts.append("<h2>Components</h2><ul>")
        parts.extend(item(c) for c in ir.get("components", []))
        parts.append("</ul>")
        boundaries = ir.get("boundaries", [])
        if boundaries:
            parts.append("<h2>Boundaries</h2><ul>")
            for boundary in boundaries:
                wraps = ", ".join(
                    html.escape(str(w)) for w in boundary.get("wraps", [])
                )
                parts.append(
                    f"<li><strong>{html.escape(str(boundary.get('label', '')))}"
                    f"</strong>: {wraps}</li>"
                )
            parts.append("</ul>")
        edges = ir.get("connections", [])
    if edges:
        parts.append("<h2>Relationships</h2><ul>")
        for edge in edges:
            label = edge.get("label")
            text = (
                f"<code>{html.escape(str(edge.get('from', '')))}</code> &rarr; "
                f"<code>{html.escape(str(edge.get('to', '')))}</code>"
            )
            if label:
                text += f": {html.escape(str(label))}"
            parts.append(f"<li>{text}</li>")
        parts.append("</ul>")
    parts.append("</body></html>\n")
    return "".join(parts)


def write_fallbacks(
    views: list[View], output_dir: Path, reason: str, revision: str | None
) -> list[str]:
    output_dir.mkdir(parents=True, exist_ok=True)
    written: list[str] = []
    for view in views:
        target = output_dir / f"{view.name}.html"
        target.write_text(fallback_html(view, reason, revision), encoding="utf-8")
        written.append(view.name)
    return written


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--ir-dir", type=Path, default=DEFAULT_IR_DIR)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--deps-dir", type=Path, default=DEFAULT_DEPS_DIR)
    parser.add_argument(
        "--views",
        nargs="*",
        default=None,
        help="view names to render (default: every view in --ir-dir)",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="validate with archify only; write nothing",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="treat a missing Node.js or archify checkout as an error",
    )
    parser.add_argument(
        "--no-fetch",
        action="store_true",
        help="never clone archify; use an existing checkout or fall back",
    )
    parser.add_argument(
        "--keep-fonts",
        action="store_true",
        help="keep archify's external Google Fonts links in the output",
    )
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    views = discover_views(args.ir_dir)
    if args.views:
        wanted = set(args.views)
        views = [v for v in views if v.name in wanted]
        missing = wanted - {v.name for v in views}
        if missing:
            log(f"unknown view(s): {sorted(missing)}")
            return EXIT_FAILED
    if not views:
        log(f"no views found under {args.ir_dir}; nothing to render.")
        return EXIT_OK

    revision = _git_head(REPO_ROOT)
    node = node_executable()
    archify_dir = (
        ensure_archify(args.deps_dir, fetch=not args.no_fetch) if node else None
    )

    if node is None or archify_dir is None:
        reason = (
            "Node.js (>= v18) was not found on PATH."
            if node is None
            else f"The pinned archify checkout ({ARCHIFY_TAG}) is unavailable."
        )
        if args.check or args.strict:
            log(f"{reason} Interactive rendering is required here.")
            return EXIT_FAILED
        written = write_fallbacks(views, args.output_dir, reason, revision)
        log(
            f"{reason} Wrote text fallbacks for {len(written)} view(s) to "
            f"{args.output_dir}."
        )
        return EXIT_UNAVAILABLE

    report = Report()
    for view in views:
        ok, message = render_view(
            view,
            node=node,
            archify_dir=archify_dir,
            output_dir=args.output_dir,
            revision=revision,
            strip_fonts=not args.keep_fonts,
            check_only=args.check,
        )
        log(message)
        (report.rendered if ok else report.failed).append(view.name)

    if report.failed:
        log(f"{len(report.failed)} view(s) failed: {', '.join(report.failed)}")
        return EXIT_FAILED
    verb = "validated" if args.check else "rendered"
    log(f"{verb} {len(report.rendered)} view(s).")
    return EXIT_OK


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
