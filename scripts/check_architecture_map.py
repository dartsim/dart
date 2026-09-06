#!/usr/bin/env python3
"""Blocking structural checks for the DART architecture map (PLAN-130 WP-130.2).

The map is the set of typed archify views under ``docs/assets/architecture/``
that ``docs/readthedocs/architecture.md`` embeds. This checker needs no Node.js
and runs inside ``pixi run check-lint``. It fails when:

* a view is not well-formed JSON with unique ids and resolvable edges;
* cited source evidence points at a missing path, an out-of-range line, or a
  qualified/CamelCase symbol that no public header defines;
* a ``dart/simulation`` directory, a ``BuiltInWorldStepStageSlot`` enumerator,
  a ``RigidBodySolver`` or ``MultibodyIntegrationFamily`` enumerator, a
  ``dart/<module>`` directory, or a ``WorldStepStage`` subclass is absent from
  the view that owns it (allowlists below carry a reason per exemption);
* a stage id used outside the step-flow view is not a node of that view;
* the published page does not embed a view or name its JSON source.

Archify's own schema and layout validation happens at render time in
``scripts/render_architecture_map.py``; the runtime drift check is advisory and
lives in ``scripts/check_architecture_map_runtime.py``.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_IR_DIR = Path("docs/assets/architecture")
DEFAULT_PAGE = Path("docs/readthedocs/architecture.md")
RENDERED_DIR_NAME = "architecture-map"

VIEW_SUFFIXES = {
    ".architecture.json": "architecture",
    ".dataflow.json": "dataflow",
}
FRAMEWORK_VIEW = "simulation-framework"
STEP_VIEW = "world-step"
COMPUTE_VIEW = "compute-graph"
LIBRARY_VIEW = "library-context"
REQUIRED_VIEWS = (FRAMEWORK_VIEW, STEP_VIEW, COMPUTE_VIEW, LIBRARY_VIEW)

STATUS_TAGS = ("Implemented", "Partial", "Planned", "Undecided")

SIMULATION_DIR = Path("dart/simulation")
COMPUTE_DIR = SIMULATION_DIR / "compute"
STAGE_SLOT_HEADER = SIMULATION_DIR / "detail" / "world_step_schedule.hpp"
STAGE_SLOT_ENUM = "BuiltInWorldStepStageSlot"
RIGID_SOLVER_HEADER = SIMULATION_DIR / "world_options.hpp"
RIGID_SOLVER_ENUM = "RigidBodySolver"
MULTIBODY_FAMILY_HEADER = SIMULATION_DIR / "multibody" / "multibody_options.hpp"
MULTIBODY_FAMILY_ENUM = "MultibodyIntegrationFamily"
STAGE_CLASS_DIRS = (COMPUTE_DIR, COMPUTE_DIR / "detail")
LIBRARY_EXTRA_PREFIXES = ("python/dartpy", "dartsim")

# Directories under dart/simulation that no view has to cite, with the reason.
SIMULATION_COVERAGE_EXEMPTIONS: dict[str, str] = {
    "dart/simulation/ecs": (
        "single internal component-access header; ECS storage stays out of "
        "every view by PLAN-130 decision 9"
    ),
}

# WorldStepStage subclasses that intentionally have no BuiltInWorldStepStageSlot.
STAGE_CLASS_ALLOWLIST: dict[str, str] = {
    "MultibodyContactStage": (
        "standalone multibody contact stage kept for explicit composition; the "
        "built-in schedule resolves contacts through UnifiedConstraint"
    ),
    "BatchedRigidBodyIntegrationStage": (
        "WorldBatch lane-parallel integration stage; the single-World schedule "
        "uses RigidBodyVelocity/RigidBodyPosition"
    ),
    "RigidBodyIntegrationStage": (
        "direct unconstrained rigid integration stage for explicit pipelines "
        "and tests; the built-in schedule uses the split "
        "RigidBodyVelocity/RigidBodyPosition stages"
    ),
}

_ENUM_RE_TEMPLATE = r"enum\s+class\s+{name}\b[^{{]*\{{(.*?)\}}\s*;"
_COMMENT_RE = re.compile(r"//[^\n]*|/\*.*?\*/", re.S)
_STAGE_CLASS_RE = re.compile(
    r"class\s+(?:DART_SIMULATION_API\s+)?(\w+Stage)\b[^{;]*?:\s*public\s+"
    r"(?:compute::)?WorldStepStage\b",
    re.S,
)
_QUALIFIED_SYMBOL_RE = re.compile(
    r"\b[A-Za-z_][A-Za-z0-9_]*(?:::[A-Za-z_][A-Za-z0-9_]*)+\b"
)
_CAMEL_SYMBOL_RE = re.compile(r"\b[A-Z][a-z0-9]+(?:[A-Z][a-z0-9]+)+\b")
_FIRST_LEVEL_DIRS_TO_SKIP = {"__pycache__"}


_NAMESPACE_OR_BRACE_RE = re.compile(r"\bnamespace\s+([\w:]+)\s*\{|[{}]")


def enclosing_namespace(text: str, position: int) -> str:
    """Qualified namespace open at ``position`` in comment-free header text.

    Tracks ``namespace a::b {`` and nested ``namespace a { namespace b {``
    openings against brace depth; anonymous namespaces and other braces
    (classes, functions) do not add a name.
    """
    depth = 0
    stack: list[tuple[int, str]] = []
    for match in _NAMESPACE_OR_BRACE_RE.finditer(text, 0, position):
        token = match.group(0)
        if token == "{":
            depth += 1
        elif token == "}":
            depth -= 1
            while stack and stack[-1][0] > depth:
                stack.pop()
        else:
            depth += 1
            stack.append((depth, match.group(1)))
    return "::".join(name for _, name in stack)


def snake_case(name: str) -> str:
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


def strip_comments(text: str) -> str:
    return _COMMENT_RE.sub("", text)


def parse_enumerators(text: str, enum_name: str) -> list[str]:
    match = re.search(_ENUM_RE_TEMPLATE.format(name=re.escape(enum_name)), text, re.S)
    if not match:
        return []
    body = strip_comments(match.group(1))
    names: list[str] = []
    for part in body.split(","):
        token = part.split("=", 1)[0].strip()
        if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", token):
            names.append(token)
    return names


@dataclass
class ViewFile:
    name: str
    diagram_type: str
    path: Path
    ir: dict

    @property
    def relpath(self) -> str:
        return self.path.as_posix()

    def nodes(self) -> list[dict]:
        key = "components" if self.diagram_type == "architecture" else "nodes"
        return [n for n in self.ir.get(key, []) if isinstance(n, dict)]

    def edges(self) -> list[dict]:
        key = "connections" if self.diagram_type == "architecture" else "flows"
        return [e for e in self.ir.get(key, []) if isinstance(e, dict)]

    def node_ids(self) -> set[str]:
        return {str(n.get("id")) for n in self.nodes() if n.get("id") is not None}

    def source_paths(self) -> list[str]:
        paths: list[str] = []
        for node in self.nodes():
            for source in node.get("sources", []) or []:
                if isinstance(source, dict) and source.get("path"):
                    paths.append(str(source["path"]))
        return paths

    def text(self) -> str:
        """Every human-readable string in the view, for vocabulary checks."""
        parts: list[str] = []
        meta = self.ir.get("meta", {})
        for key in ("title", "subtitle"):
            if meta.get(key):
                parts.append(str(meta[key]))
        for node in self.nodes():
            for key in ("id", "label", "sublabel", "tag"):
                if node.get(key):
                    parts.append(str(node[key]))
            for source in node.get("sources", []) or []:
                if isinstance(source, dict) and source.get("label"):
                    parts.append(str(source["label"]))
        for edge in self.edges():
            for key in ("label", "classification"):
                if edge.get(key):
                    parts.append(str(edge[key]))
        for stage in self.ir.get("stages", []) or []:
            if isinstance(stage, dict) and stage.get("label"):
                parts.append(str(stage["label"]))
        for boundary in self.ir.get("boundaries", []) or []:
            if isinstance(boundary, dict) and boundary.get("label"):
                parts.append(str(boundary["label"]))
        for card in self.ir.get("cards", []) or []:
            if isinstance(card, dict):
                if card.get("title"):
                    parts.append(str(card["title"]))
                parts.extend(str(item) for item in card.get("items", []) or [])
        return "\n".join(parts)


@dataclass
class Checker:
    repo_root: Path = REPO_ROOT
    ir_dir: Path = DEFAULT_IR_DIR
    page: Path = DEFAULT_PAGE
    errors: list[str] = field(default_factory=list)
    _header_text: str | None = None

    # ----------------------------------------------------------------- helpers
    def _abs(self, relative: Path | str) -> Path:
        return self.repo_root / Path(relative)

    def error(self, message: str) -> None:
        self.errors.append(message)

    def header_text(self) -> str:
        if self._header_text is None:
            parts: list[str] = []
            root = self._abs("dart")
            for header in sorted(root.rglob("*.hpp")) if root.is_dir() else []:
                try:
                    parts.append(header.read_text(encoding="utf-8", errors="ignore"))
                except OSError:
                    continue
            self._header_text = "\n".join(parts)
        return self._header_text

    def symbol_resolves(self, symbol: str) -> bool:
        """True when ``symbol`` is declared inside the namespace it names.

        ``dart::a::b::Name`` must be declared (class, struct, enum, namespace,
        using, concept, or function) by a header under ``dart/a/`` inside an
        enclosing namespace that reads exactly ``dart::a::b``, whether the
        header opens it as ``namespace dart::a::b {`` or as nested blocks. A
        namespace move or a typo in any component therefore fails instead of
        matching a same-named declaration elsewhere. Other qualified names
        must occur verbatim in the headers; unqualified CamelCase names may
        occur anywhere.
        """
        parts = symbol.split("::")
        if len(parts) == 1:
            pattern = r"\b" + re.escape(symbol) + r"\b"
            return re.search(pattern, self.header_text()) is not None
        if parts[0] != "dart":
            return re.search(re.escape(symbol), self.header_text()) is not None
        module = self._abs(Path("dart", parts[1]))
        if not module.is_dir():
            return False
        expected = "::".join(parts[:-1])
        tail = re.escape(parts[-1])
        declaration = re.compile(
            r"(?:\b(?:class|struct|enum\s+class|enum|using|concept)\s+"
            r"(?:DART_\w+_API\s+)?" + tail + r"\b)"
            r"|(?:\bnamespace\s+(?P<ns>(?:[\w]+::)*)" + tail + r"\b)"
            r"|(?:\b" + tail + r"\s*\()"
        )
        for header in sorted(module.rglob("*.hpp")):
            try:
                text = strip_comments(
                    header.read_text(encoding="utf-8", errors="ignore")
                )
            except OSError:
                continue
            for match in declaration.finditer(text):
                enclosing = enclosing_namespace(text, match.start())
                declared_prefix = (match.group("ns") or "").rstrip(":")
                if match.group("ns") is not None:
                    # A namespace declaration names the symbol itself.
                    full = "::".join(filter(None, [enclosing, declared_prefix]))
                    if full == expected:
                        return True
                elif enclosing == expected:
                    return True
        return False

    def read(self, relative: Path | str) -> str:
        try:
            return self._abs(relative).read_text(encoding="utf-8", errors="ignore")
        except OSError:
            return ""

    # ------------------------------------------------------------------ loading
    def load_views(self) -> dict[str, ViewFile]:
        views: dict[str, ViewFile] = {}
        directory = self._abs(self.ir_dir)
        if not directory.is_dir():
            self.error(f"{self.ir_dir.as_posix()}: view directory not found")
            return views
        for path in sorted(directory.iterdir()):
            diagram_type = None
            name = None
            for suffix, kind in VIEW_SUFFIXES.items():
                if path.name.endswith(suffix):
                    diagram_type = kind
                    name = path.name[: -len(suffix)]
                    break
            if diagram_type is None or name is None:
                continue
            rel = path.relative_to(self.repo_root)
            try:
                ir = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                self.error(f"{rel.as_posix()}: unreadable JSON ({exc})")
                continue
            if not isinstance(ir, dict):
                self.error(f"{rel.as_posix()}: top level must be an object")
                continue
            declared = ir.get("diagram_type")
            if declared != diagram_type:
                self.error(
                    f"{rel.as_posix()}: diagram_type {declared!r} does not match "
                    f"the file suffix ({diagram_type})"
                )
            if name in views:
                self.error(
                    f"{rel.as_posix()}: view name `{name}` is already used by "
                    f"{views[name].relpath}; the renderer would write both to "
                    f"`{name}.html`"
                )
                continue
            views[name] = ViewFile(
                name=name, diagram_type=diagram_type, path=rel, ir=ir
            )
        for required in REQUIRED_VIEWS:
            if required not in views:
                self.error(
                    f"{self.ir_dir.as_posix()}: required view `{required}` is missing"
                )
        return views

    # ----------------------------------------------------------------- structure
    def check_structure(self, view: ViewFile) -> None:
        label = view.relpath
        if not isinstance(view.ir.get("meta"), dict) or not view.ir["meta"].get(
            "title"
        ):
            self.error(f"{label}: meta.title is required")
        ids: set[str] = set()
        for node in view.nodes():
            node_id = node.get("id")
            if not node_id:
                self.error(f"{label}: a node has no id")
                continue
            if node_id in ids:
                self.error(f"{label}: duplicate id `{node_id}`")
            ids.add(str(node_id))
            tag = node.get("tag")
            if tag is None:
                self.error(
                    f"{label}: node `{node_id}` has no status tag; every node "
                    f"carries one of {', '.join(STATUS_TAGS)}"
                )
            elif tag not in STATUS_TAGS:
                self.error(
                    f"{label}: node `{node_id}` tag {tag!r} is not one of "
                    f"{', '.join(STATUS_TAGS)}"
                )
            if view.diagram_type == "dataflow":
                stages = view.ir.get("stages", [])
                stage = node.get("stage")
                if not isinstance(stage, int) or not 0 <= stage < len(stages):
                    self.error(
                        f"{label}: node `{node_id}` stage {stage!r} is not an index "
                        f"into the {len(stages)} declared stage(s)"
                    )
        if not ids:
            self.error(f"{label}: view declares no nodes")
        for edge in view.edges():
            for end in ("from", "to"):
                target = edge.get(end)
                if target not in ids:
                    self.error(
                        f"{label}: edge `{edge.get('id', edge.get('from'))}` "
                        f"{end}=`{target}` does not name a node"
                    )
            if view.diagram_type == "dataflow" and not edge.get("label"):
                self.error(
                    f"{label}: flow `{edge.get('id', edge.get('from'))}` has no "
                    "label; flows name the exchanged data"
                )
        for boundary in view.ir.get("boundaries", []) or []:
            if not isinstance(boundary, dict):
                continue
            for wrapped in boundary.get("wraps", []) or []:
                if wrapped not in ids:
                    self.error(
                        f"{label}: boundary `{boundary.get('label')}` wraps unknown "
                        f"id `{wrapped}`"
                    )

    # ------------------------------------------------------------------ evidence
    def check_evidence(self, view: ViewFile) -> None:
        label = view.relpath
        for node in view.nodes():
            node_id = node.get("id", "?")
            for source in node.get("sources", []) or []:
                if not isinstance(source, dict) or not source.get("path"):
                    self.error(f"{label}: node `{node_id}` has a source without a path")
                    continue
                rel = str(source["path"])
                target = self._abs(rel)
                if not target.exists():
                    self.error(f"{label}: node `{node_id}` cites missing path `{rel}`")
                    continue
                line = source.get("line")
                end_line = source.get("end_line")
                if line is None and end_line is None:
                    continue
                if not target.is_file():
                    self.error(
                        f"{label}: node `{node_id}` cites a line in `{rel}`, which is "
                        "a directory"
                    )
                    continue
                count = len(self.read(rel).splitlines())
                if not isinstance(line, int) or not 1 <= line <= count:
                    self.error(
                        f"{label}: node `{node_id}` cites `{rel}` line {line!r}, "
                        f"outside 1..{count}"
                    )
                    continue
                if end_line is not None and (
                    not isinstance(end_line, int) or not line <= end_line <= count
                ):
                    self.error(
                        f"{label}: node `{node_id}` cites `{rel}` end_line "
                        f"{end_line!r}, outside {line}..{count}"
                    )
            for key in ("label", "sublabel"):
                text = str(node.get(key) or "")
                for symbol in self._symbols(text):
                    if not self.symbol_resolves(symbol):
                        self.error(
                            f"{label}: node `{node_id}` {key} names `{symbol}`, which "
                            "no header under dart/ defines"
                        )

    @staticmethod
    def _symbols(text: str) -> list[str]:
        found: list[str] = []
        for token in _QUALIFIED_SYMBOL_RE.findall(text):
            if token.startswith("dartpy."):
                continue
            found.append(token)
        stripped = _QUALIFIED_SYMBOL_RE.sub(" ", text)
        found.extend(_CAMEL_SYMBOL_RE.findall(stripped))
        return found

    # ------------------------------------------------------------------ coverage
    def _dirs(self, relative: Path) -> list[str]:
        root = self._abs(relative)
        if not root.is_dir():
            return []
        return sorted(
            (relative / child.name).as_posix()
            for child in root.iterdir()
            if child.is_dir() and child.name not in _FIRST_LEVEL_DIRS_TO_SKIP
        )

    @staticmethod
    def _covered(prefix: str, paths: list[str]) -> bool:
        prefix = prefix.rstrip("/") + "/"
        return any(p == prefix.rstrip("/") or p.startswith(prefix) for p in paths)

    def check_coverage(self, views: dict[str, ViewFile]) -> None:
        framework = views.get(FRAMEWORK_VIEW)
        step = views.get(STEP_VIEW)
        compute = views.get(COMPUTE_VIEW)
        library = views.get(LIBRARY_VIEW)

        if framework is not None:
            paths = framework.source_paths()
            for directory in self._dirs(SIMULATION_DIR):
                if directory == COMPUTE_DIR.as_posix():
                    continue
                if directory in SIMULATION_COVERAGE_EXEMPTIONS:
                    continue
                if not self._covered(directory, paths):
                    self.error(
                        f"{framework.relpath}: no source cites `{directory}/`; add the "
                        "module to a component or exempt it with a reason"
                    )
            text = framework.text()
            for header, enum_name in (
                (RIGID_SOLVER_HEADER, RIGID_SOLVER_ENUM),
                (MULTIBODY_FAMILY_HEADER, MULTIBODY_FAMILY_ENUM),
            ):
                enumerators = parse_enumerators(self.read(header), enum_name)
                if not enumerators:
                    self.error(
                        f"{header.as_posix()}: could not parse `enum class "
                        f"{enum_name}`; update the checker"
                    )
                for enumerator in enumerators:
                    if not re.search(r"\b" + re.escape(enumerator) + r"\b", text):
                        self.error(
                            f"{framework.relpath}: `{enum_name}::{enumerator}` does not "
                            "appear in any label, sublabel, tag, or card"
                        )

        if compute is not None:
            paths = compute.source_paths()
            required = [COMPUTE_DIR.as_posix(), *self._dirs(COMPUTE_DIR)]
            for directory in required:
                if directory in SIMULATION_COVERAGE_EXEMPTIONS:
                    continue
                if not self._covered(directory, paths):
                    self.error(f"{compute.relpath}: no source cites `{directory}/`")

        if library is not None:
            paths = library.source_paths()
            for directory in [*self._dirs(Path("dart")), *LIBRARY_EXTRA_PREFIXES]:
                if not self._abs(directory).is_dir():
                    continue
                if not self._covered(directory, paths):
                    self.error(f"{library.relpath}: no source cites `{directory}/`")

        slots = parse_enumerators(self.read(STAGE_SLOT_HEADER), STAGE_SLOT_ENUM)
        if not slots:
            self.error(
                f"{STAGE_SLOT_HEADER.as_posix()}: could not parse `enum class "
                f"{STAGE_SLOT_ENUM}`; update the checker"
            )
        slot_ids = {snake_case(slot) for slot in slots}
        if step is not None:
            node_ids = step.node_ids()
            for slot in slots:
                if snake_case(slot) not in node_ids:
                    self.error(
                        f"{step.relpath}: stage slot `{slot}` has no node with id "
                        f"`{snake_case(slot)}`"
                    )
            for name, view in views.items():
                if name == STEP_VIEW:
                    continue
                for node_id in sorted(view.node_ids()):
                    if node_id in slot_ids and node_id not in node_ids:
                        self.error(
                            f"{view.relpath}: stage id `{node_id}` is not a node of "
                            f"the {STEP_VIEW} view"
                        )

        for class_name, header in self._stage_classes():
            base = class_name[: -len("Stage")]
            if base in slots or class_name in STAGE_CLASS_ALLOWLIST:
                continue
            self.error(
                f"{header}: `{class_name}` derives from WorldStepStage but matches "
                f"no {STAGE_SLOT_ENUM} enumerator; add the slot and its step-flow "
                "node, or allowlist the class with a reason"
            )

    def _stage_classes(self) -> list[tuple[str, str]]:
        found: list[tuple[str, str]] = []
        for directory in STAGE_CLASS_DIRS:
            root = self._abs(directory)
            if not root.is_dir():
                continue
            for header in sorted(root.glob("*.hpp")):
                text = header.read_text(encoding="utf-8", errors="ignore")
                for match in _STAGE_CLASS_RE.finditer(text):
                    rel = header.relative_to(self.repo_root).as_posix()
                    found.append((match.group(1), rel))
        return found

    # ---------------------------------------------------------------------- page
    def check_page(self, views: dict[str, ViewFile]) -> None:
        page_text = self.read(self.page)
        if not page_text:
            self.error(f"{self.page.as_posix()}: page not found or empty")
            return
        for view in views.values():
            embed = f"{RENDERED_DIR_NAME}/{view.name}.html"
            if embed not in page_text:
                self.error(
                    f"{self.page.as_posix()}: does not embed `{embed}` for view "
                    f"`{view.name}`"
                )
            if view.path.name not in page_text:
                self.error(
                    f"{self.page.as_posix()}: does not name the source "
                    f"`{view.relpath}`"
                )

    # ----------------------------------------------------------------------- run
    def run(self) -> list[str]:
        views = self.load_views()
        for view in views.values():
            self.check_structure(view)
            self.check_evidence(view)
        if views:
            self.check_coverage(views)
            self.check_page(views)
        return self.errors


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--repo-root", type=Path, default=REPO_ROOT)
    parser.add_argument("--ir-dir", type=Path, default=DEFAULT_IR_DIR)
    parser.add_argument("--page", type=Path, default=DEFAULT_PAGE)
    args = parser.parse_args(argv)

    checker = Checker(repo_root=args.repo_root, ir_dir=args.ir_dir, page=args.page)
    errors = checker.run()
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        print(f"{len(errors)} architecture map error(s)")
        return 1
    count = len(checker.load_views())
    print(f"Validated {count} architecture map view(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
