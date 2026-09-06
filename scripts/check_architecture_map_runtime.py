#!/usr/bin/env python3
"""Advisory runtime drift check for the architecture map (PLAN-130 WP-130.7).

The committed fixture ``docs/assets/architecture/compute-graph.runtime.json``
records what the reference scene in
``tests/unit/simulation/compute/test_architecture_probe.cpp`` actually ran:
the stage names of one ``World::step()`` and the node/edge sets of every
compute graph the injected executor executed. This script checks that record
against the drawn views and, when the probe binary is available, against a
fresh probe dump.

Checks (all advisory by default; ``--strict`` turns findings into failures):

* every recorded stage name is a node id of the step-flow view;
* the fixture names a guided view (``guided_view``, the schedule variant the
  reference scene exercises) and the recorded stage list equals that view's
  scheduled entries (its focus minus the sync/continuation bookends) in order;
* every node name of every recorded graph, with its level/chunk suffix
  stripped, matches a whole word of the compute-graph view text or of the
  fixture's documented ``graph_vocabulary`` (case and underscores ignored), so
  a renamed or new compute node shows up as drift;
* with ``--probe-output <json>`` or a built probe binary, the fresh dump has
  the same scene, stages, and graph node/edge sets as the fixture;
* the fixture and any fresh dump record a scene description, a nonempty stage list, and at least one
  executed compute graph whose edges are pairs of its recorded nodes without
  self-edges or cycles, so an empty, dangling, or impossible dump can neither
  pass the checks vacuously nor be committed by ``--regenerate``.

``--regenerate`` rewrites the fixture from a fresh probe dump; the guided view
it records is validated against the step-flow view and can be set with
``--guided-view <id>``. ``execution_trace`` must be a JSON boolean everywhere.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
IR_DIR = REPO_ROOT / "docs" / "assets" / "architecture"
FIXTURE = IR_DIR / "compute-graph.runtime.json"
STEP_VIEW = IR_DIR / "world-step.dataflow.json"
COMPUTE_VIEW = IR_DIR / "compute-graph.architecture.json"
PROBE_ENV = "DART_ARCHITECTURE_PROBE_OUTPUT"
DEFAULT_GUIDED_VIEW = "fused-multibody"
PROBE_BINARY_NAME = "test_architecture_probe"
BUILD_TYPES = ("Release", "Debug")


def probe_binary_candidates(environment: str = "default") -> tuple[str, ...]:
    """Relative probe paths inside one pixi environment's build tree.

    Single-config generators (Ninja, Makefiles) build in
    ``build/<env>/cpp/<Config>`` and place tests in its ``bin``; multi-config
    generators (Visual Studio) build in ``build/<env>/cpp`` and place tests in
    ``bin/<Config>`` (``dart_add_simulation_test`` in ``cmake/dart_defs.cmake``).
    """
    root = f"build/{environment}/cpp"
    return tuple(
        candidate
        for build_type in BUILD_TYPES
        for suffix in ("", ".exe")
        for candidate in (
            f"{root}/{build_type}/bin/{PROBE_BINARY_NAME}{suffix}",
            f"{root}/bin/{build_type}/{PROBE_BINARY_NAME}{suffix}",
        )
    )


PROBE_BINARY_CANDIDATES = probe_binary_candidates()


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def view_text(ir: dict) -> str:
    parts: list[str] = []
    for node in ir.get("components", []) + ir.get("nodes", []):
        for key in ("id", "label", "sublabel", "tag"):
            if node.get(key):
                parts.append(str(node[key]))
        for source in node.get("sources", []) or []:
            if source.get("label"):
                parts.append(str(source["label"]))
    for edge in ir.get("connections", []) + ir.get("flows", []):
        for key in ("label", "classification"):
            if edge.get(key):
                parts.append(str(edge[key]))
    for card in ir.get("cards", []) or []:
        parts.append(str(card.get("title", "")))
        parts.extend(str(item) for item in card.get("items", []) or [])
    return "\n".join(parts)


def graph_signature(
    graphs: list[dict],
) -> list[tuple[tuple[str, ...], tuple[tuple[str, str], ...]]]:
    signature = []
    for graph in graphs:
        nodes = tuple(sorted(str(n) for n in graph.get("nodes", [])))
        edges = tuple(sorted((str(a), str(b)) for a, b in graph.get("edges", [])))
        signature.append((nodes, edges))
    return signature


def dump_shape_findings(dump: dict, label: str) -> list[str]:
    """Reject dumps whose emptiness would let every other check pass vacuously."""
    findings: list[str] = []
    if not isinstance(dump.get("execution_trace"), bool):
        findings.append(
            f"{label} execution_trace must be a JSON boolean, not "
            f"{type(dump.get('execution_trace')).__name__}"
        )
    if not isinstance(dump.get("scene"), str) or not dump["scene"].strip():
        findings.append(
            f"{label} records no scene description; the probe must say what its "
            "reference scene covers"
        )
    stages = dump.get("stages")
    if (
        not isinstance(stages, list)
        or not stages
        or not all(isinstance(stage, str) and stage for stage in stages)
    ):
        findings.append(
            f"{label} records no stage list; the reference scene must run a full "
            "World::step()"
        )
    graphs = dump.get("graphs")
    if not isinstance(graphs, list) or not graphs:
        findings.append(
            f"{label} records no executed compute graph; the reference scene must "
            "exercise at least the kinematics graph"
        )
    else:
        for index, graph in enumerate(graphs):
            nodes = graph.get("nodes") if isinstance(graph, dict) else None
            if (
                not isinstance(nodes, list)
                or not nodes
                or not all(isinstance(node, str) and node for node in nodes)
            ):
                findings.append(f"{label} graph {index} has no nodes")
                continue
            edges = graph.get("edges")
            if not isinstance(edges, list):
                findings.append(f"{label} graph {index} edges are not a list")
                continue
            names = set(nodes)
            for position, edge in enumerate(edges):
                if not (
                    isinstance(edge, (list, tuple))
                    and len(edge) == 2
                    and all(isinstance(end, str) for end in edge)
                ):
                    findings.append(
                        f"{label} graph {index} edge {position} is not a [from, to] "
                        "pair of node names"
                    )
                elif edge[0] not in names or edge[1] not in names:
                    findings.append(
                        f"{label} graph {index} edge {position} {list(edge)} names a "
                        "node the graph did not record"
                    )
                elif edge[0] == edge[1]:
                    findings.append(
                        f"{label} graph {index} edge {position} is a self-edge; "
                        "ComputeGraph is a DAG"
                    )
            if not any(f.startswith(f"{label} graph {index} edge") for f in findings):
                if not is_acyclic(nodes, edges):
                    findings.append(
                        f"{label} graph {index} contains a cycle; ComputeGraph is a DAG"
                    )
    return findings


def is_acyclic(nodes: list[str], edges: list) -> bool:
    """Kahn's algorithm over validated [from, to] edges."""
    indegree = {node: 0 for node in nodes}
    outgoing: dict[str, list[str]] = {node: [] for node in nodes}
    for source, target in edges:
        outgoing[source].append(target)
        indegree[target] += 1
    ready = [node for node, degree in indegree.items() if degree == 0]
    visited = 0
    while ready:
        node = ready.pop()
        visited += 1
        for target in outgoing[node]:
            indegree[target] -= 1
            if indegree[target] == 0:
                ready.append(target)
    return visited == len(nodes)


def compare_dumps(fixture: dict, fresh: dict) -> list[str]:
    findings: list[str] = []
    if fresh.get("execution_trace") is not True:
        findings.append(
            "probe dump carries the built-in schedule, not an execution trace "
            "(DART_BUILD_PROFILE compiled out); rebuild with profiling for fresh "
            "evidence"
        )
    findings.extend(dump_shape_findings(fresh, "probe dump"))
    if fixture.get("scene") != fresh.get("scene"):
        findings.append(
            f"reference scene drifted: fixture {fixture.get('scene')!r} vs probe "
            f"{fresh.get('scene')!r}"
        )
    if fixture.get("stages") != fresh.get("stages"):
        findings.append(
            "stage list drifted: fixture "
            f"{fixture.get('stages')} vs probe {fresh.get('stages')}"
        )
    fixture_graphs = graph_signature(fixture.get("graphs", []))
    fresh_graphs = graph_signature(fresh.get("graphs", []))
    if len(fixture_graphs) != len(fresh_graphs):
        findings.append(
            f"graph count drifted: fixture {len(fixture_graphs)} vs probe "
            f"{len(fresh_graphs)}"
        )
    for index, (old, new) in enumerate(zip(fixture_graphs, fresh_graphs)):
        if old[0] != new[0]:
            findings.append(f"graph {index} node set drifted: {old[0]} vs {new[0]}")
        if old[1] != new[1]:
            findings.append(f"graph {index} edge set drifted")
    return findings


def step_view_bookends() -> tuple[str, ...]:
    """The step-flow nodes that are not schedule slots, shared with the checker."""
    scripts_dir = str(Path(__file__).resolve().parent)
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
    try:
        import check_architecture_map as cam

        return tuple(cam.STEP_VIEW_BOOKENDS)
    except ImportError:
        return ("sync", "continuation")


def normalize_token(token: str) -> str:
    """Compare identifiers ignoring case and underscores (`Kinematics` == `kinematics`)."""
    return re.sub(r"[^a-z0-9]", "", token.lower())


def check_against_views(
    fixture: dict, step_view: dict, compute_view: dict
) -> list[str]:
    findings: list[str] = []
    if fixture.get("execution_trace") is not True:
        findings.append(
            "fixture stages were not observed from an executed step; regenerate "
            "from a profiling-enabled build"
        )
    findings.extend(dump_shape_findings(fixture, "fixture"))
    stage_ids = {str(n.get("id")) for n in step_view.get("nodes", [])}
    stages = [str(s) for s in fixture.get("stages", [])]
    for stage in stages:
        if stage not in stage_ids:
            findings.append(f"recorded stage `{stage}` has no node in {STEP_VIEW.name}")
    guided = fixture.get("guided_view")
    if not isinstance(guided, str) or not guided:
        findings.append(
            "fixture names no guided_view, so the recorded stage order cannot be "
            "validated; set it to a `meta.views` id of the step-flow view"
        )
    else:
        views = {v.get("id"): v for v in step_view.get("meta", {}).get("views", [])}
        if guided not in views:
            findings.append(
                f"guided view `{guided}` named by the fixture does not exist"
            )
        else:
            focus = [str(f) for f in views[guided].get("focus", [])]
            bookends = step_view_bookends()
            expected = [f for f in focus if f not in bookends]
            if stages != expected:
                findings.append(
                    f"guided view `{guided}` schedules {expected}, but the fixture "
                    f"recorded {stages}"
                )
    vocabulary = (
        view_text(compute_view)
        + "\n"
        + "\n".join(str(v) for v in fixture.get("graph_vocabulary", []))
    )
    tokens = {normalize_token(t) for t in re.findall(r"[A-Za-z0-9_]+", vocabulary)}
    seen: set[str] = set()
    for graph in fixture.get("graphs", []):
        for node in graph.get("nodes", []):
            name = str(node)
            if name in seen:
                continue
            seen.add(name)
            stem = re.split(r"[\[:#(]", name, maxsplit=1)[0].strip()
            stem = re.sub(r"(?:_level_\d+)?(?:_chunk_\d+)?(?:_\d+)?$", "", stem)
            if stem and normalize_token(stem) not in tokens:
                findings.append(
                    f"compute node `{name}` is not named by {COMPUTE_VIEW.name} or "
                    "the fixture's graph_vocabulary"
                )
    return findings


def find_probe_binary() -> Path | None:
    """Locate the probe: the selected build type if set, else the newest binary.

    ``BUILD_TYPE`` (set by the pixi build tasks) or ``CMAKE_BUILD_TYPE`` picks
    the configuration; without either, the most recently built candidate wins so
    a fresh Debug build is not shadowed by a stale Release one.
    """
    environments: list[str] = []
    for environment in (os.environ.get("PIXI_ENVIRONMENT_NAME"), "default"):
        if environment and environment not in environments:
            environments.append(environment)
    found: list[Path] = []
    for environment in environments:
        for candidate in probe_binary_candidates(environment):
            path = REPO_ROOT / candidate
            if path.is_file() and os.access(path, os.X_OK):
                found.append(path)
    if not found:
        return None
    preferred = os.environ.get("BUILD_TYPE") or os.environ.get("CMAKE_BUILD_TYPE")
    if preferred:
        found = [p for p in found if f"/{preferred}/" in p.as_posix()]
        if not found:
            print(
                f"no {preferred} probe binary is built; not falling back to another "
                "build type"
            )
            return None
    return max(found, key=lambda p: p.stat().st_mtime)


def run_probe(binary: Path) -> dict | None:
    with tempfile.TemporaryDirectory(prefix="architecture-probe-") as tmp:
        output = Path(tmp) / "probe.json"
        env = dict(os.environ)
        env[PROBE_ENV] = str(output)
        completed = subprocess.run(
            [str(binary), "--gtest_filter=ArchitectureProbe.*"],
            env=env,
            capture_output=True,
            text=True,
            cwd=REPO_ROOT,
        )
        if completed.returncode != 0 or not output.is_file():
            print(
                f"probe binary failed ({completed.returncode}):\n{completed.stdout[-2000:]}"
            )
            return None
        return load_json(output)


def guided_view_ids(step_view: dict) -> list[str]:
    views = (
        step_view.get("meta", {}).get("views", [])
        if isinstance(step_view, dict)
        else []
    )
    return [str(v.get("id")) for v in views if isinstance(v, dict) and v.get("id")]


def reduce_dump(
    fresh: dict, previous: dict | None, guided_view: str | None = None
) -> dict:
    """Keep the committed fixture small and self-describing."""
    reduced = {
        "schema_version": 1,
        "execution_trace": fresh.get("execution_trace") is True,
        "source": (
            "generated by tests/unit/simulation/compute/test_architecture_probe.cpp "
            "via scripts/check_architecture_map_runtime.py --regenerate"
        ),
        "scene": fresh.get("scene"),
        "guided_view": guided_view
        or (previous or {}).get("guided_view")
        or DEFAULT_GUIDED_VIEW,
        "stages": fresh.get("stages", []),
        "graphs": [
            {"nodes": g.get("nodes", []), "edges": g.get("edges", [])}
            for g in fresh.get("graphs", [])
        ],
        "graph_vocabulary": (previous or {}).get("graph_vocabulary", []),
    }
    return reduced


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--fixture", type=Path, default=FIXTURE)
    parser.add_argument("--probe-output", type=Path, default=None)
    parser.add_argument("--probe-binary", type=Path, default=None)
    parser.add_argument("--regenerate", action="store_true")
    parser.add_argument(
        "--allow-schedule-only",
        action="store_true",
        help="let --regenerate accept a dump without an execution trace",
    )
    parser.add_argument(
        "--guided-view",
        default=None,
        help="meta.views id of the step-flow view that --regenerate records",
    )
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args(argv)

    fresh: dict | None = None
    probe_failed = False
    if args.probe_output is not None:
        fresh = load_json(args.probe_output)
    else:
        if args.probe_binary is not None and not (
            args.probe_binary.is_file() and os.access(args.probe_binary, os.X_OK)
        ):
            print(
                f"ERROR: probe binary {args.probe_binary} is missing or not executable"
            )
            return 1
        binary = args.probe_binary or find_probe_binary()
        if binary is not None:
            fresh = run_probe(binary)
            probe_failed = fresh is None
        else:
            print(
                "probe binary not built; comparing the committed fixture with the views only"
            )

    if args.regenerate:
        if fresh is None:
            print(
                "ERROR: --regenerate needs a probe dump (build the probe or pass --probe-output)"
            )
            return 1
        if not isinstance(fresh.get("execution_trace"), bool):
            print(
                "ERROR: the probe dump's execution_trace must be a JSON boolean, not "
                f"{type(fresh.get('execution_trace')).__name__}"
            )
            return 1
        if fresh.get("execution_trace") is not True and not args.allow_schedule_only:
            print(
                "ERROR: the probe dump has no execution trace (profiling compiled "
                "out); rebuild with DART_BUILD_PROFILE=ON or pass "
                "--allow-schedule-only"
            )
            return 1
        shape = dump_shape_findings(fresh, "probe dump")
        if shape:
            for finding in shape:
                print(f"ERROR: {finding}")
            print("ERROR: refusing to regenerate the fixture from an empty probe dump")
            return 1
        previous = load_json(args.fixture) if args.fixture.is_file() else None
        guided = (
            args.guided_view
            or (previous or {}).get("guided_view")
            or DEFAULT_GUIDED_VIEW
        )
        valid = guided_view_ids(load_json(STEP_VIEW)) if STEP_VIEW.is_file() else []
        if not isinstance(guided, str) or guided not in valid:
            print(
                f"ERROR: guided view {guided!r} is not a meta.views id of "
                f"{STEP_VIEW.name} ({', '.join(valid) or 'none'}); pass "
                "--guided-view <id>"
            )
            return 1
        args.fixture.write_text(
            json.dumps(reduce_dump(fresh, previous, guided), indent=2) + "\n",
            encoding="utf-8",
        )
        print(f"regenerated {args.fixture}")
        return 0

    if not args.fixture.is_file():
        print(f"ERROR: fixture {args.fixture} is missing; run with --regenerate")
        return 1
    fixture = load_json(args.fixture)
    findings = check_against_views(
        fixture, load_json(STEP_VIEW), load_json(COMPUTE_VIEW)
    )
    if probe_failed:
        findings.append(
            "probe binary failed or wrote no dump; the fixture could not be compared "
            "with fresh evidence"
        )
    if fresh is not None:
        findings.extend(compare_dumps(fixture, fresh))

    if findings:
        level = "ERROR" if args.strict else "ADVISORY"
        for finding in findings:
            print(f"{level}: {finding}")
        print(f"{len(findings)} architecture map runtime finding(s)")
        return 1 if args.strict else 0
    print(
        "Architecture map runtime fixture agrees with the views"
        + (" and the fresh probe" if fresh is not None else "")
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
