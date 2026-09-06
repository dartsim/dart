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
* the recorded stage list matches the guided view named by the fixture's
  ``guided_view`` (the schedule variant the reference scene exercises);
* every node name of every recorded graph appears in the compute-graph view
  text or in the fixture's documented ``graph_vocabulary``, so a renamed or
  new compute node shows up as drift;
* with ``--probe-output <json>`` or a built probe binary, the fresh dump has
  the same stages and graph node/edge sets as the fixture.

``--regenerate`` rewrites the fixture from a fresh probe dump.
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
PROBE_BINARY_CANDIDATES = tuple(
    f"build/default/cpp/{build_type}/bin/test_architecture_probe{suffix}"
    for build_type in ("Release", "Debug")
    for suffix in ("", ".exe")
)


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


def compare_dumps(fixture: dict, fresh: dict) -> list[str]:
    findings: list[str] = []
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


def check_against_views(
    fixture: dict, step_view: dict, compute_view: dict
) -> list[str]:
    findings: list[str] = []
    stage_ids = {str(n.get("id")) for n in step_view.get("nodes", [])}
    stages = [str(s) for s in fixture.get("stages", [])]
    for stage in stages:
        if stage not in stage_ids:
            findings.append(f"recorded stage `{stage}` has no node in {STEP_VIEW.name}")
    guided = fixture.get("guided_view")
    if guided:
        views = {v.get("id"): v for v in step_view.get("meta", {}).get("views", [])}
        if guided not in views:
            findings.append(
                f"guided view `{guided}` named by the fixture does not exist"
            )
        else:
            focus = [str(f) for f in views[guided].get("focus", [])]
            missing = [s for s in stages if s not in focus]
            if missing:
                findings.append(
                    f"guided view `{guided}` does not focus recorded stage(s) {missing}"
                )
            else:
                focused_order = [f for f in focus if f in stages]
                if focused_order != stages:
                    findings.append(
                        f"guided view `{guided}` lists stages in the order "
                        f"{focused_order}, but the step ran {stages}"
                    )
    vocabulary = (
        view_text(compute_view)
        + "\n"
        + "\n".join(str(v) for v in fixture.get("graph_vocabulary", []))
    )
    seen: set[str] = set()
    for graph in fixture.get("graphs", []):
        for node in graph.get("nodes", []):
            name = str(node)
            if name in seen:
                continue
            seen.add(name)
            stem = re.split(r"[\[:#(]", name, maxsplit=1)[0].strip()
            stem = re.sub(r"(?:_level_\d+)?(?:_chunk_\d+)?(?:_\d+)?$", "", stem)
            if stem and not re.search(re.escape(stem), vocabulary, re.IGNORECASE):
                findings.append(
                    f"compute node `{name}` is not named by {COMPUTE_VIEW.name} or "
                    "the fixture's graph_vocabulary"
                )
    return findings


def find_probe_binary() -> Path | None:
    for candidate in PROBE_BINARY_CANDIDATES:
        path = REPO_ROOT / candidate
        if path.is_file() and os.access(path, os.X_OK):
            return path
    return None


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


def reduce_dump(fresh: dict, previous: dict | None) -> dict:
    """Keep the committed fixture small and self-describing."""
    reduced = {
        "schema_version": 1,
        "source": (
            "generated by tests/unit/simulation/compute/test_architecture_probe.cpp "
            "via scripts/check_architecture_map_runtime.py --regenerate"
        ),
        "scene": fresh.get("scene"),
        "guided_view": (previous or {}).get("guided_view", "fused-multibody"),
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
        previous = load_json(args.fixture) if args.fixture.is_file() else None
        args.fixture.write_text(
            json.dumps(reduce_dump(fresh, previous), indent=2) + "\n", encoding="utf-8"
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
