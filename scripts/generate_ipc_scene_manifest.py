#!/usr/bin/env python3
"""Generate PLAN-081's IPC upstream scene corpus manifest."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
from collections import Counter
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "docs"
    / "plans"
    / "081-deformable-implicit-barrier-solver"
    / "ipc_scene_corpus_manifest.json"
)
UPSTREAM_REPO = "https://github.com/ipc-sim/IPC"
UPSTREAM_COMMIT = "573d2c7e04104d3f9baf526bdaee7745891a571a"
ASSET_PATTERN = re.compile(r"\binput/[^\s#]+")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--upstream-dir",
        type=Path,
        required=True,
        help="Path to an ipc-sim/IPC checkout at the audited commit.",
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def rel_scene_path(path: Path, upstream_dir: Path) -> str:
    return path.relative_to(upstream_dir).as_posix()


def collect_assets(scene_path: Path) -> list[str]:
    assets: set[str] = set()
    for line in scene_path.read_text(errors="replace").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        for match in ASSET_PATTERN.findall(stripped):
            token = match.strip().strip(",;")
            if token.endswith(".txt"):
                continue
            assets.add(token)
    return sorted(assets)


def classify(path: str) -> tuple[str, str, str, bool, str, str]:
    """Return family, target, priority, visual flag, invariant, and topic."""

    if path.startswith("input/failures/"):
        return (
            "failure-case",
            "test",
            "P1",
            False,
            "Scene remains classified as a known failure-mode regression with a "
            "documented DART behavior decision.",
            "failure regression",
        )

    if "/supplementB/SQPBenchmark/" in path:
        return (
            "supplementB-sqp-benchmark",
            "benchmark",
            "P2",
            False,
            "DART benchmark records IPC-style gap handling and timing for the "
            "frictionless comparison case.",
            "SQP comparison benchmark",
        )

    if "/supplementB/Utopia/" in path:
        return (
            "supplementB-utopia",
            "manual",
            "P3",
            False,
            "Manual comparison asset remains documented until a CI-safe Utopia "
            "baseline exists.",
            "external Utopia comparison",
        )

    if "/21_scalability/" in path:
        return (
            "paper-scalability",
            "benchmark",
            "P1",
            False,
            "Resolution sweep reports timing, memory, active contacts, CCD calls, "
            "and solver iterations.",
            "paper scalability sweep",
        )

    if "/tb1_diffDt/" in path:
        return (
            "paper-timestep-variants",
            "benchmark",
            "P1",
            True,
            "Time-step sweep preserves positive clearance and records dynamics "
            "and geometric accuracy diagnostics.",
            "paper timestep sweep",
        )

    if "/videoExamples/" in path:
        return (
            "paper-video-examples",
            "example",
            "P1",
            True,
            "Long-horizon headless Filament capture shows nonblank deformation "
            "without intersections.",
            "paper video example",
        )

    if path.startswith("input/paperExamples/11_erleben/"):
        return (
            "paper-erleben",
            "test",
            "P1",
            False,
            "Exact-alignment contact regression preserves nonnegative primitive "
            "clearance.",
            "Erleben exact-alignment case",
        )

    if path.startswith("input/paperExamples/"):
        return (
            "paper-top-level",
            "example",
            "P0",
            True,
            "Paper-facing scene has a runnable DART example, diagnostics, and "
            "long-horizon visual evidence.",
            "top-level paper scene",
        )

    if path.startswith("input/tutorialExamples/BC/"):
        return (
            "tutorial",
            "test",
            "P0",
            False,
            "DBC/NBC regions and time ranges match the scripted upstream behavior.",
            "boundary condition tutorial",
        )

    if path.startswith("input/tutorialExamples/MCO/"):
        return (
            "tutorial",
            "test",
            "P0",
            True,
            "Mesh collision object mode preserves positive clearance under "
            "scripted motion.",
            "mesh collision object tutorial",
        )

    if path.startswith("input/tutorialExamples/advanced/"):
        return (
            "tutorial",
            "test",
            "P0",
            False,
            "Integration option, attachment, or mesh-sequence behavior has a "
            "focused regression.",
            "advanced option tutorial",
        )

    if path.startswith("input/tutorialExamples/initVel/"):
        return (
            "tutorial",
            "test",
            "P0",
            False,
            "Initial linear/angular velocity produces expected motion and remains "
            "intersection-free.",
            "initial velocity tutorial",
        )

    if path.startswith("input/tutorialExamples/"):
        return (
            "tutorial",
            "test",
            "P0",
            False,
            "Basic tutorial scene has a focused regression for material, timestep, "
            "or gravity behavior.",
            "basic tutorial",
        )

    if path.startswith("input/otherExamples/ccd/"):
        return (
            "other-ccd",
            "test",
            "P0",
            False,
            "Conservative CCD line search preserves positive primitive clearance.",
            "CCD validation",
        )

    if path.startswith("input/otherExamples/coDimUnitTests/"):
        return (
            "other-codimensional-unit",
            "test",
            "P0",
            False,
            "Point, segment, or square codimensional contact case remains "
            "intersection-free.",
            "codimensional unit test",
        )

    if path.startswith("input/otherExamples/friction/"):
        return (
            "other-friction",
            "test",
            "P1",
            False,
            "Lagged smoothed friction reproduces the expected stick/slip or "
            "static-friction threshold.",
            "friction validation",
        )

    if path.startswith("input/otherExamples/varyResolution/"):
        return (
            "other-material-resolution-sweep",
            "benchmark",
            "P1",
            False,
            "Resolution sweep records timing, memory, and convergence diagnostics.",
            "resolution sweep",
        )

    if path.startswith("input/otherExamples/varyYoungs/"):
        return (
            "other-material-resolution-sweep",
            "benchmark",
            "P1",
            False,
            "Young's-modulus sweep records conditioning, convergence, and "
            "accuracy diagnostics.",
            "material sweep",
        )

    if path.startswith("input/otherExamples/typical/"):
        return (
            "other-typical",
            "benchmark",
            "P2",
            True,
            "Typical stress scene records performance counters and optional "
            "visual evidence.",
            "typical stress scene",
        )

    if path == "input/otherExamples/barTwist_noCollisions.txt":
        return (
            "other-bar-twist",
            "benchmark",
            "P2",
            False,
            "No-collision stress case isolates material and solver performance.",
            "bar twist without collisions",
        )

    if path == "input/otherExamples/tunnel.txt":
        return (
            "other-tunnel",
            "example",
            "P2",
            True,
            "Tunnel stress example produces long-horizon visual and clearance "
            "diagnostics.",
            "tunnel contact stress scene",
        )

    raise ValueError(f"Unclassified IPC scene: {path}")


def artifact_name(path: str) -> str:
    base = path.removeprefix("input/").removesuffix(".txt")
    return re.sub(r"[^A-Za-z0-9]+", "_", base).strip("_").lower()


def planned_artifact(path: str, target_type: str) -> str:
    name = artifact_name(path)
    if target_type == "example":
        return f"examples/ipc_{name}"
    if target_type == "benchmark":
        return f"bm_ipc_scene_corpus --benchmark_filter={name}"
    if target_type == "manual":
        return f"docs/dev_tasks/ipc_deformable_solver/manual/{name}"
    return f"CTest label ipc_scene::{name}"


def planned_command(path: str, target_type: str) -> str:
    name = artifact_name(path)
    if target_type == "example":
        return (
            f"pixi run ex ipc_{name} -- --headless --frames 240 "
            f"--out /tmp/dart_ipc_scene/{name}"
        )
    if target_type == "benchmark":
        return f"pixi run bm bm_ipc_scene_corpus -- --benchmark_filter={name}"
    if target_type == "manual":
        return f"manual baseline: docs/dev_tasks/ipc_deformable_solver/manual/{name}"
    return f"ctest -L ipc_scene::{name}"


def alias_of(path: Path, upstream_dir: Path) -> str:
    if not path.is_symlink():
        return ""
    return path.resolve().relative_to(upstream_dir).as_posix()


def scene_entry(path: Path, upstream_dir: Path) -> dict[str, object]:
    rel_path = rel_scene_path(path, upstream_dir)
    family, target_type, priority, visual_required, invariant, topic = classify(
        rel_path
    )
    assets = collect_assets(path)
    notes = "Classified from upstream path and scene option family."
    if target_type == "manual":
        notes = (
            "Manual because the upstream comparison depends on an external "
            "baseline that is not yet CI-safe for DART."
        )
    if path.is_symlink():
        notes += " Upstream path is a symlink alias and is tracked separately."
    name = artifact_name(rel_path)
    return {
        "upstream_path": rel_path,
        "upstream_commit": UPSTREAM_COMMIT,
        "alias_of": alias_of(path, upstream_dir),
        "family": family,
        "topic": topic,
        "priority": priority,
        "dart_target_type": target_type,
        "status": "manual" if target_type == "manual" else "planned",
        "dart_artifact": planned_artifact(rel_path, target_type),
        "required_assets_or_importer": assets,
        "expected_invariant": invariant,
        "dart_command_or_ctest_or_benchmark": planned_command(rel_path, target_type),
        "visual_evidence_requirement": (
            f"headless-filament:/tmp/dart_ipc_scene/{name}"
            if visual_required
            else "not-required"
        ),
        "benchmark_profile_artifact": (
            f"benchmark-results/ipc_scene_corpus/{name}.json"
            if target_type == "benchmark"
            else "not-required"
        ),
        "notes_or_gap": notes,
    }


def tracked_txt_paths(upstream_dir: Path) -> list[Path]:
    result = subprocess.run(
        [
            "git",
            "-C",
            upstream_dir.as_posix(),
            "ls-tree",
            "-r",
            "--name-only",
            "HEAD",
            "input",
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    return [
        upstream_dir / line
        for line in result.stdout.splitlines()
        if line.endswith(".txt")
    ]


def upstream_head(upstream_dir: Path) -> str:
    result = subprocess.run(
        ["git", "-C", upstream_dir.as_posix(), "rev-parse", "HEAD"],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    return result.stdout.strip()


def main() -> int:
    args = parse_args()
    upstream_dir = args.upstream_dir.resolve()
    input_dir = upstream_dir / "input"
    if not input_dir.exists():
        raise SystemExit(f"Missing IPC input directory: {input_dir}")
    commit = upstream_head(upstream_dir)
    if commit != UPSTREAM_COMMIT:
        raise SystemExit(
            f"Expected IPC commit {UPSTREAM_COMMIT}, but {upstream_dir} is {commit}"
        )

    txt_paths = tracked_txt_paths(upstream_dir)
    scenes = [scene_entry(path, upstream_dir) for path in txt_paths]
    family_counts = Counter(str(scene["family"]) for scene in scenes)
    target_counts = Counter(str(scene["dart_target_type"]) for scene in scenes)
    symlink_count = sum(1 for path in txt_paths if path.is_symlink())

    manifest = {
        "schema_version": 1,
        "source": {
            "project": "ipc-sim/IPC",
            "repository": UPSTREAM_REPO,
            "commit": UPSTREAM_COMMIT,
            "input_root": "input",
            "txt_scene_path_count": len(scenes),
            "regular_txt_file_count": len(scenes) - symlink_count,
            "symlink_txt_path_count": symlink_count,
        },
        "classification_policy": {
            "target_types": [
                "test",
                "benchmark",
                "example",
                "manual",
                "not-applicable",
            ],
            "priorities": {
                "P0": "Foundation scenes needed before IPC feature parity.",
                "P1": "Paper-facing or high-risk parity scene.",
                "P2": "Stress, comparison, or scaling follow-up scene.",
                "P3": "Manual or optional external-baseline scene.",
            },
            "unclassified_allowed": False,
        },
        "summary": {
            "family_counts": dict(sorted(family_counts.items())),
            "target_type_counts": dict(sorted(target_counts.items())),
        },
        "scenes": scenes,
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Wrote {len(scenes)} IPC scene rows to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
