#!/usr/bin/env python3
"""Generate PLAN-082's rigid-ipc upstream fixture manifest."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
from collections import Counter
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "docs"
    / "plans"
    / "082-rigid-implicit-barrier-contact"
    / "rigid_ipc_fixture_manifest.json"
)
UPSTREAM_REPO = "https://github.com/ipc-sim/rigid-ipc"
UPSTREAM_COMMIT = "23b6ba6fbf8434056444ae106356fd2209136988"
MESH_EXTENSIONS = {
    ".msh",
    ".obj",
    ".ply",
    ".stl",
    ".vtk",
    ".xml",
}
BENCHMARK_PATHS = [
    "tools/benchmark.py",
    "tools/scalability.py",
    "tools/benchmarks/chains.py",
    "tools/benchmarks/codimensional.py",
    "tools/benchmarks/friction.py",
    "tools/benchmarks/mechanisms.py",
    "tools/benchmarks/print_fixtures.py",
    "tools/benchmarks/unit_tests.py",
]
TEST_SOURCE_ROOTS = ["tests/barrier", "tests/ccd"]
COMPARISON_ROOTS = ["comparisons"]
IMPLEMENTED_ROOT_CCD_DATA_ROWS = frozenset(
    f"tests/data/ccd-test-{index:03d}.json" for index in range(4)
)
IMPLEMENTED_KINEMATIC_CCD_DATA_ROWS = frozenset(
    f"tests/data/kinematic/ccd-test-{index:03d}.json" for index in range(13)
)
IMPLEMENTED_WRECKING_BALL_CCD_DATA_ROWS = frozenset(
    f"tests/data/wrecking-ball/ccd-test-{index:03d}.json" for index in range(386)
)
IMPLEMENTED_LARGE_HASHGRID_DATA_ROWS = frozenset(
    f"tests/data/large-rb-hashgrid/large-rb-hashgrid-{index:03d}.json"
    for index in range(2)
)
IMPLEMENTED_FIXTURE_ROWS = {
    "fixtures/3D/friction/incline-plane/slopeTest_highSchoolPhysics_mu=0.49.json": {
        "test": "FrictionThresholdBelowFixtureRowSlides",
        "expected_invariant": (
            "DART covers the audited below-threshold high-school physics "
            "friction row: with tan(theta)=0.5 and mu=0.49, the block remains "
            "intersection-free and continues sliding down-slope."
        ),
        "notes_or_gap": (
            "Covered by DART-owned Fig. 18 paper experiment coverage for the "
            "3D friction fixture and paper alias. The at-threshold mu=0.5 row "
            "and broader friction corpus remain planned until they have "
            "matching DART evidence."
        ),
    },
    "fixtures/paper-figures/18-high-school-physics-friction-test-mu=0.49.json": {
        "test": "FrictionThresholdBelowFixtureRowSlides",
        "expected_invariant": (
            "DART covers the audited below-threshold high-school physics "
            "friction row: with tan(theta)=0.5 and mu=0.49, the block remains "
            "intersection-free and continues sliding down-slope."
        ),
        "notes_or_gap": (
            "Covered by DART-owned Fig. 18 paper experiment coverage for the "
            "3D friction fixture and paper alias. The at-threshold mu=0.5 row "
            "and broader friction corpus remain planned until they have "
            "matching DART evidence."
        ),
    },
    "fixtures/3D/friction/incline-plane/slopeTest_highSchoolPhysics_mu=1.json": {
        "test": "FrictionThresholdHighFixtureRowSticks",
        "expected_invariant": (
            "DART covers the audited high-friction high-school physics row: "
            "with tan(theta)=0.5 and mu=1.0, the block remains "
            "intersection-free and settles without sustained down-slope slide."
        ),
        "notes_or_gap": (
            "Covered by DART-owned Fig. 18 paper experiment coverage for the "
            "3D high-friction fixture row. The at-threshold mu=0.5 row and "
            "broader friction corpus remain planned until they have matching "
            "DART evidence."
        ),
    },
    "fixtures/3D/friction/sliding.json": {
        "test": "SlidingCubeFixtureRowIsBrakedByFriction",
        "expected_invariant": (
            "DART covers the audited 3D sliding friction row: with an initial "
            "tangential speed and mu=0.05, the cube remains intersection-free "
            "and is observably braked relative to a frictionless run."
        ),
        "notes_or_gap": (
            "Covered by DART-owned differential sliding-cube regression for "
            "the upstream 3D friction fixture mechanism. Broader friction "
            "corpus rows remain planned until they have matching DART evidence."
        ),
    },
    "fixtures/3D/friction/spolling-coin.json": {
        "test": "SpinningCoinIsBrakedByFrictionWithoutPenetration",
        "expected_invariant": (
            "DART covers the audited 3D spolling-coin friction row: a spinning "
            "disk remains intersection-free on a frictional support while "
            "contact friction dissipates angular velocity."
        ),
        "notes_or_gap": (
            "Covered by DART-owned Fig. 7 paper experiment coverage for the 3D "
            "friction fixture mechanism. The paper-figure visual alias remains "
            "planned until it has matching example and headless visual evidence."
        ),
    },
    "fixtures/3D/friction/turntable/turntable-mu=1.0.json": {
        "test": "TurntableHighFrictionFixtureRowCarriesRider",
        "expected_invariant": (
            "DART covers the audited 3D high-friction turntable row: a cube "
            "on a rotating kinematic cylinder remains intersection-free while "
            "mu=1.0 contact friction carries it tangentially."
        ),
        "notes_or_gap": (
            "Covered by DART-owned Fig. 13 paper experiment coverage for the "
            "3D high-friction turntable fixture mechanism. The mu=0.0 "
            "turntable row plus paper visual aliases remain planned until "
            "they have matching no-friction and headless visual evidence."
        ),
    },
    "fixtures/3D/friction/turntable/turntable-mu=0.5.json": {
        "test": "TurntableModerateFrictionFixtureRowCarriesRider",
        "expected_invariant": (
            "DART covers the audited 3D moderate-friction turntable row: a "
            "cube on a rotating kinematic cylinder remains intersection-free "
            "while mu=0.5 contact friction carries it tangentially."
        ),
        "notes_or_gap": (
            "Covered by DART-owned Fig. 13 paper experiment coverage for the "
            "3D moderate-friction turntable fixture mechanism. The mu=0.0 "
            "turntable row plus paper visual aliases remain planned until "
            "they have matching no-friction and headless visual evidence."
        ),
    },
    "fixtures/3D/friction/turntable/turntable-mu=0.1.json": {
        "test": "TurntableLowFrictionFixtureRowCarriesRider",
        "expected_invariant": (
            "DART covers the audited 3D low-friction turntable row: a cube on "
            "a rotating kinematic cylinder remains intersection-free while "
            "mu=0.1 contact friction carries it tangentially."
        ),
        "notes_or_gap": (
            "Covered by DART-owned Fig. 13 paper experiment coverage for the "
            "3D low-friction turntable fixture mechanism. The mu=0.0 "
            "turntable row plus paper visual aliases remain planned until "
            "they have matching no-friction and headless visual evidence."
        ),
    },
    "fixtures/3D/unit-tests/tunneling.json": {
        "test": "HighSpeedCubeDoesNotTunnelThroughWall",
        "expected_invariant": (
            "DART covers the audited 3D tunneling row: a rotated cube with a "
            "large time-step velocity toward a fixed wall remains "
            "intersection-free and reports a conservative CCD line-search hit."
        ),
        "notes_or_gap": (
            "Covered by DART-owned high-speed cube-vs-wall runtime coverage "
            "for the 3D unit-test fixture mechanism. Broader 3D unit-test "
            "fixture rows remain planned until they have matching DART "
            "runtime evidence."
        ),
    },
    "fixtures/3D/unit-tests/rotation/rotating-cube.json": {
        "test": "RotatingCubeFixtureRowAdvancesWithoutContact",
        "expected_invariant": (
            "DART covers the audited 3D rotating-cube row: a free cube with "
            "zero gravity and angular velocity advances without contact, stays "
            "finite, and does not translate."
        ),
        "notes_or_gap": (
            "Covered by DART-owned no-contact rotating-cube runtime coverage "
            "for the 3D unit-test fixture mechanism. Other rotation fixture "
            "rows remain planned until they have matching DART runtime "
            "evidence."
        ),
    },
    "fixtures/3D/unit-tests/spinning-cube-over-plane.json": {
        "test": "SpinningCubeOverPlaneFixtureRowAdvancesSafely",
        "expected_invariant": (
            "DART covers the audited 3D spinning-cube-over-plane row: a cube "
            "with zero gravity and high angular velocity spins above a fixed "
            "plane, stays finite, and preserves nonnegative clearance."
        ),
        "notes_or_gap": (
            "Covered by DART-owned no-contact spinning-cube runtime coverage "
            "for the 3D unit-test fixture mechanism. Contacting 3D unit-test "
            "fixture rows remain planned until they have matching DART "
            "runtime evidence."
        ),
    },
    "fixtures/3D/unit-tests/rotation/rotating-sphere.json": {
        "test": "RotatingScaledSphereFixtureRowAdvancesWithoutContact",
        "expected_invariant": (
            "DART covers the audited 3D rotating scaled-sphere row: a free "
            "ellipsoid mesh with zero gravity and angular velocity advances "
            "without contact, stays finite, and does not translate."
        ),
        "notes_or_gap": (
            "Covered by DART-owned no-contact scaled-sphere runtime coverage "
            "for the 3D rotation fixture mechanism. Contacting 3D unit-test "
            "fixture rows remain planned until they have matching DART "
            "runtime evidence."
        ),
    },
    "fixtures/3D/unit-tests/rotation/rotating-ellipsoid-major.json": {
        "test": "RotatingEllipsoidMajorFixtureRowAdvancesWithoutContact",
        "expected_invariant": (
            "DART covers the audited 3D major-axis rotating ellipsoid row: a "
            "free ellipsoid mesh with zero gravity and angular velocity "
            "advances without contact, stays finite, and does not translate."
        ),
        "notes_or_gap": (
            "Covered by DART-owned no-contact major-axis ellipsoid runtime "
            "coverage for the 3D rotation fixture mechanism. Contacting 3D "
            "unit-test fixture rows remain planned until they have matching "
            "DART runtime evidence."
        ),
    },
    "fixtures/3D/unit-tests/rotation/rotating-ellipsoid-intermediate.json": {
        "test": "RotatingEllipsoidIntermediateFixtureRowAdvancesWithoutContact",
        "expected_invariant": (
            "DART covers the audited 3D intermediate-axis rotating ellipsoid "
            "row: a free ellipsoid mesh with zero gravity and angular velocity "
            "advances without contact, stays finite, and does not translate."
        ),
        "notes_or_gap": (
            "Covered by DART-owned no-contact intermediate-axis ellipsoid "
            "runtime coverage for the 3D rotation fixture mechanism. "
            "Contacting 3D unit-test fixture rows remain planned until they "
            "have matching DART runtime evidence."
        ),
    },
    "fixtures/3D/unit-tests/rotation/rotating-ellipsoid-minor.json": {
        "test": "RotatingEllipsoidMinorFixtureRowAdvancesWithoutContact",
        "expected_invariant": (
            "DART covers the audited 3D minor-axis rotating ellipsoid row: a "
            "free ellipsoid mesh with zero gravity and angular velocity "
            "advances without contact, stays finite, and does not translate."
        ),
        "notes_or_gap": (
            "Covered by DART-owned no-contact minor-axis ellipsoid runtime "
            "coverage for the 3D rotation fixture mechanism. Contacting 3D "
            "unit-test fixture rows remain planned until they have matching "
            "DART runtime evidence."
        ),
    },
    "fixtures/3D/unit-tests/rotation/torque-test.json": {
        "test": "TorqueFixtureRowAcceleratesFreeBody",
        "expected_invariant": (
            "DART covers the audited 3D torque rotation row: a free disk-like "
            "body with zero gravity and applied torque remains finite, does "
            "not translate, and gains angular velocity about the torque axis."
        ),
        "notes_or_gap": (
            "Covered by DART-owned no-contact torque runtime coverage for the "
            "3D rotation fixture mechanism. Contacting 3D unit-test fixture "
            "rows remain planned until they have matching DART runtime "
            "evidence."
        ),
    },
    "fixtures/3D/unit-tests/rotation/dzhanibekov.json": {
        "test": "DzhanibekovWingNutFixtureRowAdvancesSafely",
        "expected_invariant": (
            "DART covers the audited 3D Dzhanibekov wing-nut row: a "
            "wing-nut-like rigid mesh with zero gravity, an initial tilt, and "
            "high angular velocity advances safely without contact, stays "
            "finite, and does not translate."
        ),
        "notes_or_gap": (
            "Covered by DART-owned no-contact wing-nut runtime coverage for "
            "the 3D rotation fixture mechanism. Contacting 3D unit-test "
            "fixture rows remain planned until they have matching DART "
            "runtime evidence."
        ),
    },
}
IMPLEMENTED_TEST_SOURCE_ROWS = {
    "tests/barrier/test_barriers.cpp": {
        "artifact": (
            "test_barrier_kernel::IpcBarrierKernel.*; "
            "test_rigid_ipc_barrier::RigidIpcBarrier.*"
        ),
        "command": (
            "pixi run bash -lc 'build/default/cpp/Release/bin/"
            "test_barrier_kernel --gtest_color=no && "
            "build/default/cpp/Release/bin/test_rigid_ipc_barrier "
            "--gtest_color=no --gtest_filter="
            '"RigidIpcBarrier.*:RigidIpcCcdGeometry.*"\''
        ),
        "expected_invariant": (
            "DART barrier tests preserve the upstream C2 clamped-log barrier "
            "scalar, primitive finite-difference derivatives, and rigid "
            "reduced-coordinate derivative mappings."
        ),
        "notes_or_gap": (
            "Covered by DART barrier-kernel and rigid reduced-coordinate "
            "barrier regressions; broader runtime convergence remains tracked "
            "by Phases 3 and 4."
        ),
    },
    "tests/ccd/test_rigid_body_time_of_impact.cpp": {
        "artifact": "test_rigid_ipc_fixture::RigidIpcCcdCase.*",
        "command": (
            "pixi run bash -lc 'build/default/cpp/Release/bin/"
            "test_rigid_ipc_fixture --gtest_color=no "
            '--gtest_filter="RigidIpcCcdCase.*"\''
        ),
        "expected_invariant": (
            "DART rigid CCD tests cover edge-vertex, edge-edge, face-vertex, "
            "rotational trajectory, minimum-separation, kinematic no-zero-time, "
            "and audited conservative-TOI corpus behavior."
        ),
        "notes_or_gap": (
            "Covered by DART rigid CCD parser, residual, interval-subdivision, "
            "rotational-trajectory, kinematic, and conservative-TOI corpus "
            "regressions; rigorous interval arithmetic remains tracked by "
            "Phase 2c."
        ),
    },
    "tests/ccd/test_rigid_body_hash_grid.cpp": {
        "artifact": ("bm_rigid_ipc_solver::" "BM_RigidIpcLargeHashgridSceneBounds/*"),
        "command": (
            "pixi run bm --target bm_rigid_ipc_solver --build-type Release -- "
            "--benchmark_filter=BM_RigidIpcLargeHashgridSceneBounds "
            "--benchmark_out=.benchmark_results/"
            "rigid_ipc_large_hashgrid_source_row.json "
            "--benchmark_out_format=json"
        ),
        "expected_invariant": (
            "DART covers the audited rigid-body hash-grid source row by "
            "loading the large rigid-body hash-grid corpus, computing "
            "conservative swept scene bounds, and proving those bounds contain "
            "the upstream exact scene bounds."
        ),
        "notes_or_gap": (
            "The audited source row's concrete coverage is the large-scene "
            "bounds experiment; DART owns that as benchmark evidence without "
            "exposing an upstream hash-grid API. Generic hash-grid parity "
            "remains tracked by the separate test_hash_grid.cpp row."
        ),
    },
}


def is_large_rb_hashgrid_data(path: str) -> bool:
    return path.startswith("tests/data/large-rb-hashgrid/")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--upstream-dir",
        type=Path,
        required=True,
        help="Path to an ipc-sim/rigid-ipc checkout at the audited commit.",
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def git_lines(upstream_dir: Path, args: list[str]) -> list[str]:
    result = subprocess.run(
        ["git", "-C", upstream_dir.as_posix(), *args],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    return [line for line in result.stdout.splitlines() if line]


def upstream_head(upstream_dir: Path) -> str:
    return git_lines(upstream_dir, ["rev-parse", "HEAD"])[0]


def tracked_paths(upstream_dir: Path, root: str) -> list[str]:
    return sorted(
        git_lines(upstream_dir, ["ls-tree", "-r", "--name-only", "HEAD", root])
    )


def tracked_modes(upstream_dir: Path, root: str) -> dict[str, str]:
    rows = git_lines(upstream_dir, ["ls-tree", "-r", "HEAD", root])
    modes: dict[str, str] = {}
    for row in rows:
        parts = row.split(maxsplit=3)
        if len(parts) == 4:
            modes[parts[3]] = parts[0]
    return modes


def alias_target(path: Path, upstream_dir: Path) -> str:
    if not path.is_symlink():
        return ""
    try:
        return path.resolve().relative_to(upstream_dir).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def normalize_asset(value: str, upstream_dir: Path) -> str:
    token = value.strip().replace("\\", "/")
    if not token or token.startswith("#"):
        return ""

    suffix = Path(token).suffix.lower()
    if suffix not in MESH_EXTENSIONS:
        return ""

    candidates = [Path(token)]
    if not token.startswith("meshes/"):
        candidates.append(Path("meshes") / token)

    for candidate in candidates:
        if (upstream_dir / candidate).exists():
            return candidate.as_posix()

    return candidates[-1].as_posix()


def collect_assets(value: Any, upstream_dir: Path) -> list[str]:
    assets: set[str] = set()

    def visit(node: Any) -> None:
        if isinstance(node, dict):
            for child in node.values():
                visit(child)
        elif isinstance(node, list):
            for child in node:
                visit(child)
        elif isinstance(node, str):
            asset = normalize_asset(node, upstream_dir)
            if asset:
                assets.add(asset)

    visit(value)
    return sorted(assets)


def slug(path: str) -> str:
    stem = path.rsplit("/", 1)[-1].rsplit(".", 1)[0].lower()
    return re.sub(r"[^a-z0-9]+", "_", stem).strip("_")


def fixture_family(path: str) -> str:
    parts = path.split("/")
    if len(parts) < 3:
        return "fixture"
    if parts[1] == "paper-figures":
        if len(parts) > 2 and parts[2] == "16-unit-tests":
            return "paper-unit-tests"
        if parts[-1].startswith("18-high-school-physics"):
            return "paper-friction-threshold"
        return "paper-figure"
    if parts[1] in {"2D", "3D"}:
        if len(parts) > 3 and parts[2] in {
            "bypass",
            "chain",
            "cogs",
            "codimensional",
            "filling-box",
            "friction",
            "mechanisms",
            "newtons-cradle",
            "piles",
            "restitution",
            "saw",
            "scalability",
            "simple",
            "stacking",
            "unit-tests",
        }:
            return f"{parts[1].lower()}-{parts[2]}"
        return f"{parts[1].lower()}-scene"
    return "fixture"


def classify_entry(path: str, source_kind: str) -> dict[str, str]:
    if source_kind == "fixture":
        family = fixture_family(path)
        if family in {"paper-figure", "3d-mechanisms", "3d-chain", "3d-examples"}:
            return {
                "family": family,
                "topic": "paper or promoted visual rigid-body scene",
                "priority": "P0" if family == "paper-figure" else "P1",
                "dart_target_type": "example",
                "expected_invariant": (
                    "DART replay remains intersection-free and has long-horizon "
                    "headless visual evidence plus solver diagnostics."
                ),
                "visual_evidence_requirement": "headless-filament-required",
                "benchmark_profile_artifact": "not-required",
            }
        if family in {"3d-scalability", "2d-stacking", "2d-filling-box"}:
            return {
                "family": family,
                "topic": "rigid contact scalability or packing benchmark",
                "priority": "P1",
                "dart_target_type": "benchmark",
                "expected_invariant": (
                    "DART benchmark records timing, memory, contact counts, "
                    "CCD calls, solver iterations, and final clearance."
                ),
                "visual_evidence_requirement": "optional-contact-sheet",
                "benchmark_profile_artifact": (
                    "benchmark JSON and /usr/bin/time profile packet"
                ),
            }
        if family in {
            "paper-unit-tests",
            "paper-friction-threshold",
            "3d-unit-tests",
            "3d-friction",
            "3d-minimum-separation",
            "2d-restitution",
            "2d-simple",
        }:
            return {
                "family": family,
                "topic": "rigid contact correctness regression",
                "priority": "P0",
                "dart_target_type": "test",
                "expected_invariant": (
                    "DART focused test preserves nonnegative clearance, "
                    "momentum/energy behavior where applicable, and expected "
                    "restitution or friction outcome."
                ),
                "visual_evidence_requirement": "not-required",
                "benchmark_profile_artifact": "not-required",
            }
        return {
            "family": family,
            "topic": "rigid scene parity",
            "priority": "P2",
            "dart_target_type": "example",
            "expected_invariant": (
                "DART scene import and replay have documented diagnostics and "
                "do not introduce intersections."
            ),
            "visual_evidence_requirement": "headless-filament-required",
            "benchmark_profile_artifact": "not-required",
        }

    if source_kind == "test-data":
        parts = path.split("/")
        family_token = parts[2] if len(parts) > 3 else slug(path)
        family = "ccd-data-" + family_token.replace("_", "-")
        if is_large_rb_hashgrid_data(path):
            return {
                "family": family,
                "topic": "rigid hash-grid broad-phase data regression",
                "priority": "P1",
                "dart_target_type": "benchmark",
                "expected_invariant": (
                    "DART broad-phase coverage records conservative scene "
                    "bounds for the large rigid-body hash-grid corpus and "
                    "emits reproducible profile evidence."
                ),
                "visual_evidence_requirement": "not-required",
                "benchmark_profile_artifact": "hash-grid benchmark JSON packet",
            }
        return {
            "family": family,
            "topic": "rigid CCD data regression",
            "priority": "P0",
            "dart_target_type": "test",
            "expected_invariant": (
                "DART CCD test reports a conservative time of impact and "
                "nonnegative post-step separation."
            ),
            "visual_evidence_requirement": "not-required",
            "benchmark_profile_artifact": "not-required",
        }

    if source_kind == "test-source":
        family = "unit-" + path.split("/")[1]
        return {
            "family": family,
            "topic": "rigid barrier or CCD algorithm unit test",
            "priority": "P0",
            "dart_target_type": "test",
            "expected_invariant": (
                "DART unit test covers the same barrier, hash-grid, or rigid "
                "time-of-impact responsibility with derivative or conservative "
                "bounds where applicable."
            ),
            "visual_evidence_requirement": "not-required",
            "benchmark_profile_artifact": "not-required",
        }

    if source_kind == "benchmark-script":
        return {
            "family": "benchmark-script",
            "topic": "rigid IPC benchmark harness",
            "priority": "P1",
            "dart_target_type": "benchmark",
            "expected_invariant": (
                "DART benchmark exposes the same scene family or scalability "
                "sweep with reproducible JSON output."
            ),
            "visual_evidence_requirement": "not-required",
            "benchmark_profile_artifact": "benchmark JSON packet",
        }

    if source_kind == "comparison":
        parts = path.split("/")
        family = "comparison-" + (parts[1].lower() if len(parts) > 1 else "misc")
        return {
            "family": family,
            "topic": "external baseline comparison",
            "priority": "P2",
            "dart_target_type": "manual",
            "expected_invariant": (
                "DART keeps a documented baseline decision or manual comparison "
                "path without adding runtime dependencies."
            ),
            "visual_evidence_requirement": "manual-if-promoted",
            "benchmark_profile_artifact": "manual-baseline-packet",
        }

    raise ValueError(f"unknown source kind: {source_kind}")


def row_for_path(path: str, source_kind: str, upstream_dir: Path) -> dict[str, Any]:
    row = classify_entry(path, source_kind)
    command_slug = slug(path)
    if source_kind == "fixture":
        command = f"ctest -L rigid_ipc_fixture::{command_slug}"
        artifact = f"CTest label rigid_ipc_fixture::{command_slug}"
    elif source_kind == "test-data":
        command = f"ctest -L rigid_ipc_data::{command_slug}"
        artifact = f"CTest label rigid_ipc_data::{command_slug}"
    elif source_kind == "test-source":
        command = f"ctest -L rigid_ipc_algorithm::{command_slug}"
        artifact = f"CTest label rigid_ipc_algorithm::{command_slug}"
    elif source_kind == "benchmark-script":
        command = f"pixi run bm-rigid-ipc -- --benchmark_filter={command_slug}"
        artifact = f"benchmark rigid_ipc::{command_slug}"
    else:
        command = "manual baseline comparison"
        artifact = f"manual baseline {path}"

    payload: Any = {}
    if path.endswith(".json"):
        try:
            payload = json.loads((upstream_dir / path).read_text(errors="replace"))
        except json.JSONDecodeError:
            payload = {}

    status = "planned"
    expected_invariant = row["expected_invariant"]
    notes_or_gap = (
        "Classified from upstream rigid-ipc path; retire this planned row "
        "only after DART has matching code, tests, benchmarks, and evidence."
    )
    if source_kind == "test-data" and is_large_rb_hashgrid_data(path):
        command = f"pixi run bm-rigid-ipc -- --benchmark_filter={command_slug}"
        artifact = f"benchmark rigid_ipc_hashgrid::{command_slug}"
        notes_or_gap = (
            "The audited upstream hash-grid source keeps this large-scene "
            "benchmark commented out; retire this planned row only after DART "
            "has matching broad-phase bounds coverage and benchmark profile "
            "evidence."
        )
    if source_kind == "fixture" and path in IMPLEMENTED_FIXTURE_ROWS:
        implemented = IMPLEMENTED_FIXTURE_ROWS[path]
        status = "implemented"
        test_name = implemented["test"]
        artifact = (
            f"test_rigid_ipc_paper_experiments::"
            f"RigidIpcPaperExperiments.{test_name}"
        )
        command = (
            "pixi run bash -lc 'build/default/cpp/Release/bin/"
            "test_rigid_ipc_paper_experiments --gtest_color=no "
            f"--gtest_filter=RigidIpcPaperExperiments.{test_name}'"
        )
        expected_invariant = implemented["expected_invariant"]
        notes_or_gap = implemented["notes_or_gap"]
    elif source_kind == "test-data" and path in IMPLEMENTED_LARGE_HASHGRID_DATA_ROWS:
        status = "implemented"
        artifact = (
            "bm_rigid_ipc_solver::"
            "BM_RigidIpcLargeHashgridSceneBounds/"
            f"{command_slug}"
        )
        command = (
            "pixi run bm --target bm_rigid_ipc_solver --build-type Release -- "
            "--benchmark_filter="
            f"BM_RigidIpcLargeHashgridSceneBounds/{command_slug} "
            "--benchmark_out=.benchmark_results/"
            f"rigid_ipc_{command_slug}.json --benchmark_out_format=json"
        )
        expected_invariant = (
            "DART loads compact audited large rigid-body hash-grid bounds, "
            "computes conservative swept scene bounds, and proves the benchmark "
            "bounds contain the upstream exact scene bounds."
        )
        notes_or_gap = (
            "Covered by DART-owned benchmark coverage for the audited large "
            "rigid-body hash-grid rows; the benchmark fixture records the "
            "upstream source hashes and exact scene bounds without vendoring "
            "the full upstream JSON data."
        )
    elif source_kind == "test-data" and path in IMPLEMENTED_ROOT_CCD_DATA_ROWS:
        status = "implemented"
        artifact = (
            "test_rigid_ipc_fixture::"
            "RigidIpcCcdCase.EvaluatesAuditedUpstreamRootCcdRowsAsMisses"
        )
        command = (
            "pixi run bash -lc 'build/default/cpp/Release/bin/"
            "test_rigid_ipc_fixture --gtest_color=no "
            "--gtest_filter="
            "RigidIpcCcdCase.EvaluatesAuditedUpstreamRootCcdRowsAsMisses'"
        )
        expected_invariant = (
            "DART loads the audited root direct-CCD row and reports the current "
            "full-step miss outcome without parser or topology diagnostics."
        )
        notes_or_gap = (
            "Covered by hermetic DART load and evaluator regressions for the "
            "first audited root direct-CCD rows; corpus-scale interval-root "
            "parity remains tracked by Phase 2c."
        )
    elif source_kind == "test-data" and path in IMPLEMENTED_KINEMATIC_CCD_DATA_ROWS:
        status = "implemented"
        artifact = (
            "test_rigid_ipc_fixture::"
            "RigidIpcCcdCase.EvaluatesAuditedKinematicRowsWithoutZeroTimeHits"
        )
        command = (
            "pixi run bash -lc 'build/default/cpp/Release/bin/"
            "test_rigid_ipc_fixture --gtest_color=no "
            "--gtest_filter="
            "RigidIpcCcdCase.EvaluatesAuditedKinematicRowsWithoutZeroTimeHits'"
        )
        expected_invariant = (
            "DART loads the audited kinematic direct-CCD row and preserves the "
            "upstream guard against zero-time hits when the row starts "
            "separated."
        )
        notes_or_gap = (
            "Covered by hermetic DART load and evaluator regressions for the "
            "audited kinematic direct-CCD rows; corpus-scale interval-root "
            "parity remains tracked by Phase 2c."
        )
    elif source_kind == "test-data" and path in IMPLEMENTED_WRECKING_BALL_CCD_DATA_ROWS:
        status = "implemented"
        artifact = (
            "test_rigid_ipc_fixture::"
            "RigidIpcCcdCase.EvaluatesAuditedWreckingBallCorpusConservatively"
        )
        command = (
            "pixi run bash -lc 'build/default/cpp/Release/bin/"
            "test_rigid_ipc_fixture --gtest_color=no "
            "--gtest_filter="
            "RigidIpcCcdCase.EvaluatesAuditedWreckingBallCorpusConservatively'"
        )
        expected_invariant = (
            "DART loads the audited wrecking-ball direct-CCD row and, when a "
            "time of impact is reported, replays the truncated interval through "
            "that conservative bound without another hit."
        )
        notes_or_gap = (
            "Covered by hermetic DART load and conservative-TOI evaluator "
            "regressions for the audited wrecking-ball direct-CCD corpus; "
            "corpus-scale interval-root parity remains tracked by Phase 2c."
        )
    elif source_kind == "test-source" and path in IMPLEMENTED_TEST_SOURCE_ROWS:
        status = "implemented"
        implemented = IMPLEMENTED_TEST_SOURCE_ROWS[path]
        artifact = implemented["artifact"]
        command = implemented["command"]
        expected_invariant = implemented["expected_invariant"]
        notes_or_gap = implemented["notes_or_gap"]

    return {
        "upstream_path": path,
        "upstream_commit": UPSTREAM_COMMIT,
        "source_kind": source_kind,
        "alias_of": alias_target(upstream_dir / path, upstream_dir),
        **row,
        "status": status,
        "dart_artifact": artifact,
        "required_assets_or_importer": collect_assets(payload, upstream_dir),
        "expected_invariant": expected_invariant,
        "dart_command_or_ctest_or_benchmark": command,
        "visual_evidence_requirement": row["visual_evidence_requirement"],
        "benchmark_profile_artifact": row["benchmark_profile_artifact"],
        "notes_or_gap": notes_or_gap,
    }


def collect_manifest(upstream_dir: Path) -> dict[str, Any]:
    upstream_dir = upstream_dir.resolve()
    head = upstream_head(upstream_dir)
    if head != UPSTREAM_COMMIT:
        raise SystemExit(f"upstream checkout must be {UPSTREAM_COMMIT}, got {head}")

    fixture_paths = [
        path
        for path in tracked_paths(upstream_dir, "fixtures")
        if path.endswith(".json")
    ]
    test_data_paths = [
        path
        for path in tracked_paths(upstream_dir, "tests/data")
        if path.endswith(".json")
    ]
    test_source_paths = [
        path
        for root in TEST_SOURCE_ROOTS
        for path in tracked_paths(upstream_dir, root)
        if path.endswith((".cpp", ".hpp"))
    ]
    benchmark_paths = [
        path for path in BENCHMARK_PATHS if (upstream_dir / path).exists()
    ]
    comparison_paths = [
        path
        for root in COMPARISON_ROOTS
        for path in tracked_paths(upstream_dir, root)
        if path.endswith((".cpp", ".json", ".py", ".sh", ".xml"))
    ]

    fixture_modes = tracked_modes(upstream_dir, "fixtures")
    entries = [row_for_path(path, "fixture", upstream_dir) for path in fixture_paths]
    entries.extend(
        row_for_path(path, "test-data", upstream_dir) for path in test_data_paths
    )
    entries.extend(
        row_for_path(path, "test-source", upstream_dir) for path in test_source_paths
    )
    entries.extend(
        row_for_path(path, "benchmark-script", upstream_dir) for path in benchmark_paths
    )
    entries.extend(
        row_for_path(path, "comparison", upstream_dir) for path in comparison_paths
    )
    entries = sorted(
        entries, key=lambda row: (row["source_kind"], row["upstream_path"])
    )

    source_kind_counts = Counter(row["source_kind"] for row in entries)
    family_counts = Counter(row["family"] for row in entries)
    target_counts = Counter(row["dart_target_type"] for row in entries)

    return {
        "schema_version": 1,
        "source": {
            "project": "ipc-sim/rigid-ipc",
            "repository": UPSTREAM_REPO,
            "commit": UPSTREAM_COMMIT,
            "fixture_json_path_count": len(fixture_paths),
            "regular_fixture_json_file_count": sum(
                1 for path in fixture_paths if fixture_modes.get(path) == "100644"
            ),
            "symlink_fixture_json_path_count": sum(
                1 for path in fixture_paths if fixture_modes.get(path) == "120000"
            ),
            "test_data_json_path_count": len(test_data_paths),
            "test_source_path_count": len(test_source_paths),
            "benchmark_script_path_count": len(benchmark_paths),
            "comparison_path_count": len(comparison_paths),
        },
        "classification_policy": {
            "source_kinds": [
                "fixture",
                "test-data",
                "test-source",
                "benchmark-script",
                "comparison",
            ],
            "target_types": ["test", "benchmark", "example", "manual"],
            "priorities": {
                "P0": "Foundation correctness or paper-facing parity gate.",
                "P1": "High-risk paper, benchmark, or promoted example coverage.",
                "P2": "Stress, comparison, or long-tail fixture follow-up.",
            },
            "unclassified_allowed": False,
        },
        "summary": {
            "source_kind_counts": dict(sorted(source_kind_counts.items())),
            "family_counts": dict(sorted(family_counts.items())),
            "target_type_counts": dict(sorted(target_counts.items())),
        },
        "entries": entries,
    }


def main() -> int:
    args = parse_args()
    manifest = collect_manifest(args.upstream_dir)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(manifest, indent=2) + "\n")
    print(
        "Rigid IPC fixture manifest generated: "
        f"{len(manifest['entries'])} entries -> {args.output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
