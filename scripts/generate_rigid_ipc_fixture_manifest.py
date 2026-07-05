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
    "fixtures/3D/friction/rolling/cone.json": {
        "test": "RollingConeFixtureRowAdvancesWithContact",
        "expected_invariant": (
            "DART covers the audited 3D rolling-cone row: a tilted cone with "
            "initial tangential velocity advances over a fixed frictional "
            "plane, activates rigid IPC contact, stays finite, preserves "
            "clearance, and develops angular velocity."
        ),
        "notes_or_gap": (
            "Covered by DART-owned rolling-cone runtime coverage for the "
            "upstream 3D friction fixture mechanism. The oloid row remains "
            "planned until it has matching DART evidence."
        ),
    },
    "fixtures/3D/friction/arch/arch-25-stones.json": {
        "test": "TwentyFiveVoussoirFrictionArchFixtureRowStands",
        "expected_invariant": (
            "DART covers the audited 25-stone arch row: a 25-voussoir "
            "frictional arch stands on a fixed support, activates rigid IPC "
            "contact, stays finite, and preserves nonnegative support "
            "clearance without keystone collapse."
        ),
        "notes_or_gap": (
            "Covered by DART-owned 25-voussoir arch runtime coverage for the "
            "upstream 3D friction fixture mechanism. Larger arch rows and the "
            "Fig. 11 visual alias remain planned until they have matching "
            "DART evidence."
        ),
    },
    "fixtures/3D/friction/card-house/card-tent.json": {
        "test": "CardTentFixtureRowStaysUprightWithFriction",
        "expected_invariant": (
            "DART covers the audited 3D card-tent row: two inclined card "
            "bodies stand on a fixed frictional support, activate rigid IPC "
            "contact, stay finite, preserve upright height, and report no "
            "meaningful native overlap."
        ),
        "notes_or_gap": (
            "Covered by DART-owned two-card tent runtime coverage for the "
            "upstream 3D friction fixture mechanism. Larger card-house rows "
            "remain planned until they have matching DART evidence."
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
    "fixtures/3D/unit-tests/tunnel/2-walls.json": {
        "test": "TwoWallTunnelCubeStaysBetweenWallsFixtureRow",
        "expected_invariant": (
            "DART covers the audited 3D two-wall tunnel row: a unit cube flies "
            "down a tight (1 mm) fixed floor/ceiling corridor at high speed, "
            "stays finite, keeps advancing along the tunnel axis, and never "
            "tunnels out through either fixed wall."
        ),
        "notes_or_gap": (
            "Covered by DART-owned two-wall tunnel runtime coverage for the 3D "
            "unit-test fixture mechanism. The three-wall, four-wall, and 8K "
            "tunnel rows remain planned: at their tighter 0.1 mm clearance "
            "DART's lagged-friction stiction freezes the fast cube (A/B "
            "evidence: default friction ~3 mm travel with 12 active friction "
            "constraints vs frictionIterations=0 ~1.1 m intersection-free "
            "traversal), not the conservative curved-CCD line search; and the "
            "dense 8K walls are too slow for a unit test pending dense-contact "
            "performance work."
        ),
    },
    "fixtures/3D/unit-tests/erleben/cliff-edges.json": {
        "test": "ErlebenCliffEdgesFixtureRowStaysSeparated",
        "expected_invariant": (
            "DART covers the audited Erleben cliff-edges row: a cube falls "
            "under gravity onto a fixed cliff-edge mesh, activates rigid IPC "
            "contact, stays finite, and reports no meaningful native overlap "
            "after each step."
        ),
        "notes_or_gap": (
            "Covered by DART-owned cube-on-Erleben-cliff runtime coverage for "
            "the 3D unit-test fixture mechanism. Other Erleben rows remain "
            "planned until they have topology-specific DART runtime evidence."
        ),
    },
    "fixtures/3D/unit-tests/erleben/internal-edges.json": {
        "test": "ErlebenInternalEdgesFixtureRowStaysSeparated",
        "expected_invariant": (
            "DART covers the audited Erleben internal-edges row: a cube falls "
            "under gravity onto a fixed internal-edge mesh, activates rigid "
            "IPC contact, stays finite, and reports no meaningful native "
            "overlap after each step."
        ),
        "notes_or_gap": (
            "Covered by DART-owned cube-on-Erleben-internal-edges runtime "
            "coverage for the 3D unit-test fixture mechanism. Other Erleben "
            "rows remain planned until they have topology-specific DART "
            "runtime evidence."
        ),
    },
    "fixtures/3D/unit-tests/erleben/sliding-spike.json": {
        "test": "ErlebenSlidingSpikeFixtureRowStaysSeparated",
        "expected_invariant": (
            "DART covers the audited Erleben sliding-spike row: an inverted "
            "spike slides across a fixed plane, activates rigid IPC contact, "
            "stays finite, advances laterally, and reports no meaningful "
            "native overlap after each step."
        ),
        "notes_or_gap": (
            "Covered by DART-owned sliding-spike-on-plane runtime coverage "
            "for the 3D unit-test fixture mechanism. Other Erleben rows "
            "remain planned until they have topology-specific DART runtime "
            "evidence."
        ),
    },
    "fixtures/3D/unit-tests/erleben/sliding-wedge.json": {
        "test": "ErlebenSlidingWedgeFixtureRowStaysSeparated",
        "expected_invariant": (
            "DART covers the audited Erleben sliding-wedge row: an inverted "
            "wedge slides across a fixed plane, activates rigid IPC contact, "
            "stays finite, advances laterally, and reports no meaningful "
            "native overlap after each step."
        ),
        "notes_or_gap": (
            "Covered by DART-owned sliding-wedge-on-plane runtime coverage "
            "for the 3D unit-test fixture mechanism. Other Erleben rows "
            "remain planned until they have topology-specific DART runtime "
            "evidence."
        ),
    },
    "fixtures/3D/unit-tests/erleben/spikes.json": {
        "test": "ErlebenSpikesFixtureRowStaysSeparated",
        "expected_invariant": (
            "DART covers the audited Erleben spikes row: an inverted spike "
            "rests on a fixed upright spike, activates rigid IPC contact, "
            "stays finite, and reports no meaningful native overlap after "
            "each step."
        ),
        "notes_or_gap": (
            "Covered by DART-owned spike-on-spike runtime coverage for the "
            "3D unit-test fixture mechanism. Other Erleben rows remain "
            "planned until they have topology-specific DART runtime evidence."
        ),
    },
    "fixtures/3D/unit-tests/erleben/wedges.json": {
        "test": "ErlebenWedgesFixtureRowStaysSeparated",
        "expected_invariant": (
            "DART covers the audited Erleben wedges row: an inverted wedge "
            "rests on a fixed upright wedge, activates rigid IPC contact, "
            "stays finite, and reports no meaningful native overlap after "
            "each step."
        ),
        "notes_or_gap": (
            "Covered by DART-owned wedge-on-wedge runtime coverage for the "
            "3D unit-test fixture mechanism. Other Erleben rows remain "
            "planned until they have topology-specific DART runtime evidence."
        ),
    },
    "fixtures/3D/unit-tests/erleben/spike-and-wedge.json": {
        "test": "ErlebenSpikeAndWedgeFixtureRowStaysSeparated",
        "expected_invariant": (
            "DART covers the audited Erleben spike-and-wedge row: an inverted "
            "spike rests on a fixed upright wedge, activates rigid IPC "
            "contact, stays finite, and reports no meaningful native overlap "
            "after each step."
        ),
        "notes_or_gap": (
            "Covered by DART-owned spike-on-wedge runtime coverage for the "
            "3D unit-test fixture mechanism. Crack and hole Erleben rows "
            "remain planned until they have topology-specific DART runtime "
            "evidence."
        ),
    },
    "fixtures/3D/unit-tests/erleben/spike-in-crack.json": {
        "test": "ErlebenSpikeInCrackFixtureRowStaysSeparated",
        "expected_invariant": (
            "DART covers the audited Erleben spike-in-crack row: an inverted "
            "spike advances through a fixed crack mesh under the upstream "
            "velocity and force direction, activates rigid IPC contact, stays "
            "finite, and reports no meaningful native overlap after each step."
        ),
        "notes_or_gap": (
            "Covered by DART-owned spike-in-crack runtime coverage for the "
            "3D unit-test fixture mechanism. The hole row remains planned "
            "until it has topology-specific DART runtime evidence."
        ),
    },
    "fixtures/3D/unit-tests/erleben/wedge-in-crack.json": {
        "test": "ErlebenWedgeInCrackFixtureRowStaysSeparated",
        "expected_invariant": (
            "DART covers the audited Erleben wedge-in-crack row: an inverted "
            "wedge advances through a fixed crack mesh under the upstream "
            "velocity and force direction, activates rigid IPC contact, stays "
            "finite, and reports no meaningful native overlap after each step."
        ),
        "notes_or_gap": (
            "Covered by DART-owned wedge-in-crack runtime coverage for the "
            "3D unit-test fixture mechanism. The hole row remains planned "
            "until it has topology-specific DART runtime evidence."
        ),
    },
    "fixtures/3D/unit-tests/erleben/spike-in-hole.json": {
        "test": "ErlebenSpikeInHoleFixtureRowStaysSeparated",
        "expected_invariant": (
            "DART covers the audited Erleben spike-in-hole row: an inverted "
            "spike advances through a fixed hole mesh under the upstream "
            "velocity direction, activates rigid IPC contact, stays finite, "
            "and reports no meaningful native overlap after each step."
        ),
        "notes_or_gap": (
            "Covered by DART-owned spike-in-hole runtime coverage for the 3D "
            "unit-test fixture mechanism. This retires the audited Erleben "
            "unit-test fixture rows tracked in the current P0 manifest slice."
        ),
    },
    "fixtures/3D/unit-tests/tessellated-plane/two-triangles.json": {
        "test": "CubeSettlesOnTwoTrianglePlaneFixtureRow",
        "expected_invariant": (
            "DART covers the audited 3D two-triangle tessellated-plane row: a "
            "cube falls onto a fixed two-triangle mesh plane, activates rigid "
            "IPC contact, stays finite, and preserves nonnegative clearance."
        ),
        "notes_or_gap": (
            "Covered by DART-owned cube-on-two-triangle-plane runtime "
            "coverage for the 3D unit-test fixture mechanism. The 8K "
            "tessellated-plane row is covered separately by topology-specific "
            "runtime evidence."
        ),
    },
    "fixtures/3D/unit-tests/tessellated-plane/two-triangles-tet.json": {
        "test": "TetCornerFallsOnTwoTrianglePlaneFixtureRow",
        "expected_invariant": (
            "DART covers the audited 3D two-triangle tet row: a tetrahedral "
            "corner falls under gravity onto a fixed two-triangle mesh plane, "
            "activates rigid IPC contact, stays finite, and reports no "
            "meaningful native overlap after each step."
        ),
        "notes_or_gap": (
            "Covered by DART-owned tet-corner-on-two-triangle-plane runtime "
            "coverage for the 3D unit-test fixture mechanism. The 8K "
            "tessellated-plane row is covered separately by topology-specific "
            "runtime evidence."
        ),
    },
    "fixtures/3D/unit-tests/tessellated-plane/8K-triangles.json": {
        "test": "CubeContactsEightKTrianglePlaneFixtureRow",
        "expected_invariant": (
            "DART covers the audited 8K tessellated-plane row: a cube starts "
            "near a fixed 8192-triangle mesh plane, activates rigid IPC "
            "contact, stays finite, and preserves nonnegative clearance."
        ),
        "notes_or_gap": (
            "Covered by DART-owned 64x64 tessellated-plane runtime coverage "
            "for the 3D unit-test fixture mechanism."
        ),
    },
    "fixtures/3D/unit-tests/5-cubes.json": {
        "test": "FiveCubesFixtureRowStacksWithoutPenetration",
        "expected_invariant": (
            "DART covers the audited 3D five-cubes row: five aligned cubes "
            "fall under gravity onto a fixed support, activate stacked rigid "
            "IPC contacts, stay finite, and preserve nonnegative support and "
            "cube-cube clearance."
        ),
        "notes_or_gap": (
            "Covered by DART-owned stacked-box runtime coverage for the 3D "
            "unit-test fixture mechanism and its non-visual Fig. 16 "
            "paper-unit alias. Edge-feature and tessellated stress rows "
            "remain planned until they have matching DART runtime evidence."
        ),
    },
    "fixtures/paper-figures/16-unit-tests/5-cubes.json": {
        "test": "FiveCubesFixtureRowStacksWithoutPenetration",
        "expected_invariant": (
            "DART covers the non-visual Fig. 16 five-cubes unit-test alias: "
            "five aligned cubes fall under gravity onto a fixed support, "
            "activate stacked rigid IPC contacts, stay finite, and preserve "
            "nonnegative support and cube-cube clearance."
        ),
        "notes_or_gap": (
            "This paper-unit row aliases the audited 3D five-cubes unit-test "
            "fixture, so it is retired by the same DART-owned runtime "
            "coverage. Visual paper-figure rows still require example and "
            "headless visual evidence before retirement."
        ),
    },
    "fixtures/3D/unit-tests/cube-falling-on-edge.json": {
        "test": "CubeFallingOnEdgeFixtureRowStaysSeparated",
        "expected_invariant": (
            "DART covers the audited 3D cube-falling-on-edge row: a tilted "
            "cube falls under gravity onto a separate tilted fixed box edge, "
            "activates rigid IPC contact, stays finite, and reports no "
            "meaningful native overlap after each step."
        ),
        "notes_or_gap": (
            "Covered by DART-owned tilted-cube-on-edge runtime coverage for "
            "the 3D unit-test fixture mechanism. Edge-edge rows remain "
            "planned until they have topology-specific DART runtime evidence."
        ),
    },
    "fixtures/3D/unit-tests/face-vertex.json": {
        "test": "FaceVertexFixtureRowStaysSeparated",
        "expected_invariant": (
            "DART covers the audited 3D face-vertex row: a tetrahedral "
            "pyramid face falls under gravity toward a separate fixed "
            "tetrahedral pyramid vertex, activates rigid IPC contact, stays "
            "finite, and reports no meaningful native overlap after each step."
        ),
        "notes_or_gap": (
            "Covered by DART-owned tetrahedral face-vertex runtime coverage "
            "for the 3D unit-test fixture mechanism and its non-visual Fig. 16 "
            "paper-unit alias. Edge-edge rows remain planned until they have "
            "topology-specific DART runtime evidence."
        ),
    },
    "fixtures/paper-figures/16-unit-tests/face-vertex.json": {
        "test": "FaceVertexFixtureRowStaysSeparated",
        "expected_invariant": (
            "DART covers the non-visual Fig. 16 face-vertex unit-test alias: "
            "a tetrahedral pyramid face falls under gravity toward a separate "
            "fixed tetrahedral pyramid vertex, activates rigid IPC contact, "
            "stays finite, and reports no meaningful native overlap after "
            "each step."
        ),
        "notes_or_gap": (
            "This paper-unit row aliases the audited 3D face-vertex unit-test "
            "fixture, so it is retired by the same DART-owned runtime "
            "coverage. Visual paper-figure rows still require example and "
            "headless visual evidence before retirement."
        ),
    },
    "fixtures/3D/unit-tests/vertex-face.json": {
        "test": "VertexFaceFixtureRowStaysSeparated",
        "expected_invariant": (
            "DART covers the audited 3D vertex-face row: a tetrahedral "
            "pyramid vertex falls under gravity toward a separate fixed "
            "tetrahedral pyramid face, activates rigid IPC contact, stays "
            "finite, and reports no meaningful native overlap after each step."
        ),
        "notes_or_gap": (
            "Covered by DART-owned tetrahedral vertex-face runtime coverage "
            "for the 3D unit-test fixture mechanism and its non-visual Fig. 16 "
            "paper-unit alias. Edge-edge rows remain planned until they have "
            "topology-specific DART runtime evidence."
        ),
    },
    "fixtures/paper-figures/16-unit-tests/vertex-face.json": {
        "test": "VertexFaceFixtureRowStaysSeparated",
        "expected_invariant": (
            "DART covers the non-visual Fig. 16 vertex-face unit-test alias: "
            "a tetrahedral pyramid vertex falls under gravity toward a "
            "separate fixed tetrahedral pyramid face, activates rigid IPC "
            "contact, stays finite, and reports no meaningful native overlap "
            "after each step."
        ),
        "notes_or_gap": (
            "This paper-unit row aliases the audited 3D vertex-face unit-test "
            "fixture, so it is retired by the same DART-owned runtime "
            "coverage. Visual paper-figure rows still require example and "
            "headless visual evidence before retirement."
        ),
    },
    "fixtures/3D/unit-tests/vertex-vertex.json": {
        "test": "VertexVertexFixtureRowStaysSeparated",
        "expected_invariant": (
            "DART covers the audited 3D vertex-vertex row: a tetrahedral "
            "corner vertex falls under gravity toward a separate fixed "
            "tetrahedral corner vertex above a fixed support plane, activates "
            "rigid IPC contact, stays finite, and reports no meaningful native "
            "overlap after each step."
        ),
        "notes_or_gap": (
            "Covered by DART-owned tetrahedral vertex-vertex runtime coverage "
            "for the 3D unit-test fixture mechanism and its non-visual Fig. 16 "
            "paper-unit alias. Edge-edge rows remain planned until they have "
            "topology-specific DART runtime evidence."
        ),
    },
    "fixtures/paper-figures/16-unit-tests/vertex-vertex.json": {
        "test": "VertexVertexFixtureRowStaysSeparated",
        "expected_invariant": (
            "DART covers the non-visual Fig. 16 vertex-vertex unit-test alias: "
            "a tetrahedral corner vertex falls under gravity toward a separate "
            "fixed tetrahedral corner vertex above a fixed support plane, "
            "activates rigid IPC contact, stays finite, and reports no "
            "meaningful native overlap after each step."
        ),
        "notes_or_gap": (
            "This paper-unit row aliases the audited 3D vertex-vertex unit-test "
            "fixture, so it is retired by the same DART-owned runtime "
            "coverage. Visual paper-figure rows still require example and "
            "headless visual evidence before retirement."
        ),
    },
    "fixtures/3D/unit-tests/tet-corner.json": {
        "test": "TetCornerFixtureRowStaysSeparated",
        "expected_invariant": (
            "DART covers the audited 3D tet-corner row: a tetrahedral corner "
            "falls under gravity into a fixed three-wall and support-plane "
            "corner, activates rigid IPC contact, stays finite, and reports no "
            "meaningful native overlap after each step."
        ),
        "notes_or_gap": (
            "Covered by DART-owned tetrahedral corner-in-walls runtime "
            "coverage for the 3D unit-test fixture mechanism and its "
            "non-visual Fig. 16 paper-unit alias. Edge-edge rows remain "
            "planned until they have topology-specific DART runtime evidence."
        ),
    },
    "fixtures/paper-figures/16-unit-tests/tet-corner.json": {
        "test": "TetCornerFixtureRowStaysSeparated",
        "expected_invariant": (
            "DART covers the non-visual Fig. 16 tet-corner unit-test alias: a "
            "tetrahedral corner falls under gravity into a fixed three-wall "
            "and support-plane corner, activates rigid IPC contact, stays "
            "finite, and reports no meaningful native overlap after each step."
        ),
        "notes_or_gap": (
            "This paper-unit row aliases the audited 3D tet-corner unit-test "
            "fixture, so it is retired by the same DART-owned runtime "
            "coverage. Visual paper-figure rows still require example and "
            "headless visual evidence before retirement."
        ),
    },
    "fixtures/3D/unit-tests/large-mass-ratio.json": {
        "test": "LargeMassRatioFixtureRowStaysSeparated",
        "expected_invariant": (
            "DART covers the audited 3D large-mass-ratio row: a large heavy "
            "cube falls toward a small cube resting above a fixed mesh plane, "
            "the large-small gap closes into the activation range, and both "
            "contacts preserve nonnegative clearance with finite state."
        ),
        "notes_or_gap": (
            "Covered by DART-owned large-mass-ratio runtime coverage for the "
            "3D unit-test fixture mechanism and its non-visual Fig. 16 "
            "paper-unit alias. Broader stacked and edge-feature 3D unit-test "
            "fixture rows remain planned until they have matching DART "
            "runtime evidence."
        ),
    },
    "fixtures/paper-figures/16-unit-tests/large-mass-ratio.json": {
        "test": "LargeMassRatioFixtureRowStaysSeparated",
        "expected_invariant": (
            "DART covers the non-visual Fig. 16 large-mass-ratio unit-test "
            "alias: a large heavy cube falls toward a small cube resting above "
            "a fixed mesh plane, the large-small gap closes into the "
            "activation range, and both contacts preserve nonnegative "
            "clearance with finite state."
        ),
        "notes_or_gap": (
            "This paper-unit row aliases the audited 3D large-mass-ratio "
            "unit-test fixture, so it is retired by the same DART-owned "
            "runtime coverage. Visual paper-figure rows still require example "
            "and headless visual evidence before retirement."
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
    "tests/ccd/collision_generator.cpp": {
        "artifact": (
            "test_rigid_ipc_fixture::"
            "RigidIpcCcdCase.GeneratedPointEdgeLinearImpactsMatchExpectedToi"
        ),
        "command": (
            "pixi run bash -lc 'build/default/cpp/Release/bin/"
            "test_rigid_ipc_fixture --gtest_color=no --gtest_filter="
            "RigidIpcCcdCase.GeneratedPointEdgeLinearImpactsMatchExpectedToi'"
        ),
        "expected_invariant": (
            "DART synthesizes point-edge rigid CCD rows from expected "
            "time-of-impact and edge-coordinate parameters, then verifies both "
            "the residual equation and interval-subdivision query recover the "
            "generated impact time while preserving a parallel miss row."
        ),
        "notes_or_gap": (
            "Covered by DART-owned deterministic generator-style point-edge "
            "coverage rather than porting the upstream Catch2 random generator; "
            "corpus-scale interval-root parity remains tracked by Phase 2c."
        ),
    },
    "tests/ccd/collision_generator.hpp": {
        "artifact": (
            "test_rigid_ipc_fixture::"
            "RigidIpcCcdCase.GeneratedPointEdgeLinearImpactsMatchExpectedToi"
        ),
        "command": (
            "pixi run bash -lc 'build/default/cpp/Release/bin/"
            "test_rigid_ipc_fixture --gtest_color=no --gtest_filter="
            "RigidIpcCcdCase.GeneratedPointEdgeLinearImpactsMatchExpectedToi'"
        ),
        "expected_invariant": (
            "DART synthesizes point-edge rigid CCD rows from expected "
            "time-of-impact and edge-coordinate parameters, then verifies both "
            "the residual equation and interval-subdivision query recover the "
            "generated impact time while preserving a parallel miss row."
        ),
        "notes_or_gap": (
            "Covered by DART-owned deterministic generator-style point-edge "
            "coverage rather than porting the upstream Catch2 random generator; "
            "corpus-scale interval-root parity remains tracked by Phase 2c."
        ),
    },
    "tests/ccd/test_edge_vertex_ccd.cpp": {
        "artifact": (
            "test_rigid_ipc_fixture::"
            "RigidIpcCcdCase.GeneratedPointEdgeLinearImpactsMatchExpectedToi"
        ),
        "command": (
            "pixi run bash -lc 'build/default/cpp/Release/bin/"
            "test_rigid_ipc_fixture --gtest_color=no --gtest_filter="
            "RigidIpcCcdCase.GeneratedPointEdgeLinearImpactsMatchExpectedToi'"
        ),
        "expected_invariant": (
            "DART covers the audited edge-vertex CCD source row with generated "
            "3D point-edge analogs: midpoint and interior impacts, flipped "
            "edge order, a translating rigid edge, and a parallel miss."
        ),
        "notes_or_gap": (
            "Covered by DART-owned point-edge residual and interval-subdivision "
            "regressions for the upstream edge-vertex responsibility; rigorous "
            "interval arithmetic remains tracked by Phase 2c."
        ),
    },
    "tests/ccd/test_hash_grid.cpp": {
        "artifact": (
            "test_spatial_hash::"
            "SpatialHashBroadPhase.SweptPrimitiveCandidatePairsMatchBruteForce"
        ),
        "command": (
            "pixi run bash -lc 'build/default/cpp/Release/bin/"
            "test_spatial_hash --gtest_color=no --gtest_filter="
            "SpatialHashBroadPhase.SweptPrimitiveCandidatePairsMatchBruteForce'"
        ),
        "expected_invariant": (
            "DART covers the audited generic hash-grid source row by comparing "
            "native spatial-hash broad-phase candidate pairs against brute "
            "force for swept point, edge, and triangle AABBs."
        ),
        "notes_or_gap": (
            "Covered by DART-owned swept-primitive broad-phase parity coverage; "
            "full upstream impact-filter parity remains tracked by the direct "
            "CCD and runtime fixture rows."
        ),
    },
    "tests/ccd/test_time_of_impact.cpp": {
        "artifact": (
            "test_rigid_ipc_fixture::"
            "RigidIpcCcdCase.GeneratedPointEdgeLinearImpactsMatchExpectedToi; "
            "test_rigid_ipc_fixture::"
            "RigidIpcCcdCase.EvaluatesUpstreamStyleRigidToiRows"
        ),
        "command": (
            "pixi run bash -lc 'build/default/cpp/Release/bin/"
            "test_rigid_ipc_fixture --gtest_color=no --gtest_filter="
            '"RigidIpcCcdCase.GeneratedPointEdgeLinearImpactsMatchExpectedToi:'
            "RigidIpcCcdCase.EvaluatesUpstreamStyleRigidToiRows\"'"
        ),
        "expected_invariant": (
            "DART covers the audited generic TOI source row with generated "
            "point-edge impacts plus upstream-style point-edge, edge-edge, and "
            "face-vertex rigid TOI rows."
        ),
        "notes_or_gap": (
            "Covered by DART-owned rigid CCD load, residual, and "
            "interval-subdivision regressions; corpus-scale interval-root "
            "parity remains tracked by Phase 2c."
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
