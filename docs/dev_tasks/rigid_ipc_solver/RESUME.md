# Resume: Rigid IPC Solver

## Current Reality (2026-06-06)

Use this folder's `README.md`, `docs/plans/dashboard.md`, and the current code as
the live status. The long session log below is historical evidence for the rigid
IPC corpus/parity effort; branch names, `/home/js/...` checkout paths, pushed
feature-branch notes, and "No push or PR mutation" statements belong to those
past slices. Current rigid IPC work should keep using the DART-owned
`RigidBodySolver::Ipc` method-family opt-in, the shared built-in World step
schedule, and the open README/dashboard next steps: robust normal-push for
kinematic obstacles, the performance climb, remaining corpus/parity coverage,
and articulated-scene support without exposing solver registries, ECS storage,
external project names, or backend resources in public API.

## Session 2026-07-04: two-wall tunnel fixture row (+ 3/4-wall/8K deferral)

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for the audited two-wall tunnel unit-test
  fixture row (`fixtures/3D/unit-tests/tunnel/2-walls.json`) in
  `RigidIpcPaperExperiments.TwoWallTunnelCubeStaysBetweenWallsFixtureRow`: a unit
  cube flies down a tight (1 mm) fixed floor/ceiling corridor at high speed,
  activates rigid IPC contact, stays finite, keeps advancing along the tunnel
  axis (asserted with a real `startX + 0.1` margin), and stays within the fixed
  wall clearances (tracked from the cube corners so the invariant is not polluted
  by native static wall-vs-wall overlaps from the support geometry).
- Marked that one upstream 3D unit-test fixture row implemented in the generated
  manifest (planned 339 -> 338, implemented 459 -> 460).
- Deferred the three-wall, four-wall, and 8K tunnel rows with evidence. A pre-PR
  adversarial review flagged that a bare `> startX` progress assertion could pass
  a frozen cube; adding a `startX + 0.1` margin then EXPOSED that at the faithful
  0.1 mm clearance of `3-walls.json`/`4-walls.json` the conservative curved-CCD
  line search limits each step to about the separation distance, so the cube is
  arrested (intersection-free but only ~3 mm / ~56 mm of travel) rather than
  traversing. Loosening the gap would be an overclaim (their fixtures' defining
  feature is the 0.1 mm tolerance), so those rows stay planned as a
  conservative-CCD/performance gap. The 8K variant additionally holds active
  contact against two 8192-triangle walls every step (~minutes/step), too slow
  for a unit test -- tied to the dense-contact performance work in
  `benchmarks.md`.

Environment note (this checkout): plain `pixi run ninja ...` compiles but fails
at link with `undefined reference to __libc_dlopen_mode@GLIBC_PRIVATE` because
the conda `ld` shadows the CMake-configured system `/usr/bin/ld`. Build with
`pixi run bash -c 'PATH=/usr/bin:$PATH ninja -C build/default/cpp/Release <target>'`.

Validation in this slice:

- `pixi run bash -c 'PATH=/usr/bin:$PATH ninja -C build/default/cpp/Release test_rigid_ipc_paper_experiments'`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter='RigidIpcPaperExperiments.TwoWallTunnelCubeStaysBetweenWallsFixtureRow'`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`

No push or PR mutation has been made from this slice.

## Session 2026-06-01: generic hash-grid source row

Delivered a bounded Phase 2/6 algorithm-manifest slice:

- Added DART-owned swept primitive broad-phase coverage in
  `SpatialHashBroadPhase.SweptPrimitiveCandidatePairsMatchBruteForce`: swept
  point, edge, and triangle AABBs produce the same candidate pairs with native
  spatial hash as with brute force, including expected point-edge and
  edge-triangle overlaps.
- Marked the remaining audited CCD/barrier test-source row,
  `tests/ccd/test_hash_grid.cpp`, implemented in the generated manifest. All
  audited upstream test-source rows in the current manifest are now
  implemented.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_spatial_hash`
- `./build/default/cpp/Release/bin/test_spatial_hash --gtest_color=no --gtest_filter=SpatialHashBroadPhase.SweptPrimitiveCandidatePairsMatchBruteForce`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-06-01: generated point-edge TOI source rows

Delivered a bounded Phase 2/6 algorithm-manifest slice:

- Added deterministic generated point-edge TOI coverage in
  `RigidIpcCcdCase.GeneratedPointEdgeLinearImpactsMatchExpectedToi`: generated
  midpoint/interior/flipped-edge impacts are synthesized from expected
  time-of-impact and edge-coordinate parameters, verified against the residual
  equations, and recovered by the interval-subdivision query; a parallel miss
  row remains non-colliding.
- Marked the audited collision-generator, edge-vertex CCD, and generic TOI
  source rows implemented in the generated manifest:
  `tests/ccd/collision_generator.cpp`,
  `tests/ccd/collision_generator.hpp`,
  `tests/ccd/test_edge_vertex_ccd.cpp`, and
  `tests/ccd/test_time_of_impact.cpp`. The generic hash-grid source row remains
  planned.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_fixture`
- `./build/default/cpp/Release/bin/test_rigid_ipc_fixture --gtest_color=no --gtest_filter=RigidIpcCcdCase.GeneratedPointEdgeLinearImpactsMatchExpectedToi`
- `./build/default/cpp/Release/bin/test_rigid_ipc_fixture --gtest_color=no --gtest_filter="RigidIpcCcdCase.GeneratedPointEdgeLinearImpactsMatchExpectedToi:RigidIpcCcdCase.EvaluatesUpstreamStyleRigidToiRows"`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-06-01: card-tent fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for
  `fixtures/3D/friction/card-house/card-tent.json`: two inclined card bodies
  stand on a fixed frictional support, activate rigid IPC contact, stay finite,
  preserve upright height, and report no meaningful native overlap in
  `RigidIpcPaperExperiments.CardTentFixtureRowStaysUprightWithFriction`.
- Marked that upstream card-tent fixture row implemented in the generated
  manifest. Larger card-house rows remain planned until they have matching DART
  evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.CardTentFixtureRowStaysUprightWithFriction`

No push or PR mutation has been made from this slice.

## Session 2026-06-01: 25-stone arch fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for
  `fixtures/3D/friction/arch/arch-25-stones.json`: a generated 25-voussoir
  frictional arch stands on a fixed support, activates rigid IPC contact, stays
  finite, preserves support clearance, and keeps the keystone from collapsing in
  `RigidIpcPaperExperiments.TwentyFiveVoussoirFrictionArchFixtureRowStands`.
- Marked that upstream 25-stone arch fixture row implemented in the generated
  manifest. Larger arch rows and the Fig. 11 visual alias remain planned until
  they have matching DART evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.TwentyFiveVoussoirFrictionArchFixtureRowStands`

No push or PR mutation has been made from this slice.

## Session 2026-06-01: rolling-cone friction fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for
  `fixtures/3D/friction/rolling/cone.json`: a tilted cone advances over a fixed
  frictional plane, activates rigid IPC contact, stays finite, preserves
  clearance, and develops angular velocity in
  `RigidIpcPaperExperiments.RollingConeFixtureRowAdvancesWithContact`.
- Marked that upstream rolling-cone friction fixture row implemented in the
  generated manifest. The oloid rolling fixture row remains planned until it has
  matching DART evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.RollingConeFixtureRowAdvancesWithContact`

No push or PR mutation has been made from this slice.

## Session 2026-06-01: 8K tessellated-plane fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for
  `fixtures/3D/unit-tests/tessellated-plane/8K-triangles.json`: a cube starts
  near a fixed generated 8192-triangle mesh plane, activates rigid IPC contact,
  stays finite, and preserves nonnegative clearance in
  `RigidIpcPaperExperiments.CubeContactsEightKTrianglePlaneFixtureRow`.
- Marked that upstream 8K tessellated-plane unit-test fixture row implemented
  in the generated manifest. The earlier two-triangle tessellated-plane rows now
  point at their own runtime coverage while noting that the 8K topology has
  separate evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.CubeContactsEightKTrianglePlaneFixtureRow`

No push or PR mutation has been made from this slice.

## Session 2026-06-01: Erleben spike-in-hole fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for
  `fixtures/3D/unit-tests/erleben/spike-in-hole.json`: an inverted spike
  advances through a fixed hole mesh under the upstream velocity direction,
  activates rigid IPC contact, stays finite, and reports no meaningful native
  overlap after each step in
  `RigidIpcPaperExperiments.ErlebenSpikeInHoleFixtureRowStaysSeparated`.
- Marked that upstream Erleben unit-test fixture row implemented in the
  generated manifest. This retires the audited Erleben unit-test fixture rows
  tracked in the current P0 manifest slice.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.ErlebenSpikeInHoleFixtureRowStaysSeparated`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-06-01: Erleben wedge-in-crack fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for
  `fixtures/3D/unit-tests/erleben/wedge-in-crack.json`: an inverted wedge
  advances through a fixed crack mesh under the upstream velocity and force
  direction, activates rigid IPC contact, stays finite, and reports no
  meaningful native overlap after each step in
  `RigidIpcPaperExperiments.ErlebenWedgeInCrackFixtureRowStaysSeparated`.
- Marked that upstream Erleben unit-test fixture row implemented in the
  generated manifest. The hole row remains planned until it has
  topology-specific DART runtime evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.ErlebenWedgeInCrackFixtureRowStaysSeparated`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-06-01: Erleben spike-in-crack fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for
  `fixtures/3D/unit-tests/erleben/spike-in-crack.json`: an inverted spike
  advances through a fixed crack mesh under the upstream velocity and force
  direction, activates rigid IPC contact, stays finite, and reports no
  meaningful native overlap after each step in
  `RigidIpcPaperExperiments.ErlebenSpikeInCrackFixtureRowStaysSeparated`.
- Marked that upstream Erleben unit-test fixture row implemented in the
  generated manifest. The hole row remains planned until it has
  topology-specific DART runtime evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.ErlebenSpikeInCrackFixtureRowStaysSeparated`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-06-01: Erleben spike-and-wedge fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for
  `fixtures/3D/unit-tests/erleben/spike-and-wedge.json`: an inverted spike
  rests on a fixed upright wedge, activates rigid IPC contact, stays finite,
  and reports no meaningful native overlap after each step in
  `RigidIpcPaperExperiments.ErlebenSpikeAndWedgeFixtureRowStaysSeparated`.
- Marked that upstream Erleben unit-test fixture row implemented in the
  generated manifest. Crack and hole Erleben rows remain planned until they
  have topology-specific DART runtime evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.ErlebenSpikeAndWedgeFixtureRowStaysSeparated`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-06-01: Erleben wedges fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for
  `fixtures/3D/unit-tests/erleben/wedges.json`: an inverted wedge rests on a
  fixed upright wedge, activates rigid IPC contact, stays finite, and reports
  no meaningful native overlap after each step in
  `RigidIpcPaperExperiments.ErlebenWedgesFixtureRowStaysSeparated`.
- Marked that upstream Erleben unit-test fixture row implemented in the
  generated manifest. Other Erleben rows remain planned until they have
  topology-specific DART runtime evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.ErlebenWedgesFixtureRowStaysSeparated`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-06-01: Erleben spikes fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for
  `fixtures/3D/unit-tests/erleben/spikes.json`: an inverted spike rests on a
  fixed upright spike, activates rigid IPC contact, stays finite, and reports
  no meaningful native overlap after each step in
  `RigidIpcPaperExperiments.ErlebenSpikesFixtureRowStaysSeparated`.
- Marked that upstream Erleben unit-test fixture row implemented in the
  generated manifest. Other Erleben rows remain planned until they have
  topology-specific DART runtime evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.ErlebenSpikesFixtureRowStaysSeparated`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-06-01: Erleben sliding-wedge fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for
  `fixtures/3D/unit-tests/erleben/sliding-wedge.json`: an inverted wedge slides
  across a fixed plane, activates rigid IPC contact, stays finite, advances
  laterally, and reports no meaningful native overlap after each step in
  `RigidIpcPaperExperiments.ErlebenSlidingWedgeFixtureRowStaysSeparated`.
- Marked that upstream Erleben unit-test fixture row implemented in the
  generated manifest. Other Erleben rows remain planned until they have
  topology-specific DART runtime evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.ErlebenSlidingWedgeFixtureRowStaysSeparated`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-06-01: Erleben sliding-spike fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for
  `fixtures/3D/unit-tests/erleben/sliding-spike.json`: an inverted spike slides
  across a fixed plane, activates rigid IPC contact, stays finite, advances
  laterally, and reports no meaningful native overlap after each step in
  `RigidIpcPaperExperiments.ErlebenSlidingSpikeFixtureRowStaysSeparated`.
- Marked that upstream Erleben unit-test fixture row implemented in the
  generated manifest. Other Erleben rows remain planned until they have
  topology-specific DART runtime evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.ErlebenSlidingSpikeFixtureRowStaysSeparated`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-06-01: Erleben internal-edges fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for
  `fixtures/3D/unit-tests/erleben/internal-edges.json`: a cube falls under
  gravity onto a fixed internal-edge mesh, activates rigid IPC contact, stays
  finite, and reports no meaningful native overlap after each step in
  `RigidIpcPaperExperiments.ErlebenInternalEdgesFixtureRowStaysSeparated`.
- Marked that upstream Erleben unit-test fixture row implemented in the
  generated manifest. Other Erleben rows remain planned until they have
  topology-specific DART runtime evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.ErlebenInternalEdgesFixtureRowStaysSeparated`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-06-01: Erleben cliff-edges fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for
  `fixtures/3D/unit-tests/erleben/cliff-edges.json`: a cube falls under gravity
  onto a fixed cliff-edge mesh, activates rigid IPC contact, stays finite, and
  reports no meaningful native overlap after each step in
  `RigidIpcPaperExperiments.ErlebenCliffEdgesFixtureRowStaysSeparated`.
- Marked that upstream Erleben unit-test fixture row implemented in the
  generated manifest. Other Erleben rows remain planned until they have
  topology-specific DART runtime evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.ErlebenCliffEdgesFixtureRowStaysSeparated`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-06-01: two-triangle tet fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for
  `fixtures/3D/unit-tests/tessellated-plane/two-triangles-tet.json`: a
  tetrahedral corner falls onto a fixed two-triangle mesh plane, activates rigid
  IPC contact, stays finite, and reports no meaningful native overlap after each
  step in
  `RigidIpcPaperExperiments.TetCornerFallsOnTwoTrianglePlaneFixtureRow`.
- Marked that upstream 3D unit-test fixture row implemented in the generated
  manifest. The 8K tessellated-plane row remains planned until it has
  topology-specific DART runtime evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.TetCornerFallsOnTwoTrianglePlaneFixtureRow`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-06-01: tet-corner fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for
  `fixtures/3D/unit-tests/tet-corner.json`: a tetrahedral corner falls under
  gravity into a fixed three-wall and support-plane corner, activates rigid IPC
  contact, stays finite, and reports no meaningful native overlap after each
  step in `RigidIpcPaperExperiments.TetCornerFixtureRowStaysSeparated`.
- Marked that upstream 3D unit-test fixture row and its non-visual Fig. 16
  paper-unit alias implemented in the generated manifest. Edge-edge rows remain
  planned until they have topology-specific DART runtime evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.TetCornerFixtureRowStaysSeparated`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-06-01: vertex-vertex fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for
  `fixtures/3D/unit-tests/vertex-vertex.json`: a tetrahedral corner vertex
  falls under gravity toward a separate fixed tetrahedral corner vertex above a
  fixed support plane, activates rigid IPC contact, stays finite, and reports no
  meaningful native overlap after each step in
  `RigidIpcPaperExperiments.VertexVertexFixtureRowStaysSeparated`.
- Marked that upstream 3D unit-test fixture row and its non-visual Fig. 16
  paper-unit alias implemented in the generated manifest. Edge-edge and
  tet-corner rows remain planned until they have topology-specific DART runtime
  evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.VertexVertexFixtureRowStaysSeparated`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-06-01: vertex-face fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for
  `fixtures/3D/unit-tests/vertex-face.json`: a tetrahedral pyramid vertex falls
  under gravity toward a separate fixed tetrahedral pyramid face, activates
  rigid IPC contact, stays finite, and reports no meaningful native overlap
  after each step in
  `RigidIpcPaperExperiments.VertexFaceFixtureRowStaysSeparated`.
- Marked that upstream 3D unit-test fixture row and its non-visual Fig. 16
  paper-unit alias implemented in the generated manifest. Vertex-vertex,
  edge-edge, and tet-corner rows remain planned until they have
  topology-specific DART runtime evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.VertexFaceFixtureRowStaysSeparated`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-06-01: face-vertex fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for
  `fixtures/3D/unit-tests/face-vertex.json`: a tetrahedral pyramid face falls
  under gravity toward a separate fixed tetrahedral pyramid vertex, activates
  rigid IPC contact, stays finite, and reports no meaningful native overlap
  after each step in
  `RigidIpcPaperExperiments.FaceVertexFixtureRowStaysSeparated`.
- Marked that upstream 3D unit-test fixture row and its non-visual Fig. 16
  paper-unit alias implemented in the generated manifest. Vertex-face,
  vertex-vertex, edge-edge, and tet-corner rows remain planned until they have
  topology-specific DART runtime evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.FaceVertexFixtureRowStaysSeparated`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: cube-falling-on-edge fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for
  `fixtures/3D/unit-tests/cube-falling-on-edge.json`: a tilted cube falls under
  gravity onto a separate tilted fixed box edge, activates rigid IPC contact,
  stays finite, and reports no meaningful native overlap after each step in
  `RigidIpcPaperExperiments.CubeFallingOnEdgeFixtureRowStaysSeparated`.
- Marked that upstream 3D unit-test fixture row implemented in the generated
  manifest. Edge-edge, vertex/face, and tet-corner rows remain planned until
  they have topology-specific DART runtime evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.CubeFallingOnEdgeFixtureRowStaysSeparated`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: five-cubes fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for
  `fixtures/3D/unit-tests/5-cubes.json`: five aligned cubes fall under gravity
  onto a fixed support, activate stacked rigid IPC contacts, stay finite, and
  preserve nonnegative support and cube-cube clearance in
  `RigidIpcPaperExperiments.FiveCubesFixtureRowStacksWithoutPenetration`.
- Marked that upstream 3D unit-test fixture row and its non-visual Fig. 16
  paper-unit alias implemented in the generated manifest. Edge-feature and
  tessellated stress rows remain planned until they have matching DART runtime
  evidence. A stricter exploratory variant using a two-triangle support did not
  satisfy clearance, so that evidence was not used.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.FiveCubesFixtureRowStacksWithoutPenetration`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: large-mass-ratio fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for
  `fixtures/3D/unit-tests/large-mass-ratio.json`: a large heavy cube falls
  toward a small cube resting above a fixed mesh plane, the large-small gap
  closes into the activation range, and both contacts preserve nonnegative
  clearance with finite state in
  `RigidIpcPaperExperiments.LargeMassRatioFixtureRowStaysSeparated`.
- Marked that upstream 3D unit-test fixture row and its non-visual Fig. 16
  paper-unit alias implemented in the generated manifest. Broader stacked and
  edge-feature 3D unit-test fixture rows remain planned until they have matching
  DART runtime evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.LargeMassRatioFixtureRowStaysSeparated`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: two-triangle plane fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned runtime coverage for
  `fixtures/3D/unit-tests/tessellated-plane/two-triangles.json`: a cube falls
  onto a fixed two-triangle mesh plane, activates rigid IPC contact, stays
  finite, and preserves nonnegative clearance in
  `RigidIpcPaperExperiments.CubeSettlesOnTwoTrianglePlaneFixtureRow`.
- Marked that upstream 3D unit-test fixture row implemented in the generated
  manifest. The two-triangle tet row and 8K tessellated-plane row remain
  planned until they have matching topology-specific DART runtime evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.CubeSettlesOnTwoTrianglePlaneFixtureRow`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: Dzhanibekov wing-nut fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned no-contact runtime coverage for
  `fixtures/3D/unit-tests/rotation/dzhanibekov.json`: a wing-nut-like rigid
  mesh with zero gravity, an initial tilt, and high angular velocity remains
  finite, does not translate, and advances orientation in
  `RigidIpcPaperExperiments.DzhanibekovWingNutFixtureRowAdvancesSafely`.
- Marked that upstream 3D unit-test fixture row implemented in the generated
  manifest. Contacting edge/vertex unit-test rows remain planned until they
  have matching DART runtime evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.DzhanibekovWingNutFixtureRowAdvancesSafely`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: spinning-cube-over-plane fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned no-contact runtime coverage for
  `fixtures/3D/unit-tests/spinning-cube-over-plane.json`: a cube with zero
  gravity and high angular velocity spins above a fixed near plane, remains
  finite, and preserves nonnegative clearance in
  `RigidIpcPaperExperiments.SpinningCubeOverPlaneFixtureRowAdvancesSafely`.
- Marked that upstream 3D unit-test fixture row implemented in the generated
  manifest. Contacting edge/vertex unit-test rows remain planned until they
  have matching DART runtime evidence; an exploratory edge-edge runtime probe
  failed locally and was not retired.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.SpinningCubeOverPlaneFixtureRowAdvancesSafely`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: torque rotation fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned no-contact runtime coverage for
  `fixtures/3D/unit-tests/rotation/torque-test.json`: a disk-like free body
  with zero gravity and applied torque remains finite, does not translate, and
  gains angular velocity about the torque axis in
  `RigidIpcPaperExperiments.TorqueFixtureRowAcceleratesFreeBody`.
- Marked that upstream 3D unit-test fixture row implemented in the generated
  manifest. The Dzhanibekov wing-nut row remains planned until it has
  geometry-specific runtime evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.TorqueFixtureRowAcceleratesFreeBody`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: scaled-sphere and ellipsoid rotation rows

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned no-contact runtime coverage for the audited scaled-sphere /
  ellipsoid rotation rows:
  `fixtures/3D/unit-tests/rotation/rotating-sphere.json`,
  `fixtures/3D/unit-tests/rotation/rotating-ellipsoid-major.json`,
  `fixtures/3D/unit-tests/rotation/rotating-ellipsoid-intermediate.json`, and
  `fixtures/3D/unit-tests/rotation/rotating-ellipsoid-minor.json`.
- Each row uses a scaled ellipsoid mesh with zero gravity and row-specific
  angular velocity, then verifies the opt-in rigid IPC runtime stage advances
  orientation, keeps finite state, and does not translate.
- Marked those four upstream 3D unit-test fixture rows implemented in the
  generated manifest. Dzhanibekov and torque rotation rows remain planned until
  they have matching geometry/force evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter='RigidIpcPaperExperiments.RotatingScaledSphereFixtureRowAdvancesWithoutContact:RigidIpcPaperExperiments.RotatingEllipsoidMajorFixtureRowAdvancesWithoutContact:RigidIpcPaperExperiments.RotatingEllipsoidIntermediateFixtureRowAdvancesWithoutContact:RigidIpcPaperExperiments.RotatingEllipsoidMinorFixtureRowAdvancesWithoutContact'`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: rotating-cube fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned coverage for
  `fixtures/3D/unit-tests/rotation/rotating-cube.json`: a free cube with zero
  gravity and angular velocity advances orientation through the opt-in rigid IPC
  runtime stage, stays finite, and does not translate in
  `RigidIpcPaperExperiments.RotatingCubeFixtureRowAdvancesWithoutContact`.
- Marked that upstream 3D unit-test fixture row implemented in the generated
  manifest.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.RotatingCubeFixtureRowAdvancesWithoutContact`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: high-speed tunneling fixture row

Delivered a bounded Phase 3/6 runtime-manifest slice:

- Added DART-owned coverage for `fixtures/3D/unit-tests/tunneling.json`: a
  rotated cube with a large time-step velocity toward a fixed wall remains
  intersection-free, and the opt-in rigid IPC runtime stage reports a
  conservative CCD line-search hit in
  `RigidIpcPaperExperiments.HighSpeedCubeDoesNotTunnelThroughWall`.
- Generalized the fixture manifest's implemented-row map so non-friction
  fixture rows can be retired with the same explicit test evidence path.
- Marked that upstream 3D unit-test fixture row implemented in the generated
  manifest.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.HighSpeedCubeDoesNotTunnelThroughWall`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: low-friction turntable fixture row

Delivered a bounded Phase 4/6 friction-manifest slice:

- Added DART-owned coverage for
  `fixtures/3D/friction/turntable/turntable-mu=0.1.json`: a cube resting on a
  rotating kinematic cylinder remains intersection-free while `mu=0.1` contact
  friction carries it tangentially in
  `RigidIpcPaperExperiments.TurntableLowFrictionFixtureRowCarriesRider`.
- Marked that upstream 3D friction fixture row implemented in the generated
  manifest.
- Kept `mu=0.0` and the Fig. 13 paper visual aliases planned until they have
  matching no-friction evidence and headless visual/example coverage.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.TurntableLowFrictionFixtureRowCarriesRider`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: moderate-friction turntable fixture row

Delivered a bounded Phase 4/6 friction-manifest slice:

- Added DART-owned coverage for
  `fixtures/3D/friction/turntable/turntable-mu=0.5.json`: a cube resting on a
  rotating kinematic cylinder remains intersection-free while `mu=0.5` contact
  friction carries it tangentially in
  `RigidIpcPaperExperiments.TurntableModerateFrictionFixtureRowCarriesRider`.
- Marked that upstream 3D friction fixture row implemented in the generated
  manifest.
- Kept `mu=0.0`, `mu=0.1`, and the Fig. 13 paper visual aliases planned until
  they have matching coefficient-specific evidence and headless visual/example
  coverage.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.TurntableModerateFrictionFixtureRowCarriesRider`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: high-friction turntable fixture row

Delivered a bounded Phase 4/6 friction-manifest slice:

- Added DART-owned coverage for
  `fixtures/3D/friction/turntable/turntable-mu=1.0.json`: a cube resting on a
  rotating kinematic cylinder remains intersection-free while `mu=1.0` contact
  friction carries it tangentially in
  `RigidIpcPaperExperiments.TurntableHighFrictionFixtureRowCarriesRider`.
- Marked that upstream 3D friction fixture row implemented in the generated
  manifest.
- Kept the lower-friction turntable fixture rows and Fig. 13 paper visual
  aliases planned. A direct `mu=0.0` check still showed small tangential drift,
  and the paper aliases still require DART examples plus headless visual
  evidence.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.TurntableHighFrictionFixtureRowCarriesRider`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: spolling coin friction fixture row

Delivered a bounded Phase 4/6 friction-manifest slice:

- Marked `fixtures/3D/friction/spolling-coin.json` implemented in the generated
  manifest. The owning DART coverage is
  `RigidIpcPaperExperiments.SpinningCoinIsBrakedByFrictionWithoutPenetration`:
  a spinning disk remains intersection-free on a frictional support while
  contact friction dissipates angular velocity.
- Kept the Fig. 7 paper visual alias
  `fixtures/paper-figures/07-spolling-coint.json` planned. It still needs a
  DART example plus headless visual evidence before retirement.

Validation in this slice:

- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.SpinningCoinIsBrakedByFrictionWithoutPenetration`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: sliding friction fixture row

Delivered a bounded Phase 4/6 friction-manifest slice:

- Added exact DART-owned coverage for `fixtures/3D/friction/sliding.json`:
  a cube with initial tangential velocity and `mu=0.05` remains
  intersection-free and is observably braked relative to a frictionless run in
  `RigidIpcPaperExperiments.SlidingCubeFixtureRowIsBrakedByFriction`.
- Marked that upstream 3D friction fixture row implemented in the generated
  manifest.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.SlidingCubeFixtureRowIsBrakedByFriction`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: rigid-body hash-grid source row

Delivered a bounded Phase 6 algorithm-source manifest slice:

- Marked the audited source row `tests/ccd/test_rigid_body_hash_grid.cpp`
  implemented in the generated manifest. Its concrete upstream coverage is the
  large rigid-body hash-grid scene-bounds experiment, which DART now owns as
  `BM_RigidIpcLargeHashgridSceneBounds`.
- Kept the generic `tests/ccd/test_hash_grid.cpp` row planned. DART still needs
  brute-force-vs-culled broad-phase parity coverage before that row should be
  retired.

Validation in this slice:

- `pixi run bm --target bm_rigid_ipc_solver --build-type Release -- --benchmark_filter=BM_RigidIpcLargeHashgridSceneBounds --benchmark_min_time=0.001s --benchmark_repetitions=1 --benchmark_out=/tmp/rigid_ipc_large_hashgrid_source_row.json --benchmark_out_format=json`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: high-friction incline fixture row

Delivered a bounded Phase 4/6 friction-manifest slice:

- Added exact DART-owned coverage for the audited high-friction incline row:
  `mu=1.0` against `tan(theta)=0.5` remains intersection-free and settles
  without sustained down-slope slide in
  `RigidIpcPaperExperiments.FrictionThresholdHighFixtureRowSticks`.
- Marked
  `fixtures/3D/friction/incline-plane/slopeTest_highSchoolPhysics_mu=1.json`
  implemented in the generated manifest.
- Kept the `mu=0.5` at-threshold rows planned. They still need matching DART
  stick evidence before retirement.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.FrictionThresholdHighFixtureRowSticks`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: below-threshold friction fixture row

Delivered a bounded Phase 4/6 friction-manifest slice:

- Added exact DART-owned Fig. 18 coverage for the audited below-threshold
  high-school-physics row: `mu=0.49` against `tan(theta)=0.5` remains
  intersection-free and keeps sliding down-slope in
  `RigidIpcPaperExperiments.FrictionThresholdBelowFixtureRowSlides`.
- Marked the upstream 3D friction fixture row
  `fixtures/3D/friction/incline-plane/slopeTest_highSchoolPhysics_mu=0.49.json`
  and its Fig. 18 paper alias implemented in the generated manifest.
- Kept the `mu=0.5` at-threshold rows planned. Direct verification showed the
  current runtime does not yet provide stick evidence for that exact row, so it
  should not be retired until the solver behavior or acceptance criterion is
  tightened.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_paper_experiments`
- `./build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no --gtest_filter=RigidIpcPaperExperiments.FrictionThresholdBelowFixtureRowSlides`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: barrier and rigid TOI source rows

Delivered a bounded Phase 6 algorithm-source audit slice:

- Marked the upstream barrier derivative source row
  `tests/barrier/test_barriers.cpp` as `implemented` in the generated manifest.
  The owning DART coverage is `IpcBarrierKernel.*` plus
  `RigidIpcBarrier.*`/`RigidIpcCcdGeometry.*`.
- Marked the upstream rigid-body time-of-impact source row
  `tests/ccd/test_rigid_body_time_of_impact.cpp` as `implemented` in the
  generated manifest. The owning DART coverage is `RigidIpcCcdCase.*`, including
  parser, residual, interval-subdivision, rotational-trajectory, kinematic, and
  conservative-TOI corpus regressions.
- Kept the remaining six `test-source` rows planned: collision-generator helper
  files, generic edge-vertex/generic TOI tests, generic hash-grid tests, and
  rigid-body hash-grid source coverage still need tighter DART matches before
  retirement.

Validation in this slice:

- `./build/default/cpp/Release/bin/test_barrier_kernel --gtest_color=no`
- `./build/default/cpp/Release/bin/test_rigid_ipc_barrier --gtest_color=no --gtest_filter='RigidIpcBarrier.*:RigidIpcCcdGeometry.*'`
- `./build/default/cpp/Release/bin/test_rigid_ipc_fixture --gtest_color=no --gtest_filter='RigidIpcCcdCase.*'`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: large hash-grid benchmark evidence

Delivered a bounded Phase 6 benchmark-evidence slice:

- Added `tests/fixtures/rigid_ipc/large_hashgrid_bounds.tsv`, a compact
  audited fixture for the remaining
  `tests/data/large-rb-hashgrid/large-rb-hashgrid-000..001.json` rows. It
  records upstream source hashes, body/source primitive counts, per-body
  pose/local-bound records, and upstream exact scene bounds without vendoring
  the full upstream JSON data.
- Added
  `BM_RigidIpcLargeHashgridSceneBounds/large_rb_hashgrid_{000,001}` to
  `bm_rigid_ipc_solver`. The benchmark computes conservative swept scene
  bounds from the compact per-body records, verifies those bounds contain the
  upstream exact scene bounds, and emits profile counters.
- Updated the rigid IPC manifest generator so all 405 `test-data` rows are now
  emitted as `implemented`. Broader Phase 6 remains open on fixture, algorithm,
  benchmark-script, comparison, and visual-evidence rows.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target bm_rigid_ipc_solver`
- `pixi run bm --target bm_rigid_ipc_solver --build-type Release -- --benchmark_filter=BM_RigidIpcLargeHashgridSceneBounds --benchmark_min_time=0.001s --benchmark_repetitions=1 --benchmark_out=/tmp/rigid_ipc_large_hashgrid_benchmark_pixi.json --benchmark_out_format=json`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: large hash-grid manifest semantics

Delivered a bounded Phase 6 manifest cleanup slice:

- Reclassified the remaining
  `tests/data/large-rb-hashgrid/large-rb-hashgrid-000..001.json` rows as
  planned broad-phase benchmark rows instead of direct CCD TOI rows.
- Recorded the upstream provenance gap: the audited rigid-ipc hash-grid source
  keeps the large-scene benchmark commented out, so DART should not retire
  those rows until it has matching scene-bounds coverage and reproducible
  profile evidence.
- Kept Phase 6 open: the two large-rigid-body hash-grid data rows remain
  planned, now with accurate benchmark-target evidence requirements.

Validation in this slice:

- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: audited kinematic CCD manifest retirement

Delivered a bounded Phase 6 manifest slice:

- Extended the audited kinematic zero-time-hit regression from
  `tests/data/kinematic/ccd-test-000..002.json` through
  `tests/data/kinematic/ccd-test-012.json`.
- Updated the rigid IPC fixture manifest generator so all audited kinematic
  direct-CCD rows are emitted as `implemented`.
- Added a checked-in wrecking-ball direct-CCD corpus fixture and
  conservative-TOI regression for
  `tests/data/wrecking-ball/ccd-test-000..385.json`: if DART reports a TOI,
  replaying only through that bound does not report another hit.
- Updated the rigid IPC fixture manifest generator so all tracked wrecking-ball
  direct-CCD rows are emitted as `implemented`.
- Kept Phase 6 open: the remaining manifest corpus and broader CCD data rows
  still need matching DART tests, examples, benchmarks, or evidence before row
  retirement.

Validation in this slice:

- `cmake --build build/default/cpp/Release --target test_rigid_ipc_fixture && ./build/default/cpp/Release/bin/test_rigid_ipc_fixture --gtest_filter=RigidIpcCcdCase.EvaluatesAuditedKinematicRowsWithoutZeroTimeHits --gtest_break_on_failure`
- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `cmake --build build/default/cpp/Release --target test_rigid_ipc_fixture && ./build/default/cpp/Release/bin/test_rigid_ipc_fixture --gtest_filter=RigidIpcCcdCase.EvaluatesAuditedWreckingBallCorpusConservatively --gtest_break_on_failure`
- `cmake --build build/default/cpp/Release --target test_rigid_ipc_fixture && ./build/default/cpp/Release/bin/test_rigid_ipc_fixture --gtest_filter=RigidIpcCcdCase.EvaluatesFirstAuditedWreckingBallRowsConservatively --gtest_break_on_failure`
- `pixi run lint`
- `pixi run build`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: implemented manifest status for first kinematic CCD rows

Delivered a bounded Phase 6 manifest slice:

- Updated the rigid IPC fixture manifest generator so
  `tests/data/kinematic/ccd-test-000..002.json` are emitted as `implemented`
  rows with the owning zero-time-hit DART regression and command recorded in the
  row metadata.
- Added generator unit coverage that proves the first audited kinematic rows are
  promoted while the next uncovered kinematic row remains planned.
- Kept Phase 6 open: the rest of the kinematic corpus and broader CCD data rows
  still need matching DART evidence before row retirement.

Validation in this slice:

- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: kinematic CCD rows avoid zero-time hits

Delivered a bounded Phase 2c CCD parity slice:

- Fixed the internal interval-root CCD result reporting so a terminal root box
  whose lower time edge is `0` only reports `t=0` when the primitives are
  actually touching at the start pose. Separated starts now receive a positive
  conservative lower bound instead of a zero-time hit.
- Added first audited kinematic upstream row coverage for
  `tests/data/kinematic/ccd-test-000..002.json`, proving the current DART
  evaluator preserves the upstream guard against zero-time hits for separated
  rows.
- Kept Phase 2c open: this does not yet claim rigorous interval arithmetic or
  full corpus reference semantics.

Validation in this slice:

- `pixi run build-simulation-experimental-tests`
- `pixi run bash -lc 'build/default/cpp/Release/bin/test_rigid_ipc_fixture --gtest_color=no --gtest_filter=RigidIpcCcdCase.EvaluatesAuditedKinematicRowsWithoutZeroTimeHits'`
- `pixi run bash -lc 'build/default/cpp/Release/bin/test_rigid_ipc_fixture --gtest_color=no'`
- `pixi run bash -lc 'build/default/cpp/Release/bin/test_rigid_ipc_barrier --gtest_color=no'`
- `pixi run bash -lc 'build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no'`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: implemented manifest status for first root CCD rows

Delivered a bounded Phase 6 manifest slice:

- Updated the rigid IPC fixture manifest generator so
  `tests/data/ccd-test-000..003.json` are emitted as `implemented` rows with the
  owning DART regression and command recorded in the row metadata.
- Added generator unit coverage that proves those audited root rows are promoted
  while uncovered CCD rows remain planned.
- Kept Phase 6 open: every remaining planned manifest row still needs matching
  DART tests, examples, benchmarks, comparison packets, and evidence before it
  can be retired.

Validation in this slice:

- `pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc`
- `pixi run pytest tests/test_rigid_ipc_fixture_manifest_tools.py`
- `pixi run lint`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: reusable one-step fixture runtime replay

Delivered a bounded fixture/runtime helper slice for Phases 1 and 5:

- Added `populateAndStepRigidIpcReplayWorld(...)` in the internal rigid IPC
  fixture module. The helper populates an DART 7 `World`, applies parsed
  fixture solver metadata to the supplied `RigidIpcContactStageOptions`, runs
  exactly one opt-in `RigidIpcContactStage` step, and can return the resulting
  `RigidIpcSolverStats`.
- Added runtime coverage proving the helper applies fixture-owned barrier and
  lagged-friction policy over caller defaults: a replay scene whose caller
  supplies an inactive narrow band becomes active through parsed `dHat`, while
  parsed zero friction iterations suppress active friction rows.
- Closed Phase 1/1b in the dev-task checklist. The broader rigid IPC task
  remains open on CCD corpus parity, dense-contact robustness/convergence,
  broader friction coverage, persisted scene policy, mixed coupling, and
  manifest-row completion.

Validation in this slice:

- `pixi run lint`
- `pixi run build-simulation-experimental-tests`
- `pixi run bash -lc 'build/default/cpp/Release/bin/test_rigid_ipc_fixture --gtest_color=no --gtest_filter=RigidIpcFixtureReplay.RuntimeReplayHelperAppliesStagePolicy'`
- `pixi run bash -lc 'build/default/cpp/Release/bin/test_rigid_ipc_fixture --gtest_color=no'`
- `git diff --check`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: legacy VTK fixture mesh replay

Delivered a bounded fixture-import slice for Phase 1:

- Audited the upstream IPC comparison scripts at the pinned rigid-ipc commit and
  confirmed the current comparison-script importer already covers every
  command/body-row option present in that corpus.
- Added legacy VTK `UNSTRUCTURED_GRID` mesh loading to rigid IPC fixture replay.
  The loader supports ASCII and binary point data, accepts triangle/polygon/quad
  surface cells, triangulates supported surface cells, and attaches the result
  as the same native mesh collision shape used by the existing OBJ/OFF/MSH/STL
  paths.
- Expanded the mesh-format replay regression with binary and ASCII VTK tetra
  surfaces, covering the remaining mesh extension present in the audited
  upstream corpus.

Validation in this slice:

- `pixi run build-simulation-experimental-tests`
- `pixi run bash -lc 'build/default/cpp/Release/bin/test_rigid_ipc_fixture --gtest_color=no --gtest_filter=RigidIpcFixtureReplay.LoadsOffStlMshAndVtkMeshAssets'`
- `pixi run bash -lc 'build/default/cpp/Release/bin/test_rigid_ipc_fixture --gtest_color=no'`
- `pixi run lint`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: reusable rigid IPC fixture stage policy

Delivered a bounded fixture/runtime policy slice for Phases 1 and 5:

- Added `applyRigidIpcFixtureStageOptions(...)` in the internal rigid IPC
  fixture module. The helper applies parsed `dHat`, lagged-friction iteration
  count, `epsv`/static-friction speed, and absolute velocity-convergence
  tolerance to `RigidIpcContactStageOptions` while preserving caller-provided
  options when fixture metadata is missing, invalid, or only relative.
- Reused that helper in the runtime fixture replay solver-settings regression
  instead of open-coding the metadata mapping in the test.
- Added direct helper coverage proving exact fixture metadata is applied and
  invalid/relative metadata leaves existing stage policy unchanged.

Validation in this slice:

- `pixi run build-simulation-experimental-tests`
- `pixi run bash -lc 'build/default/cpp/Release/bin/test_rigid_ipc_fixture --gtest_color=no --gtest_filter=RigidIpcFixture.AppliesFixtureMetadataToStageOptions:RigidIpcFixtureReplay.RuntimeReplayCanUseParsedSolverSettings'`
- `pixi run bash -lc 'build/default/cpp/Release/bin/test_rigid_ipc_fixture --gtest_color=no'`
- `pixi run lint`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: rigid IPC fixture kinematic replay

Delivered a bounded fixture/runtime policy slice for Phases 1 and 5:

- Changed `populateRigidIpcReplayWorld(...)` so fixture bodies parsed as
  `RigidIpcBodyMode::Kinematic` become runtime kinematic bodies via
  `RigidBody::setKinematic(true)` instead of staying static. Fully fixed-DOF
  rows still populate as static unless the fixture explicitly marks the body
  kinematic.
- Updated replay coverage to assert parsed kinematic fixture bodies are
  runtime-kinematic, not static.
- Added a runtime replay regression that loads an inline-polygon kinematic
  fixture body, steps a `RigidIpcContactStage`, and verifies the stage advances
  both prescribed linear and angular motion even when no dynamic bodies are
  present.

Validation in this slice:

- `pixi run build-simulation-experimental-tests`
- `pixi run bash -lc 'build/default/cpp/Release/bin/test_rigid_ipc_fixture --gtest_color=no --gtest_filter=RigidIpcFixtureReplay.PopulatesWorldWithFixtureStateAndMetadata:RigidIpcFixtureReplay.RuntimeReplayAdvancesParsedKinematicBody'`
- `pixi run bash -lc 'build/default/cpp/Release/bin/test_rigid_ipc_fixture --gtest_color=no'`
- `pixi run lint`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: rigid IPC runtime friction tolerances

Delivered another bounded stage-policy slice for Phases 1, 4, and 5:

- Extended `RigidIpcContactStageOptions` with
  `staticFrictionSpeedBound` (`epsv`) and
  `frictionConvergenceTolerance`. Defaults preserve the existing runtime stage
  behavior: static friction uses `1e-3 * dt`, and zero convergence tolerance
  requires the configured lagged-friction pass count.
- Routed those options into the opt-in runtime projected-Newton solve. The
  stage converts static-friction speed to the solver's per-step static-friction
  displacement and passes the absolute friction convergence tolerance through
  to the internal lagged-friction early-stop check.
- Expanded the fixture replay stage-policy regression so parsed
  `friction_constraints.static_friction_speed_bound` and
  `ipc_solver.velocity_conv_tol` can drive `RigidIpcContactStageOptions`.
  Coverage now proves zero static-friction speed disables runtime friction rows
  and a high convergence tolerance stops lagged-friction passes after the first
  active pass.

Validation in this slice:

- `pixi run build-simulation-experimental-tests`
- `pixi run bash -lc 'build/default/cpp/Release/bin/test_rigid_ipc_fixture --gtest_color=no --gtest_filter=RigidIpcFixtureReplay.RuntimeReplayCanUseParsedSolverSettings'`
- `pixi run bash -lc 'build/default/cpp/Release/bin/test_rigid_ipc_fixture --gtest_color=no'`
- `pixi run bash -lc 'build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter=World.RigidIpcContactStage*'`
- `pixi run lint`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: rigid IPC stage options for fixture solver metadata

Delivered a bounded stage-policy slice for Phases 1, 4, and 5:

- Added `RigidIpcContactStageOptions` with max projected-Newton iterations,
  barrier activation distance (`dHat`), and lagged-friction iteration count.
  The existing max-iteration constructor remains supported, while the options
  constructor keeps the stage policy explicit for callers that load solver
  metadata from fixtures or comparison scripts.
- Routed the options into the opt-in runtime IPC stage so `dHat` controls the
  barrier active band and zero friction iterations disable lagged friction rows
  without disabling barrier contact.
- Added a fixture replay regression that parses `distance_barrier_constraint`
  and `friction_constraints` metadata, applies it to `RigidIpcContactStage`,
  and verifies narrow-band contact stays inactive while zero friction iterations
  suppress runtime friction rows on an otherwise active contact.

Validation in this slice:

- `pixi run build-simulation-experimental-tests`
- `pixi run bash -lc 'build/default/cpp/Release/bin/test_rigid_ipc_fixture --gtest_color=no --gtest_filter=RigidIpcFixtureReplay.RuntimeReplayCanUseParsedSolverSettings'`
- `pixi run bash -lc 'build/default/cpp/Release/bin/test_rigid_ipc_fixture --gtest_color=no'`
- `pixi run bash -lc 'build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter=World.RigidIpcContactStage*'`
- `pixi run lint`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: fixture replay rigid IPC friction coverage

Delivered a bounded fixture/runtime bridge slice for Phases 1 and 4:

- Added a fixture replay regression that parses two equivalent inline-polygon
  scenes with zero vs positive `coefficient_friction`, populates experimental
  `World` instances through `populateRigidIpcReplayWorld(...)`, and steps them
  with the opt-in `RigidIpcContactStage`.
- The regression verifies replayed friction metadata remains attached to the
  created bodies, the frictionless scene reports no active friction constraints,
  and the frictional scene reports active friction passes while braking
  tangential slide without reversing it.
- Updated the rigid IPC dev-task checklist and changelog to record this as
  fixture replay IPC runtime coverage. The broader rigid IPC task remains open.

Validation in this slice:

- `pixi run build-simulation-experimental-tests`
- `pixi run bash -lc 'build/default/cpp/Release/bin/test_rigid_ipc_fixture --gtest_color=no --gtest_filter=RigidIpcFixtureReplay.RuntimeReplayCarriesFrictionIntoRigidIpcStage'`
- `pixi run bash -lc 'build/default/cpp/Release/bin/test_rigid_ipc_fixture --gtest_color=no'`
- `pixi run lint`

No push or PR mutation has been made from this slice.

## Session 2026-05-31: current-main merge + sufficient-decrease line search

Branch `feature/rigid-ipc-paper-parity` was reconciled with current
`origin/main` by local merge commit `46e4af0f5a3`
(`Merge origin/main into rigid IPC paper parity`). The merge conflict was limited
to the Python experimental binding/stub surface and preserved both newer
`CollisionShapeType.CAPSULE` / `CollisionShape.capsule(...)` and this branch's
`CollisionShapeType.MESH` / `CollisionShape.mesh(...)`. No push or PR mutation
has been made from this session.

Delivered a bounded solver-quality slice for Phase 3:

- Added sufficient-decrease backtracking to
  `solveRigidIpcProjectedNewtonBarrierSystem(...)`. The solve now evaluates the
  assembled objective for feasible Newton candidates after conservative CCD
  scaling and backtracks with an Armijo-style criterion before mutating copied
  poses.
- Added aggregate internal diagnostics for sufficient-decrease objective checks
  and backtracks, and surfaced those counters through the opt-in runtime stage
  stats.
- Made the policy safe for lagged-friction/active-set cases: if the finite
  Armijo budget misses strict sufficient decrease but found a finite
  objective-decreasing candidate, the solve accepts that best decreasing
  candidate; if no candidate decreases the objective, the bounded solve stops as
  non-converged rather than reporting an unsafe CCD line-search failure.
- Added focused regressions for both the strict Armijo backtracking path and the
  finite-budget decreasing-candidate fallback.

Validation in this session:

- `pixi run lint`
- `pixi run build-simulation-experimental-tests`
- `pixi run build-py`
- `pixi run bash -lc 'build/default/cpp/Release/bin/test_rigid_ipc_barrier --gtest_color=no'`
- `pixi run bash -lc 'build/default/cpp/Release/bin/test_rigid_ipc_paper_experiments --gtest_color=no'`

## Session 2026-05-30: paper-parity examples + Filament rendering fidelity

Branch `feature/rigid-ipc-paper-parity` off `main` (the rigid IPC manifest work
is now merged to `main` via PR #2777). Delivered, each built + verified +
committed:

- **Paper-parity C++ experiment suite**
  (`tests/unit/simulation/contact/test_rigid_ipc_paper_experiments.cpp`).
  Encodes the rigid-ipc paper figures the free-body stage can express, asserting
  the two IPC invariants (intersection-free + Coulomb threshold):
  - Fig. 18 high-school friction test via tilted gravity over flat ground
    (slide below tan(theta)=0.5, stick above; friction monotonically resists,
    slide-vs-rest discriminated by final speed since the scaffold allows a few cm
    of transient creep near the exact threshold).
  - Fig. 7 spolling coin: a spun triangulated disk braked by friction,
    intersection-free.
  - Figs. 16/17 Erleben degenerate edge-on-face drop settles without
    penetration/divergence.
- **Filament rendering fidelity** (`dart/gui/detail/render_environment.cpp`):
  WINDOWED (real-GPU) views gain GTAO (bent normals, HIGH) + bloom +
  screen-space contact shadows + 4x MSAA + a 4096/4-cascade shadow atlas. Both
  modes gain cheap shader-space FXAA + temporal dithering. NOTE: an adversarial
  review flagged that enabling the GPU-heavy screen-space passes in HEADLESS
  risked the CI software rasterizer (llvmpipe); they were reverted to
  windowed-only, and headless keeps the original light shadow cost (2048 / 3
  cascades) plus FXAA + dithering. Verified by before/after headless captures and
  the headless demos-cycle smoke.
- **Python mesh `CollisionShape` binding**: `CollisionShape.mesh(vertices,
triangles)` + `CollisionShapeType.MESH` (closing the Python gap vs C++
  `makeMesh`); unit-tested through the rigid IPC path; stub regenerated.
- **New py-demo** `sx_rigid_ipc_edge_drop` (Figs. 16/17 degenerate drop),
  verified real-time (~24 ms/step, ~42 fps physics-only) and in the demos-cycle
  smoke (10 passed). The existing `sx_rigid_ipc_incline` already covers Fig. 18.

KINEMATIC / SCRIPTED BODY SUPPORT — LANDED (Fig. 13 turntable). A kinematic body
(`RigidBody::setKinematic`) advances by its prescribed velocity and acts as a
moving support/driver: the barrier/dynamics see it at its end pose, the
projected-Newton solve overrides the lagged-friction pose to the start pose (so a
moving obstacle drags its contacts) and sweeps start->end in the line search
(anti-tunneling for a moving obstacle). All overrides are gated on `hasKinematic`,
so non-kinematic scenes are byte-identical (14 runtime + 51 barrier tests
unchanged). Verified: prescribed-velocity advance, conveyor friction-drag, and a
turntable carrying a resting box CCW (Fig. 13). LIMITATION (documented): supported
motion is tangential/co-moving (drag); a kinematic body prescribed to move
normally INTO a dynamic body faster than the barrier can push it aside is not
guaranteed intersection-free (prescribed motion cannot be slowed by contact) — the
free box's inertial anchor overwhelms the barrier and the obstacle passes through.
The Python `RigidBody.is_kinematic` binding is landed (tested). Follow-ups: robust
normal-pushing, and a real-time kinematic demo (a box-on-box turntable measured
~100-190 ms/step as the rider's contact set grows, so it is covered by the C++
`RigidIpcKinematicTurntableCarriesRestingBox` regression rather than shipped as a
demo). The `KinematicBodyTag` is runtime-only (not serialized).

Dense-contact robustness -- improved (unfroze the friction arch, Fig. 11).
Dense resting contacts used to settle a few steps then freeze: the conservative
curved ACCD line search exhausted its iteration budget on a couple of tight pairs
(`Indeterminate`), which the line search treated as a zero step ->
`LineSearchBlocked` every step. Fix (`rigid_ipc_ccd.cpp` + `rigid_ipc_barrier.cpp`):
`curvedAccdAdvance` now reports the conservatively-reached `timeOfImpact` on the
`Indeterminate` exit (a proven lower bound on the true TOI), and the line search
uses it as a positive limiting step instead of blocking -- intersection-free
preserved, blocking only on zero proven progress. The line-search CCD budget was
raised 64 -> 256. Follow-up fix: zero-step line-search blocks now give adaptive
kappa a bounded retry path, the runtime stage carries raised kappa forward across
active-contact steps, and exact zero-progress resting-contact plateaus write back
the unchanged safe pose instead of surfacing as persistent failed solves. Result:
the five-voussoir `FiveVoussoirFrictionArchStandsInEquilibrium` test (Fig. 11)
now holds, the two-box stack converges faster, and
`LineSearchUsesProvenSafeTimeOnIterationExhaustion` asserts the safe-time
contract.

Still out of scope: articulated paper scenes (lock box, mechanisms, bolt,
punching press, wrecking-ball/anchor chains, card house) need joints; the larger
many-body scenes (3D packing, 560-box wrecking ball) are gated on broader corpus
coverage plus the ~3-orders-of-magnitude per-step
perf gap.

## Last Session Summary

Created PLAN-082's rigid IPC implementation surface: a dedicated plan, active
dev-task handoff, website research-catalog entry, generated upstream manifest,
manifest generator, manifest validator, and Python regression tests for the
manifest tooling. The next slices added an internal C++ fixture reader for the
first rigid fixture JSON subset plus diagnostics, then a direct CCD test-data
reader for upstream edge-edge, edge-vertex, and face-vertex rows. The current
slice added the first internal 3D rigid curved-trajectory ACCD query for
face-vertex, edge-edge, and point-edge primitives, plus rotational regressions
where endpoint-linear primitive CCD misses a mid-step contact and first
minimum-separation regressions. The latest slice added the first internal
fixture replay path that populates an DART 7 `World` with body state and
preserves mesh/fixed-DOF/kinematic metadata. The newest slice added
DART 7 mesh collision shapes, OBJ/OFF/MSH/STL fixture mesh loading, native
collision mapping, and fixture replay regressions proving loaded mesh bodies
participate in `World::collide()`. The newest slice preserves inline
`vertices`/`edges`/`polygons`, applies fixture torque during replay, replays
polygonal inline geometry as native-backed mesh collision shapes, and adds
rigid-ipc MSH surface mesh replay for comparison corpus assets. The latest
fixture slice imports the upstream IPC comparison `.txt` shape-row subset and
replays MSH-backed comparison rows into an DART 7 `World`; path-loaded
scripts now remember their source directory so upstream relative mesh paths can
replay without an explicit `assetRoot`. The newest importer slice preserves
comparison-script material Young/Poisson values plus energy model, warm-start,
self-collision, and gravity-disable metadata. It also adds the first runtime
replay regression that enters simulation mode and steps a fixture-populated
DART 7 `World` through the default sequential-impulse policy, verifying
fixture replay does not silently select IPC solver behavior. Opt-in IPC stepping
is now routed through `WorldOptions::rigidBodySolver` /
`RigidBodySolver::Ipc`. The latest CCD slice embeds
the first audited root `tests/data/ccd-test-000..003.json` rows as hermetic load
regressions and full-step miss regressions, and adds first upstream-style
edge-vertex, edge-edge, and face-vertex expected-TOI evaluator rows. The newest
CCD slice adds internal parameter-space residual helpers for the edge-vertex,
edge-edge, and face-vertex equations used by the reference interval-root
formulation, plus first parameter-box subdivision queries that find those
expected contacts within the reference TOI tolerance. The direct CCD row
evaluator now routes those first rows through the subdivision queries with the
audited reference TOI tolerance. Corpus-scale direct evaluator parity remains
open because DART does not yet use rigorous interval arithmetic or match
reference semantics across the audited corpus. The newest solver slice adds the
first internal rigid barrier scaffold for face-vertex and edge-edge terms by
transforming rigid local primitives at an interpolated pose and reusing the
current DART IPC C2 clamped-log world-primitive kernel. It is covered by focused
parity tests. The newest barrier slice adds a two-body reduced-coordinate
chain-rule layer for those face-vertex and edge-edge barrier terms, mapping
world-primitive gradients/Hessians into 6-DOF rigid-pose coordinates with local
PSD projection and finite-difference coverage. The newest barrier slice
completes the first local two-body primitive set by adding edge-vertex and
vertex-vertex barrier kernels and rigid reduced-coordinate wrappers, again with
finite-difference coverage. The newest assembly slice adds internal scene-level
active barrier records, dynamic/static body DOF offsets, global gradients, and
sparse PSD-projected Hessian rows for vertex-vertex, edge-vertex, edge-edge,
and face-vertex constraints. The latest line-search slice adds the first
conservative rigid surface line-search feasibility helper over matching
start/end surfaces. It checks edge-vertex, edge-edge, and face-vertex rows with
curved rigid CCD, treats initial separation violations and indeterminate CCD
exhaustion as unsafe zero-step results, and keeps vertex-vertex as barrier-only
coverage for now. The newest Newton slice adds a first internal
projected-Newton step helper over the assembled barrier gradient and sparse
PSD-projected Hessian, including diagonal regularization, descent-step
statistics, and line-search bound scaling/blocking. It still does not wire the
Newton step into runtime solver behavior. The newest solve-loop slice adds a
first internal barrier-only projected-Newton loop over copied rigid surfaces.
It recomputes assembly each iteration, applies reduced pose deltas to dynamic
surfaces, records convergence/progress statistics, and uses conservative
line-search bounds to scale or block candidate steps. It still does not include
physical runtime mass/inertia construction, durable runtime diagnostics,
friction, or `World` method selection. The newest dynamics-objective slice adds
first internal per-body generalized dynamics terms: diagonal quadratic pose
weights and generalized force/torque vectors assembled into the same 6-DOF
global system as barrier rows. The Newton loop can now move copied dynamic
surfaces toward those targets even without active contact. The newest physical
dynamics slice constructs those internal terms from pose, generalized velocity,
mass, diagonal inertia, generalized force/torque, and timestep. The newest
runtime slice adds an opt-in `RigidIpcContactStage` that extracts mesh-like
rigid surfaces and physical dynamics terms from free rigid bodies, runs the
internal projected-Newton IPC solve, and writes solved poses/velocities back
without replacing the default contact stage. The latest slices add same-domain
`World` rigid-solver selection, deterministic runtime sphere triangulation, and
durable stage diagnostics for solve status, last step norm, last line-search
bound, and aggregate conservative CCD line-search counters. The newest runtime
contact slice adds an activated mesh-barrier regression that moves a dynamic
body away from a static mesh surface and checks nonzero line-search diagnostics.
The latest line-search slice adds point-point curved ACCD and wires
vertex-vertex line-search checks plus point-point diagnostics into the rigid IPC
solve path. The latest runtime extraction slice rejects malformed mesh topology,
non-finite mesh vertices, and invalid box extents before those shapes reach
barrier assembly or CCD. The newest runtime convergence-policy slice adds a
max-iteration knob and prevents non-converged solve results from writing partial
poses back silently. The latest friction slices expand lagged smoothed Coulomb
friction potentials across the first rigid IPC primitive-family set:
vertex-vertex, edge-vertex, edge-edge, and face-vertex world-coordinate terms
plus reduced-coordinate coverage for vertex-vertex and edge-vertex terms. The
projected-Newton objective now assembles first lagged friction rows from active
lagged barrier constraints, uses per-body friction coefficients, and reports
active-friction diagnostics from the opt-in runtime stage.

## Current Branch

CURRENT (2026-05-30): `feature/rigid-ipc-paper-parity`, branched off `main`,
pushed to `origin/feature/rigid-ipc-paper-parity`. All earlier rigid IPC manifest

- solver work is now merged to `main` (PR #2777). This branch's commits (all
  pushed): rigid IPC paper-parity experiment suite; Filament rendering fidelity;
  Python mesh `CollisionShape` binding + `sx_rigid_ipc_edge_drop` demo; kinematic
  (prescribed-motion) rigid-body support + turntable/conveyor regressions; Python
  `is_kinematic` binding; dev-task doc updates. See the dated session summary at the
  top of this file for details. Push is authorized (sole maintainer, no PR-review
  gating).

HISTORICAL (pre-merge `feature/rigid-ipc-manifest`): pushed to
`origin/feature/rigid-ipc-manifest`, was `0 behind` / `67 ahead` of `origin/main`
after merging the latest `origin/main` three times (PLAN-081 deformable IPC, PR
#2762 py-demos, Eigen 5 / pixi upgrade #2765/#2768) before being merged via #2777.
That session added, on top of the earlier uncommitted work (now the checkpoint
commit "Add opt-in experimental rigid IPC contact stage with lagged friction"):

- Phase 4d runtime friction-behavior regression.
- First rigid IPC performance benchmark (`bm_rigid_ipc_solver`) + methodology
  ([`benchmarks.md`](benchmarks.md)).
- Broad-phase AABB cull for barrier assembly (O(N^2) -> O(N), behavior-preserving
  with an equivalence regression).
- `origin/main` merge with documented conflict resolutions.
- Isolated correctness tests for the rigid CCD pose primitives.
- Same-scene `World::step()` comparison benchmark (sequential impulse vs rigid
  IPC) establishing baseline #1: the rigid IPC scaffold is currently ~3 orders
  of magnitude slower per step and scales super-linearly (see
  [`benchmarks.md`](benchmarks.md)). This is the optimization gap to close.
- Swept broad-phase cull in the conservative line search (roadmap step 1),
  reusing the curved-trajectory speed bound plus the CCD convergence tolerance.
  Adversarially verified (>100M numerical samples + audit), which caught and
  fixed an off-by-`convergeAbs` reach bug; guarded by anti-tunneling, far-skip,
  and tolerance-band regressions. Shaved ~10–20% off the rigid IPC step.
- Measured + rejected a PSD Cholesky fast path for `projectToPsd` (net ~15%
  slower; the active reduced Hessians are typically indefinite). See the
  "Per-primitive barrier kernels" finding in [`benchmarks.md`](benchmarks.md).
- Replaced the barrier-assembly and line-search all-pairs O(N^2) surface
  enumeration with a sort-and-sweep broad phase reusing the deformable IPC sweep
  utilities (`deformable_contact::detail`, shared IPC primitives, Workstream 8),
  keeping the exact cull on candidates so results are identical.
- Merged `origin/main` a second time (now 0 behind) to pick up the deformable
  IPC advances and PR #2762 (Python `py-demos` framework).
- Added the first Rigid IPC GUI example: `sx_rigid_ipc` py-demos scene
  (Experimental category) — a free box settles on static ground via
  `World.rigid_body_solver = IPC`; verified settling (z=0.262, stable) and the
  demos-cycle smoke test. New Rigid IPC GUI examples register in
  `python/examples/demos/registry.py` under the Experimental group.
- Merged `origin/main` a third time to pick up the Eigen 5 SVD migration and
  pixi dependency upgrade (#2765, #2768); rebuilt and re-greened the
  simulation-experimental suite. NOTE: the pixi upgrade moved urdfdom 4.0->5.1,
  so the full C++/dartpy libraries must be rebuilt (`pixi run build`) before the
  Python demos load; the experimental tests do not link urdfdom and stayed green
  throughout.
- Fixed the freeze-on-contact "sink-then-stick" bug with IPC adaptive barrier
  stiffness (see the RESOLVED note below).

RESOLVED (this session): the freeze-on-contact "sink-then-stick" bug. The
runtime IPC stage used to FREEZE a free rigid body once a barrier constraint
became active. Root cause (pinned via a C++ stage-stats diagnostic): with a
fixed `kappa = 1` the barrier was orders of magnitude too soft, so under gravity
the box crept ~0.0007/step DEEPER into the barrier band (each individual step's
motion did not cross contact, so the conservative line search never limited it —
`lsLimited=0`, `lsBound=1`). Once a step finally crossed into penetration (~gap
0.001), the line search reported initial-separation violations (`lsZero=3`) ->
`LineSearchBlocked` -> `result.failed` -> the stage skipped the result, and since
the body stayed penetrating EVERY subsequent step blocked identically ->
permanent freeze.

Fix (standard IPC adaptive-kappa, ported from the `ipc-sim/rigid-ipc`
reference's `initial_barrier_stiffness` / `update_barrier_stiffness` onto DART's
squared-distance clamped-log barrier; see
`computeInitialRigidIpcBarrierStiffness` / `updateRigidIpcBarrierStiffness` in
`rigid_ipc_barrier.cpp`): the solve now picks an initial kappa that balances the
unit-barrier gradient against the inertial energy gradient
(`-gradB.dot(gradE)/|gradB|^2`), clamped to `[kappa_min, 100*kappa_min]` with
`kappa_min = minStiffnessScale * averageMass / (4*d0^2 * b''(d0^2, dhat^2))`,
`d0 = 1e-8 * bboxDiagonal`, `minStiffnessScale = 1e11`; and doubles kappa when
the closest pair keeps approaching inside the `dhatEpsilonScale = 1e-9` band.
The opt-in stage feeds it the world-AABB diagonal and average dynamic-body mass.
Two supporting changes: a relative projected-Newton gradient-convergence floor
(`relativeGradientTolerance = 1e-6`; the stiff barrier makes the absolute 1e-10
tolerance unreachable at a resting contact), and an apply policy that writes back
the best intersection-free configuration a bounded solve reaches (matching the
reference, which steps with the optimizer's best feasible iterate) rather than
discarding any not-fully-converged result. The anti-tunneling guarantee is
unchanged: a `failed` (line-search-blocked / factorization-failed) solve is still
never applied.

Verified: a box at z=0.258 over static ground with v=(1,0,0) under gravity now
slides forward and is friction-braked toward rest (kappa adapts ~7e5), z stays
~0.2565-0.2577 (gap > 0, never penetrates), `lsZero=0`, converged every step.
Covered by `RigidIpcContactStageSlidingContactDoesNotFreeze` (runtime) and the
`RigidIpcAdaptiveStiffness` detail unit tests. The drop demo settles stably; a
friction-slide GUI demo is now viable.

Next perf targets: a cheaper PSD projection or fewer active-primitive
evaluations via primitive-level candidate sets (NOT an LDLT skip), then
warm-start/active-set reuse, then the gated GPU port. Biggest correctness gates
for reference/paper parity: stiff-contact convergence robustness (above) and
Phase 2 rigorous interval-arithmetic CCD + corpus parity. All work is pushed to
`origin/feature/rigid-ipc-manifest` (sole-maintainer authorization, no
PR-review gating).

All green: `build-simulation-experimental-tests`, `test-simulation-experimental`
(23/23, including the new `RigidIpcAdaptiveStiffness` unit tests, the no-freeze
`RigidIpcContactStageSlidingContactDoesNotFreeze` regression, and the
`RigidIpcContactStageTwoBoxStackSettlesWithoutPenetration` body-body regression),
the py-demos cycle smoke (incl. `sx_rigid_ipc` + `sx_rigid_ipc_slide`), and the
manifest checks. NOTE: after the Eigen 5 merge a full clean `pixi run build` was
required to relink dartpy (urdfdom 5.1); incremental builds left mixed-ABI
objects that crashed `import dartpy`. C++ lint uses `pixi run
lint-simulation-experimental` (clang-format 22); the cached `pixi run lint`
cmake `format` target still references a stale clang-format-21 path.

## Immediate Next Step

Phase 3 production-convergence milestone landed: adaptive barrier stiffness
resolved the freeze-on-contact bug (see the RESOLVED note above), so the runtime
stage now produces continued, intersection-free contact dynamics (sliding,
friction braking, stable settling) instead of freezing. This unblocks
sliding/friction/stacking demos and the contact corpus.

Next slices, in rough priority:

1. DONE: clean `pixi run build` (urdfdom 5.1 relink) restored dartpy. The rigid
   IPC py-demos suite is FIVE real-time Experimental-category scenes, each
   verified to behave correctly AND to run at an interactive frame rate
   (per-scene trajectory + per-step wall-clock timing + demos-cycle smoke):
   `sx_rigid_ipc` (box drop, ~30ms/step settled), `sx_rigid_ipc_slide` (friction
   slide), `sx_rigid_ipc_incline` (slope friction), `sx_rigid_ipc_pile` (boxes
   into a pile), and `sx_rigid_ipc_tunnel` (no-tunneling / intersection-free
   guarantee). Heavier scenes were measured and found too slow for real-time and
   intentionally NOT shipped as demos (they "looked stuck"): a triangulated
   sphere (~175 ms/step, ~6 fps), a tight 3-box stack (~190 ms/step), and
   sphere-on-box (~300 ms/step, ~3 fps) — the known IPC perf gap (sphere = 62
   triangulation verts; tight stacks = dense face-face contacts). Those stay
   covered by C++ regressions; the demos return once the perf work lands.
   Articulated/jointed paper scenes (chains, mechanisms, octopus, pendulums,
   compactors) remain out of scope: the rigid IPC stage handles only free rigid
   bodies (box/sphere/mesh), no joints.
2. DONE (body-body): `RigidIpcContactStageTwoBoxStackSettlesWithoutPenetration`
   C++ regression + the `sx_rigid_ipc_stack`/`_pile`/`_sphere_box` demos confirm
   multiple dynamic bodies and body-body contact settle stably. Remaining:
   resting/stacking convergence corpus + rotational-settling coverage.
3. Continue Phase 4 (broader runtime friction fixture behavior) and Phase 2
   (rigorous interval CCD + corpus parity).

Known follow-up from the adversarial review of the freeze fix (low severity, not
blocking): the projected-Newton loop accepts each step on a descent-direction +
conservative-CCD-feasibility gate but has NO Armijo / monotone-energy
sufficient-decrease backtracking (the reference `NewtonSolver::line_search`
halves the step until the barrier objective decreases). The applied iterate is
therefore the last intersection-free iterate, not an energy-minimized one. This
is safe (every accepted step is penetration-free; NaN/indefinite/blocked all
route to skip) but can in principle allow minor energy non-monotonicity/jitter at
stiff or ill-conditioned contacts. Adding the sufficient-decrease backtrack would
restore reference parity and is the recommended next solver-quality slice (test
it changes step acceptance, so verify the freeze fix and demos still hold).

Continue Phase 4 from `docs/dev_tasks/rigid_ipc_solver/README.md`: extend
friction into broader runtime fixture behavior, corpus coverage, and production
convergence criteria.

Performance pillar is now in scope (maintainer directive): benchmark the rigid
IPC path against the current DART rigid contact path, the audited reference
(`/tmp/rigid-ipc` `tools/benchmark.py`), and the paper scene families, then
optimize until DART beats them. This is gated on completing the algorithm's
correctness (rigorous interval CCD, corpus parity, production convergence). The
first DART-owned rigid IPC benchmark (`bm_rigid_ipc_solver`) and the comparison
methodology, baseline snapshot, and open findings live in
[`benchmarks.md`](benchmarks.md). The first optimization (broad-phase AABB cull,
scene assembly O(N^2) -> O(N)) has landed; the next perf targets are a spatial
index for the residual all-pairs enumeration, the per-primitive kernel cost, and
the first DART-rigid-vs-IPC comparison benchmark. Phase 3 convergence/contact
corpus criteria, Phase 2 corpus parity, and Phase 1 runtime example coverage
remain open fallback slices.

## Context That Would Be Lost

- The audited upstream checkout is `/tmp/rigid-ipc` at commit
  `23b6ba6fbf8434056444ae106356fd2209136988`.
- The generated manifest currently has 798 entries and zero unclassified rows:
  300 fixture JSON paths, 405 CCD test data paths, 8 test source rows, 8
  benchmark scripts, and 77 comparison rows.
- Keep same-domain rigid method selection DART-owned and backend-neutral. Do
  not expose solver registries, ECS storage, external project names as solver
  selectors, or GPU resources in public API.
- The importer currently lives in
  `dart/simulation/io/detail/rigid_ipc_fixture.*` and is covered
  by `tests/unit/simulation/io/test_rigid_ipc_fixture.cpp`.
- `populateRigidIpcReplayWorld()` is an internal bridge only: it creates
  experimental `RigidBody` entries, preserves fixture metadata, and attaches
  supported OBJ, OFF, rigid-ipc MSH, binary STL, ASCII STL, and polygonal inline
  geometry as native-backed mesh collision shapes. It does not select a solver
  method, and unsupported or missing mesh assets are recorded in replay metadata
  instead of treated as implemented coverage.
- `loadRigidIpcComparisonScript()` imports the upstream IPC comparison `.txt`
  shape-row subset into the same fixture model. It preserves mesh poses, scale,
  material density/Young/Poisson values, initial velocities, prescribed
  velocities, Neumann body forces, energy model, warm-start flag,
  self-collision flag, gravity-disable flag, friction, barrier distance,
  friction iterations, and tolerance metadata, but it still does not implement
  solver behavior or comparison-baseline parity.
- Path-loaded fixture and comparison script imports set
  `RigidIpcFixture::sourceDirectory`, and replay uses it as the default relative
  mesh base when `RigidIpcReplayOptions::assetRoot` is empty. This matches the
  upstream comparison corpus layout while preserving the explicit override path
  for tests and generated fixtures.
- The first runtime replay regression uses inline mesh geometry, a force, and
  gravity to populate an DART 7 `World`, enter simulation mode, take one
  default `World::step()`, and assert the ordinary rigid-body velocity/position
  integration result. This is intentionally not a public example yet because the
  importer still lives under `io::detail`.
- Direct CCD rows currently cover the upstream `ee`, `ev`, and `fv` schemas as
  data records and can be replayed through the internal subdivision-backed
  curved CCD dispatcher. The first rigid curved-trajectory query lives in
  `dart/simulation/detail/rigid_ipc/rigid_ipc_ccd.*` and is intentionally
  internal.
- The first root direct CCD rows from the audited upstream corpus are covered as
  parser/topology and full-step miss regressions. Do not mark them complete in
  the manifest until DART matches the reference interval-root evaluator behavior
  and accepted tolerances.
- Direct evaluator tests now include first upstream-style expected-TOI rows for
  edge-vertex translation/rotation, edge-edge translation, and face-vertex
  translation, plus residual checks for the parameter-space equations at those
  expected contacts and subdivision-query checks that recover the same
  contacts.
- The new curved CCD tests use simple rotational face-vertex, edge-edge, and
  point-edge cases where endpoint-linear primitive CCD sees unchanged endpoints,
  but the rigid curved trajectory hits within the step.
- The first rigid barrier scaffold lives in
  `dart/simulation/detail/rigid_ipc/rigid_ipc_barrier.*` and is covered by
  `tests/unit/simulation/contact/test_rigid_ipc_barrier.cpp`. It
  returns world-primitive barrier derivatives after rigid pose interpolation and
  now has local two-body reduced-coordinate derivatives for face-vertex and
  edge-edge terms. The local barrier primitive set also includes edge-vertex
  and vertex-vertex terms via shared point-edge/point-point kernels in
  `dart/simulation/detail/deformable_contact/barrier_kernel.hpp`.
  The same module now assembles cross-body surface constraints into active row
  records, global dynamic-body gradients, and sparse PSD-projected Hessians.
  It also computes the first conservative line-search bound over matching
  start/end surfaces for vertex-vertex, edge-vertex, edge-edge, and
  face-vertex constraints, treating initial separation violations and
  indeterminate CCD exhaustion as unsafe zero-step results. A one-step
  projected-Newton helper now consumes the assembled gradient/Hessian and
  optional line-search bound, and a barrier-only iterative solve helper updates
  copied dynamic surface poses with per-iteration assembly and line-search
  checks. First generalized dynamics terms now add diagonal quadratic pose
  weights and generalized force/torque vectors to that assembly, and a physical
  construction helper now maps mass, diagonal inertia, generalized velocity,
  generalized force/torque, and timestep into those terms.
  An opt-in runtime stage now extracts mesh/box/sphere free rigid-body state,
  runs the projected-Newton IPC solve, writes solved poses/velocities back, and
  exposes solve status plus aggregate line-search counters as last-stage
  diagnostics. A focused runtime regression now covers activated mesh barriers
  moving a dynamic body away from a static surface. The runtime extractor now
  skips malformed meshes, non-finite mesh vertices, and invalid box extents
  before barrier assembly or CCD. The same stage now reports whether a result
  was applied and skips non-converged solve results instead of applying partial
  runtime poses. The first lagged friction helpers now cover vertex-vertex,
  edge-vertex, edge-edge, and face-vertex contacts in world coordinates, with
  reduced-coordinate coverage for vertex-vertex and edge-vertex. The first
  lagged friction rows are now assembled into the projected-Newton objective
  and reported in runtime diagnostics. The internal projected-Newton solve now
  supports bounded outer lagged-friction passes, zero-iteration friction
  disable, and refreshed momentum-balance/pass diagnostics. Broader geometry
  corpus coverage, runtime fixture behavior, production convergence criteria,
  and default solver activation remain open.
- The first implementation rows should come from P0 correctness fixtures:
  `fixtures/3D/unit-tests/tunneling.json`, direct `tests/data/ccd-test-*`, and
  one simple paper-facing fixture.
- The five-voussoir Fig. 11 friction arch is now a runtime regression for dense
  exact-contact resting plateaus. The opt-in stage retries zero-step line-search
  blocks by raising adaptive kappa, carries raised kappa forward while contacts
  remain active, and treats an exact zero-progress resting-contact plateau as a
  safe unchanged-pose write-back instead of a persistent failed solve.

## How To Resume

```bash
cd /home/js/dev/dartsim/dart/task_6
# Latest work is on this branch (off main); main has everything through #2777.
git checkout feature/rigid-ipc-paper-parity   # or: git checkout main
git status && git log -8 --oneline
pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc
pixi run build-simulation-experimental-tests
pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_rigid_ipc_barrier$'
pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_rigid_ipc_paper_experiments$'
pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_world$'   # incl. RigidIpcKinematic* regressions
pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_rigid_ipc_fixture$'
pixi run test-simulation-experimental
# Rendering / demos (needs a full `pixi run build` first to relink dartpy):
pixi run build
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py -q
```

Next high-value slices (none started): (1) robust normal-push for kinematic
obstacles (the documented limitation); (2) the IPC performance climb (the
~3-orders-of-magnitude per-step gap gates the remaining many-body paper scenes
-- 3D packing, wrecking ball, and larger corpus rows -- plus a real-time
turntable/conveyor demo); (3)
articulated paper scenes (lock box, mechanisms, bolt, punching press, chains)
need a joint/articulation path the free-body rigid IPC stage does not have.
Otherwise continue from the README's "Immediate Next Steps".
