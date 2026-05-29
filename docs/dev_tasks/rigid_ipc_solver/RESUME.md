# Resume: Rigid IPC Solver

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
fixture replay path that populates an experimental `World` with body state and
preserves mesh/fixed-DOF/kinematic metadata. The newest slice added
experimental mesh collision shapes, OBJ/OFF/MSH/STL fixture mesh loading, native
collision mapping, and fixture replay regressions proving loaded mesh bodies
participate in `World::collide()`. The newest slice preserves inline
`vertices`/`edges`/`polygons`, applies fixture torque during replay, replays
polygonal inline geometry as native-backed mesh collision shapes, and adds
rigid-ipc MSH surface mesh replay for comparison corpus assets. The latest
fixture slice imports the upstream IPC comparison `.txt` shape-row subset and
replays MSH-backed comparison rows into an experimental `World`; path-loaded
scripts now remember their source directory so upstream relative mesh paths can
replay without an explicit `assetRoot`. The newest importer slice preserves
comparison-script material Young/Poisson values plus energy model, warm-start,
self-collision, and gravity-disable metadata. It also adds the first runtime
replay regression that enters simulation mode and steps a fixture-populated
experimental `World` through the default rigid-body pipeline, verifying fixture
replay still does not select IPC solver behavior. The latest CCD slice embeds
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

`feature/rigid-ipc-manifest` - local branch, committed and currently `0 behind`
/ `48 ahead` of `origin/main` after merging the latest `origin/main` (merge
commit integrates the parallel deformable IPC effort, PLAN-081). No PR is open
yet (push/PR deferred per maintainer choice). Stale upstream tracking was
cleared. This session added, on top of the earlier uncommitted work (now the
checkpoint commit "Add opt-in experimental rigid IPC contact stage with lagged
friction"):

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
  and tolerance-band regressions. Shaved ~10–20% off the rigid IPC step. Next
  perf targets: spatial index for the residual all-pairs enumeration, the
  per-primitive kernel/PSD cost, then warm-start/active-set reuse, then GPU.

All green: `build-simulation-experimental-tests`, `test-simulation-experimental`
(23/23), `check-lint`, and the manifest checks.

## Immediate Next Step

Phase 4d landed: a differential `World` regression
(`RigidIpcContactStageFrictionBraketsTangentialSlide`) now proves lagged
friction observably brakes a tangential slide at an activated mesh contact
relative to the frictionless solve. Continue Phase 4 from
`docs/dev_tasks/rigid_ipc_solver/README.md`: extend friction into broader
runtime fixture behavior, corpus coverage, and production convergence criteria.

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
  `dart/simulation/experimental/io/detail/rigid_ipc_fixture.*` and is covered
  by `tests/unit/simulation/experimental/io/test_rigid_ipc_fixture.cpp`.
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
  gravity to populate an experimental `World`, enter simulation mode, take one
  default `World::step()`, and assert the ordinary rigid-body velocity/position
  integration result. This is intentionally not a public example yet because the
  importer still lives under `io::detail`.
- Direct CCD rows currently cover the upstream `ee`, `ev`, and `fv` schemas as
  data records and can be replayed through the internal subdivision-backed
  curved CCD dispatcher. The first rigid curved-trajectory query lives in
  `dart/simulation/experimental/detail/rigid_ipc_ccd.*` and is intentionally
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
  `dart/simulation/experimental/detail/rigid_ipc_barrier.*` and is covered by
  `tests/unit/simulation/experimental/contact/test_rigid_ipc_barrier.cpp`. It
  returns world-primitive barrier derivatives after rigid pose interpolation and
  now has local two-body reduced-coordinate derivatives for face-vertex and
  edge-edge terms. The local barrier primitive set also includes edge-vertex
  and vertex-vertex terms via shared point-edge/point-point kernels in
  `dart/simulation/experimental/detail/deformable_contact/barrier_kernel.hpp`.
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

## How To Resume

```bash
cd /home/js/dev/dartsim/dart/task_6
git checkout feature/rigid-ipc-manifest
git status && git log -3 --oneline
pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc
pixi run build-simulation-experimental-tests
pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_rigid_ipc_barrier$'
pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_continuous_collision_step|test_deformable_body)$'
pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_rigid_ipc_fixture$'
pixi run test-simulation-experimental
```

Then continue from the README's "Immediate Next Steps".
