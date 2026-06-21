# Issue 3056 DART 6 Performance

## Current Snapshot

Bottom line: DART 6 performance work is continuing on `release-6.20` as a PR
stack. The current managed PR, #3071 `perf/dart6-parallel-islands`, is the
parallel-stepping follow-up after #3111 and #3112 landed the core
`setNumSimulationThreads` path on the base branch. It should only be merged as
a performance PR with evidence that the thread knob preserves final state and
has a real scaling window.

Current #3071 evidence on an Intel i7-13800H, pinned to CPUs 0-15:

- Original #3056 SDF, Bullet collision detection, DART 6 dynamics/constraints,
  default deactivation, 3000 steps: RTF `1.50858` at 1 thread, `1.52074` at 2,
  `1.5381` at 4, `1.55879` at 8, and `1.51651` at 16. All runs advanced
  exactly `3000 / 3000` frames, ended finite, and matched final hash
  `0x2375f1927218cd43`. This scene is already resting-fast-path dominated, so
  thread scaling is intentionally modest.
- Same original SDF with `--disable-deactivation`, 300 active steps: RTF
  `0.0228284` at 1 thread, `0.0489761` at 8, and `0.046549` at 16, with
  identical final hash `0x1b1e6f3c78c0e01e` and `5005` final contacts across
  `3003` contact pairs. This shows a real active-pipeline speedup to 8 threads,
  while also showing that active contact/collision work remains below real
  time.
- Deterministic articulated workload
  `SCENE=chains LINKS=8 THREADS=N pixi run bm-boxes-headless -- 16 300 0`:
  `1793.239 ms` at 1 thread, `1353.723 ms` at 2, `1293.283 ms` at 4,
  `1006.875 ms` at 8, and `1182.029 ms` at 16, with identical final checksum
  at every thread count. Peak local speedup is `1.78x` at 8 threads; 16 threads
  regresses on this machine.
- Smaller workloads are not good proof points: 512 falling boxes improved only
  from `8806.970 ms` to `8168.554 ms` at 8 threads and regressed at 16, while
  the 64-chain workload regressed at every tested thread count. The PR evidence
  should present this caveat instead of implying unlimited CPU scaling.

If #3071 lands, the next performance phase should import and benchmark the DART
7 native collision detector into DART 6.20's `dart/collision/dart/` path,
compare it against Bullet/FCL/ODE on the same original #3056 commands, and only
prefer it if it beats Bullet while preserving final-state and focused collision
correctness.

- Current active branch: `perf/dart6-parallel-islands`, based on
  `origin/release-6.20`.
- Exact issue scene used for the current evidence: `/tmp/3k_shapes.sdf`,
  3003 mobile sphere/box/cylinder bodies plus the static ground plane from the
  original issue report. The ground link warning about missing `<inertial>` is
  from that static ground model in the issue input; the mobile bodies have
  inertials.
- Current exact-scene command:
  `pixi run ex contact_benchmark -- /tmp/3k_shapes.sdf --steps 3000 --warmup 0 --checkpoint 0 --quiet --collision bullet --sdf-plane-shapes --world-threads 16 --max-contacts 12000`.
  This uses Bullet only for collision detection; DART 6 still owns dynamics,
  constraints, sleep/deactivation, and the LCP solve.
- Current exact-scene result, default deactivation enabled: RTF `1.50` across
  repeated 16-thread samples (`1.47286`, `1.50528`, `1.51054`, `1.51119`,
  `1.50657`, `1.51609` through `pixi run ex contact_benchmark -- ...`),
  finite final state, final hash `0x2375f1927218cd43`, final resting
  `3003 / 3003`, final contacts `0`, and frame/time advanced exactly
  `3000 / 3000` steps.
- Same exact command with `--world-threads 1` gives RTF `1.46391`; with
  `--world-threads 4` gives RTF `1.49184`. The current win is therefore from
  prompt deactivation of the initially settled issue scene, not from thread
  scaling.
- Same exact command with `--disable-deactivation` gives RTF `0.0559457` to
  `0.0604573`, finite final state, final hash `0x8aa3ab9f1fe495c6`, final
  contacts `7005`, and final contact pairs `3003`.
- Exact-scene final dump comparison, default-on vs disabled deactivation,
  matched all `3003` mobile shapes: max position delta
  `3.4147863965736968e-06` m, mean position delta
  `1.8350550992622476e-06` m, max quaternion L2 delta
  `1.9714098667727154e-06`.
- Current profiler shape on the exact scene: only `3` full
  constraint/collision solves, followed by `2997` all-resting cached steps.
  Inclusive simulation time was `1.993 s`; all-resting readiness was `1.221 s`,
  all-resting fast path was `265 ms`, and the three Bullet collision calls were
  `452 ms` total. The next measured hotspot is the readiness scan, not the
  LCP solve.
- FCL and ODE exact-scene attempts with the same SDF plane-shape command did
  not reach the setup summary within practical local budgets (FCL stopped after
  about `5m57s`, ODE after about `1m42s`). These are incomplete setup attempts,
  not valid RTF numbers. Bullet is the only complete exact-scene collision
  backend sample in the current pass.
- Correctness coverage added for the prompt initial-rest path:
  `IslandDeactivation.InitiallySettledShallowContactCanSleepPromptly` proves a
  shallow zero-velocity support contact can consume initial dwell but still
  takes one final solved impulse before freezing.
  `IslandDeactivation.InitialMobilePairContactDoesNotSleepPromptly` and
  `IslandDeactivation.InitialWallContactDoesNotSleepPromptly` prove the dwell
  credit is not granted to unsupported mobile-mobile contacts or vertical wall
  contacts. The existing
  `IslandDeactivation.UnconvergedContactClearsSleepCandidate` guards the deep
  penetration case.
- Downstream gate status on this branch: gz-physics passed in the previous
  full `pixi run -e gazebo test-gz` attempt. The gz-sim portion initially
  failed before exercising physics because the source-built gz-physics install
  placed engine plugins under `lib64` and left the unversioned installed engine
  alias broken; `scripts/run_gz_sim_task.sh` now discovers `lib64` and includes
  the valid source-build plugin alias for the local integration gate. A fresh
  `pixi run -e gazebo test-gz-sim` rerun passed
  `INTEGRATION_entity_system`.

- PR0 `jslee02/issue-3056-baseline-sdf-perf` / #3089 adds the baseline
  benchmark, GUI verifier, reset path, HUD, drop-state scene generation, and
  final-state checks. Review fix `a2621f80185` rejects signed size arguments
  before `std::stoull` and makes final-state digest size hashing portable.
- Baseline evidence, Bullet collision, 120 generated objects, 9000 steps,
  default sleeping: RTF `0.882581`, final contacts `360`, final resting
  `81 / 120`, finite state true, final hash `0x212c1143bd0a2fb3`.
- Baseline evidence, same scene with deactivation disabled: RTF `0.740188`,
  final contacts `360`, final resting `0 / 120`, finite state true, final hash
  `0xcf3145a17b2cada2`.
- PR1 #3085 added primitive plane and contact-cap collision support, passed
  Codex review, and is merged into `release-6.20`.
- PR1 current default Bullet run, same 120-object/9000-step command as PR0:
  RTF `0.915975`, final contacts `360`, final resting `81 / 120`, finite state
  true, final hash `0x212c1143bd0a2fb3`. This preserves PR0 final state.
- PR1 `--max-contacts-per-pair 4`, same Bullet command: RTF `0.897063`, same
  final hash `0x212c1143bd0a2fb3`. This is correctness-preserving here but not
  a speedup for Bullet.
- PR1 FCL primitive short check, 120 objects, 3000 steps: uncapped RTF
  `0.0511826`, capped at 4 contacts/pair RTF `0.131733`, but final hashes and
  state diverged substantially. That cap must remain explicit for this path
  unless follow-up work proves a physically equivalent cap policy.
- Next collision diff should implement a contact-manifold reduction/selection
  algorithm before making any four-contact-per-pair behavior default-on. The
  goal is to choose representative, well-distributed contact points for each
  pair instead of truncating backend contact output in iteration order. Required
  evidence: same original #3056 command line, baseline vs previous vs current
  RTF, final-state hash/summaries, and focused tests showing preserved contact
  support for boxes, cylinders, spheres, and planes.
- Prior-art search found three useful sources: closed PR #2366 had a generic
  `constraint::ContactManifoldCache` with persistent pair history, deepest
  contact plus spatially spread contact selection, a box-stacking GUI
  comparison, C++/dartpy tests, and `bm_contact_manifold_cache`; merged PR
  #2125 and release backport #2902 added an ODE-specific DART 6 contact-history
  cache for sparse capsule contacts; and merged DART 7 native-collision PRs
  #2652/#2688 added `ContactManifold`, `PersistentManifoldCache`, and box-box
  contact reduction under `dart/collision/native`.
- Sequencing decision: keep #3085 as the plane/contact-cap plumbing and evidence
  PR. The default-on behavior belongs in a later manifold-reduction PR that
  mines #2366 and DART 7 native collision, then proves equivalence before
  changing core defaults.
- PR2 #3086 must make any behavior-preserving performance path default-on if it
  is safe for gz-physics compatibility. Any default speedup must be backed by a
  fidelity/correctness test, not just an RTF improvement.
- PR2 current default-on Bullet run after gz-physics fidelity fixes, same
  120-object/9000-step generated drop command: RTF `2.52099`, final contacts
  `0`, final resting `120 / 120`, finite state true, final hash
  `0xf8661e0f2baad9f9`. The zero-contact final state is expected only after
  all mobile skeletons are resting and the all-resting fast path is active.
- PR2 current explicit `--disable-deactivation` run on the same command: RTF
  `0.489052`, final contacts `360`, final resting `0 / 120`, finite state true,
  final hash `0xc68e1f3b0fa9dc83`.
- PR2 current final-scene comparison, default-on vs disabled, 121 dumped shapes:
  max position delta `0.00118920391212` m, mean position delta
  `5.35687462563e-5` m, max quaternion L2 delta `0.00118926675519`. The sleep
  contact penetration gate is
  tightened to `1e-5`, and
  `IslandDeactivation.DefaultEnabledSettlesCloseToAlwaysActivePath` now guards
  the default-on fidelity bar on a focused drop-and-settle case.
- The direct single-body LCP shortcut is disabled pending fidelity evidence
  because it changed the explicit `--disable-deactivation` baseline. PR2's
  default speedup must come from resting-world deactivation, not from a solver
  shortcut that changes always-active physics.
- Current PR2 review fixes keep the all-resting fast path conservative: a
  sleeping body with externally edited velocity wakes before the cached step is
  reused, and contacts inside a frozen mobile island are preserved during an
  awake impact so wakeup remains island-atomic.
- Current PR2 gz-physics fixes keep sleep transitions from advancing the final
  solved impulse into pose, zero body velocity caches when entering rest, clear
  stale sleep candidacy when an active contact group cannot rest, and prevent a
  quiet support contact from sleeping while another mobile body in the world is
  still above the wake-speed band.
- Current PR2 filter-cache fix keys the all-resting fast path on the solver
  collision-filter pointer and BodyNode-pair blacklist revision. That wakes
  sleepers when support contacts are removed by a solver filter edit while
  preserving the GUI sleep-color/visual-only no-op path.
- Current PR2 compatibility review fixes keep the legacy solver sleeping setter
  as a source-compatible alias and preserve ordinary self-collision/blacklist
  filtering before the solver-local resting-island contact preservation path.
- Current PR2 custom-filter fix disables all-resting snapshot reuse for filters
  whose decision state is not revision-tracked, so stateful custom filters
  cannot hide changed contact decisions behind the cached fast path.
- Parallel follow-up: #3071 has been reconciled with the current
  `setNumSimulationThreads` / `ConstraintThreadPool` API. The useful remaining
  diff exposes the control in dartpy, adds an in-tree Pixi benchmark task for
  `boxes_headless`, and records deterministic scaling evidence. It should not
  claim the resting original #3056 scene scales linearly; the active original
  scene and larger articulated workload are the evidence for the thread knob.

## Default-On Correctness Rule

Performance behavior should be default-on only when the physical behavior stays
the same within the intended DART 6 compatibility surface. A fast path that
changes contact generation, wake/sleep transitions, reset behavior, final
finite-state checks, or final state summaries must be treated as a correctness
bug until proven otherwise.

## PR Split

1. PR0: reproducible baseline tests, benchmark/example, GUI/headless verifier,
   final-state consumption, and `.benchmark_results/` ignore coverage.
2. PR1: collision detector and contact-generation improvements. Compare against
   PR0 and add focused collision correctness regressions.
3. PR2: resting-world/deactivation and solver hot-loop improvements. Compare
   against PR0 and PR1, and verify default-on behavior preserves fidelity.
4. PR3 candidate: contact-manifold reduction/selection. This should be a
   separate review from PR1 because PR1's per-pair cap is a measurement and
   control knob, while this follow-up must preserve contact-patch fidelity well
   enough to justify default-on behavior.
5. Parallel-safety follow-up: keep #3086's opt-in independent-island solve on
   the serial path when solver type, manual/custom constraints, or shared
   non-reactive dependencies make parallel independence uncertain. This is a
   correctness-preserving prerequisite for later parallel speedups, not a
   performance-claim PR.
6. Later PRs: larger backend swaps or DART 7 native collision detector ports,
   only after available collision-engine comparisons show the dependency and
   correctness tradeoffs clearly.

## Validation Log

- Built `test_CollisionGroups` after the FCL shared-object review fix.
- Ran `pixi run ./build/default/cpp/ReleaseNoProfile/tests/integration/test_CollisionGroups`
  after the FCL shared-object review fix: 22 tests passed.
- Built `test_Collision`, `test_SdfParser`, and `contact_benchmark` after the FCL
  refresh and example changes.
- Ran `pixi run ./build/default/cpp/ReleaseNoProfile/tests/integration/test_Collision`:
  28 tests passed.
- Ran `pixi run ./build/default/cpp/ReleaseNoProfile/tests/integration/test_SdfParser`:
  6 tests passed.
- Ran `pixi run lint`.
- For PR0 review fix #3089, built `contact_benchmark`, verified
  `--generate-objects -1` exits with status 1, ran a 6-object/20-step smoke run
  that advanced time and emitted finite final state, and ran `pixi run lint`.
- For PR2 default-on fidelity, built `contact_benchmark`,
  `test_IslandDeactivation`, `test_World`, and `test_ContactSurface`; ran
  `test_IslandDeactivation` after adding the always-active comparison test: 19
  tests passed.
- Ran `test_ContactSurface` after the PR2 contact/sleep changes: 12 tests
  passed.
- Ran `test_World` after making deactivation default-on: 6 tests passed. The
  existing `World.Cloning` test still emits a large non-finite-warning stream.
- After #3085 merged, merged `release-6.20` into PR2 #3086, removed the stale
  `sdf_perf_test` target path, rebuilt `contact_benchmark`,
  `test_IslandDeactivation`, `test_World`, and `test_ContactSurface`, ran those
  three focused tests through CTest, ran a 3-object `contact_benchmark` smoke,
  and ran `pixi run lint`.
- For the PR2 Codex support-edit review, collision geometry/collidability
  edits now invalidate the all-resting cache, support removal wakes remaining
  sleepers, and `test_IslandDeactivation` covers support removal, support
  collidability disable, and support shape transform/resize. Rebuilt
  `contact_benchmark`, `test_IslandDeactivation`, `test_World`, and
  `test_ContactSurface`; CTest passed the three focused tests; the 3-object
  `contact_benchmark` smoke advanced time with finite final state.
- For the PR2 Codex velocity/island review, `test_IslandDeactivation` now covers
  velocity edits while the all-resting cache is hot and same-step wake of a
  frozen mobile island. Rebuilt `test_IslandDeactivation`, then rebuilt
  `contact_benchmark`, `test_World`, and `test_ContactSurface`; CTest passed
  the three focused tests, and the 3-object `contact_benchmark` smoke advanced
  time with finite final state.
- Reran the 120-object/9000-step Bullet comparison after those review fixes:
  default-on RTF `3.45052`, disabled RTF `0.490135`, both finite. Dumped both
  final scenes and compared 121 shapes: max position delta `0.00122575` m, mean
  position delta `5.74756e-5` m, max quaternion L2 delta `0.00122581`.
- After #3085 merged and #3086 failed gz-physics CI, reproduced
  `COMMON_TEST_joint_features_dartsim` and `UNIT_SDFFeatures_TEST` locally,
  added deactivation regressions for final sleep-transition pose/velocity,
  unconverged contacts, and waiting for other active mobile bodies, rebuilt
  `contact_benchmark`, `test_IslandDeactivation`, `test_World`, and
  `test_ContactSurface`, and passed those three focused DART tests through
  CTest.
- Reinstalled DART into the Gazebo pixi environment and passed the focused
  gz-physics failures:
  `COMMON_TEST_joint_features_dartsim` and `UNIT_SDFFeatures_TEST`.
- Reran the 120-object/9000-step/drop-height-0.2 Bullet comparison after the
  gz-physics fidelity fix: default-on RTF `2.52099`, disabled RTF `0.489052`,
  both finite. Dumped both final scenes and compared 121 shapes: max position
  delta `0.00118920391212` m, mean position delta `5.35687462563e-5` m, max
  quaternion L2 delta `0.00118926675519`.
- Ran `pixi run -e gazebo test-gz-physics`: gz-physics build succeeded, 199/199
  tests plus 4/4 performance tests passed, and the DART plugin linked against
  the pixi-installed DART libraries.
- For the PR2 Codex collision-filter review, added a solver-filter support
  removal regression and narrowed the all-resting snapshot revision to the
  BodyNode-pair blacklist so visual-only color changes remain cached. Rebuilt
  `contact_benchmark`, `test_IslandDeactivation`, `test_World`, and
  `test_ContactSurface`; CTest passed the three focused tests, and the 3-object
  `contact_benchmark` smoke advanced time with finite final state.
- Rebuilt gz-physics' test-enabled DART plugin tree against the current DART
  install and passed the focused downstream checks:
  `COMMON_TEST_joint_features_dartsim` and `UNIT_SDFFeatures_TEST`.
- For the PR2 Codex compatibility/filter-order reviews, rebuilt
  `test_IslandDeactivation` and `test_ConstraintSolver`, then passed both
  through focused CTest.
- For the PR2 Codex custom-filter review, rebuilt `test_IslandDeactivation` and
  passed the focused CTest after adding a stateful custom-filter regression.
- For the parallel-safety follow-up, built `test_ConstraintSolver` and
  `test_World`, passed focused new tests covering exact built-in LCP solvers,
  randomized PGS, manual constraints, custom contact constraints, and shared
  non-reactive dependencies, then passed both integration test binaries through
  focused CTest.
- For the deactivation parallel-solve / exact-issue pass, rebuilt
  `contact_benchmark`, `test_IslandDeactivation`, and `test_ConstraintSolver`;
  ran `test_IslandDeactivation` (50 tests passed) and `test_ConstraintSolver`
  (18 tests passed), then ran `pixi run lint` and reran the same focused build
  and tests successfully. Exact issue SDF evidence used Bullet collision
  detection with SDF plane shapes, 3000 measured steps, final-state consumption,
  and final scene dumps for default-on vs `--disable-deactivation`.
- For the same branch, a full `pixi run -e gazebo test-gz` attempt passed the
  gz-physics build, 199/199 gz-physics tests, and 4/4 gz-physics performance
  tests, then the gz-sim clone hit a network timeout. A later
  `pixi run -e gazebo test-gz-sim` reached `INTEGRATION_entity_system` but
  failed with `Failed to find plugin [gz-physics-dartsim-plugin]`; inspection
  showed the source-built gz-physics install under `lib64` and a broken
  installed unversioned engine-plugin symlink. After updating
  `scripts/run_gz_sim_task.sh` to discover `lib64` and include the valid
  build-tree plugin alias, a fresh `pixi run -e gazebo test-gz-sim` rerun passed
  `INTEGRATION_entity_system`.
