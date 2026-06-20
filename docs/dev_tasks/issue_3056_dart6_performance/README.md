# Issue 3056 DART 6 Performance

## Current Snapshot

Bottom line: DART 6 performance work is continuing on `release-6.20` as a PR
stack. The baseline PR records the workload and consumes final state so RTF
numbers are not dead-code artifacts. Follow-up PRs must compare baseline,
previous PR, and current PR on the same command line whenever they claim a
speedup.

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
5. Later PRs: larger backend swaps or DART 7 native collision detector ports,
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
