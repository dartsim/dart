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
- PR1 #3085 adds primitive plane and contact-cap collision support. Current
  review fix preserves FCL backend object identity across shape refresh and
  removes shared objects from the group that originally registered them, rather
  than from the object's current shape type.
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
- PR2 #3086 must make any behavior-preserving performance path default-on if it
  is safe for gz-physics compatibility. Any default speedup must be backed by a
  fidelity/correctness test, not just an RTF improvement.
- PR2 current default-on Bullet run, same 120-object/9000-step generated drop
  command: RTF `6.50174`, final contacts `0`, final resting `120 / 120`,
  finite state true, final hash `0x5a46e38c66967e04`. The zero-contact final
  state is expected only after all mobile skeletons are resting and the
  all-resting fast path is active.
- PR2 current explicit `--disable-deactivation` run on the same command: RTF
  `0.977486`, final contacts `360`, final resting `0 / 120`, finite state
  true, final hash `0x40e36d5812803d4`.
- PR2 current final-scene comparison, default-on vs disabled, 121 dumped shapes:
  max position delta `8.5907e-4` m, mean position delta `5.6539e-5` m, max
  quaternion L2 delta `8.5911e-4`. The sleep contact penetration gate is
  tightened to `1e-5`, and `IslandDeactivation.DefaultEnabledSettlesCloseToAlwaysActivePath`
  now guards the default-on fidelity bar on a focused drop-and-settle case.
- The direct single-body LCP shortcut is disabled pending fidelity evidence
  because it changed the explicit `--disable-deactivation` baseline. PR2's
  default speedup must come from resting-world deactivation, not from a solver
  shortcut that changes always-active physics.

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
4. Later PRs: larger backend swaps or DART 7 native collision detector ports,
   only after Bullet/FCL/ODE/native comparisons show the dependency and
   correctness tradeoffs clearly.

## Validation Log

- Built `test_CollisionGroups` after the FCL shared-object review fix.
- Ran `pixi run ./build/default/cpp/ReleaseNoProfile/tests/integration/test_CollisionGroups`
  after the FCL shared-object review fix: 22 tests passed.
- Built `test_Collision`, `test_SdfParser`, and `sdf_perf_test` after the FCL
  refresh and example changes.
- Ran `pixi run ./build/default/cpp/ReleaseNoProfile/tests/integration/test_Collision`:
  28 tests passed.
- Ran `pixi run ./build/default/cpp/ReleaseNoProfile/tests/integration/test_SdfParser`:
  6 tests passed.
- Ran `pixi run lint`.
- For PR0 review fix #3089, built `sdf_perf_test`, verified
  `--generate-objects -1` exits with status 1, ran a 6-object/20-step smoke run
  that advanced time and emitted finite final state, and ran `pixi run lint`.
- For PR2 default-on fidelity, built `sdf_perf_test`, `test_IslandDeactivation`,
  `test_World`, and `test_ContactSurface`; ran `test_IslandDeactivation`
  after adding the always-active comparison test: 19 tests passed.
- Ran `test_ContactSurface` after the PR2 contact/sleep changes: 12 tests
  passed.
- Ran `test_World` after making deactivation default-on: 6 tests passed. The
  existing `World.Cloning` test still emits a large non-finite-warning stream.
