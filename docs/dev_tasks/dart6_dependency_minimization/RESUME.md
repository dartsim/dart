# RESUME — DART 6.20 dependency minimization

Read [README.md](README.md) first, then
[03-native-collision-port-scoping.md](03-native-collision-port-scoping.md)
for the execution contract,
[08-backend-lifecycle-decision.md](08-backend-lifecycle-decision.md) for the backend
lifecycle decision, and
[04-orchestration-dashboard.md](04-orchestration-dashboard.md) for live lane
status.

## Current reality

Verified 2026-07-30:

- `origin/release-6.20` is
  `2ffe228c14c67e120d2a946ce9d36e8a9658044f` at this refresh.
- [PR #3381](https://github.com/dartsim/dart/pull/3381) merged at
  `2026-07-30T04:38:18Z`.
  Its exact reviewed head was
  `64d476b68ad5ae0dcca4e98abb9bba15b6962b87`, and its squash merge was
  `46719bfbd75e1f70e69b2c76fb34a3fa2b78edd5`.
- PR #3406 then Black-formatted `scripts/agent_capture.py` to repair the
  post-merge lint failure created when #3381's new file met #3405's wider
  Black scope. It changed formatting only, not detector behavior.
- The DART-owned backend is `DARTCollisionDetector`, selected by factory key
  `"dart"` and implemented under `dart/collision/dart/`.
- FCL remains the DART 6.20 built-in default. Both
  `ConstraintSolver` constructors still create
  `FCLCollisionDetector`, `WorldConfig` still defaults to
  `CollisionDetectorType::Fcl`, and the package/dependency surface remains
  FCL-required.
- The PR changed no source under `dart/collision/{fcl,bullet,ode}`, no
  default-selection source, and no package/dependency metadata.
- Only `release-6.19` and `release-6.20` exist remotely, and the only open
  DART 6 milestone is `DART 6.20.0`. Do not begin phase 6 or phase 7 on
  `release-6.20`.

This task folder remains active because the later-release default-flip and
dependency-decoupling phases are real work. The completed DART 6.20 slice is
not permission to perform either change on this branch.

The historical default-environment OSG demotion is a separate held proposal,
not an implicit remaining phase. #3116 removed GLUT and `lodepng` while
retaining OSG; #3393 proved the C++ no-OSG configuration. OSG and ImGui remain
in the default Pixi environment, and dartpy still hard-requires
`dart-gui-osg`. Do not implement that split without fresh maintainer scope
covering dartpy, published wheels, and the installed GUI component.

## DART 6.20 delivery status

| Phase | Deliverable | Status |
| --- | --- | --- |
| 0 | Incumbent baseline and acceptance envelope | Complete, #3271 |
| 1 | C++17/no-EnTT collision math core | Complete, #3281 |
| 2 | DART 6 detector adapter and shape-pair coverage | Complete through #3350 |
| 3 | Distance, raycast, voxel, CCD, and manifold parity | Complete through #3360 |
| 4 | Evidence-driven detector performance | Closed for this lane: #3362, #3364, and #3368 landed; remaining general performance work belongs to the performance-generalization task |
| 5 | Canonical backend decision | Current `DARTCollisionDetector` architecture accepted in #3381 |
| 6 | Default flip | Deferred beyond DART 6.20; no implementation branch or milestone exists |
| 7 | FCL decoupling and backend facades | Pending after an accepted phase-6 flip on a future release |

## PR #3381 acceptance evidence

The exact head satisfies the DART 6.20 contract:

- Fresh top-level Codex review reported no major issues on
  `64d476b68a`; all ten review threads are resolved.
- Review `4812900603`'s two findings are present in the merged source: the
  installed `DARTCollide` header and wrapper symbols remain available, and
  ellipsoid conversion passes its generated triangle faces into `ConvexShape`
  instead of recomputing them from all vertex triples.
- No-cache Release build, 155/155 C++ tests, 223/223 dartpy tests, and the
  43-test DART detector target in Release and assertions-enabled
  configurations passed.
- `pixi run lint`, no-cache `pixi run check-lint`, AI
  command/infrastructure checks, 357 focused AI tests, and seven AI scenarios
  passed.
- The pinned downstream aggregate passed 199/199 gz-physics tests, 4/4
  plugin/performance checks, and the gz-sim entity-system integration test.
- Older 6.20 headers against the new library preserved detector,
  group, and object sizes (`32/376/320`) and passed 20/20 guarded lifecycle
  runs. The released 6.19.4 detector header also passed 20/20.
- All ten released `DARTCollide` symbols remain exported.
- Alternating, same-core incumbent-backend A/Bs observed no FCL, ODE, or
  Bullet regression. Contact counts were identical; base/head medians were
  ODE `868/867-868 ms`, FCL `257/255-256 ms`, and Bullet `267/266 ms`.
  CPU scaling was enabled, so this is a no-regression result, not a speedup
  claim.
- Direct FCL and DART visual captures passed their text and image-quality
  oracles, including explicit ellipsoid/capsule/cone contacts and zero skipped
  contacts. Engine-rendered overlay regressions passed.
- The exact-head hosted Release job also passed its visual smoke and uploaded
  `agent-visual-smoke` artifact `8748503596`. Its FCL `box_on_ground` and DART
  `dart_shape_contacts` manifests both report `pass: true` and
  `skipped_contacts: 0`; manual inspection confirmed visible geometry, contact
  arrows, labels, and collision bounds rather than blank or failed captures.

At merge time, exact-head hosted CI was nonterminal. It is now complete:
Linux run `30510684936` finished successfully at
`2026-07-30T06:21:21Z`; Debug, coverage, Release, assertions-enabled, Eigen
alignment, and AI infrastructure all passed. Release includes the visual
smoke, AddressSanitizer, and install gates. The final named PR rollup is 18
successes, one expected skipped documentation deploy, zero pending, and zero
failures.

The squash-merge push exposed one non-collision integration failure: both
macOS jobs stopped in `Check Lint` because #3405 made Black inspect
`scripts/agent_capture.py`, whose #3381 line wrapping predated that wider
scope. PR #3406 applied Black's output and is now the release tip. The
In the `ac7b9462612` push matrix, macOS Release and Debug both passed
`Check Lint` (`2026-07-30T05:48:49Z` and `06:00:23Z`) and continued into
tests. That directly clears the merge-time formatter regression. The wider
release-tip matrix remained nonterminal at the `06:22Z` snapshot, so it is
not represented as a fully green matrix or substituted for #3381's terminal
exact-head evidence.

## Immediate next action

1. Manage [PR #3409](https://github.com/dartsim/dart/pull/3409) through review
   and CI without adding collision behavior, defaults, or dependency changes.
2. After it lands, wait for a future DART 6 release line and milestone plus
   explicit authorization before resuming phases 6 or 7.

There is no further collision-backend implementation authorized on
`release-6.20`.

## Later-release resume gate

When a maintainer creates and authorizes the proposing release line:

1. Refresh all branch, package, milestone, gz-physics, and installed-component
   evidence. Do not reuse 6.20 timing numbers as current proof.
2. Re-capture the phase-0 matrix on the default-flip PR parent and compare the
   candidate within that same capture. Preserve finite state, determinism,
   contact caps, scene-dump tolerances, and capability parity.
3. Prove the `"dart"` detector meets the accepted performance bar
   without using sleeping, contact loss, or changed physics to hide a
   regression.
4. Change the complete default surface together: both `ConstraintSolver`
   constructors, `WorldConfig`, `World` selection behavior, and parser
   fallbacks. Run full C++/Python, installed-prefix, ABI, visual, and
   gz-physics/gz-sim gates.
5. Only after that flip is accepted may phase 7 decouple FCL from core,
   rebuild the FCL/Bullet/ODE compatibility components as approved facades,
   and remove external packages.

Open design decision: whether `OdeCollisionDetector` becomes a subclassable
facade over the DART engine or follows a coordinated gz-physics change that
first removes `GzOdeCollisionDetector` inheritance. Preserve all four factory
keys and the installed component names either way.

## Standing constraints

- FCL is the DART 6.20 default. Do not change it in this task on this branch.
- `"dart"` remains the DART-owned detector key.
- Keep C++17, pybind11, installed DART 6 headers/components, released class
  layouts, and gz-physics/gz-sim behavior.
- Run `pixi run lint` before every commit.
- Use `pixi run test` and `pixi run test-py` for runtime coverage;
  `pixi run test-all` only builds.
- Run the Gazebo gate for collision, constraint, package, or default-selection
  changes:

  ```bash
  N=${DART_SAFE_JOBS:-$(python3 scripts/parallel_jobs.py)}
  DART_PARALLEL_JOBS=$N CTEST_PARALLEL_LEVEL=$N \
    pixi run -e gazebo test-gz
  ```

- Merge the latest target branch into an already-published PR branch; never
  rebase or force-push it without explicit maintainer approval.
- Do not pick up `WP-PG.*` packets from this folder. Remaining general
  performance work is owned by
  `docs/dev_tasks/dart6_performance_generalization/`.

## Historical evidence

- Phase-0 summary and acceptance envelope:
  [05-phase0-baseline-packet.md](05-phase0-baseline-packet.md).
- Raw row summaries, capture driver, analyzer, and scene-dump digests:
  [05-artifacts.md](05-artifacts.md).
- Phase-1 port contract:
  [06-phase1-port-packet.md](06-phase1-port-packet.md).
- Historical phase-2 adapter plan:
  [07-phase2-adapter-scoping.md](07-phase2-adapter-scoping.md).
- Backend lifecycle decision:
  [08-backend-lifecycle-decision.md](08-backend-lifecycle-decision.md).
