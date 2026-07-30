# HANDOFF — DART 6.20 dependency minimization

> Refreshed 2026-07-30 after the verified merge of PR #3381. Read
> [README.md](README.md), [RESUME.md](RESUME.md), and
> [03-native-collision-port-scoping.md](03-native-collision-port-scoping.md)
> before acting.

## Outcome

The executable DART 6.20 DART-owned collision-backend slice is merged.

- Release tip:
  `2ffe228c14c67e120d2a946ce9d36e8a9658044f` at this refresh.
- Merge:
  [PR #3381](https://github.com/dartsim/dart/pull/3381), merged
  `2026-07-30T04:38:18Z` as
  `46719bfbd75e1f70e69b2c76fb34a3fa2b78edd5`.
- Exact reviewed head:
  `64d476b68ad5ae0dcca4e98abb9bba15b6962b87`.
- Public DART-owned detector:
  `DARTCollisionDetector`, canonical factory key `"dart"`.
- DART 6.20 default:
  FCL, unchanged.
- Post-merge lint repair:
  #3406 Black-formatted `scripts/agent_capture.py` after #3405 widened the
  Black scope. It is formatting-only and is included in the release tip above.

The FCL, Bullet, and ODE implementations, installed components, dependencies,
and default-selection paths are unchanged. The released `DARTCollide` header
and all ten historical symbols remain available through adapters.

## Phase status

| Phase | Status |
| --- | --- |
| 0 — baseline packet | Complete, #3271 |
| 1 — collision math core | Complete, #3281 |
| 2 — DART 6 adapter and pair coverage | Complete through #3350 |
| 3 — distance/raycast/voxel/CCD/manifolds | Complete through #3360 |
| 4 — measured detector optimization | Closed for this lane after #3362, #3364, #3368, and #3381; remaining general performance work is owned by the performance-generalization task |
| 5 — canonical backend decision | Current architecture accepted in #3381; future facade lifecycle remains designed but unimplemented |
| 6 — default flip | Deferred beyond DART 6.20 |
| 7 — FCL decoupling and external-backend facades | Pending after a future accepted default flip |

The task folder remains active by maintainer direction because phases 6 and 7
are real later-release work. Do not retire it as a substitute for those phases.

The default-environment OSG demotion is not one of those phases and is not
silently authorized by this handoff. #3116 removed GLUT and `lodepng`, not OSG;
#3393 added a no-OSG C++ gate, but OSG/ImGui remain default Pixi dependencies
and dartpy still hard-requires `dart-gui-osg`. The #3107 maintainer hold remains
in force until a fresh scope covers dartpy, wheel publishing, and the installed
GUI component.

## Acceptance evidence

Review:

- The final top-level Codex review found no major issues on
  `64d476b68a`.
- All ten review threads are resolved.
- Review `4812900603`'s two findings were addressed: the installed
  `DARTCollide` header and wrapper symbols were retained, and ellipsoid
  conversion now reuses its generated triangle faces instead of rebuilding
  convex faces from all vertex triples.
- Review fixes included released-layout sidecar lifetime ownership,
  synchronization of all one-time shape warnings, and AABB rejection before
  filters/narrowphase in both serial and parallel two-group paths.

Local exact-head gates:

- 155/155 C++ tests and 223/223 dartpy tests.
- 43/43 DART detector tests in Release and assertions-enabled builds.
- `pixi run lint`, no-cache `pixi run check-lint`, 357 focused AI tests, and
  seven AI scenarios.
- 199/199 gz-physics tests, 4/4 plugin/performance checks, and the gz-sim
  entity-system integration test.
- Old-header/current-library ABI canaries preserve detector/group/object sizes
  `32/376/320` and pass 20/20 lifecycle runs; the released 6.19.4 detector
  header also passes 20/20.
- All ten historical `DARTCollide` symbols remain exported.
- FCL/DART visual captures and four engine-rendered overlay regressions pass.
- The hosted exact-head Release visual smoke passed and uploaded
  `agent-visual-smoke` artifact `8748503596`. Both the FCL `box_on_ground` and
  DART `dart_shape_contacts` manifests report `pass: true` with zero skipped
  contacts. Manual inspection confirmed visible scene geometry, contact
  arrows, labels, and collision bounds.

Incumbent-backend runtime guard:

| Backend | Base median | Head median | Contacts |
| --- | ---: | ---: | ---: |
| ODE | 868 ms | 867–868 ms | 205 |
| FCL | 257 ms | 255–256 ms | 81 |
| Bullet | 267 ms | 266 ms | 90 |

The runs alternated base/head on one CPU with five repetitions per invocation.
CPU scaling remained enabled, so the supported claim is no observed regression,
not a speedup.

Hosted exact-head state at merge:

- 16 successful checks;
- one expected skipped documentation deploy;
- zero failed checks;
- Linux coverage, Release, and assertions-enabled still running;
- Linux Eigen alignment still queued.

Post-merge terminal result: Linux run `30510684936` completed successfully at
`2026-07-30T06:21:21Z`. All six jobs passed: Debug, coverage, Release,
assertions-enabled, Eigen alignment, and AI infrastructure. Release includes
the verified visual artifact, AddressSanitizer, and install gates. The final
named PR rollup is 18 successes, one expected skip, zero pending, and zero
failures.

The `46719b` squash-merge push had a separate macOS lint-only failure:
#3405's wider Black scope began checking #3381's new
`scripts/agent_capture.py`. Both macOS configurations stopped before tests.
PR #3406 applied Black's formatter output and advanced the release tip to
`ac7b9462612`. In macOS run `30516350133`, Release and Debug passed
`Check Lint` at `2026-07-30T05:48:49Z` and `06:00:23Z`, respectively, then
continued into tests. This clears the formatter regression. The wider
release-tip matrix remained nonterminal at the `06:22Z` snapshot and is not
claimed as a fully green matrix.

The documentation closeout is
[PR #3409](https://github.com/dartsim/dart/pull/3409) on
`docs/dependency-minimization-closeout`.

## Exact next action

1. Manage PR #3409 through review and CI without changing collision behavior,
   defaults, or dependencies.
2. After it lands, wait for an explicitly authorized future DART 6 release
   before resuming phases 6 or 7.

After that, stop DART 6.20 collision-backend implementation. No default flip,
facade conversion, component removal, or dependency removal is authorized on
this branch.

## Later-release continuation

No `release-6.21` or `release-6.22` branch or milestone exists as of
2026-07-30. When a future proposing release is created and explicitly
authorized:

1. refresh the full baseline and downstream contract;
2. prove a default flip with current parent-vs-candidate correctness,
   determinism, performance, ABI, package, visual, and gz evidence;
3. change the entire default-selection surface together;
4. only after acceptance, decouple FCL from core and implement the approved
   FCL/Bullet/ODE facade/component plan.

The remaining ratification point is ODE: keep a subclassable
`OdeCollisionDetector` facade for `GzOdeCollisionDetector`, or coordinate a
gz-physics change that removes that inheritance first.

## Load-bearing constraints

- FCL remains the DART 6.20 default in both `ConstraintSolver` constructors,
  `WorldConfig`, `World`, and parser fallback behavior.
- `"dart"` remains the DART-owned detector key.
- Preserve C++17, pybind11, installed headers/components, released class
  layouts, and gz-physics/gz-sim behavior.
- Keep FCL/Bullet/ODE runtime paths structurally isolated from DART detector
  implementation work.
- Every future collision/default/package PR requires the Gazebo gate.
- Performance gains may not come from lost contacts, altered sleeping,
  changed physics, cap hits, or non-finite state.
- Remaining `WP-PG.*` implementation belongs to
  `docs/dev_tasks/dart6_performance_generalization/`, not this folder.

## Historical evidence

- [05-phase0-baseline-packet.md](05-phase0-baseline-packet.md): incumbent
  envelope and future default-flip acceptance contract.
- [05-artifacts.md](05-artifacts.md): raw summaries, artifact digests, capture
  driver, and tolerance analyzer.
- [06-phase1-port-packet.md](06-phase1-port-packet.md): internal C++17/no-EnTT
  port rules.
- [07-phase2-adapter-scoping.md](07-phase2-adapter-scoping.md): historical
  adapter execution plan.
- [08-backend-lifecycle-decision.md](08-backend-lifecycle-decision.md):
  current backend architecture and later lifecycle decision.
