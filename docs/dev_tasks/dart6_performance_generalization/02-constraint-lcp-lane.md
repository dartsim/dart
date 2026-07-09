# WS-A — Constraint/LCP pipeline lane

The active-regime wall: profile smoke shows the Dantzig LCP solve scope at
~66% of step time (caveat: single run, without the `Construct LCP` vs
solve-proper split — WP-PG.01/WP-PG.10 must record that split before
committing effort ratios), and every island builds a dense `(3n)^2` A
matrix via `O(n^2)` unit-impulse ABI passes
(`BoxedLcpConstraintSolver.cpp:244`). The #3142 direct-assembly fast path
covers only islands whose contacts all share one reactive body. Round 1
already parallelized/cached the constraint *build*; this lane attacks
assembly and solve *structure*, plus the penetration-creep root cause
that keeps piles active in the first place (WP-PG.15).

Common gates for all packets: the README envelope, the full per-packet
gate list, and the #3203 evidence table. Determinism bar: bit-identical
final-state hashes on all guard scenes for default-on packets.

#### WP-PG.10 — LCP pipeline instrumentation and island census

- Status: done — #3339 (`wp-pg-10-lcp-profile-census`)
- Objective: add missing `DART_PROFILE_*` scopes so assembly vs solve vs
  build split is measurable per island, and emit an island-size histogram
  (bodies, contacts, LCP rows) from `contact_benchmark --profile` runs.
- Value: converts the 66% smoke number into per-stage, per-island-size
  evidence that sizes WP-PG.12/13/14 and arbitrates D3.
- Scope: profile scopes in `ConstraintSolver.cpp` /
  `BoxedLcpConstraintSolver.cpp`; a stats counter surfaced through the
  existing profiler dump; baseline histograms recorded in
  01-baseline-evidence.md.
- Non-goals: any behavior change; solver API or physics semantics changes.
- Acceptance evidence: histogram + stage-share tables for the five guard
  scenes; zero hash drift on all detectors.
- Dependencies: WP-PG.01.
- Local evidence (2026-07-07, current base
  `origin/release-6.20` @ `b78a8b8cbe7`): added fixed-label text-profiler
  counters for constrained-group island census plus solver scopes for
  clear/update/build/solve, boxed-LCP term construction, primary solve,
  fallback solve, and impulse application. Profile artifacts:
  `/tmp/wp_pg10_profile_20260707T132241`.

  | Row | Groups | LCP rows | Max rows | Build share | Group-solve share | Construct share | Primary-solve share | Hash |
  | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
  | S1 dart, 120 container | 1 | 816 | 816 | 0.28% | 95.9% | 5.69% | 90.0% | `0x2757590b13e917ee` |
  | S2 dart, settled 3k | 0 final (initial warm solve rows only) | 0 final | 0 final | 2.43% | 9.27% | 2.12% | 1.96% | `0x8ddc9a81f2d28a7f` |
  | S3 dart, active 3k | 3003 | 15015 | 9 | 5.44% | 3.97% | 0.89% | 1.02% | `0xcf0ba6eaa97be038` |
  | S4 dart, generated 900 | 900 | 5400 | 12 | 6.30% | 3.63% | 0.26% | 0.21% | `0x76205ad68f4293bb` |
  | S5 dart, serial 90 | 90 | 540 | 12 | 6.10% | 10.3% | 3.91% | 2.91% | `0x726d1ff51bdb717` |

  The solver scopes/counters use runtime-gated profiling so ordinary
  `World::step()` execution stays allocation-neutral even when DART is built
  with `DART_BUILD_PROFILE=ON`; `contact_benchmark --profile` enables the gate
  around the measured run. Final profile smoke:
  `/tmp/wp_pg10_profile_smoke_final.txt` (`--generate-objects 30`, 20 steps,
  DART collision) printed the solver construct scope and island counters with
  final hash `0x31cf9caf33b82e3c`.

  Guard artifact `/tmp/wp_pg10_guard_20260707T132321` matched current-base
  parent and WP-PG.10 for S1 120-object dart/ode rows and S2-S5 all-detector
  rows. The old baseline table's S4/S5 FCL hashes had already drifted on the
  unmodified parent (`S4_fcl = 0xea9b68f8b062600d`, `S5_fcl =
  0x8277be4f0c14212`); the WP-PG.10 branch reproduced the same parent values.
  Final local verification passed `UNIT_common_Profile`,
  `INTEGRATION_StepAllocation` (native DART measured allocation gates reported
  zero operator-new/raw-malloc/base-allocator growth), capped `ALL`, and
  `DART_PARALLEL_JOBS=8 pixi run -e gazebo test-gz`.

#### WP-PG.11 — Remove per-step RTTI and full scans from the solver

- Status: evidence-gated (current-base rejected)
- Objective: replace per-step `dynamic_cast`/type-check classification
  with cached type tags/flags where it actually runs every step: (1) the
  resting-group `dynamic_cast<ContactConstraint>` classification under
  deactivation (`ConstraintSolver.cpp:2052` — note this win applies to
  deactivation-ON scenes: S2/S4/S5 rows, not the disable-deactivation
  rows); (2) the per-active-contact type checks in the build pass
  (`ConstraintSolver.cpp:1359-1378`). De-prioritized: the
  `hasCustomContactConstraint` (`:2177`) / `hasSharedNonReactiveDependency`
  (`:2192`) lambdas run only inside `canSolveGroupsInParallel()`
  (`:2339-2344`), which is gated behind the opt-in
  `>=128-group` threaded path — cache their facts only if a threads-on
  packet shows them in profiles.
- Value: O(constraints) per-step overhead removed on the default serial
  path; near-zero behavior risk; default-on.
- Prior art to mine (triaged, unmerged, hash-preserving): the unique
  commits on `origin/perf/dart6-single-reactive-raw-root` (drop the
  `mRootSkeleton` shared_ptr churn on the all-single-reactive path —
  measured RTF 0.245 → 0.272, `buildConstrainedGroups` 67.9 → 40.8 ms)
  and `origin/perf/dart6-single-reactive-union-reset` (stamped-index
  union reset, ~4x scope reduction). Re-measure both on the round-2
  baseline as part of this packet.
- Local rejection note (2026-07-06): cpp-only mining of those two
  single-reactive commits on `wp-pg-11-solver-rtti-scans` was reverted
  after current-base A/B on `origin/release-6.20` @ `2e11928288c`
  regressed the median on S2 ODE 3k settled (0.87x), S4 generated-900
  DART (0.93x), S4 Bullet (0.99x), and S4 ODE (0.98x). FCL and active S1
  ODE improved, but not enough for the general-performance bar; all guards
  were identical. Artifacts: `/tmp/wp_pg11_ab/current_2e119_repeat`.
- Scope: `dart/constraint/*` cpp + minimal additive header changes
  (virtual additions only on classes gz does not subclass — verify
  `ConstraintBase` is not part of the frozen-vtable set before landing;
  if it is, use a non-virtual flag member instead).
- Non-goals: changing island composition or solve order.
- Assumptions: `ConstraintBase` vtable is not subclassed by gz-physics
  (gz adds `WeldJointConstraint` instances but does not subclass
  ConstraintBase itself — re-verify in `.deps/gz-physics`).
- Acceptance evidence: identical hashes/contacts on all guard scenes and
  detectors; measurable step-time delta on 900/3000-object scenes.
- Dependencies: WP-PG.01.

#### WP-PG.12 — Generalized direct LCP assembly for single-free-body islands

- Status: deprioritized by WP-PG.01 evidence — on the dense-pile fixture
  `Construct LCP` is only 7.4% of step time (solve-proper is 88.1%), and
  one-body islands already use #3142's direct path; claim only if
  WP-PG.10's census finds a scene class where assembly dominates.
- Objective: extend the #3142 direct-assembly path
  (`BoxedLcpConstraintSolver.cpp:272-383`) to islands whose members are
  all 6-DoF single-free-body skeletons: build A blockwise from cached
  spatial Jacobians and per-body 6x6 inverse spatial inertias instead of
  O(rows) `applyUnitImpulse` ABI passes.
- Value: containers/piles of free bodies (the gz 3k case and the #3209
  scene) are exactly this shape; removes the dominant O(n^2) ABI-pass
  cost in assembly.
- Scope: `dart/constraint/BoxedLcpConstraintSolver.*` (cpp-first; scratch
  behind existing solver internals); classification reuses WP-PG.30's
  cached single-free-body facts if available, else local. Unlike #3142's
  single-reactive shape, container piles have body–body contacts: the
  blockwise assembly must fill the off-diagonal coupling blocks between
  two free bodies — that generalization is the core of this packet.
- Non-goals: multi-DOF articulated islands; solver algorithm changes.
- Assumptions/open decisions: direct assembly changes FP rounding vs
  unit-impulse tests; #3142 precedent shows identical hashes are
  achievable on guarded scenes — target bit-identical, escalate to D1
  process if not achievable.
- Acceptance evidence: full #3203 matrix; hash-identical on guard scenes
  (or maintainer-approved re-baseline with rationale); ≥ measurable
  active-scene RTF win at 120/900 objects.
- Dependencies: WP-PG.01, WP-PG.10; benefits from WP-PG.30.

#### WP-PG.13 — Row-island decomposition within constrained groups

- Status: evidence-gated on WP-PG.10
- Objective: decompose each assembled Delassus system into independent
  row-islands (adjacency-based DFS, not dense column scans) and solve
  each sub-LCP separately; optionally solve sub-LCPs in parallel behind
  the existing opt-in thread pool.
- Premise check (from review): `ContactConstraint::uniteSkeletons` skips
  non-reactive bodies, so 6.20 constrained groups are already
  contact-graph connected components — a 120-body pile is one component
  and this packet would no-op there. It pays off only if WP-PG.10's
  island census shows groups **coarser** than contact connectivity
  (e.g. via joint constraints or shared reactive bodies). Claim only
  with that evidence.
- Value: where groups are coarser than connectivity, smaller dense solves
  are super-linearly cheaper.
- Scope: `BoxedLcpConstraintSolver.cpp`; transplant the *idea* of main's
  `unified_constraint.cpp:302-357/890-966` but with an adjacency
  structure (main's own dense-scan DFS is O(rows^2) — do not copy).
- Non-goals: changing Dantzig/PGS internals; default-on parallelism.
- Acceptance evidence: identical hashes serial; documented determinism
  for the opt-in parallel path (fixed sub-island ordering, per-island
  scratch); island-census-backed before/after from WP-PG.10.
- Dependencies: WP-PG.10; independent of WP-PG.12.

#### WP-PG.14 — Island-size-gated matrix-free solve path (gated, opt-in)

- Status: blocked on D3
- Objective: for islands above a size threshold, offer a matrix-free
  sequential-impulse/PGS path over cached Jacobians (no dense (3n)^2 A),
  as an opt-in `BoxedLcpConstraintSolver` option defaulting to current
  behavior.
- Value: removes O(n^2) memory and O(n^2..3) time for very large
  islands; the only lever that changes the asymptote.
- Scope: new solver option + implementation; docs; dartpy binding only if
  trivially additive.
- Non-goals: changing the default solve; friction-cone semantics changes
  silently (must be documented as solver-option semantics).
- Assumptions/open decisions: D3 (opt-in confirmed?); solution
  non-uniqueness vs Dantzig means hashes differ by construction when the
  option is enabled — guard scenes run with the option OFF must stay
  bit-identical.
- Acceptance evidence: option-off = bit-identical everywhere; option-on
  evidence table with contacts/resting/finite checks + convergence
  metrics at 900/3000 objects.
- Dependencies: D3 (note its revisit trigger in the README: if WP-PG.10
  confirms solve-proper dominance on single-connected islands, opt-in
  leaves the primary fixture without a default-on solve remedy — D3 gets
  re-decided with that evidence), WP-PG.10, WP-PG.13 insights.

#### WP-PG.15 — Penetration creep vs island-rest veto (root cause)

- Status: claimed on #3353 as the D7 default-remediation PR; local evaluator
  evidence passes, hosted CI/review and full closeout gates still pending. This
  is a behavior-changing PR class and needs explicit old/new evidence rather
  than hash-identical guard acceptance.
- Objective: fix #3209 root-cause finding 2 so compacting piles stop
  creeping and can sleep: investigate the interaction of the contact
  error-reduction budget (baseline `DART_MAX_ERV = 1e-3` m/s,
  `ContactConstraint.cpp:49`; public setter
  `setMaxErrorReductionVelocity` exists), large-island LCP convergence,
  and the island-rest veto requiring every contact ≤ 1e-5 m penetration at
  baseline
  (`kSleepContactPenetrationTolerance`, `ConstraintSolver.cpp:2050`,
  `World.cpp:1229`). Candidate remedies (per D7): penetration-aware ERV
  scaling under pile load, convergence budget for large islands, and/or a
  bounded-penetration rest tolerance with a stability rationale.
- Value: the largest *general-case* lever in the plan — in real gz scenes
  (deactivation on), creeping piles currently stay active forever; fixing
  this converts sustained active-regime cost into settled-regime cost and
  is the gz-visible closer for reopened #3056 (success criterion 2).
- Scope: `dart/constraint/ContactConstraint.*`,
  `ConstraintSolver.cpp` / `World.cpp` rest-veto sites; behavior-changing
  — carries tolerance rationale, old/new guard rows for ALL detectors,
  and maintainer re-baseline sign-off per envelope rule 2.
- Current #3353 implementation: promotes the D7 evaluator evidence into the
  default policy by raising the static contact ERV default from `0.001` to
  `0.1`, but applies that higher effective ERV only to dense islands with
  mobile-mobile contacts. Single-mobile static-support islands keep the legacy
  effective cap (`0.001`) unless the public ERV setter is used. The automatic
  sleeping contact tolerance also remains externally defaulted to `1e-5`, while
  dense contact islands use `0.005` internally for the solver rest-veto gate
  under the default policy. Analytic `PlaneShape` support contacts use the same
  bounded contact-rest tolerance after the #3355 base merge exposed Bullet
  plane contacts converging at millimeter scale while ordinary box/sphere/
  capsule static-support islands remain on the strict legacy tolerance. The
  process-wide tolerance setter is tracked separately from the numeric value, so
  explicitly setting `1e-5` still reproduces the strict legacy rest-veto policy.
  The patch also adds a dense-contact-island sleep-candidate path when every
  mobile member is below the wake band and at least one member has sustained
  dwell evidence. `contact_benchmark` keeps the ERV/tolerance CLI overrides so
  old-default A/B rows remain reproducible, and now reports island and
  dwell/velocity diagnostics needed to explain why a pile does or does not
  sleep.
- Local D7 evaluator evidence (2026-07-08, branch
  `docs/close-dart6-performance-generalization`, binary
  `build/default/cpp/Release/bin/contact_benchmark`; full logs:
  `/tmp/wp_pg15_ab_erv_reset_20260709T012537Z/summary.tsv`):

  | Row | Wall time | RTF | Contacts / pairs | Over sleep tol | Max penetration | Resting | Hash |
  | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
  | S6 current defaults | 94.1562 s | 0.212413 | 0 / 0 | 0 | 0 | 71/71 | `0xec80f734df6d5e74` |
  | S6 old-default override (`ERV=0.001`, tol `1e-5`) | 197.777 s | 0.101124 | 162 / 141 | 39 | 0.364241 | 0/71 | `0x159825257114c5d5` |
  | S6 explicit evaluator (`ERV=0.1`, tol `0.005`) | 73.1579 s | 0.273381 | 0 / 0 | 0 | 0 | 71/71 | `0x877687e64e1011b9` |

  The default row is 2.10x faster than the old-default override on this run
  and reaches the WP-PG.15 pile-sleep outcome under default settings. The
  explicit evaluator row is retained as the upper-bound comparison; the broader
  global policy was rejected after `Issue1445` and split-impulse guards exposed
  simple-contact regressions. S4/S5 detector guard rows in the same artifact
  stayed finite across DART, FCL, Bullet, and ODE, and every new-default S4/S5
  row matched the old-default hash/contact/resting state.

  Headless and visual closeout artifacts:
  S2 DART 3k-shapes guard
  `/tmp/wp_pg15_examples_20260708T223506Z/S2_dart_3k_shapes.log`
  (`RTF 29.7321`, `3003/3003` resting, hash
  `0x8ddc9a81f2d28a7f`); S6 final-scene dump
  `/tmp/wp_pg15_visual_20260708T223506Z/S6_final_scene.jsonl`; S6 GUI capture
  `/tmp/wp_pg15_gui_20260708T223653Z/S6_gui.png` with a passing non-blank
  `image-verdict`.
- Non-goals: solver algorithm swaps (that is WP-PG.14); changing gz-visible
  default semantics beyond the approved D7 envelope.
- Acceptance evidence: S6 reproducer ends with bounded `max_penetration`
  and 71/71 resting under default settings; S2 settled guard stays
  resting-complete; physical-plausibility note (no pile explosion,
  finite states) on all guard scenes; gz gate green.
- Dependencies: D7; WP-PG.01 (S6 baseline rows); informed by WP-PG.10's
  convergence data.
