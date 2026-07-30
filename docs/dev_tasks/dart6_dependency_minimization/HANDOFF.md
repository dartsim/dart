# HANDOFF — DART 6.20 dependency minimization (native collision port)

> Session handoff, refreshed 2026-07-29. Read [README.md](README.md) (overall
> SSOT), then [RESUME.md](RESUME.md) (exact next step), then
> [08-phase5-facade-decision.md](08-phase5-facade-decision.md) (the unresolved
> proposal). Use [03-native-collision-port-scoping.md](03-native-collision-port-scoping.md)
> for acceptance gates and historical evidence; the phase-2 execution documents
> are closed history, not current instructions. This file records what merged,
> what remains, and the portable operational lessons.
>
> **Current reality (2026-07-29):** PR #3381 is merged. Final head
> `64d476b68ad5ae0dcca4e98abb9bba15b6962b87` became release merge
> `46719bfbd75e1f70e69b2c76fb34a3fa2b78edd5`. The native engine is now part of
> `DARTCollisionDetector`; no publication, review, CI-monitoring, or merge work
> remains for that PR. The next action is later-release Phase 5/6/7 planning
> and maintainer ratification, not more DART 6.20 consolidation work.
>
> **Update 2026-07-23:** the phase-6 default flip described above was
> **reverted** per maintainer direction. PR #3381 ultimately shipped the
> detector consolidation only; the built-in default remains **`fcl`**,
> and the flip is deferred to a later PR. See RESUME.md "Update
> 2026-07-23" for details.

## 1. The mission in one paragraph

DART 6.20 dependency minimization removed every standalone-removable dependency
and merged the C++17 DART-owned collision engine into `DARTCollisionDetector`.
The remaining project is a later-release compatibility decision: ratify the
facade/default-flip/dependency-removal sequence that could eventually make
**FCL/Bullet/ODE optional**. DART 6.20 is closed to that implementation:
**FCL remains its default**, and any future phase must start from an approved
release and pass its own gz-gated acceptance packet.

## 2. Phase status (plan of record: `03`, phases 0–7)

| Phase | What | Status |
| --- | --- | --- |
| 0 | Baseline evidence packet (incumbent A/B envelope + default-flip verdict) | ✅ merged (#3271) |
| 1 | Native math core (Aabb/Gjk/Mpr/BoxBox/SphereSphere/shapes/Span shim) internal-only | ✅ merged (#3281) |
| 2 | DART 6 detector adapter over the native core (bridge, sliced P1–P9 + P10 coverage) | ✅ complete (#3350) |
| 3 | Capability parity (distance→FCL, raycast→Bullet, CCD, manifolds, voxel) | ✅ complete (#3360) |
| 4 | Evidence-driven performance optimization (broadphase, SIMD, manifold reuse) | ✅ closed for this lane — #3364 and AABB-tree broadphase #3368 merged; remaining general performance work belongs to `dart6_performance_generalization` |
| 5 | Bullet/ODE/FCL facade decision (must keep gz subclassing) | 🔄 **active** — decision doc drafted with maintainer direction; ODE facade-vs-coordinated-gz change remains to ratify |
| 6 | Default flip in both `ConstraintSolver` ctors (point of no return) | ⏸ deferred beyond 6.20 |
| 7 | FCL decoupling from core (the dependency win) + retire this task folder | ⬜ not started |

## 3. What merged this session (native-collision-port lane)

- **#3271** phase-0 baseline packet (`05-phase0-baseline-packet.md`,
  `05-artifacts.md`). Verdict: native default NOT allowed at that tip (no engine
  yet). Carries the phase-6 acceptance envelope + guard-row hashes.
- **#3281** phase-1 native math core → `dart/collision/native/` (internal-only,
  FCL default). Includes `06-phase1-port-packet.md`.
- **#3298** fix: `maxNumContacts == 0` short-circuit in native sphere-sphere
  (release-6.20). Its main-branch dual **#3283** is also merged.
- **#3302** phase-2 execution plan (`07-phase2-adapter-scoping.md`) + this RESUME.
- **#3303** phase-2 **P1**: native BruteForce broadphase (internal-only).
- **#3306** phase-2 **P2**: narrowphase dispatcher (sphere/box only) →
  `dart/collision/native/narrow_phase/NarrowPhase.{hpp,cpp}`.
- **#3318** phase-2 **P3a**: adapter skeleton + sphere/box conversion,
  intentionally unregistered. The `"native"` factory key still does not exist
  until P3b.
- **#3319** phase-2 **P3b**: bridge translation, `"native"` factory
  registration, `sphere_box`, and native-vs-fcl/dart parity.
- **#3321** phase-2 **P4**: capsule primitive pairs.
- **#3322** phase-2 **P5**: convex foundation and capsule-capsule.
- **#3324** phase-2 **P6**: cylinder collision pairs.
- **#3325** phase-2 **P7**: mesh collision pairs.
- **#3343** phase-2 **P8/P9**: distance module and plane primitive/convex
  coverage.
- **#3350** phase-2 **P10**: mixed-scene FCL/DART/native parity coverage.
- **#3352** phase-3 **D1**: native detector distance adapter, DART 6-style
  native basename normalization, and native-vs-FCL distance benchmarks.
- **#3355** phase-3 **D2**: native detector raycast adapter and native-vs-Bullet
  raycast benchmarks.
- **#3358** phase-3 **D3**: native `VoxelGridShape`/octree replacement via
  compound voxel boxes, plus compound collision/distance/raycast routing.
- **#3359** phase-3 **D4**: native CCD engine support for rigid sphere/capsule
  casts, primitive point-triangle/edge-edge CCD, shape+transform dispatcher
  entry points, and compound-target earliest-hit selection.
- **#3360** phase-3 **D5**: native persistent manifold cache, contact reduction,
  cache refresh/removal, and native cached-impulse seed/write-back plumbing.
- **#3362** phase-4 benchmark surfacing: native rows in the contact-container
  dashboard.
- **#3364** phase-4 solver-facing manifold optimization: capped the then-opt-in native
  detector manifolds at three contacts while honoring stricter per-pair caps.

Current `origin/release-6.20` tip at this owner refresh: `ac7b9462612` (#3406,
after #3381). Do not start new 6.20 implementation from this snapshot.

## 4. Consolidation complete; later-release phases remain

Historical implementation lineage (see `07` §0–§1): the port bypassed DART 7's
EnTT `CollisionWorld`; the interim `NativeCollisionDetector/Group/Object`
adapter reused DART 6's `shared_ptr`-based `CollisionObjectManager`. Those
interim names/selector were later folded into `DARTCollisionDetector` by #3381.

| Slice | Scope | Status |
| --- | --- | --- |
| **P1** | BroadPhase base + BruteForce (pure engine) | ✅ **merged (#3303)** |
| **P2** | Narrowphase dispatcher, sphere/box only (bespoke §2.1 trim) | ✅ **merged (#3306)** |
| **P3a** | Adapter skeleton + `NativeShapeConversion`(sphere,box), intentionally **unregistered**; `collide()` is a documented **stub** | ✅ **merged (#3318)** |
| **P3b** | Bridge `collide()` translation + `"native"` factory registration + `sphere_box` collider + normal calibration (R1) + parity vs **fcl and dart** | ✅ **merged (#3319)** |
| **P4** | `capsule_sphere`, `capsule_box` (no-span primitives) | ✅ **merged (#3321)** |
| **P5** | `convex_convex` (keystone) + `capsule_capsule` (first span pair) | ✅ **merged (#3322)** |
| **P6** | `cylinder_collision` (needs convex_convex) | ✅ **merged (#3324)** |
| **P7** | `mesh_mesh` (needs convex_convex; largest file) | ✅ **merged (#3325)** |
| **P8** | `distance` module (engine-only; needs span shim) | ✅ **merged (#3343)**, combined with P9 |
| **P9** | `plane_sphere` (needs `distance`) → completes primitive+convex+mesh+plane | ✅ **merged (#3343)**, combined with P8 |
| **P10** | mixed-scene fcl/dart/native parity integration test | ✅ **merged (#3350)** |
| **D1** | DART 6 `NativeCollisionDetector::distance()` adapter + native basename normalization | ✅ **merged (#3352)** |
| **D2** | DART 6 `NativeCollisionDetector::raycast()` adapter + native-vs-Bullet raycast benchmarks | ✅ **merged (#3355)** |
| **D3** | `VoxelGridShape`/octree replacement via native compound boxes + compound collision/distance/raycast recursion | ✅ **merged (#3358)** |
| **D4** | Native CCD engine: rigid sphere/capsule casts, primitive point-triangle/edge-edge CCD, and shape+transform dispatcher support | ✅ **merged (#3359)** |
| **D5** | Persistent manifold cache, manifold reduction/reuse, and solver cached-impulse seed/write-back | ✅ **merged (#3360)** |
| **P11** | Native contact-container dashboard rows | ✅ **merged (#3362)** |
| **D6** | Solver-facing native manifold cap with performance evidence | ✅ **merged (#3364)** |

### Historical Phase 4 closeout record (closed; do not resume)

The following #3364 evidence is retained to explain the accepted lineage. It is
not a current branch/PR recipe: Phase 4 is closed for this lane, and general
performance work belongs to `../dart6_performance_generalization/`.

| Benchmark row | Parent native | Slice native | Delta | Contacts |
| --- | ---: | ---: | ---: | ---: |
| `BM_ContactContainerActive/60/4/1_mean` | 202.383 ms | 202.918 ms | -0.3% | 84 -> 80 |
| `BM_ContactContainerActive/60/4/16_mean` | 204.547 ms | 203.067 ms | +0.7% | 84 -> 80 |
| `BM_ContactContainerActive/120/4/1_mean` | 2268.942 ms | 1118.032 ms | +50.7% | 282 -> 251 |
| `BM_ContactContainerActive/120/4/16_mean` | 2202.924 ms | 1124.906 ms | +48.9% | 282 -> 251 |
| `BM_ContactContainerActive/120/4/4_mean` | 2193.106 ms | 1167.577 ms | +46.8% | 282 -> 251 |
| `BM_ContactContainerDeactivation/60/4/16/iterations:1_mean` | 30.589 ms | 29.391 ms | +3.9% | 101 -> 97 |

Post-#3364 detector comparison on
`BM_ContactContainerActive/120/*/1_mean`: native 1118.032 ms / 251 contacts,
DART 1452.013 ms / 242 contacts, FCL 1475.995 ms / 243 contacts, Bullet
1544.000 ms / 256 contacts.

Local gates captured for #3364 were `pixi run check-lint`, `pixi run build`,
Release `ctest -R 'UNIT_collision_native|test_ConstraintSolver$'`, Debug
`UNIT_collision_native_detector_adapter`, the portability scan, and the full
Gazebo gate. Historical candidate optimization areas included data layout,
scratch caches, broadphase pair pruning, manifold reuse, scene-local allocation
control, optional SIMD, and thread-safe contact aggregation.

The current next action is Phase 5 maintainer ratification. Keep FCL as the
default until an approved later-release Phase 6; do not touch `World`,
`ConstraintSolver` defaults, `WorldConfig`, or dependency/package metadata
before that phase. Any later collision-performance or default-change PR must
recapture Release and Debug tests, lint, contact-rich A/B evidence, the Gazebo
gate, portability, and public `Contact` layout compatibility on its own current
parent.

## 5. Merged closeout / loose ends

- **PR #3381 detector consolidation — MERGED.** The following review trail is
  historical and must not be resumed. The PR removed the unreleased
  `"native"` selector, preserved the released `DARTCollide` header/symbols via
  thin wrappers, addressed all current-head Codex findings, proved FCL remains
  the default, ran the no-cache C++/Python/gz and installed-prefix gates, then
  retriggered review on the pushed exact head. The PR does not authorize a
  default flip or dependency removal. Its CI evidence included an explicit
  `DARTCollisionDetector` capture of ellipsoid/capsule/cone contacts. Soft-body
  deformation/contact parity and raycast hit/sort/filter behavior stay
  text-first because pixels do not prove those numerical contracts.
  The 2026-07-29 local review-fix candidate is green on 155/155 C++ tests,
  223/223 dartpy tests, `check-lint`, the complete AI aggregate (357 focused
  tests plus seven scenarios), and the pinned Gazebo aggregate (199
  gz-physics tests + four performance/symbol checks + one gz-sim integration
  test). Mixed-header ABI canaries preserve the pre-consolidation 6.20
  detector/group/object sizes (`32/376/320`) through 20 guarded lifecycle
  runs, and the released 6.19.4 detector header passes 20 runs. Two alternating
  five-repetition incumbent-backend A/Bs show no regression: ODE base/head
  medians `868/867-868 ms`, FCL `257/255-256 ms`, Bullet `267/266 ms`, with
  identical contact counts. Local visual smoke and all four rendered-overlay
  regressions pass. This evidence predated publication; exact-head hosted CI,
  artifact inspection, and fresh Codex review were still mandatory at that
  point and led to the later review iterations below. The review of pushed head
  `fd96521bb14` then found that new out-of-line group/object
  destructors cannot clean sidecars for binaries built with the old inline
  destructors. The correction ties sidecar ownership to the existing
  detector/cached-shape shared-pointer members, restores defaulted inline
  destructors, and adds a direct lifetime regression. It was published as
  `af2ac200e26`; the review thread is resolved. Its `32/376/320` mixed-header
  canaries pass 20/20 and the private test accessor is absent from the install.
  Fresh Codex review `4814689190` of `af2ac200e26` found a separate race in the
  four one-time shape-warning sets. The implementation-only correction
  synchronizes every warning category behind one registry mutex without
  locking supported-shape paths and adds an eight-thread regression covering
  all categories. The full 43-test engine target passes in Release and
  assertions-enabled builds. It was published as `2f2d45d99da`, and its thread
  is resolved. Fresh Codex review `4814842827` of that head found the
  single-thread two-group path missing the parallel path's AABB rejection. The
  final correction rejects disjoint pairs before filters and narrowphase in
  both paths, matching the released ordering; its regression proves eight
  isolated cross-group overlaps visit eight candidates rather than all 64
  Cartesian pairs. The focused detector target passes in Release and
  assertions-enabled builds. Fresh no-cache gates pass all 155 C++ tests, all
  223 dartpy tests, lint/check-lint, all 199 gz-physics tests, all four
  gz-physics performance/symbol checks, and the gz-sim entity-system
  integration test. The correction was included in final head
  `64d476b68ad5ae0dcca4e98abb9bba15b6962b87`, merged as
  `46719bfbd75e1f70e69b2c76fb34a3fa2b78edd5`; no exact-head publication,
  review, CI-monitoring, or merge action remains.
- **Phase 4 native performance** — #3362, #3364, and #3368 are merged and this
  lane is closed. General remaining performance work is owned by
  `../dart6_performance_generalization/`, not this handoff.
- **#3353** is merged on `release-6.20` for the separate
  performance-generalization plan parking lane.
- **#3357** is merged; it renamed the DART 6 autonomous AI workflow to
  `dart-ultrawork` and is unrelated to this native-collision code slice.
- **#3283 (main sphere-sphere `enableContact` fix)** and **#3348** (MSVC
  toolchain policy) are MERGED.

## 6. Load-bearing invariants (do not violate)

- **FCL stays the default throughout DART 6.20.** Never touch `WorldConfig`,
  `ConstraintSolver`'s two FCL-hardcoded ctors, `World::toCollisionDetectorKey`,
  or add a `CollisionDetectorType::Native` enum. Explicit consolidated-engine
  selection uses the canonical `"dart"` factory key only; `"native"` is not an
  alias. There is no `World::setCollisionDetector(const char*)` overload.
- **No EnTT, no new dependency, C++17 only.** The internal namespace may remain
  `dart::collision::native`, but its sources live under `dart/collision/dart/`.
  The C++17 `detail/Span.hpp` shim replaces `std::span`.
- **gz gate every PR** (`pixi run -e gazebo test-gz`); on engine-only slices it
  is a non-regression guard (trivially green), substantive from P3b onward.
- **`NarrowPhase.{hpp,cpp}` is a bespoke reduced dispatcher** (`07` §2.1): it is
  hand-trimmed and re-expands capability-by-capability. Each dispatcher-touching
  PR must keep the `git diff origin/main` explainable: span shim, dropped
  `CollisionObject` overloads/world-layer branches, DART 6 compile fixes, and
  only the current routed capability deltas.
- **Prefer fewer PRs for remaining phases.** Combine cohesive capability wiring,
  parity coverage, and mechanical native-file cleanup in one PR when the local
  validation envelope remains clear; split only when review risk or ownership
  boundaries require it.
- **Prefer clean, long-term interfaces.** Do not retain unreleased aliases.
  Preserve shipped DART 6 source/link surfaces, such as `DARTCollide`, with
  thin adapters when necessary, and prove downstream compatibility through
  installed-prefix builds plus the gz gate.
- **Lazy geometry refresh** now lives in
  `DARTCollisionObject::updateEngineData` and its shape cache. It must keep
  detecting **shape identity changes + null**, not version alone (a fresh shape
  starts at version 1, so a version-only guard misses a swap). `07` §1.4
  records the historical pre-consolidation implementation.

## 7. Portable operational lessons

- **Worktrees and branches:** do not rely on a developer-specific path or an old
  phase worktree. Before use, verify the exact branch, dirty state, upstream,
  writability, and live PR/remote state. Create future work from the approved
  target release rather than resurrecting a historical branch.
- **Hooks and lint:** run mandatory `pixi run lint` before every commit and let
  the repository pre-commit hook run. `DART_SKIP_HOOKS=1` is only the emergency
  escape hatch documented in `AGENTS.md`, never the routine path.
- **Moving bases:** fetch and re-check the approved target tip before every
  capture, branch, or push. Merge the latest base into a published PR branch
  before pushing; never rebase it.
- **Branch test gates:** `pixi run test-all` is the Release-only default CMake
  aggregate. The branch config pins `BUILD_TESTING=ON`, so its `ALL` graph
  builds the defaults and runs both CTest and pytest. Run `pixi run lint`
  separately; use `pixi run test` or `pixi run test-py` for focused reruns and
  clearer attribution. Diagnose cache failures from the exact DART 6 pybind11
  configuration rather than importing main-only dependency guidance.
- **Sphere-fix lesson (why #3283 was reverted):** the `enableContact` binary
  check must use the **squared** overlap predicate `dx²+dy²+dz² <= (r1+r2)²`,
  identical to the contact path's generic branch, so binary and contact agree.
  A `std::hypot` "fix" for a P3 overflow at non-physical ~1e154 magnitudes
  introduced a **worse** real-scale 1-ULP disagreement and was withdrawn
  (#3305 closed). The squared form's overflow at ~1e154 is a non-issue and
  matches the contact path's own behavior.

## 8. Worktree and branch cleanup

- No historical Phase 0–4 or #3381 worktree is a resume target. Treat it as a
  cleanup candidate only after live `git status`, `git branch -vv`, merge/tree
  comparison, and PR-state checks prove it is clean and merged; deletion remains
  an explicit maintainer action.

## 9. Evidence artifacts (for phase 6)

The phase-0 baseline packet (`05-phase0-baseline-packet.md`) and committed
appendix (`05-artifacts.md`) are the durable record of incumbent guard rows,
tolerances, and JSONL SHA-256 digests. Treat all old git-ignored/local dumps as
unavailable unless their bytes are independently recovered and match those
digests. A future phase-6 proposal should normally recapture on its own parent
and compare within that same packet. The committed capture driver and analyzer
are checkout-relative and portable.

## 10. Coordination

The perf-generalization lane (`../dart6_performance_generalization/`) tracks this
port as **WS-F (external owner)**; its WP-PG.42 (SoA broadphase) gates on this
lane's phase status, its D8 (manifold reduction) re-cuts at this lane's phase 3,
and its WS-B depth re-reviews at this lane's phase 5. Do not pick up WP-PG.*
packets from this folder. All remote mutations (merges) are the maintainer's.
