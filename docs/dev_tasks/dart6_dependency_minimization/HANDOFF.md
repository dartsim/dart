# HANDOFF — DART 6.20 dependency minimization (native collision port)

> Session handoff, refreshed 2026-07-09. Read [README.md](README.md) (overall SSOT),
> then [03-native-collision-port-scoping.md](03-native-collision-port-scoping.md)
> (plan of record for the remaining work), then
> [07-phase2-adapter-scoping.md](07-phase2-adapter-scoping.md) (phase-2 execution
> plan), then [RESUME.md](RESUME.md) (exact next step). This file is the
> transfer note: what is merged, what is open, exactly where to resume, and the
> operational gotchas learned this session.

## 1. The mission in one paragraph

DART 6.20 dependency minimization. Every standalone-removable dependency is
already gone; the **only** remaining lever is the **native collision detector
port**: port DART 7's `dart/collision/native/` engine to DART 6.20 as C++17
(no EnTT, no new dependency), make it the default detector, and thereby make
**FCL/Bullet/ODE optional** — the final win (phase 7) is dropping FCL from the
core `dart` target and `DART_PKG_EXTERNAL_DEPS`. This is an **XL, multi-PR,
multi-week** initiative on a compatibility support branch; **FCL stays the
default detector until the phase-6 flip**, and every phase is gz-gated.

## 2. Phase status (plan of record: `03`, phases 0–7)

| Phase | What | Status |
| --- | --- | --- |
| 0 | Baseline evidence packet (incumbent A/B envelope + default-flip verdict) | ✅ merged (#3271) |
| 1 | Native math core (Aabb/Gjk/Mpr/BoxBox/SphereSphere/shapes/Span shim) internal-only | ✅ merged (#3281) |
| 2 | DART 6 detector adapter over the native core (bridge, sliced P1–P9 + P10 coverage) | ✅ complete (#3350) |
| 3 | Capability parity (distance→FCL, raycast→Bullet, CCD, manifolds, voxel) | ✅ complete (#3360) |
| 4 | Evidence-driven performance optimization (broadphase, SIMD, manifold reuse) | 🔄 **active** — #3364 merged; AABB-tree broadphase PR #3368 open (bit-identical guards, S3 −65%); remaining: S3 narrowphase delta, small-scene overhead, S6 resting profile (see RESUME) |
| 5 | Bullet/ODE/FCL facade decision (must keep gz subclassing) | ⬜ not started |
| 6 | Default flip in both `ConstraintSolver` ctors (point of no return) | ⬜ not started |
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
- **#3364** phase-4 solver-facing manifold optimization: caps opt-in native
  detector manifolds at three contacts while honoring stricter per-pair caps.

`origin/release-6.20` tip at this refresh: `613241385ae` (moves as the
maintainer merges; **always re-fetch before branching/capturing**).

## 4. Phases 2 and 3 complete; Phase 4 performance active

Bridge design (see `07` §0–§1): **bypass DART 7's EnTT `CollisionWorld`**; the
adapter (`NativeCollisionDetector/Group/Object` + `NativeShapeConversion`)
reuses DART 6's existing `shared_ptr`-based `CollisionObjectManager`, driving
**BruteForce broadphase → narrowphase dispatcher → DART 6 `Contact`**.

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

### Exact next step: Phase 4 closeout or one consolidated follow-up

1. Start a fresh branch from current `origin/release-6.20`; #3364 is already
   merged. Do not continue from `feature/native-phase4-solver-manifolds`.
2. Re-run the contact-container dashboard and focused `contact_benchmark
   --profile` rows on the latest base. If native still clears the Phase 4 bar,
   record a Phase 4 closeout packet and move to Phase 5. If a measured hot path
   remains, make **one cohesive** optimization PR with full evidence rather than
   several small CI-expensive PRs.
3. #3364 benchmark evidence to carry forward:

   | Benchmark row | Parent native | Slice native | Delta | Contacts |
   | --- | ---: | ---: | ---: | ---: |
   | `BM_ContactContainerActive/60/4/1_mean` | 202.383 ms | 202.918 ms | -0.3% | 84 -> 80 |
   | `BM_ContactContainerActive/60/4/16_mean` | 204.547 ms | 203.067 ms | +0.7% | 84 -> 80 |
   | `BM_ContactContainerActive/120/4/1_mean` | 2268.942 ms | 1118.032 ms | +50.7% | 282 -> 251 |
   | `BM_ContactContainerActive/120/4/16_mean` | 2202.924 ms | 1124.906 ms | +48.9% | 282 -> 251 |
   | `BM_ContactContainerActive/120/4/4_mean` | 2193.106 ms | 1167.577 ms | +46.8% | 282 -> 251 |
   | `BM_ContactContainerDeactivation/60/4/16/iterations:1_mean` | 30.589 ms | 29.391 ms | +3.9% | 101 -> 97 |

4. Post-#3364 detector comparison on `BM_ContactContainerActive/120/*/1_mean`:
   native 1118.032 ms / 251 contacts, DART 1452.013 ms / 242 contacts, FCL
   1475.995 ms / 243 contacts, Bullet 1544.000 ms / 256 contacts.
5. Local gates captured for #3364:
   `pixi run check-lint`, `pixi run build`, Release
   `ctest -R 'UNIT_collision_native|test_ConstraintSolver$'`, Debug
   `UNIT_collision_native_detector_adapter`, `rg 'entt|std::span'
   dart/collision/native tests/unit/collision/test_NativeCollisionDetector.cpp`,
   and `DART_PARALLEL_JOBS=8 CTEST_PARALLEL_LEVEL=8 pixi run -e gazebo test-gz`.
6. Candidate follow-up optimization areas are the `03` Phase 4 list: data
   layout, scratch caches, broadphase pair pruning, manifold reuse beyond #3364,
   scene-local allocation control, optional SIMD from #3229, and thread-safe
   contact aggregation. Do not mix in phase-5 facade decisions or phase-6
   defaults.
7. Phase 5 is next after Phase 4 closeout: decide whether Bullet/ODE/FCL stay
   real optional components or become facade-over-native compatibility
   components. Preserve gz subclassing, `find_package(DART COMPONENTS
   collision-bullet collision-ode ...)`, detector factory keys, and
   `SetWorldCollisionDetector("bullet"/"ode"/"fcl"/"dart")`.
8. Keep FCL as the default detector until Phase 6; do not touch `World`,
   `ConstraintSolver` detector defaults, `WorldConfig`, or dependency/package
   metadata before the phase that owns it.
9. Gates for **every** native-collision performance PR:
   Release build + **explicit Debug build** (`pixi run build` is Release-only);
   **run** the tests with `ctest --test-dir build/default/cpp/Release -R
   UNIT_collision_native --output-on-failure` (`pixi run test-all` only *builds*);
   full `pixi run check-lint`; contact-rich benchmark comparison evidence;
   `DART_PARALLEL_JOBS=8 pixi run -e gazebo test-gz` (199 + 4 + 1); portability
   `rg 'entt|std::span'` on new files → empty. Each performance PR must report
   parent-vs-PR and native-vs-incumbent rows in the style of #3307. Preserve the
   public `dart::collision::Contact` layout on this release branch.

## 5. Open PRs / loose ends

- **Phase 4 native performance** — #3362 and #3364 are merged. Next session
  should either close out Phase 4 with fresh evidence or produce one cohesive
  measured follow-up PR before Phase 5.
- **#3353** is merged on `release-6.20` for the separate
  performance-generalization plan parking lane.
- **#3357** is merged; it renamed the DART 6 autonomous AI workflow to
  `dart-ultrawork` and is unrelated to this native-collision code slice.
- **#3283 (main sphere-sphere `enableContact` fix)** and **#3348** (MSVC
  toolchain policy) are MERGED.

## 6. Load-bearing invariants (do not violate)

- **FCL stays the default** until phase 6. Never touch `WorldConfig`,
  `ConstraintSolver`'s two FCL-hardcoded ctors, `World::toCollisionDetectorKey`,
  or add a `CollisionDetectorType::Native` enum. Selection is the **factory
  pointer** only: `CollisionDetector::getFactory()->create("native")` (there is
  **no** `World::setCollisionDetector(const char*)` overload).
- **No EnTT, no new dependency, C++17 only.** `rg entt dart/collision/native`
  stays empty. Only C++20 feature in the ported tree is `std::span` → the
  `detail/Span.hpp` shim.
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
- **No header-compatibility shims as a default strategy.** Prefer clean,
  long-term interfaces. Preserve backward compatibility where gz-physics/gz-sim
  actually depend on it, and prove that compatibility through the gz gate and
  source-level subclassing/build checks.
- **Lazy geometry refresh** (P3b `NativeCollisionObject::updateEngineData`) must
  key on **shape identity + null**, not version alone (a fresh shape starts at
  version 1 → a version-only guard misses a swap). See `07` §1.4.

## 7. Operational gotchas / lessons (this session)

- **Codex worker + git worktrees:** the Codex sandbox only writes under
  `/home/js/dev/dartsim/dart/task_1` and `/tmp`. Worktrees for delegated work
  **must** live under `.claude/worktrees/` (inside task_1), not a sibling like
  `dart/worktrees/`. The worker also **cannot `git commit`** (worktree `.git`
  metadata is read-only in its sandbox) — it stages/writes files and the
  orchestrator commits. A fresh worktree has **no `.pixi` env**; the P2 worker
  symlinked `.pixi/envs/default` → the parent task_1 env to build (works,
  relocatable toolchain), or just let `pixi run config` solve it (~15–20 min).
- **Pre-commit hook is slow** (re-runs `check-lint-quick` = a full config): it
  can time out a 2-min `git commit`. Run `pixi run lint` (and `check-lint`)
  separately, then commit with `DART_SKIP_HOOKS=1` — lint is the real gate.
- **Base branch moves constantly** (maintainer merges the perf lane + others):
  re-`git fetch` and re-check the tip before every capture/branch/push; merge
  `origin/release-6.20` into a published PR branch before pushing, never rebase.
- **Work in the main clone when possible.** Old worktrees from early phase-2
  slices are stale; do not resume from them without checking branch, dirty state,
  and whether the remote head still exists.
- **`nanobind` Debug-config poison:** a `test-all` "Build Debug FAILED" is often
  a poisoned `CMakeCache.txt` (`nanobind_DIR-NOTFOUND`), not a code bug — `rm`
  the Debug cache + reconfigure.
- **Sphere-fix lesson (why #3283 was reverted):** the `enableContact` binary
  check must use the **squared** overlap predicate `dx²+dy²+dz² <= (r1+r2)²`,
  identical to the contact path's generic branch, so binary and contact agree.
  A `std::hypot` "fix" for a P3 overflow at non-physical ~1e154 magnitudes
  introduced a **worse** real-scale 1-ULP disagreement and was withdrawn
  (#3305 closed). The squared form's overflow at ~1e154 is a non-issue and
  matches the contact path's own behavior.

## 8. Worktrees at handoff (cleanup)

- Prefer `/home/js/dev/dartsim/dart/task_1` on `release-6.20` for the next
  native-collision session. Any old local worktree or stale remote branch must
  be treated as historical until live `git status`, `git branch -vv`, and
  `gh pr list --head <branch>` prove otherwise.

## 9. Evidence artifacts (for phase 6)

The phase-0 baseline packet (`05-phase0-baseline-packet.md`) + committed appendix
(`05-artifacts.md`) hold the incumbent guard rows, tolerances, and SHA-256
digests of the JSONL scene dumps. The full dumps are **local, git-ignored**
(`.omc/artifacts/native-collision-phase0*/`). For the phase-6 default-flip
tolerance gate: retrieve dumps matching those digests **or** recapture on the
flip PR's parent and compare within that same recapture (see `05` §"Verdict"
and RESUME "Standing constraints"). The capture driver + analyzer are embedded
in `05-artifacts.md` and are checkout-relative (runnable from any clone).

## 10. Coordination

The perf-generalization lane (`../dart6_performance_generalization/`) tracks this
port as **WS-F (external owner)**; its WP-PG.42 (SoA broadphase) gates on this
lane's phase status, its D8 (manifold reduction) re-cuts at this lane's phase 3,
and its WS-B depth re-reviews at this lane's phase 5. Do not pick up WP-PG.*
packets from this folder. All remote mutations (merges) are the maintainer's.
