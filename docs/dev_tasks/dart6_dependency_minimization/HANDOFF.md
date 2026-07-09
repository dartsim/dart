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
| 3 | Capability parity (distance→FCL, raycast→Bullet, CCD, manifolds, voxel) | 🔄 **in progress** — see §4 |
| 4 | Evidence-driven performance optimization (broadphase, SIMD, manifold reuse) | ⬜ not started |
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

`origin/release-6.20` tip at this refresh: `1a33843125d` (moves as the
maintainer merges; **always re-fetch before branching/capturing**).

## 4. Phase 2 complete; Phase 3 D5 manifold cache active

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
| **D5** | Persistent manifold cache and manifold reuse/reduction follow-up | 🔄 **active local branch** `feature/native-manifold-cache` |

### Exact next step: finish Phase 3 D5 persistent manifold cache

1. Continue with the **Phase 3 D5** branch `feature/native-manifold-cache`,
   based on `origin/release-6.20` at or after `1a33843125d`.
2. Keep it as one PR: port DART 7's persistent manifold cache with DART 6
   PascalCase file names, wire native detector cached-impulse reuse, and seed/
   write back cached impulses through `ContactConstraint` for solvers that
   honor `ConstraintInfo::x` initial guesses.
3. Cover cache matching/reduction/refresh, native detector cached-impulse reuse
   for same-group and cross-group contacts, and solver cached-impulse
   write-back.
4. Keep FCL as the default detector; do not touch `World`,
   `ConstraintSolver` detector defaults, `WorldConfig`, or dependency/package
   metadata in this slice.
5. Gates for **every** native-collision capability PR:
   Release build + **explicit Debug build** (`pixi run build` is Release-only);
   **run** the tests with `ctest --test-dir build/default/cpp/Release -R
   UNIT_collision_native --output-on-failure` (`pixi run test-all` only *builds*);
   full `pixi run check-lint`; contact-rich benchmark comparison evidence;
   `DART_PARALLEL_JOBS=8 pixi run -e gazebo test-gz` (199 + 4 + 1); portability
   `rg 'entt|std::span'` on new files → empty. The D5 scope necessarily also
   touches `dart/constraint/ContactConstraint.cpp` for native cached-impulse
   seed/write-back; keep any further scope expansion out. Preserve the public
   `dart::collision::Contact` layout on this release branch.

## 5. Open PRs / loose ends

- **Phase 3 D5 persistent manifold cache** — active on
  `feature/native-manifold-cache` as one PR-sized slice to minimize CI overhead.
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

- `.claude/worktrees/phase2-p2` — P2 merged; retained only because it has
  untracked local notes.
- `.claude/worktrees/phase2-p4` — dirty local performance-generalization work;
  not part of this lane's next code slice.
- `task_1_pr3239` — pre-existing, not this lane's.

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
