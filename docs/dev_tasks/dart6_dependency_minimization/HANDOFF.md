# HANDOFF — DART 6.20 dependency minimization (native collision port)

> Session handoff, 2026-07-06. Read [README.md](README.md) (overall SSOT),
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
| 1 | Native math core (aabb/gjk/mpr/box_box/sphere_sphere/shapes/span shim) internal-only | ✅ merged (#3281) |
| 2 | DART 6 detector adapter over the native core (bridge, sliced P1–P9) | 🔄 **in progress** — see §4 |
| 3 | Capability parity (distance→FCL, raycast→Bullet, CCD, manifolds, voxel) | ⬜ not started |
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
  (release-6.20). Its main-branch dual is **#3283 (still OPEN — see §5)**.
- **#3302** phase-2 execution plan (`07-phase2-adapter-scoping.md`) + this RESUME.
- **#3303** phase-2 **P1**: native BruteForce broadphase (internal-only).
- **#3306** phase-2 **P2**: narrowphase dispatcher (sphere/box only) →
  `dart/collision/native/narrow_phase/narrow_phase.{hpp,cpp}`.
- **#3318** phase-2 **P3a**: adapter skeleton + sphere/box conversion,
  intentionally unregistered. The `"native"` factory key still does not exist
  until P3b.

`origin/release-6.20` tip at this refresh: `634c20d1b9a1` (moves as the
maintainer merges; **always re-fetch before branching/capturing**).

## 4. Phase 2 — the active work (plan: `07`, PR slices P1–P9 + P10)

Bridge design (see `07` §0–§1): **bypass DART 7's EnTT `CollisionWorld`**; the
adapter (`NativeCollisionDetector/Group/Object` + `NativeShapeConversion`)
reuses DART 6's existing `shared_ptr`-based `CollisionObjectManager`, driving
**BruteForce broadphase → narrowphase dispatcher → DART 6 `Contact`**.

| Slice | Scope | Status |
| --- | --- | --- |
| **P1** | BroadPhase base + BruteForce (pure engine) | ✅ **merged (#3303)** |
| **P2** | Narrowphase dispatcher, sphere/box only (bespoke §2.1 trim) | ✅ **merged (#3306)** |
| **P3a** | Adapter skeleton + `NativeShapeConversion`(sphere,box), intentionally **unregistered**; `collide()` is a documented **stub** | ✅ **merged (#3318)** |
| **P3b** | Bridge `collide()` translation + `"native"` factory registration + `sphere_box` collider + normal calibration (R1) + parity vs **fcl and dart** | 🔄 **open (#3319)** |
| **P4** | `capsule_sphere`, `capsule_box` (no-span primitives) | after P3b |
| **P5** | `convex_convex` (keystone) + `capsule_capsule` (first span pair) | after P4 |
| **P6** | `cylinder_collision` (needs convex_convex) | after P5 |
| **P7** | `mesh_mesh` (needs convex_convex; largest file) | after P5 |
| **P8** | `distance` module (engine-only; needs span shim) | independent of P1–P7 |
| **P9** | `plane_sphere` (needs `distance`) → completes primitive+convex+mesh+plane | after P8 |
| **P10** | (optional) mixed-scene fcl/dart/native parity integration test | after P9 |

### Exact next step: execute P3b

1. Continue with **#3319** (`feature/native-detector-bridge`), after merging
   the latest `origin/release-6.20` into the PR branch before any push.
2. Implement/verify per `07` **§1.5** (bridge `collide()` translation,
   pair-only result mode, global/per-pair caps, normal orientation) and
   **§1.6** (delayed `"native"` factory registration; **no**
   `CollisionDetectorType::Native` enum), and the **P3b row in §3**.
3. Proof gates specific to P3b: `EXPECT_TRUE(getFactory()->canCreate("native"))`
   after the bridge produces real contacts; SKEL `"native"` resolves to the
   new detector; parity vs both **fcl** and **dart** on sphere-sphere, box-box,
   and sphere-box.
4. Gates for **every** phase-2 PR (see `07` §3 — note the corrected commands):
   Release build + **explicit Debug build** (`pixi run build` is Release-only);
   **run** the tests with `ctest --test-dir build/default/cpp/Release -R
   UNIT_collision_native --output-on-failure` (`pixi run test-all` only *builds*);
   full `pixi run check-lint`; guard-row hashes unchanged;
   `DART_PARALLEL_JOBS=8 pixi run -e gazebo test-gz` (199 + 4 + 1); portability
   `rg 'entt|std::span'` on new files → empty; scope-diff touches only
   `dart/collision/native/**` + its CMakeLists + `tests/**`.

## 5. Open PRs / loose ends

- **#3319 (P3b bridge)** — OPEN; next phase-2 merge target after #3318.
- **#3283 (main sphere-sphere `enableContact` fix)** — OPEN; the **main-branch
  dual** of the merged release-6.20 #3298. It was **reverted to the squared
  overlap predicate** (commit `afb1ea58959`) after review — see the lesson in
  §7. It still needs to merge so `main` and `release-6.20` stay consistent for
  the eventual phase-6 diff. Re-review was requested.

## 6. Load-bearing invariants (do not violate)

- **FCL stays the default** until phase 6. Never touch `WorldConfig`,
  `ConstraintSolver`'s two FCL-hardcoded ctors, `World::toCollisionDetectorKey`,
  or add a `CollisionDetectorType::Native` enum. Selection is the **factory
  pointer** only: `CollisionDetector::getFactory()->create("native")` (there is
  **no** `World::setCollisionDetector(const char*)` overload).
- **No EnTT, no new dependency, C++17 only.** `rg entt dart/collision/native`
  stays empty. Only C++20 feature in the ported tree is `std::span` → the
  `detail/span.hpp` shim.
- **gz gate every PR** (`pixi run -e gazebo test-gz`); on engine-only slices it
  is a non-regression guard (trivially green), substantive from P3b onward.
- **`narrow_phase.{hpp,cpp}` is a bespoke reduced dispatcher** (`07` §2.1): it is
  hand-trimmed and re-expands in P4–P9. Each dispatcher-touching PR must keep the
  `git diff origin/main` reduced to {span shim; dropped `CollisionObject`
  overloads + Compound + non-routed branches; the `enum class ShapeType;` compile
  fix}; P9 must re-converge to `main` modulo those deltas.
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

- `.claude/worktrees/phase2-p1` — P1 (#3303, merged), removable after confirming
  no unpushed local-only evidence is needed.
- `.claude/worktrees/phase2-p2` — P2 (**merged**), **removable now**; holds the
  untracked `08-p2-dispatcher-packet.md` working spec (disposable — the
  dispatcher approach is captured in `07` §2.1) and a symlinked `.pixi` env.
- `.claude/worktrees/dart7-sphere-fix` — #3283 (main), keep until it merges.
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
