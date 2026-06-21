# Scoping: Native collision detector port (DART 7 → DART 6.20)

> **Status: SCOPING / PLANNING ONLY** — no code, no PR yet. This is the design
> artifact for the maintainer's go/no-go on the native-collision initiative
> recorded in `02-default-environment-split.md` (decision c). Sub-doc of the
> `dart6_dependency_minimization` task.

## Why (the real dependency win)

DART 6.20 hard-requires **FCL** as the core collision dependency (default
detector + linked into `dart` + in `DART_PKG_EXTERNAL_DEPS`), and ships Bullet/
ODE as alternative backends. The only way to actually *remove* external collision
dependencies (not just hide them) is to make a **native** detector the default
and turn FCL/Bullet/ODE optional — exactly what DART 7 did (PLAN-035, PRs #2652
/#2688/#2700). Goal for 6.20: native default that is **gz-compatible**, **feature-
complete**, and **evidence-driven faster** than Bullet/ODE/FCL.

## What DART 7 provides (the source) — `dart/collision/native/` (~80 files)

- **Broadphase:** AABB-tree, brute-force, spatial-hash, sweep-and-prune (`broad_phase/`).
- **Narrowphase:** GJK + MPR (`gjk.cpp`, `mpr.cpp`), box-box SAT + face-clip + contact-reduction (`narrow_phase/box_box/`), sphere/box/capsule/cylinder/plane/mesh pairs, convex-convex, `distance.cpp`, `raycast.cpp`, `ccd.cpp`/`primitive_ccd.cpp`.
- **SDF:** dense / ESDF / TSDF fields (`sdf/`).
- **Manifolds:** persistent 4-point contact manifolds with warm-starting (`persistent_manifold_cache.*`, `contact_manifold.*`).
- **Default mechanism:** `DartCollisionDetector` registers under aliases (`"fcl"`/`"bullet"`/`"ode"`→native); FCL/ODE detectors became **header-only facades** (`collision/{fcl,ode}/compat/`) over native; no external collision deps.

## The target (DART 6.20) — `dart/collision/`

- Clean factory interface: `CollisionDetector` (virtuals: `collide×2`, `distance×2`, `raycast`, `createCollisionGroup`, `createCollisionObject`, `refreshCollisionObject`, `getType`, `cloneWithoutCollisionObjects`), `CollisionGroup`, `CollisionObject`, `CollisionResult`, `Contact`, `CollisionOption`.
- Detectors: **`dart`** (basic, narrowphase-only, no broadphase/distance/raycast, limited shapes), **`fcl`** (full; **hardcoded default** created in *both* `ConstraintSolver` constructors via `FCLCollisionDetector::create()` — `dart/constraint/ConstraintSolver.cpp:322` & `:342`; core), `ode`, `bullet`.
- FCL coupling to break: default detector · `VoxelGridShape`/octree (only FCL via `fcl::OcTree`) · distance queries (only FCL/Bullet).

## ⚠️ The DART 6 ↔ DART 7 gap (dominant effort/risk)

The port is **not a copy**:
1. **EnTT (ECS):** DART 7's `native` (`collision_world`, `comps/`) is built on EnTT — a dependency DART 6.20 does not have. Pulling EnTT in would *add* a dependency (defeats the goal), so the ECS-backed parts must be **reworked to plain C++17** (object-oriented `CollisionGroup`/`CollisionObject`) — significant.
2. **C++23 → C++17:** DART 7 native uses C++23 idioms; DART 6.20 is C++17 (Ubuntu-LTS policy). Must downgrade (`std::span`, concepts, ranges, etc.).
3. **API adaptation:** DART 7's collision API differs from DART 6's `CollisionDetector`/`Group`/`Object`. The native algorithms (GJK/MPR/SAT/SDF — largely pure math on Eigen types) port relatively cleanly; the *world/group/object plumbing* needs rewriting against DART 6's interface.

**Implication:** port the **algorithm core** (broadphase + narrowphase + distance/ccd/raycast/manifolds — Eigen-only math) and re-plumb it behind DART 6's `CollisionDetector`/`CollisionGroup` in C++17 without EnTT.

## gz-physics / gz-sim compatibility matrix (hard constraint — `pixi run -e gazebo test-gz`)

| gz requirement | Evidence | Native-port obligation |
| --- | --- | --- |
| `find_package(DART COMPONENTS collision-bullet collision-ode utils utils-urdf)` | `.deps/gz-physics/CMakeLists.txt` | Keep `collision-bullet` + `collision-ode` **components resolvable** (real or facade). |
| Subclasses `BulletCollisionDetector` / `OdeCollisionDetector` (`GzBulletCollisionDetector`/`GzOdeCollisionDetector` add `SetCollisionPairMaxContacts`) | `dartsim/src/GzCollisionDetector.*` | Those detector classes must stay **subclassable** with the same interface (facade-over-native is OK only if it preserves it). |
| `SetWorldCollisionDetector("bullet"/"ode"/"fcl"/"dart")` | `dartsim/src/WorldFeatures.cc` | All four names must keep resolving. |
| `world->getLastCollisionResult().getContacts()`; `Contact{point,normal,penetrationDepth,force,collisionObject1/2}` | `dartsim/src/SimulationFeatures.cc` | Native `Contact` must populate the same fields with equivalent semantics. |
| Per-pair contact capping | `GzCollisionDetector::LimitCollisionPairMaxContacts` | Native must honor `CollisionOption.maxNumContactsPerPair`. |
| CI gate | `scripts/run_gz_physics_task.sh` (ctest + plugin-links-DART check) | `pixi run -e gazebo test-gz` must stay green on every phase. |

## Feature-parity matrix (native must match the default detector)

Shape pairs (box/sphere/capsule/cylinder/plane/mesh/convex), **distance queries**, **raycast**, **CCD**, **persistent manifolds**, and **VoxelGrid/octree** (today FCL-only). Parity bar = **native determinism** (a stable `contact_benchmark` final-state hash with native held fixed) **plus tolerance-based scene-dump diffs** vs the incumbent detectors on the corpus — bit-exact match vs FCL/Bullet/ODE is *not* required (see Risks).

## Evidence-driven performance plan (your bar: native ≥ Bullet/ODE/FCL)

- **Harness exists:** `examples/contact_benchmark` takes `--collision dart|fcl|bullet|ode` and emits **RTF + final-state hash + contact stats** → same scene, swap detector, compare.
- **Macro:** parametrize `BM_INTEGRATION_boxes` (currently hardcodes Bullet, `boxes_scene.hpp:91`) across all four detectors; run `contact_benchmark` at 10/30/120 objects × {default, `--disable-deactivation`}.
- **Micro:** port DART 7 #2688's comparative benchmarks (`bm_narrow_phase`, `bm_stacked_scenes`) for broadphase pairs/s + narrowphase contacts/s.
- **Success** = perf: RTF(native) ≥ RTF(bullet/ode/fcl); **and** correctness: a stable native final-state hash + tolerance-based scene-dump diffs vs incumbents (per the parity bar — *not* bit-exact hash equality; see `docs/dev_tasks/issue_3056_dart6_performance` + Risks).

## Phased plan (each phase = its own reviewable PR; gz gate every phase)

0. **This scoping** + stand up the benchmark/parity harness (parametrize `BM_boxes`, port comparative benchmarks). No behavior change.
1. **Land the native engine** adapted to DART 6 (C++17, **no EnTT**) as an internal library, exposed via the existing `dart` detector (broadphase + core narrowphase). FCL stays default. Unit tests per shape pair.
2. **Feature parity**: distance, raycast, CCD, manifolds, all shape pairs; correctness = stable native hash + tolerance-based scene-dump diffs vs FCL/Bullet/ODE (not bit-exact hash equality — see Risks).
3. **Performance**: meet the evidence bar (macro + micro benchmarks); fix hot paths.
4. **Flip the default** to native in *both* `ConstraintSolver` constructors (`ConstraintSolver.cpp:322` & `:342`) — not just one — and confirm the runtime `setCollisionDetector` path (`:578`) is unaffected; make FCL/Bullet/ODE **optional** (facade or optional components) while keeping gz's `collision-bullet`/`collision-ode` components subclassable + `test-gz` green.
5. **Decouple FCL from core**: native VoxelGrid/octree path → drop `fcl` from the `dart` target and `DART_PKG_EXTERNAL_DEPS` → **the dependency win** (FCL/Bullet/ODE/ccd no longer required by a default build).

## Risks / open questions (for maintainer)

- **EnTT removal + C++23→C++17** is the bulk of the effort; confirm "no new deps" (EnTT must not be introduced).
- **gz subclassing**: is facade-over-native acceptable for `Bullet/OdeCollisionDetector`, or must Bullet/ODE remain real for gz? (DART 7 used facades; verify gz's `Gz*CollisionDetector` still compiles/links against facades.)
- **Bit-exact parity** with FCL/Bullet contacts is likely infeasible; agree the correctness bar is the `contact_benchmark` hash *for the native detector held fixed*, plus tolerance-based scene-dump diffs vs incumbents.
- **LTS risk**: this is a large change on a release branch. Phasing keeps each PR reviewable + gz-gated; default-flip (phase 4) is the point of no return — gate it on the perf evidence + a full Gazebo run.
- **Interaction with the open #3114** island-deactivation/collision fix — rebase onto it once merged.
- **Scale**: DART 7 delivered this across PLAN-035 (#2652/#2688/#2700) over months; in 6.20 it's a multi-PR, multi-week initiative even reusing the algorithms.

## Recommendation

Feasible and the right end-state, but **XL**. Recommend starting at **Phase 0** (harness + this scoping) on maintainer go-ahead, then Phase 1 as a standalone PR, re-evaluating after parity (Phase 2) before committing to the default-flip (Phase 4). If 6.20 LTS appetite for a change this large is limited, an alternative is to keep FCL core in 6.20 and target the native default for a **6.21** minor.
