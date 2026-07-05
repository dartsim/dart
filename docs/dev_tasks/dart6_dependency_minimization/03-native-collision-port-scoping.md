# Scoping: Native collision detector port (DART 7 -> DART 6.20)

> **Status: PLANNING REFRESH / EXECUTION PLAN** - DART 6.20 has already landed a
> substantial performance stack for issue #3056, but the DART 7 native collision
> engine has **not** been ported to DART 6 and FCL is still the release-branch
> default. This document is the go/no-go and sequencing artifact for the native
> port/default-flip work recorded in `02-default-environment-split.md`
> (decision c). Sub-doc of the `dart6_dependency_minimization` task.

## Why (the real dependency win)

DART 6.20 hard-requires **FCL** as the core collision dependency (default
detector + linked into `dart` + in `DART_PKG_EXTERNAL_DEPS`), and ships Bullet/
ODE as alternative backends. The only way to actually *remove* external collision
dependencies (not just hide them) is to make a **native** detector the default
and turn FCL/Bullet/ODE optional - exactly what DART 7 did (PLAN-035, PRs #2652
/#2688/#2700). Goal for 6.20: native default that is **gz-compatible**, **feature-
complete**, and **evidence-driven faster** than Bullet/ODE/FCL.

## Current evidence refresh (2026-07-05)

Refs checked:

- `origin/release-6.20`: `949a9c2ff5ed6309beef0aa1345101d36c813f02`.
- This phase-0 packet branch after the base merge: `1e6a8332a730a994450c14ee8a780780c5e069bb`.
- `origin/main`: `5c75381f79a0431909f8c1b0a04fca1fbaa256ed`.
- Related remote heads: `feature/native-occupancy-grid`,
  `feature/native-collision-core`, `task/native-collision-performance-exec`,
  `docs/dart6-performance-dashboard`, `backport/2490-to-release-6.20`,
  `fix/gz-physics-joint-detach-6.20`.

Live conclusion:

- `release-6.20` still has no `dart/collision/native/` port. The DART 6
  `dart` detector remains the old limited detector; the default path still
  resolves through FCL.
- DART 7 source/reference PRs remain #2652 (native default), #2688 (coverage and
  performance superset), and #2700 (native CCD). Use these as reference only;
  do not copy the EnTT/C++23 world plumbing directly into DART 6.
- DART 6 performance work for #3056 landed through the release stack (#3123,
  #3125, #3126, #3129, #3133, #3135, #3139, #3141, #3143, #3144, #3170, #3171,
  #3172, #3183, #3188, #3191, #3192, #3194, #3199, #3203). That stack resolves
  the headless-performance issue path, but it is **not** the native-engine port
  and does not by itself make FCL optional.
- Open release-6.20 work relevant to this plan:
  - #3209 (`perf/contact-rich-benchmark-case`) adds the deterministic
    contact-rich container benchmark needed as a default-flip gate.
  - #3230 (`docs/dart6-performance-dashboard`) adds the DART 6 performance
    dashboard workflow and benchmark-history plumbing.
  - #3229 (`backport/2490-to-release-6.20`) backports the DART 7 SIMD
    abstraction to C++17; useful for native hot paths, not a correctness
    prerequisite.
  - #3226 and #3227 are correctness/performance-adjacent release work and must
    be watched before interpreting collision-result drift.
- Issue #3056 is still open, but its latest maintainer evidence says the DART 6
  performance task was completed by #3199/#3203. Its final benchmark comment is
  still useful as a non-regression guardrail: DART-native was around RTF 65 on
  the `3k_shapes.sdf` case with a stable hash, while ODE plane-shape parsing was
  fixed from very low RTF to high RTF on the same large scene. Treat those rows
  as historical guardrails, not proof that the full DART 7 native detector is
  ported.

Local A/B probe on this branch (freshly rebuilt `contact_benchmark`; stale
CMake compiler launcher cache entries were cleared because the old build tree
pointed to a removed Pixi `sccache`):

```bash
pixi run ./build/default/cpp/Release/bin/contact_benchmark \
  --generate-objects 120 --steps 300 --warmup 0 --checkpoint 0 --quiet \
  --collision <detector> --world-threads 1 --max-contacts 20000
```

| Detector | RTF | Avg step | Contacts | Resting | Hash | Notes |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| `default` | 0.773604 | 1.29265 ms | 240 | 60/120 | `0xf69bb3c8f91330` | Matches FCL hash/outcome, slower in this short run. |
| `dart` | 3.33242 | 0.300082 ms | 240 | 80/120 | `0x8d8b20bbb24e90ee` | Faster here, but old limited DART detector is not the DART 7 native port. |
| `fcl` | 1.96733 | 0.508303 ms | 240 | 60/120 | `0xf69bb3c8f91330` | Current explicit incumbent default detector. |
| `ode` | 12.8554 | 0.0777883 ms | 0 | 120/120 | `0x7ab7392133b33c99` | Fast but physically different; zero final contacts means not a speed-only win. |
| `bullet` | 0.994733 | 1.00529 ms | 268 | 76/120 | `0x3abd10e684addd83` | Finite state, different contact/resting profile. |

Immediate implication: a native-default decision cannot use RTF alone. Every
performance row must be paired with final contacts, contact-cap status, resting
count, finite-state status, final hash, and scene-dump diffs. Different
detectors legitimately produce different hashes, but a new native detector must
be internally stable and must stay within documented tolerance envelopes against
the incumbent for each capability.

## What DART 7 provides (the source) - `dart/collision/native/` (~80 files)

- **Broadphase:** AABB-tree, brute-force, spatial-hash, sweep-and-prune (`broad_phase/`).
- **Narrowphase:** GJK + MPR (`gjk.cpp`, `mpr.cpp`), box-box SAT + face-clip + contact-reduction (`narrow_phase/box_box/`), sphere/box/capsule/cylinder/plane/mesh pairs, convex-convex, `distance.cpp`, `raycast.cpp`, `ccd.cpp`/`primitive_ccd.cpp`.
- **SDF:** dense / ESDF / TSDF fields (`sdf/`).
- **Manifolds:** persistent 4-point contact manifolds with warm-starting (`persistent_manifold_cache.*`, `contact_manifold.*`).
- **Default mechanism:** `DartCollisionDetector` registers under aliases
  (`"fcl"`/`"bullet"`/`"ode"` -> native); FCL/ODE detectors became
  **header-only facades** (`collision/{fcl,ode}/compat/`) over native; no
  external collision deps.

## The target (DART 6.20) - `dart/collision/`

- Clean factory interface: `CollisionDetector` (virtuals: `collide×2`, `distance×2`, `raycast`, `createCollisionGroup`, `createCollisionObject`, `refreshCollisionObject`, `getType`, `cloneWithoutCollisionObjects`), `CollisionGroup`, `CollisionObject`, `CollisionResult`, `Contact`, `CollisionOption`.
- Detectors: **`dart`** (basic, narrowphase-only, no broadphase/distance/raycast, limited shapes), **`fcl`** (full; **hardcoded default** created in *both* `ConstraintSolver` constructors via `FCLCollisionDetector::create()` - `dart/constraint/ConstraintSolver.cpp:322` & `:342`; core), `ode`, `bullet`.
- FCL coupling to break: default detector, `VoxelGridShape`/octree (only FCL, via `fcl::OcTree`), distance queries (**FCL-only** - Bullet/ODE/DART `distance()` are warn-and-return stubs).
- Capability incumbents (verified in-tree, for parity targeting): **distance -> FCL only** (`::fcl::distance`; others stub), **raycast -> Bullet only** (only `BulletCollisionDetector` overrides `raycast()`; FCL/ODE/DART fall back to the base "not supported"), **VoxelGrid/octree -> FCL only**.

## The DART 6 vs DART 7 gap (dominant effort/risk)

The port is **not a copy**:
1. **EnTT (ECS):** DART 7's `native` (`collision_world`, `comps/`) is built on EnTT - a dependency DART 6.20 does not have. Pulling EnTT in would *add* a dependency (defeats the goal), so the ECS-backed parts must be **reworked to plain C++17** (object-oriented `CollisionGroup`/`CollisionObject`) - significant.
2. **C++23 -> C++17:** DART 7 native uses C++23 idioms; DART 6.20 is C++17 (Ubuntu-LTS policy). Must downgrade (`std::span`, concepts, ranges, etc.).
3. **API adaptation:** DART 7's collision API differs from DART 6's `CollisionDetector`/`Group`/`Object`. The native algorithms (GJK/MPR/SAT/SDF — largely pure math on Eigen types) port relatively cleanly; the *world/group/object plumbing* needs rewriting against DART 6's interface.

**Implication:** port the **algorithm core** (broadphase + narrowphase + distance/ccd/raycast/manifolds — Eigen-only math) and re-plumb it behind DART 6's `CollisionDetector`/`CollisionGroup` in C++17 without EnTT.

## gz-physics / gz-sim compatibility matrix (hard constraint - `pixi run -e gazebo test-gz`)

| gz requirement | Evidence | Native-port obligation |
| --- | --- | --- |
| `find_package(DART COMPONENTS collision-bullet collision-ode utils utils-urdf)` | `.deps/gz-physics/CMakeLists.txt` | Keep `collision-bullet` + `collision-ode` **components resolvable** (real or facade). |
| Subclasses `BulletCollisionDetector` / `OdeCollisionDetector` (`GzBulletCollisionDetector`/`GzOdeCollisionDetector` add `SetCollisionPairMaxContacts`) | `dartsim/src/GzCollisionDetector.*` | Those detector classes must stay **subclassable** with the same interface (facade-over-native is OK only if it preserves it). |
| `SetWorldCollisionDetector("bullet"/"ode"/"fcl"/"dart")` | `dartsim/src/WorldFeatures.cc` | All four names must keep resolving. |
| `world->getLastCollisionResult().getContacts()`; `Contact{point,normal,penetrationDepth,force,collisionObject1/2}` | `dartsim/src/SimulationFeatures.cc` | Native `Contact` must populate the same fields with equivalent semantics. |
| Per-pair contact capping | `GzCollisionDetector::LimitCollisionPairMaxContacts` | Native must honor `CollisionOption.maxNumContactsPerPair`. |
| CI gate | `scripts/run_gz_physics_task.sh` (ctest + plugin-links-DART check) | `pixi run -e gazebo test-gz` must stay green on every phase. |

## Feature-parity matrix (native must match the default detector)

Shape pairs (box/sphere/capsule/cylinder/plane/mesh/convex), **distance queries** (incumbent: **FCL only**), **raycast** (incumbent: **Bullet only**), **CCD**, **persistent manifolds**, and **VoxelGrid/octree** (**FCL only**, via `fcl::OcTree`). Parity bar = **native determinism** (a stable `contact_benchmark` final-state hash with native held fixed) **plus tolerance-based scene-dump diffs** vs the incumbent detectors on the corpus — bit-exact match vs FCL/Bullet/ODE is *not* required (see Risks).

## Evidence-driven performance plan (native >= Bullet/ODE/FCL)

Minimum evidence packet before implementation starts:

- Merge #3209 or carry an equivalent local patch for the contact-rich container
  benchmark.
- Merge #3230 or carry an equivalent local JSON/HTML capture path so benchmark
  rows are durable and comparable across commits.
- Fix local build-cache state before running numbers: either configure from a
  clean build directory, provide a valid `sccache`/`ccache`, or explicitly clear
  the launcher cache entries with
  `-DCMAKE_C_COMPILER_LAUNCHER= -DCMAKE_CXX_COMPILER_LAUNCHER=`.
- Record commit SHAs (`origin/release-6.20`, branch head, and any reference
  `origin/main` SHA), compiler, CPU/governor, Pixi environment, exact command
  lines, and whether optional detectors were built.

Macro gates:

- `contact_benchmark`: generated 120/900/3000 shape scenes, `3k_shapes.sdf`,
  SDF PlaneShape mode, and contact-rich container once #3209 lands.
- `BM_INTEGRATION_boxes` or successor: parameterized across native/FCL/Bullet/ODE
  instead of hardcoding Bullet.
- `BM_INTEGRATION_contact_container`: single-threaded and multi-threaded rows
  with contacts/resting/mobile summaries.
- Gazebo-visible scenes: `pixi run -e gazebo test-gz` plus a small
  `fix/gz-physics-joint-detach-6.20` regression scene when #3227 lands.

Micro gates:

- Port or backport DART 7 #2688-style narrowphase and broadphase benchmarks:
  pairs/s, contacts/s, manifold update cost, raycast cost, distance-query cost,
  and mesh/SDF query cost.
- Use #3229 SIMD only after the scalar implementation is correct and measured;
  SIMD is an optimization packet, not a substitute for the C++17 port.
- Profile before optimizing. Require a before/after table for every hot-path
  change and reject changes that improve RTF by dropping contacts, breaking
  sleeping, or changing finite-state behavior without an accepted tolerance
  rationale.

Success means **both**:

- Performance: native RTF and microbenchmark throughput are >= the fastest
  incumbent detector for each supported capability, or a maintainer-approved
  exception is recorded before the default flip.
- Correctness: native is internally deterministic; final state is finite; contact
  cap, contact pairs, contacts, resting counts, and scene-dump diffs are within
  documented tolerances against the capability incumbent (FCL for distance and
  default contacts, Bullet for raycast, existing DART behavior where it is the
  only incumbent).

## Phased plan (each phase = its own reviewable PR; gz gate every phase)

0. **Evidence harness and queue cleanup.** Land or unblock #3209 and #3230, decide
   whether #3229 is a prerequisite for native optimization, and capture a
   baseline packet on current `release-6.20`. No behavior change.
1. **Native core skeleton.** Port the pure math/native algorithm core to DART 6
   as C++17 with no EnTT and no new dependency: AABB/broadphase scaffolding,
   GJK/MPR/SAT primitives, contact data structures, and deterministic tests.
   Keep it internal; FCL remains default.
2. **DART 6 detector adapter.** Plumb the native core behind the existing
   `CollisionDetector`/`CollisionGroup`/`CollisionObject` contracts, initially
   behind the `dart` detector or a new internal factory alias. Cover primitive,
   convex, mesh, plane, and SDF/voxel paths. FCL remains default.
3. **Capability parity.** Add distance, raycast, CCD, persistent manifolds,
   contact reduction, `CollisionOption.maxNumContactsPerPair`, and VoxelGrid/
   octree replacement. Gate each capability against its incumbent: FCL for
   distance/default contacts, Bullet for raycast, and current DART behavior where
   no other incumbent exists.
4. **Performance optimization.** Optimize only with measured evidence: data
   layout, scratch caches, broadphase pair pruning, manifold reuse, scene-local
   allocation control, optional SIMD from #3229, and thread-safe contact
   aggregation. Every optimization PR carries parent-vs-PR and detector-vs-
   detector tables.
5. **Compatibility facade decision.** Decide whether Bullet/ODE/FCL components
   stay real optional components or become facade-over-native compatibility
   components. This phase must prove gz subclassing still works, not just
   `find_package`.
6. **Default flip.** Flip native in *both* `ConstraintSolver` constructors
   (`ConstraintSolver.cpp:322` & `:342`) and confirm runtime
   `setCollisionDetector` remains unaffected. Require the full A/B packet,
   `pixi run test-all`, and `pixi run -e gazebo test-gz` before merge.
7. **FCL decoupling.** Only after the default flip is green, drop FCL from the
   `dart` target and `DART_PKG_EXTERNAL_DEPS`, move Bullet/ODE/FCL packages out
   of the default build, and update package/export docs. This is the dependency
   win.

## Risks / open questions (for maintainer)

- **EnTT removal + C++23→C++17** is the bulk of the effort; confirm "no new deps" (EnTT must not be introduced).
- **gz subclassing**: is facade-over-native acceptable for `Bullet/OdeCollisionDetector`, or must Bullet/ODE remain real for gz? (DART 7 used facades; verify gz's `Gz*CollisionDetector` still compiles/links against facades.)
- **Bit-exact parity** with FCL/Bullet contacts is likely infeasible; agree the correctness bar is the `contact_benchmark` hash *for the native detector held fixed*, plus tolerance-based scene-dump diffs vs incumbents.
- **LTS risk**: this is a large change on a release branch. Phasing keeps each PR reviewable + gz-gated; default-flip (phase 6) is the point of no return - gate it on the perf evidence + a full Gazebo run.
- **#3114** (FCL null contact when contact generation is disabled — the island-deactivation regression) is **already merged** into `release-6.20` and present in this branch (`e3b76ddf`); no rebase needed, the native port builds on top of it.
- **Scale**: DART 7 delivered this across PLAN-035 (#2652/#2688/#2700) over months; in 6.20 it's a multi-PR, multi-week initiative even reusing the algorithms.

## Immediate work packet

1. Refresh this dashboard and branch state whenever a release-6.20 PR lands.
2. ~~Unblock/merge #3209 and #3230~~ — **done** (both merged by 2026-07-03,
   along with #3226/#3227/#3229/#3234).
3. ~~Re-run the local A/B command above at larger scale~~ — **done
   2026-07-04; recaptured 2026-07-05 after the base moved** (canonical
   guard-scene protocol from
   `../dart6_performance_generalization/01-baseline-evidence.md`,
   cross-referenced against the perf lane's WP-PG.01 packet #3263 instead
   of duplicating its cells).
4. ~~Write the baseline packet before porting code~~ — **done**:
   `05-phase0-baseline-packet.md` (raw evidence: `05-artifacts.md`) with
   command lines, raw output, hashes, scene-dump tolerances, CPU
   metadata, and the explicit "native default NOT allowed at this tip"
   verdict plus the phase-6 acceptance envelope.
5. Start Phase 1 only after the baseline packet is reviewed.

## Recommendation

Feasible and the right end-state, but **XL**. The native detector port should
not start as a default-flip branch. Start with Phase 0 evidence harness and a
baseline packet, then land Phase 1/2 as internal non-default work. Re-evaluate
after capability parity and performance optimization before committing to the
default flip. If 6.20 LTS appetite for a change this large is limited, keep FCL
core in 6.20 and target the native default for a later 6.x minor.
