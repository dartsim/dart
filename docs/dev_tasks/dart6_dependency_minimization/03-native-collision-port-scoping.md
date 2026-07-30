# Scoping: DART collision backend evolution

> **Status: DART 6.20 BACKEND COMPLETE / LATER PHASES DEFERRED
> (2026-07-30).** PR #3381 established `DARTCollisionDetector` behind the
> canonical `"dart"` key. FCL remains the DART 6.20 default and a core
> dependency. This document keeps the historical phase evidence, but any
> default flip or FCL decoupling is later-release work gated on the full
> acceptance envelope and explicit maintainer authorization.

## Why (the real dependency win)

DART 6.20 hard-requires **FCL** as the core collision dependency (default
detector + linked into `dart` + in `DART_PKG_EXTERNAL_DEPS`), and ships Bullet/
ODE as alternative backends. Removing those dependencies eventually requires
the `dart` detector to become the default, as DART 7 did (PLAN-035, PRs
#2652/#2688/#2700). For 6.20, the authorized goal was the current DART backend:
remain **gz-compatible**, preserve FCL as the default, and prove that
FCL/Bullet/ODE paths incur no runtime regression.

## Current evidence refresh (2026-07-30)

Refs checked:

- Phase-0 original `origin/release-6.20`:
  `949a9c2ff5ed6309beef0aa1345101d36c813f02`.
- Phase-0 packet branch after the base merge:
  `1e6a8332a730a994450c14ee8a780780c5e069bb`.
- #3281 later merged via `f1916fd843f931eb7304b9a9dc8089eaf3586b38` on
  `origin/release-6.20` = `135cc8f20765d39040e3e3ae87536dabac8f5401`.
- Current `origin/release-6.20`:
  `2ffe228c14c67e120d2a946ce9d36e8a9658044f` at this refresh.
- PR #3381 verified squash merge:
  `46719bfbd75e1f70e69b2c76fb34a3fa2b78edd5`.
- PR #3381 exact reviewed head:
  `64d476b68ad5ae0dcca4e98abb9bba15b6962b87`.
- Current `origin/main`:
  `83110ef54abf41f54c1e03500e49c1c12c305b8a`.
- Related remote heads: `feature/native-occupancy-grid`,
  `task/native-collision-performance-exec`,
  `docs/dart6-performance-dashboard`, `backport/2490-to-release-6.20`,
  `fix/gz-physics-joint-detach-6.20`.

Current conclusion:

- `release-6.20` contains `DARTCollisionDetector` under
  `dart/collision/dart/`, selected by public key `"dart"`.
- It has no default flip: both `ConstraintSolver` constructors and the complete
  default-selection surface still resolve through FCL. FCL remains a core
  dependency until a future phase 6 is accepted and phase 7 is executed.
- The FCL, Bullet, and ODE implementations, package components, dependencies,
  and default-selection paths are source-identical to the PR parent. The
  released `DARTCollide` surface and detector/group/object layouts are
  preserved.
- Remaining general collision/solver performance work is owned by
  `docs/dev_tasks/dart6_performance_generalization/`, not this lane.
- DART 7 source/reference PRs remain #2652 (default backend), #2688 (coverage
  and performance superset), and #2700 (CCD). Use these as reference only; do
  not copy the EnTT/C++23 world plumbing directly into DART 6.
- DART 6 performance work for #3056 landed through the release stack (#3123,
  #3125, #3126, #3129, #3133, #3135, #3139, #3141, #3143, #3144, #3170, #3171,
  #3172, #3183, #3188, #3191, #3192, #3194, #3199, #3203). That stack resolves
  the headless-performance issue path, but it is **not** the native-engine port
  and does not by itself make FCL optional.
- Recently landed release-6.20 work relevant to this plan:
  - #3209 (`perf/contact-rich-benchmark-case`) added the deterministic
    contact-rich container benchmark needed as a default-flip gate.
  - #3230 (`docs/dart6-performance-dashboard`) added the DART 6 performance
    dashboard workflow and benchmark-history plumbing.
  - #3229 (`backport/2490-to-release-6.20`) backported the DART 7 SIMD
    abstraction to C++17; useful for native hot paths, not a correctness
    prerequisite.
  - #3226 and #3227 landed correctness/performance-adjacent release work to keep
    in mind when interpreting collision-result drift.
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

## Historical DART 7 collision source (~80 files)

- **Broadphase:** AABB-tree, brute-force, spatial-hash, sweep-and-prune (`broad_phase/`).
- **Narrowphase:** GJK + MPR (`Gjk.cpp`, `Mpr.cpp`), box-box SAT + face-clip + contact-reduction (`narrow_phase/box_box/`), sphere/box/capsule/cylinder/plane/mesh pairs, convex-convex, `Distance.cpp`, `raycast.cpp`, `ccd.cpp`/`primitive_ccd.cpp`.
- **SDF:** dense / ESDF / TSDF fields (`sdf/`).
- **Manifolds:** persistent 4-point contact manifolds with warm-starting (`persistent_manifold_cache.*`, `contact_manifold.*`).
- **Default mechanism:** `DartCollisionDetector` registers under aliases
  (`"fcl"`/`"bullet"`/`"ode"` -> native); FCL/ODE detectors became
  **header-only facades** (`collision/{fcl,ode}/compat/`) over native; no
  external collision deps.

## Historical pre-port DART 6.20 target — `dart/collision/`

- Clean factory interface: `CollisionDetector` (virtuals: `collide×2`, `distance×2`, `raycast`, `createCollisionGroup`, `createCollisionObject`, `refreshCollisionObject`, `getType`, `cloneWithoutCollisionObjects`), `CollisionGroup`, `CollisionObject`, `CollisionResult`, `Contact`, `CollisionOption`.
- Detectors at the original scoping baseline: **`dart`** (basic,
  narrowphase-only, no broadphase/distance/raycast, limited shapes), **`fcl`**
  (full; **hardcoded default** created in *both* `ConstraintSolver`
  constructors via `FCLCollisionDetector::create()` -
  `dart/constraint/ConstraintSolver.cpp:416` & `:433`; core), `ode`, `bullet`.
- FCL coupling at that baseline: default detector, `VoxelGridShape`/octree
  (only FCL, via `fcl::OcTree`), and distance queries (**FCL-only**;
  Bullet/ODE/DART `distance()` were warn-and-return stubs).
- Capability incumbents at that baseline (for parity targeting):
  **distance -> FCL only** (`::fcl::distance`; others stub),
  **raycast -> Bullet only** (only `BulletCollisionDetector` overrode
  `raycast()`; FCL/ODE/DART fell back to the base "not supported"), and
  **VoxelGrid/octree -> FCL only**.

## Historical DART 6 vs DART 7 gap

The port is **not a copy**:
1. **EnTT (ECS):** DART 7's `native` (`collision_world`, `comps/`) is built on EnTT - a dependency DART 6.20 does not have. Pulling EnTT in would *add* a dependency (defeats the goal), so the ECS-backed parts must be **reworked to plain C++17** (object-oriented `CollisionGroup`/`CollisionObject`) - significant.
2. **C++23 -> C++17:** DART 7 native uses C++23 idioms; DART 6.20 is C++17 (Ubuntu-LTS policy). Must downgrade (`std::span`, concepts, ranges, etc.).
3. **API adaptation:** DART 7's collision API differs from DART 6's `CollisionDetector`/`Group`/`Object`. The native algorithms (GJK/MPR/SAT/SDF — largely pure math on Eigen types) port relatively cleanly; the *world/group/object plumbing* needs rewriting against DART 6's interface.

**Implication:** port the **algorithm core** (broadphase + narrowphase + distance/ccd/raycast/manifolds — Eigen-only math) and re-plumb it behind DART 6's `CollisionDetector`/`CollisionGroup` in C++17 without EnTT.

## gz-physics / gz-sim compatibility matrix (hard constraint - `pixi run -e gazebo test-gz`)

| gz requirement | Evidence | Native-port obligation |
| --- | --- | --- |
| `find_package(DART COMPONENTS collision-bullet collision-ode utils utils-urdf)` | `.deps/gz-physics/CMakeLists.txt` | Keep `collision-bullet` + `collision-ode` **components resolvable** (real or facade). |
| Subclasses `OdeCollisionDetector` **only** (`GzOdeCollisionDetector` adds `SetCollisionPairMaxContacts`; there is no `GzBulletCollisionDetector` — Bullet is used via plain `create()`) | `dartsim/src/GzOdeCollisionDetector.{hh,cc}` | `OdeCollisionDetector` must stay **subclassable** with overridable `collide()` (facade-over-native is OK only if it preserves that); Bullet/FCL only need `create()` + name resolution. |
| `SetWorldCollisionDetector("bullet"/"ode"/"fcl"/"dart")` | `dartsim/src/WorldFeatures.cc` | All four names must keep resolving. |
| `world->getLastCollisionResult().getContacts()`; `Contact{point,normal,penetrationDepth,force,collisionObject1/2}` | `dartsim/src/SimulationFeatures.cc` | Native `Contact` must populate the same fields with equivalent semantics. |
| Per-pair contact capping | `GzOdeCollisionDetector::LimitCollisionPairMaxContacts` | Native must honor `CollisionOption.maxNumContactsPerPair`. |
| CI gate | `scripts/run_gz_physics_task.sh` (ctest + plugin-links-DART check) | `pixi run -e gazebo test-gz` must stay green on every phase. |

## Feature-parity matrix (native must match the default detector)

Shape pairs (box/sphere/capsule/cylinder/plane/mesh/convex), **distance queries** (incumbent: **FCL only**), **raycast** (incumbent: **Bullet only**), **CCD**, **persistent manifolds**, and **VoxelGrid/octree** (**FCL only**, via `fcl::OcTree`). Parity bar = **native determinism** (a stable `contact_benchmark` final-state hash with native held fixed) **plus tolerance-based scene-dump diffs** vs the incumbent detectors on the corpus — bit-exact match vs FCL/Bullet/ODE is *not* required (see Risks).

## Evidence-driven performance plan (native >= Bullet/ODE/FCL)

Minimum evidence packet before implementation starts:

- Use the merged #3209 contact-rich container benchmark.
- Use the merged #3230 JSON/HTML capture path so benchmark rows are durable and
  comparable across commits.
- Fix local build-cache state before running numbers: either configure from a
  clean build directory, provide a valid `sccache`/`ccache`, or explicitly clear
  the launcher cache entries with
  `-DCMAKE_C_COMPILER_LAUNCHER= -DCMAKE_CXX_COMPILER_LAUNCHER=`.
- Record commit SHAs (`origin/release-6.20`, branch head, and any reference
  `origin/main` SHA), compiler, CPU/governor, Pixi environment, exact command
  lines, and whether optional detectors were built.

Macro gates:

- `contact_benchmark`: generated 120/900/3000 shape scenes, `3k_shapes.sdf`,
  SDF PlaneShape mode, and the #3209 contact-rich container.
- `BM_INTEGRATION_boxes` or successor: parameterized across native/FCL/Bullet/ODE
  instead of hardcoding Bullet.
- `BM_INTEGRATION_contact_container`: single-threaded and multi-threaded rows
  with contacts/resting/mobile summaries.
- Gazebo-visible scenes: `pixi run -e gazebo test-gz` plus the #3227
  `fix/gz-physics-joint-detach-6.20` regression scene.

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

0. **Evidence harness and queue cleanup.** With #3209 and #3230 landed, decide
   whether #3229 is a prerequisite for native optimization, and capture a
   baseline packet on current `release-6.20`. No behavior change. **Done:
   #3271.**
1. **Native core skeleton.** Port the pure math/native algorithm core to DART 6
   as C++17 with no EnTT and no new dependency: AABB/broadphase scaffolding,
   GJK/MPR/SAT primitives, contact data structures, and deterministic tests.
   Keep it internal; FCL remains default. **Done: #3281.**
2. **DART 6 detector adapter.** Plumb the native core behind the existing
   `CollisionDetector`/`CollisionGroup`/`CollisionObject` contracts, initially
   behind the `dart` detector or a new internal factory alias. Cover primitive,
   convex, mesh, plane, and SDF/voxel paths. FCL remains default. **Done:
   #3303, #3306, #3318, #3319, #3321, #3322, #3324, #3325, #3343, #3350.**
3. **Capability parity.** Add distance, raycast, CCD, persistent manifolds,
   contact reduction, `CollisionOption.maxNumContactsPerPair`, and VoxelGrid/
   octree replacement. Gate each capability against its incumbent: FCL for
   distance/default contacts, Bullet for raycast, and current DART behavior where
   no other incumbent exists. **Done: #3352, #3355, #3358, #3359, #3360.**
4. **Performance optimization.** Optimize only with measured evidence: data
   layout, scratch caches, broadphase pair pruning, manifold reuse, scene-local
   allocation control, optional SIMD from #3229, and thread-safe contact
   aggregation. Every optimization PR carries parent-vs-PR and detector-vs-
   detector tables. **Closed for this lane:** #3362 added dashboard rows,
   #3364 capped solver-facing manifolds, #3368 added the AABB-tree broadphase,
   and #3381 completed the DART detector with exact incumbent-backend
   no-regression evidence. Remaining general performance packets belong to the
   performance-generalization task.
5. **Compatibility facade decision.** Decide whether Bullet/ODE/FCL components
   stay real optional components or become facade-over-DART compatibility
   components. This phase must prove gz subclassing still works, not just
   `find_package`. **Decision 1 complete:** #3381 established `"dart"` as the
   sole DART-owned backend. The future facade implementation and ODE/gz
   ratification point remain pending.
6. **Default flip.** Make the `dart` detector the default in *both*
   `ConstraintSolver` constructors
   (`ConstraintSolver.cpp:416` & `:433`) and confirm runtime
   `setCollisionDetector` remains unaffected. Require the full A/B packet,
   `pixi run test-all`, and `pixi run -e gazebo test-gz` before merge.
   **Deferred beyond DART 6.20.**
7. **FCL decoupling.** Only after the default flip is green, drop FCL from the
   `dart` target and `DART_PKG_EXTERNAL_DEPS`, move Bullet/ODE/FCL packages out
   of the default build, and update package/export docs. This is the dependency
   win. **Pending on a future release after phase 6.**

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
   metadata, and the explicit "`dart` default NOT allowed at this tip"
   verdict plus the phase-6 acceptance envelope.
5. ~~Phase 1 (#3281) has landed as an internal-only, no-default-change math
   core. Do not start Phase 2/default-adapter work until this phase-0 packet is
   accepted.~~ **Superseded:** phases 2 and 3 merged, Phase 4 closed for this
   lane, and the DART 6.20 backend work is complete.
6. ~~Close Phase 4 and execute the canonical-backend part of Phase 5.~~
   **Done:** #3368 and #3381. PR #3381 preserves FCL as the 6.20 default,
   keeps FCL/Bullet/ODE paths structurally unchanged, and carries local
   C++/Python/gz/ABI/visual plus incumbent-backend no-regression evidence.
7. ~~Finish the post-merge evidence/docs closeout.~~ **Done locally
   2026-07-30:** exact-head Linux run `30510684936` completed successfully
   with all six jobs green, including Release, AddressSanitizer, install,
   coverage, assertions, Eigen alignment, and AI infrastructure. The verified
   Release visual artifact has passing FCL/DART manifests, zero skipped
   contacts, and manually accepted images. #3406's macOS Release and Debug
   jobs both passed the Black lint step that failed on the squash merge. The
   documentation-only closeout is tracked by
   [PR #3409](https://github.com/dartsim/dart/pull/3409).
8. After that, stop implementation on `release-6.20`. Resume phases 6 and 7
   only when a future release line and milestone exist and a maintainer
   explicitly authorizes the default-flip proposal.

## Recommendation

The DART 6.20 deliverable is complete: phases 0–3 landed, Phase 4 closed for
this lane, and Phase 5's canonical-backend decision shipped in #3381 while FCL
remained the default. The dependency end-state is still **XL** and incomplete:
phase 6 must be proposed and proven on an authorized later release, then phase
7 may remove FCL from the core dependency surface and implement the approved
compatibility facades. Do not use the completed DART 6.20 backend work as
permission to jump those release gates.
