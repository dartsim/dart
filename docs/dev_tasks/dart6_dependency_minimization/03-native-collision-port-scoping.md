# Scoping: Native collision detector port (DART 7 -> DART 6.20)

> **Status: POST-CONSOLIDATION PLAN / CURRENT RESUME MAP (2026-07-29)** - PR
> #3381 merged the DART-owned engine into `DARTCollisionDetector`; `"dart"` is
> the sole canonical key and the unreleased interim `"native"` key is removed.
> FCL remains the DART 6.20 default and a core dependency. This document keeps
> the historical phase evidence, but any default flip or FCL decoupling is now
> later-release work gated on the full acceptance envelope.

## Why (the real dependency win)

DART 6.20 hard-requires **FCL** as the core collision dependency (default
detector + linked into `dart` + in `DART_PKG_EXTERNAL_DEPS`), and ships Bullet/
ODE as alternative backends. Removing those dependencies eventually requires
the consolidated `dart` detector to become the default, as DART 7 did
(PLAN-035, PRs #2652/#2688/#2700). For 6.20, the authorized goal is
consolidation only: remain **gz-compatible**, preserve FCL as the default, and
prove that FCL/Bullet/ODE paths incur no runtime regression.

## Current evidence refresh (2026-07-29)

Refs checked:

- Phase-0 original `origin/release-6.20`:
  `949a9c2ff5ed6309beef0aa1345101d36c813f02`.
- Phase-0 packet branch after the base merge:
  `1e6a8332a730a994450c14ee8a780780c5e069bb`.
- #3281 later merged via `f1916fd843f931eb7304b9a9dc8089eaf3586b38` on
  `origin/release-6.20` = `135cc8f20765d39040e3e3ae87536dabac8f5401`.
- Current `origin/release-6.20`: `ac7b9462612a9ef54eeb9d6841375c7789cf23d8`
  after detector consolidation #3381 and formatting repair #3406.
- Current `origin/main`: `83110ef54abf41f54c1e03500e49c1c12c305b8a`
  after merged AI-infrastructure update #3403.
- Surviving related remote heads: `feature/native-occupancy-grid` and
  `task/native-collision-performance-exec`. The older dashboard, SIMD-backport,
  and gz joint-detach topic heads have already been removed from the remote.

Historical conclusion at that refresh:

- `release-6.20` contained the phase-1 native math core, phase-2 DART 6
  detector adapter, interim `"native"` factory key, phase-3 capability parity
  pieces, native dashboard rows, and #3364's solver-facing manifold
  optimization. Merged PR #3381 supersedes that interim public naming with the
  sole canonical `"dart"` key.
- It still has no default flip: the default path still resolves through FCL,
  and FCL remains a core dependency until phases 6 and 7.
- DART 7 source/reference PRs remain #2652 (native default), #2688 (coverage and
  performance superset), and #2700 (native CCD). Use these as reference only;
  do not copy the EnTT/C++23 world plumbing directly into DART 6.
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

### Historical pre-consolidation A/B

This local probe predates #3381 and is retained only as a guardrail snapshot.
Its `"dart"` row refers to the deleted narrowphase-only implementation, not the
current consolidated detector. The raw local build/artifacts are not durable
resume inputs.

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
- **Narrowphase:** GJK + MPR (`Gjk.cpp`, `Mpr.cpp`), box-box SAT + face-clip + contact-reduction (`narrow_phase/box_box/`), sphere/box/capsule/cylinder/plane/mesh pairs, convex-convex, `Distance.cpp`, `raycast.cpp`, `ccd.cpp`/`primitive_ccd.cpp`.
- **SDF:** dense / ESDF / TSDF fields (`sdf/`).
- **Manifolds:** persistent 4-point contact manifolds with warm-starting (`persistent_manifold_cache.*`, `contact_manifold.*`).
- **Default mechanism:** `DartCollisionDetector` registers under aliases
  (`"fcl"`/`"bullet"`/`"ode"` -> native); FCL/ODE detectors became
  **header-only facades** (`collision/{fcl,ode}/compat/`) over native; no
  external collision deps.

## Current DART 6.20 state after #3381 - `dart/collision/`

- The DART 6 factory interface remains `CollisionDetector` plus
  `CollisionGroup`, `CollisionObject`, `CollisionResult`, `Contact`, and
  `CollisionOption`.
- **`dart`** now names the consolidated DART-owned engine under
  `dart/collision/dart/`. It includes broadphase, the ported rigid and soft
  collision coverage, distance, raycast, CCD, persistent manifolds, and
  VoxelGrid compound support. The interim `"native"` key and classes are gone.
- **`fcl`** remains the built-in default, created in both
  `ConstraintSolver` constructors (`ConstraintSolver.cpp:416` and `:433` at the
  merge), and FCL remains linked into core/package metadata. Bullet and ODE
  remain real optional backends/components; all four released keys still
  resolve.
- Distance/raycast/voxel parity work is merged, but each backend retains its
  documented semantics. Pixels are not proof of those numerical/query
  contracts; use the focused text-first tests and the gz gate.

## Historical DART 6 vs DART 7 port constraints (resolved by phases 1–3)

The completed port was **not a copy**:
1. **EnTT (ECS):** DART 7's `native` (`collision_world`, `comps/`) is built on EnTT - a dependency DART 6.20 does not have. Pulling EnTT in would *add* a dependency (defeats the goal), so the ECS-backed parts must be **reworked to plain C++17** (object-oriented `CollisionGroup`/`CollisionObject`) - significant.
2. **C++23 -> C++17:** DART 7 native uses C++23 idioms; DART 6.20 is C++17 (Ubuntu-LTS policy). Must downgrade (`std::span`, concepts, ranges, etc.).
3. **API adaptation:** DART 7's collision API differs from DART 6's `CollisionDetector`/`Group`/`Object`. The native algorithms (GJK/MPR/SAT/SDF — largely pure math on Eigen types) port relatively cleanly; the *world/group/object plumbing* needs rewriting against DART 6's interface.

**Implemented result:** the algorithm core was adapted behind DART 6's
`CollisionDetector`/`CollisionGroup` API in C++17 without adding EnTT.

## gz-physics / gz-sim compatibility matrix (hard constraint - `pixi run -e gazebo test-gz`)

| gz requirement | Evidence | Consolidated-engine obligation |
| --- | --- | --- |
| `find_package(DART COMPONENTS collision-bullet collision-ode utils utils-urdf)` | `.deps/gz-physics/CMakeLists.txt` | Keep `collision-bullet` + `collision-ode` **components resolvable** until an approved transition preserves this downstream contract. |
| Subclasses `OdeCollisionDetector` **only** (`GzOdeCollisionDetector` adds `SetCollisionPairMaxContacts`; there is no `GzBulletCollisionDetector` — Bullet is used via plain `create()`) | `dartsim/src/GzOdeCollisionDetector.{hh,cc}` | `OdeCollisionDetector` must stay **subclassable** with overridable `collide()` unless a coordinated gz change lands first. A compatibility facade would delegate to the canonical `dart` engine; Bullet/FCL only need `create()` + name resolution. |
| `SetWorldCollisionDetector("bullet"/"ode"/"fcl"/"dart")` | `dartsim/src/WorldFeatures.cc` | All four names must keep resolving. |
| `world->getLastCollisionResult().getContacts()`; `Contact{point,normal,penetrationDepth,force,collisionObject1/2}` | `dartsim/src/SimulationFeatures.cc` | The canonical `dart` engine and any compatibility facades must populate the same fields with equivalent semantics. |
| Per-pair contact capping | `GzOdeCollisionDetector::LimitCollisionPairMaxContacts` | The canonical `dart` engine and any ODE facade must honor `CollisionOption.maxNumContactsPerPair`. |
| CI gate | `scripts/run_gz_physics_task.sh` (ctest + plugin-links-DART check) | `pixi run -e gazebo test-gz` must stay green on every phase. |

## Historical feature-parity matrix (completed by phases 2–3)

At plan time, the target covered shape pairs
(box/sphere/capsule/cylinder/plane/mesh/convex), **distance queries**
(incumbent: **FCL only**), **raycast** (incumbent: **Bullet only**), **CCD**,
**persistent manifolds**, and **VoxelGrid/octree** (incumbent: **FCL only**).
The accepted bar was native determinism plus tolerance-based scene-dump diffs;
bit-exact cross-detector matching was not required. Those capabilities and
their focused tests are now merged.

## Historical evidence-driven performance plan (closed for this lane)

The implementation used this minimum evidence packet:

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
   detector tables. **Started: #3362 added native dashboard rows; #3364 capped
   solver-facing manifolds and made native faster than DART/FCL/Bullet on the
   120-object contact-container row in that run; #3368 added the AABB-tree
   broadphase. **Closed for this lane by the consolidation; general remaining
   performance work is owned by `dart6_performance_generalization`.**
5. **Compatibility facade decision.** Decide whether FCL/Bullet/ODE components
   stay real optional components or become compatibility facades over the
   canonical `dart` engine. This phase must prove gz subclassing still works,
   not just `find_package`. **Decision drafted in
   `08-phase5-facade-decision.md`; facade-over-dart for ODE versus a
   coordinated gz-physics change still needs maintainer ratification.**
6. **Default flip.** Change the default from FCL to Dart in *both*
   `ConstraintSolver` constructors (`ConstraintSolver.cpp:416` & `:433`) and
   the other flip surfaces documented in `08-phase5-facade-decision.md`;
   confirm runtime detector selection remains unaffected. Require the full A/B
   packet, the `pixi run test-all` default build-and-runtime aggregate,
   focused `pixi run test`/`test-py` reruns, and
   `pixi run -e gazebo test-gz` before merge.
7. **FCL decoupling.** Only after the default flip is green, drop FCL from the
   `dart` target and `DART_PKG_EXTERNAL_DEPS`, move Bullet/ODE/FCL packages out
   of the default build, and update package/export docs. This is the dependency
   win.

## Risks / open questions (for maintainer)

- **Port constraints resolved:** the consolidated engine is C++17 and does not
  add EnTT; future phases must preserve that no-new-dependency result.
- **gz subclassing:** ratify facade-over-dart for `OdeCollisionDetector` or a
  coordinated gz-physics change that removes the subclass first. Bullet and
  FCL do not have gz subclasses.
- **Bit-exact parity** with FCL/Bullet contacts is likely infeasible; agree the
  correctness bar is the `contact_benchmark` hash for the consolidated `dart`
  detector held fixed, plus tolerance-based scene-dump diffs vs incumbents.
- **LTS risk**: this is a large change on a release branch. Phasing keeps each PR reviewable + gz-gated; default-flip (phase 6) is the point of no return - gate it on the perf evidence + a full Gazebo run.
- **#3114** (FCL null contact when contact generation is disabled — the
  island-deactivation regression) is historical prerequisite work already
  present in `release-6.20` (`e3b76ddf`); no follow-up action remains here.
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
5. ~~Phase 1 (#3281) has landed as an internal-only, no-default-change math
   core. Do not start Phase 2/default-adapter work until this phase-0 packet is
   accepted.~~ **Superseded:** phases 2 and 3 are merged, and Phase 4 has begun.
6. ~~Refresh and close Phase 4 in this lane.~~ **Done/superseded by #3368 and
   the merged consolidation; later general performance work has a separate
   owner.**
7. Current next packet: obtain maintainer ratification for the open Phase 5
   ODE/gz choice and record the target later-release sequence. Prefer clean,
   long-term interfaces over header-compatibility shims; preserve gz-physics
   and gz-sim behavior where they depend on subclassing, component names,
   detector factory keys, or contact semantics. Do not implement the default
   flip or dependency removal on `release-6.20`.

## Recommendation

Feasible and the right end-state, but still **XL** across later releases. The
6.20 consolidation is merged and Phase 4 is closed for this lane. Do not jump
directly to the default flip: first ratify Phase 5 with gz compatibility proof.
Only on an approved later release, after recapturing the full acceptance
packet, should Phase 6 flip the default in both `ConstraintSolver`
constructors; only after a green default-flip packet should Phase 7 remove FCL
from the core dependency surface.
