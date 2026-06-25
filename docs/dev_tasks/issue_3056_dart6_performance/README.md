# Issue 3056 DART 6 Performance

## Current Snapshot

Bottom line: #3129, #3133, #3135, #3139, #3140, #3141, #3142, #3143,
#3144, #3146, #3147, #3148, #3149, #3150, #3151, #3152, #3153, #3154,
#3170, #3171, #3172, and #3183 are merged. There are no open performance PRs.
Each follow-up slice must remain local-only until it can be published directly
against `release-6.20`; do not open it against another PR branch. Opening
stacked PRs against parent PR branches lets GitHub close or supersede child PRs
when the parent branch is merged or deleted. The current unpublished local
stack, in order, is `perf/dart6-contact-pair-metadata-cache`,
`perf/dart6-contact-pair-skip-flags`,
`perf/dart6-axis-plane-contact-bounds`,
`perf/dart6-default-surface-version-cache`,
`perf/dart6-lazy-dart-shape-cache`,
`perf/dart6-free-joint-root-integration`,
`perf/dart6-parallel-surface-scan`,
`perf/dart6-broadphase-cache-refresh-order`,
`perf/dart6-parallel-pair-flag-resize`,
`perf/dart6-single-reactive-union-reset`, and
`perf/dart6-single-reactive-raw-root`. The first slice,
`perf/dart6-contact-pair-metadata-cache`, is rebuilt directly on the merged
`release-6.20` head and is PR-ready.

The merged #3172 slice adds a narrow native broadphase setup fast path.
DART-native collision objects cache local bounds center/half-extents when their
shape cache refreshes. When a collision object's transform has an exactly
identity linear part, the backend computes world bounds from translation plus
those cached local values instead of doing the general matrix/absolute-linear
bound calculation. Rotated or otherwise general transforms keep the existing
path.

The active unpublished candidate keeps the #3183 parallel default-contact
build, then reduces the remaining per-step pair metadata overhead. Contact
candidate collection stores the pair's already-validated `BodyNode` and
`ShapeNode` pointers in `ContactPairCount`; the build phase reuses those
pointers instead of re-reading them from the first contact. The same pair record
also caches the default-surface decision once for the step, and the parallel
eligibility scan replaces the per-pair `unordered_set` insertion path with a
retained vector, sort, and adjacent-duplicate check. Contact order is unchanged
because workers still write constraints by candidate index and the main thread
activates them in order.

A local follow-up experiment then precomputes the fixed-zero-velocity support
skip flags once per contact pair during the serial parallel-eligibility scan and
uses a thread-local open-addressed body set for the remaining duplicate-body
check. The parallel reset maps those pair-order flags back to each contact's
original object order, avoiding repeated body lookups and hash-set probes in the
worker loop while keeping the same `ContactConstraint::initialize()` fallback
checks for every side that was not proven skippable.

The next local collision follow-up accelerates the DART-native disjoint contact
proof for exact world-axis planes. The finite-plane duplicate-contact proof
normally projects each finite AABB through all eight corners onto the plane. If
the normalized plane normal is exactly aligned with a world axis, the projection
keeps the two orthogonal AABB ranges unchanged and collapses only the plane
axis to the plane point. The proof also stores only projected min/max bounds in
scratch instead of copying full broadphase entries through the sort. Non-axis
planes keep the original corner-projection path.

The newest local solver follow-up keeps the per-step `ContactPairCount` surface
decision cache, then extends the lower-level default-surface-property cache into
a larger thread-local, versioned direct-mapped cache. Each entry keys on the
`ShapeNode` pointer and `ShapeNode::getVersion()`, so friction, slip,
restitution, or friction-direction updates invalidate cached default-material
decisions while steady-state default scenes avoid repeated dynamics-aspect
queries. The branch also splits the shared-body profiling scope into body-scan
and surface-scan subscopes.

The local collision setup follow-up moves DART-native shape-cache refresh
into broadphase entry construction, then skips the separate DART collision-group
object-update pass. Direct object-object collision still refreshes each object
through the existing `updateEngineData()` path. Group collision already visits
every DART object while constructing broadphase entries, so the lazy refresh
keeps shape replacement and `ShapeNode` relative-transform edits visible without
a second full object pass.

The newest local integration follow-up narrows `FreeJoint` hot paths for the
root-joint shape used by the SDF issue scene: identity child joint frames and
translation-only parent model poses. Position integration, relative-transform
refresh, and relative-Jacobian refresh skip the general child-frame inverse and
full transform products in those cases. Non-identity child joint frames and
rotated parent joint frames keep the general path.

The newest local solver follow-up parallelizes the default-surface-property
scan that remains before contact-constraint rebuild. The shared-body/body
duplicate scan stays serial, but large contact-pair sets now compute the
default-material predicate in the existing constraint thread pool and then
reduce a scratch flag back on the main thread. Small contact sets and any
fallback surface-parameter path keep the original serial
`ensureDefaultSurfaceParamsChecked()` behavior.

The newest local DART-native broadphase hardening follow-up refreshes cached
shape metadata before broadphase entry classification and before the serial
transform-cache prepass used by the parallel broadphase builder. Worker threads
then build entries from that prepared cache. This preserves the lazy transform
cache safety boundary when a `ShapeNode` changes from an identity-relative
collision shape to a non-identity relative transform after the collision group
already exists.

The newest local collision scratch follow-up stops zero-filling the parallel
finite-plane collision flag vector before dispatch. Every parallel work item
overwrites its own flag before the serial merge reads it, so retaining capacity
with `resize()` preserves behavior while avoiding a redundant per-step fill.

The newest local solver bookkeeping follow-up narrows the constrained-group
union reset for the all-single-reactive-contact path. That path only stamps
`mUnionIndex` while building one reactive-skeleton group per contact island; it
does not union skeleton roots. The follow-up resets only those stamped group
root indices, while all generic constraint paths that call `uniteSkeletons()`
keep the full `Skeleton::resetUnion()` pass.

The #3183 candidate folds default contact-surface parameter
initialization into the existing per-pair parallel default-contact rebuild. The
serial cached prepass remains for fallback paths. Default-material pairs compute
their direct surface parameters in the same worker that resets that pair's
reusable built-in `ContactConstraint` objects, while any non-default
`DefaultContactSurfaceHandler` fallback is precomputed by the serial
surface-parameter prepass before worker resets begin. That preserves the old
safety boundary for custom friction directions and direction frames whose world
transforms may be lazily cached. Active constraint order is unchanged because
workers still write constraints by candidate index and the main thread
activates them in order.

Each performance PR in this stack must keep ODE in the comparison table along
with FCL and Bullet. ODE is the relevant gz-physics/gz-sim baseline, so
DART-native improvements are not considered complete on this workload unless
the evidence shows native collision ahead of all three backends on the same
DART 6 dynamics and solver pipeline.

Active issue-scene evidence,
`.deps/gz-sim/examples/worlds/3k_shapes.sdf`, DART-native collision, DART 6
dynamics, deactivation disabled, `--world-threads 16`,
`--max-contacts 12000`, `--max-contacts-per-pair 4`, 300 steps:

| Run | Collision backend | RTF | Final state |
| --- | --- | ---: | --- |
| #3172 merged baseline, no profile | DART native | `0.179381` | finite, hash `0x6a043ac1e7558218`, contacts `5005`, pairs `3003` |
| #3183 head `0b158d44126`, no profile | DART native | `0.127794` | finite, same hash, contacts `5005`, pairs `3003` |
| #3183 head `0b158d44126`, text profile | DART native | `0.128960` | finite, same hash, contacts `5005`, pairs `3003`; `build contact constraints` `605.13 ms`, `shared-body check` `388.96 ms`, `parallel reset` `108.10 ms`, `solveConstrainedGroups` `318.56 ms`, `collide` `477.22 ms` |
| Metadata-cache candidate rebased on #3185 base, no profile | DART native | `0.186148`; pre-#3184 rerun `0.182836`; earlier repeats `0.202726`, `0.187945`, `0.204494`, `0.204368`, `0.212991`, `0.195666` | finite, same hash, contacts `5005`, pairs `3003` |
| Metadata-cache candidate rebased on #3185 base, text profile | DART native | `0.185689`; pre-#3184 rerun `0.203355` | finite, same hash, contacts `5005`, pairs `3003`; current `build contact constraints` `266.09 ms`, `shared-body check` `150.77 ms`, `parallel reset` `74.14 ms`, `solveConstrainedGroups` `271.31 ms`, `collide` `377.69 ms` |
| Local skip-flags/body-set experiment, no profile | DART native | `0.196971` current rerun; `0.196594` post-rebase rerun; `0.185274` retained-bucket rerun; `0.213820`, `0.215162`, `0.199266`, `0.198770` prior repeats | finite, same hash, contacts `5005`, pairs `3003` |
| Local skip-flags/body-set experiment, text profile | DART native | `0.171489` current noisy rerun; `0.205666` retained-bucket rerun; `0.189984`, `0.165932` noisy reruns; `0.212505`, `0.215980` prior runs | finite, same hash, contacts `5005`, pairs `3003`; current `build contact constraints` `307.89 ms`, `shared-body check` `193.68 ms`, `parallel reset` `73.30 ms`, `solveConstrainedGroups` `283.68 ms`, `collide` `406.00 ms`; retained-bucket profile had `build contact constraints` `239.58 ms`, `shared-body check` `138.92 ms`, `parallel reset` `68.59 ms` |
| Local axis-plane contact-bounds experiment, no profile | DART native | `0.223206` compact-bound rerun; `0.213167` prior axis-only rerun | finite, same hash, contacts `5005`, pairs `3003` |
| Local axis-plane contact-bounds experiment, text profile | DART native | `0.227034` compact-bound rerun; `0.207763` prior axis-only rerun | finite, same hash, contacts `5005`, pairs `3003`; latest projected contact-bound separation `16.32 ms`, finite-plane pairs `113.66 ms`, `collide` `268.50 ms`, `build contact constraints` `225.11 ms`, `solveConstrainedGroups` `245.09 ms` |
| Local default-surface version-cache experiment, no profile | DART native | `0.230490` | finite, same hash, contacts `5005`, pairs `3003` |
| Local default-surface version-cache experiment, text profile | DART native | `0.227376` | finite, same hash, contacts `5005`, pairs `3003`; `build contact constraints` `183.10 ms`, `shared-body check` `85.97 ms`, `shared-body surface scan` `54.71 ms`, `shared-body body scan` `26.36 ms`, `parallel reset` `65.45 ms`, `collide` `278.13 ms` |
| Local lazy DART shape-cache experiment, no profile | DART native | `0.234474`, `0.239789`, `0.230595` | finite, same hash, contacts `5005`, pairs `3003` |
| Local lazy DART shape-cache experiment, text profile | DART native | `0.240786` | finite, same hash, contacts `5005`, pairs `3003`; `collide` `240.55 ms`, `finite-plane pairs` `114.05 ms`, `build broadphase entries` `93.57 ms`, no separate `CollisionGroup update objects` scope |
| Local FreeJoint root-integration experiment, no profile | DART native | `0.236713`, `0.236395` post-lint rerun | finite, same hash, contacts `5005`, pairs `3003` |
| Local FreeJoint root-integration experiment, text profile | DART native | `0.236918` | finite, same hash, contacts `5005`, pairs `3003`; `World::step - Integrate positions` `201.67 ms`, `World::step - Integrate velocity` `180.50 ms`, `collide` `254.99 ms` |
| Local parallel surface-scan experiment, no profile | DART native | `0.251073` | finite, same hash, contacts `5005`, pairs `3003` |
| Local parallel surface-scan experiment, text profile | DART native | `0.252496` | finite, same hash, contacts `5005`, pairs `3003`; `build contact constraints` `146.61 ms`, `shared-body check` `68.71 ms`, `shared-body surface scan` `36.97 ms`, `shared-body body scan` `26.58 ms`, `parallel reset` `44.96 ms`, `collide` `249.49 ms` |
| Local broadphase cache-refresh-order hardening, no profile | DART native | `0.252928`, `0.248405` | finite, same hash, contacts `5005`, pairs `3003` |
| Local broadphase cache-refresh-order hardening, text profile | DART native | `0.242797` | finite, same hash, contacts `5005`, pairs `3003`; `DART native build broadphase entries` `119.05 ms`, `DART native finite-plane pairs` `116.19 ms`, `build contact constraints` `157.92 ms` |
| Local parallel-pair flag resize cleanup, no profile | DART native | `0.247575` | finite, same hash, contacts `5005`, pairs `3003` |
| Local parallel-pair flag resize cleanup, text profile | DART native | `0.247692` | finite, same hash, contacts `5005`, pairs `3003`; `collide` `260.19 ms`, `DART native build broadphase entries` `110.30 ms`, `DART native finite-plane pairs` `114.67 ms`, `DART native finite-plane collide workers` `34.75 ms`, `DART native finite-plane merge contacts` `45.76 ms` |
| Local single-reactive union-reset cleanup, no profile | DART native | `0.245203` | finite, same hash, contacts `5005`, pairs `3003` |
| Local single-reactive union-reset cleanup, text profile | DART native | `0.256963` | finite, same hash, contacts `5005`, pairs `3003`; `buildConstrainedGroups` `67.89 ms`, `build constrained group map` `38.89 ms`, `reset constraint unions` `3.260 ms`, `build contact constraints` `138.26 ms`, `collide` `254.00 ms` |
| Local single-reactive raw-root cleanup, no profile | DART native | `0.272246` | finite, same hash, contacts `5005`, pairs `3003` |
| Local single-reactive raw-root cleanup, text profile | DART native | `0.258153` | finite, same hash, contacts `5005`, pairs `3003`; `buildConstrainedGroups` `40.75 ms`, `build constrained group map` `19.99 ms`, `reset constraint unions` `2.658 ms`, `build contact constraints` `151.32 ms`, `collide` `264.97 ms` |
| #3183 local candidate, no profile | FCL primitive | `0.145341` | finite, hash `0x6088ea0177efa6a`, contacts `3003`, pairs `3003` |
| #3183 local candidate, no profile | Bullet | `0.144310` | finite, hash `0x11fdd70a9952f98e`, contacts `5005`, pairs `3003` |
| #3183 local candidate, no profile | ODE | `0.0100767` | finite, hash `0x2a3d53060f661c4c`, contacts `9009`, pairs `3003` |

Using the latest native reruns after the #3184 base update, the unpublished
metadata-cache candidate is about `1.4x` the #3183 parent on this noisy
workstation while preserving the final hash, contact count, and contact-pair
count. The scoped
profile shows the improvement concentrated in contact construction:
`build contact constraints` drops from `605.13 ms` to `266.09 ms`, the
`shared-body check` drops from `388.96 ms` to `150.77 ms`, and `parallel reset`
drops from `108.10 ms` to `74.14 ms`. The earlier #3183 scoped profile showed
the serial surface-parameter pass removed from the parallel steady-state path;
this follow-up then reduces the remaining parallel-eligibility scan and
repeated per-pair lookup work. The active no-deactivation scene is still far
below RTF `1`, so the next target remains the largest remaining active-step
costs: collision, integration, constrained-group solve, and remaining contact
construction overhead.

The skip-flags/body-set experiment keeps the no-profile result in the same
noisy range. The current scoped rerun was noisier (`0.171489`, with higher
contact-construction and collision timings), while the retained-bucket profile
recorded `0.205666` and reduced the local contact-construction slices versus
the metadata-cache profile. Treat this as the next local follow-up after the
metadata-cache slice, not as part of that publishable PR.

The axis-plane contact-bounds experiment keeps the same final hash while
cutting the projected contact-bound separation proof from the prior local
`46.31 ms` sample to `16.32 ms`. In the same profiled run, `collide` dropped
to `268.50 ms`, and the no-profile repeat reached RTF `0.223206`. Treat this
as a later collision follow-up after the contact-construction slices.

The default-surface version-cache experiment keeps the same final hash, contact
count, and pair count while reducing the measured surface portion of the
parallel eligibility scan. In the latest text-profile sample,
`build contact constraints` dropped from the axis-plane `225.11 ms` sample to
`183.10 ms`, `shared-body check` dropped from the repeated local `101.66 ms`
sample to `85.97 ms`, and the newly split surface scan was `54.71 ms`. The
fresh no-profile rerun reached RTF `0.230490`.

The lazy DART shape-cache experiment keeps the same final hash, contact count,
and pair count while removing the separate DART group object-update pass. The
latest scoped profile has no `CollisionGroup update objects` scope; the previous
same-session local profile measured that pass at `34.41 ms`, and the current
profile records `collide` at `240.55 ms`. No-profile repeats were in the same
high local range as the parent (`0.230595`-`0.239789`).

The FreeJoint root-integration experiment keeps the same final hash, contact
count, and pair count while trimming the active-scene integration path. The
latest scoped profile records `World::step - Integrate positions` at
`201.67 ms`; the previous same-session lazy shape-cache profile recorded
`209.85 ms` for the same scope. No-profile RTF remains in the same noisy local
range (`0.236395`-`0.236713`).

The parallel surface-scan experiment keeps the same final hash, contact count,
and pair count while reducing the remaining surface-scan portion of contact
construction. The latest scoped profile records `shared-body surface scan` at
`36.97 ms`, down from the default-surface version-cache sample's `54.71 ms`
and the FreeJoint parent sample's roughly `57 ms` range, while
`build contact constraints` drops from the FreeJoint parent profile's
`180.12`-`179.86 ms` range to `146.61 ms`.

The broadphase cache-refresh-order hardening keeps the same final hash, contact
count, and pair count while closing a stale-cache edge in the parallel
DART-native broadphase path. It is correctness hardening rather than a proven
speedup: the no-profile samples remain in the same local range
(`0.248405`-`0.252928`), while the text-profile sample is noisier and records
RTF `0.242797`.

The parallel-pair flag resize cleanup keeps the same final hash, contact count,
and pair count while removing a redundant zero-fill before parallel finite-plane
collision workers. Treat it as a small scratch-buffer cleanup, not a standalone
speedup: the profiled sample remains in the same noisy local range, with
`DART native finite-plane pairs` at `114.67 ms`.

The single-reactive union-reset cleanup keeps the same final hash, contact
count, and pair count while reducing the constrained-group union reset in the
all-single-reactive-contact path. The latest profile records
`reset constraint unions` at `3.260 ms`, down from the prior scratch-buffer
sample's `14.15 ms`, and `buildConstrainedGroups` at `67.89 ms`.

The single-reactive raw-root cleanup keeps the same final hash, contact count,
and pair count while avoiding a per-group `shared_ptr` root handle in the same
all-single-reactive-contact path. That path already solves from
`mSingleReactiveSkeleton` and the temporary union index, so the generic
`mRootSkeleton` handle is unnecessary refcount churn. The latest profile records
`buildConstrainedGroups` at `40.75 ms` and `build constrained group map` at
`19.99 ms`, down from the union-reset cleanup sample's `67.89 ms` and
`38.89 ms`.

A rejected eager-transform experiment updated `BodyNode` transforms immediately
after position integration to move work out of the next collision prepass. It
preserved the final hash but regressed the no-profile active issue scene to RTF
`0.244883`, so the edit was reverted and should not be retried without a
different guard.

Rejected local scratch experiments in the same area:

- `perf/dart6-contiguous-contact-pair-ranges` tracked contiguous candidate
  ranges but regressed the profiled contact-build path (`build contact
  constraints` `159.60 ms`, `parallel reset` `50.25 ms`).
- `perf/dart6-contact-pair-bucket-generations` replaced contact-pair bucket
  clearing with generation stamps but regressed `collect contact candidates` to
  `65.92 ms`.
- `perf/dart6-direct-single-contact-lcp` specialized one-contact identity-body
  direct LCP fill but regressed the profiled solve path (`solveConstrainedGroups`
  `248.39 ms`, `Construct LCP` `65.73 ms`).
- `perf/dart6-parallel-reset-reuse-precheck` removed duplicate reusable-contact
  checks from the threaded reset path but did not improve the target profile
  (`build contact constraints` `154.86 ms`, `parallel reset` `50.48 ms`).
- `perf/dart6-activation-skip-short-circuit` made the single-reactive
  activation dependency check lazy, but the target activation scope regressed
  to `19.12 ms`.
- `perf/dart6-colliding-flag-skip-repeat` skipped repeated deprecated
  colliding-flag writes but regressed `collect contact candidates` to
  `60.55 ms`.
- `perf/dart6-single-free-forward-dynamics` skipped the dynamic `FreeJoint`
  `updateJointForceFD()` no-op for one-body roots, but the no-profile run
  regressed to RTF `0.249708` and the profile stayed within noise.
- An automatic joint-constraint cache revision experiment replaced the
  per-step solver skeleton hash with a solver membership revision plus
  `Skeleton`'s global structural revision, but it required a new `Skeleton`
  friendship and did not improve the target scene. No-profile RTF was
  `0.250410` and `0.260183`, while the text-profile run recorded RTF
  `0.246615` with `scan joint constraints` already down to only `830.94 us`.
- `perf/dart6-single-reactive-group-last-cache` cached the previous
  single-reactive skeleton while building constrained groups, but the
  no-profile issue scene regressed to RTF `0.230829` despite preserving the
  final hash after the focused `test_ConstraintSolver` build and CTest passed.

On the original default-sleeping target command, the same current local head
reaches RTF `61.1724` for 3000 steps with DART-native collision, advances
`3000 / 3000` frames, ends finite with hash `0x131b6af79a44ff90`, and has all
`3003 / 3003` mobile skeletons resting with zero final contacts.

The unpublished metadata-cache head has passed `pixi run lint`, `pixi run
test-all` on the current #3185-based branch, the focused `test_ConstraintSolver`
build and CTest, and exact issue-scene no-profile and profile reruns after
rebasing directly onto the #3184 base (`0.186148` no profile, `0.185689` text
profile). The current branch is rebased onto #3185, which only changed unrelated
`dart6_backport_audit` dev-task documentation. The current `pixi run test-all`
pass completed C++ tests `115/115` and Python tests `60/60`; before the #3184
rebase, the same metadata-cache slice also passed
`DART_PARALLEL_JOBS=24 CTEST_PARALLEL_LEVEL=24 CMAKE_BUILD_PARALLEL_LEVEL=24 pixi run test-all`.

The local skip-flags/body-set experiment has passed the targeted
`cmake --build build/default/cpp/Release --parallel 5 --target test_ConstraintSolver contact_benchmark`
build, focused `test_ConstraintSolver` CTest, and the two exact-scene benchmark
runs above. After rebasing the unpublished skip-flags branch onto
`perf/dart6-contact-pair-metadata-cache` at `9d66466fb37`, the focused build
and `test_ConstraintSolver` CTest were rerun successfully on the current local
skip-flags branch. The current post-rebase no-profile rerun recorded RTF
`0.196971`; the current text-profile rerun recorded RTF `0.171489`; both kept
the same final hash, contacts, and pair count. After the retained-bucket
skip-check refinement,
`N=$(python3 scripts/parallel_jobs.py); DART_PARALLEL_JOBS=$N CTEST_PARALLEL_LEVEL=$N CMAKE_BUILD_PARALLEL_LEVEL=$N pixi run test-all`
passed with C++ tests `115/115` and Python tests `60/60`.

The local axis-plane contact-bounds experiment has passed:
`cmake --build build/default/cpp/Release --parallel 5 --target test_Collision test_DARTCollisionDetector contact_benchmark`,
`ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_Collision|test_DARTCollisionDetector)$'`,
and the exact issue-scene no-profile/profile benchmark runs above.

The local default-surface version-cache experiment has passed:
`cmake --build build/default/cpp/Release --parallel 5 --target test_ConstraintSolver contact_benchmark`,
`ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_ConstraintSolver$'`,
and the exact issue-scene no-profile/profile benchmark runs above. The focused
solver regression heats the default-surface cache, mutates the contact dynamics,
and verifies the versioned cache tracks the same state as a cold-cache reference
after the mutation.

The local lazy DART shape-cache experiment has passed:
`cmake --build build/default/cpp/Release --parallel 5 --target contact_benchmark test_Collision test_DARTCollisionDetector test_ConstraintSolver`,
`ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_Collision|test_DARTCollisionDetector|test_ConstraintSolver)$'`,
and the exact issue-scene no-profile/profile benchmark runs above. Existing
DART collision regressions cover changed shape geometry and `ShapeNode`
relative-transform updates after collision groups already exist.

The local FreeJoint root-integration experiment has passed:
`pixi run lint`,
`cmake --build build/default/cpp/Release --parallel 5 --target test_Joints contact_benchmark`,
`ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_Joints$'`,
and the exact issue-scene no-profile/profile benchmark runs above. The focused
regression compares `Skeleton::integratePositions()` against the public
stateless integration overload for identity-frame, translated-parent, and
fully offset FreeJoint frame configurations.

The local parallel surface-scan experiment has passed:
`pixi run lint`,
`cmake --build build/default/cpp/Release --parallel 5 --target test_ConstraintSolver contact_benchmark`,
`ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_ConstraintSolver$'`,
and the exact issue-scene no-profile/profile benchmark runs above.

The local broadphase cache-refresh-order hardening has passed:
`cmake --build build/default/cpp/Release --parallel 5 --target test_Collision test_DARTCollisionDetector contact_benchmark`,
`ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_Collision|test_DARTCollisionDetector)$'`,
`cmake --build build/default/cpp/Release --parallel 5 --target test_DARTCollisionDetector contact_benchmark`,
`ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_DARTCollisionDetector$'`,
and the exact issue-scene no-profile/profile benchmark runs above. The focused
unit regression forces the parallel finite-plane path, then changes all cached
identity-relative `ShapeNode` transforms before the next threaded collide.

The local parallel-pair flag resize, single-reactive union-reset, and
single-reactive raw-root cleanups have passed:
`pixi run lint`,
`cmake --build build/default/cpp/Release --parallel 5 --target test_ConstraintSolver contact_benchmark`,
`ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_ConstraintSolver$'`,
and the exact issue-scene no-profile/profile benchmark runs above. The
parallel-pair flag resize also passed the focused collision build and
`ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_Collision|test_DARTCollisionDetector)$'`.

An earlier fixed-support contact-build relaxation crashed because the parallel
worker indexed `thread_local` contact-pair scratch storage from the worker
thread. The current candidate fixes that by capturing explicit main-thread
scratch pointers for the parallel rebuild and keeps the fixed-support
relative-velocity skip decision serial.

#3170 parallelizes DART-native broadphase entry construction for large
collision groups. It fills a scratch entry vector in collision-object order,
then performs the same serial finite/plane/other partition, preserving
deterministic pair order and final contact state while reducing the measured
native broadphase setup cost.

#3154 trims default contact construction by replacing the
per-step linear-scan default-surface-property cache with a small direct-mapped
`ShapeNode` cache, plus remembering the most recent first-side and second-side
lookups inside each solver update. Cache misses recompute the same surface
property predicate as before, so collisions, surface parameters, contact counts,
and final hashes are unchanged.

Active issue-scene evidence,
`.deps/gz-sim/examples/worlds/3k_shapes.sdf`, DART-native collision, DART 6
dynamics, deactivation disabled, `--world-threads 16`,
`--max-contacts 12000`, `--max-contacts-per-pair 4`, 300 steps:

| Run | Collision backend | RTF | Final state |
| --- | --- | ---: | --- |
| #3153 parent, text profile | DART native | `0.109055` | finite, hash `0x6a043ac1e7558218`, contacts `5005`, pairs `3003`; `build contact constraints` `532.65 ms` |
| #3154 current head, text profile | DART native | `0.110145` | finite, same hash, contacts `5005`, pairs `3003`; `build contact constraints` `508.96 ms`, surface params `143.01 ms`, serial fallback `360.00 ms` |
| #3154 current head, no profile | DART native | `0.109151` latest repeat, `0.105346` prior repeat | finite, same hash, contacts `5005`, pairs `3003` |
| #3154 current head, no profile | FCL primitive | `0.093295` | finite, hash `0x6088ea0177efa6a`, contacts `3003`, pairs `3003` |
| #3154 current head, no profile | Bullet | `0.0883701` | finite, hash `0x11fdd70a9952f98e`, contacts `5005`, pairs `3003` |
| #3154 current head, no profile | ODE | `0.00460932` | finite, hash `0x2a3d53060f661c4c`, contacts `9009`, pairs `3003` |

Current-head DART-native is about `1.17x` FCL primitive, `1.24x` Bullet, and
`23.7x` ODE on the latest active issue-scene rerun. The local slice is a small
contact-construction win, not a step-change; the larger remaining active-scene
costs are still collision, constrained-group solve, and integration.

#3153 targets active DART-native finite-shape-vs-plane collision
for the default solver filter. For exact `BodyNodeCollisionFilter` queries where
all finite objects belong to mobile skeletons, all plane objects belong to
static skeletons, there are no explicit body-node pair blacklists, and the
solver's resting-contact filter is inactive, the DART-native backend can skip
the serial body-node filter prepass. Those conditions are exactly the default
active issue-scene shape. Filter subclasses, explicit blacklists, same-body or
same-skeleton pairs, non-collidable bodies, non-BodyNode objects, and
solver-resting filtering all stay on the legacy filtered path.

Active issue-scene evidence,
`.deps/gz-sim/examples/worlds/3k_shapes.sdf`, DART 6 dynamics, constraints,
and solver, deactivation disabled, `--world-threads 16`,
`--max-contacts 12000`, `--max-contacts-per-pair 4`, 300 steps:

| Run | Collision backend | RTF | Final state |
| --- | --- | ---: | --- |
| #3152 parent, text profile | DART native | `0.104728` | finite, hash `0x6a043ac1e7558218`, contacts `5005`, pairs `3003`; `collide` `655.08 ms`, finite-plane pairs `326.39 ms`, finite-plane prefilter `98.97 ms` |
| #3153 current head, text profile | DART native | `0.109055` | finite, same hash, contacts `5005`, pairs `3003`; `collide` `625.30 ms`, finite-plane pairs `296.06 ms`, no finite-plane prefilter scope |
| #3153 current head, no profile | DART native | `0.108879` | finite, same hash, contacts `5005`, pairs `3003` |
| #3153 current head, no profile | FCL primitive | `0.0899919` | finite, hash `0x6088ea0177efa6a`, contacts `3003`, pairs `3003` |
| #3153 current head, no profile | Bullet | `0.0913725` | finite, hash `0x11fdd70a9952f98e`, contacts `5005`, pairs `3003` |
| #3153 current head, no profile | ODE | `0.00466557` | finite, hash `0x2a3d53060f661c4c`, contacts `9009`, pairs `3003` |

Current-head DART-native is about `1.21x` FCL primitive, `1.19x` Bullet, and
`23.3x` ODE on this active issue-scene rerun.

Focused correctness evidence: `test_Collision` now compares serial and
four-thread DART-native finite-plane contacts for BodyNode-backed mobile boxes
against a static plane using the exact default `BodyNodeCollisionFilter`. The
same test also verifies that an explicit body-node blacklist and a custom
`BodyNodeCollisionFilter` subclass still match the serial filtered path and
remove the targeted contacts.

#3152 targets the default settled-scene path. Once the original 3k issue scene
has fully gone to sleep, the all-resting fast path no longer rebuilds the
last-step per-skeleton resting snapshot every frame when that snapshot is
already valid. The all-resting kinematic snapshot remains the correctness guard
for pose, velocity, collision-option, collision-filter, and structural edits.

Default sleeping issue-scene evidence,
`.deps/gz-sim/examples/worlds/3k_shapes.sdf`, DART 6 dynamics, constraints,
and solver, `--world-threads 16`, `--max-contacts 12000`,
`--max-contacts-per-pair 4`, 3000 steps:

| Run | Collision backend | RTF | Final state |
| --- | --- | ---: | --- |
| #3151 parent, text profile | DART native | `10.4306` | finite, hash `0x131b6af79a44ff90`, resting `3003 / 3003`, contacts `0`; all-resting fast path `228.74 ms` |
| #3152 current head, text profile | DART native | `56.2441` | finite, same hash, resting `3003 / 3003`, contacts `0`; all-resting fast path `106.99 us`, readiness check `1.952 ms` |
| #3152 current head, no-profile | DART native | `59.2748` latest rerun, `55.5385` prior run | finite, same hash, resting `3003 / 3003`, contacts `0` |
| #3152 current head, no-profile | FCL primitive | `57.3796` | finite, hash `0x266da31836a314a6`, resting `3003 / 3003`, contacts `0` |
| #3152 current head, no-profile | Bullet | `5.49559` | finite, hash `0x2375f1927218cd43`, resting `3003 / 3003`, contacts `0` |
| #3152 current head, no-profile | ODE | `0.02744` | finite, hash `0x4e5f6556657720`, resting `3003 / 3003`, contacts `0` |

The latest DART-native no-profile rerun is about `1.03x` FCL primitive,
`10.8x` Bullet, and `2160x` ODE on this fully rested workload. The earlier
DART repeat was `55.5385`, so DART/FCL on this short all-resting path is close
and timing-noise sensitive; active no-deactivation measurements remain the
primary collision-backend comparison.

#3147 targets active contact-construction cost by reusing exact built-in
default `ContactConstraint` objects across steps while preserving exact ODE
final-state hashes. #3148 targets DART-native collision transform setup for
identity-relative `ShapeNode` collision objects by reusing the owning
`BodyNode` world transform. #3149 rebuilds large independent built-in default
contact sets by collision pair in parallel, while custom contact handlers,
small contact sets, and pairs that share a non-skipped body stay on the
existing serial path.

#3150 attacks the measured DART-native finite-plane merge hot path. Queries
that can prove finite-plane contact footprints are mutually disjoint may keep
the existing per-pair duplicate check while bypassing the global
duplicate-contact grid for those pair results. Accepted fast-path contacts are
still published to the global duplicate index when later fallback pair phases or
collision filters may need that state. Overlapping, multi-plane, and cross-phase
queries keep the existing global duplicate path.

#3151 attacks the remaining default contact-build cost. It keeps moving fixed
supports observable, but skips relative-velocity work for zero-velocity fixed
supports, caches repeated default surface-property checks within each solver
update, and makes the threaded default-contact rebuild path avoid cold-start
contact allocation.

Latest exact issue-scene evidence
`.deps/gz-sim/examples/worlds/3k_shapes.sdf`, DART 6 dynamics, constraints,
and solver, `--steps 300`, `--world-threads 16`,
`--max-contacts 12000`, `--max-contacts-per-pair 4`, deactivation disabled.
ODE is included here because it is the downstream backend baseline.

| Run | Collision backend | RTF | Final state |
| --- | --- | ---: | --- |
| #3149 parent, active no-profile | DART native | `0.0984786` latest rerun, `0.0982859` prior run | finite, hash `0x6a043ac1e7558218`, contacts `5005`, pairs `3003` |
| #3150 parent, active no-profile | DART native | `0.104694` | finite, same hash, contacts `5005`, pairs `3003` |
| #3150 parent, active text profile | DART native | `0.104939` | finite, same hash, contacts `5005`, pairs `3003`; `collide` `660.38 ms`, finite-plane merge `113.36 ms`, contact-bound separation check `24.44 ms` |
| #3151 current head, active no-profile | DART native | `0.110902` | finite, same hash, contacts `5005`, pairs `3003` |
| #3151 current head, active text profile | DART native | `0.107484` | finite, same hash, contacts `5005`, pairs `3003`; `build contact constraints` `537.94 ms`, default surface params `176.20 ms`, serial fallback `355.90 ms` |
| #3151 current head, active no-profile | FCL primitive | `0.0978488` | finite, hash `0x6088ea0177efa6a`, contacts `3003`, pairs `3003` |
| #3151 current head, active no-profile | Bullet | `0.0865244` | finite, hash `0x11fdd70a9952f98e`, contacts `5005`, pairs `3003` |
| #3151 current head, active no-profile | ODE | `0.00466348` | finite, hash `0x2a3d53060f661c4c`, contacts `9009`, pairs `3003` |

Current-head DART-native is about `1.13x` FCL primitive, `1.28x` Bullet, and
`23.8x` ODE on the active issue scene. It is still far below RTF `1` with
deactivation disabled, so the follow-up target is the remaining DART 6
constraint pipeline cost rather than declaring issue #3056 complete. The latest
text profile shows `World::step - Solve constraints` at `2.065 s` of `2.790 s`
over 300 steps. Within that, `updateConstraints` is `1.347 s`, `collide` is
`666.62 ms`, `build contact constraints` is `537.94 ms`, and
`solveConstrainedGroups` is `513.23 ms`.

Focused correctness evidence: `test_ConstraintSolver` now compares a serial
world against a four-thread world with 160 independent default box-plane
contacts over repeated steps, including final positions, velocities,
transforms, spatial velocities, and contact counts. ODE remains the downstream
comparison baseline and should be included in each refreshed performance table,
even when the current slice only changes the DART-native backend.

Focused collision evidence: `test_Collision` compares serial and threaded
DART-native finite-plane contacts in legacy order, and
`test_DARTCollisionDetector` keeps duplicate-contact coverage for overlapping
finite shapes near duplicate-grid boundaries. The current candidate also passed
`test_CollisionGroups`.

Latest active issue-scene evidence with DART-native collision, DART 6 dynamics,
300 active steps, `--world-threads 16`, `--max-contacts 12000`,
`--max-contacts-per-pair 4`, and deactivation disabled:

| Run | RTF | Final state |
| --- | ---: | --- |
| #3142 stack, no profile | `0.0696281` | finite, hash `0x6a043ac1e7558218`, contacts `5005`, pairs `3003` |
| #3143 stack, no profile | `0.0751396` | finite, same hash, contacts `5005`, pairs `3003` |
| #3144 contact-merge candidate, no profile | `0.0817217` | finite, same hash, contacts `5005`, pairs `3003` |
| #3142 stack, text profile | `0.0734006` | finite, same hash, contacts `5005`, pairs `3003` |
| #3143 stack, text profile | `0.0754405` | finite, same hash, contacts `5005`, pairs `3003` |
| #3144 contact-merge candidate, text profile | `0.0824554` | finite, same hash, contacts `5005`, pairs `3003` |

The profile movement is concentrated where expected:
the DART-native `collide` scope drops from about `1.680 s` on #3142 to
`1.629 s` on #3143 and `1.101 s` on the #3144 contact-merge candidate over 300
active steps. The next largest measured costs are now `build contact constraints` at
about `762 ms`, `solveConstrainedGroups` at about `562 ms`, and
velocity/position integration at about `741 ms` combined, so the next larger
wins should continue in contact construction, constrained-group solving,
integration, or deeper native collision algorithm changes.

#3133 parallelizes finite-shape-vs-plane collision queries for the existing
DART-native backend while keeping low `maxNumContacts` queries on the legacy
serial early-exit path. #3135 makes explicit per-pair contact caps select the
deepest contact plus spatially distributed support points instead of preserving
backend iteration-order truncation. #3139 reduces broadphase setup work by
computing transformed cached local bounds from center and half-extents instead
of visiting all eight local bounding-box corners. #3140 enables the existing
direct LCP assembly path for single-free-body groups, but only when every
constraint is an exact built-in `ContactConstraint`. #3141 trims two small
DART-native collision hot-path costs: unused scratch-result lookup caches and
single-plane pair-index division. #3142 skips redundant parallel-safety scans
once the active contact set and built constrained groups prove they are exact
built-in fixed-support contact groups. #3143 caches a compact primitive shape
kind for DART-native plane dispatch. #3144 reduces serial contact-merge
duplicate-grid probes while preserving duplicate behavior.
#3146 targets cached all-resting step readiness by replacing the repeated
mobile-DOF velocity scan with an external velocity-edit generation guard.
#3147 targets active contact-construction overhead by reusing exact built-in
default contact constraints and avoiding redundant pair-index work.

Recent rejected local experiments are kept as named stashes, not PRs: caching
world-plane transforms preserved final hashes but regressed/noised the active
RTF; rewriting local-point computation avoided inverse transforms but changed
the ODE final-state hash and was dropped; and parallel contact-constraint
construction was repaired past the thread-local scratch misuse but did not
produce convincing active RTF/profile gains. Those results point the next
larger work back toward native collision algorithms and safe constraint-build
structure, rather than cache-only edits.

This slice is a bounded DART-native collision hot-path improvement, not the
larger native-detector port. It parallelizes finite-shape-vs-plane collision
queries for the existing `dart/collision/dart/` backend when the query has many
plane pairs and contact results are requested. Collision filters are applied
serially before parallel work so the normal `ConstraintSolver` default filter
does not disable the threaded path. It also adds a direct cached-shape plane
dispatch path and a squared-distance duplicate-contact check. Pair results are
merged serially in the same order as the legacy path, so contact ordering and
final-state hashes remain deterministic.

Threading is wired through the existing `World::setNumSimulationThreads()` /
`ConstraintSolver::setNumSimulationThreads()` control, only for
`DARTCollisionDetector`. The default one-thread path remains available and uses
the same serial code path for small/filter-sensitive queries.

The original issue scene remains the primary active benchmark:
`/tmp/3k_shapes.sdf`, with `3003` mobile sphere/box/cylinder bodies plus the
static ground plane from the issue report. Command shape:
`pixi run ex contact_benchmark -- /tmp/3k_shapes.sdf --steps 300 --warmup 0
--checkpoint 0 --quiet --collision <engine> --sdf-plane-shapes --world-threads
16 --max-contacts 12000 --disable-deactivation`.

Current active original-scene evidence, all with DART 6 dynamics, constraints,
and solver; only collision detection changes:

| Run | RTF | Final state |
| --- | ---: | --- |
| #3129 DART native active baseline | `0.05735` | finite, hash `0x6b50e84cd691f6e2`, contacts `5005`, pairs `3003` |
| Current DART native active, 16 threads | `0.0624274` profile run | finite, hash `0x6b50e84cd691f6e2`, contacts `5005`, pairs `3003` |
| Current DART native default sleeping, 16 threads | `1.88586` latest rerun, `1.96845` prior run | finite, hash `0x131b6af79a44ff90`, resting `3003 / 3003`, contacts `0`, frame delta `3000 / 3000` |

A no-profile rerun after the transform-copy review fix measured RTF
`0.059358`, with the same final hash, contact count, and contact-pair count. A
prior no-profile rerun after the solver-filter fix measured RTF `0.0598292`;
earlier no-rebuild repeats after the direct plane dispatch were RTF
`0.0599254`, `0.0577443`, and `0.058838`, all with the same DART-native final
hash and contact counts. Active-thread scaling on the same scene was:

| Threads | RTF | Final state |
| ---: | ---: | --- |
| 1 | `0.0219988` | finite, hash `0x6b50e84cd691f6e2`, contacts `5005`, pairs `3003` |
| 2 | `0.0308776` | finite, hash `0x6b50e84cd691f6e2`, contacts `5005`, pairs `3003` |
| 4 | `0.037332` | finite, hash `0x6b50e84cd691f6e2`, contacts `5005`, pairs `3003` |
| 8 | `0.0476012` | finite, hash `0x6b50e84cd691f6e2`, contacts `5005`, pairs `3003` |
| 16 | `0.0593455` | finite, hash `0x6b50e84cd691f6e2`, contacts `5005`, pairs `3003` |

Contact-selection follow-up evidence:

| Run | RTF | Final state |
| --- | ---: | --- |
| #3133 branch, explicit cap disabled | `0.0576553` | finite, hash `0x6b50e84cd691f6e2`, contacts `5005`, pairs `3003` |
| Contact selection, `--max-contacts-per-pair 4` | `0.0551375` | finite, same hash `0x6b50e84cd691f6e2`, contacts `5005`, pairs `3003` |

The explicit four-contact cap does not change the original issue scene because
the DART-native primitive plane pairs already produce at most three contacts
per pair. The value of this slice is correctness infrastructure for future
default-on cap/manifold work: focused coverage proves the cap keeps the deepest
support point and a spatially spread support point instead of taking the first
backend contacts. The latest review fix also caps the selection target by the
remaining global contact slots, so a query with only one global slot left keeps
the deepest representative contact instead of letting backend order truncate
the selected set. Duplicate backfill keeps the reserve contacts in the same
depth/spread priority order, so overlapping pairs do not fall back to backend
iteration order after one selected contact is skipped as a duplicate.

The measured win is real but modest. The profiler moved the inclusive
`collide` scope from the #3129 baseline `1.840 s` to `1.585 s` over 300 active
steps, while `updateConstraints` moved from `2.650 s` to `2.446 s`. Remaining
active hotspots are still native collision, contact-constraint construction,
and constrained-group solving. This is worth a narrowly scoped PR because it
adds deterministic scaling for the exact issue workload, but it is not the main
optimization endpoint.

Correctness guardrails for this slice:

- The benchmark consumes the final state and preserves the exact DART-native
  final hash, contact count, and contact-pair count on the original issue scene.
- `test_Collision` adds a serial-vs-threaded DART-native finite-plane contact
  regression with more than the parallelization threshold and a default
  `BodyNodeCollisionFilter`, then compares contact object identity, point,
  normal, and penetration depth in legacy order.
- A `--max-contacts 1` exact-scene check preserves early exit with final
  contacts `1`, contact cap hit `true`, and final contact pairs `1`.
- `pixi run build-tests` passed after the current changes.
- `pixi -q run ctest --test-dir build/default/cpp/Release -R
  'test_Collision|test_ContactConstraint|test_ConstraintSolver'
  --output-on-failure` passed after the current changes.

The current slice is worth a small PR if kept narrowly scoped, but it is not the
major win. The north-star path remains importing/porting the DART 7 native
collision feature set into DART 6.20's dependency-free
`dart/collision/dart/` backend with equal or better coverage and benchmarks.

The previous managed PR, #3071 `perf/dart6-parallel-islands`, landed as the
parallel-stepping follow-up after #3111 and #3112 added the core
`setNumSimulationThreads` path on the base branch. Its evidence showed that the
thread knob preserves final state and has a real, bounded scaling window on
active independent articulated workloads; the original #3056 scene remains
mostly collision/resting-fast-path dominated.

Current #3071 evidence on an Intel i7-13800H (14 cores / 20 logical CPUs),
pinned to CPUs 0-15:

- Original #3056 SDF, Bullet collision detection, DART 6 dynamics/constraints,
  default deactivation, 3000 steps: RTF `1.50858` at 1 thread, `1.52074` at 2,
  `1.5381` at 4, `1.55879` at 8, and `1.51651` at 16. All runs advanced
  exactly `3000 / 3000` frames, ended finite, and matched final hash
  `0x2375f1927218cd43`. This scene is already resting-fast-path dominated, so
  thread scaling is intentionally modest.
- Same original SDF with `--disable-deactivation`, 300 active steps: RTF
  `0.0228284` at 1 thread, `0.0489761` at 8, and `0.046549` at 16, with
  identical final hash `0x1b1e6f3c78c0e01e` and `5005` final contacts across
  `3003` contact pairs. This shows a real active-pipeline speedup to 8 threads,
  while also showing that active contact/collision work remains below real
  time.
- Deterministic articulated workload
  `SCENE=chains LINKS=8 THREADS=N pixi run bm-boxes-headless 16 300 0`:
  `1793.239 ms` at 1 thread, `1353.723 ms` at 2, `1293.283 ms` at 4,
  `1006.875 ms` at 8, and `1182.029 ms` at 16, with identical final checksum
  at every thread count. Peak local speedup is `1.78x` at 8 threads; 16 threads
  regresses on this machine.
- Larger deterministic articulated workload
  `SCENE=chains LINKS=8 THREADS=N pixi run bm-boxes-headless 24 300 0`:
  `4986.764 ms` at 1 thread, `3985.350 ms` at 2, `3439.460 ms` at 4,
  `3134.201 ms` at 8, `3085.127 ms` at 12, and `3378.107 ms` at 16, again
  with identical final checksum at every thread count. This confirms a useful
  scaling window through 8-12 threads (`1.62x` peak), while 16 logical threads
  still regresses.
- A still larger check
  `SCENE=chains LINKS=8 THREADS=N pixi run bm-boxes-headless 32 200 0`:
  `9134.861 ms` at 1 thread, `6931.920 ms` at 4, `6753.507 ms` at 8,
  `6407.391 ms` at 12, and `6674.239 ms` at 16, with identical final checksum.
  This supports the same conclusion: the PR is beneficial and scales on larger
  independent articulated workloads, but the current implementation should be
  described as a mid-core scaling win rather than a linear all-core speedup.
- Smaller workloads are not good proof points: 512 falling boxes improved only
  from `8806.970 ms` to `8168.554 ms` at 8 threads and regressed at 16, while
  the 64-chain workload regressed at every tested thread count. The PR evidence
  should present this caveat instead of implying unlimited CPU scaling.

Native-collision follow-up: continue porting DART 7 native algorithms into
DART 6.20's `dart/collision/dart/` path in dependency-free slices. The merged
native slices added finite-shape broadphase filtering, plane/sphere/box/cylinder
contacts, per-thread scratch reuse, faster active contact aggregation,
DART-native capsule primitive contacts, and cached native-shape metadata. The
active stacked follow-ups are finite-plane query threading, representative
contact selection for explicit caps, and faster cached AABB transform setup.
The next larger native slices should keep mining DART 7 for missing shape
support, distance/CCD/raycast coverage, and manifold behavior while preserving
gz-physics-facing API compatibility.
- Exact issue scene used for the current evidence: `/tmp/3k_shapes.sdf`,
  3003 mobile sphere/box/cylinder bodies plus the static ground plane from the
  original issue report. The ground link warning about missing `<inertial>` is
  from that static ground model in the issue input; the mobile bodies have
  inertials.
- Current exact-scene native command:
  `pixi run ex contact_benchmark -- /tmp/3k_shapes.sdf --steps 3000 --warmup 0 --checkpoint 0 --quiet --collision dart --sdf-plane-shapes --world-threads 16 --max-contacts 12000`.
  This uses the dependency-free DART collision backend for collision detection;
  DART 6 also owns dynamics, constraints, sleep/deactivation, and the LCP solve.
- Current exact-scene native result, default deactivation enabled: RTF
  `1.88616`, finite final state, final hash `0x131b6af79a44ff90`, final
  resting `3003 / 3003`, final contacts `0`, and frame/time advanced exactly
  `3000 / 3000` steps. The earlier same-scene Bullet comparison on the merged
  native slice gave RTF `1.5567`.
- Same exact command with `--world-threads 1` gives RTF `1.46391`; with
  `--world-threads 4` gives RTF `1.49184`. The current win is therefore from
  prompt deactivation of the initially settled issue scene, not from thread
  scaling.
- Same exact command with `--steps 300 --disable-deactivation --profile` gives
  current DART native RTF `0.0536378`, finite final state, hash
  `0x6b50e84cd691f6e2`, final contacts `5005`, and final contact pairs `3003`.
  The current Bullet comparison gives RTF `0.0607573`, hash
  `0x1b1e6f3c78c0e01e`, final contacts `5005`, and final contact pairs `3003`.
- Exact-scene final dump comparison, default-on vs disabled deactivation,
  matched all `3003` mobile shapes: max position delta
  `3.4147863965736968e-06` m, mean position delta
  `1.8350550992622476e-06` m, max quaternion L2 delta
  `1.9714098667727154e-06`.
- Current default-sleeping profiler shape on the exact scene: only `3` full
  constraint/collision solves, followed by `2997` all-resting cached steps.
  Inclusive simulation time was `1.588 s`; all-resting readiness was `1.249 s`,
  all-resting fast path was `267 ms`, and the three native collision calls were
  `14.33 ms` total. The next measured default-sleeping hotspot is the readiness
  scan, not the LCP solve.
- FCL and ODE exact-scene comparison: FCL primitive did not complete setup
  within a 180 s timeout. ODE completed but measured RTF `0.0257735`, far below
  DART native and Bullet on this scene.
- Correctness coverage added for the prompt initial-rest path:
  `IslandDeactivation.InitiallySettledShallowContactCanSleepPromptly` proves a
  shallow zero-velocity support contact can consume initial dwell but still
  takes one final solved impulse before freezing.
  `IslandDeactivation.InitialMobilePairContactDoesNotSleepPromptly` and
  `IslandDeactivation.InitialWallContactDoesNotSleepPromptly` prove the dwell
  credit is not granted to unsupported mobile-mobile contacts or vertical wall
  contacts. The existing
  `IslandDeactivation.UnconvergedContactClearsSleepCandidate` guards the deep
  penetration case.
- Downstream gate status on this branch: gz-physics passed in the previous
  full `pixi run -e gazebo test-gz` attempt. The gz-sim portion initially
  failed before exercising physics because the source-built gz-physics install
  placed engine plugins under `lib64` and left the unversioned installed engine
  alias broken; `scripts/run_gz_sim_task.sh` now discovers `lib64` and includes
  the valid source-build plugin alias for the local integration gate. A fresh
  `pixi run -e gazebo test-gz-sim` rerun passed
  `INTEGRATION_entity_system`.

- PR0 `jslee02/issue-3056-baseline-sdf-perf` / #3089 adds the baseline
  benchmark, GUI verifier, reset path, HUD, drop-state scene generation, and
  final-state checks. Review fix `a2621f80185` rejects signed size arguments
  before `std::stoull` and makes final-state digest size hashing portable.
- Baseline evidence, Bullet collision, 120 generated objects, 9000 steps,
  default sleeping: RTF `0.882581`, final contacts `360`, final resting
  `81 / 120`, finite state true, final hash `0x212c1143bd0a2fb3`.
- Baseline evidence, same scene with deactivation disabled: RTF `0.740188`,
  final contacts `360`, final resting `0 / 120`, finite state true, final hash
  `0xcf3145a17b2cada2`.
- PR1 #3085 added primitive plane and contact-cap collision support, passed
  Codex review, and is merged into `release-6.20`.
- PR1 current default Bullet run, same 120-object/9000-step command as PR0:
  RTF `0.915975`, final contacts `360`, final resting `81 / 120`, finite state
  true, final hash `0x212c1143bd0a2fb3`. This preserves PR0 final state.
- PR1 `--max-contacts-per-pair 4`, same Bullet command: RTF `0.897063`, same
  final hash `0x212c1143bd0a2fb3`. This is correctness-preserving here but not
  a speedup for Bullet.
- PR1 FCL primitive short check, 120 objects, 3000 steps: uncapped RTF
  `0.0511826`, capped at 4 contacts/pair RTF `0.131733`, but final hashes and
  state diverged substantially. That cap must remain explicit for this path
  unless follow-up work proves a physically equivalent cap policy.
- Next collision diff should implement a contact-manifold reduction/selection
  algorithm before making any four-contact-per-pair behavior default-on. The
  goal is to choose representative, well-distributed contact points for each
  pair instead of truncating backend contact output in iteration order. Required
  evidence: same original #3056 command line, baseline vs previous vs current
  RTF, final-state hash/summaries, and focused tests showing preserved contact
  support for boxes, cylinders, spheres, and planes.
- Prior-art search found three useful sources: closed PR #2366 had a generic
  `constraint::ContactManifoldCache` with persistent pair history, deepest
  contact plus spatially spread contact selection, a box-stacking GUI
  comparison, C++/dartpy tests, and `bm_contact_manifold_cache`; merged PR
  #2125 and release backport #2902 added an ODE-specific DART 6 contact-history
  cache for sparse capsule contacts; and merged DART 7 native-collision PRs
  #2652/#2688 added `ContactManifold`, `PersistentManifoldCache`, and box-box
  contact reduction under `dart/collision/native`.
- Sequencing decision: keep #3085 as the plane/contact-cap plumbing and evidence
  PR. The default-on behavior belongs in a later manifold-reduction PR that
  mines #2366 and DART 7 native collision, then proves equivalence before
  changing core defaults.
- PR2 #3086 must make any behavior-preserving performance path default-on if it
  is safe for gz-physics compatibility. Any default speedup must be backed by a
  fidelity/correctness test, not just an RTF improvement.
- PR2 current default-on Bullet run after gz-physics fidelity fixes, same
  120-object/9000-step generated drop command: RTF `2.52099`, final contacts
  `0`, final resting `120 / 120`, finite state true, final hash
  `0xf8661e0f2baad9f9`. The zero-contact final state is expected only after
  all mobile skeletons are resting and the all-resting fast path is active.
- PR2 current explicit `--disable-deactivation` run on the same command: RTF
  `0.489052`, final contacts `360`, final resting `0 / 120`, finite state true,
  final hash `0xc68e1f3b0fa9dc83`.
- PR2 current final-scene comparison, default-on vs disabled, 121 dumped shapes:
  max position delta `0.00118920391212` m, mean position delta
  `5.35687462563e-5` m, max quaternion L2 delta `0.00118926675519`. The sleep
  contact penetration gate is
  tightened to `1e-5`, and
  `IslandDeactivation.DefaultEnabledSettlesCloseToAlwaysActivePath` now guards
  the default-on fidelity bar on a focused drop-and-settle case.
- The direct single-body LCP shortcut is disabled pending fidelity evidence
  because it changed the explicit `--disable-deactivation` baseline. PR2's
  default speedup must come from resting-world deactivation, not from a solver
  shortcut that changes always-active physics.
- Current PR2 review fixes keep the all-resting fast path conservative: a
  sleeping body with externally edited velocity wakes before the cached step is
  reused, and contacts inside a frozen mobile island are preserved during an
  awake impact so wakeup remains island-atomic.
- Current PR2 gz-physics fixes keep sleep transitions from advancing the final
  solved impulse into pose, zero body velocity caches when entering rest, clear
  stale sleep candidacy when an active contact group cannot rest, and prevent a
  quiet support contact from sleeping while another mobile body in the world is
  still above the wake-speed band.
- Current PR2 filter-cache fix keys the all-resting fast path on the solver
  collision-filter pointer and BodyNode-pair blacklist revision. That wakes
  sleepers when support contacts are removed by a solver filter edit while
  preserving the GUI sleep-color/visual-only no-op path.
- Current PR2 compatibility review fixes keep the legacy solver sleeping setter
  as a source-compatible alias and preserve ordinary self-collision/blacklist
  filtering before the solver-local resting-island contact preservation path.
- Current PR2 custom-filter fix disables all-resting snapshot reuse for filters
  whose decision state is not revision-tracked, so stateful custom filters
  cannot hide changed contact decisions behind the cached fast path.
- Parallel follow-up: #3071 has been reconciled with the current
  `setNumSimulationThreads` / `ConstraintThreadPool` API. The useful remaining
  diff exposes the control in dartpy, adds an in-tree Pixi benchmark task for
  `boxes_headless`, and records deterministic scaling evidence. It should not
  claim the resting original #3056 scene scales linearly; the active original
  scene and larger articulated workload are the evidence for the thread knob.

## Default-On Correctness Rule

Performance behavior should be default-on only when the physical behavior stays
the same within the intended DART 6 compatibility surface. A fast path that
changes contact generation, wake/sleep transitions, reset behavior, final
finite-state checks, or final state summaries must be treated as a correctness
bug until proven otherwise.

## PR Split

1. PR0: reproducible baseline tests, benchmark/example, GUI/headless verifier,
   final-state consumption, and `.benchmark_results/` ignore coverage.
2. PR1: collision detector and contact-generation improvements. Compare against
   PR0 and add focused collision correctness regressions.
3. PR2: resting-world/deactivation and solver hot-loop improvements. Compare
   against PR0 and PR1, and verify default-on behavior preserves fidelity.
4. PR3 candidate: contact-manifold reduction/selection. This should be a
   separate review from PR1 because PR1's per-pair cap is a measurement and
   control knob, while this follow-up must preserve contact-patch fidelity well
   enough to justify default-on behavior.
5. Parallel-safety follow-up: keep #3086's opt-in independent-island solve on
   the serial path when solver type, manual/custom constraints, or shared
   non-reactive dependencies make parallel independence uncertain. This is a
   correctness-preserving prerequisite for later parallel speedups, not a
   performance-claim PR.
6. Later PRs: larger backend swaps or DART 7 native collision detector ports,
   only after available collision-engine comparisons show the dependency and
   correctness tradeoffs clearly.

## Validation Log

- Built `test_CollisionGroups` after the FCL shared-object review fix.
- Ran `pixi run ./build/default/cpp/ReleaseNoProfile/tests/integration/test_CollisionGroups`
  after the FCL shared-object review fix: 22 tests passed.
- Built `test_Collision`, `test_SdfParser`, and `contact_benchmark` after the FCL
  refresh and example changes.
- Ran `pixi run ./build/default/cpp/ReleaseNoProfile/tests/integration/test_Collision`:
  28 tests passed.
- Ran `pixi run ./build/default/cpp/ReleaseNoProfile/tests/integration/test_SdfParser`:
  6 tests passed.
- Ran `pixi run lint`.
- For PR0 review fix #3089, built `contact_benchmark`, verified
  `--generate-objects -1` exits with status 1, ran a 6-object/20-step smoke run
  that advanced time and emitted finite final state, and ran `pixi run lint`.
- For PR2 default-on fidelity, built `contact_benchmark`,
  `test_IslandDeactivation`, `test_World`, and `test_ContactSurface`; ran
  `test_IslandDeactivation` after adding the always-active comparison test: 19
  tests passed.
- Ran `test_ContactSurface` after the PR2 contact/sleep changes: 12 tests
  passed.
- Ran `test_World` after making deactivation default-on: 6 tests passed. The
  existing `World.Cloning` test still emits a large non-finite-warning stream.
- After #3085 merged, merged `release-6.20` into PR2 #3086, removed the stale
  `sdf_perf_test` target path, rebuilt `contact_benchmark`,
  `test_IslandDeactivation`, `test_World`, and `test_ContactSurface`, ran those
  three focused tests through CTest, ran a 3-object `contact_benchmark` smoke,
  and ran `pixi run lint`.
- For the PR2 Codex support-edit review, collision geometry/collidability
  edits now invalidate the all-resting cache, support removal wakes remaining
  sleepers, and `test_IslandDeactivation` covers support removal, support
  collidability disable, and support shape transform/resize. Rebuilt
  `contact_benchmark`, `test_IslandDeactivation`, `test_World`, and
  `test_ContactSurface`; CTest passed the three focused tests; the 3-object
  `contact_benchmark` smoke advanced time with finite final state.
- For the PR2 Codex velocity/island review, `test_IslandDeactivation` now covers
  velocity edits while the all-resting cache is hot and same-step wake of a
  frozen mobile island. Rebuilt `test_IslandDeactivation`, then rebuilt
  `contact_benchmark`, `test_World`, and `test_ContactSurface`; CTest passed
  the three focused tests, and the 3-object `contact_benchmark` smoke advanced
  time with finite final state.
- Reran the 120-object/9000-step Bullet comparison after those review fixes:
  default-on RTF `3.45052`, disabled RTF `0.490135`, both finite. Dumped both
  final scenes and compared 121 shapes: max position delta `0.00122575` m, mean
  position delta `5.74756e-5` m, max quaternion L2 delta `0.00122581`.
- After #3085 merged and #3086 failed gz-physics CI, reproduced
  `COMMON_TEST_joint_features_dartsim` and `UNIT_SDFFeatures_TEST` locally,
  added deactivation regressions for final sleep-transition pose/velocity,
  unconverged contacts, and waiting for other active mobile bodies, rebuilt
  `contact_benchmark`, `test_IslandDeactivation`, `test_World`, and
  `test_ContactSurface`, and passed those three focused DART tests through
  CTest.
- Reinstalled DART into the Gazebo pixi environment and passed the focused
  gz-physics failures:
  `COMMON_TEST_joint_features_dartsim` and `UNIT_SDFFeatures_TEST`.
- Reran the 120-object/9000-step/drop-height-0.2 Bullet comparison after the
  gz-physics fidelity fix: default-on RTF `2.52099`, disabled RTF `0.489052`,
  both finite. Dumped both final scenes and compared 121 shapes: max position
  delta `0.00118920391212` m, mean position delta `5.35687462563e-5` m, max
  quaternion L2 delta `0.00118926675519`.
- Ran `pixi run -e gazebo test-gz-physics`: gz-physics build succeeded, 199/199
  tests plus 4/4 performance tests passed, and the DART plugin linked against
  the pixi-installed DART libraries.
- For the PR2 Codex collision-filter review, added a solver-filter support
  removal regression and narrowed the all-resting snapshot revision to the
  BodyNode-pair blacklist so visual-only color changes remain cached. Rebuilt
  `contact_benchmark`, `test_IslandDeactivation`, `test_World`, and
  `test_ContactSurface`; CTest passed the three focused tests, and the 3-object
  `contact_benchmark` smoke advanced time with finite final state.
- Rebuilt gz-physics' test-enabled DART plugin tree against the current DART
  install and passed the focused downstream checks:
  `COMMON_TEST_joint_features_dartsim` and `UNIT_SDFFeatures_TEST`.
- For the PR2 Codex compatibility/filter-order reviews, rebuilt
  `test_IslandDeactivation` and `test_ConstraintSolver`, then passed both
  through focused CTest.
- For the PR2 Codex custom-filter review, rebuilt `test_IslandDeactivation` and
  passed the focused CTest after adding a stateful custom-filter regression.
- For the parallel-safety follow-up, built `test_ConstraintSolver` and
  `test_World`, passed focused new tests covering exact built-in LCP solvers,
  randomized PGS, manual constraints, custom contact constraints, and shared
  non-reactive dependencies, then passed both integration test binaries through
  focused CTest.
- For the deactivation parallel-solve / exact-issue pass, rebuilt
  `contact_benchmark`, `test_IslandDeactivation`, and `test_ConstraintSolver`;
  ran `test_IslandDeactivation` (50 tests passed) and `test_ConstraintSolver`
  (18 tests passed), then ran `pixi run lint` and reran the same focused build
  and tests successfully. Exact issue SDF evidence used Bullet collision
  detection with SDF plane shapes, 3000 measured steps, final-state consumption,
  and final scene dumps for default-on vs `--disable-deactivation`.
- For the same branch, a full `pixi run -e gazebo test-gz` attempt passed the
  gz-physics build, 199/199 gz-physics tests, and 4/4 gz-physics performance
  tests, then the gz-sim clone hit a network timeout. A later
  `pixi run -e gazebo test-gz-sim` reached `INTEGRATION_entity_system` but
  failed with `Failed to find plugin [gz-physics-dartsim-plugin]`; inspection
  showed the source-built gz-physics install under `lib64` and a broken
  installed unversioned engine-plugin symlink. After updating
  `scripts/run_gz_sim_task.sh` to discover `lib64` and include the valid
  build-tree plugin alias, a fresh `pixi run -e gazebo test-gz-sim` rerun passed
  `INTEGRATION_entity_system`.
- For the native finite-plane parallel collision follow-up after #3129, rebuilt
  all tests with `pixi run build-tests`, passed the focused CTest selection
  `test_Collision|test_ContactConstraint|test_ConstraintSolver`, and reran the
  exact issue scene with final-state hashing. Active DART-native runs preserved
  hash `0x6b50e84cd691f6e2`, contacts `5005`, and pairs `3003` across 1, 2, 4,
  8, and 16 simulation threads. The default-sleeping 3000-step run advanced
  `3000 / 3000` frames, reached RTF `1.96845`, and ended finite with hash
  `0x131b6af79a44ff90`.
- For the `perf/dart6-parallel-surface-params` slice, rebuilt
  `contact_benchmark`, `test_ConstraintSolver`, and `test_Collision`; passed
  focused CTest for `test_ConstraintSolver|test_Collision`; ran
  `pixi run lint`; passed
  `DART_PARALLEL_JOBS=24 CTEST_PARALLEL_LEVEL=24 CMAKE_BUILD_PARALLEL_LEVEL=24 pixi run test-all`
  with C++ tests 115/115 and Python tests 60/60; and reran the active issue SDF
  across DART-native, FCL primitive, Bullet, and ODE with final hashes preserved
  for each backend.
