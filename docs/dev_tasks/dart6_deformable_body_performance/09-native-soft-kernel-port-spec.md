# SPEC — Native-Owned Soft Kernels (packet: remove soft/ellipsoid fallback bridge)

Branch `wp-db-native-soft-fallback` @ 79acb2b06bd. Goal: soft×{plane,sphere,box,ellipsoid,soft} resolve inside `dart/collision/native` with NO `DARTCollisionObject` mirror, output BIT-EQUAL to today's bridge (`emitDartFallbackContacts`), per-step cost strictly < dart factory. Verified coverage: DARTCollide soft dispatch (`DARTCollide.cpp:4360,4429,4691-4783`) covers soft×{plane,sphere,ellipsoid,box,soft} ONLY — no soft×capsule/cylinder/mesh kernel exists, so those currently emit nothing; native lanes covering the five pairs match 100% of bridge soft coverage.

## 1. Native soft shape kind (first-class)

- Add `ShapeType::SoftMesh` to `native/shapes/Shape.hpp:61-72` (append; do not reorder — enum ordinals leak into no serialized state, but keep stable).
- New `native/shapes/SoftMeshShape.{hpp,cpp}` : `public native::Shape`.
  - Owns the RETAINED cache (see §2) — this is the shape's geometry, refreshed in place.
  - `getType()==SoftMesh`; `computeLocalAabb()` returns cached local bounds (recomputed during refresh, NOT re-derived here).
  - `refreshFromSoftBody(const dynamics::SoftBodyNode&)`: per-step local-vertex refresh. Vertex source MUST match `SoftMeshShape::updateBoundingBox` / `DARTCollisionObject::refreshSoftMeshCache` exactly: `mAspectState.mPointStates[i].mPositions + mAspectProperties.mPointProps[i].mX0` (SoA fast path, `DARTCollisionObject.cpp:319-333`), else `pointMass->getPositions()+getRestingPosition()` (`:335-346`). Any divergence breaks checksums.
- Conversion: `NativeShapeConversion::create` (`detail/NativeShapeConversion.cpp:203`) add a `SoftMeshShape::getStaticType()` branch that builds the native soft shape with TOPOLOGY ONLY (faces from `mAspectProperties.mFaces`, `firstFaceByPointMass`), then one `refreshFromSoftBody`. Reachable only because §6 flips the fallback predicates.
- `NativeCollisionObject` (`NativeCollisionObject.cpp`):
  - `shapeUsesDartFallback` (`:79`) and `shapeUsesSoftMeshFallback` (`:97`) STOP returning true for `SoftMeshShape`. Add classification `mIsNativeSoftShape` (new member, hpp:91-93 flag block) set when shape is SoftMesh.
  - `updateEngineData` (`:194`): SoftMesh now takes the NATIVE branch (`:225-238`), NOT the fallback branch (`:208-223`). BUT the id/version gate is blind to soft motion — `dynamics::Shape::getVersion()` on SoftMeshShape does NOT bump on `PointMass::setPositions` (only SoftBodyNode version does; `SoftMeshShape::update` sets bounding-box-dirty only). So add a soft carve-out mirroring `DARTCollisionObject.cpp:198-207`: when `mIsNativeSoftShape`, skip the `shapeChanged` early-out and call `mNativeShape->refreshFromSoftBody(...)` UNCONDITIONALLY each step (gate topology rebuild on `SoftBodyNode::getVersion()` + point-mass count + per-vertex `cwiseEqual` compare, exactly the triggers at `DARTCollisionObject.cpp:328,352-355`).
  - Transform: `mNativeTransform = getTransform()` (native branch already does this). AABB: `Aabb::transformed(mNativeShape->computeLocalAabb(), mNativeTransform)` — one transform/step (`:230-232`), same count as dart factory.

## 2. Retained native soft cache

- Data (lift the value structs from `DARTCollisionObject.hpp:71-97` verbatim into `native/shapes/SoftMeshShape.hpp` or a shared `native/shapes/SoftMeshCache.hpp`):
  - `localVertices` (`std::vector<Vector3d>`), `firstFaceByPointMass` (`std::vector<int>`).
  - `CachedSoftFace` {indices, a, edge0, edge1, unit normal, centroid, boundsMin/Max, planeOffset=normal·a, barycentric d00/d01/d11/denom, valid} — `DARTCollisionObject.hpp:71-87`.
  - `CachedSoftFaceBvhNode` {boundsMin/Max, left, right, first, count} — median-split, leaf≤4 — `DARTCollisionObject.hpp:89-97`.
  - Local bounds quartet (min/max/center/half-extents).
- Location: OWNED BY the native `SoftMeshShape` (per-shape geometry), reachable from `NativeCollisionObject` via `getNativeShape()`. This keeps the transform/AABB on the object and the deforming geometry on the shape.
- Refresh/versioning (port the 5 methods `refreshSoftMeshCache:279`, `refreshSoftFaceCacheIfNeeded:408`, `refreshSoftFaceBvhCache:479`, `refreshSoftFaceBvhBounds:569`, plus `updateCachedBounds:61`):
  - Per-step: gather vertices, per-vertex change compare → `softGeometryChanged`; recompute local bounds. Topology change (count/faces/version) → rebuild `firstFaceByPointMass` + face `indices`, set dirty flags.
  - LAZY face/BVH recompute: face plane cache + BVH built on FIRST narrowphase access (via the accessors), gated on `mCachedSoftFacesDirty` — NOT during object update. Preserves dart's lazy cadence.
  - BVH: rebuild (`nth_element` median split, leaf 4) only on topology/validity change; else REFIT node bounds bottom-up (`refreshSoftFaceBvhBounds`) — this refit-vs-rebuild split is exactly what `native::MeshShape` lacks and must be ported.
- Zero steady-state allocation: all vectors reserved once at topology build; per-step refresh writes in place (no resize when count stable). No `mDartFallbackObject` unique_ptr ever allocated for soft (see §4).

## 3. Kernel ports + dispatch wiring

Port to `native/narrow_phase/SoftMeshKernels.{hpp,cpp}` (new). Lift from `DARTCollide.cpp`:
- Primitive lanes (both orderings, verbatim math): `collidePlaneSoftMesh:1567`/`collideSoftMeshPlane:1614`, `collideSphereSoftMesh:1661`/`collideSoftMeshSphere:1713`, `collideEllipsoidSoftMesh:1777`/`collideSoftMeshEllipsoid:1837`, `collideBoxSoftMesh:2305`/`collideSoftMeshBox:2354`. Each iterates soft point masses, transforms each cached local vertex into the primitive frame, cheap inside test, emits ONE contact per penetrating vertex tagged with that point mass's `firstFace` (→ triID on the soft side).
- Soft-soft lane: `collideSoftMeshSoftMesh:2244` = two symmetric `addSoftPointFaceContacts:2169` passes (each body's points vs other's cached faces + BVH). Helpers to lift: `findSoftPointFaceContact:2032`, `addCachedSoftFaceCandidate:1959`, `projectPointInsideCachedTriangle:1910`, `distanceSquaredToAabb:1940`, `findContainingBoxFaceFast:2271`, `computeEllipsoidPointPenetrationDepth:1765`.
- Shared access layer: `SoftPointCacheView:1481`, `RelativeTransformView:1488`+`makeRelativeTransformView:1499`, `transformPoint:1510`, `getSoftPointCount/LocalPosition/FirstFace:1541-1559`, `kSoftContactShell=1e-6`. THE ONLY ENTANGLEMENT is `makeSoftPointCacheView`'s `dynamic_cast<const DARTCollisionObject*>` (`:1524`) and the soft-soft entry's dynamic_cast (`:2259-2262`). REPLACE that seam: kernels bind to the native soft cache via `getNativeShape()`-derived accessors (or a small `SoftCacheView` built from `native::SoftMeshShape`). Everything downstream is mechanical/wrapper-agnostic.
- Dispatch: `NarrowPhase::collideShapes` (`native/narrow_phase/NarrowPhase.cpp:122`) — add if-branches for `SoftMesh × {Plane, Sphere, Box, Ellipsoid, SoftMesh}` and flipped orderings via the existing `collideWithFlippedNormals` helper; add `isSupported` entries. Ellipsoid needs a native shape to dispatch (see §4/§9 staging).
- BIT-COMPATIBILITY (checksum-critical): the kernels already set `collisionObject1/2 = o1/o2`, `point`, `normal` (sign per ordering), `penetrationDepth = max(0,...)`, `triID1/triID2` (face index on soft side). Feed the NATIVE objects directly as `o1/o2` in BROADPHASE ENUMERATION ORDER (`id1<id2`, `NativeCollisionDetector.cpp:1054-1056`) so object order = today's bridge order. Emit via a copy-through equal to `emitDartFallbackContacts:761` (preserve kernel `point/normal/depth/triID`, keep the `depth<0` and `isZeroNormal` filters `:781-787`, per-pair cap `getEffectiveMaxNumContactsPerPair`, global `maxNumContacts`). Do NOT route through `emitContacts:717` (it overwrites triID from `featureIndex1/2` — would diverge unless the native kernels populate a `native::CollisionResult` with `featureIndex=faceIndex`; the copy-through path is the lower-risk equal-output route). Contact budget: honor `kSolverFacingManifoldContactTarget=3` cap ONLY where the bridge did — bridge soft goes through `emitDartFallbackContacts` which does NOT apply the native 3-cap (that cap is `makeNativeOption:109`, native branch only). So native soft emit must use the SAME per-pair cap the bridge used (`getEffectiveMaxNumContactsPerPair`), NOT clamp to 3. Verify against checksum battery.

## 4. Bridge disposition

- After §6, `shouldUseDartFallback` (`NativeCollisionDetector.cpp:121`) returns false for all covered soft pairs → they take the native branch in `processNativePair:867-903` (both objects now have `getNativeShape()!=null`).
- RETAIN the bridge ONLY for pairs native soft kernels do not cover. Functionally that is NONE today (DARTCollide has no soft×capsule/cylinder/mesh kernel). Keep `processDartFallbackPair:799` + `getDartFallbackObject:165` in place but reachable only if a future soft×X kernel is added to DARTCollide but not yet ported — gate `shouldUseDartFallback` on "soft AND partner-type ∈ {capsule,cylinder,mesh,convex}" so those emit-nothing pairs stay identical (they emit nothing either way; simplest is to leave them native-skipped). Non-sphere ellipsoid: soft×ellipsoid needs the ellipsoid params natively — see §9 (interim: retain bridge for soft×non-sphere-ellipsoid until native::EllipsoidShape lands; `shouldUseDartFallback` keeps firing for that one pair).
- Manifold cache: native soft MUST BYPASS the persistent manifold cache. Reason (verified): `PersistentManifold::refresh`/`findMatch` assume rigid constant `localPointA` under one `getTransform()`; per-vertex-deforming soft violates this → warm-start mis-association. Today the group gate `hasSoftMeshFallbackShape` (`:1042-1043,206`) disables the ENTIRE group cache when any soft is present. That is too coarse once soft is native-owned (soft would satisfy `ownsNativeManifoldShape:182` and re-enable the cache for itself AND stop disabling it for rigid neighbors). SPLIT "is soft" from "uses bridge": add `isNativeSoftShape()`; make `ownsNativeManifoldShape:182`, `shouldUseNativeManifoldContact:163`, and the group gate exclude native soft while LEAVING rigid-rigid pairs in the same group cache-eligible. Net: soft pairs bypass warm-start (userData stays null, matching bridge behavior — bridge contacts always had null userData); rigid pairs KEEP the cache (improvement over today's blanket disable). `nativeManifoldContactFound` (`:892`) must stay false for soft pairs.

## 5. Triangle/contact-neighborhood coverage extension (native-lane only, checksum-CHANGING)

- Gap: the primitive lanes (§3) are VERTEX-ONLY — a box/sphere/ellipsoid poking through a soft FACE INTERIOR with no soft vertex inside the primitive emits NO contact today (verified: `collidePlaneSoftMesh`/`collideBoxSoftMesh` etc. only loop point masses). The soft-soft lane already does point-vs-face (`projectPointInsideCachedTriangle`).
- Extension: in EACH native soft-primitive kernel add a SECOND pass — for the primitive's representative feature (sphere/ellipsoid center, box vertices/edge-midpoints), find closest point on each candidate soft face (reuse `projectPointInsideCachedTriangle` + face BVH `distanceSquaredToAabb` traversal from the soft-soft lane) and emit a contact when penetrating. Slots into the same kernel file, gated behind `nativeOption` flag `enableSoftFaceInteriorContacts` (default OFF for parity).
- Checksum impact: adds contacts the DART bridge never produced → NOT bit-compatible. MUST be native-lane-only, behind the flag, validated against a SEPARATE (non-bridge) reference. Ship in a late stage (§9) after the bit-equal core is green. Keep it out of the checksum-equivalence battery; add its own physical-correctness tests.

## 6. Predicate flips (the un-own switch)

`NativeCollisionObject.cpp`: `shapeUsesDartFallback:79` and `shapeUsesSoftMeshFallback:97` stop matching SoftMesh; add `mIsNativeSoftShape`. `NativeShapeConversion.cpp:203` gains the SoftMesh branch so `getNativeShape()!=null`. This single flip removes the bridge for covered pairs AND (with §4 split) fixes the manifold gate.

## 7. Cost audit (must be strictly < dart factory for a small soft scene)

Verified baseline: today the soft point-mass gather is NOT doubled — the fallback branch runs `refreshSoftMeshCache` ONCE via the mirror (`NativeCollisionObject.cpp:213`). The +3.8–6.4% gap is FIXED per-step BRIDGE overhead, not kernel math.
- DOUBLE WORK THAT DISAPPEARS: the whole `DARTCollisionObject` mirror lifecycle layered ON TOP of the native object's own bookkeeping — mirror construction (`getDartFallbackObject` unique_ptr alloc), the mirror's `updateEngineData`→`refreshShapeCache` dispatch shell, its cached-bounds/transform bookkeeping, `getWorldTransformForCollision` indirection (`:216`), `mNativeShape.reset()` per step (`:211`), and per soft PAIR: 2× `getDartFallbackObject()` derefs + `shouldUseDartFallback` re-check + `processDartFallbackPair` frame (`:867-869,811-812`). Native-owned collapses the two-object bookkeeping to one.
- WHAT REMAINS (equal to dart): exactly ONE `refreshSoftMeshCache` gather + one lazy face/BVH build-or-refit + one world-AABB transform; the soft kernel math (byte-identical). The point-mass gather is relocated into the native soft shape, NOT duplicated.
- NEW WORK ADDED: native soft-shape topology build (once, amortized); per-step vertex refresh keyed on SoftBodyNode version + vertex compare = the SAME work as `refreshSoftMeshCache`, net zero if not doubled.
- STANDING LOSS RISKS (must offset or fix):
  1. Native broadphase is `BruteForceBroadPhase` O(n²), no sweep-and-prune, no SIMD, scalar `overlapsFast` (`BruteForce.cpp:67`), plus `updateRange` copies AABB+6 scalars/object/step into `entries_` and keeps an `unordered_map indices_` live (`NativeCollisionGroup.cpp:151-154`). Dart factory uses sorted SAP + x-axis early-break + 4-wide SIMD (`computeFiniteOverlapMask<4>`, `DARTCollisionDetector.cpp:308,2609-2652`) and builds one on-the-fly entry. For small n the absolute delta is tiny; but it is strictly heavier fixed overhead. If parity tests show native still trailing, port SAP+SIMD to the native broadphase (separate stage). This is the PRIMARY place native can still lose.
  2. Per-step `hasSoftMeshFallbackShape` scan (`:1043`) becomes pure waste once soft is native — replace with a cached group flag set on add/remove (or reuse the §4 `isNativeSoftShape` count).
- Acceptance (`06-pr-evidence.md:23-36`): direct native ties dart within 2% on all 4 trailing rows (adaptive_deformable/1, soft_bodies/1, soft_cubes/1, soft_cubes/16). soft_cubes/16 depends on §8 threading.

## 8. Threading

- Verified: no parallelism in the native path — `setNumCollisionThreads:1007` only stores the count; `collide` walks pairs serially via `visitPairsInline:1053`. Dart factory parallelizes soft-soft finite pairs: `processFiniteFiniteCandidatePairs` (gate `kMinParallelSoftSoftPairs=4`, `DARTCollisionDetector.cpp:2508-2517`) with `prepareParallelSoftSoftCaches` pre-warm.
- Map onto native pipeline: collect soft-soft (and soft-primitive) candidate pairs from `visitPairsInline` into a vector; when `count >= kMinParallelSoftSoftPairs` and `getNumCollisionThreads()>1`, dispatch pair narrowphase across workers. Per-worker state required: one `ScratchCollisionResult`/soft pair-result buffer per worker (today one shared `fallbackPairResult:1052`), plus per-worker output buffers merged deterministically in enumeration order to preserve checksum. Pre-warm the lazy face/BVH cache BEFORE the parallel region (equivalent to `prepareParallelSoftSoftCaches`) since first-access recompute mutates shape cache and is not thread-safe. Manifold attach stays a serial post-pass (soft bypasses it anyway, §4). Broadphase `entries_` is read-only during the visit → pairing is safe; the constraints are the shared result + scratch.

## 9. Staged landing order (each independently gated, smallest safe slice first)

1. **Native soft shape + cache, no dispatch.** Add `ShapeType::SoftMesh`, `native::SoftMeshShape` + retained cache (§1,§2), conversion branch, `updateEngineData` soft refresh path, `mIsNativeSoftShape`. KEEP `shouldUseDartFallback` firing for soft (bridge still owns narrowphase). Gate: allocation tests confirm zero steady-state alloc + native soft AABB matches fallback AABB bit-for-bit. Behavior unchanged.
2. **soft×plane native lane.** Port plane kernels + dispatch + copy-through emit; flip predicates so soft×plane takes native branch. Retain bridge for all other soft pairs. Gate: checksum battery on soft-vs-ground rows equals pre-change bridge output.
3. **soft×sphere + soft×box native lanes.** Same pattern. Gate: per-row checksum equality.
4. **soft×soft native lane.** Port soft-soft + BVH helpers. Gate: soft_cubes/soft_bodies checksum equality.
5. **Manifold-gate split (§4).** Introduce `isNativeSoftShape` in `ownsNativeManifoldShape`/`shouldUseNativeManifoldContact`/group gate; verify rigid neighbors in soft scenes now keep warm-start while soft bypasses. Gate: rigid-in-soft-scene manifold tests + no soft userData.
6. **native::EllipsoidShape (radii-only) + soft×ellipsoid lane.** Add `ShapeType::Ellipsoid` carrying radii so soft-ellipsoid dispatches natively; add EXPLICIT no-contact guards for ellipsoid×{rigid primitives} in `collideShapes` to preserve current skip behavior (DART emits nothing there — do NOT let the convex-fallback catch-all fabricate contacts). Retire the soft×non-sphere-ellipsoid bridge. Gate: ellipsoid×rigid still emits nothing; soft×ellipsoid checksum equal.
7. **Threading (§8).** Per-worker scheduler for soft pair narrowphase. Gate: soft_cubes/16 timing + determinism (checksum identical single- vs multi-thread).
8. **Broadphase SAP+SIMD (conditional).** Only if §2–§7 leave native trailing on the cost gate. Gate: cost audit ties/beats dart on all 4 rows.
9. **Face-interior coverage extension (§5, checksum-CHANGING).** Behind `enableSoftFaceInteriorContacts` flag, native-lane-only, own correctness tests, excluded from bridge-parity battery.

## Test plan

- Port `test_DARTCollisionDetector` soft cases (soft×{plane,sphere,box,ellipsoid,soft}) to native equivalents in `test_NativeCollisionDetector` — assert contact count, point, normal sign, penetration, triID1/triID2 equal to the DART factory / pre-change bridge for each covered pair and BOTH object orderings (normal-sign convention is ordering-dependent, §3).
- Checksum battery: extend the existing native-vs-bridge checksum harness (`06-pr-evidence.md` rows) to run per stage; each of stages 2–6 must hold bit-equality vs the pre-stage bridge output.
- Allocation gates: extend the retained-cache allocation test to assert ZERO steady-state heap allocation across N steps for a deforming soft scene (no `mDartFallbackObject`, no per-step vector resize); mirror the existing WP-DB allocation-gate pattern.
- Manifold: new test — soft + two rigid boxes in one group; assert rigid-rigid contacts carry warm-start `userData` (cache active) while soft contacts have null userData (bypass), post stage 5.
- Threading: determinism test — same scene single- vs multi-thread yields identical contact set/order (checksum), post stage 7.
- New parity tests: soft-vertex-on-primitive-face boundary cases (grazing, exactly-on-surface `contactTolerance=1e-9`), degenerate soft faces (`valid=false`), empty-BVH linear-scan fallback path.
- Stage 9 only: physical-correctness tests for face-interior contacts (primitive poking a coarse soft face between vertices), NOT run against the bridge reference.

## Key target files

- New: `native/shapes/SoftMeshShape.{hpp,cpp}`, `native/narrow_phase/SoftMeshKernels.{hpp,cpp}` (+ shared `native/shapes/SoftMeshCache.hpp`).
- Edit: `native/shapes/Shape.hpp` (ShapeType), `native/detail/NativeShapeConversion.cpp:203` (conversion), `native/NativeCollisionObject.{hpp,cpp}` (predicates, soft refresh branch, `mIsNativeSoftShape`), `native/narrow_phase/NarrowPhase.cpp:122` (dispatch + isSupported), `native/NativeCollisionDetector.cpp` (`shouldUseDartFallback:121`, manifold gates `:163-215,1042-1048`, soft emit copy-through), `native/NativeCollisionGroup.cpp` (group soft flag), optionally `native/broad_phase/BruteForce.{hpp,cpp}` (stage 8 SAP+SIMD).
- Source-of-truth to mirror (do not edit): `dart/collision/dart/DARTCollide.cpp:1464-2401` (kernels), `dart/collision/dart/DARTCollisionObject.{hpp,cpp}` (cache structs + refresh pipeline), `dart/dynamics/SoftMeshShape.cpp:102-231` (vertex formula).
## Adversarial critique findings (2026-07-11) — binding revisions

Two independent verification passes against the code confirmed the
bit-equality plan while overturning parts of the plan above. These revisions
are binding on the implementation packet:

1. **Stage 0 (new, mandatory): measured attribution.** The 3.8-6.4% winner
   gap is attributed only structurally, in both directions: the bridge shell
   this packet removes is per-step tiny (the mirror gather and kernels are
   shared), and the counter-claim that the O(n^2) BruteForce broadphase
   dominates is equally unmeasured — the tracked scenes have 2-6 objects.
   Before any porting, profile direct `native` vs `dart` on `soft_cubes`
   THREADS=1 and attribute the ~0.25 ms/step delta with profiler scopes
   (collision side has them; include the copy-through emit, mirror
   transform/AABB bookkeeping, per-pair fallback checks, manifold-gate
   scans, and broadphase). The measured breakdown decides whether the
   winner gate is won by the kernel port alone, needs the stage-8
   broadphase work promoted, or needs a different fix entirely. The kernel
   port remains the correct end state regardless (seam removal + coverage).
2. **Per-pair gating, not per-shape flip.** Keep the soft fallback shape
   flag; gate in `shouldUseDartFallback` per pair: a native-soft x partner
   pair takes the native branch only when that pair's kernel is ported
   (and the partner has a native shape); otherwise it stays on the bridge.
   The mixed-state object must NOT maintain both caches: the native
   `SoftMeshShape` cache becomes the single source and the bridge pairs
   read through it (or the stage is landed whole for that pair type).
   Ellipsoid pairs stay bridged until the native `EllipsoidShape` stage.
3. **Manifold guard lands with the first predicate change**, not at stage
   5: without it, clearing the soft flag re-enables the group manifold
   cache and warm-starts soft contacts mid-rollout. Note the guard split
   also ADDS `refreshManifoldCache` work in soft scenes with >=2 rigid
   native-owning objects — benchmark those rows before/after.
4. **Unclamped soft generation option**: `makeNativeOption` clamps
   generation to 3 contacts (`kSolverFacingManifoldContactTarget`); soft
   dispatch must receive the full per-pair cap at GENERATION time, not
   only at emit.
5. **Excise, do not lift, the accessor fallback**: `getSoftLocalPosition`'s
   non-cache branch reads `getLocalPosition()` — a different vertex
   formula than the mandated `mPositions + mX0`. The native cache view
   must fail loudly on mismatch instead of silently switching formulas.
6. **Stage-1 AABB gate carve-out** for non-finite soft bounds (bridge
   returns a default Aabb; the native branch would transform the raw one).
7. Removed-work accounting corrections: mirror construction is one-time,
   `mNativeShape.reset()` on the fallback branch is a no-op; do not count
   them as per-step savings.
