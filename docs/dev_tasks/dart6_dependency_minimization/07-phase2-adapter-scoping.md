# DART 6.20 — Phase-2 Execution Plan: Native Collision Detector Adapter

> **Status: FINAL — adversarial review incorporated (gz-compat, cpp17-entt, scope lenses; all three returned SOUND-WITH-FIXES). Ready to execute.**
> Every concrete defect the three reviews raised is resolved in-plan (see §7). Where a review found an infeasibility (`Shape::computeAabb(tf)`, `World::setCollisionDetector("native")`), the plan is corrected to the real `release-6.20` API rather than hand-waved.

**Scope:** Add an internal, non-default `dart::collision::NativeCollisionDetector` on `release-6.20` that bridges DART 6's public `CollisionDetector`/`CollisionGroup`/`CollisionObject` contract to the already-ported `dart::collision::native` geometry engine, driving **BruteForce broadphase → narrowphase dispatcher → DART 6 `CollisionResult`/`Contact`**. FCL stays the default detector. No new dependency (no EnTT). C++17 only. Every PR is `pixi run -e gazebo test-gz`-gated (with the honest caveat in §3 that this gate is a non-regression guard on the engine-only slices) and leaves guard-row hashes unchanged.

---

## 0. The one architectural decision that shapes everything

**Do NOT port `native::CollisionWorld` (the EnTT/ECS world layer). Bypass it.** The whole "de-ECS a generational slotmap + fold five registry-only fields into `BatchStorage` + audit the `getRegistry()` public leak" problem *evaporates* in phase 2, because DART 6 already owns the exact facility EnTT was providing:

| EnTT facility (the "genuinely hard" ECS parts) | Phase-2 replacement (already exists on `release-6.20`) |
|---|---|
| `entt::registry` object store + `create/destroy/clear` | DART 6 `CollisionObjectManager` (Sharable) on the base `CollisionDetector` — `shared_ptr<CollisionObject>` keyed by `ShapeFrame*` |
| generational slot handle + `registry.valid()` stale-handle safety | DART 6 `ObjectInfo{ CollisionObjectPtr mObject; … }` holds a **`shared_ptr`** — the object provably outlives every use; no generational arena needed |
| `getRegistry()` public `entt::registry&` leak (ABI break) | Never enters `release-6.20` — `CollisionWorld` is not ported |
| `native::CollisionFilter` (handle embeds `entt::entity`) | Not ported — filtering uses DART 6's `CollisionFilter::ignoresCollision(o1,o2)` at the adapter level |
| `m_idToEntity`, `BatchStorage`, `comps/`, `batch.hpp` | Not ported — replaced by a ~15-line per-group id map (§1.1) |

What phase 2 *does* consume from the native engine is exactly the EnTT-free lower layers: `shapes/shape.hpp`, `aabb`, `types.hpp` (native `CollisionResult`/`CollisionOption`/`ContactPoint`), `contact_point`/`contact_manifold`, `gjk`/`mpr`, `narrow_phase/*`, and `broad_phase/{broad_phase.hpp, brute_force}`. Broadphase/narrowphase have **zero** EnTT coupling; the only EnTT touch in the ported set is `narrow_phase.hpp`'s `#include collision_object.hpp` + its `const CollisionObject&` convenience overloads, which we drop (§2, PR P2). An exhaustive grep of the port set (broad_phase + brute_force + all pair colliders + distance) confirms **zero** real EnTT references (the lone `mesh_mesh.cpp` hit is a false positive on the substring `SegmENTTriangle`).

**Verified tree state (this session, against `origin/release-6.20`):**
- Native tree already has: `types.hpp` (defines `class DART_COLLISION_NATIVE_API CollisionResult`, `struct CollisionOption`), `contact_point.hpp` (`ContactPoint{position, normal, depth, featureIndex1/2}`), `shapes/shape.hpp` (Sphere/Box/Capsule/Cylinder/Plane/Convex/Mesh/Sdf/Compound with ctors; base exposes **`virtual Aabb computeLocalAabb() const`** only — no transform-taking overload), `aabb` (with `static Aabb transformed(const Aabb& local, const Eigen::Isometry3d&)`), `gjk`/`gjk_inl`/`mpr`, `box_box*`, `sphere_sphere`, `detail/span.hpp`. **No `broad_phase/`, no `narrow_phase/narrow_phase.{hpp,cpp}` dispatcher, no adapter.**
- Native sources compile **directly into the core `dart` target** via `dart_add_core_headers`/`dart_add_core_sources` (explicit list, not glob). Adapter classes in `dart::collision` call `native::` with **no extra link step**; every new file MUST be added to the CMake list or it silently won't build.
- `broad_phase.hpp` / `distance.cpp` / most pair headers use `std::span`; `brute_force.*` is pure C++/Eigen. The proven span substitution is `#include <dart/collision/native/detail/span.hpp>` + `span<T>` (already live in `box_box.cpp` / `sphere_sphere` on the branch).
- Native `CollisionOption` fields: `bool enableContact=true; std::size_t maxNumContacts=1000; const CollisionFilter* collisionFilter=nullptr;` — **no `maxNumContactsPerPair`, no `allowNegativePenetrationDepthContacts`** (adapter enforces those DART 6 semantics itself). Native `CollisionResult` is **move-only** with `addContact(const ContactPoint&)`, `clear()`, `numContacts()`, `getContact(i)`.
- DART 6 `CollisionResult::addContact` **asserts non-null** `collisionObject1/2` (via `addObjectToCaches`, `CollisionResult.cpp:178-183`). The adapter must populate both.

---

## 1. Bridge design

Three new classes in namespace `dart::collision` + one shape converter, mirroring the `dart/collision/dart/DART*` adapter template, substituting the `native::` engine for DART's internal SAT/broadphase.

### 1.1 Files and the two-namespace directory (scope-review fix #4)

Adapter files live in `dart/collision/native/` as **PascalCase** beside the existing snake_case engine:

```
dart/collision/native/NativeCollisionDetector.{hpp,cpp}       // namespace dart::collision
dart/collision/native/NativeCollisionGroup.{hpp,cpp}          // namespace dart::collision
dart/collision/native/NativeCollisionObject.{hpp,cpp}         // namespace dart::collision
dart/collision/native/detail/NativeShapeConversion.{hpp,cpp}  // dynamics::Shape -> native::Shape
```

**This directory intentionally hosts two namespaces** and is *not* structurally identical to `dart/collision/dart/`. In the DART adapter dirs (`dart/`, `fcl/`, `bullet/`, `ode/`) the directory holds a single namespace (`dart::collision`). Here, `dart/collision/native/` already carries the engine sub-namespace `dart::collision::native` (snake_case files); the adapter adds `dart::collision` (PascalCase files) alongside it. The case split *is* the disambiguator: **snake_case ⇒ `dart::collision::native` engine; PascalCase ⇒ `dart::collision` adapter.** We choose co-location (over a sibling `native_adapter/` directory) to keep engine + adapter under one CMakeLists and one `dart_add_core_*` list; the intentional two-namespace layout is documented here and must be called out in the P3a PR description so reviewers do not expect the `dart/collision/dart/` structure.

### 1.2 The plain-C++17 replacement for the entt registry/store

No monolithic "world"; state is split across the DART 6 objects that already exist.

**Per-object native state lives on `NativeCollisionObject`** (subclass of `dart::collision::CollisionObject`):
```cpp
class NativeCollisionObject : public CollisionObject {
  friend class NativeCollisionGroup;
  friend class NativeCollisionDetector;
public:
  const native::Shape* getNativeShape() const { return mNativeShape.get(); }
  const Eigen::Isometry3d& getNativeTransform() const { return mNativeTransform; }
  const native::Aabb& getNativeAabb() const { return mNativeAabb; }
protected:
  NativeCollisionObject(CollisionDetector* d, const dynamics::ShapeFrame* f)
      : CollisionObject(d, f) { rebuildNativeShape(); }
  void updateEngineData() override;                        // §1.4 tier-3
private:
  void rebuildNativeShape();                               // dynamics::Shape -> native::Shape via converter
  std::unique_ptr<native::Shape> mNativeShape;             // null if shape unsupported (warn+skip)
  Eigen::Isometry3d mNativeTransform{Eigen::Isometry3d::Identity()};
  native::Aabb mNativeAabb;
  const dynamics::Shape* mLastKnownShape = nullptr;        // identity guard (shape swap)
  std::size_t mLastKnownShapeVersion = 0;                  // version guard (in-place edit)
};
```

**Per-group id↔object bookkeeping + the broadphase live on `NativeCollisionGroup`** (subclass of `dart::collision::CollisionGroup`). This block is the *entire* replacement for `entt::registry` + `m_idToEntity` + `BatchStorage`:
```cpp
class NativeCollisionGroup : public CollisionGroup {
  friend class NativeCollisionDetector;
public:
  explicit NativeCollisionGroup(const CollisionDetectorPtr& d)
      : CollisionGroup(d),
        mBroadPhase(std::make_unique<native::BruteForceBroadPhase>()) {}
protected:
  // The six engine hooks (base CollisionGroup pure virtuals):
  void initializeEngineData() override {}
  void addCollisionObjectToEngine(CollisionObject* o) override;
  void addCollisionObjectsToEngine(const std::vector<CollisionObject*>&) override;
  void removeCollisionObjectFromEngine(CollisionObject* o) override;
  void removeAllCollisionObjectsFromEngine() override;
  void updateCollisionGroupEngineData() override;   // push refreshed AABBs into broadphase
private:
  std::size_t assignId(NativeCollisionObject* o);   // free-list, else mNextId++
  std::unique_ptr<native::BroadPhase>                     mBroadPhase;         // BruteForce (phase-2)
  std::vector<CollisionObject*>                           mCollisionObjects;   // deterministic add order
  std::unordered_map<std::size_t, NativeCollisionObject*> mIdToObject;
  std::unordered_map<CollisionObject*, std::size_t>       mObjectToId;
  std::vector<std::size_t>                                mFreeIds;
  std::size_t                                             mNextId = 0;
};
```
- `assignId` uses `mFreeIds` recycling (ids need only be **unique+stable while resident**; BruteForce keys an `unordered_map<size_t,Aabb>`, so ids need not be dense).
- Ids are **per-group** (broadphase is per-group). The Sharable manager shares one `CollisionObject` across groups; each group assigns its own id. `removeAllCollisionObjectsFromEngine()` clears all four containers + `mBroadPhase->clear()`. No cross-group id leakage (Risk R4).
- **No generational counter, no `valid()`** — the `shared_ptr` in the base `ObjectInfo` guarantees liveness for the duration of any `collide()`.

### 1.3 Mapping: DART 6 `CollisionObject` ↔ native object state

| DART 6 side (base-provided) | Native side (adapter-owned) | Sync point |
|---|---|---|
| `ShapeFrame*` (identity key) | — | claim/create |
| `getShape()` (`dynamics::Shape`) | `mNativeShape` (`native::Shape`) via `NativeShapeConversion` | ctor + lazy on version change |
| `getTransform()` (= `mShapeFrame->getWorldTransform()`) | `mNativeTransform` | `updateEngineData()` each frame |
| (derived) | `mNativeAabb` = `native::Aabb::transformed(mNativeShape->computeLocalAabb(), mNativeTransform)` | `updateEngineData()` each frame |
| `CollisionObject*` | `std::size_t` broadphase id | `addCollisionObjectToEngine` |

`NativeShapeConversion::create(const dynamics::Shape&) -> std::unique_ptr<native::Shape>`:
`SphereShape→native::SphereShape(r)`; `BoxShape→native::BoxShape(size/2)`; `CapsuleShape→native::CapsuleShape(r,h)`; `CylinderShape→native::CylinderShape(r,h)`; `PlaneShape→native::PlaneShape(n,offset)`; `MeshShape→native::MeshShape(...)`; convex-hull families (`MultiSphereConvexHullShape`, `ConeShape`, `PyramidShape`)→`native::ConvexShape(vertices)`. **Unsupported** (`EllipsoidShape`, `SoftMeshShape`, `HeightmapShape`, `VoxelGridShape`, `LineSegmentShape`) → `nullptr` + one-shot `dtwarn` (mirror FCL/DART unsupported handling). Null-native-shape objects are skipped in `collide()` and never produce contacts (Risk R5).

### 1.4 The three-tier refresh, realized (AABB call corrected — cpp17-entt/scope fix)

1. **Membership** (base `CollisionGroup::update()`): unchanged; drives the six hooks.
2. **Geometry** (tier-2): the detector-level hook stays a **no-op**; refresh is lazy — `NativeCollisionObject::updateEngineData()` rebuilds `mNativeShape` when **either the shape pointer changed** (a `ShapeFrame` re-assigned to a different `Shape` — fresh shapes commonly start at version 1, so a version-only guard would miss a swap; and the shape can be cleared to null) **or** the same shape's version advanced (an in-place edit). It mirrors `DARTCollisionObject`'s shape-cache refresh, which likewise keys on identity, not version alone.
3. **Transform** (tier-3):
```cpp
void NativeCollisionObject::updateEngineData() {
  // Rebuild on shape *swap* (identity) or in-place *edit* (version); handles
  // a shape being cleared to null. refreshCollisionObject() is a no-op, so
  // this is the only path that replaces stale native geometry (review fix).
  const dynamics::Shape* shape = getShape();
  if (shape != mLastKnownShape
      || (shape && shape->getVersion() != mLastKnownShapeVersion))
    rebuildNativeShape();  // resets mLastKnownShape + mLastKnownShapeVersion (null-safe)
  mNativeTransform = getTransform();                   // = mShapeFrame->getWorldTransform()
  if (mNativeShape)
    // native::Shape has no computeAabb(tf); combine locally then transform.
    mNativeAabb = native::Aabb::transformed(mNativeShape->computeLocalAabb(), mNativeTransform);
}
void NativeCollisionGroup::updateCollisionGroupEngineData() {
  for (auto* co : mCollisionObjects) {                 // base already called each obj's updateEngineData()
    auto* n = static_cast<NativeCollisionObject*>(co);
    auto it = mObjectToId.find(co);
    if (it != mObjectToId.end() && n->getNativeShape())
      mBroadPhase->update(it->second, n->getNativeAabb());
  }
}
```
> **API note (was flagged "verified" but wrong in the draft):** `native::Shape::computeAabb(const Eigen::Isometry3d&)` does **not** exist. The base exposes only `Aabb computeLocalAabb() const` (`shapes/shape.hpp:80`); the world-space combine that lived on the un-ported `native::CollisionObject::computeAabb()` is replaced by `native::Aabb::transformed(local, tf)` (`aabb.hpp:64` / `aabb.cpp:130`). Fully C++17, no new dependency.

### 1.5 How DART 6 `collide()` drives BruteForce → narrowphase → `Contact`

`checkGroupValidity` is **not** inherited — FCL/DART/Bullet each define their own `static bool checkGroupValidity(...)` in their `.cpp` (e.g. `DARTCollisionDetector.cpp:560`, `FCLCollisionDetector.cpp:734`). `NativeCollisionDetector.cpp` defines its **own copy**.

```cpp
bool NativeCollisionDetector::collide(
    CollisionGroup* group, const CollisionOption& option, CollisionResult* result) {
  if (result) result->clear();
  if (option.maxNumContacts == 0u) return false;
  if (!checkGroupValidity(this, group)) return false;      // detector-owned static; NOT inherited
  auto* g = static_cast<NativeCollisionGroup*>(group);
  g->updateEngineData();                                    // tiers 1-3 + broadphase AABBs
  if (g->mCollisionObjects.empty()) return false;

  bool collisionFound = false;
  native::CollisionOption nopt;                            // translate DART6 -> native option
  nopt.enableContact   = option.enableContact && (result != nullptr);
  nopt.maxNumContacts  = option.enableContact ? option.getEffectiveMaxNumContactsPerPair() : 1u;
  nopt.collisionFilter = nullptr;                          // DART6 filter applied at adapter level

  g->mBroadPhase->visitPairs([&](std::size_t idA, std::size_t idB) -> bool {
    auto* a = g->mIdToObject.at(idA);
    auto* b = g->mIdToObject.at(idB);
    if (!a->getNativeShape() || !b->getNativeShape()) return true;      // unsupported -> skip
    if (option.collisionFilter &&
        option.collisionFilter->ignoresCollision(a, b)) return true;    // DART6 filter

    native::CollisionResult scratch;                                    // move-only, per-pair
    const bool hit = native::NarrowPhase::collide(
        a->getNativeShape(), a->getNativeTransform(),
        b->getNativeShape(), b->getNativeTransform(), nopt, scratch);
    if (!hit) return true;
    collisionFound = true;
    if (!result) return false;                                          // binary mode: early-out
    if (!option.enableContact) {                                        // pair-only result mode
      Contact pairOnly;
      pairOnly.collisionObject1 = a;                                    // populate result caches
      pairOnly.collisionObject2 = b;                                    // no point/normal/depth
      result->addContact(pairOnly);
      return result->getNumContacts() < option.maxNumContacts;          // global cap reached?
    }
    if (emitContacts(scratch, a, b, option, *result)) return false;     // global cap reached
    return true;                                                        // keep visiting
  });
  return collisionFound;
}
```

**Contact / pair translation — the gz-required fields** (native `ContactPoint` → DART 6 `Contact`; `addContact` asserts non-null `collisionObject1/2`):
```cpp
Contact c;
c.point            = cp.position;                    // world-frame point                [gz-required]
c.normal           = orientNormal(cp.normal, a, b);  // world normal, obj2->obj1         [gz-required, R1]
c.penetrationDepth = cp.depth;                        //                                  [gz-required]
c.collisionObject1 = a;                              // MUST be non-null                 [gz-required]
c.collisionObject2 = b;                              // MUST be non-null                 [gz-required]
c.triID1 = cp.featureIndex1; c.triID2 = cp.featureIndex2;   // optional/legacy
// c.force left zero — set by the constraint solver, matches how FCL already behaves
if (!option.allowNegativePenetrationDepthContacts && c.penetrationDepth < 0.0) skip;   // R2
result.addContact(c);
```
When `option.enableContact == false` but `result != nullptr`, the adapter still
adds one pair-only `Contact` with just `collisionObject1/2` set. This preserves
DART's `CollisionResult` contract: `isCollision()` plus body/frame caches record
the colliding pair, while point/normal/depth remain default because contact
geometry was disabled. This pair-only path uses the same global
`option.maxNumContacts` cap and avoids calling `emitContacts()` on an empty
native scratch result.

`orientNormal` enforces DART 6's convention (**normal from `collisionObject2` toward `collisionObject1`**, per `Contact.hpp`'s field comment). The native dispatcher's canonical/flipped-normal plumbing fixes one order per pair, so the sign is validated **empirically against both `fcl` and `dart`** (§3, R1) and encoded once. Per-pair cap = `option.getEffectiveMaxNumContactsPerPair()` (native lacks this field → truncate `scratch` before emitting); global cap = `option.maxNumContacts` (stop `visitPairs` when reached).

**Cross-group `collide(g1, g2, …)`:** DART 6 semantics = pairs with one object from each group; intra-group pairs NOT checked. Implement as a nested loop over `g1->mCollisionObjects × g2->mCollisionObjects` (each `updateEngineData()` first), same filter + narrowphase + translation. BruteForce is per-group, so a transient combined broadphase is unnecessary in phase 2.

**`distance()` ×2:** stub exactly like DART — `dtwarn` + return `0.0` (phase-3 wires `native::NarrowPhase::distance`). **`raycast()`:** inherit the base warning (phase-3).

### 1.6 Detector class + registration (factory-string-only — gz-compat/cpp17 fix #1/#3)

```cpp
class NativeCollisionDetector : public CollisionDetector {
public:
  using CollisionDetector::createCollisionGroup;                 // un-hide variadic templates
  static std::shared_ptr<NativeCollisionDetector> create();
  static const std::string& getStaticType();                     // returns "native"
  const std::string& getType() const override { return getStaticType(); }
  std::shared_ptr<CollisionDetector> cloneWithoutCollisionObjects() const override;
  std::unique_ptr<CollisionGroup> createCollisionGroup() override;
  bool   collide(CollisionGroup*, const CollisionOption&, CollisionResult*) override;   // x2
  double distance(...) override;                                  // x2 (stubbed)
protected:
  NativeCollisionDetector() { mCollisionObjectManager.reset(
      new ManagerForSharableCollisionObjects(this)); }           // as DART
  std::unique_ptr<CollisionObject> createCollisionObject(const dynamics::ShapeFrame*) override;
  void refreshCollisionObject(CollisionObject*) override {}       // no-op; lazy in object
private:
  static Registrar<NativeCollisionDetector> mRegistrar;
};
// .cpp:
const std::string& NativeCollisionDetector::getStaticType(){ static const std::string t="native"; return t; }
NativeCollisionDetector::Registrar<NativeCollisionDetector>
  NativeCollisionDetector::mRegistrar{ getStaticType(),
    []() -> std::shared_ptr<CollisionDetector> { return create(); } };
```

Registration is self-contained: the static `Registrar` inserts `("native" → create)` into the process-wide `CollisionDetector::Factory` at static-init (the same `Registrar<Derived> mRegistrar{getStaticType(), lambda}` idiom FCL/DART use verbatim).

**Selection surface — there are three factory entry points, and the correct opt-in call does NOT use a string overload of `World`:**

1. **Factory pointer (the gz/downstream path):**
   ```cpp
   world->setCollisionDetector(
       dart::collision::CollisionDetector::getFactory()->create("native"));
   ```
   `World::setCollisionDetector` has **only** `(const collision::CollisionDetectorPtr&)` and `(CollisionDetectorType enum)` overloads (`World.hpp:262-266`). **There is no `setCollisionDetector(const char*)` / string overload** — `world->setCollisionDetector("native")` does **not compile**. Downstream/gz selection therefore goes through the factory-created pointer, exactly as gz-physics already selects detectors.
2. **Direct factory:** `CollisionDetector::getFactory()->create("native")`.
3. **SKEL parser:** `SkelParser.cpp:792-793` routes any `<collision_detector>` value through `getFactory()->create(cdType)` (unknown → warn → fcl fallback, `SkelParser.cpp:795-799`). Registering `"native"` makes `<collision_detector>native</collision_detector>` resolve to the new detector where it previously warned + fell back to FCL. This is **inert** for all shipped content (no shipped SKEL references `"native"`) and is **not** gz-visible (gz does not parse SKEL), but it is a real new resolution path and gets a guard test in P3a.

**Phase 2 adds NO `simulation::CollisionDetectorType::Native` enum.** It does NOT touch `World.hpp/.cpp`, `WorldConfig::collisionDetector` (stays `Fcl`), `ConstraintSolver` (stays hard-coded FCL), `resolveCollisionDetector`, or `toCollisionDetectorKey`. Adding a `Native` enum value is **explicitly forbidden** — it would extend a public enum consumed by `WorldConfig` and force edits to `World.cpp/.hpp` (the `switch` at `World.cpp:83-99`), i.e. exactly the gz-visible surface obligation #1 promises never to touch. The obvious "fix" for the non-existent string overload is that enum edit; we forbid it and use the factory pointer instead. This is the "new internal factory alias, no default flip" the brief requires.

---

## 2. Narrowphase completion (order + the bespoke-dispatcher decision)

Coverage target = **primitive (sphere/box/capsule/cylinder) + convex + mesh + plane**. Already on `release-6.20`: `sphere_sphere`, `box_box*`. To port (all EnTT-free; the only substitutions are `std::span`→`detail/span.hpp span`, and dropping `collision_object.hpp` + its `const CollisionObject&` overloads from the dispatcher):

| Ported file | Provides | `<span>`? | Hard dep |
|---|---|---|---|
| `sphere_box` | `collideSphereBox` (**distinct explicit branch**, not a box_box/convex fallthrough) | none | ported-base only |
| `capsule_sphere` | `collideCapsuleSphere` | none | ported-base only |
| `capsule_box` | `collideCapsuleBox` | none | ported-base only |
| `capsule_capsule` | `collideCapsules` | span | ported-base only |
| `convex_convex` | `collideConvexConvex` (GJK/MPR/EPA fallback — **keystone**), `distanceConvexConvex` | span | rides ported gjk/mpr |
| `cylinder_collision` | `collideCylinder{Sphere,Box,Capsule,Plane}`, `collideCylinders` | span | **`#include`s `convex_convex.hpp`** |
| `mesh_mesh` | `collideMeshMesh`, `collidePrimitiveMesh`, `collidePlaneMesh` | span | **`#include`s `convex_convex.hpp`** |
| `distance` | `distancePlaneShape` (+ support helpers) | span | ported-base only |
| `plane_sphere` | `collidePlane{Sphere,Box,Capsule,Convex}` — **all plane-vs-X live here** | none | **`#include`s `distance.hpp`** |
| `narrow_phase` (dispatcher) | `NarrowPhase::collide(Shape*,tf,Shape*,tf,opt,res)`, `collideBatch`, `isSupported`; `collideShapes` if-chain + `collideWithFlippedNormals` | span | **needs every pair it routes**; **DROP `#include collision_object.hpp` + the `const CollisionObject&` overloads** |

**Forced ordering (verified by real `#include` edges):** `convex_convex` → {`cylinder_collision`, `mesh_mesh`}; `distance` → `plane_sphere`. `convex_convex` is mandatory even for sphere/box-vs-convex — there are no explicit Convex branches; convex pairs fall through `collideShapes` to `collideConvexConvex`. `collideWithFlippedNormals` (canonical-order normal flip) and keeping `isSupported` in sync with the dispatch chain are required connective tissue. Note `plane_sphere.cpp:33` includes **only** `distance.hpp`, so P9 needs P8 but **not** P5.

### 2.1 `narrow_phase.{hpp,cpp}` is a BESPOKE reduced dispatcher, not a clean port (scope fix #1, cpp17 fix #4)

`origin/main`'s `narrow_phase.cpp` is **1344 lines** and `#include`s all 14 pair headers, calling every pair function — it cannot compile without every pair `.cpp`. So P2 must hand-trim it to sphere/box, and P4/P5/P6/P7/P9 each re-expand it. **This one file therefore never lands as a verbatim copy**; treat it as a bespoke artifact and gate it explicitly:

- **Per-PR fidelity gate:** on each dispatcher-touching PR, `git diff origin/main -- dart/collision/native/narrow_phase/narrow_phase.cpp` must reduce to **exactly** {span-shim substitution, dropped-`CollisionObject`-overloads-and-their-supporting-`Ray`/`CcdOption`/`DistanceResult`-types, the branches not yet added}. Any other delta fails the PR.
- **Header fork:** dropping `collision_object.hpp` forces removing the `const CollisionObject&` declarations of `collide`/`distance`/`raycast`/`sphereCast`/`capsuleCast` from `narrow_phase.hpp`; the header keeps **only** the `Shape*+tf` entry points. This widening of the header fork is expected and covered by the same gate.
- **P9 convergence check:** after P9, `narrow_phase.cpp` must equal `main`'s **modulo** the known deltas above (span shim + dropped `CollisionObject` overloads). A CI assertion enforces this so the fork provably re-converges.
- **Per-PR invariant scoping:** the "unrouted pair → `false`" invariant (P2) and the "everything convex falls through to `collideConvexConvex`" invariant (§2) are **both true only at their own phase**. The convex fallthrough (`narrow_phase.cpp:439-445`) does not exist until `convex_convex` lands in P5, so pre-P5 convex/mesh pairs also return `false` (acceptable — the converter cannot yet produce them). Each PR states which invariant holds at its phase.

Every PR description for P2/P4/P5/P6/P7/P9 flags `narrow_phase.{hpp,cpp}` as a **bespoke reduced dispatcher** so reviewers do not expect a clean `main` diff.

---

## 3. PR slicing (phase 2, ordered)

Two tracks. **Track A** = pure `dart::collision::native` engine (no wiring — cannot affect FCL, safest). **Track B** = adapter (wiring, but non-default). **Bridge-early ordering** de-risks the DART6↔native translation (R1) before ~5k lines of narrowphase land. The P3 bridge is split into **P3a (skeleton + registration)** and **P3b (translation + parity)** so the review-critical surface is not one oversized PR (scope fix #2).

**Every PR:** milestone **DART 7.0**; branch off `release-6.20`; no AI-attribution trailer; address codecov gaps on new files before merge. Re-request a Codex review after each push (`@codex review`) **only within the approval boundary of `docs/onboarding/ai-tools.md`** — a bare agent needs explicit maintainer/user approval before posting PR comments; this is pre-authorized only when the active task (e.g. the current dependency-minimization goal) grants it. **Gates** (note `pixi run build`/`test-all` alone are insufficient — see below):
- **Debug + Release build:** `pixi run build` is Release-only (`BUILD_TYPE=Release` is fixed in `pixi.toml`), so build Debug explicitly, e.g. `pixi run cmake -S . -B build/default/cpp/Debug -DCMAKE_BUILD_TYPE=Debug …` then `cmake --build build/default/cpp/Debug`. (Debug "nanobind not found" is the known poisoned-CMakeCache issue — `rm` the Debug `CMakeCache.txt` + reconfigure.)
- **Runtime tests:** `pixi run test-all` only *builds* the CMake `ALL` target; it does **not** run tests. Execute them with `ctest --test-dir build/default/cpp/Release -R UNIT_collision_native --output-on-failure` (and `pixi run test` for the broader suite where practical).
- `pixi run check-lint` (full aggregate), guard-row hash capture unchanged, `pixi run -e gazebo test-gz`, and a **scope-diff assertion** (touches only `dart/collision/native/**` + its `CMakeLists` + `tests/**` — never `World.*`, `ConstraintSolver.*`, `WorldConfig`, `simulation/`, `constraint/`, or any `fcl/`/`bullet/`/`ode/` file, and never adds a `CollisionDetectorType` enum value).

> **Honest gate caveat (scope fix #5):** on the engine-only slices **P1, P2, P8** — and on **P3a**, whose `collide()` is still a stub — nothing new is reachable from the FCL-default gz path, so `test-gz` is a **non-regression guard, expected trivially green**; the substantive gates there are the new unit tests + unchanged guard-row hashes. Genuine gz-relevant signal on the new subsystem begins at **P3b**. P2 deliberately lands a dispatcher that no product code calls until P3b (and that P4/P5 re-expand); this is the accepted cost of keeping Track A pure, and it is bounded by the §2.1 bespoke-dispatcher gate.

| PR | Scope | New/edited files | Depends | Notes / invariant |
|---|---|---|---|---|
| **P1 (FIRST)** | BroadPhase base + BruteForce | +`broad_phase/broad_phase.hpp`, +`broad_phase/brute_force.{hpp,cpp}`, +`tests/unit/collision/native/test_brute_force.cpp`, edit 2 CMake lists | phases 0-1 | Pure engine, zero wiring. `test-gz` = non-regression guard. Span-shim `broad_phase.hpp` only. |
| **P2** | Minimal NarrowPhase dispatcher (sphere/box only) | +`narrow_phase/narrow_phase.{hpp,cpp}` **bespoke-trimmed** to `sphere_sphere`+`box_box` branches; drop `collision_object.hpp` + all `CollisionObject` overloads; span-shim; +`test_narrow_phase_dispatch.cpp`; edit CMake | phases 0-1 | Header exposes `Shape*+tf` only. Unrouted → `false` (valid at this phase; §2.1). Pure engine; §2.1 fork gate applies. |
| **P3a** | Adapter skeleton + converter (sphere/box) + `"native"` registration | +`NativeCollisionDetector/Group/Object.{hpp,cpp}`, +`detail/NativeShapeConversion.{hpp,cpp}` (sphere,box); +`tests/unit/collision/test_NativeCollisionDetector.cpp` (creation/registration only); edit CMake; edit `tests/integration/test_Collision.cpp` (add `EXPECT_TRUE(canCreate("native"))`) | P1,P2 | `collide()` is a documented stub returning `false`. **Proof gates:** `EXPECT_TRUE(getFactory()->canCreate("native"))`; SKEL guard test (unknown/default string → warn → fcl, unchanged). Object/group creation validated. Two-namespace dir documented in PR body. |
| **P3b** | Bridge translation + `sphere_box` collider + normal calibration + parity | implement `collide()` translation in `NativeCollisionDetector.cpp` (`emitContacts`, `orientNormal`, per-pair/global caps, neg-depth); +`sphere_box.{hpp,cpp}` + its `collideShapes`/`isSupported` branch (pulled forward from Map-4 PR1 so the converter's sphere+box pairs route); extend parity test | P3a | **The bridge, validated.** Parity vs **both `fcl` and `dart`** on sphere-sphere / box-box / **sphere-box** (normal sign, depth, contact count). R1 normal sign encoded once here. No default flip. §2.1 fork gate applies. |
| **P4** | Remaining no-span primitives | +`capsule_sphere`,+`capsule_box`; dispatcher branches + `isSupported`; converter(capsule); extend parity; CMake | P3b | Map 4 PR1 remainder (`sphere_box` already landed in P3b). |
| **P5** | Convex foundation + span exemplar | +`convex_convex`,+`capsule_capsule`; dispatcher branches; converter(convex-hull families); tests; CMake | P4 | Map 4 PR2. Enables all convex-vs-* via fallback. First span shim in a pair file. |
| **P6** | Cylinder | +`cylinder_collision`; dispatcher branches (Cyl×{Sph,Box,Cap,Plane}); converter(cylinder); tests; CMake | P5 (`convex_convex`) | Map 4 PR3. |
| **P7** | Mesh | +`mesh_mesh`; dispatcher branches (Mesh×Mesh, primitive×Mesh, Plane×Mesh); converter(mesh); tests; CMake | P5 (`convex_convex`) | Map 4 PR4. Largest file. |
| **P8** | Distance module | +`distance.{hpp,cpp}` (**needs span shim — `distance.cpp` includes `<span>`**); CMake | phases 0-1 | Map 4 PR5, pulled forward for plane. `distance()` detector methods stay stubbed in phase 2. `test-gz` = non-regression guard. |
| **P9** | Plane coverage (completes phase 2) | +`plane_sphere`; dispatcher branches (Plane×{Sph,Box,Cap,Convex}); converter(plane); parity tests; CMake; **§2.1 P9 convergence check** | P8 (`distance`) | Map 4 PR6. After this, `isSupported`/parity cover primitive+convex+mesh+plane; dispatcher re-converges to `main`. |

**Optional P10 (recommended follow-up, still phase 2):** a mixed-scene integration parity test running the covered shape set through **`fcl`, `dart`, and `native`**, asserting equivalent collision decisions (guards the whole bridge; extends the `test_DARTCollisionDetector.cpp` template with an FCL comparison lane). Order of P4–P7 is flexible except `convex_convex` (P5) must precede P6/P7; `distance` (P8) must precede P9.

---

## 4. First PR — precise spec (P1: BroadPhase base + BruteForce)

**Rationale:** smallest self-contained unit on the adapter's critical path; verified fully EnTT-free (`broad_phase.hpp` includes only `aabb.hpp`/`export.hpp`/std; `brute_force.hpp` pulls only `broad_phase.hpp`+`<unordered_map>`; `.cpp` only `<algorithm>`); zero wiring ⇒ zero behavior change ⇒ `test-gz` trivially green; establishes the phase-2 native-port pattern (CMake + test). BruteForce is the correct phase-2 starting broadphase.

**Files to ADD** (port from `origin/main`, apply span shim):
- `dart/collision/native/broad_phase/broad_phase.hpp` — abstract `BroadPhase` with **7 pure virtuals** (`clear`, `add`, `update`, `remove`, `queryPairs`, `queryOverlapping`, `size`; `broad_phase.hpp:128-138`) + header-inline defaults (`visitPairs`, `queryPairs(out)`, `build`, `updateRange`, `buildDebugSnapshot`) + `queryPairsFiltered` template + `BroadPhasePair`/`BroadPhasePairVisitor` + debug POD structs. **No `broad_phase.cpp` exists on `main` — the base is header-only; do not invent one.** Substitute `#include <span>`+`std::span`→`#include <dart/collision/native/detail/span.hpp>`+`span` (`build`/`updateRange` at lines 168/180 take the span).
- `dart/collision/native/broad_phase/brute_force.hpp` — `class DART_COLLISION_NATIVE_API BruteForceBroadPhase : public BroadPhase` (`BruteForceBroadPhase() = default`).
- `dart/collision/native/broad_phase/brute_force.cpp` — `unordered_map<size_t,Aabb>` + sorted `orderedIds_`; O(n²) `queryPairs`/`visitPairs` using `Aabb::overlaps` (`aabb.hpp:48`), canonical `first<second`.
- `tests/unit/collision/native/test_brute_force.cpp` — **adapted from `main`'s flat `tests/unit/collision/test_brute_force.cpp` (252 lines; includes only `brute_force.hpp` + gtest + `<algorithm>`, no `std::span`/`DebugSnapshot`), relocated to the `release-6.20` `native/` subdir.** GoogleTest: add/update/remove changes `size()`; `queryPairs()` finds exactly the overlapping pairs on a hand-built set (incl. a non-overlapping control); pairs canonically ordered + deterministic across two runs; `visitPairs` early-out honored (return `false` stops); `queryOverlapping(aabb)` correctness.

**Files to MODIFY:**
- `dart/collision/native/CMakeLists.txt` — add `broad_phase/broad_phase.hpp`, `broad_phase/brute_force.hpp` to `collision_native_headers`; add `broad_phase/brute_force.cpp` to `collision_native_sources`.
- `tests/unit/collision/native/CMakeLists.txt` — append:
  ```cmake
  dart_add_test("unit" UNIT_collision_native_brute_force test_brute_force.cpp)
  ```

**Registration/wiring:** NONE. The broadphase is an internal engine type; not factory-registered; touches no `dart::collision` public surface. No `World`/`ConstraintSolver`/enum changes.

**Gates (all must pass before merge):**
1. **Release build:** `pixi run config` + `pixi run cmake --build build/default/cpp/Release --target ALL --parallel 8`. **Debug build:** configure a Debug tree explicitly (`pixi run` is Release-fixed) — `pixi run cmake -S . -B build/default/cpp/Debug -DCMAKE_BUILD_TYPE=Debug -DCMAKE_PREFIX_PATH=$CONDA_PREFIX …` then build. (Debug "nanobind not found" is the poisoned-CMakeCache issue: `rm` the Debug `CMakeCache.txt` + reconfigure — not a code bug.)
2. **Run the tests** (not just `test-all`, which only builds): `ctest --test-dir build/default/cpp/Release -R UNIT_collision_native --output-on-failure` — the new `UNIT_collision_native_brute_force` + existing `UNIT_collision_native_*` green.
3. `pixi run -e gazebo test-gz` — **non-regression guard, expected trivially green** (nothing FCL/World/solver changed).
4. `pixi run check-lint` — full aggregate (formatting, include order, license header on the 4 new files).
5. **Guard-row hashes unchanged** — run the committed phase-0 capture/guard script; confirm identical hashes.
6. **Portability lint** — `git grep -nE 'entt|EnTT|#include <span>|std::span' dart/collision/native/broad_phase/` returns nothing.
7. **Scope-diff assertion** — `git diff --name-only origin/release-6.20...HEAD` lists only the 4 new files + the 2 CMake edits; nothing under `simulation/`, `constraint/`, or `collision/{fcl,bullet,ode}/`.

---

## 5. gz-compat obligations (must hold on every phase-2 PR)

**Reviewed against `origin/release-6.20`; verdict SOUND-WITH-FIXES — no evidence-backed gz-visible perturbation under the scope gates. The gz-compat guarantee is entirely load-bearing on the §3/§4 scope-diff gate; enforce it literally.**

1. **FCL stays default, byte-for-byte.** Do not touch `WorldConfig::collisionDetector` (stays `Fcl`), `ConstraintSolver`'s two FCL-hard-coded ctors, `resolveCollisionDetector` (returns `nullptr` for the `Fcl` default, `World.cpp:155-172`), or `World::toCollisionDetectorKey` (`World.cpp:83-99`). The native detector is reachable ONLY via explicit factory selection. **No `CollisionDetectorType::Native` enum in phase 2** — adding it is explicitly forbidden in the P3a checklist because it drags in `World.cpp/.hpp` + `WorldConfig` edits (the exact gz surface obligation #1 protects). This keeps the public typed API and gz's selection surface unchanged.
2. **Selection is factory-pointer, not a `World` string overload.** gz selects via `world->setCollisionDetector(CollisionDetector::getFactory()->create("native"))`. `World::setCollisionDetector` has no `const char*` overload; documenting a `setCollisionDetector("native")` call would be a non-compiling instruction, so the plan/tests use the factory-pointer path (verified compilable). The SKEL entry point (`SkelParser.cpp:792`) is noted and guard-tested (unknown/default → warn → fcl, unchanged).
3. **Do not port the EnTT world layer.** `collision_world.*`, `collision_object.*`, `batch.hpp`, `comps/`, native `collision_filter.*`, `persistent_manifold_cache.*` stay off `release-6.20` in phase 2. Consequently the `getRegistry()` `entt::registry&` public leak (an ABI break) and the entt-in-`CollisionFilter` coupling never reach the release branch — no dartpy/test/`dart_collision_*` audit needed. CI lint: `git grep entt dart/collision/native` stays empty across all phase-2 PRs.
4. **Adapter implements exactly the DART 6 public contract** (`CollisionDetector`/`Group`/`Object`/`Result`/`Contact`) that gz-physics/gz-sim consume — a standard detector to every downstream caller. Base classes are **not** modified (no new pure-virtual on `CollisionObject`/`CollisionGroup`), so Bullet/Ode/downstream subclasses still compile; the `collision-bullet`/`collision-ode` CMake components are untouched (the native adapter compiles into the core `dart` target via `dart_add_core_*`, registers no `add_component`). Populate `point`/`normal`/`penetrationDepth`/`collisionObject1`/`collisionObject2` on every emitted `Contact` (`addContact` asserts the two object pointers non-null); leave `force` zero for the solver (matches FCL's existing behavior — `force` is default-zeroed by the `Contact` ctor).
5. **`CollisionOption` struct is not perturbed.** `maxNumContactsPerPair`, `allowNegativePenetrationDepthContacts`, and `getEffectiveMaxNumContactsPerPair()` (`min(perPair,global)`, or global when per-pair is 0) are consumed by the adapter itself; the struct FCL reads is unchanged.
6. **Merge gates:** `pixi run -e gazebo test-gz` green + guard-row hashes identical on every PR — both trivially so on P1/P2/P3a/P8 (non-regression) precisely because the FCL codepath is never touched; substantively meaningful from P3b onward.
7. **Release-branch working principles** (`docs/ai/principles.md`): source-of-truth for the ported engine code is `origin/main` (DART 7 already has it), so these are main→`release-6.20` ports, not the usual bugfix dual-PR — no forward-port to main required. `narrow_phase.{hpp,cpp}` is the one **bespoke** exception (§2.1), not a clean port. Confirm the additive/opt-in framing (new files + one inert factory key) with the maintainer before the adapter PR (P3a) per the approval-boundary rule.

---

## 6. Risks + mitigations

**Correctness / parity (rank-ordered):**
- **R1 — Normal-direction convention (top risk).** DART 6 `Contact.normal` = obj2→obj1; the native sign is set by the dispatcher's canonical/flipped-normal plumbing per pair-order. Mitigation: in **P3b**, land `orientNormal` and a parity test that drops a sphere on a box and asserts the native normal sign matches **both `fcl` (gz's production backend) and `dart`** on the covered set; encode the correction once, re-verify per new pair family (P4–P9). (gz-compat review: assert against FCL specifically, since `dart`'s contact conventions are not identical to FCL's in all pair cases.)
- **R2 — `CollisionOption` feature gap.** Native option lacks `maxNumContactsPerPair` and `allowNegativePenetrationDepthContacts`. The adapter enforces both: truncate per-pair to `getEffectiveMaxNumContactsPerPair()`; drop `depth<0` unless `allowNegativePenetrationDepthContacts`. Covered by P3b tests.
- **R3 — Contact-order determinism.** DART 6 relies on deterministic contact order for clone parity (`ObjectInfoList` is a deliberate `std::vector`). Keep `mCollisionObjects` in add order; BruteForce yields canonical `first<second` pairs (P1 test asserts determinism); visit order tied to id order.
- **R4 — Per-group id scoping.** Sharable manager shares one `CollisionObject` across groups; ids are per-group. `removeAllCollisionObjectsFromEngine` resets all four containers + `mBroadPhase->clear()`. Test: same frame in two groups collides independently.
- **R5 — Shape coverage gaps.** Ellipsoid/SoftMesh/Heightmap/VoxelGrid/LineSegment → converter returns `nullptr` + one-shot warn; `collide()` skips null-shape objects (never crash). Mirror FCL/DART. Document the covered set in each PR.
- **R6 — Lazy geometry refresh.** `updateEngineData()` must rebuild the native shape on `Shape` version change and re-push the AABB (via `native::Aabb::transformed(computeLocalAabb(), tf)`) to the broadphase; guard with `mLastKnownShapeVersion`.

**Portability / build:**
- **R7 — EnTT leakage via the dispatcher header.** `narrow_phase.hpp` (main) `#include`s `collision_object.hpp` (EnTT) for its `const CollisionObject&` overloads. **P2 drops that include + those overloads + their `Ray`/`CcdOption`/`DistanceResult` supporting decls**, keeping only `Shape*+tf`. CI lint `git grep entt dart/collision/native` stays empty across all phase-2 PRs.
- **R8 — `std::span` substitution.** Every ported file with `<span>`/`std::span` → `detail/span.hpp span` (proven in `box_box`/`sphere_sphere`). Applies to `broad_phase.hpp` (P1), the dispatcher (P2+), the span-bearing pair files (P5+), and **`distance.cpp` (P8)**. Lint-gated.
- **R9 — CMake explicit list.** Every new `.hpp`/`.cpp` must join `collision_native_headers`/`collision_native_sources` (tests → the test CMake) or it silently won't compile/run. Per-PR checklist item.
- **R10 — `narrow_phase` bespoke-fork drift.** The dispatcher is hand-trimmed then progressively re-expanded (§2.1). Guard with the per-PR `git diff origin/main` reduction gate and the P9 convergence check so each re-added branch is provably upstream-faithful.
- **R11 — Debug-config false failure.** `test-all` "Build Debug FAILED" is frequently the poisoned-CMakeCache `nanobind_DIR-NOTFOUND`, not a code bug — clear the Debug cache + reconfigure.
- **R12 — Static-init registration silently not linking.** A static `Registrar` can fail to link into the final target. **P3a proves it** with `EXPECT_TRUE(getFactory()->canCreate("native"))` added to `Collision.Factory` (`tests/integration/test_Collision.cpp`).

**Key absolute paths:** engine dir `dart/collision/native/`; CMake `dart/collision/native/CMakeLists.txt`; adapter template `dart/collision/dart/DARTCollision{Detector,Group,Object}.{hpp,cpp}`; test dirs `tests/unit/collision/native/` (+ its `CMakeLists.txt`) and `tests/unit/collision/test_DARTCollisionDetector.cpp` (adapter-test template); factory enumeration test `tests/integration/test_Collision.cpp` (`Collision.Factory`); base contract `dart/collision/{CollisionDetector,CollisionGroup,CollisionObject,Contact,CollisionResult,CollisionOption}.hpp`; selection sites `dart/simulation/World.{hpp,cpp}`, `dart/utils/SkelParser.cpp`.

---

## 7. Resolved critique items

Three adversarial reviews (gz-compat, cpp17-entt, scope) each returned **SOUND-WITH-FIXES**. Every concrete defect is resolved here:

| # | Source lens(es) | Defect | Resolution in this plan |
|---|---|---|---|
| 1 | gz-compat, cpp17-entt, scope | `World::setCollisionDetector("native")` has no matching overload (won't compile); the naive enum "fix" would edit `World.*`/`WorldConfig` (a gz-compat break) | §1.6 / obligation #2: opt-in is `world->setCollisionDetector(CollisionDetector::getFactory()->create("native"))`; **`CollisionDetectorType::Native` enum is explicitly forbidden** in the P3a checklist; P3b parity exercises the factory-string path, never a `World` string call. |
| 2 | cpp17-entt, scope | `native::Shape::computeAabb(const Isometry3d&)` does not exist (used twice, mislabeled "verified") | §1.3/§1.4: replaced both sites with `native::Aabb::transformed(mNativeShape->computeLocalAabb(), mNativeTransform)` (`aabb.hpp:64`) — C++17, no new dep. |
| 3 | cpp17-entt | P3 asserted **sphere-box** parity, but `sphere_box` collider lands in P4; the trimmed dispatcher returns `false` while FCL reports a contact → test fails | `sphere_box.{hpp,cpp}` + its `collideShapes`/`isSupported` branch **pulled forward into P3b** (converter already covers sphere+box); P4 becomes the capsule-primitive remainder. |
| 4 | scope, cpp17-entt | `narrow_phase.{hpp,cpp}` is a hand-trimmed divergent fork across six PRs, not a clean port; reviewers can't diff against `main` | §2.1: designated a **bespoke reduced dispatcher** with a per-PR `git diff origin/main` reduction gate, a header-fork note, per-phase invariant scoping, and a **P9 convergence check** back to `main` modulo known deltas. |
| 5 | scope | P3 was the heaviest slice yet presented as one atomic "small" PR (bridge + converter + registration + R1 calibration + parity) | Split into **P3a** (skeleton + converter + `"native"` registration, `collide()` stub) and **P3b** (translation + `sphere_box` + normal calibration + parity). |
| 6 | gz-compat | SKEL parser is a third factory entry point omitted from the selection enumeration | §1.6 point 3 + obligation #2: SKEL path documented; **P3a adds a guard test** that unknown/default `<collision_detector>` still warns → falls back to fcl, unchanged. |
| 7 | gz-compat | R1 parity oracle cited only the `dart` detector; gz's production backend is **FCL** | R1 + P3b/P10: parity asserts native `point`/`normal`/`penetrationDepth` against **both `fcl` and `dart`**. |
| 8 | cpp17-entt | `checkGroupValidity` read as inherited; it is a per-detector `static` in each `.cpp` | §1.5: `NativeCollisionDetector.cpp` defines its **own** `static checkGroupValidity`, stated explicitly. |
| 9 | scope | `test-gz` "every PR gz-gated" oversold coverage on engine-only PRs; P2 lands dead code | §3 gate caveat: `test-gz` is a **non-regression guard (trivially green)** on P1/P2/P3a/P8; substantive gates there are unit tests + guard-row hashes; genuine gz signal starts at P3b; P2's inert dispatcher is bounded by the §2.1 fork gate. |
| 10 | scope | Placing `dart::collision` adapter in `dart/collision/native/` makes one dir host two namespaces — not "structurally identical" to `dart/collision/dart/` | §1.1: the two-namespace layout is **documented as intentional** (snake_case=engine, PascalCase=adapter), the false "structurally identical" claim dropped, co-location justified, and the P3a PR body must call it out. |
| 11 | scope | P1 accuracy slips: test path is flat `tests/unit/collision/test_brute_force.cpp` on main; "8 pure virtuals" is 7; `distance.cpp` needs span shim | §4: test relabeled "adapted from main's flat path, relocated to `native/`"; corrected to **7 pure virtuals**; **P8 row now states the span shim** (`distance.cpp` includes `<span>`). |
| 12 | scope | P3 factory registration had no proof gate (static-init can silently fail to link) | §3/§6 R12: **P3a adds `EXPECT_TRUE(getFactory()->canCreate("native"))`** to `Collision.Factory`. |

**PR-review refinements (#3302).** Four further defects raised on the plan PR are resolved above: (13) the lazy geometry guard now keys on **shape identity + null**, not version alone, so a `ShapeFrame` re-assigned to a fresh shape (version 1) or cleared still rebuilds the native geometry (§1.4); (14) the gate text now spells out the **real Debug build** and **`ctest`** commands, since `pixi run build` is Release-only and `pixi run test-all` only builds (§3, §4); (15) the `@codex review` step is now qualified by the **`docs/onboarding/ai-tools.md` approval boundary** for agent-run PRs (§3); (16) the disabled-contact path now records a pair-only `Contact` when a `CollisionResult` is provided, preserving `isCollision()` and object caches while respecting `maxNumContacts` (§1.5).

**Net:** No fundamental infeasibility survived review. Nothing implicitly requires EnTT, a C++20/23 feature beyond the already-shimmed `std::span`, or a DART-7-only API. The two build-breaking items (non-existent `Shape::computeAabb(tf)`, non-existent `World::setCollisionDetector(const char*)`) are corrected to real `release-6.20` APIs; the PR-slicing bug (sphere-box parity before its collider) is fixed by pulling `sphere_box` into P3b; the reviewability defects (bespoke `narrow_phase` fork, oversized P3) are addressed with explicit gates and a P3a/P3b split. FCL-as-default and zero gz-visible behavior change are preserved and load-bearing on the scope-diff gate.
