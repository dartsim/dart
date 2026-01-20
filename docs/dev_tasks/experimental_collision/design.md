# Experimental Collision Module - Design Document

> **Version**: 0.1 (Draft)
> **Last Updated**: 2026-01-19

## Table of Contents

1. [Design Philosophy](#design-philosophy)
2. [Core Data Structures](#core-data-structures)
3. [Architecture](#architecture)
4. [API Reference](#api-reference)
5. [Determinism Guarantees](#determinism-guarantees)
6. [Extension Points](#extension-points)

---

## Design Philosophy

### Principles

1. **Standalone First**: Shapes and collision logic should work without `dart::dynamics` dependency
2. **Extensible Contact Representation**: Support point, edge, face, and analytic contacts for future deformable/cloth/codimensional simulation
3. **Bit-Exact Determinism**: Same input produces identical output across runs and platforms where possible
4. **Simple Start, Optimize Later**: Begin with brute-force, add sophisticated algorithms as needed
5. **Test-Driven**: Every feature has correctness tests and benchmarks before implementation

### ⚠️ CRITICAL: Performance Target

**This module MUST outperform existing collision backends (FCL, Bullet, ODE) to justify its existence.**

| Requirement                    | Description                                                       |
| ------------------------------ | ----------------------------------------------------------------- |
| **Faster than FCL/Bullet/ODE** | Every narrow-phase algorithm must beat or match existing backends |
| **Equal or better accuracy**   | Contact position, normal, and depth must be as accurate or better |
| **Feature parity**             | Support all shape pairs that existing backends support            |
| **Comparative benchmarks**     | All benchmarks must include FCL, Bullet, ODE results side-by-side |

**Benchmark Requirements:**

- Single-pair collision: Compare against `fcl::collide()`, `btCollisionWorld::contactPairTest()`, `dCollide()`
- Distance queries: Compare against `fcl::distance()`, Bullet GJK, ODE distance
- Broad-phase: Compare against FCL's `BroadPhaseCollisionManager`, Bullet's `btDbvtBroadphase`
- Full world collision: Compare CollisionWorld vs FCL/Bullet/ODE world collision

**Accuracy Verification:**

- Contact positions must match within 1e-6 tolerance
- Contact normals must match within 1e-6 tolerance
- Penetration depth must match within 1e-6 tolerance
- Number of contacts should be comparable (may differ due to algorithm differences)

**If we cannot demonstrate performance wins, this module should not be integrated into DART.**

### Naming Conventions

| Convention       | Example                                     |
| ---------------- | ------------------------------------------- |
| Files            | `snake_case.hpp`, `snake_case.cpp`          |
| Classes          | `PascalCase` (e.g., `CollisionObject`)      |
| Abbreviations    | `PascalCase` (e.g., `Aabb` not `AABB`)      |
| Functions        | `camelCase` (e.g., `getTransform()`)        |
| Member variables | `snake_case_` with trailing underscore      |
| Constants        | `kPascalCase` (e.g., `kDefaultMaxContacts`) |

---

## Core Data Structures

### Contact Point

```cpp
namespace dart::collision::experimental {

/// A single contact point between two collision objects.
///
/// Normal Convention: Points FROM object2 TO object1.
/// This matches existing DART convention for backward compatibility.
///
/// Penetration Convention: Positive = overlapping, zero = touching.
struct ContactPoint {
  /// Contact position in world frame
  Eigen::Vector3d position;

  /// Contact normal (from object2 to object1) in world frame
  Eigen::Vector3d normal;

  /// Penetration depth (positive = overlapping)
  double depth;

  /// Pointers to colliding objects (may be nullptr for standalone tests)
  const CollisionObject* object1 = nullptr;
  const CollisionObject* object2 = nullptr;

  /// Feature identification for mesh contacts (-1 = not applicable)
  int featureIndex1 = -1;
  int featureIndex2 = -1;

  // === Static Utilities ===

  /// Epsilon for zero-normal detection
  static constexpr double kNormalEpsilon = 1e-6;

  /// Check if normal is effectively zero (degenerate contact)
  [[nodiscard]] static bool isZeroNormal(const Eigen::Vector3d& n) {
    return n.squaredNorm() < kNormalEpsilon * kNormalEpsilon;
  }
};

} // namespace dart::collision::experimental
```

### Contact Manifold (Extensible)

```cpp
namespace dart::collision::experimental {

/// Type of contact geometry
enum class ContactType {
  Point,    ///< Single point contact (sphere-sphere, vertex-face)
  Edge,     ///< Line contact (edge-edge, edge-face)
  Face,     ///< Planar polygon contact (face-face)
  Patch,    ///< Curved surface contact (for future)
  Unknown   ///< Unclassified or mixed
};

/// A manifold groups related contacts between two objects.
///
/// For simple cases (sphere-sphere), this contains a single ContactPoint.
/// For complex cases (box-box face contact), this groups multiple
/// ContactPoints that share a common contact plane.
///
/// Design Note: This is designed to be extensible for future deformable
/// body, cloth, and codimensional object simulation.
class ContactManifold {
public:
  ContactManifold() = default;

  /// Add a contact point to this manifold
  void addContact(const ContactPoint& contact);

  /// Clear all contacts
  void clear();

  // === Accessors ===

  [[nodiscard]] std::size_t numContacts() const { return contacts_.size(); }
  [[nodiscard]] bool hasContacts() const { return !contacts_.empty(); }
  [[nodiscard]] const ContactPoint& getContact(std::size_t i) const;
  [[nodiscard]] std::span<const ContactPoint> getContacts() const;

  /// Get the contact type (inferred from contact geometry)
  [[nodiscard]] ContactType getType() const { return type_; }
  void setType(ContactType type) { type_ = type; }

  /// Get shared normal (for Face/Edge contacts, contacts share a normal)
  /// Returns zero vector if contacts have different normals.
  [[nodiscard]] Eigen::Vector3d getSharedNormal() const;

  /// Get the colliding objects
  [[nodiscard]] const CollisionObject* getObject1() const { return object1_; }
  [[nodiscard]] const CollisionObject* getObject2() const { return object2_; }
  void setObjects(const CollisionObject* o1, const CollisionObject* o2);

private:
  std::vector<ContactPoint> contacts_;
  ContactType type_ = ContactType::Unknown;
  const CollisionObject* object1_ = nullptr;
  const CollisionObject* object2_ = nullptr;
};

} // namespace dart::collision::experimental
```

### Collision Result

```cpp
namespace dart::collision::experimental {

/// Container for collision query results.
///
/// Supports both manifold-based access (recommended) and flat contact
/// list access (for compatibility).
class CollisionResult {
public:
  CollisionResult() = default;

  // === Modification ===

  /// Add a contact point (creates implicit single-point manifold)
  void addContact(const ContactPoint& contact);

  /// Add a complete manifold
  void addManifold(ContactManifold manifold);

  /// Clear all results
  void clear();

  // === Query ===

  /// Check if any collision occurred
  [[nodiscard]] bool isCollision() const { return !manifolds_.empty(); }
  [[nodiscard]] explicit operator bool() const { return isCollision(); }

  /// Get total contact count across all manifolds
  [[nodiscard]] std::size_t numContacts() const;

  /// Get number of manifolds
  [[nodiscard]] std::size_t numManifolds() const { return manifolds_.size(); }

  // === Access ===

  /// Manifold-based access (recommended)
  [[nodiscard]] const ContactManifold& getManifold(std::size_t i) const;
  [[nodiscard]] std::span<const ContactManifold> getManifolds() const;

  /// Flat contact access (for compatibility)
  /// Note: This is less efficient as it iterates all manifolds.
  [[nodiscard]] const ContactPoint& getContact(std::size_t i) const;

private:
  std::vector<ContactManifold> manifolds_;

  // Cached flat view (lazily computed)
  mutable std::vector<const ContactPoint*> flat_contacts_cache_;
  mutable bool flat_cache_valid_ = false;
  void invalidateCache() { flat_cache_valid_ = false; }
  void updateFlatCache() const;
};

} // namespace dart::collision::experimental
```

### Collision Option

```cpp
namespace dart::collision::experimental {

/// Options for collision queries.
struct CollisionOption {
  /// Whether to compute contact details (point, normal, depth).
  /// If false, only boolean collision result is computed (faster).
  bool enableContact = true;

  /// Maximum number of contacts to detect.
  /// Query terminates early once this limit is reached.
  /// Set to 1 for binary collision check.
  /// Set to 0 to short-circuit (returns false immediately).
  std::size_t maxNumContacts = 1000;

  /// Collision filter (optional).
  /// If set, pairs where filter->ignoresCollision() returns true are skipped.
  // CollisionFilter* filter = nullptr;  // Phase 2

  // === Factory Methods ===

  /// Create option for binary collision check (fastest)
  [[nodiscard]] static CollisionOption binaryCheck() {
    return {.enableContact = false, .maxNumContacts = 1};
  }

  /// Create option for full contact computation
  [[nodiscard]] static CollisionOption fullContacts(std::size_t max = 1000) {
    return {.enableContact = true, .maxNumContacts = max};
  }
};

} // namespace dart::collision::experimental
```

### Aabb (Axis-Aligned Bounding Box)

```cpp
namespace dart::collision::experimental {

/// Axis-Aligned Bounding Box for broad-phase culling.
class Aabb {
public:
  Aabb() : min_(Eigen::Vector3d::Zero()), max_(Eigen::Vector3d::Zero()) {}
  Aabb(const Eigen::Vector3d& min, const Eigen::Vector3d& max);

  // === Query ===

  /// Check if this AABB overlaps with another
  [[nodiscard]] bool overlaps(const Aabb& other) const;

  /// Check if this AABB contains a point
  [[nodiscard]] bool contains(const Eigen::Vector3d& point) const;

  /// Get center point
  [[nodiscard]] Eigen::Vector3d center() const;

  /// Get half-extents
  [[nodiscard]] Eigen::Vector3d halfExtents() const;

  /// Get volume
  [[nodiscard]] double volume() const;

  // === Modification ===

  /// Merge with another AABB (this becomes the union)
  void merge(const Aabb& other);

  /// Expand by margin in all directions
  void expand(double margin);

  // === Factory ===

  /// Create AABB for a sphere at origin
  [[nodiscard]] static Aabb forSphere(double radius);

  /// Create AABB for a box at origin
  [[nodiscard]] static Aabb forBox(const Eigen::Vector3d& halfExtents);

  /// Transform a local AABB to world frame
  [[nodiscard]] static Aabb transformed(
      const Aabb& local,
      const Eigen::Isometry3d& transform);

  // === Data ===
  Eigen::Vector3d min_;
  Eigen::Vector3d max_;
};

} // namespace dart::collision::experimental
```

---

## Architecture

### Layer Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                     User Code / DART Dynamics                   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    CollisionGroup (Phase 2)                     │
│  - Manages CollisionObjects                                     │
│  - Orchestrates broad-phase + narrow-phase                      │
│  - collide() / distance() queries                               │
└─────────────────────────────────────────────────────────────────┘
                              │
            ┌─────────────────┴─────────────────┐
            ▼                                   ▼
┌───────────────────────────┐     ┌───────────────────────────────┐
│        Broad-Phase        │     │         Narrow-Phase          │
│  BroadPhase (interface)   │     │  collide() dispatch function  │
│  - BruteForceBroadPhase   │     │                               │
│  - (future: BvhBroadPhase)│     │  Specialized algorithms:      │
│                           │     │  - collideSphereSphere()      │
│  Input: CollisionObjects  │     │  - collideBoxBox()            │
│  Output: Candidate pairs  │     │  - collideSphereBox()         │
└───────────────────────────┘     │  - (future: mesh algorithms)  │
                                  └───────────────────────────────┘
                                                │
                                                ▼
                                  ┌───────────────────────────────┐
                                  │        CollisionResult        │
                                  │  - manifolds_                 │
                                  │  - isCollision()              │
                                  │  - numContacts()              │
                                  └───────────────────────────────┘
```

### Narrow-Phase Dispatch

```cpp
namespace dart::collision::experimental {

/// Main narrow-phase entry point.
/// Dispatches to specialized shape-pair algorithms.
///
/// @param object1 First collision object
/// @param object2 Second collision object
/// @param option Collision options
/// @param[out] result Collision result (contacts appended)
/// @return Number of contacts generated (0 = no collision)
[[nodiscard]] int collide(
    const CollisionObject* object1,
    const CollisionObject* object2,
    const CollisionOption& option,
    CollisionResult& result);

// === Specialized Algorithms (in detail namespace) ===

namespace detail {

/// Sphere-sphere collision
[[nodiscard]] int collideSphereSphere(
    const CollisionObject* o1,
    double radius1,
    const Eigen::Isometry3d& T1,
    const CollisionObject* o2,
    double radius2,
    const Eigen::Isometry3d& T2,
    const CollisionOption& option,
    CollisionResult& result);

/// Box-box collision (SAT-based)
[[nodiscard]] int collideBoxBox(
    const CollisionObject* o1,
    const Eigen::Vector3d& halfExtents1,
    const Eigen::Isometry3d& T1,
    const CollisionObject* o2,
    const Eigen::Vector3d& halfExtents2,
    const Eigen::Isometry3d& T2,
    const CollisionOption& option,
    CollisionResult& result);

/// Sphere-box collision
[[nodiscard]] int collideSphereBox(
    const CollisionObject* sphereObj,
    double radius,
    const Eigen::Isometry3d& T_sphere,
    const CollisionObject* boxObj,
    const Eigen::Vector3d& halfExtents,
    const Eigen::Isometry3d& T_box,
    const CollisionOption& option,
    CollisionResult& result);

} // namespace detail
} // namespace dart::collision::experimental
```

### Broad-Phase Interface

```cpp
namespace dart::collision::experimental {

/// Broad-phase algorithm selection for CollisionWorld.
enum class BroadPhaseType {
  AabbTree,       ///< Dynamic AABB tree with SAH (default, best general-purpose)
  SweepAndPrune,  ///< Sorted endpoint lists (good for mostly-static scenes)
  BruteForce      ///< O(n²) all-pairs (debugging, very small N)
};

/// Abstract broad-phase interface.
///
/// Broad-phase algorithms identify candidate collision pairs
/// using bounding volume tests, reducing the number of expensive
/// narrow-phase checks.
class BroadPhase {
public:
  virtual ~BroadPhase() = default;

  /// Add an object to the broad-phase structure
  virtual void add(CollisionObject* object) = 0;

  /// Remove an object from the broad-phase structure
  virtual void remove(CollisionObject* object) = 0;

  /// Update an object's position (call after transform change)
  virtual void update(CollisionObject* object) = 0;

  /// Remove all objects
  virtual void clear() = 0;

  /// Get number of objects
  [[nodiscard]] virtual std::size_t size() const = 0;

  /// Compute potentially colliding pairs.
  /// Pairs are returned in deterministic order.
  [[nodiscard]] virtual std::vector<std::pair<CollisionObject*, CollisionObject*>>
  computePotentialPairs() const = 0;
};

} // namespace dart::collision::experimental
```

### Broad-Phase Algorithm Selection Guide

CollisionWorld supports runtime selection of broad-phase algorithms via the constructor:

```cpp
// Default: AABB Tree (best general-purpose performance)
CollisionWorld world;
CollisionWorld world(BroadPhaseType::AabbTree);

// For mostly-static scenes with few moving objects
CollisionWorld world(BroadPhaseType::SweepAndPrune);

// For debugging or very small object counts (< 20)
CollisionWorld world(BroadPhaseType::BruteForce);

// For uniform distributions with known object sizes
CollisionWorld world(BroadPhaseType::SpatialHash);
```

#### Algorithm Comparison

| Algorithm         | Complexity       | Best For                       | Trade-offs                               |
| ----------------- | ---------------- | ------------------------------ | ---------------------------------------- |
| **AABB Tree**     | O(n log n) query | Dynamic scenes, general use    | Higher insertion cost, memory overhead   |
| **Spatial Hash**  | O(1) avg query   | Uniform distributions, dense   | Cell size tuning, large objects are slow |
| **Sweep & Prune** | O(n + k) query   | Mostly-static, coherent motion | Must resort on large movements           |
| **Brute Force**   | O(n²) query      | Debugging, N < 20              | Simple but doesn't scale                 |

#### Performance Characteristics

**AABB Tree** (default):

- Uses Surface Area Heuristic (SAH) for balanced insertion
- Fat AABBs reduce update frequency for small movements
- 95-188x faster than brute-force at 500-1000 objects
- Best choice when objects move frequently

**Spatial Hash**:

- Objects hashed into 3D grid cells based on AABB
- O(1) average query time for uniform distributions
- Cell size should match average object size for best performance
- Large objects spanning many cells can degrade performance
- Good for dense scenes with similar-sized objects

**Sweep-and-Prune**:

- Maintains sorted endpoint lists per axis
- Exploits temporal coherence (objects don't move much between frames)
- Efficient when most objects are static with few moving
- Requires full resort if many objects move large distances

**Brute Force**:

- Simple O(n²) pairwise AABB tests
- No data structure overhead
- Useful for debugging or when N is very small
- Reference implementation for correctness testing

#### When to Use Each

| Scenario                              | Recommended Algorithm |
| ------------------------------------- | --------------------- |
| General robotics simulation           | AabbTree (default)    |
| Uniform object sizes, dense scene     | SpatialHash           |
| Static environment + one moving robot | SweepAndPrune         |
| Debugging collision issues            | BruteForce            |
| < 20 objects                          | Any (BruteForce OK)   |
| > 100 objects with frequent movement  | AabbTree              |
| > 1000 objects, mostly static         | SweepAndPrune         |
| Particle systems, uniform grids       | SpatialHash           |

#### Querying the Active Algorithm

```cpp
CollisionWorld world(BroadPhaseType::AabbTree);

// Query the type
BroadPhaseType type = world.getBroadPhaseType();

// Access the underlying broad-phase (for advanced use)
BroadPhase& bp = world.getBroadPhase();
```

---

## Determinism Guarantees

### Requirements

1. **Same Input = Same Output**: Identical collision queries produce identical results
2. **Order Independence**: Result does not depend on object insertion order (after sorting)
3. **Platform Consistency**: Same results on Linux/macOS/Windows (where floating-point allows)

### Implementation Strategy

| Requirement              | Implementation                                      |
| ------------------------ | --------------------------------------------------- |
| Ordered iteration        | Use `std::vector` for all collections               |
| Consistent pair ordering | Always order pairs by pointer: `(min_ptr, max_ptr)` |
| Stable contact order     | Add contacts in deterministic narrow-phase order    |
| No hash iteration        | Avoid `unordered_map/set` in hot paths              |
| Floating-point care      | Use consistent operations, avoid reassociation      |

### Batch Query Ordering

Batch APIs (e.g., collideAll/distanceAll/raycastAll) must return results in a deterministic order that does not depend on threading or insertion timing.

- Assign stable object IDs on creation; use IDs for ordering in public batch results.
- Sort pair results by `(id1, id2, query_index)` with `id1 < id2`.
- Sort raycast results by `(distance, object_id)` to keep hit order stable.
- Parallel implementations must merge per-thread results with a stable sort.

### Utility Function

```cpp
namespace dart::collision::experimental::detail {

/// Order a pair consistently by pointer value
inline std::pair<const CollisionObject*, const CollisionObject*>
orderPair(const CollisionObject* a, const CollisionObject* b) {
  return (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
}

} // namespace dart::collision::experimental::detail
```

---

## Extension Points

### Future Shape Support

```cpp
// shapes/shape.hpp - Base class for standalone shapes

namespace dart::collision::experimental {

/// Shape type enumeration
enum class ShapeType {
  Sphere,
  Box,
  Capsule,
  Cylinder,
  Cone,
  Plane,
  Mesh,
  Convex,
  // Future
  HeightField,
  PointCloud
};

/// Base class for collision shapes (standalone, no dynamics dependency)
class Shape {
public:
  virtual ~Shape() = default;

  [[nodiscard]] virtual ShapeType getType() const = 0;
  [[nodiscard]] virtual Aabb computeLocalAabb() const = 0;

  // For future: signed distance function
  // [[nodiscard]] virtual double signedDistance(const Eigen::Vector3d& point) const;
};

class SphereShape : public Shape {
public:
  explicit SphereShape(double radius) : radius_(radius) {}

  [[nodiscard]] ShapeType getType() const override { return ShapeType::Sphere; }
  [[nodiscard]] Aabb computeLocalAabb() const override;

  [[nodiscard]] double getRadius() const { return radius_; }

private:
  double radius_;
};

class BoxShape : public Shape {
public:
  explicit BoxShape(const Eigen::Vector3d& halfExtents) : halfExtents_(halfExtents) {}

  [[nodiscard]] ShapeType getType() const override { return ShapeType::Box; }
  [[nodiscard]] Aabb computeLocalAabb() const override;

  [[nodiscard]] const Eigen::Vector3d& getHalfExtents() const { return halfExtents_; }

private:
  Eigen::Vector3d halfExtents_;
};

} // namespace dart::collision::experimental
```

### Future Contact Types

For deformable bodies, cloth, and codimensional objects:

```cpp
// Reserved for future implementation

/// Edge contact (line contact between two objects)
struct EdgeContact {
  Eigen::Vector3d point1, point2;
  Eigen::Vector3d edgeDirection;
  Eigen::Vector3d normal;
  double depth1, depth2;
};

/// Face contact (planar polygon contact)
struct FaceContact {
  std::vector<Eigen::Vector3d> polygon;  // CCW ordered
  Eigen::Vector3d normal;
  Eigen::Vector3d centroid;
  double area;
};

/// Parametric contact (for analytic/differentiable simulation)
struct ParametricContact {
  // Curve parameter or (u,v) surface parameters
  Eigen::Vector2d param1, param2;

  // Differential geometry information
  Eigen::Matrix2d firstFundamentalForm;
  Eigen::Matrix2d secondFundamentalForm;
};
```

---

## References

- DART existing collision module: `dart/collision/`
- DART experimental simulation: `dart/simulation/experimental/`
- Test patterns: `tests/unit/collision/`
