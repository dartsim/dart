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

/// Brute-force O(n^2) broad-phase with AABB culling.
///
/// Simple but effective for small object counts (< 100).
/// Deterministic: pairs are generated in insertion order.
class BruteForceBroadPhase : public BroadPhase {
public:
  void add(CollisionObject* object) override;
  void remove(CollisionObject* object) override;
  void update(CollisionObject* object) override;
  void clear() override;
  [[nodiscard]] std::size_t size() const override;

  [[nodiscard]] std::vector<std::pair<CollisionObject*, CollisionObject*>>
  computePotentialPairs() const override;

private:
  std::vector<CollisionObject*> objects_;  // Ordered for determinism
};

} // namespace dart::collision::experimental
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
