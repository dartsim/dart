# Experimental Collision Module - Progress Tracker

> **Last Updated**: 2026-01-19
> **Current Focus**: Standalone Library Development

## Status Summary

| Phase                      | Status       | Progress |
| -------------------------- | ------------ | -------- |
| Core Types & Primitives    | **Complete** | 100%     |
| Standalone CollisionWorld  | **Complete** | 100%     |
| Additional Shapes          | **Complete** | 100%     |
| Distance Queries           | **Complete** | 100%     |
| Comparative Benchmarks     | **Complete** | 100%     |
| Raycast Support            | **Complete** | 100%     |
| ECS Refactoring            | **Complete** | 100%     |
| GJK/EPA Algorithm          | **Complete** | 100%     |
| Convex/Mesh Shapes         | **Complete** | 100%     |
| Continuous Collision (CCD) | **Complete** | 100%     |
| Visual Verification        | Not Started  | 0%       |
| DART Integration           | **Deferred** | -        |

---

## Completed Components (363 tests)

| Component                        | Files                                    | Tests | Notes                                               |
| -------------------------------- | ---------------------------------------- | ----- | --------------------------------------------------- |
| ContactPoint, ContactManifold    | types.hpp/.cpp                           | 21    | Core contact representation                         |
| CollisionResult, CollisionOption | types.hpp/.cpp                           | -     | (included above)                                    |
| Aabb                             | aabb.hpp/.cpp                            | 26    | Axis-aligned bounding box                           |
| All Shapes                       | shapes/shape.hpp/.cpp                    | 24    | Sphere, Box, Capsule, Cylinder, Plane, Convex, Mesh |
| GJK/EPA Algorithm                | narrow_phase/gjk.hpp/.cpp                | 15    | Generic convex collision detection                  |
| Sphere-sphere                    | narrow_phase/sphere_sphere.hpp/.cpp      | 17    | Narrow-phase                                        |
| Box-box (SAT)                    | narrow_phase/box_box.hpp/.cpp            | 18    | Separating Axis Theorem                             |
| Sphere-box                       | narrow_phase/sphere_box.hpp/.cpp         | 19    | Mixed pair                                          |
| Capsule collision                | narrow*phase/capsule*\*.hpp/.cpp         | 14    | All capsule pairs                                   |
| Cylinder collision               | narrow_phase/cylinder_collision.hpp/.cpp | 18    | All cylinder pairs                                  |
| Plane collision                  | narrow_phase/plane_sphere.hpp/.cpp       | 11    | All plane pairs                                     |
| Distance queries                 | narrow_phase/distance.hpp/.cpp           | 18    | 6 primitive pairs + Convex/Mesh                     |
| Raycast                          | narrow_phase/raycast.hpp/.cpp            | 39    | 7 shape types incl. Mesh/Convex                     |
| CCD (sphere/capsule cast)        | narrow_phase/ccd.hpp/.cpp                | 62    | Swept volume + conservative advancement             |
| BruteForceBroadPhase             | broad_phase/brute_force.hpp/.cpp         | 15    | O(n²) broad-phase                                   |
| CollisionObject                  | collision_object.hpp/.cpp                | 12    | Lightweight ECS handle                              |
| NarrowPhase                      | narrow_phase/narrow_phase.hpp/.cpp       | 7     | Shape-type dispatch                                 |
| CollisionWorld                   | collision_world.hpp/.cpp                 | 8     | Standalone collision detection                      |

---

## Next Steps

### Priority 1: More Shapes

| Shape         | Implementation | Tests | Notes                                                |
| ------------- | -------------- | ----- | ---------------------------------------------------- |
| CapsuleShape  | **Complete**   | 14    | Sphere-swept line segment                            |
| CylinderShape | **Complete**   | 3     | Axis-aligned cylinder (shape only, no collision yet) |
| PlaneShape    | **Complete**   | 11    | Infinite half-space                                  |
| ConvexShape   | **Complete**   | 5     | Vertex-based convex hull with support function       |
| MeshShape     | **Complete**   | 4     | Triangle mesh with support function                  |

### Priority 2: Shape Pairs ✅ COMPLETE (all primitive pairs)

| Pair              | Status       | Tests | Notes                      |
| ----------------- | ------------ | ----- | -------------------------- |
| Capsule-capsule   | **Complete** | 14    | Common in robotics         |
| Capsule-sphere    | **Complete** | -     | Included in capsule tests  |
| Capsule-box       | **Complete** | -     | Included in capsule tests  |
| Plane-sphere      | **Complete** | 11    | Ground contact             |
| Plane-box         | **Complete** | -     | Included in plane tests    |
| Plane-capsule     | **Complete** | -     | Included in plane tests    |
| Cylinder-cylinder | **Complete** | 18    | Full cylinder support      |
| Cylinder-sphere   | **Complete** | -     | Included in cylinder tests |
| Cylinder-box      | **Complete** | -     | Included in cylinder tests |
| Cylinder-capsule  | **Complete** | -     | Included in cylinder tests |
| Cylinder-plane    | **Complete** | -     | Included in cylinder tests |
| Mesh-primitive    | Pending      |       | GJK/EPA based              |

### Priority 3: Distance Queries ✅ COMPLETE (6 primitive pairs + Convex/Mesh)

| Query Type           | Status       | Notes                |
| -------------------- | ------------ | -------------------- |
| Sphere-sphere dist   | **Complete** | 18 tests total       |
| Sphere-box dist      | **Complete** | Included in distance |
| Box-box dist         | **Complete** | Included in distance |
| Capsule-capsule dist | **Complete** | Included in distance |
| Capsule-sphere dist  | **Complete** | Included in distance |
| Capsule-box dist     | **Complete** | Included in distance |
| Convex-Convex dist   | **Complete** | GJK-based, 4 tests   |
| Convex-Mesh dist     | **Complete** | GJK-based            |
| Mesh-Mesh dist       | **Complete** | GJK-based            |

### Priority 4: Raycast ✅ COMPLETE (7 shapes)

| Feature      | Status       | Tests | Notes                        |
| ------------ | ------------ | ----- | ---------------------------- |
| Ray-sphere   | **Complete** | 5     | Hit/miss, inside, max dist   |
| Ray-box      | **Complete** | 5     | All faces, rotated           |
| Ray-capsule  | **Complete** | 3     | Cylindrical + spherical caps |
| Ray-cylinder | **Complete** | 4     | Curved surface + caps        |
| Ray-plane    | **Complete** | 6     | Backface culling, offset     |
| Ray-mesh     | **Complete** | 8     | Moller-Trumbore algorithm    |
| Ray-convex   | **Complete** | 7     | GJK-based binary search      |

**CollisionWorld raycast API:**

- `raycast(ray, option, result)` - Returns closest hit
- `raycastAll(ray, option, results)` - Returns all hits sorted by distance

### Priority 5: Benchmarks ✅ COMPLETE

**Comparative benchmarks created:** `tests/benchmark/collision/bm_comparative.cpp`

| Shape Pair      | Experimental | FCL          | Bullet       | ODE          | Speedup |
| --------------- | ------------ | ------------ | ------------ | ------------ | ------- |
| Sphere-Sphere   | **41 ns**    | 774-1107 ns  | 411-586 ns   | 991-1002 ns  | **10x** |
| Box-Box         | **210 ns**   | 2429-2486 ns | 1094-1147 ns | 2210-2226 ns | **5x**  |
| Capsule-Capsule | **41 ns**    | 242 ns       | 440 ns       | 1381-1382 ns | **6x**  |
| Distance        | **7 ns**     | 286-288 ns   | N/A          | N/A          | **40x** |

**Accuracy verification:** PASSED

### Priority 6: ECS Refactoring (Next)

**Goal:** Adopt ECS pattern from `dart/simulation/experimental` for better data locality.

Current architecture uses `shared_ptr<CollisionObject>`. Target architecture:

| Current                       | Target (ECS)                               |
| ----------------------------- | ------------------------------------------ |
| `shared_ptr<CollisionObject>` | Lightweight handle (entity ID + world ptr) |
| Shape\* owned by object       | ShapeComponent in registry                 |
| Transform in object           | TransformComponent in registry             |
| AABB in object                | AabbComponent in registry                  |
| `std::vector<shared_ptr>`     | `entt::registry`                           |

**Benefits:**

- Better cache locality for iteration
- Copyable handles (no pointer ownership issues)
- Consistent with simulation/experimental pattern

### Priority 7: Continuous Collision Detection (CCD) ✅ COMPLETE

CCD prevents fast-moving objects from tunneling through thin obstacles.

| Algorithm                    | Status       | Tests | Notes                                               |
| ---------------------------- | ------------ | ----- | --------------------------------------------------- |
| Sphere-cast (all primitives) | **Complete** | 37    | Sphere, Box, Capsule, Cylinder, Plane, Convex, Mesh |
| Capsule-cast (all shapes)    | **Complete** | 16    | Sphere, Box, Capsule, Plane, Cylinder, Convex, Mesh |
| Conservative advancement     | **Complete** | 4     | General convex-convex via GJK                       |
| NarrowPhase dispatcher       | **Complete** | 3     | `sphereCast()`, `isSphereCastSupported()`           |

**Implemented API:**

```cpp
// Sphere-cast functions (sweep sphere against static target)
bool sphereCastSphere/Box/Capsule/Cylinder/Plane/Convex/Mesh(...);

// Capsule-cast functions (sweep capsule against static target)
bool capsuleCastSphere/Box/Capsule/Plane/Cylinder/Convex/Mesh(...);

// Conservative advancement for general convex pairs
bool conservativeAdvancement(shapeA, tfAStart, tfAEnd, shapeB, tfB, option, result);

// NarrowPhase dispatcher
bool NarrowPhase::sphereCast(start, end, radius, target, option, result);
bool NarrowPhase::isSphereCastSupported(ShapeType type);
```

### Priority 8: Visual Verification

Options:

- **rerun** - Python-based, good for debugging
- **raylib** - Simple C library, minimal deps

Features needed:

- Render shapes with transforms
- Visualize contact points and normals
- Show AABBs
- Step-through collision pairs

---

## Module Structure

```
dart/collision/experimental/
├── CMakeLists.txt              # Build configuration
├── export.hpp                  # DLL export macros
├── fwd.hpp                     # Forward declarations
├── types.hpp/.cpp              # ContactPoint, Ray, RaycastResult, etc.
├── aabb.hpp/.cpp               # Axis-aligned bounding box
├── collision_object.hpp/.cpp   # Shape + Transform wrapper
├── collision_world.hpp/.cpp    # Standalone collision detection
├── shapes/
│   └── shape.hpp/.cpp          # All primitive shapes
├── narrow_phase/
│   ├── narrow_phase.hpp/.cpp   # Shape-type dispatch
│   ├── sphere_sphere.hpp/.cpp  # Sphere-sphere detection
│   ├── box_box.hpp/.cpp        # Box-box SAT detection
│   ├── sphere_box.hpp/.cpp     # Sphere-box detection
│   ├── capsule_*.hpp/.cpp      # Capsule collision pairs
│   ├── cylinder_collision.hpp/.cpp  # Cylinder collision pairs
│   ├── plane_sphere.hpp/.cpp   # Plane collision pairs
│   ├── distance.hpp/.cpp       # Distance queries
│   └── raycast.hpp/.cpp        # Ray-shape intersections
└── broad_phase/
    ├── broad_phase.hpp         # BroadPhase interface
    └── brute_force.hpp/.cpp    # O(n^2) broad-phase
```

---

## Build Commands

```bash
# Configure with experimental collision
pixi run cmake -B build -DDART_BUILD_COLLISION_EXPERIMENTAL=ON -DDART_BUILD_TESTS=ON

# Build all experimental tests
cmake --build build --target dart_collision_experimental_tests

# Run all tests
for t in test_types test_aabb test_shapes test_sphere_sphere test_brute_force \
         test_box_box test_sphere_box test_collision_object test_narrow_phase \
         test_collision_world; do
  ./build/bin/$t
done
```

---

## Decisions Log

| Date       | Decision                                      | Rationale                                        |
| ---------- | --------------------------------------------- | ------------------------------------------------ |
| 2026-01-19 | Defer DART integration                        | Build standalone library to feature parity first |
| 2026-01-19 | Use `dart::collision::experimental` namespace | Matches simulation/experimental pattern          |
| 2026-01-19 | Keep DART's normal convention (obj2 -> obj1)  | Backward compatibility                           |
| 2026-01-19 | Design manifold-based contacts                | Future extensibility for deformable/cloth        |
| 2026-01-19 | Start with brute-force broad-phase            | Simple first, optimize later                     |
| 2026-01-19 | Standalone shapes (no dynamics dep)           | Enables pure collision testing                   |
| 2026-01-19 | Use PascalCase for abbreviations (Aabb)       | Codebase convention                              |
| 2026-01-19 | BoxShape stores full extents internally       | API takes half-extents for compatibility         |
| 2026-01-19 | CollisionWorld as standalone API              | Simpler than full CollisionDetector integration  |

---

## Blockers

_None currently_
