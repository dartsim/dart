# Experimental Collision Module - Progress Tracker

> **Last Updated**: 2026-01-19
> **Current Phase**: Phase 2 - IN PROGRESS

## Status Summary

| Phase | Status | Progress |
|-------|--------|----------|
| Phase 0: Design & Harness | **Complete** | 100% |
| Phase 1: Build Module | **Complete** | 100% |
| Phase 2: Integrate Backend | **In Progress** | 50% |
| Phase 3: Default (DART 7) | Not Started | 0% |
| Phase 4: Remove External (DART 8) | Not Started | 0% |

---

## Phase 0: Design & Harness (COMPLETE)

- [x] **0.1.1** Research existing codebase patterns
- [x] **0.1.2** Research collision detection best practices
- [x] **0.1.3** Design core data structures
- [x] **0.1.4** Create epic documentation
- [x] **0.2** Create module skeleton
- [x] **0.3** Create test harness
- [x] **0.4** Create benchmark harness (Aabb benchmark)

---

## Phase 1: Build Module (COMPLETE)

134 tests passing across 8 test files.

| Task | Description | Status | Tests |
|------|-------------|--------|-------|
| 1.1 | ContactPoint + CollisionResult | Complete | 21 |
| 1.2 | Aabb implementation | Complete | 26 |
| 1.3 | Standalone Shape classes (Sphere, Box) | Complete | 8 |
| 1.4 | Sphere-sphere narrow-phase | Complete | 17 |
| 1.5 | Box-box narrow-phase (SAT) | Complete | 18 |
| 1.6 | Sphere-box narrow-phase | Complete | 19 |
| 1.7 | BruteForceBroadPhase | Complete | 15 |
| 1.8 | CollisionObject wrapper | Complete | 10 |

---

## Phase 2: Integrate Backend (IN PROGRESS)

149 tests passing across 10 test files.

| Task | Description | Status | Tests |
|------|-------------|--------|-------|
| 2.1 | NarrowPhase dispatcher | Complete | 7 |
| 2.2 | CollisionWorld class | Complete | 8 |
| 2.3 | Integration with DART API | Pending | - |

### New Components

```
dart/collision/experimental/
├── narrow_phase/
│   └── narrow_phase.hpp/.cpp    # Shape-type dispatch to narrow-phase algorithms
└── collision_world.hpp/.cpp     # Standalone collision detection world

tests/unit/collision/experimental/
├── test_narrow_phase.cpp        # 7 tests
└── test_collision_world.cpp     # 8 tests
```

### CollisionWorld Features
- Object add/remove/update with automatic AABB tracking
- Broad-phase filtering via BruteForceBroadPhase
- Narrow-phase dispatch via NarrowPhase::collide()
- Direct pair collision testing
- Mixed shape support (sphere-sphere, box-box, sphere-box)

---

## Module Structure (Current)

```
dart/collision/experimental/
├── CMakeLists.txt              # Build configuration
├── export.hpp                  # DLL export macros
├── fwd.hpp                     # Forward declarations
├── types.hpp/.cpp              # ContactPoint, ContactManifold, CollisionResult, CollisionOption
├── aabb.hpp/.cpp               # Axis-aligned bounding box
├── collision_object.hpp/.cpp   # Shape + Transform wrapper
├── collision_world.hpp/.cpp    # Standalone collision detection
├── shapes/
│   └── shape.hpp/.cpp          # SphereShape, BoxShape
├── narrow_phase/
│   ├── narrow_phase.hpp/.cpp   # Shape-type dispatch
│   ├── sphere_sphere.hpp/.cpp  # Sphere-sphere detection
│   ├── box_box.hpp/.cpp        # Box-box SAT detection
│   └── sphere_box.hpp/.cpp     # Sphere-box detection
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

| Date | Decision | Rationale |
|------|----------|-----------|
| 2026-01-19 | Use `dart::collision::experimental` namespace | Matches simulation/experimental pattern |
| 2026-01-19 | Keep DART's normal convention (obj2 -> obj1) | Backward compatibility |
| 2026-01-19 | Design manifold-based contacts | Future extensibility for deformable/cloth |
| 2026-01-19 | Start with brute-force broad-phase | Simple first, optimize later |
| 2026-01-19 | Standalone shapes (no dynamics dep) | Enables pure collision testing |
| 2026-01-19 | Use PascalCase for abbreviations (Aabb) | Codebase convention |
| 2026-01-19 | BoxShape stores full extents internally | API takes half-extents for compatibility |
| 2026-01-19 | Public min/max fields in Aabb | Direct access, consistent with other tests |
| 2026-01-19 | CollisionWorld as standalone API | Simpler than full CollisionDetector integration |

---

## Phase 2.3: Integration with DART API (NEXT)

### Goals
- Wire experimental module as opt-in CollisionDetector backend
- Support dynamics::ShapeFrame integration
- Pass existing collision integration tests

### Pending Tasks
- [ ] Create ExperimentalCollisionDetector class
- [ ] Implement CollisionGroup adapter
- [ ] Shape type mapping from dynamics::Shape
- [ ] Add integration tests

---

## Blockers

*None currently*
