# Experimental Collision Module - Progress Tracker

> **Last Updated**: 2026-01-19
> **Current Phase**: Phase 1 - COMPLETE

## Status Summary

| Phase | Status | Progress |
|-------|--------|----------|
| Phase 0: Design & Harness | **Complete** | 100% |
| Phase 1: Build Module | **Complete** | 100% |
| Phase 2: Integrate Backend | Not Started | 0% |
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

All 134 tests passing across 8 test files.

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

### Module Structure (Final)

```
dart/collision/experimental/
├── CMakeLists.txt              # Build configuration
├── export.hpp                  # DLL export macros
├── fwd.hpp                     # Forward declarations
├── types.hpp/.cpp              # ContactPoint, ContactManifold, CollisionResult, CollisionOption
├── aabb.hpp/.cpp               # Axis-aligned bounding box
├── collision_object.hpp/.cpp   # Shape + Transform wrapper
├── shapes/
│   └── shape.hpp/.cpp          # SphereShape, BoxShape
├── narrow_phase/
│   ├── sphere_sphere.hpp/.cpp  # Sphere-sphere detection
│   ├── box_box.hpp/.cpp        # Box-box SAT detection
│   └── sphere_box.hpp/.cpp     # Sphere-box detection
└── broad_phase/
    ├── broad_phase.hpp         # BroadPhase interface
    └── brute_force.hpp/.cpp    # O(n^2) broad-phase

tests/unit/collision/experimental/
├── test_types.cpp              # 21 tests
├── test_aabb.cpp               # 26 tests
├── test_shapes.cpp             # 8 tests
├── test_sphere_sphere.cpp      # 17 tests
├── test_box_box.cpp            # 18 tests
├── test_sphere_box.cpp         # 19 tests
├── test_brute_force.cpp        # 15 tests
└── test_collision_object.cpp   # 10 tests
```

---

## Build Commands

```bash
# Configure with experimental collision
pixi run cmake -B build -DDART_BUILD_COLLISION_EXPERIMENTAL=ON -DDART_BUILD_TESTS=ON

# Build all experimental tests
cmake --build build --target dart_collision_experimental_tests

# Run all tests
for t in test_types test_aabb test_shapes test_sphere_sphere test_brute_force test_box_box test_sphere_box test_collision_object; do
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

---

## Phase 2: Integrate Backend (NEXT)

### Goals
- Wire experimental module as opt-in backend
- Implement CollisionDetector interface
- Pass existing collision integration tests

### Pending Tasks
- [ ] Create ExperimentalCollisionDetector class
- [ ] Implement CollisionGroup integration
- [ ] Wire to existing collision detection API
- [ ] Add integration tests

---

## Blockers

*None currently*
