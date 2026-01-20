# Experimental Built-in Collision Detection Module

> **Epic Status**: Standalone Library Development
> **Last Updated**: 2026-01-19
> **Location**: `dart/collision/experimental/`
> **Housekeeping**: This folder includes local filesystem paths (e.g., `/home/js/...`) for active work; remove or sanitize them once all experimental_collision tasks are complete.

## North Star

DART should ultimately rely on a single, fully built-in collision detection implementation that we can iterate on indefinitely, eliminating external collision-detection backend dependencies.

## ⚠️ CRITICAL: Performance Goal

**The experimental collision module MUST be faster than existing backends (FCL, Bullet, ODE) while maintaining equal or better accuracy and feature parity.**

This is the primary success metric. All benchmarks must:

1. **Compare against existing engines** - Every benchmark should include FCL, Bullet, and ODE results for the same scenario
2. **Demonstrate performance wins** - We only ship when we're faster
3. **Verify accuracy parity** - Same or better contact quality (position, normal, depth accuracy)
4. **Match feature coverage** - Support all shape pairs that existing backends support

If we can't beat existing backends in performance, there's no point in building this module.

## Quick Links

- [Design Document](./design.md) - API design, architecture decisions
- [Phase Plan](./phases.md) - Detailed phase breakdown with deliverables
- [Progress Tracker](./progress.md) - Current status and completed tasks
- [Parallelization Plan](./parallelization_plan.md) - Multi-core batch collision roadmap
- [ECS Data Layout](./ecs_data_layout.md) - Batch-friendly data layout and interface notes
- [ReactPhysics3D ECS Review](./reactphysics3d_ecs_review.md) - ECS data layout and pipeline notes
- [Reference Comparison](./reference_comparison.md) - FCL/Bullet/ODE feature and algorithm comparison
- [Gap Analysis](./gap_analysis.md) - Implementation gaps vs reference backends (priority roadmap)
- [Conventions](./conventions.md) - Normal/sign conventions and adapter rules
- [Contact Manifolds](./contact_manifolds.md) - Contact representation notes
- [SDF + Gradients](./sdf_and_gradients.md) - SDF survey and gradient guidance
- [Benchmark Catalog](./benchmark_catalog.md) - Surveyed benchmark cases and datasets
- [Benchmark Results](./benchmark_results.md) - Latest structured suite results and gates

## Current Status

**Standalone Library Development** (Active)

The module is being developed as a **standalone collision library** first, before DART API integration. Integration is deferred until feature parity with existing backends.

### What's Built (149 tests passing)

- Core types: ContactPoint, ContactManifold, CollisionResult, CollisionOption
- Shapes: SphereShape, BoxShape
- Narrow-phase: sphere-sphere, box-box (SAT), sphere-box
- Broad-phase: BruteForceBroadPhase
- CollisionObject wrapper
- NarrowPhase dispatcher
- CollisionWorld (standalone collision detection)

### What's Next

1. **More shapes**: Capsule, Cylinder, Plane, Mesh
2. **Distance queries**: Signed distance, closest points
3. **Raycast support**
4. **Comprehensive benchmarks**
5. **Visual verification GUI** (rerun or raylib)

## Key Design Decisions

| Decision               | Choice                                       | Rationale                                     |
| ---------------------- | -------------------------------------------- | --------------------------------------------- |
| Namespace              | `dart::collision::experimental`              | Follows simulation/experimental pattern       |
| File naming            | snake_case                                   | Matches experimental convention               |
| Normal convention      | From object2 to object1                      | Matches existing DART convention              |
| Contact representation | Extensible manifold-based                    | Supports rigid/deformable/cloth/codimensional |
| Broad-phase            | Start with brute-force, extensible interface | Simple first, optimize later                  |
| Determinism            | Bit-exact when possible                      | Required for reproducible simulation          |
| Development approach   | Standalone library first                     | Defer DART integration until feature parity   |

## Directory Structure

```
dart/collision/experimental/
├── CMakeLists.txt
├── export.hpp
├── fwd.hpp
├── types.hpp/.cpp          # ContactPoint, ContactManifold, CollisionResult, CollisionOption
├── aabb.hpp/.cpp           # Axis-aligned bounding box
├── collision_object.hpp/.cpp
├── collision_world.hpp/.cpp  # Standalone collision detection
├── shapes/
│   └── shape.hpp/.cpp      # SphereShape, BoxShape
├── narrow_phase/
│   ├── narrow_phase.hpp/.cpp  # Shape-type dispatch
│   ├── sphere_sphere.hpp/.cpp
│   ├── box_box.hpp/.cpp
│   └── sphere_box.hpp/.cpp
└── broad_phase/
    ├── broad_phase.hpp     # Interface
    └── brute_force.hpp/.cpp
```

## Build & Test

```bash
# Configure with experimental collision
pixi run cmake -B build -DDART_BUILD_COLLISION_EXPERIMENTAL=ON -DDART_BUILD_TESTS=ON

# Build all experimental tests
cmake --build build --target dart_collision_experimental_tests

# Run all tests (149 tests across 10 files)
for t in test_types test_aabb test_shapes test_sphere_sphere test_brute_force \
         test_box_box test_sphere_box test_collision_object test_narrow_phase \
         test_collision_world; do
  ./build/bin/$t
done
```

## Success Criteria

### Standalone Library Complete When:

- [ ] All primitive shapes: Sphere, Box, Capsule, Cylinder, Plane
- [ ] Mesh-primitive collision
- [ ] Distance queries (signed distance, closest points)
- [ ] Raycast support
- [ ] Comprehensive benchmarks established
- [ ] Visual verification tool working

### DART Integration (Deferred):

- [ ] Wired as additional backend (opt-in)
- [ ] Passes existing collision integration tests
- [ ] Documentation complete
