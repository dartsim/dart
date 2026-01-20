# Experimental Built-in Collision Detection Module

> **Epic Status**: Standalone Library Development
> **Last Updated**: 2026-01-20
> **Location**: `dart/collision/experimental/`

## North Star

DART should ultimately rely on a single, fully built-in collision detection implementation that we can iterate on indefinitely, eliminating external collision-detection backend dependencies.

## CRITICAL: Performance Goal

**The experimental collision module MUST be faster than existing backends (FCL, Bullet, ODE) while maintaining equal or better accuracy and feature parity.**

This is the primary success metric. All benchmarks must:

1. **Compare against existing engines** - Every benchmark should include FCL, Bullet, and ODE results for the same scenario
2. **Demonstrate performance wins** - We only ship when we're faster
3. **Verify accuracy parity** - Same or better contact quality (position, normal, depth accuracy)
4. **Match feature coverage** - Support all shape pairs that existing backends support

If we can't beat existing backends in performance, there's no point in building this module.

## Quick Links

### Core Documentation

- [Design Document](./design.md) - API design, architecture decisions
- [Progress Tracker](./progress.md) - Current status and completed tasks
- [Gap Analysis](./gap_analysis.md) - Implementation gaps vs reference backends (priority roadmap)

### Architecture

- [ECS Architecture](./ecs_architecture.md) - Batch-friendly data layout, SoA storage, parallel pipeline
- [Parallelization Plan](./parallelization_plan.md) - Multi-core batch collision roadmap
- [Conventions](./conventions.md) - Normal/sign conventions and adapter rules
- [Contact Manifolds](./contact_manifolds.md) - Contact representation notes

### Reference & Benchmarks

- [Reference Comparison](./reference_comparison.md) - FCL/Bullet/ODE feature and algorithm comparison
- [SDF + Gradients](./sdf_and_gradients.md) - SDF survey and gradient guidance
- [Benchmark Catalog](./benchmark_catalog.md) - Surveyed benchmark cases and datasets
- [Benchmark Results](./benchmark_results.md) - Latest structured suite results and gates

### Archive

- [archive/](./archive/) - Historical research notes (RP3D ECS investigation)

## Current Status

**Standalone Library Development** (Active)

The module is being developed as a **standalone collision library** first, before DART API integration. Integration is deferred until feature parity with existing backends.

### What's Built (413 tests passing)

**Shapes:**

- SphereShape, BoxShape, CapsuleShape, CylinderShape, PlaneShape, ConvexShape

**Narrow-phase algorithms:**

- Sphere-sphere, sphere-box, sphere-capsule, sphere-cylinder, sphere-plane
- Box-box (SAT), box-capsule, box-cylinder, box-plane
- Capsule-capsule, capsule-cylinder, capsule-plane
- Cylinder-cylinder, cylinder-plane
- Plane-plane
- Convex-convex (GJK/EPA)

**Broad-phase:**

- BruteForceBroadPhase, SweepAndPrune, SpatialHash, AABBTree

**Query types:**

- Collision detection with contact manifolds
- Distance queries (signed distance, closest points)
- Raycast (all shapes)
- Continuous collision detection (CCD)

**Infrastructure:**

- CollisionWorld with ECS (entt) backend
- Batch API with SoA storage and BatchView
- Stable object IDs for determinism

### What's Next (Priority)

1. **P0 - Collision filtering** (groups/masks)
2. **P0 - Compound shapes**
3. **P0 - Parallel narrowphase**
4. **P1 - DART backend integration**

See [gap_analysis.md](./gap_analysis.md) for full priority roadmap.

## Key Design Decisions

| Decision               | Choice                                   | Rationale                                     |
| ---------------------- | ---------------------------------------- | --------------------------------------------- |
| Namespace              | `dart::collision::experimental`          | Follows simulation/experimental pattern       |
| File naming            | snake_case                               | Matches experimental convention               |
| Normal convention      | From object2 to object1                  | Matches existing DART convention              |
| Contact representation | Extensible manifold-based                | Supports rigid/deformable/cloth/codimensional |
| Broad-phase            | Extensible interface (4 implementations) | Different algorithms for different use cases  |
| Determinism            | Bit-exact when possible                  | Required for reproducible simulation          |
| Development approach   | Standalone library first                 | Defer DART integration until feature parity   |
| ECS backend            | entt + SoA BatchStorage                  | Cache-friendly, scales to 10k+ objects        |

## Build & Test

```bash
# Build and run all collision-experimental tests
pixi run build && ctest --test-dir build/default/cpp/Release -L collision-experimental -j$(nproc)

# Expected: 413 tests across 26 test binaries
```

## Success Criteria

### Standalone Library Complete When:

- [x] All primitive shapes: Sphere, Box, Capsule, Cylinder, Plane
- [x] Convex shape (GJK/EPA)
- [x] Distance queries (signed distance, closest points)
- [x] Raycast support
- [x] Continuous collision detection
- [x] Multiple broadphase algorithms
- [x] ECS batch infrastructure
- [ ] Collision filtering (groups/masks)
- [ ] Compound shapes
- [ ] Parallel narrowphase
- [ ] Performance parity with FCL

### DART Integration (Deferred):

- [ ] Wired as additional backend (opt-in)
- [ ] Passes existing collision integration tests
- [ ] Documentation complete
