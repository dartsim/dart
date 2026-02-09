# Experimental Built-in Collision Detection Module

> **Epic Status**: DART Backend Integration Complete
> **Last Updated**: 2026-02-03
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

**DART Backend Integration Complete** (Active — optimization phase)

The standalone library and DART backend integration are feature-complete. Distance queries, raycast, and expanded shape adapters are wired. Remaining work is performance optimization and benchmark validation.

### What's Built (575+ tests passing)

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

**Filtering:**

- Collision filtering (groups/masks) with bitmask and callback APIs

### What's Next (Priority)

1. ~~**P0 - Collision filtering** (groups/masks)~~ **COMPLETE**
2. ~~**P0 - Compound shapes**~~ **COMPLETE** (2026-02-03)
3. ~~**P0 - Parallel narrowphase**~~ **COMPLETE** (2026-02-03)
4. ~~**P1 - DART backend integration**~~ **COMPLETE** (2026-02-03)
5. ~~**P1 - Visual verification CI**~~ **COMPLETE** (2026-02-03)
6. ~~**P1 - Distance query wiring**~~ **COMPLETE** (2026-02-09)
7. ~~**P1 - Raycast wiring**~~ **COMPLETE** (2026-02-09)
8. ~~**P1 - Shape adapter expansion** (Cone, Ellipsoid, Heightmap, MultiSphere)~~ **COMPLETE** (2026-02-09)
9. ~~**P1 - Integration test expansion**~~ **COMPLETE** (2026-02-09)
10. **P1 - Performance parity with FCL** — benchmarking needed
11. **P1 - Persistent manifolds** — contact caching for warm-starting
12. **P2 - BVH traversal for mesh-mesh optimization**

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

# Expected: 444 tests across 27 test binaries
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
- [x] Collision filtering (groups/masks)
- [x] Compound shapes (2026-02-03)
- [x] Parallel narrowphase (2026-02-03)
- [ ] Performance parity with FCL

### DART Integration:

- [x] Wired as additional backend (opt-in) — registered as `"experimental"` (2026-02-03)
- [x] Shape adapter: `dynamics::Shape` → `experimental::Shape` (2026-02-03)
- [x] Contact conversion: `experimental::ContactPoint` → `collision::Contact` (2026-02-03)
- [x] Distance query wiring (2026-02-09)
- [x] Raycast wiring (2026-02-09)
- [x] Expanded shape adapters: Cone, Ellipsoid, Heightmap, MultiSphere (2026-02-09)
- [x] Passes existing collision integration tests — CollisionGroups passes (2026-02-09)
- [ ] Full integration test coverage (some pre-existing SEGFAULTs in non-experimental tests)
- [ ] Documentation complete
