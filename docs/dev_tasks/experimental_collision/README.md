# Experimental Built-in Collision Detection Module

> **Epic Status**: Phase 0 - Design & Planning
> **Last Updated**: 2026-01-19
> **Location**: `dart/collision/experimental/`

## North Star

DART should ultimately rely on a single, fully built-in collision detection implementation that we can iterate on indefinitely, eliminating external collision-detection backend dependencies.

## Quick Links

- [Design Document](./design.md) - API design, architecture decisions
- [Phase Plan](./phases.md) - Detailed phase breakdown with deliverables
- [Progress Tracker](./progress.md) - Current status and completed tasks

## Current Phase

**Phase 0: Design & Harness** (In Progress)

- [x] Research existing patterns (FCL, terminology, manifolds)
- [x] Design core data structures
- [ ] Create module skeleton
- [ ] Implement test harness
- [ ] Implement benchmark harness

## Key Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Namespace | `dart::collision::experimental` | Follows simulation/experimental pattern |
| File naming | snake_case | Matches experimental convention |
| Normal convention | From object2 to object1 | Matches existing DART convention |
| Contact representation | Extensible manifold-based | Supports rigid/deformable/cloth/codimensional |
| Broad-phase | Start with brute-force, extensible interface | Simple first, optimize later |
| Determinism | Bit-exact when possible | Required for reproducible simulation |

## Directory Structure

```
dart/collision/experimental/
├── CMakeLists.txt
├── export.hpp
├── fwd.hpp
├── types.hpp              # Contact, CollisionResult, CollisionOption
├── aabb.hpp/.cpp          # Axis-Aligned Bounding Box
├── collision_object.hpp/.cpp
├── collision_group.hpp/.cpp
├── narrow_phase/
│   ├── narrow_phase.hpp   # Dispatch interface
│   ├── sphere_sphere.hpp/.cpp
│   ├── box_box.hpp/.cpp
│   └── sphere_box.hpp/.cpp
├── broad_phase/
│   ├── broad_phase.hpp    # Interface
│   └── brute_force.hpp/.cpp
└── shapes/
    ├── shape.hpp          # Standalone shape representations
    ├── sphere.hpp/.cpp
    └── box.hpp/.cpp
```

## Success Criteria

### Phase 0 Complete When:
- [ ] Module skeleton builds with `pixi run build`
- [ ] Test harness runs sphere-sphere correctness tests
- [ ] Benchmark harness measures sphere-sphere performance
- [ ] CI integration works on Linux/macOS/Windows

### Phase 1 Complete When:
- [ ] All primitive narrow-phase tests pass (sphere, box)
- [ ] Brute-force broad-phase works
- [ ] Performance baselines established

### Phase 2 Complete When:
- [ ] Wired as additional backend (opt-in)
- [ ] Passes existing collision integration tests
- [ ] Documentation complete

### Phase 3 Complete When (DART 7):
- [ ] Default backend
- [ ] Performance competitive with current backends
- [ ] All common shape pairs supported

### Phase 4 Complete When (DART 8):
- [ ] External backends removed from default distribution
- [ ] Migration guide published
