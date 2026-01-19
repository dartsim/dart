# Experimental Collision Module - Phase Plan

> **Last Updated**: 2026-01-19

## Phase Overview

```
Phase 0 ─────► Phase 1 ─────► Phase 2 ─────► Phase 3 ─────► Phase 4
 Design        Build          Integrate      Default        Remove
 Harness       Module         Backend        (DART 7)       External
                                                            (DART 8)
```

---

## Phase 0: Design & Harness

**Goal**: Establish foundation - design, test harness, benchmark harness

**Duration**: ~2 weeks

### Deliverables

| # | Deliverable | Status | Notes |
|---|-------------|--------|-------|
| 0.1 | Research & design document | Done | `docs/dev_tasks/experimental_collision/design.md` |
| 0.2 | Module skeleton | Pending | `dart/collision/experimental/` |
| 0.3 | Test harness | Pending | `tests/unit/collision/experimental/` |
| 0.4 | Benchmark harness | Pending | `tests/benchmark/collision/experimental/` |
| 0.5 | CI integration | Pending | `.github/workflows/` updates |

### Files to Create

```
dart/collision/experimental/
├── CMakeLists.txt
├── export.hpp
├── fwd.hpp
└── (empty stubs for now)

tests/unit/collision/experimental/
├── CMakeLists.txt
├── helpers/
│   └── collision_test_utils.hpp
└── test_contact_point.cpp    # First test

tests/benchmark/collision/experimental/
├── CMakeLists.txt
└── bm_placeholder.cpp        # Benchmark skeleton
```

### Success Criteria

- [ ] `pixi run build` succeeds with new module
- [ ] `pixi run test-unit` runs experimental collision tests
- [ ] Benchmark harness executes (even if placeholder)
- [ ] CI passes on all platforms

---

## Phase 1: Build Module

**Goal**: Implement core collision functionality (standalone, not integrated)

**Duration**: ~4 weeks

### Deliverables

| # | Deliverable | Status | Notes |
|---|-------------|--------|-------|
| 1.1 | Core types (ContactPoint, CollisionResult) | Pending | With tests |
| 1.2 | Aabb implementation | Pending | With tests + benchmarks |
| 1.3 | Standalone shapes (Sphere, Box) | Pending | No dynamics dependency |
| 1.4 | Sphere-sphere collision | Pending | First narrow-phase |
| 1.5 | Box-box collision (SAT) | Pending | Most complex primitive |
| 1.6 | Sphere-box collision | Pending | |
| 1.7 | Brute-force broad-phase | Pending | Simple O(n^2) |
| 1.8 | CollisionObject wrapper | Pending | Wraps shape + transform |
| 1.9 | Performance baselines | Pending | Document baseline numbers |

### Implementation Order

```
1.1 ContactPoint ──► 1.2 Aabb ──► 1.3 Shapes
         │                              │
         ▼                              ▼
    1.4 sphere-sphere ◄──────────────────┘
         │
         ▼
    1.5 box-box ──► 1.6 sphere-box
         │
         ▼
    1.7 BruteForceBroadPhase
         │
         ▼
    1.8 CollisionObject ──► 1.9 Baselines
```

### Files to Create

```
dart/collision/experimental/
├── types.hpp / types.cpp
├── aabb.hpp / aabb.cpp
├── collision_object.hpp / collision_object.cpp
├── shapes/
│   ├── shape.hpp
│   ├── sphere.hpp / sphere.cpp
│   └── box.hpp / box.cpp
├── narrow_phase/
│   ├── narrow_phase.hpp
│   ├── sphere_sphere.hpp / sphere_sphere.cpp
│   ├── box_box.hpp / box_box.cpp
│   └── sphere_box.hpp / sphere_box.cpp
└── broad_phase/
    ├── broad_phase.hpp
    └── brute_force.hpp / brute_force.cpp

tests/unit/collision/experimental/
├── test_aabb.cpp
├── test_sphere_sphere.cpp
├── test_box_box.cpp
├── test_sphere_box.cpp
└── test_broad_phase.cpp

tests/benchmark/collision/experimental/
├── bm_sphere_sphere.cpp
├── bm_box_box.cpp
└── bm_broad_phase.cpp
```

### Success Criteria

- [ ] All narrow-phase tests pass
- [ ] Determinism tests pass (100 runs, identical output)
- [ ] Benchmarks establish baselines
- [ ] Edge cases handled (concentric spheres, touching, etc.)

---

## Phase 2: Integrate as Backend

**Goal**: Wire as additional collision backend (opt-in)

**Duration**: ~3 weeks

### Deliverables

| # | Deliverable | Status | Notes |
|---|-------------|--------|-------|
| 2.1 | ExperimentalCollisionDetector | Pending | Implements CollisionDetector |
| 2.2 | ExperimentalCollisionGroup | Pending | Implements CollisionGroup |
| 2.3 | ExperimentalCollisionObject | Pending | Implements CollisionObject |
| 2.4 | Shape adapter layer | Pending | dart::dynamics::Shape → experimental::Shape |
| 2.5 | Integration tests | Pending | Run against existing tests |
| 2.6 | Documentation | Pending | Feature matrix, limitations |

### Files to Create

```
dart/collision/experimental/
├── experimental_collision_detector.hpp / .cpp
├── experimental_collision_group.hpp / .cpp
├── experimental_collision_object.hpp / .cpp
└── adapters/
    └── dynamics_shape_adapter.hpp / .cpp

tests/integration/collision/
└── test_ExperimentalBackend.cpp
```

### Success Criteria

- [ ] Can be selected via API: `ExperimentalCollisionDetector::create()`
- [ ] Passes core integration tests
- [ ] Documentation clearly states supported shapes and limitations
- [ ] No regressions in existing backends

---

## Phase 3: Default Backend (DART 7)

**Goal**: Promote to default collision backend

**Duration**: ~4 weeks

### Deliverables

| # | Deliverable | Status | Notes |
|---|-------------|--------|-------|
| 3.1 | Additional shape support | Pending | Capsule, Cylinder as needed |
| 3.2 | Mesh support | Pending | At least mesh-primitive |
| 3.3 | Performance optimization | Pending | Meet or exceed current backends |
| 3.4 | Default selection logic | Pending | Make experimental the default |
| 3.5 | Migration guide | Pending | For users |
| 3.6 | Deprecation warnings | Pending | For old backends |

### Quality Gates

| Gate | Requirement |
|------|-------------|
| Correctness | 100% of existing collision tests pass |
| Performance | Within 20% of best current backend |
| Coverage | Supports shapes used by DART examples |
| Stability | Zero crash bugs in fuzz testing |

### Success Criteria

- [ ] Default for new users
- [ ] All examples work without modification
- [ ] Performance benchmarks published
- [ ] Migration guide available

---

## Phase 4: Remove External Backends (DART 8)

**Goal**: Remove external collision backends from default distribution

**Duration**: ~2 weeks

### Deliverables

| # | Deliverable | Status | Notes |
|---|-------------|--------|-------|
| 4.1 | Backend removal | Pending | FCL, Bullet, ODE backends |
| 4.2 | Build simplification | Pending | Remove optional dependencies |
| 4.3 | Documentation update | Pending | Remove references to old backends |
| 4.4 | Final migration notes | Pending | For users still on old backends |

### Considerations

- May keep compatibility layer for specific use cases
- External backends could move to separate repos if community needs them

### Success Criteria

- [ ] Clean build with no external collision dependencies
- [ ] All functionality preserved
- [ ] Smaller distribution size
- [ ] Simplified installation

---

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Performance regression | Extensive benchmarking, optimization phase |
| Missing shape pairs | Prioritize based on DART example usage |
| Numerical edge cases | Comprehensive edge case testing |
| User disruption | Long deprecation period, clear migration guides |
| Platform differences | CI on all platforms, determinism tests |

---

## Timeline Summary

| Phase | Duration | Target |
|-------|----------|--------|
| Phase 0 | 2 weeks | Immediate |
| Phase 1 | 4 weeks | +6 weeks |
| Phase 2 | 3 weeks | +9 weeks |
| Phase 3 | 4 weeks | DART 7.0 |
| Phase 4 | 2 weeks | DART 8.0 |
