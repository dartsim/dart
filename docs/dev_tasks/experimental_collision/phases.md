# Experimental Collision Module - Phase Plan

> **Last Updated**: 2026-01-19

## Strategy Change: Standalone Library First

**Previous plan**: Build module → Integrate as backend → Make default → Remove external

**Revised plan**: Build standalone library to feature parity → Then integrate with DART

**Rationale**: Integration couples the module to DART's dynamics system. By developing as a standalone library first:

1. Faster iteration (no dynamics dependencies to manage)
2. Easier testing and benchmarking
3. Can achieve feature parity before committing to integration API
4. Visual verification tools work independently

---

## Phase Overview (Revised)

```
Standalone Library Development (Current)
├── Core collision primitives ✓
├── More shapes (Capsule, Cylinder, Plane, Mesh)
├── Distance queries
├── Raycast support
├── Comprehensive benchmarks
└── Visual verification GUI

Integration Phase (Deferred)
├── Wire as CollisionDetector backend
├── Shape adapters from dart::dynamics
└── Pass existing integration tests

Default Backend (DART 7)
└── Promote to default, deprecate external

Remove External (DART 8)
└── Remove FCL, Bullet, ODE backends
```

---

## Standalone Library Development

### Completed (149 tests)

| Component                                      | Tests | Notes                          |
| ---------------------------------------------- | ----- | ------------------------------ |
| ContactPoint, ContactManifold, CollisionResult | 21    | Core types                     |
| Aabb                                           | 26    | Axis-aligned bounding box      |
| SphereShape, BoxShape                          | 8     | Standalone shapes              |
| Sphere-sphere collision                        | 17    | First narrow-phase             |
| Box-box collision (SAT)                        | 18    | Separating Axis Theorem        |
| Sphere-box collision                           | 19    | Mixed shape pair               |
| BruteForceBroadPhase                           | 15    | O(n^2) broad-phase             |
| CollisionObject                                | 10    | Shape + transform wrapper      |
| NarrowPhase dispatcher                         | 7     | Shape-type dispatch            |
| CollisionWorld                                 | 8     | Standalone collision detection |

### In Progress

| Component                | Status      | Priority |
| ------------------------ | ----------- | -------- |
| CapsuleShape             | Not started | High     |
| CylinderShape            | Not started | High     |
| PlaneShape               | Not started | High     |
| Capsule collision pairs  | Not started | High     |
| Cylinder collision pairs | Not started | High     |
| Plane collision pairs    | Not started | High     |
| Distance queries         | Not started | High     |
| Raycast support          | Not started | Medium   |
| MeshShape                | Not started | Medium   |
| Mesh-primitive collision | Not started | Medium   |
| Benchmark suite          | Not started | Medium   |
| Visual verification GUI  | Not started | Medium   |

### Shape Pair Matrix (Target)

|          | Sphere | Box | Capsule | Cylinder | Plane | Mesh |
| -------- | ------ | --- | ------- | -------- | ----- | ---- |
| Sphere   | ✓      | ✓   |         |          |       |      |
| Box      |        | ✓   |         |          |       |      |
| Capsule  |        |     |         |          |       |      |
| Cylinder |        |     |         |          |       |      |
| Plane    |        |     |         |          |       |      |
| Mesh     |        |     |         |          |       |      |

### Success Criteria

- [ ] All primitive shape pairs implemented
- [ ] Distance queries for all shape pairs
- [ ] Raycast against all shapes
- [ ] Benchmark suite with baseline numbers
- [ ] Visual verification tool functional

---

## Integration Phase (Deferred)

**Goal**: Wire as additional collision backend (opt-in)

### Deliverables

| #   | Deliverable                   | Status  | Notes                                       |
| --- | ----------------------------- | ------- | ------------------------------------------- |
| I.1 | ExperimentalCollisionDetector | Pending | Implements CollisionDetector                |
| I.2 | ExperimentalCollisionGroup    | Pending | Implements CollisionGroup                   |
| I.3 | Shape adapter layer           | Pending | dart::dynamics::Shape → experimental::Shape |
| I.4 | Integration tests             | Pending | Run against existing tests                  |
| I.5 | Documentation                 | Pending | Feature matrix, limitations                 |

### Success Criteria

- [ ] Can be selected via API: `ExperimentalCollisionDetector::create()`
- [ ] Passes core integration tests
- [ ] Documentation clearly states supported shapes and limitations
- [ ] No regressions in existing backends

---

## Default Backend Phase (DART 7)

**Goal**: Promote to default collision backend

### Quality Gates

| Gate        | Requirement                           |
| ----------- | ------------------------------------- |
| Correctness | 100% of existing collision tests pass |
| Performance | Within 20% of best current backend    |
| Coverage    | Supports shapes used by DART examples |
| Stability   | Zero crash bugs in fuzz testing       |

---

## Remove External Phase (DART 8)

**Goal**: Remove external collision backends from default distribution

### Considerations

- May keep compatibility layer for specific use cases
- External backends could move to separate repos if community needs them

---

## Risk Mitigation

| Risk                   | Mitigation                                 |
| ---------------------- | ------------------------------------------ |
| Performance regression | Extensive benchmarking, optimization phase |
| Missing shape pairs    | Prioritize based on DART example usage     |
| Numerical edge cases   | Comprehensive edge case testing            |
| Platform differences   | CI on all platforms, determinism tests     |
