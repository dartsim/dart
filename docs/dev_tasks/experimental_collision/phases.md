# Experimental Collision Module - Phase Plan

> **Last Updated**: 2026-01-19

## ⚠️ CRITICAL: Performance Requirement

**This module MUST outperform existing collision backends (FCL, Bullet, ODE).**

All benchmarks must include comparative results against:

- **FCL** (current default) - `fcl::collide()`, `fcl::distance()`
- **Bullet** - `btCollisionWorld`, `btGjkPairDetector`
- **ODE** - `dCollide()`, `dSpaceCollide()`

**If we cannot demonstrate performance wins with equal accuracy, this module should not ship.**

See [design.md](./design.md) for detailed performance requirements.

---

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

| Component                    | Status      | Priority |
| ---------------------------- | ----------- | -------- |
| CapsuleShape                 | Not started | High     |
| CylinderShape                | Not started | High     |
| PlaneShape                   | Not started | High     |
| Capsule collision pairs      | Not started | High     |
| Cylinder collision pairs     | Not started | High     |
| Plane collision pairs        | Not started | High     |
| Distance queries             | Not started | High     |
| Raycast support              | Not started | Medium   |
| MeshShape                    | Not started | Medium   |
| Mesh-primitive collision     | Not started | Medium   |
| Benchmark suite (structured) | In progress | Medium   |
| Visual verification GUI      | Not started | Medium   |

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

- [x] All primitive shape pairs implemented (25 combinations complete)
- [x] Distance queries for core shape pairs (6 pairs)
- [ ] Raycast against all shapes
- [x] Baseline comparative benchmarks for a subset (`bm_comparative.cpp`)
- [ ] Structured benchmark suite (comparative + scenarios + fixtures)
- [ ] Visual verification tool functional

### Benchmark Results (Structured Suite)

Benchmark outputs and summaries (including legacy baseline numbers) live in
`docs/dev_tasks/experimental_collision/benchmark_results.md`.

Structured suite layout and coverage are tracked in
`docs/dev_tasks/experimental_collision/benchmark_catalog.md`.

Key benchmark locations:

- Comparative benchmarks: `tests/benchmark/collision/comparative/`
- Scenario benchmarks: `tests/benchmark/collision/scenarios/`
- Experimental microbenchmarks: `tests/benchmark/collision/experimental/`
  (CCD microbench in `bm_ccd.cpp`)

### Batch Benchmark Requirements (Comparative)

**Definition:** Batch = many queries per frame over a scene (broadphase + narrowphase), not just single-pair microbenchmarks.

**Scenarios (minimum set):**

- Mixed primitives scene (sphere/box/capsule) with 1k and 10k objects
- Dense cluster vs sparse distribution (broadphase stress)
- Mesh-heavy scene (static meshes + moving primitives)
- Batched raycasts (10k rays) with hit/miss mix
- Edge-case matrix for narrow-phase/distance/raycast (grazing, deep penetration, thin features, near-parallel faces)
- Scale sweeps (tiny/nominal/large, e.g. 1e-3, 1, 1e3)

**Metrics to record:**

- Pairs/sec and queries/sec
- Time per frame (update + broadphase + narrowphase)
- Contact count and accuracy deltas vs baseline backends
- Peak memory and allocation counts in the hot path
- Cross-backend consistency summary (hit/no-hit, distance, normal, depth)

**Fairness rules:**

- Pre-create objects and meshes outside timed loops
- Fixed RNG seeds and identical transforms across backends
- Match options (contact flags, tolerances) per backend
- Warm-up iterations before timing

**Acceptance thresholds:**

- Experimental throughput must be **>= best backend** for each supported scenario
- If within 10% of best, document the gap and add an optimization task before integration
- If worse, block integration until the batch gap is closed

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
| Backend disagreements  | Cross-backend unit tests + analytic checks |
