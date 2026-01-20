# Simulation Experimental API Epic

## Executive Summary

The **experimental simulation API** (`dart::simulation::experimental`) is a next-generation, ECS-based rewrite of DART's simulation core using the [EnTT](https://github.com/skypjack/entt) library. It aims to replace the inheritance-heavy classic API with a data-oriented, composable architecture that enables:

- **Cache-friendly data layouts** for improved performance
- **Composition over inheritance** for flexibility
- **Runtime extensibility** via custom ECS components
- **Parallel execution potential** for multi-core systems
- **Automatic serialization** of simulation state

**Current State**: Phases 0-2 and 5 complete; Phase 3-4 (testing/perf) still partial. `World::step()` runs forward dynamics and resolves collisions/constraints via a classic-API adapter.

**End State Vision**: `dart::simulation::experimental::World` will become `dart::simulation::World` in DART 8, with full Python bindings and feature parity with the classic API.

---

## Current State Inventory

### Key Modules & Files

| Directory       | Purpose                                | Completeness          |
| --------------- | -------------------------------------- | --------------------- |
| `world.hpp/cpp` | Main World container with ECS registry | Core done             |
| `frame/`        | Frame, FreeFrame, FixedFrame handles   | Complete              |
| `multi_body/`   | MultiBody, Link, Joint handles         | Partial (no dynamics) |
| `comps/`        | ECS components (data structures)       | Partial (data only)   |
| `space/`        | StateSpace + mappers for optimization  | Core done             |
| `io/`           | Binary serialization                   | Complete              |
| `common/`       | Logging, profiling, exceptions         | Complete              |
| `ecs/`          | EntityObject template infrastructure   | Complete              |
| `body/`         | RigidBody handle with physics props    | Complete              |

### Public Headers & Entrypoints

| Header                      | Purpose                    | Status            |
| --------------------------- | -------------------------- | ----------------- |
| `world.hpp`                 | Main user entry point      | Stable            |
| `multi_body/multi_body.hpp` | Robot/skeleton creation    | Stable            |
| `multi_body/link.hpp`       | Link creation + options    | Stable            |
| `multi_body/joint.hpp`      | Joint handle class         | Full state access |
| `frame/frame.hpp`           | Frame base class           | Stable            |
| `frame/free_frame.hpp`      | Free transform frames      | Stable            |
| `space/state_space.hpp`     | Optimization state mapping | Stable            |

### Joint Type Implementation Status

| Joint Type | DOF | Data Model | Kinematics | Tests |
| ---------- | --- | ---------- | ---------- | ----- |
| Revolute   | 1   | ✅         | ✅         | ✅    |
| Prismatic  | 1   | ✅         | ✅         | ✅    |
| Fixed      | 0   | ✅         | N/A        | ✅    |
| Screw      | 1   | ✅         | ✅         | ✅    |
| Universal  | 2   | ✅         | ✅         | ✅    |
| Ball       | 3   | ✅         | ✅         | ✅    |
| Planar     | 3   | ✅         | ✅         | ✅    |
| Free       | 6   | ✅         | ✅         | ✅    |

**Note**: All joint types are fully supported at the data model level and forward kinematics is implemented for all joint transforms (Phase 5.1).

### Python Bindings Status

**Current**: Phase 2 complete

- `dartpy.simulation_experimental` module exists and is functional
- Bound classes: World, MultiBody, Link, Joint, Frame, FreeFrame, FixedFrame, RigidBody, ShapeNode, ShapeNodeOptions, StateSpace, StateSpaceVariable
- Bound enums: JointType
- Python tests: 16 tests in `python/tests/unit/simulation_experimental/`

### Tests & Examples

| Test File                        | Status   | Coverage                                          |
| -------------------------------- | -------- | ------------------------------------------------- |
| `test_world.cpp`                 | Good     | Mode control, basic creation                      |
| `test_multi_body.cpp`            | Good     | Link/joint creation, trees                        |
| `test_serialization.cpp`         | Good     | Save/load worlds                                  |
| `test_frames.cpp`                | Basic    | Frame operations                                  |
| `test_state_space.cpp`           | Basic    | StateSpace API                                    |
| `test_joint.cpp`                 | **Good** | 41 tests: types, DOF, state accessors, limits     |
| `test_link.cpp`                  | **Good** | 14 tests: name, parent joint, frame, copy, chains |
| `test_rigid_body.cpp`            | **Good** | 14 tests: mass, inertia, pose, velocity, forces   |
| `test_shape_node.cpp`            | Basic    | Shape node creation, transforms, properties       |
| `test_collision_integration.cpp` | Basic    | Contact constraints with shape nodes              |
| `bm_ecs_safety.cpp`              | Basic    | ECS access benchmark                              |

**Python Tests**: `python/tests/unit/simulation_experimental/test_experimental_world.py` (18 tests)

**Examples**: `examples/simulation_experimental_hello_world/` (C++ example)

---

## History & Rationale

### Key PRs/Commits

| PR/Commit | Date         | Description                                                   |
| --------- | ------------ | ------------------------------------------------------------- |
| `#2386`   | ~2 weeks ago | Introduce simulation-experimental module, remove dart8 naming |
| `#2393`   | ~2 weeks ago | Phase 15: span inputs for simulation-experimental             |
| `#2401`   | ~2 weeks ago | Phase 21: string_view inputs                                  |
| `#2410`   | ~10 days ago | Fix CTest library paths                                       |
| `#2438`   | ~1 day ago   | Simplify copyright headers                                    |

### Design Principles (from source code comments)

1. **Data-Oriented Design**: Components store data, systems process it. No virtual dispatch.
2. **No Polymorphism**: Single `JointComponent` handles all types via `JointType` enum.
3. **Lightweight Handles**: `MultiBody`, `Link`, `Joint` are just entity ID + World pointer. O(1) copy.
4. **Compile-time Validation**: `EntityObjectWith` template enforces component access at compile time.
5. **Explicit State**: Design mode vs Simulation mode separation.
6. **Configurable State Views**: Multiple `StateSpace` instances for same World.

### Why ECS over Classic API?

| Classic API                  | Experimental API               |
| ---------------------------- | ------------------------------ |
| Deep inheritance hierarchies | Flat component composition     |
| Virtual dispatch overhead    | Enum-based type switching      |
| Shared pointers everywhere   | Lightweight entity handles     |
| Hard to extend               | Custom components via registry |
| Single-threaded              | ECS enables parallel systems   |

---

## Risks & Open Questions

### API Stability Risks

- **EnTT exposure**: Public headers expose `entt::entity` and `entt::registry`. ABI stability concern.
- **Handle invalidation**: Handles become dangling if entity destroyed. No lifetime tracking.
- **Missing feature parity**: Major features missing vs classic API.

### Performance Concerns

- **Cache behavior**: Not yet benchmarked for large worlds.
- **ECS overhead**: `safeGet<>()` vs raw registry access (benchmark exists but no optimization).

### Missing Infrastructure

- **No model loading**: Cannot load URDF/SDF/MJCF into experimental World.
- **No migration tooling**: Classic-to-experimental conversion helpers are still needed.

### Open Questions

1. Python module naming: `dartpy.simulation_experimental` vs `dartpy.next`?
2. Classic to experimental interop: How to migrate existing code?
3. Deprecation timeline for classic World?
4. Performance targets for ECS approach?

---

## Multi-Phase Plan

### Phase 0: Ground-Truth + Inventory (Quick Wins)

**Duration**: 1-2 weeks

**Scope**:

- Create this epic tracking document
- Fill empty test files (`test_joint.cpp`, `test_link.cpp`)
- Add missing joint types (Fixed, Ball at minimum)
- Add Doxygen comments to all public headers
- Create simple "hello world" example

**Tasks**:

- [ ] Implement `test_joint.cpp` with basic joint property tests
- [ ] Implement `test_link.cpp` with link property tests
- [ ] Implement `JointType::Fixed` (0-DOF, simplest)
- [ ] Implement `JointType::Ball` (3-DOF, quaternion-based)
- [ ] Add Doxygen `@brief` to all public class/method declarations
- [ ] Create `examples/simulation_experimental/hello_world/main.cpp`
- [ ] Update CMakeLists.txt to build example

**Acceptance Criteria**:

- `pixi run test-unit` passes with new tests
- All public headers have Doxygen comments
- Example compiles and runs
- No empty test files

**Done means**: Basic API is documented and tested; new contributors can understand the module.

---

### Phase 1: API Cleanup & Coherence

**Duration**: 2-3 weeks

**Scope**:

- Naming consistency audit
- Error handling standardization
- Ownership/lifetime clarity
- Design mode enforcement completeness

**Tasks**:

- [ ] Audit all naming for snake_case consistency (per code-style.md)
- [ ] Review exception hierarchy (`InvalidArgumentException`, `InvalidOperationException`, etc.)
- [ ] Document lifetime rules for handles in Doxygen
- [ ] Add `ensureDesignMode()` guards to all mutating methods
- [ ] Implement remaining joint types (Screw, Universal, Planar, Free)
- [ ] Add `RigidBody` proper implementation (not placeholder)
- [ ] Add `clear()` method tests
- [ ] Add boundary/edge case tests for all handle classes

**Acceptance Criteria**:

- All joint types implemented and tested
- No naming inconsistencies
- All mutating methods check design mode
- Edge cases covered in tests

**Done means**: API is coherent, predictable, and follows DART conventions.

---

### Phase 2: Python Parity & Ergonomics

**Duration**: 3-4 weeks

**Scope**:

- Create `dartpy.simulation_experimental` module
- Bind all public classes
- Pythonic API design
- Examples and docstrings

**Tasks**:

- [ ] Create `python/dartpy/simulation/experimental/` directory
- [ ] Create `module.cpp` for submodule registration
- [ ] Bind `World` class with all methods
- [ ] Bind `MultiBody`, `Link`, `Joint` handle classes
- [ ] Bind `Frame`, `FreeFrame`, `FixedFrame`
- [ ] Bind `StateSpace` and mappers
- [ ] Add Python docstrings to all bindings
- [ ] Create `python/examples/simulation_experimental/` examples
- [ ] Add `test_simulation_experimental.py` Python tests
- [ ] Enable `DART_EXPERIMENTAL_BUILD_PYTHON` in CI

**Acceptance Criteria**:

- `import dartpy.simulation_experimental` works
- All public C++ classes accessible from Python
- Python examples run successfully
- Python tests pass in CI

**Done means**: Python users can use experimental API with full feature access.

---

### Phase 3: Testing Strategy

**Duration**: 2-3 weeks

**Scope**:

- Unit test coverage > 80%
- Integration tests with classic DART
- Golden tests for serialization
- Regression test infrastructure

**Tasks**:

- [ ] Audit test coverage with `gcov`/`lcov`
- [ ] Write missing unit tests to reach 80% coverage
- [ ] Create serialization golden tests (known-good binary files)
- [ ] Add integration test: load URDF then convert to experimental
- [ ] Add property-based tests for StateSpace
- [ ] Benchmark test: create 1000-body world, serialize, deserialize
- [ ] Add CI check for test coverage threshold
- [x] Document test patterns in `tests/unit/simulation/experimental/README.md`

**Acceptance Criteria**:

- Test coverage at least 80% for `dart/simulation/experimental/`
- All tests pass on Linux, macOS, Windows
- Golden tests catch serialization regressions
- Benchmark test completes in < 10 seconds

**Done means**: Changes can be made with confidence; regressions are caught.

---

### Phase 4: Performance & Benchmarking

**Duration**: 2-3 weeks (can run parallel with Phase 3)

**Scope**:

- Establish performance baseline
- Optimize ECS access patterns
- Compare with classic API

**Tasks**:

- [ ] Create benchmark: MultiBody creation (100, 1000, 10000 bodies)
- [ ] Create benchmark: Frame transform computation
- [ ] Create benchmark: StateSpace extraction/injection
- [ ] Profile ECS component access patterns
- [ ] Optimize hot paths identified by profiling
- [ ] Compare kinematics performance vs classic `Skeleton`
- [ ] Document performance characteristics
- [ ] Add CI performance regression check (optional)

**Acceptance Criteria**:

- Benchmark suite exists in `tests/benchmark/simulation/experimental/`
- Performance within 2x of classic API for equivalent operations
- No memory leaks (valgrind clean)
- Documentation of known performance characteristics

**Done means**: Performance is understood and acceptable for production use.

---

### Phase 5: Physics Integration (MAJOR)

**Duration**: 6-8 weeks

**Scope**:

- Implement `World::step()` physics loop
- Integrate with collision detection
- Integrate with constraint solver
- ABA/RNEA for forward/inverse dynamics

**Tasks**:

- [x] Design physics pipeline for ECS architecture
- [x] Implement mass property components
- [x] Implement forward kinematics system
- [x] Implement forward dynamics (ABA) system
- [x] Integrate `dart::collision::CollisionDetector`
- [x] Integrate `dart::constraint::ConstraintSolver`
- [x] Implement `World::step()` with configurable integrator
- [x] Add physics unit tests
- [x] Add physics integration tests
- [x] Validate against classic API (same inputs produce same outputs)

**Acceptance Criteria**:

- `World::step()` advances simulation
- Gravity affects falling bodies
- Contact constraints resolved
- Joint limits enforced
- Physics matches classic API within tolerance

**Done means**: Experimental World can replace classic World for simulation.

---

### Phase 6: Migration Story

**Duration**: 3-4 weeks

**Scope**:

- Deprecation plan for classic World
- Migration guides
- Adapters/converters
- User communication

**Tasks**:

- [ ] Write migration guide: classic to experimental
- [ ] Create adapter: `classicWorld.toExperimental()` (if feasible)
- [ ] Create adapter: `experimentalWorld.toClassic()` (if feasible)
- [ ] Add deprecation warnings to classic World in DART 7
- [ ] Update all examples to show both APIs (where applicable)
- [ ] Announce deprecation timeline in release notes
- [ ] Update `docs/onboarding/release-roadmap.md`
- [ ] Create FAQ for migration questions

**Acceptance Criteria**:

- Migration guide exists and is accurate
- At least one-way conversion works
- Deprecation warnings compile-time visible
- Timeline documented

**Done means**: Users have clear path to migrate; DART 8 transition is smooth.

---

## Progress Tracker

### Phase 0: Ground-Truth (Target: Week 1-2) ✅ COMPLETE

- [x] Epic document created
- [x] `test_joint.cpp` implemented (41 tests total)
- [x] `test_link.cpp` implemented (14 tests)
- [x] `test_rigid_body.cpp` implemented (14 tests)
- [x] All joint types tested (Fixed, Revolute, Prismatic, Screw, Universal, Ball, Planar, Free)
- [x] Doxygen comments reviewed (already present in public headers)
- [x] Hello world example created (`examples/simulation_experimental_hello_world/`)

### Phase 1: API Cleanup (Target: Week 3-5) ✅ COMPLETE

- [x] Naming audit complete (camelCase methods, PascalCase classes, snake_case files)
- [x] Exception review complete (InvalidOperationException for mode violations)
- [x] Lifetime docs complete (all handle classes have @note about invalidation)
- [x] Design mode guards complete (World and MultiBody check simulation mode)
- [x] All joint types implemented (data model complete, kinematics is Phase 5)
- [x] Joint state accessors (position, velocity, acceleration, torque) - 41 tests
- [x] Joint limits accessors (position lower/upper, velocity, effort)
- [x] RigidBody implemented (mass, inertia, pose, velocity, forces) - 14 tests

### Phase 2: Python Bindings (Target: Week 6-9) ✅ COMPLETE

- [x] Module structure created (`python/dartpy/simulation_experimental/`)
- [x] World bound with all methods
- [x] MultiBody/Link/Joint bound
- [x] Frame classes bound (Frame, FreeFrame, FixedFrame)
- [x] RigidBody bound
- [x] JointType enum bound
- [x] StateSpace bound with Variable struct
- [x] Python tests passing (16 tests)
- [x] Python docstrings: deferred (follows existing dartpy pattern - C++ Doxygen is source of truth)

### Phase 3: Testing (Target: Week 10-12) 🔄 IN PROGRESS

- [ ] Coverage audit complete (requires coverage build)
- [ ] 80% coverage achieved
- [x] Golden tests created (5 new tests: format stability, version header, stress test, all joint types)
- [ ] Integration tests created (BLOCKED: URDF→experimental requires Phase 6 adapter)

### Phase 4: Performance (Target: Week 10-12, parallel) 🔄 IN PROGRESS

- [x] Benchmarks created (`bm_multi_body.cpp` with 12 comprehensive benchmarks)
- [x] ECS profiling covered (existing `bm_ecs_safety.cpp`)
- [ ] Optimizations applied (pending profiling results)
- [ ] Documentation written
- [ ] Classic comparison (BLOCKED: requires Phase 5 kinematics)

### Phase 5: Physics (Target: Week 13-20) ✅ COMPLETE

#### Phase 5.1: Forward Kinematics ✅ COMPLETE

- [x] Joint transform functions (`kinematics/joint_transform.hpp/cpp`)
- [x] All 8 joint types supported: Fixed, Revolute, Prismatic, Screw, Universal, Ball, Planar, Free
- [x] Link::getLocalTransform() integrated with joint positions
- [x] Cache invalidation on joint position changes
- [x] 74 unit tests for joint transforms (`test_joint_transform.cpp`)
- [x] 5 FK integration tests in `test_link.cpp`

#### Phase 5.2: Forward Dynamics ✅ COMPLETE

- [x] Spatial math utilities (`dynamics/spatial_math.hpp/cpp`)
- [x] Motion subspace computation (`dynamics/motion_subspace.hpp/cpp`)
- [x] ABA workspace data structures (`dynamics/articulated_body.hpp/cpp`)
- [x] ForwardDynamicsSystem with full ABA (`dynamics/forward_dynamics.hpp/cpp`)
- [x] CoM offset support in MassProperties
- [x] External force/torque support on Links
- [x] Exact validation against classic DART (1e-10 tolerance)
- [x] 57 dynamics unit tests passing

#### Phase 5.5: World::step() ✅ COMPLETE

- [x] Semi-implicit Euler integration (velocity then position)
- [x] Configurable gravity, timeStep
- [x] Time and frame tracking
- [x] Automatic force clearing after step
- [x] Physics correctness tests (free-fall, external forces)

#### Phase 5.3-5.4: Collision & Constraints ✅ COMPLETE

- [x] Collision detection integration (classic adapter to `dart::collision`)
- [x] Constraint solver integration (classic adapter to `dart::constraint`)
- [x] ShapeNode support wired into classic collision + constraint solving

### Phase 6: Migration (Target: Week 21-24)

- [ ] Guide written
- [ ] Adapters created
- [ ] Deprecations added
- [ ] Timeline announced

---

## Proposed Follow-Up GitHub Issues

| Title                                                       | Description                                                                                                                               |
| ----------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| `[simulation-experimental] Implement remaining joint types` | Add Fixed, Ball, Universal, Planar, Free, Screw joint types to complete the JointType enum. Priority: Fixed, Ball (most commonly needed). |
| `[simulation-experimental] Fill empty test files`           | `test_joint.cpp` and `test_link.cpp` are 0 bytes. Add basic unit tests for Joint and Link handle classes.                                 |
| `[simulation-experimental] Create Python bindings`          | Create `dartpy.simulation_experimental` module. Start with World, MultiBody, Link, Joint bindings.                                        |
| `[simulation-experimental] Design physics pipeline`         | RFC/design doc for integrating ABA, collision, constraints into ECS architecture. Major architecture decision.                            |
| `[simulation-experimental] Add hello_world example`         | Create minimal example in `examples/simulation_experimental/` demonstrating World, MultiBody, and frame creation.                         |

---

## Related Documents

- `docs/dev_tasks/world_split/00_design.md` - Original design rationale
- `docs/dev_tasks/world_split/01_migration.md` - Migration phases
- `docs/onboarding/release-roadmap.md` - DART 7/8 release strategy
- `docs/onboarding/code-style.md` - snake_case conventions for experimental code
