# Resume: Experimental Collision Module

## Last Session Summary (2026-02-09 — Session 3)

**Major milestone**: FCL/Bullet/ODE fully decoupled from libdart.so into separate shared libraries. Experimental backend is now the default collision detector. Build succeeds (547 test targets), all key tests pass.

### Session 3 Accomplishments

1. **FCL/Bullet/ODE → separate .so files**: `dart-collision-fcl`, `dart-collision-bullet`, `dart-collision-ode` now build independently, linking `PUBLIC dart + external_lib`
2. **Default backend = experimental**: Factory fallback chain: experimental → fcl → dart
3. **Source decoupled**: constraint_solver.cpp, world.cpp, skel_parser.cpp use factory only (no FCL/Bullet/ODE includes)
4. **BVH mesh support**: Per-mesh AABB BVH with median split, mesh-mesh dual-tree traversal
5. **Persistent manifold cache**: Contact caching with warm-start in ContactConstraint
6. **Integration tests updated**: 7 test suites include experimental, all pass
7. **Benchmark baseline**: Experimental 5-25x faster than FCL
8. **Test fixes**: World tests updated for new default, adaptShape ambiguity resolved

## Current Branch

`feature/new_coll` — uncommitted changes pending commit

```bash
cd /home/js/dev/dartsim/dart/task_2
git checkout feature/new_coll
git status && git log -3 --oneline
```

## Test Status

```bash
# Unit collision: 20/20 pass
ctest --test-dir build/default/cpp/Release -R "UNIT_collision" -j$(nproc)

# Collision-experimental: 28/30 pass (2 pre-existing RayStartsInside failures)
ctest --test-dir build/default/cpp/Release -L collision-experimental -j$(nproc)

# Integration collision: 3/3 pass (CollisionGroups, Collision, CollisionAccuracy)
ctest --test-dir build/default/cpp/Release -R "INTEGRATION_collision_Collision" -j$(nproc)

# World integration: 1/1 pass
ctest --test-dir build/default/cpp/Release -R "INTEGRATION_simulation_World" -j$(nproc)
```

## Architecture (Current State)

```
libdart.so (core)
├── collision/ (interfaces + dart backend + experimental_backend adapter)
├── constraint/, dynamics/, simulation/, etc.
└── Links: Eigen, assimp, fmt, spdlog

dart-collision-experimental.so (narrow phase engine)
└── Links: dart

dart-collision-fcl.so (optional, for tests/benchmarks/gazebo)
└── Links: dart, fcl

dart-collision-bullet.so (optional, for tests/benchmarks/gazebo)
└── Links: dart, Bullet

dart-collision-ode.so (optional, for tests/benchmarks/gazebo)
└── Links: dart, ODE
```

## Remaining Work (Priority Order)

### Phase 2: Gazebo Backward Compatibility (NEXT)

**Constraint**: `pixi run -e gazebo test-gz` must pass. gz-physics **inherits** from BulletCollisionDetector and OdeCollisionDetector.

**Approach (Option D — Hybrid Facade)**: Keep class names and inheritance surfaces (FCLCollisionDetector, BulletCollisionDetector, OdeCollisionDetector) but make their collide()/distance()/raycast() internally delegate to experimental. The external deps (libfcl, libbullet, libode) remain linked for ABI compat but actual collision work uses experimental.

**Key files to modify**:

- `dart/collision/fcl/fcl_collision_detector.cpp` — override collide/distance to forward to experimental
- `dart/collision/bullet/bullet_collision_detector.cpp` — same
- `dart/collision/ode/ode_collision_detector.cpp` — same
- Headers stay identical (gz-physics includes them and subclasses)

### Phase 3: Break dynamics↔collision Circular Dependency

**Goal**: collision module should NOT include dynamics headers. dynamics depends on collision.

**The coupling**: collision's `CollisionObject` holds `ShapeFrame*` (dynamics type). Experimental narrow phase includes dynamics shape headers (BoxShape, SphereShape, etc.).

**Approach**: Move Shape hierarchy to `dart/geometry/` or create collision-side abstract interfaces. This is an XL task requiring careful planning.

### Phase 4: Final Gates

- `pixi run lint`, full test suite, `pixi run -e gazebo test-gz`
- Commit, PR

## Context That Would Be Lost

- **Bullet/ODE CMakeLists**: Had `dart_add_core_headers` leak — now removed, builds as separate .so
- **DART_HAVE_FCL**: Added to `config.hpp.in` as `#cmakedefine01`
- **Factory pattern**: All detector creation uses `CollisionDetector::getFactory()->create("name")`
- **gz-physics inherits**: `GzBulletCollisionDetector : BulletCollisionDetector`, `GzOdeCollisionDetector : OdeCollisionDetector`
- **gz-physics includes**: `<dart/collision/bullet/BulletCollisionDetector.hpp>`, `<dart/collision/fcl/FCLCollisionDetector.hpp>`, `<dart/collision/ode/OdeCollisionDetector.hpp>`

## Key Files Reference

| Component               | Location                                                                  |
| ----------------------- | ------------------------------------------------------------------------- |
| Backend detector        | `dart/collision/experimental_backend/experimental_collision_detector.cpp` |
| Shape adapter           | `dart/collision/experimental_backend/shape_adapter.cpp`                   |
| FCL separate lib        | `dart/collision/fcl/CMakeLists.txt`                                       |
| Bullet separate lib     | `dart/collision/bullet/CMakeLists.txt`                                    |
| ODE separate lib        | `dart/collision/ode/CMakeLists.txt`                                       |
| Core CMake              | `dart/CMakeLists.txt` (no FCL/Bullet/ODE link)                            |
| World (factory default) | `dart/simulation/world.cpp`                                               |
| Constraint solver       | `dart/constraint/constraint_solver.cpp`                                   |
| Config defines          | `dart/config.hpp.in` (DART_HAVE_FCL/BULLET/ODE)                           |
| Integration test CMake  | `tests/integration/CMakeLists.txt`                                        |
| Progress docs           | `docs/dev_tasks/experimental_collision/progress.md`                       |
