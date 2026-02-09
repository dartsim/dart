# Resume: Experimental Collision Module

## Last Session Summary (2026-02-09 — Session 5)

**Major milestone**: Phase 3 complete — all collision→dynamics .cpp dependencies broken via bridge pattern. Zero dynamics includes remain in collision core .cpp files. All tests pass with zero regressions.

### Session 5 Accomplishments

1. **Phase 3 Bridge Pattern**: Moved dynamics-dependent implementations from `dart/collision/*.cpp` into 4 new bridge files under `dart/dynamics/detail/`
2. **12 collision files stripped**: Zero `dart/dynamics/` includes in collision core .cpp files
3. **Gazebo fix**: Conditionally exclude `experimental_collision_bridge.cpp` when `DART_BUILD_COLLISION_EXPERIMENTAL=OFF`
4. **Full verification**: 293/299 tests (same 6 pre-existing), 65/65 Gazebo, lint clean
5. **Committed**: `e96bcf436ec` on `feature/new_coll`

### Previous Sessions (commits)

- `e0ea977ddab` — Wire distance/raycast queries, shape adapters
- `3229fb81add` — Decouple FCL/Bullet/ODE into facades routing through experimental backend
- `b6c9bd655ab` — Restore native narrow phase for FCL/Bullet/ODE backends

## Current Branch

`feature/new_coll` — clean working tree, all committed

```bash
cd /home/js/dev/dartsim/dart/task_2
git checkout feature/new_coll
git status && git log -5 --oneline
```

## Test Status

```bash
# Full suite: 293/299 pass (6 pre-existing failures)
ctest --test-dir build/default/cpp/Release -j$(nproc) --output-on-failure

# Gazebo: 65/65 pass
pixi run -e gazebo test-gz

# Pre-existing failures (NOT caused by our changes):
# 4 - INTEGRATION_collision_MeshContactRegression
# 19 - INTEGRATION_constraint_ContactConstraint
# 57 - INTEGRATION_simulation_Issue410
# 229 - UNIT_gui_vsg_geometry_builders (SEGFAULT)
# 258 - test_diagnostics_profiling (Not Run)
# 291 - test_raycast (Failed)
```

## Architecture (Current State)

```
libdart.so (core)
├── collision/ (interfaces + dart backend + experimental_backend adapter)
│   └── .cpp files have ZERO dynamics includes (bridge pattern)
├── dynamics/detail/
│   ├── collision_bridge.cpp              # core collision→dynamics bridge
│   ├── collision_group_bridge.hpp        # template impls (addShapeFramesOf)
│   ├── dart_collision_bridge.cpp         # DART backend collision math
│   └── experimental_collision_bridge.cpp # adaptShape() (conditional)
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

### Optional: Performance Optimization
- Benchmark experimental vs FCL/Bullet for specific workloads
- Persistent manifold warm-starting
- BVH traversal for mesh-mesh

### Optional: Collision Header Cleanup
- Collision `.hpp` files still use `dart/dynamics/fwd.hpp` forward declarations
- Full decoupling would require extracting shapes to `dart/geometry/` module (XL task)

### Phase 2 (Gazebo Facades): DEFERRED
- Native narrow phase was restored for FCL/Bullet/ODE in commit `b6c9bd655ab`
- Facades not needed currently since backends use their own native implementations

## Context That Would Be Lost

- **Bridge pattern key insight**: C++ allows member function implementations in any .cpp that compiles into the same binary target. Since `dart/collision/` and `dart/dynamics/` both use `dart_add_core_sources()` → `libdart.so`, member functions of collision classes can be defined in `dart/dynamics/detail/*.cpp`.
- **CMake auto-discovery**: `dart/dynamics/CMakeLists.txt` uses `file(GLOB detail_srcs "detail/*.cpp")`, so new bridge files are auto-discovered.
- **Gazebo conditional**: `experimental_collision_bridge.cpp` must be excluded when `DART_BUILD_COLLISION_EXPERIMENTAL=OFF` because it references `dart::collision::experimental::*` symbols that live in the separate `.so`.
- **6 pre-existing test failures**: These existed before Phase 3 and are unrelated to our changes.

## Key Files Reference

| Component                    | Location                                                                  |
| ---------------------------- | ------------------------------------------------------------------------- |
| Core bridge (collision→dyn)  | `dart/dynamics/detail/collision_bridge.cpp`                               |
| Template bridge              | `dart/dynamics/detail/collision_group_bridge.hpp`                         |
| DART backend bridge          | `dart/dynamics/detail/dart_collision_bridge.cpp`                          |
| Experimental bridge          | `dart/dynamics/detail/experimental_collision_bridge.cpp`                  |
| Dynamics CMakeLists          | `dart/dynamics/CMakeLists.txt` (conditional exclusion)                    |
| Backend detector             | `dart/collision/experimental_backend/experimental_collision_detector.cpp` |
| Shape adapter (stripped)     | `dart/collision/experimental_backend/shape_adapter.cpp`                   |
| FCL separate lib             | `dart/collision/fcl/CMakeLists.txt`                                       |
| Bullet separate lib          | `dart/collision/bullet/CMakeLists.txt`                                    |
| ODE separate lib             | `dart/collision/ode/CMakeLists.txt`                                       |
| Progress docs                | `docs/dev_tasks/experimental_collision/progress.md`                       |

## Dependency Verification

```bash
# Verify no dynamics includes in collision core .cpp files (should be ZERO):
grep -r '#include.*dart/dynamics/' dart/collision/ --include='*.cpp' | grep -v '/fcl/' | grep -v '/bullet/' | grep -v '/ode/'

# Check remaining dynamics includes in collision headers (fwd.hpp only):
grep -r '#include.*dart/dynamics/' dart/collision/ --include='*.hpp' | grep -v fwd.hpp
```
