# Resume: Simulation Experimental API Epic

## Quick Status

**Phases 0, 1, 2 COMPLETE. Phase 3 & 4 IN PROGRESS.**

| Phase | Status         | Description                                              |
| ----- | -------------- | -------------------------------------------------------- |
| 0     | ✅ Complete    | Ground-truth: tests, docs, example                       |
| 1     | ✅ Complete    | API cleanup: all joint types, state accessors, RigidBody |
| 2     | ✅ Complete    | Python bindings: all classes + StateSpace                |
| 3     | 🔄 In Progress | Testing strategy: golden tests done, coverage pending    |
| 4     | 🔄 In Progress | Performance: benchmarks done, profiling pending          |
| 5     | Future         | Physics integration: `World::step()`                     |
| 6     | Future         | Migration story                                          |

## Current Branch

```
Branch: feature/sim_exp
Status: Up to date with origin (previously pushed)
Working tree: Modified (uncommitted Phase 3/4 work)
```

## Last Session Summary

Added Phase 3 & 4 deliverables:

- **Golden tests**: 5 new tests in `test_serialization.cpp` (format stability, deterministic output, large world stress test)
- **Benchmarks**: New `bm_multi_body.cpp` with comprehensive benchmarks for MultiBody creation, serialization, Frame transforms, StateSpace operations, and joint state access

## Immediate Next Step

1. **Commit and push** Phase 3/4 progress
2. **Run coverage audit** (`pixi run coverage-report`) to identify gaps
3. **Complete remaining Phase 3 task**: URDF → experimental integration test

## What Was Added This Session

### Phase 3: Testing (Partial)

| Item                  | Status | Location                                                                |
| --------------------- | ------ | ----------------------------------------------------------------------- |
| Golden tests          | ✅     | `tests/unit/simulation/experimental/world/test_serialization.cpp`       |
| Format stability test | ✅     | `SerializationGolden::DeterministicOutput`                              |
| Version header test   | ✅     | `SerializationGolden::FormatVersionPresent`                             |
| Large world stress    | ✅     | `SerializationGolden::LargeWorldStressTest` (100 robots × 7 links)      |
| All joint types test  | ✅     | `SerializationGolden::AllJointTypesRoundTrip`                           |
| Coverage audit        | ⏳     | Pending: run `pixi run coverage-report`                                 |
| URDF integration test | ⏳     | Pending: load URDF → convert to experimental                            |

### Phase 4: Performance (Partial)

| Benchmark              | Args                  | Location                                                         |
| ---------------------- | --------------------- | ---------------------------------------------------------------- |
| BM_CreateMultiBodies   | 100, 1000, 10000      | `tests/benchmark/simulation/experimental/bm_multi_body.cpp`      |
| BM_CreateLinkChain     | 10, 50, 100, 500      | Same file                                                        |
| BM_CreateBranchingTree | 3, 5, 7, 10 (depth)   | Same file                                                        |
| BM_SerializeWorld      | 10, 100, 1000 robots  | Same file                                                        |
| BM_DeserializeWorld    | 10, 100, 1000 robots  | Same file                                                        |
| BM_SerializeRoundTrip  | 10, 100, 1000 robots  | Same file                                                        |
| BM_JointStateAccess    | 10, 50, 100, 500      | Same file                                                        |
| BM_FrameChainTransform | 10, 50, 100, 500      | Same file                                                        |
| BM_FrameTransformUpdate| 10, 100, 1000         | Same file                                                        |
| BM_StateSpaceCreate    | 10, 100, 1000         | Same file                                                        |
| BM_StateSpaceGetBounds | 10, 100, 1000         | Same file                                                        |
| BM_StateSpaceVariableLookup | 10, 100, 1000    | Same file                                                        |
| ECS profiling          | ⏳                    | Pending: detailed component access profiling                     |
| Classic comparison     | ⏳                    | Pending: compare vs `Skeleton::computeForwardKinematics`         |

## How to Resume

```bash
# 1. Checkout and verify branch state
git checkout feature/sim_exp
git status
git diff --stat

# 2. Verify tests pass (should all pass)
pixi run build
ctest -L simulation-experimental --test-dir build/default/cpp/Release
pixi run pytest python/tests/unit/simulation_experimental/ -v

# 3. Commit Phase 3/4 progress
git add tests/benchmark/simulation/experimental/bm_multi_body.cpp
git add tests/benchmark/simulation/experimental/CMakeLists.txt
git add tests/unit/simulation/experimental/world/test_serialization.cpp
git commit -m "feat(simulation-experimental): Add benchmarks and golden tests (Phase 3/4)"

# 4. Run coverage audit
pixi run config-coverage && pixi run coverage-report

# 5. Run benchmarks
./build/default/cpp/Release/tests/benchmark/simulation/experimental/bm_multi_body
```

## Test Verification Commands

```bash
# C++ tests (13 test binaries)
ctest -L simulation-experimental --test-dir build/default/cpp/Release

# Python tests (16 tests)
pixi run pytest python/tests/unit/simulation_experimental/ -v

# Full validation
pixi run test-all

# Run benchmarks
./build/default/cpp/Release/tests/benchmark/simulation/experimental/bm_multi_body --benchmark_min_time=0.5s
```

## Remaining Phase 3 Tasks

- [ ] Audit test coverage → Coverage build was started but takes 30+ minutes. Check with:
  ```bash
  pgrep -f 'ninja|cmake_build' || pixi run coverage-report
  ```
- [ ] Write missing unit tests to reach 80% coverage (after coverage report)
- [x] Integration test: BLOCKED - requires Phase 6 adapter (classic→experimental conversion)

**Note**: The URDF→experimental integration test requires a conversion adapter that will be built in Phase 6. For now, the golden tests provide comprehensive serialization coverage.

## Remaining Phase 4 Tasks

- [x] Profile ECS component access patterns → Covered by existing `bm_ecs_safety.cpp`
- [x] Classic comparison → BLOCKED - requires Phase 5 kinematics implementation
- [ ] Document performance characteristics (after running benchmarks)

## Related Files

- **Epic tracker**: `docs/dev_tasks/simulation_experimental_api_epic/README.md`
- **C++ source**: `dart/simulation/experimental/`
- **Python bindings**: `python/dartpy/simulation_experimental/`
- **Python tests**: `python/tests/unit/simulation_experimental/`
- **C++ tests**: `tests/unit/simulation/experimental/`
- **Benchmarks**: `tests/benchmark/simulation/experimental/`
