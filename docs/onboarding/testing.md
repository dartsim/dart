# DART Test Suite

This directory contains the complete test suite for DART (Dynamic Animation and Robotics Toolkit). The tests are organized by type and module to facilitate easy navigation, maintenance, and scalability.

## Start here next time

- Local build/test workflow: [building.md](building.md)
- CI monitoring and expectations: [ci-cd.md](ci-cd.md)
- Gazebo / gz-physics integration: [build-system.md](build-system.md#gazebo-integration-feature)

## Verification Tiers

Pick the cheapest tier that still proves your change, then escalate. Tiers are
monotonic supersets (`quick` ⊂ bare ⊂ `full`); the bare verb is the everyday
default. See [`docs/design/local_verification_pipeline.md`](../design/local_verification_pipeline.md)
for the full naming scheme and rationale.

| When                     | Gate (lint+build+test) | Tests only   | Benchmarks    |
| ------------------------ | ---------------------- | ------------ | ------------- |
| Inner loop (every save)  | `verify-quick`         | `test-quick` | `bench-quick` |
| Pre-commit               | `verify`               | `test-core`  | `bench`       |
| Pre-push / authoritative | `verify-full`          | `test-full`  | `bench-full`  |

(`test-core` runs unit + integration; `test-simulation-quick` adds the
DART 7 simulation tests minus the long poles; `test-full` runs every label.)

`verify-full` is the authoritative gate (the former `test-all`, still available
as an alias). Subsystem scope stays orthogonal (`test-math`, `test-io`, …), and
the environment is always a `-e` flag (`pixi run -e cuda verify-full`).

Build and test parallelism are load-aware by default (ninja `-l`, ctest
`--test-load`), so several clones on one machine share the CPU without
oversubscription. Override the hard job cap with `DART_PARALLEL_JOBS` and the
load ceiling with `DART_BUILD_LOAD_LIMIT` (or pin both for every clone in
`~/.dart-dev/parallelism.env`). Opt-in build accelerators: `DART_USE_MOLD` (mold
linker, Linux non-CUDA) and `DART_NORMALIZE_BUILD_PATHS` (cross-clone
compiler-cache sharing).

## Fast Iteration Loop

Smallest repeatable local loop before a full CI run.

Choose a parallelism cap around two-thirds of logical cores, then set
`DART_PARALLEL_JOBS` and `CTEST_PARALLEL_LEVEL` to that value (see
[building.md](building.md) for details). On memory-constrained machines, use
a lower `N` with targeted builds and tests rather than changing build files.

Suggested (Unverified, Linux example):

```bash
N=$(( ( $(nproc) * 2 ) / 3 ))
```

Lint/format pass (fastest local sanity check).

Suggested (Unverified):

```bash
pixi run lint
```

Targeted build + test (optional, fastest when a single target fails).

Suggested (Unverified):

```bash
cmake --build <BUILD_DIR> --target <TARGET>
ctest --test-dir <BUILD_DIR> --output-on-failure -R <TEST_REGEX>
```

Example:

```bash
pixi run lint
CMAKE_BUILD_PARALLEL_LEVEL=$N cmake --build build/default/cpp/Release \
  --target UNIT_gui_MeshShapeNodeMaterialUpdates -j "$N"
CTEST_PARALLEL_LEVEL=1 ctest --test-dir build/default/cpp/Release \
  --output-on-failure -R UNIT_gui_MeshShapeNodeMaterialUpdates -j 1
```

Signals to look for:

- The lint task completes without reporting errors
- Any auto-formatting changes are expected and reviewed before committing

Targeted tests (optional, but recommended before pushing when behavior changes).

Suggested (Unverified):

```bash
N=$(( ( $(nproc) * 2 ) / 3 ))
DART_PARALLEL_JOBS=$N CTEST_PARALLEL_LEVEL=$N pixi run test
```

Signals to look for:

- The test runner ends with `100% tests passed`

Alignment-sensitive C++ build/test pass. Run this when allocator, placement-new,
Eigen storage, SIMD-sensitive math, or object-pool code changes. It forces Eigen
to use a 64-byte static alignment contract without requiring AVX-512 hardware.

Suggested (Unverified):

```bash
DART_PARALLEL_JOBS=$N CTEST_PARALLEL_LEVEL=$N pixi run test-eigen-overalignment
```

Signals to look for:

- The dedicated `eigen64-align` build configures successfully
- The C++ tests built in that configuration end with `100% tests passed`

Full validation.

Suggested (Unverified):

```bash
N=$(( ( $(nproc) * 2 ) / 3 ))
DART_PARALLEL_JOBS=$N CTEST_PARALLEL_LEVEL=$N pixi run test-all
if [ "$(uname -s)" = "Linux" ] && command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; then
  DART_PARALLEL_JOBS=$N CTEST_PARALLEL_LEVEL=$N pixi run -e cuda test-all
fi
DART_PARALLEL_JOBS=$N CTEST_PARALLEL_LEVEL=$N pixi run -e gazebo test-gz
```

Signals to look for:

- The full test run ends with `✓ All tests passed!`
- On a Linux CUDA host, the CUDA-environment run also ends with
  `✓ All tests passed!`
- The Gazebo integration workflow prints `✓ Full gz-physics suite passed and DART plugin links successfully!`

### CUDA Validation

`pixi run test-all` validates the default CPU/dartpy environment. It does not
prove CUDA runtime coverage by itself. When a Linux host exposes an NVIDIA CUDA
runtime, also run:

```bash
DART_PARALLEL_JOBS=$N CTEST_PARALLEL_LEVEL=$N pixi run -e cuda test-all
```

The CUDA-environment `test-all` preserves the `cuda` Pixi environment for nested
tasks, builds the CUDA targets, runs CTest coverage labelled
`simulation-cuda`, and then runs `pixi run -e cuda test-cuda` when a
CUDA runtime is detected. The CUDA smoke task currently covers:

- `test_rigid_body_state_batch_cuda`
- `test_deformable_psd_projection_cuda`
- `test_vbd_block_descent_cuda`
- `bm_cuda_rigid_body_state_batch`
- `bm_vbd_cuda`

When `DART_CUDA_ARCHITECTURES` is unset and `nvidia-smi` reports visible GPUs,
the CUDA Pixi config auto-detects compute capabilities and passes concrete CUDA
architectures to CMake. This avoids relying on PTX JIT compatibility between a
newer Pixi CUDA toolkit and the installed NVIDIA driver. Set
`DART_CUDA_ARCHITECTURES=<arch>` to override the detected list.

If the `cuda` environment is active but no CUDA runtime is detected, `test-all`
still validates the CUDA build path and reports that runtime tests were skipped.
Do not report GPU runtime validation from that run.

## Gotchas

- The first lint run can take a while and it runs auto-fixers; expect diffs, rerun if interrupted, and check `git status` before committing.
- `pixi run test-all` runs linting and documentation builds as part of the suite; expect longer runtime and potential formatting diffs, so review changes before committing.
- `pixi run -e cuda test-all` is Linux-only because the `cuda` Pixi environment
  is Linux-only. It requires a visible NVIDIA runtime to execute GPU runtime
  checks; otherwise CUDA tests are build-only or skipped.
- `pixi run lint` can rewrite identifiers via codespell; if a spelling or casing is intentional, add it to `.codespellrc` and re-run lint.
- If pixi reports a missing task or feature, your `pixi.toml` may be out of sync with the target branch; update it before retrying.

## Next-Time Accelerators

- Run lint early (and again after test-all if it reformats) to surface formatting/codespell changes before longer build/test cycles.
- When `pixi run test-py` is quiet for a long time, isolate the Python file or
  test directly instead of assuming the CMake target is still building. The
  generated `pytest` target can buffer output for the whole file. Use the build
  output paths explicitly so the direct run matches the in-tree dartpy build:

  ```bash
  PYTHONPATH=build/default/cpp/Debug/python:build/default/cpp/Debug/python/dartpy \
    DARTPY_RUNTIME_DIR=build/default/cpp/Debug/python/dartpy \
    .pixi/envs/default/bin/python3.14 -u -m pytest -vv -s <TEST_PATH>::<TEST_NAME>
  ```

  Debug builds make deformable and scene-style integration tests much more
  expensive than Release. Keep smoke fixtures small enough for Debug CI while
  preserving the behavior under test; put paper-scale or visual showcase
  coverage in benchmark/demo surfaces instead of routine Python integration
  tests.

## Directory Structure

```
tests/
├── benchmark/
│   ├── collision/           # Collision detection benchmarks
│   ├── dynamics/            # Dynamics and kinematics benchmarks
│   ├── integration/         # Integration benchmark template
│   └── unit/                # Unit benchmark template
├── integration/
│   ├── collision/           # Collision detection, groups, accuracy, self-collision
│   ├── constraint/          # Constraint solver, contact, friction, LCP stability
│   ├── dynamics/            # Multi-body dynamics, joints, kinematics, state management
│   ├── io/                  # File parsers (DART, MJCF, SDF, URDF, Skel)
│   ├── optimization/        # IK, multi-objective optimization
│   ├── simulation/          # World simulation, soft body dynamics, momentum conservation
│   └── utils/               # Resource retrievers, aspects, frames, signals
├── unit/
│   ├── collision/           # Collision API tests (distance, raycast)
│   ├── common/              # Allocators, factory, logging, memory, strings
│   ├── constraint/          # Constraint solver API
│   ├── dynamics/            # Joint tests, inertia calculations, shape node API
│   ├── gui/                 # DART GUI descriptor and public-boundary tests
│   ├── io/                  # Tests for parsers (URDF, SDF error paths)
│   ├── math/                # Geometry, math operations, random, meshes, LCP solvers
│   ├── sensor/              # Tests for the sensor API
│   ├── simulation/          # Tests for simulation world
│   └── utils/               # Tests for utility classes
├── helpers/
│   └── GTestUtils.hpp       # Shared GoogleTest utilities
└── README.md                # Points to this comprehensive guide
```

## Test Categories

### Integration Tests (`integration/`)

Integration tests verify that **multiple components work correctly together** in end-to-end scenarios. These tests:

- Test interactions between 2+ major subsystems (e.g., dynamics + collision + simulation)
- Set up complex scenarios with full physics simulations
- Validate cross-module functionality and data flow
- Use high-level objects (World, Skeleton, ConstraintSolver, etc.)
- May take longer to run than unit tests

**Key characteristics:**

- Tests system behavior, not component behavior
- Validates that components integrate correctly
- Uses file parsing, complete simulations, or multi-object interactions

**Organized by module:**

- **collision/**: Multi-component collision tests (e.g., collision groups with dynamics and simulation)
- **constraint/**: Constraint solving in full physics context (contact constraints, friction with dynamics)
- **dynamics/**: Multi-body dynamics, complex kinematics workflows, skeleton interactions
- **io/**: File loaders and parsers (DART, MJCF, SDF, URDF, Skel formats)
- **optimization/**: Inverse kinematics, multi-objective optimization with dynamics
- **simulation/**: World simulation with multiple skeletons, building complex scenes
- **utils/**: Resource retrievers, aspects, frames, signals with full system context

### Unit Tests (`unit/`)

Unit tests focus on testing **individual classes or functions in isolation**. These tests:

- Test a single component without dependencies on other major subsystems
- Are fast to execute (milliseconds)
- Have minimal dependencies (e.g., only dart-math, only the collision module)
- Test specific functionality, edge cases, and API correctness
- Use simple test fixtures (SimpleFrame, basic objects)

**Key characteristics:**

- Tests component behavior, not system behavior
- Validates correctness of individual algorithms or classes
- Can run independently without complex setup

**Organized by module:**

- **collision/**: Single-component collision tests (distance queries, raycasting API)
- **common/**: Allocators, factory patterns, logging, memory management, strings, URIs
- **constraint/**: Constraint solver API tests (without full physics)
- **dynamics/**: Individual joint tests, inertia calculations, single component tests
- **gui/**: Filament descriptor, scene extraction, and public-boundary guards.
  GUI tests should keep renderer implementation types out of public headers and
  prefer DART-owned concepts such as renderables, cameras, picking, debug draw,
  screenshots, and viewer lifecycle state.
- **io/**: Parser tests (URDF, SDF error paths)
- **math/**: Geometry primitives, icosphere generation, mathematical operations, random number generation, triangle meshes, LCP solvers (in `lcp/` subdirectory)
- **sensor/**: Sensor API tests
- **simulation/**: Simulation world tests
- **utils/**: Utility class tests

### How to Decide: Integration vs Unit?

**Use Integration Test if:**

- ✅ Tests interaction between World, Skeleton, and ConstraintSolver
- ✅ Loads files and runs full simulations
- ✅ Tests multiple modules working together
- ✅ Validates end-to-end workflows
- ✅ Depends on dart-utils or multiple dart-\* libraries

**Use Unit Test if:**

- ✅ Tests a single class or function in isolation
- ✅ Has minimal dependencies (1-2 modules max)
- ✅ Uses simple test fixtures (not full simulations)
- ✅ Tests API correctness, edge cases, or algorithms
- ✅ Can run in milliseconds

### Regression Test Naming

All regression tests use descriptive names organized by module, with GitHub issue references in comments:

```cpp
// Regression test for Issue #1184 (collision accuracy)
TEST_F(CollisionTest, test_CollisionAccuracy) { ... }
```

### Fallback Path Regressions

When optimized, batched, or parallel code can fall back to a simpler
implementation, add regression coverage that forces the fallback trigger and
asserts externally visible behavior. Do not rely only on equivalence checks
against the delegated implementation; a fallback can drop world options, cached
state, or execution context while still matching a shallow delegate comparison.

### Strided Map Regressions

When an API exposes Eigen-backed views into caller-owned storage, add regression
coverage for the view shape that downstream users are likely to hold, not only
for contiguous storage. This matters most for nested typed views, such as
Lie-group maps whose component accessors return submaps.

For these APIs, prefer tests that:

- instantiate both `Eigen::InnerStride<>` and
  `Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>` when nested maps are
  supported;
- call the nested accessor directly, then read and write through the returned
  view;
- verify that writes touch only the intended strided slots in the backing
  buffer.

### Benchmarks (`benchmark/`)

Performance benchmarks measure execution time and resource usage:

- **collision/**: Collision detection performance with various shapes and scenarios
- **dynamics/**: Kinematics computation, cache-friendly allocation, and lifecycle benchmarks
- **integration/**: End-to-end system performance
- **unit/**: Individual component performance

#### Benchmark Comparison Tool

`scripts/compare_benchmarks.py` compares two Google Benchmark JSON files to detect regressions:

```bash
# Run benchmarks and save JSON output
pixi run bm  # runs scripts/run_cpp_benchmark.py

# Compare baseline vs optimized results
python scripts/compare_benchmarks.py baseline.json optimized.json
```

The tool supports `--metric cpu_time|real_time` and `--aggregate` options.
See the script's `--help` for details.

Allocator benchmarks also have focused gates:

```bash
pixi run bm-check
pixi run bm-allocator-comparative-check
```

`bm-check` compares DART allocator workloads against the default allocator.
`bm-allocator-comparative-check` runs `bm_allocators_comparative` and fails
when DART does not beat the selected foonathan/memory or standard-library
baseline on matching workloads. It also rejects compared aggregate rows whose
coefficient of variation exceeds `--max-cv` (default 10%), because noisy rows are
not valid evidence for strict allocator decisions unless the saved benchmark
mean/stddev/repetition aggregates still show DART's normal-approximation 95%
confidence interval strictly below the selected baseline's confidence interval.

Pass `--include-entt-registry` to add allocator-aware EnTT registry/component
storage churn rows. Those rows are an evidence surface for registry allocator
policy work until the production registry allocator path consistently beats the
selected baselines. Use `--only-entt-registry` for focused registry allocator
optimization loops without rerunning the broader allocator benchmark set.

### DART 7 Simulation-Loop Allocation Gates

For DART 7 `World::step()` work, the no-allocation contract is "after bake":
`World::enterSimulationMode()` and stage `prepare()` may reserve memory for the
current shape, but the measured simulation loop starts on the first post-bake
step. The final gate should prove all applicable allocation surfaces stay quiet:

- World base allocator counters do not grow across same-shape steps.
- Global `operator new` counters report zero allocations.
- Raw malloc-family counters report zero allocations on supported hosts when the
  path uses Eigen dynamic matrices, decompositions, sparse factorizations, or
  other malloc-backed scratch.

Existing helper patterns live in `tests/unit/simulation/world/test_world.cpp`:
`expectNoWorldBaseAllocatorActivityDuringBakedSteps`,
`expectNoGlobalHeapAllocationsDuringBakedSteps`, and
`expectNoRawHeapAllocationsDuringFirstPostBakeSteps` start counting on the first
post-bake step. `expectNoRawHeapAllocationsDuringSteadyStateBakedSteps` is for
raw-malloc regressions that still need unmeasured warm-up steps before the
counter starts; those are not final evidence for the stricter
no-allocation-after-bake requirement. `pixi run
check-plan122-allocation-matrix` verifies that every `Closed` coverage row cites
an existing test. Track final and steady-state rows in
[`../plans/122-simulation-loop-allocation-hardening/coverage-matrix.md`](../plans/122-simulation-loop-allocation-hardening/coverage-matrix.md).

## Adding New Tests

### Naming Conventions

- **Integration tests**: `test_<ModuleOrFeature>.cpp` (e.g., `test_Collision.cpp`)
- **Unit tests**: `test_<ClassName>.cpp` (e.g., `test_Factory.cpp`)
- **Issue-based regressions**: Use a descriptive filename in `unit/` or `integration/`, and include the GitHub issue number/link in a comment near the test.
- **Benchmarks**: `bm_<feature>.cpp` (e.g., `bm_boxes.cpp`)

### Where to Add Your Test

1. **Is it testing a single class/function in isolation?** → Add to `unit/<module>/`
2. **Is it testing multiple components working together?** → Add to `integration/<module>/`
3. **Is it verifying a bug fix from a GitHub issue?** → Add to `unit/<module>/` or `integration/<module>/` (depending on scope) and include the issue number/link in a comment.
4. **Is it measuring performance?** → Add to `benchmark/<category>/`

**Common starting points (pick based on what you're changing):**

- C++ unit tests: `tests/unit/<module>/`
- C++ integration tests: `tests/integration/<module>/`
- Python unit tests (dartpy): `python/tests/unit/<module>/`

Suggested (Unverified): If you don't know the module yet, search for similar tests by symbol name, e.g. `rg -n "<ClassOrFeature>" tests python`.

Examples (Suggested, Unverified):

- `tests/integration/<module>/test_<Feature>.cpp`
- `python/tests/unit/<module>/`

### Steps to Add a New Test

1. **Create your test file** in the appropriate directory following naming conventions
2. **Update the CMakeLists.txt** in that directory:
   ```cmake
   dart_add_test("integration" test_YourTest)
   # If you need additional libraries:
   target_link_libraries(test_YourTest dart-utils)
   ```
3. **Write your test** using GoogleTest framework:

   ```cpp
   #include <gtest/gtest.h>
   #include "dart/dynamics/All.hpp"

   TEST(YourTestSuite, YourTestCase)
   {
     // Your test code here
     EXPECT_TRUE(condition);
   }
   ```

4. **Build and run** your test:
   ```bash
   cmake --build build
   cd build && ctest -R test_YourTest
   ```

## Test Utilities

The test helpers are organized by dependency weight to minimize compilation overhead.

### GTestUtils.hpp (Lightweight)

Contains utility functions for GoogleTest with **minimal dependencies** (only `dart/math`):

- Custom Eigen matrix/vector comparison macros (`EXPECT_VECTOR_NEAR`, `EXPECT_MATRIX_NEAR`, etc.)
- Floating-point comparison functions
- Transform and rotation equality checks

**Dependencies:** `dart/math`, `Eigen`, `gtest`

**Usage:** Suitable for all tests, especially lightweight unit tests

```cpp
#include "helpers/GTestUtils.hpp"
```

### common_helpers.hpp (Lightweight)

Provides mock utilities for common/utils testing with **lightweight dependencies**:

- `TestResource` - Mock implementation of Resource interface
- `PresentResourceRetriever` - Mock retriever that always succeeds
- `AbsentResourceRetriever` - Mock retriever that always fails

**Dependencies:** `dart/common` only

**Usage:** For IO and utils tests that need resource retriever mocks

```cpp
#include "helpers/common_helpers.hpp"
```

### dynamics_helpers.hpp (Heavy Dependencies)

Provides skeleton and object creation utilities for **integration tests**:

- Skeleton creation (`createThreeLinkRobot`, `createNLinkRobot`, `createNLinkPendulum`, etc.)
- Object creation (`createGround`, `createBox`, `createSphere`, etc.)
- Joint configuration utilities

**Dependencies:** Full DART library (dynamics, collision, math, etc.)

**Usage:** Only for integration tests that need complex skeleton setup

```cpp
#include "helpers/dynamics_helpers.hpp"
```

### Best Practice

Choose helpers based on what you actually need:

- **Math/geometry unit tests**: Use only `GTestUtils.hpp`
- **Common/utils tests**: Use `GTestUtils.hpp` + `common_helpers.hpp`
- **Dynamics unit tests**: Avoid helpers if possible; use `dynamics_helpers.hpp` sparingly
- **Integration tests**: Use `dynamics_helpers.hpp` for complex skeleton creation

**Principle:** Pay the compilation cost only for dependencies you actually use

## Running Tests

### Run all tests:

```bash
cd build
ctest
```

### Run tests by category:

````bash
ctest -L integration
ctest -L unit

### Run tests by category:

```bash
ctest -L integration
ctest -L unit
````

### Run a specific test:

```bash
ctest -R test_Collision
```

### Run tests in parallel:

```bash
ctest -j8  # Run 8 tests in parallel
```

### Run tests with verbose output:

```bash
ctest --verbose
ctest --output-on-failure  # Only show output for failing tests
```

### Run benchmarks:

```bash
pixi run bm boxes
pixi run bm kinematics
pixi run bm lcp_compare -- --benchmark_filter=BM_LCP_COMPARE_SMOKE
pixi run bm-dashboard-surfaces --surface lcp-solvers --dry-run
pixi run bm --pixi-help
```

> **Note:** LCP solver comparisons use the solver-agnostic harness and the
> `BM_LCP_COMPARE` benchmark so all solvers share the same contract and fixtures.
> The `lcp-solvers` dashboard surface keeps a bounded standard, boxed,
> friction-index, world-contact, billiards, mass-ratio stack, and card-pile
> slice reproducible for dashboard runs. See `tests/common/lcpsolver` and
> `tests/benchmark/lcpsolver` for the sources, and keep benchmark outputs under
> the build tree.

### Manual Python Testing

For faster iteration on Python bindings without full rebuilding:

```bash
# Point PYTHONPATH to the build artifacts (adjust path as needed)
PYTHONPATH=build/default/cpp/Release/python pytest python/tests/unit/optimizer/
```

This avoids the overhead of the full `pixi run test-py` task but requires the bindings to be already built (e.g., via `pixi run build-py-dev`).

## CMake Integration

The test suite uses custom CMake functions defined in `/tests/CMakeLists.txt`:

### `dart_add_test(test_type target_name [source_files...])`

Adds a new test executable:

```cmake
dart_add_test("integration" test_Collision)
dart_add_test("unit" test_Factory test_Factory.cpp)
```

### `dart_build_tests(...)`

Builds multiple tests from a list of sources:

```cmake
dart_build_tests(
  TYPE integration
  TARGET_PREFIX INTEGRATION
  LINK_LIBRARIES dart
  SOURCES
    test_Aspect.cpp
    test_Common.cpp
)
```

### `dart_get_tests(output_var test_type)`

Retrieves all tests of a given type:

```cmake
dart_get_tests(integration_tests "integration")
```

## Best Practices

1. **Keep tests focused**: Each test should verify one specific behavior
2. **Use descriptive names**: Test names should clearly indicate what they're testing
3. **Avoid test interdependencies**: Tests should be independent and runnable in any order
4. **Clean up resources**: Use RAII or proper teardown to clean up test resources
5. **Use fixtures for shared setup**: Create test fixtures for common initialization code
6. **Document complex tests**: Add comments explaining non-obvious test logic
7. **Keep tests fast**: Unit tests should run in milliseconds; longer tests belong in integration
8. **Test edge cases**: Don't just test the happy path
9. **Use appropriate assertions**: Choose the right assertion for better error messages
   - `EXPECT_EQ`, `EXPECT_NE`, `EXPECT_LT`, etc. for comparisons
   - `EXPECT_TRUE`, `EXPECT_FALSE` for boolean conditions
   - `EXPECT_NEAR` for floating-point comparisons
10. **Update tests with code changes**: Keep tests in sync with the code they verify

## Coverage Strategy

### Coverage Organization

Coverage work is test-suite maintenance, not fragmentary line-counting. Start by
reviewing the existing test layout, fixtures, and nearby coverage before adding
new tests. Prefer revising, extending, consolidating, splitting, or moving tests
when that makes behavior ownership clearer, reduces duplicate setup, or keeps
future code changes easy to cover.

Add a new test file only for a distinct behavior family, dependency boundary, or
fixture shape. Keep coverage targets focused on meaningful public behavior and
backend dispatch paths; do not chase brittle coverage for GUI/OpenGL paths,
DART 7 simulation internals, debug-only fatal assertions, or unreachable
defensive branches unless the task explicitly calls for them.

When auditing core coverage, treat test binary coverage as part of the capture
phase and filter test sources only after lcov records the data. This keeps
template-heavy library headers and smart pointer wrappers counted when they are
instantiated only from focused tests. Use aggregate headless-core metrics for
planning, then pick the next suite slice from real workflow gaps rather than
per-file percentage chasing.

### Coverage Patterns

Areas that commonly need additional test coverage (identified from test-coverage-audit):

| Pattern                | What to Test                                                         | Example                                                |
| ---------------------- | -------------------------------------------------------------------- | ------------------------------------------------------ |
| **Smart pointers**     | Lifecycle, expiration, owner lifetime                                | `BodyNodePtr`, `NodePtr`, `WeakPtr` variants           |
| **Aspect system**      | `has<T>()`, `get<T>()`, `removeAspect<T>()`, `isSpecializedFor<T>()` | Composite template methods                             |
| **Constraint solvers** | Empty inputs, duplicate skeletons, timestep changes, null detectors  | `ConstraintSolver` edge cases                          |
| **Collision filters**  | Self-collision, blacklist edge cases, composite behavior             | `CollisionFilter` combinations                         |
| **Template classes**   | All template instantiations, not just common types                   | `TemplateBodyNodePtr<BodyNode>` vs `<const BodyNode>`  |
| **Allocators**         | Dispatch variants, no-op deallocation, growth, overflow, printing    | `MemoryManager`, `FreeListAllocator`, `FrameAllocator` |
| **Optimization APIs**  | Null/no-objective problems, base copy paths, wrapper dispatch        | `Problem`, `Solver`, `GradientDescentSolver`           |

**Key insight**: Template classes and smart pointer wrappers often have lower coverage because tests only exercise common instantiations. Add explicit tests for const variants, weak references, and edge cases like expired pointers.

Coverage audits are most useful when tests exercise meaningful API branches rather than just line counts. Prefer focused tests for public dispatch paths and backend helpers that are linked into core modules but missed by higher-level workflows.

## Common Pitfalls and Solutions

### Missing Test Namespace

**Problem:** Compilation errors like `'equals' was not declared in this scope` when using helper utilities.

**Solution:** Always include the test namespace when using helpers:

```cpp
#include "helpers/GTestUtils.hpp"
using namespace dart::test;  // Critical for equals(), verifyTransform(), etc.
```

### Incorrect Include Order

**Problem:** Clang-format reorders includes causing build issues.

**Solution:** Follow the include ordering pattern (most specific to most general):

1. Test helper headers (quoted paths: `"helpers/GTestUtils.hpp"`)
2. DART library headers (angle brackets, ordered by layer: `<dart/math/...>`, `<dart/dynamics/...>`, etc.)
3. External dependencies (`<gtest/gtest.h>`, `<Eigen/Dense>`, etc.)
4. System headers (`<iostream>`, `<vector>`, etc.)

### Missing Eigen Namespace

**Problem:** Compilation errors in test files using Eigen types like `Matrix3d`, `Isometry3d`, etc.

**Solution:** Add Eigen namespace when needed:

```cpp
using namespace dart;
using namespace dart::math;
using namespace Eigen;  // For Eigen types in test code
```

### Benchmark Tests Not Building

**Problem:** Benchmark tests fail to link with "undefined reference to benchmark::State" errors.

**Solution:** Ensure benchmark tests link against the benchmark library:

```cmake
target_link_libraries(bm_yourtest dart-utils benchmark::benchmark)
```

### Windows DLL Export Issues in Tests

**Problem:** New tests call methods that aren't exported from Windows DLLs, causing linker errors like "unresolved external symbol" on Windows CI.

**Solution:** Add `DART_API` exports to the methods being tested:

- For regular classes: Add `DART_API` at class level in the header
- For CRTP template classes (e.g., `FixedJacobianNode`): Add `DART_API` to individual methods, NOT class level (class-level causes MSVC C2512)
- If a class already has class-level `DART_API`, do NOT add method-level (causes MSVC C2487)

**Pattern:** When adding tests that call new public APIs, verify Windows CI passes or add exports proactively.

### Using Deprecated Headers

**Problem:** Warnings treated as errors when using deprecated aggregate headers.

**Solution:** Use modern header names:

- `dart/utils/utils.hpp` → `dart/utils/All.hpp`
- `dart/simulation/simulation.hpp` → `dart/simulation/All.hpp`
- `dart/constraint/constraint.hpp` → `dart/constraint/All.hpp`
- `dart/collision/bullet/bullet.hpp` → `dart/collision/bullet/All.hpp`

### Using `M_PI` Instead of `dart::math::pi`

**Problem:** `M_PI` is not defined by the C++ standard. MSVC does not provide it unless `_USE_MATH_DEFINES` is defined before `<cmath>`, causing compilation failures on Windows CI.

**Solution:** Always use `dart::math::pi` (from `<dart/math/Constants.hpp>`):

```cpp
// WRONG - fails on Windows MSVC
joint->setPositionLimits(0, -M_PI, M_PI);

// CORRECT - portable across all platforms
joint->setPositionLimits(0, -dart::math::pi, dart::math::pi);
```

### Windows URI and Path Portability in Tests

**Problem:** Tests that construct URIs from filesystem paths or embed paths in XML model files can fail on Windows due to backslash separators, missing drive letters, or broken mesh resolution.

**Key patterns:**

1. **Use `Uri::createFromPath()` instead of `Uri(path.string())`**: The `Uri` constructor stores backslashes on Windows, which breaks mesh resolution in parsers. `Uri::createFromPath()` properly converts to a `file:///` URI with forward slashes and a drive letter.

   ```cpp
   // WRONG — fails on Windows when used as a base URI for mesh resolution
   common::Uri uri(path.string());

   // CORRECT — portable across all platforms
   common::Uri uri = common::Uri::createFromPath(path.string());
   ```

2. **Use the two-arg `loadMesh(uri, retriever)` overload**: The single-arg `MeshShape::loadMesh(filePath)` internally prepends `"file://"` without drive letter handling on Windows. Use the two-arg overload with a proper URI instead.

   ```cpp
   // WRONG — broken on Windows (produces "file:///mesh.obj" without drive letter)
   meshShape->loadMesh(meshPath);

   // CORRECT — portable
   auto uri = common::Uri::createFromPath(meshPath);
   auto retriever = std::make_shared<common::LocalResourceRetriever>();
   meshShape->loadMesh(uri.toString(), retriever);
   ```

3. **Use absolute paths with forward slashes in XML test fixtures**: Relative mesh paths like `<file_name>mesh.obj</file_name>` resolve to `file:///mesh.obj` (no drive letter) on Windows. When writing paths into XML programmatically, convert backslashes to forward slashes:

   ```cpp
   std::string meshPathStr = meshPath.string();
   std::replace(meshPathStr.begin(), meshPathStr.end(), '\\', '/');
   // Then embed meshPathStr in the XML
   ```

4. **Use `std::filesystem::temp_directory_path()` instead of `/tmp`**: The `/tmp` path does not exist on Windows.

5. **Compare filesystem paths as paths, not raw strings**: Components may return equivalent Windows paths with different separators. Use `std::filesystem::equivalent()` when the file exists, or compare normalized `std::filesystem::path` values instead of expecting one separator style.

6. **Keep file-backed parser includes rooted in the filesystem**: Tests that write SDF or URDF files with relative includes should exercise local-file loading through the parser. The parser should resolve sibling includes from the parent directory on every platform; avoid masking failures by making the fixture absolute unless the behavior under test is unrelated to include resolution.

**Important:** Do NOT modify `Uri::fromPath()` in `uri.cpp` — the existing behavior (storing backslashes in the URI on Windows) is relied upon by `getFilesystemPath()` round-trip tests.

### Resource Retriever Failure Paths

When testing retrievers that cache remote resources, assert failed retrieval does not leave a cache entry that later becomes a false hit. This is especially important on Windows, where deleting an open file fails while POSIX systems allow unlinking it.

### Signal Slot Ordering Is Non-Deterministic

**Problem:** `dart::common::Signal` stores connections in a `std::set` with `owner_less` ordering. Tests that register multiple slots and assert a specific invocation order will pass on some platforms but fail on others.

**Solution:** When testing Signal behavior, either:

- Test with a **single slot** to avoid ordering assumptions
- Collect results from all slots into a container and check **set equality** (not sequence equality)

### Debug Assertions Crash on Invalid Access

**Problem:** Passing out-of-bounds indices or invalid arguments to DART APIs triggers `DART_ASSERT` in debug builds, which aborts the process rather than returning an error. Tests that intentionally call invalid APIs will crash (SEGFAULT/abort) instead of failing gracefully.

**Solution:** Do not write tests that intentionally trigger `DART_ASSERT`. Only test valid API usage. If testing error handling, test cases where the API returns errors or throws exceptions, not cases guarded by debug-only assertions.

### AddressSanitizer (ASan) Sensitivity in Allocator Tests

**Problem:** Tests exercising complex allocation/deallocation patterns (e.g., `PoolAllocator` edge cases like cross-block deallocation or block expansion) can trigger AddressSanitizer false positives in CI Release builds. These patterns may be valid C++ but ASan flags them as memory errors.

**Solution:** Keep allocator tests simple — test basic alloc/dealloc, null pointer handling, and diagnostics (e.g., `operator<<`). Avoid stress-testing internal memory block management, as these patterns are sensitive to ASan instrumentation.

### Numerical Test Tolerances Vary by Platform

**Problem:** Floating-point results can differ across platforms (x86 vs ARM64, different compilers, optimization levels). Tests with overly tight tolerances (e.g., `1e-6`) may pass locally but fail in CI on different architectures.

**Solution:** Use `EXPECT_NEAR` with tolerances appropriate for the computation:

- Simple arithmetic: `1e-12` to `1e-10`
- Single integration step: `1e-6` to `1e-4`
- Multi-step simulation: `1e-4` to `1e-2`
- When in doubt, start with `1e-6` and relax if CI shows platform-dependent failures

## Design Principles

The test suite follows these core principles:

1. **Prefer Simplicity Over Premature Generalization**
   - Flat helper structure (`helpers/`) not deeply nested
   - Add complexity only when pain points emerge

2. **Explicit Over Implicit**
   - No global helper includes
   - Tests declare exactly what they need
   - Clear dependencies

3. **Dependencies Drive Organization**
   - Helpers split by dependency weight (GTestUtils < common_helpers < dynamics_helpers)
   - Heavy helpers separated from lightweight ones
   - Pay compilation cost only for what you use

## Continuous Integration

Tests are automatically run on:

- Pull requests (before merging)
- Commits to main branches
- Scheduled nightly builds

All tests must pass before code can be merged.

## Debugging Tests

### Run a test in a debugger:

```bash
gdb ./tests/integration/collision/test_Collision
lldb ./tests/integration/collision/test_Collision
```

### Run a specific test case:

```bash
./tests/integration/collision/test_Collision --gtest_filter="CollisionTest.BoxBox"
```

### List all test cases without running:

```bash
./tests/integration/collision/test_Collision --gtest_list_tests
```

### Run with verbose output:

```bash
./tests/integration/collision/test_Collision --gtest_print_time=1
```

## Contributing

When contributing tests:

1. Follow the organizational structure outlined in this document
2. Add your test to the appropriate CMakeLists.txt
3. Ensure your test passes locally before submitting
4. Include test coverage for any new features or bug fixes
5. Update this README if you add new test categories or utilities

## Questions?

If you have questions about the test suite or where to add a new test, please:

- Check this README first
- Review existing tests in the same category
- Ask in the DART development forum or GitHub discussions
- Consult the main DART documentation at https://docs.dartsim.org/
