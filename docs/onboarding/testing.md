# DART Test Suite

This directory contains the complete test suite for DART (Dynamic Animation and Robotics Toolkit). The tests are organized by type and module to facilitate easy navigation, maintenance, and scalability.

## Start here next time

- Local build/test workflow: [building.md](building.md)
- CI monitoring and expectations: [ci-cd.md](ci-cd.md)
- Gazebo / gz-physics integration: [build-system.md](build-system.md#gazebo-integration-feature)

## Fast Iteration Loop

Smallest repeatable local loop before a full CI run.

Choose a parallelism cap around two-thirds of logical cores, then set `DART_PARALLEL_JOBS` and `CTEST_PARALLEL_LEVEL` to that value (see [building.md](building.md) for details).

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
cmake --build build/default/cpp/Release --target UNIT_gui_MeshShapeNodeMaterialUpdates
ctest --test-dir build/default/cpp/Release --output-on-failure -R UNIT_gui_MeshShapeNodeMaterialUpdates
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

Full validation.

Suggested (Unverified):

```bash
N=$(( ( $(nproc) * 2 ) / 3 ))
DART_PARALLEL_JOBS=$N CTEST_PARALLEL_LEVEL=$N pixi run test-all
DART_PARALLEL_JOBS=$N CTEST_PARALLEL_LEVEL=$N pixi run -e gazebo test-gz
```

Signals to look for:

- The full test run ends with `✓ All tests passed!`
- The Gazebo integration workflow prints `✓ DART plugin built successfully with DART integration!`

## Gotchas

- The first lint run can take a while and it runs auto-fixers; expect diffs, rerun if interrupted, and check `git status` before committing.
- `pixi run test-all` runs linting and documentation builds as part of the suite; expect longer runtime and potential formatting diffs, so review changes before committing.
- `pixi run lint` can rewrite identifiers via codespell; if a spelling or casing is intentional, add it to `.codespellrc` and re-run lint.
- If pixi reports a missing task or feature, your `pixi.toml` may be out of sync with the target branch; update it before retrying.

## Next-Time Accelerators

- Run lint early (and again after test-all if it reformats) to surface formatting/codespell changes before longer build/test cycles.

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
│   ├── gui/                 # Tests for OSG nodes and ImGui integration
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
- **gui/**: OSG nodes and ImGui integration tests
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

### Regression Tests (Migrated)

> **Note:** The `tests/regression/` directory has been **completely removed**. All issue-based regression tests have been renamed with descriptive names and placed in appropriate unit/ or integration/ directories.

Previously, all issue-based regression tests lived in a separate `tests/regression/` directory with names like `test_Issue1234.cpp`. They have now been:

1. **Renamed** with descriptive names that indicate what they test (e.g., `test_CollisionAccuracy.cpp`, `test_SkeletonState.cpp`)
2. **Distributed** to appropriate module directories based on their test type

**Migration examples:**

- `test_Issue1184.cpp` → `integration/collision/test_CollisionAccuracy.cpp`
- `test_Issue1243.cpp` → `integration/dynamics/test_SkeletonState.cpp`
- `test_Issue986.cpp` → `unit/dynamics/test_CreateShapeNodeApi.cpp`

**Rationale for migration:**

- Descriptive test names improve code readability and maintainability
- Organizing by module makes related tests easier to find
- Eliminates redundant categorization (regression vs unit/integration)
- Issue references are preserved in comments within each test file
- Follows the principle: organize by WHAT you test, not WHY it was written

### Benchmarks (`benchmark/`)

Performance benchmarks measure execution time and resource usage:

- **collision/**: Collision detection performance with various shapes and scenarios
- **dynamics/**: Kinematics computation performance
- **integration/**: End-to-end system performance
- **unit/**: Individual component performance

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

```bash
ctest -L integration
ctest -L unit
ctest -L regression
```

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
pixi run bm --pixi-help
```

> **Note:** LCP solver comparisons use the solver-agnostic harness and the
> `BM_LCP_COMPARE` benchmark so all solvers share the same contract and fixtures.
> See `tests/common/lcpsolver` and `tests/benchmark/lcpsolver` for the sources,
> and keep benchmark outputs under the build tree.

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

## Coverage Patterns

Areas that commonly need additional test coverage (identified from test-coverage-audit):

| Pattern                | What to Test                                                         | Example                                               |
| ---------------------- | -------------------------------------------------------------------- | ----------------------------------------------------- |
| **Smart pointers**     | Lifecycle, expiration, owner lifetime                                | `BodyNodePtr`, `NodePtr`, `WeakPtr` variants          |
| **Aspect system**      | `has<T>()`, `get<T>()`, `removeAspect<T>()`, `isSpecializedFor<T>()` | Composite template methods                            |
| **Constraint solvers** | Empty inputs, duplicate skeletons, timestep changes, null detectors  | `ConstraintSolver` edge cases                         |
| **Collision filters**  | Self-collision, blacklist edge cases, composite behavior             | `CollisionFilter` combinations                        |
| **Template classes**   | All template instantiations, not just common types                   | `TemplateBodyNodePtr<BodyNode>` vs `<const BodyNode>` |

**Key insight**: Template classes and smart pointer wrappers often have lower coverage because tests only exercise common instantiations. Add explicit tests for const variants, weak references, and edge cases like expired pointers.

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
