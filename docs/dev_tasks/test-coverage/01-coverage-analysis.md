# Coverage Analysis — Current State

## Test Suite Structure

| Category              | Files | Framework        |
| --------------------- | ----- | ---------------- |
| C++ Unit Tests        | 89    | GoogleTest       |
| C++ Integration Tests | 68    | GoogleTest       |
| C++ Benchmarks        | 11    | Google Benchmark |
| Python Tests          | 47    | pytest           |
| Source Files          | 761   | -                |

## Current Codecov Configuration

Located in `codecov.yml`:

```yaml
coverage:
  ignore:
    - "tests" # Correct - don't measure test coverage
    - "dart/external" # Correct - third-party code
    - "dart/gui" # Reasonable - hard to test in CI
```

## Coverage Gaps by Priority

### Critical (0% Coverage)

| Module         | Status               | Action                                             |
| -------------- | -------------------- | -------------------------------------------------- |
| `dart/sensor/` | Zero tests, NEW code | Create `test_Sensor.cpp`, `test_SensorManager.cpp` |

### High Priority (Implicit Coverage Only)

| Component      | Status            | Action                              |
| -------------- | ----------------- | ----------------------------------- |
| CapsuleShape   | No dedicated test | Port `test_SphereShape.cpp` pattern |
| ConeShape      | No dedicated test | Port `test_SphereShape.cpp` pattern |
| CylinderShape  | No dedicated test | Port `test_SphereShape.cpp` pattern |
| EllipsoidShape | No dedicated test | Port `test_SphereShape.cpp` pattern |
| PlaneShape     | No dedicated test | Port `test_SphereShape.cpp` pattern |

### Medium Priority (No Unit Tests)

| Component   | Status           | Action                  |
| ----------- | ---------------- | ----------------------- |
| EndEffector | Integration-only | Add isolated unit tests |
| Marker      | Integration-only | Add isolated unit tests |
| PointMass   | No tests         | Add unit tests          |

### Python Gaps

| File               | Status                     | Action               |
| ------------------ | -------------------------- | -------------------- |
| `test_skeleton.py` | Minimal (1 test, 24 lines) | Expand significantly |
| Python fixtures    | None                       | Create `fixtures.py` |

## What's Working Well

- `tests/helpers/GTestUtils.hpp` — Excellent Eigen comparison macros
- `tests/helpers/dynamics_helpers.hpp` — Good robot factory functions
- Monte Carlo testing patterns in `math/` tests
- Cross-backend collision verification

## Structural Issues

- Some tests in `integration/` test unit-level logic
- Monolithic tests (e.g., `test_Skeleton.cpp` restructuring: 200+ lines)
- Disabled tests with TODOs (Jacobian comparisons, serialization)

## Test Infrastructure

### C++ Helpers (in `tests/helpers/`)

| File                   | Purpose                  | Dependencies       |
| ---------------------- | ------------------------ | ------------------ |
| `GTestUtils.hpp`       | Eigen comparison macros  | `dart/math`, Eigen |
| `common_helpers.hpp`   | Resource retriever mocks | `dart/common`      |
| `dynamics_helpers.hpp` | Robot/skeleton factories | Full DART          |

### Python Infrastructure

| File          | Purpose                   |
| ------------- | ------------------------- |
| `conftest.py` | PYTHONPATH configuration  |
| `util.py`     | Asset path resolution     |
| (missing)     | Python skeleton factories |

## Recommended Exclusions (Additional)

| Path                              | Rationale               | Risk |
| --------------------------------- | ----------------------- | ---- |
| `dart/config.hpp`                 | CMake-generated         | Low  |
| `dart/simulation/experimental/**` | Unstable APIs           | Low  |
| `**/SmartPointer.hpp`             | Deprecated for DART 7.0 | Low  |

## Paths That Should NOT Be Excluded

| Path                               | Rationale                         |
| ---------------------------------- | --------------------------------- |
| `dart/collision/{bullet,ode,fcl}/` | Wrapper code written by DART      |
| `dart/utils/**`                    | Core parsing/loading code         |
| `dart/sensor/**`                   | Product code (currently untested) |
