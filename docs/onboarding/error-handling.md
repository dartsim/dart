# Error Handling in DART

This guide describes DART's error handling philosophy, APIs, and best practices.

## Overview

DART uses a **dual-strategy** error handling approach:

| Strategy        | Use Case                                         | Mechanism                           |
| --------------- | ------------------------------------------------ | ----------------------------------- |
| **Exceptions**  | Programmer errors (invariant violations)         | `dart::common::Exception` hierarchy |
| **Result type** | Expected failures (file not found, parse errors) | `dart::common::Result<T,E>`         |

## Quick Reference

```cpp
#include <dart/common/Exception.hpp>
#include <dart/common/Result.hpp>

using namespace dart::common;

// Throw for programmer errors
DART_THROW_T_IF(ptr == nullptr, NullPointerException, "Skeleton pointer is null");
DART_THROW_T_IF(index >= size, OutOfRangeException, "Index {} >= size {}", index, size);

// Return Result for expected failures
Result<Skeleton*> loadSkeleton(const std::string& path) {
    if (!fileExists(path))
        return Result<Skeleton*>::err(Error("File not found: " + path));
    return Result<Skeleton*>::ok(parseSkeleton(path));
}
```

## Exception Types

| Exception                   | When to Use                          |
| --------------------------- | ------------------------------------ |
| `Exception`                 | Generic error (base class)           |
| `InvalidArgumentException`  | Caller passed invalid arguments      |
| `OutOfRangeException`       | Index or iterator out of bounds      |
| `NullPointerException`      | Null pointer where non-null required |
| `NotImplementedException`   | Feature not yet implemented          |
| `InvalidOperationException` | Operation invalid in current state   |
| `FileNotFoundException`     | File or resource not found           |
| `ParseException`            | Error parsing file format            |

## Exception Macros

```cpp
// Unconditional throw
DART_THROW("Something went wrong");
DART_THROW_T(InvalidArgumentException, "Value {} is invalid", value);

// Conditional throw (if condition is TRUE)
DART_THROW_IF(x < 0, "x must be non-negative, got {}", x);
DART_THROW_T_IF(ptr == nullptr, NullPointerException, "ptr is null");
```

All macros support `std::format()` style formatting.

## Result Type

Use `Result<T, E>` when failure is **expected** and the caller should handle it:

```cpp
Result<Skeleton*> result = loadSkeleton("robot.urdf");

// Check success
if (result.isOk()) {
    Skeleton* skel = result.value();
}

// Or use operator bool
if (result) {
    Skeleton* skel = *result;  // operator* also works
}

// Access error on failure
if (result.isErr()) {
    std::cerr << result.error().message << std::endl;
}

// Default value
Skeleton* skel = result.valueOr(nullptr);

// Monadic operations
auto name = loadSkeleton("robot.urdf")
    .map([](Skeleton* s) { return s->getName(); })
    .valueOr("unknown");
```

## When to Use What

| Situation                   | Approach            | Example                                                      |
| --------------------------- | ------------------- | ------------------------------------------------------------ |
| Null pointer passed         | Exception           | `DART_THROW_T_IF(ptr == nullptr, NullPointerException, ...)` |
| Index out of bounds         | Exception           | `DART_THROW_T_IF(index >= size, OutOfRangeException, ...)`   |
| File not found              | Result              | `return Result<T>::err(Error("File not found"))`             |
| Parse error                 | Result or Exception | Result if recovery possible, Exception otherwise             |
| Internal invariant violated | `DART_ASSERT`       | Debug-only check                                             |

## Exception-Free Mode

For real-time or embedded systems, DART can be built without exceptions:

```bash
cmake -DDART_DISABLE_EXCEPTIONS=ON ..
```

In this mode:

1. `DART_THROW*` macros log the error and call `std::terminate()`
2. You can set a custom handler to intercept errors:

```cpp
dart::common::setErrorHandler([](const char* type, const char* msg, auto loc) {
    // Log to your system before terminate
    myLogger.fatal("[{}] {} at {}:{}", type, msg, loc.file_name(), loc.line());

    // Flush buffers, send telemetry, etc.
    // Note: Handler cannot prevent termination - it's called BEFORE std::terminate()
    flushLogs();
});
```

## Assertions

Use assertions for debug-time invariant checking:

```cpp
#include <dart/common/Macros.hpp>

DART_ASSERT(mBodyNodes.size() > 0);
DART_ASSERT(mParentJoint != nullptr, "Parent joint must be set");
```

Assertions are **disabled in release builds** (`NDEBUG` defined).

## Logging

For non-fatal issues, use logging instead of exceptions:

```cpp
#include <dart/common/Logging.hpp>

DART_WARN("Deprecated function called, use newFunction() instead");
DART_ERROR("Failed to load texture, using default");
DART_INFO("Loaded {} bodies from file", count);
```

## Best Practices

1. **Be specific**: Use typed exceptions (`OutOfRangeException`) not generic ones
2. **Include context**: Format messages with relevant values
3. **Fail early**: Check preconditions at function entry
4. **Document contracts**: Specify what exceptions a function may throw
5. **Never ignore Result**: The `[[nodiscard]]` attribute enforces this

## Migration from Legacy Code

Legacy DART code used logging + return nullptr/false:

```cpp
// Legacy pattern (avoid in new code)
if (index >= mBodyNodes.size()) {
    DART_ERROR("Index out of range");
    return nullptr;
}

// Modern pattern
DART_THROW_T_IF(index >= mBodyNodes.size(), OutOfRangeException,
    "Index {} out of range [0, {})", index, mBodyNodes.size());
```

Migration status: DART now treats programmer errors with typed exceptions and expected failures with `Result<T, E>`, replacing legacy log-and-return patterns. The source of truth is the implementation in `dart/common/` and the updated call sites across IO and parsing modules. Keep new work aligned with this dual strategy and avoid introducing new logging-only error paths.

## Numerical Validation Policy

DART's dynamics pipeline uses `DART_ASSERT` for debug-only invariant checks. Some of these guards validate data that originates from user input (model files, API calls) and can legitimately contain non-finite values (NaN, infinity, overflow). When such data reaches a `DART_ASSERT`, it crashes the process in debug builds — unacceptable for simulation loops where a single bad model should not bring down the entire application.

This section defines when to use `DART_ASSERT` versus `DART_WARN` for numerical validation.

### Decision Tree

Classify each numerical check into one of three categories:

| Category               | Location                                                                                                             | Data Source                     | Macro                           | Recovery                     |
| ---------------------- | -------------------------------------------------------------------------------------------------------------------- | ------------------------------- | ------------------------------- | ---------------------------- |
| **A — Public API**     | Setters like `Joint::setTransformFromParentBodyNode`                                                                 | Direct user input               | `DART_WARN`                     | Reject value, keep previous  |
| **B — Per-joint math** | Joint-type-specific algorithms (e.g., `RevoluteJoint::getRelativeJacobian`)                                          | Computed from validated inputs  | `DART_ASSERT`                   | None (true invariant)        |
| **C-upstream**         | Internal functions that depend on user data (e.g., `BodyNode::updateTransform`, `GenericJoint::addChildArtInertia*`) | Derived from user poses/inertia | `DART_WARN` or `DART_WARN_ONCE` | Use identity/zero fallback   |
| **C-downstream**       | Deep pipeline functions protected by upstream guards                                                                 | Already validated               | `DART_ASSERT`                   | None (protected by upstream) |

### Category A — Public API Setters

```cpp
// Example: Joint::setTransformFromParentBodyNode
if (!math::verifyTransform(_T)) {
  DART_WARN("Non-finite transform rejected in setTransformFromParentBodyNode");
  return;  // Keep previous value
}
```

**Rationale**: Users can pass anything through the public API. Gazebo's SDF parser may produce `1e308` values that overflow to infinity during pose composition. Reject silently with a warning.

### Category B — Algorithmic Invariants

```cpp
// Example: Inside RevoluteJoint::getRelativeJacobian
DART_ASSERT(math::verifyTransform(result));
```

**Rationale**: If inputs are valid (enforced by Category A/C guards), the algorithm should produce valid outputs. A failure here indicates a genuine bug.

### Category C-upstream — Defense-in-Depth

```cpp
// Example: BodyNode::updateTransform
if (!math::verifyTransform(mT)) {
  DART_WARN("Non-finite world transform in BodyNode::updateTransform. "
            "Using identity.");
  mT.setIdentity();
}
```

**Rationale**: Even with public API guards, overflow can emerge during dynamics computation (e.g., `transformInertia()` producing infinity from large-but-finite inputs). These guards prevent cascading NaN propagation through the simulation. Use `DART_WARN_ONCE` when the check is in a hot loop.

### Parsers (MJCF, URDF, SDF)

Parser validation of transforms from model files follows Category A rules: validate and warn, never assert. Model files are external user data.

```cpp
// Example: MJCF body parser
if (!math::verifyRotation(R)) {
  DART_WARN("Non-finite rotation in MJCF body. Using identity.");
  R = Eigen::Matrix3d::Identity();
}
```

### When to Add New Guards

When adding numerical assertions to dynamics code, ask:

1. **Can this value trace back to user input** (model files, API calls, external forces)? → Category A or C-upstream: use `DART_WARN`
2. **Is this a pure algorithmic result from validated inputs?** → Category B: use `DART_ASSERT`
3. **Is this deep in the pipeline with upstream guards already in place?** → Category C-downstream: use `DART_ASSERT`

### Key Files

- `dart/common/Macros.hpp` — `DART_ASSERT` definition (wraps `assert()`)
- `dart/common/Logging.hpp` — `DART_WARN`, `DART_WARN_ONCE`, `DART_ERROR`
- `dart/math/geometry.cpp` — `math::verifyTransform()`, `math::verifyRotation()`
- `dart/math/helpers.hpp` — `math::isNan()`, `math::isInf()`
