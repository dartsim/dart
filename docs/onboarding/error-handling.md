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
    // Log to your system
    myLogger.fatal("[{}] {} at {}:{}", type, msg, loc.file_name(), loc.line());

    // Optionally attempt recovery or controlled shutdown
    gracefulShutdown();
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

See [docs/dev_tasks/error_handling/PLAN.md](../dev_tasks/error_handling/PLAN.md) for the full migration plan.
