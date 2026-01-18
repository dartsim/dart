# Phase 1: Foundation Implementation

**Status**: Complete  
**Duration**: Initial implementation

## Overview

Phase 1 establishes the foundational infrastructure for modern error handling in DART without breaking existing code.

## Files Created

### Core Headers

| File                        | Description                               |
| --------------------------- | ----------------------------------------- |
| `dart/common/Exception.hpp` | Exception hierarchy with source location  |
| `dart/common/Exception.cpp` | Error handler implementation              |
| `dart/common/Result.hpp`    | Monadic result type for expected failures |

### Documentation

| File                                    | Description                           |
| --------------------------------------- | ------------------------------------- |
| `docs/onboarding/error-handling.md`     | User-facing error handling guidelines |
| `docs/dev_tasks/error_handling/PLAN.md` | Multi-phase migration plan            |

## Exception Hierarchy

```
dart::common::Exception (base)
├── InvalidArgumentException  - Bad arguments from caller
├── OutOfRangeException       - Index/iterator bounds violation
├── NullPointerException      - Null where non-null required
├── NotImplementedException   - Unimplemented functionality
├── InvalidOperationException - Invalid operation for current state
├── FileNotFoundException     - File/resource not found
└── ParseException            - File format parsing errors
```

## Usage Examples

### Throwing Exceptions

```cpp
#include <dart/common/Exception.hpp>

void setIndex(int index, int size) {
    // Using macro (recommended)
    DART_THROW_T_IF(index < 0 || index >= size, OutOfRangeException,
        "Index {} out of range [0, {})", index, size);

    // Or explicit throw
    if (index < 0 || index >= size) {
        throw dart::common::OutOfRangeException(
            std::format("Index {} out of range [0, {})", index, size));
    }
}
```

### Using Result Type

```cpp
#include <dart/common/Result.hpp>

using namespace dart::common;

Result<Skeleton*> loadSkeleton(const std::string& path) {
    auto file = openFile(path);
    if (!file) {
        return Result<Skeleton*>::err(Error("File not found: " + path));
    }

    auto skeleton = parseFile(file);
    if (!skeleton) {
        return Result<Skeleton*>::err(Error("Parse failed"));
    }

    return Result<Skeleton*>::ok(skeleton);
}

void usage() {
    auto result = loadSkeleton("robot.urdf");

    // Option 1: Check and access
    if (result) {
        auto* skel = result.value();
    } else {
        std::cerr << result.error().message << std::endl;
    }

    // Option 2: Default value
    auto* skel = result.valueOr(nullptr);

    // Option 3: Monadic chaining
    auto name = loadSkeleton("robot.urdf")
        .map([](Skeleton* s) { return s->getName(); })
        .valueOr("unknown");
}
```

### Exception-Free Mode

```cpp
// Build with -DDART_DISABLE_EXCEPTIONS=ON

// Set custom handler before any DART operations
dart::common::setErrorHandler([](const char* type, const char* msg, auto loc) {
    myLogger.fatal("[{}] {} at {}:{}", type, msg, loc.file_name(), loc.line());
    // Optionally attempt recovery or controlled shutdown
});
```

## Integration Notes

### CMake Changes Required

Add to `dart/common/CMakeLists.txt`:

```cmake
set(DART_COMMON_SOURCES
  # ... existing sources ...
  Exception.cpp
)

set(DART_COMMON_HEADERS
  # ... existing headers ...
  Exception.hpp
  Result.hpp
)
```

### Compile-Time Configuration

| Define                    | Effect                                           |
| ------------------------- | ------------------------------------------------ |
| (default)                 | Exceptions enabled, throws on error              |
| `DART_DISABLE_EXCEPTIONS` | Exception-free mode, calls handler or terminates |

## Testing

Tests should be added to `tests/unit/common/`:

- `test_Exception.cpp` - Exception throwing and catching
- `test_Result.cpp` - Result type operations

## Migration Path

This phase is **non-breaking**. Existing code continues to work unchanged. New code can optionally use the new error handling infrastructure.

Next phase (Phase 2) will audit public APIs and establish error contracts.
