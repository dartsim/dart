# DART Error Handling Modernization

This directory contains the planning and tracking documents for modernizing DART's error handling.

## Quick Links

| Document                                     | Description                      |
| -------------------------------------------- | -------------------------------- |
| [PLAN.md](PLAN.md)                           | Master plan with all 5 phases    |
| [MIGRATION_TRACKER.md](MIGRATION_TRACKER.md) | Progress tracking for all phases |
| [phase1/README.md](phase1/README.md)         | Phase 1 implementation details   |
| [phase2/README.md](phase2/README.md)         | Phase 2 planning and audit       |

## User Documentation

For user-facing error handling documentation, see:

- [docs/onboarding/error-handling.md](../../onboarding/error-handling.md)

## Implementation Files

Phase 1 created these files in `dart/common/`:

| File            | Description                              |
| --------------- | ---------------------------------------- |
| `Exception.hpp` | Exception hierarchy with source location |
| `Exception.cpp` | Error handler implementation             |
| `Result.hpp`    | Monadic Result<T,E> type                 |

## Status Summary

- **Phase 1**: Complete - Foundation infrastructure
- **Phase 2**: Planning - API boundary hardening
- **Phase 3**: Not started - Core module migration
- **Phase 4**: Not started - Parser migration
- **Phase 5**: Not started - Documentation completion

## Getting Started

To use the new error handling in DART code:

```cpp
#include <dart/common/Exception.hpp>
#include <dart/common/Result.hpp>

using namespace dart::common;

// Throw for programmer errors
DART_THROW_T_IF(index >= size, OutOfRangeException,
    "Index {} out of range [0, {})", index, size);

// Return Result for expected failures
Result<Skeleton*> loadSkeleton(const std::string& path) {
    if (!fileExists(path))
        return Result<Skeleton*>::err(Error("File not found"));
    return Result<Skeleton*>::ok(parseSkeleton(path));
}
```

## Contributing

When working on error handling migration:

1. Check [MIGRATION_TRACKER.md](MIGRATION_TRACKER.md) for current status
2. Follow patterns in phase documentation
3. Update tracker after completing items
4. Ensure backward compatibility
5. Add tests for new exception behavior
