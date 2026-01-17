# DART Error Handling Modernization Plan

**Status**: In Progress  
**Created**: 2026-01-17  
**Last Updated**: 2026-01-17

## Overview

This document outlines a multi-phase plan to modernize DART's error handling from a mixed legacy approach to a consistent, modern C++17/20 system. The goal is to provide:

1. **Consistency** - One clear pattern for each error category
2. **Configurability** - Support for exception-free builds (real-time/embedded)
3. **Debuggability** - Rich error context with source locations
4. **Compatibility** - Gradual migration without breaking existing code

## Current State Analysis

### Error Handling Mechanisms Found

| Mechanism                       | Usage Count | Primary Locations                 |
| ------------------------------- | ----------- | --------------------------------- |
| `DART_WARN/ERROR` + return      | 672+        | dynamics, collision, utils        |
| Exceptions (std::runtime_error) | ~52         | resource loading, Python bindings |
| `DART_ASSERT`                   | 357+        | invariant checks throughout       |
| Error codes (MJCF only)         | 1 module    | utils/mjcf                        |
| Modern exceptions               | 1 module    | simulation/experimental           |

### Key Problems

1. **Inconsistent API contracts** - Users don't know when to expect exceptions vs check returns
2. **Silent failures** - Many nullptr returns go unchecked
3. **No exception-free option** - Can't disable for real-time systems
4. **Poor debuggability** - Logging without stack traces
5. **Console spam** - Raw std::cout/cerr in production code

## Phase Overview

```
┌─────────────────────────────────────────────────────────────────┐
│  PHASE 1: Foundation (This PR)                                  │
│  - Port experimental exception system to dart/common/           │
│  - Create Result<T,E> type                                      │
│  - Document error handling guidelines                           │
│  - Add [[nodiscard]] infrastructure                             │
├─────────────────────────────────────────────────────────────────┤
│  PHASE 2: API Boundary Hardening                                │
│  - Audit public APIs for error contracts                        │
│  - Add exceptions for programmer errors at API boundaries       │
│  - Convert parsing APIs to use Result or exception              │
│  - Remove raw std::cout/cerr                                    │
├─────────────────────────────────────────────────────────────────┤
│  PHASE 3: Core Module Migration                                 │
│  - dynamics/ module migration                                   │
│  - collision/ module migration                                  │
│  - constraint/ module migration                                 │
├─────────────────────────────────────────────────────────────────┤
│  PHASE 4: I/O & Parser Migration                                │
│  - Unify URDF, SDF, SKEL, MJCF error handling                   │
│  - Implement batch error collection pattern                     │
│  - Add error recovery options                                   │
├─────────────────────────────────────────────────────────────────┤
│  PHASE 5: Documentation & Testing                               │
│  - Complete API documentation with error contracts              │
│  - Add error handling tests                                     │
│  - Performance benchmarks for exception overhead                │
└─────────────────────────────────────────────────────────────────┘
```

---

## Phase 1: Foundation

**Goal**: Establish the infrastructure for modern error handling without breaking existing code.

**Duration**: 1-2 weeks  
**Breaking Changes**: None  
**Files Created**:

- `dart/common/Exception.hpp` - Exception hierarchy
- `dart/common/Result.hpp` - Result<T,E> type
- `docs/onboarding/error-handling.md` - Guidelines

### 1.1 Exception System (dart/common/Exception.hpp)

Port and extend `dart/simulation/experimental/common/exceptions.hpp`:

```cpp
namespace dart::common {

/// Base exception with source location
class Exception : public std::runtime_error {
public:
    explicit Exception(
        const std::string& message,
        const std::source_location& location = std::source_location::current());

    [[nodiscard]] const std::string& message() const noexcept;
    [[nodiscard]] const std::source_location& location() const noexcept;
};

// Derived types
class InvalidArgumentException : public Exception { using Exception::Exception; };
class OutOfRangeException : public Exception { using Exception::Exception; };
class NullPointerException : public Exception { using Exception::Exception; };
class NotImplementedException : public Exception { using Exception::Exception; };
class InvalidOperationException : public Exception { using Exception::Exception; };
class FileNotFoundException : public Exception { using Exception::Exception; };
class ParseException : public Exception { using Exception::Exception; };

// Custom error handler for exception-free mode
using ErrorHandler = void (*)(const char* type, const char* msg, const std::source_location&);
void setErrorHandler(ErrorHandler handler);
ErrorHandler getErrorHandler();

} // namespace dart::common

// Macros (controlled by DART_DISABLE_EXCEPTIONS)
#define DART_THROW(...)
#define DART_THROW_IF(condition, ...)
#define DART_THROW_T(ExceptionType, ...)
#define DART_THROW_T_IF(condition, ExceptionType, ...)
```

### 1.2 Result Type (dart/common/Result.hpp)

A monadic result type for expected failures:

```cpp
namespace dart::common {

/// Error information for Result failures
struct Error {
    std::string message;
    std::source_location location;

    Error(std::string msg,
          std::source_location loc = std::source_location::current());
};

/// Result type for operations that may fail
template<typename T, typename E = Error>
class Result {
public:
    // Construction
    static Result ok(T value);
    static Result err(E error);

    // Queries
    [[nodiscard]] bool isOk() const noexcept;
    [[nodiscard]] bool isErr() const noexcept;
    explicit operator bool() const noexcept;

    // Access (throws if wrong state)
    [[nodiscard]] T& value() &;
    [[nodiscard]] const T& value() const &;
    [[nodiscard]] T&& value() &&;
    [[nodiscard]] E& error() &;
    [[nodiscard]] const E& error() const &;

    // Safe access
    [[nodiscard]] T valueOr(T defaultValue) const;

    // Monadic operations
    template<typename F> auto map(F&& f) -> Result<decltype(f(value())), E>;
    template<typename F> auto andThen(F&& f) -> decltype(f(value()));
    template<typename F> auto orElse(F&& f) -> Result<T, decltype(f(error()))>;

private:
    std::variant<T, E> storage_;
};

// Void specialization
template<typename E>
class Result<void, E>;

} // namespace dart::common
```

### 1.3 Error Handling Guidelines

Create comprehensive documentation at `docs/onboarding/error-handling.md`.

### 1.4 Deliverables Checklist

- [ ] `dart/common/Exception.hpp` - Exception hierarchy
- [ ] `dart/common/detail/Exception-impl.hpp` - Implementation
- [ ] `dart/common/Result.hpp` - Result type
- [ ] `dart/common/detail/Result-impl.hpp` - Implementation
- [ ] `tests/unit/common/test_Exception.cpp` - Exception tests
- [ ] `tests/unit/common/test_Result.cpp` - Result tests
- [ ] `docs/onboarding/error-handling.md` - Guidelines
- [ ] Update `dart/common/All.hpp` to include new headers

---

## Phase 2: API Boundary Hardening

**Goal**: Establish clear error contracts at public API boundaries.

**Duration**: 2-3 weeks  
**Breaking Changes**: Minimal (new exceptions where nullptr was returned)

### 2.1 Audit Public APIs

Document and enforce error contracts for:

- `Skeleton::create()` and related factory methods
- `World::addSkeleton()`, `World::removeSkeleton()`
- All parser entry points (`readSkeleton`, `readWorld`)
- `CollisionDetector::create()` methods

### 2.2 Remove Console Spam

Replace all raw `std::cout`/`std::cerr` with `DART_DEBUG`/`DART_INFO`:

- ~91 instances in dart/ directory
- Focus on constraint solver debug output

### 2.3 Deliverables Checklist

- [ ] `docs/dev_tasks/error_handling/API_CONTRACTS.md` - API error contracts
- [ ] Remove all raw std::cout/cerr from dart/
- [ ] Add `[[nodiscard]]` to factory methods
- [ ] Add null-check exceptions to public factory methods

---

## Phase 3: Core Module Migration

**Goal**: Migrate core physics modules to new error handling.

**Duration**: 4-6 weeks  
**Breaking Changes**: Some APIs will throw where they previously returned nullptr

### 3.1 Migration Priority

1. **dart/dynamics/** - Most user-facing, highest impact
2. **dart/collision/** - Used by all simulations
3. **dart/constraint/** - Internal, lower risk

### 3.2 Migration Pattern

```cpp
// Before
BodyNode* Skeleton::getBodyNode(std::size_t index) {
    if (index >= mBodyNodes.size()) {
        DART_ERROR("Index {} out of range [0, {})", index, mBodyNodes.size());
        return nullptr;
    }
    return mBodyNodes[index];
}

// After (throwing version for API boundary)
BodyNode* Skeleton::getBodyNode(std::size_t index) {
    DART_THROW_T_IF(
        index >= mBodyNodes.size(),
        OutOfRangeException,
        "Index {} out of range [0, {})", index, mBodyNodes.size());
    return mBodyNodes[index];
}

// Or (Result version for operations that commonly fail)
Result<BodyNode*> Skeleton::tryGetBodyNode(std::size_t index) {
    if (index >= mBodyNodes.size()) {
        return Result<BodyNode*>::err(
            Error("Index out of range", std::source_location::current()));
    }
    return Result<BodyNode*>::ok(mBodyNodes[index]);
}
```

### 3.3 Deliverables Checklist

- [ ] `docs/dev_tasks/error_handling/MIGRATION_dynamics.md`
- [ ] `docs/dev_tasks/error_handling/MIGRATION_collision.md`
- [ ] `docs/dev_tasks/error_handling/MIGRATION_constraint.md`
- [ ] Migrate dynamics module
- [ ] Migrate collision module
- [ ] Migrate constraint module

---

## Phase 4: I/O & Parser Migration

**Goal**: Unify error handling across all file parsers.

**Duration**: 3-4 weeks  
**Breaking Changes**: Parser functions may throw or return Result

### 4.1 Parser Unification

All parsers should follow MJCF pattern with accumulated errors:

```cpp
struct ParseResult {
    std::shared_ptr<Skeleton> skeleton;  // May be partial
    std::vector<ParseError> errors;
    std::vector<ParseWarning> warnings;

    [[nodiscard]] bool hasErrors() const;
    [[nodiscard]] bool hasWarnings() const;
    explicit operator bool() const;  // true if no errors
};

ParseResult readSkeleton(const Uri& uri, const ResourceRetrieverPtr& retriever);
```

### 4.2 Deliverables Checklist

- [ ] `dart/io/ParseResult.hpp` - Unified parse result type
- [ ] Migrate URDF parser
- [ ] Migrate SDF parser
- [ ] Migrate SKEL parser
- [ ] Extend MJCF parser pattern
- [ ] Update all examples and tests

---

## Phase 5: Documentation & Testing

**Goal**: Complete documentation and ensure reliability.

**Duration**: 2 weeks  
**Breaking Changes**: None

### 5.1 Documentation

- Complete Doxygen comments with `@throws` specifications
- Add error handling section to user guide
- Update all tutorials with proper error handling

### 5.2 Testing

- Unit tests for all exception types
- Integration tests for error propagation
- Performance benchmarks comparing exception vs return-code paths

### 5.3 Deliverables Checklist

- [ ] All public APIs documented with `@throws`
- [ ] `docs/tutorials/error-handling.md`
- [ ] Performance benchmark results
- [ ] CI checks for exception safety

---

## Success Criteria

### Quantitative

| Metric                                           | Current | Target |
| ------------------------------------------------ | ------- | ------ |
| Raw std::cout/cerr in dart/                      | 91      | 0      |
| APIs with documented error contracts             | ~5%     | 100%   |
| Functions with `[[nodiscard]]` where appropriate | ~10%    | 90%    |
| Exception-throwing APIs with tests               | ~5%     | 100%   |

### Qualitative

- [ ] Any DART user can determine error handling contract from header
- [ ] Exception-free build mode works for real-time applications
- [ ] Error messages include source location for debugging
- [ ] Consistent pattern across all modules

---

## Related Documents

- [Phase 1 Implementation](./phase1/README.md)
- [Phase 2 Implementation](./phase2/README.md)
- [API Error Contracts](./API_CONTRACTS.md)
- [Migration Tracking](./MIGRATION_TRACKER.md)

---

## Appendix: Decision Log

### Why not just use exceptions everywhere?

1. **Real-time constraints** - Physics simulation in robotics often has hard real-time requirements
2. **Embedded targets** - Some platforms don't support exceptions efficiently
3. **Python interop** - Need to translate to Python exceptions anyway
4. **Performance** - Exception path is slow; hot paths need predictable performance

### Why not just use error codes everywhere?

1. **Ergonomics** - Error codes are easy to ignore
2. **Context loss** - Hard to propagate rich error information
3. **Modern C++** - Exceptions are idiomatic for programmer errors

### Why Result<T,E> in addition to exceptions?

- Exceptions for **programmer errors** (null pointer, out of range)
- Result for **expected failures** (file not found, parse error)
- Clear semantic distinction guides API design
