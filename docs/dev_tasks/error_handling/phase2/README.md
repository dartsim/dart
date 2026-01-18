# Phase 2: API Boundary Hardening

**Status**: Planning  
**Prerequisites**: Phase 1 Complete

## Objectives

1. Audit all public APIs and document error contracts
2. Remove raw `std::cout`/`std::cerr` from library code
3. Add `[[nodiscard]]` to factory methods returning pointers/optionals
4. Add exceptions for programmer errors at API boundaries

## Audit Scope

### Priority 1: Factory Methods (Most User-Facing)

| Module       | Function                      | Current Behavior       | Target                              |
| ------------ | ----------------------------- | ---------------------- | ----------------------------------- |
| `dynamics`   | `Skeleton::create()`          | Returns shared_ptr     | Add null check on name conflict     |
| `dynamics`   | `BodyNode::create*()`         | Returns raw/shared ptr | Document or throw on invalid parent |
| `simulation` | `World::create()`             | Returns shared_ptr     | Ok as-is                            |
| `collision`  | `CollisionDetector::create()` | Returns shared_ptr     | Ok as-is                            |

### Priority 2: Parser Entry Points

| Parser | Function         | Current Behavior       | Target                       |
| ------ | ---------------- | ---------------------- | ---------------------------- |
| URDF   | `readSkeleton()` | Returns nullptr + logs | Return `Result<SkeletonPtr>` |
| SDF    | `readSkeleton()` | Returns nullptr + logs | Return `Result<SkeletonPtr>` |
| SKEL   | `readSkeleton()` | Returns nullptr + logs | Return `Result<SkeletonPtr>` |
| MJCF   | `readSkeleton()` | Returns errors vector  | Already good pattern         |

### Priority 3: World/Skeleton Operations

| Class      | Method               | Current Behavior          | Target                       |
| ---------- | -------------------- | ------------------------- | ---------------------------- |
| `World`    | `addSkeleton()`      | Logs warning on nullptr   | Throw `NullPointerException` |
| `World`    | `removeSkeleton()`   | Logs warning on not found | Return bool or throw         |
| `Skeleton` | `getBodyNode(index)` | Logs + returns nullptr    | Throw `OutOfRangeException`  |
| `Skeleton` | `getBodyNode(name)`  | Returns nullptr           | Ok (name lookup can fail)    |

## Console Spam Removal

### Files with raw std::cout/std::cerr

```
dart/constraint/ConstraintSolver.cpp - 35 instances (debug output)
dart/constraint/JointLimitConstraint.cpp - 5 instances
dart/constraint/SoftContactConstraint.cpp - 21 instances (commented)
dart/dynamics/EulerJoint.cpp - 6 instances (singular config warnings)
dart/dynamics/SoftBodyNode.cpp - 2 instances
dart/utils/urdf/urdf_world_parser.cpp - 5 instances
dart/common/VersionCounter.cpp - 2 instances
dart/common/Stopwatch.cpp - 4 instances (intentional print functions)
```

### Action Items

1. **ConstraintSolver.cpp**: Wrap debug output in `DART_DEBUG` or `#ifndef NDEBUG`
2. **EulerJoint.cpp**: Convert singular config warnings to `DART_WARN`
3. **urdf_world_parser.cpp**: Convert to `DART_INFO`/`DART_WARN`
4. **Stopwatch.cpp**: Keep as-is (these are intentional print functions)

## [[nodiscard]] Additions

Add `[[nodiscard]]` to functions where ignoring return value is likely a bug:

```cpp
// Functions returning error indicators
[[nodiscard]] bool addSkeleton(SkeletonPtr skel);

// Factory methods
[[nodiscard]] static SkeletonPtr create(const std::string& name);

// Functions returning Result
[[nodiscard]] Result<SkeletonPtr> readSkeleton(const Uri& uri);
```

## API Contract Documentation

Each public API should document:

1. **Preconditions**: What must be true before calling
2. **Postconditions**: What will be true after successful call
3. **Exceptions**: What exceptions may be thrown and when
4. **Return value**: What nullptr/empty/Result::err means

Example documentation:

```cpp
/// @brief Get the BodyNode at the specified index.
/// @param[in] index Index of the BodyNode [0, getNumBodyNodes())
/// @return Pointer to the BodyNode
/// @throws OutOfRangeException if index >= getNumBodyNodes()
/// @note Use tryGetBodyNode() for non-throwing version
BodyNode* getBodyNode(std::size_t index);
```

## Deliverables Checklist

- [ ] `docs/dev_tasks/error_handling/API_CONTRACTS.md` - Full API contract documentation
- [ ] Remove/replace all raw std::cout/cerr in dart/ (except Stopwatch print functions)
- [ ] Add `[[nodiscard]]` to ~50 key functions
- [ ] Add exception throwing to null-sensitive APIs
- [ ] Update all affected tests

## Breaking Changes Assessment

| Change                   | Impact | Mitigation                                  |
| ------------------------ | ------ | ------------------------------------------- |
| New exceptions thrown    | Medium | Callers must add try-catch or let propagate |
| `[[nodiscard]]` warnings | Low    | Callers can cast to void if intentional     |
| Console output reduction | None   | Positive change                             |

## Testing Requirements

1. Test that exceptions are thrown for documented conditions
2. Test that `[[nodiscard]]` warnings appear when expected
3. Verify no regression in existing functionality
