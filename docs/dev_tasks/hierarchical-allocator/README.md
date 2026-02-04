# Hierarchical Allocator: World-Level Memory Management

**Status**: IN PROGRESS (Phase 4 partially applied, code does not compile)
**Branch**: working tree on `main` (no commits yet)
**Started**: Previous session
**Goal**: World owns MemoryManager → provides tailored allocators to components

## Architecture (Agreed Design)

```
WorldConfig { MemoryAllocator* baseAllocator = nullptr; }
    │
    ▼
World always creates MemoryManager(baseAllocator ?: GetDefault())
    │
    ├── MemoryManager owns:
    │   ├── FreeListAllocator(baseAllocator)     [existing]
    │   │     └── PoolAllocator(freeListAlloc)   [existing]
    │   └── FrameAllocator(baseAllocator)        [NEW - sibling to FreeList]
    │
    ├── ConstraintSolver borrows FrameAllocator* from MemoryManager
    │   └── Uses for per-step constraint allocation via FrameStlAllocator<T>
    │
    └── Skeleton borrows FreeListAllocator& from MemoryManager
        └── BodyNodePool uses it as base for chunk allocation
```

**Key principles:**

- `World::getMemoryManager()` always returns non-null reference
- Components BORROW allocators (don't own them)
- Cloned World/Skeleton get their own fresh allocators (same base allocator, independent state)
- Without World, components fall back to default behavior

## Completed Work (Phases 1-3)

### New Files (untracked)

| File                                                | Purpose                                                | Tests                 |
| --------------------------------------------------- | ------------------------------------------------------ | --------------------- |
| `dart/common/frame_allocator.hpp`                   | Bump allocator inheriting `MemoryAllocator`            | 9 unit tests          |
| `dart/common/frame_allocator.cpp`                   | Implementation (uses baseAllocator for backing memory) | ↑                     |
| `dart/dynamics/detail/body_node_pool.hpp`           | Per-Skeleton contiguous pool, 32-byte aligned          | 6 unit tests          |
| `tests/unit/common/test_frame_allocator.cpp`        | FrameAllocator unit tests                              | registered ✓          |
| `tests/unit/dynamics/test_body_node_pool.cpp`       | BodyNodePool unit tests                                | **NOT registered** ⚠️ |
| `tests/benchmark/dynamics/bm_dynamics_cache.cpp`    | Comprehensive dynamics benchmarks                      | built ✓               |
| `tests/benchmark/dynamics/bm_dynamics_cache_io.cpp` | Real robot model benchmarks (needs dart-io)            | built ✓               |
| `docs/design/hierarchical_allocator.md`             | Design doc (outdated, predates current design)         | —                     |
| `scripts/compare_benchmarks.py`                     | Benchmark comparison utility                           | —                     |

### Modified Files (tracked, uncommitted)

| File                                    | Phase | State                                                     |
| --------------------------------------- | ----- | --------------------------------------------------------- |
| `dart/dynamics/skeleton.hpp`            | 1     | ✅ Done: pool members, helpers                            |
| `dart/dynamics/skeleton.cpp`            | 1     | ✅ Done: pool ctor/dtor/clone/move                        |
| `dart/dynamics/detail/skeleton.hpp`     | 1     | ✅ Done: if-constexpr routing                             |
| `dart/constraint/constraint_solver.hpp` | 4     | ⚠️ Partial: unique_ptr member, needs→borrowed ptr         |
| `dart/constraint/constraint_solver.cpp` | 4     | 🔴 Broken: FrameStlAllocator ctor type mismatch           |
| `dart/simulation/world.hpp`             | 4     | ⚠️ Partial: correct types declared                        |
| `dart/simulation/world.cpp`             | 4     | 🔴 Broken: 5+ compile errors (see below)                  |
| `tests/benchmark/CMakeLists.txt`        | 1     | ✅ Done                                                   |
| `tests/unit/CMakeLists.txt`             | 1     | ⚠️ Partial: frame_allocator registered, pool test missing |
| `scripts/run_cpp_benchmark.py`          | 1     | ✅ Done                                                   |
| `.gitignore`                            | 1     | ✅ Done                                                   |

### Files NOT YET Modified (need Phase 4)

| File                             | Required Change                                              |
| -------------------------------- | ------------------------------------------------------------ |
| `dart/common/memory_manager.hpp` | Add `Frame` to Type enum, add FrameAllocator member + getter |
| `dart/common/memory_manager.cpp` | Construct FrameAllocator in ctor, implement getter           |

## Compile-Breaking Bugs (Current State)

### world.cpp (5 errors)

1. `config.memoryManager` → field doesn't exist, should derive from `config.baseAllocator`
2. `mMemoryManager.get()` → value member, no `.get()` method
3. `if (mMemoryManager)` → value member, no `operator bool`
4. `mMemoryManager->getBaseAllocator()` → should use `.` not `->`
5. `config.memoryManager = mMemoryManager` in clone() → wrong field name

### constraint_solver.cpp (2 errors)

1. `FrameStlAllocator<T>(mFrameAllocator)` at 6+ sites → passes `unique_ptr`, needs `FrameAllocator&`
2. `mFrameAllocator.reset()` → destroys the unique_ptr (should call `mFrameAllocator->reset()` for arena reset)

### Test registration (1 gap)

1. `test_body_node_pool.cpp` exists but not in any CMakeLists.txt

## Remaining Work (Ordered)

### Step 1: Add FrameAllocator to MemoryManager

- Add `Type::Frame` to enum in `memory_manager.hpp`
- Add `unique_ptr<FrameAllocator>` + debug variant members
- Add `getFrameAllocator()` getter
- Construct in `MemoryManager::MemoryManager()` with debug wrapping
- Add `allocateUsingFrame` / `deallocateUsingFrame` routing

### Step 2: Fix world.cpp to match world.hpp

- Constructor: `mMemoryManager(config.baseAllocator ? *config.baseAllocator : MemoryAllocator::GetDefault())`
- `getMemoryManager()`: return `mMemoryManager;` (ref, not pointer)
- Propagation: remove `if (mMemoryManager)` guards (always valid)
- `addSkeleton()`: pass `&mMemoryManager.getBaseAllocator()` to skeleton
- `setConstraintSolver()`: pass `&mMemoryManager.getFrameAllocator()` to solver
- `clone()`: `config.baseAllocator = &mMemoryManager.getBaseAllocator();`

### Step 3: Fix ConstraintSolver to borrow FrameAllocator

- Change `unique_ptr<FrameAllocator> mFrameAllocator` → `FrameAllocator* mFrameAllocator = nullptr`
- Remove FrameAllocator construction from ConstraintSolver ctor
- Add `setFrameAllocator(FrameAllocator*)` (replaces `setBaseAllocator`)
- Fix all 6 `FrameStlAllocator` sites: `FrameStlAllocator<T>(*mFrameAllocator)`
- Fix `mFrameAllocator.reset()` → `mFrameAllocator->reset()` (arena reset, not ptr reset)
- Add null-safety: standalone ConstraintSolver (no World) skips arena allocation

### Step 4: Wire Skeleton to borrow base allocator from World

- Add `MemoryAllocator* mBaseAllocator = nullptr` to Skeleton
- Add `setBaseAllocator(MemoryAllocator*)` method
- Make pool construction lazy: only when first BodyNode is created
- Pool ctor passes `*mBaseAllocator` (or `GetDefault()` if null)
- World::addSkeleton() calls `skeleton->setBaseAllocator(&memMgr.getBaseAllocator())`

### Step 5: Register missing test + build verification

- Add `test_body_node_pool.cpp` to `tests/unit/CMakeLists.txt`
- `pixi run build` — must compile cleanly
- `pixi run test-unit` — all 248+ tests must pass
- `pixi run lint` — format check

### Step 6: Update design doc + benchmarks

- Update `docs/design/hierarchical_allocator.md` to match final architecture
- Run benchmarks to verify no regressions from Phase 1-3 wins

## Open Design Questions

1. **FrameAllocator debug wrapping**: Follow existing pattern (debug variant in non-release)?
   - Recommendation: YES, match FreeList/Pool pattern
2. **Standalone ConstraintSolver fallback**: Create own FrameAllocator or skip arena?
   - Recommendation: Create default FrameAllocator lazily if null when solve() called
3. **Skeleton pool lazy vs eager**: Delay pool creation until first BodyNode?
   - Recommendation: Eager with default allocator, swap allocator on setBaseAllocator
4. **Pre-existing allocator bugs**: Fix in this PR or defer?
   - Recommendation: DEFER — separate PR for FreeList/Pool alignment/overflow bugs

## Benchmark Baselines (from Phases 1-3)

- Lifecycle: -30-41% improvement (PRESERVE)
- Contact WorldStep: -13% improvement (PRESERVE)
- ParallelWorlds: -25-36% improvement (PRESERVE)
- Small-skeleton FK: +13-54% regression (FIX via hierarchical refactor)
- Small WorldStep: +65% regression (FIX via hierarchical refactor)

Regressions were caused by inline allocators bloating sizeof(Skeleton/ConstraintSolver).
Hierarchical design moves allocators OUT of components → should fix regressions.
