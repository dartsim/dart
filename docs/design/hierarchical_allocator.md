# Hierarchical Allocator Design: World-Level Memory Management

## Status: PROPOSAL (not yet implemented)

## 1. Current State

### Existing Infrastructure

- `MemoryAllocator` base class (`dart/common/memory_allocator.hpp`): `allocate(bytes)`, `deallocate(ptr, bytes)` -- virtual interface for all allocators
- `MemoryManager` (`dart/common/memory_manager.hpp`): composite containing FreeListAllocator + PoolAllocator, accepts a base `MemoryAllocator&`
- `PoolAllocator`: fast fixed-size pool (max 1024 bytes per object)
- `FreeListAllocator`: arbitrary size allocations with coalescing

Both exist but are **unused by dynamics or constraint modules**.

### Recent Additions (Cache-Friendliness Optimizations)

- `BodyNodePool<T>` (`dart/dynamics/detail/body_node_pool.hpp`): per-Skeleton contiguous pool for BodyNode allocation, 32-byte aligned slots
- `FrameAllocator` (`dart/common/frame_allocator.hpp`): per-step bump allocator for constraint temporaries, supports aligned allocation

Both are **standalone** -- they bypass MemoryManager and manage their own memory.

## 2. Problem Statement

Each component manages memory independently:

- Skeleton creates its own BodyNodePool in the constructor
- ConstraintSolver creates its own FrameAllocator as a member
- No coordination or configuration from above

This makes it impossible to:

1. Configure memory strategy at the World level (e.g., pre-allocate for known workload)
2. Share memory budgets across components
3. Use custom allocators (e.g., arena per-thread for parallel simulation)
4. Monitor total memory usage from a single point

## 3. Proposed Architecture

### Ownership Hierarchy

```
World
+-- MemoryManager (owned, configurable)
|   +-- base: MemoryAllocator& (user-replaceable)
|   +-- pool: PoolAllocator (for small objects)
|   +-- free: FreeListAllocator (for large objects)
|
+-- Skeleton 1 (receives MemoryManager* on addSkeleton)
|   +-- BodyNodePool (allocates from MemoryManager's pool)
|
+-- Skeleton 2
|   +-- BodyNodePool (same MemoryManager)
|
+-- ConstraintSolver (receives MemoryManager* on construction)
    +-- FrameAllocator (allocates initial buffer from MemoryManager)
```

### API Sketch

```cpp
// World gains optional allocator configuration
class World {
public:
  void setMemoryManager(std::shared_ptr<MemoryManager> manager);
  MemoryManager& getMemoryManager();

private:
  std::shared_ptr<MemoryManager> mMemoryManager;
};

// Skeleton receives allocator at construction time
class Skeleton {
public:
  static SkeletonPtr create(
      const std::string& name,
      MemoryManager* allocator = nullptr);  // nullptr = use default

protected:
  MemoryManager* mMemoryManager;  // non-owning, from World
};

// ConstraintSolver receives allocator configuration
class ConstraintSolver {
public:
  void setMemoryManager(MemoryManager* manager);

private:
  MemoryManager* mMemoryManager;  // non-owning, from World
};
```

### User-Level Usage

```cpp
// Default: zero configuration needed (backward compatible)
auto world = World::create();
world->addSkeleton(robot);  // uses default MemoryManager

// Advanced: custom allocator for the entire world
auto customAllocator = std::make_shared<MemoryManager>(myBaseAllocator);
world->setMemoryManager(customAllocator);
world->addSkeleton(robot);  // robot's pool uses customAllocator
```

## 4. Alignment Gap in MemoryAllocator

### Current Issue

```cpp
// MemoryAllocator base class -- NO alignment parameter
virtual void* allocate(size_t bytes) noexcept = 0;

// FrameAllocator -- HAS alignment parameter
void* allocate(size_t bytes, size_t alignment = 32);

// BodyNodePool -- uses 32-byte aligned slots internally
```

The base interface cannot express alignment requirements, preventing FrameAllocator from deriving from MemoryAllocator.

### Proposed Fix

```cpp
class MemoryAllocator {
public:
  virtual void* allocate(size_t bytes) noexcept = 0;           // existing
  virtual void* allocate(size_t bytes, size_t alignment) noexcept;  // new
};
```

Default: delegates to `allocate(bytes)` -- backward compatible for existing subclasses.

## 5. Migration Path

| Phase | Description                                                         | Status   |
| ----- | ------------------------------------------------------------------- | -------- |
| 1     | BodyNodePool and FrameAllocator standalone                          | **Done** |
| 2     | World owns MemoryManager, passes to Skeleton and ConstraintSolver   | Planned  |
| 3     | MemoryAllocator gains alignment; FrameAllocator derives from it     | Planned  |
| 4     | Per-World memory budgets, thread-local allocators for parallel step | Future   |

## 6. Benefits

| Benefit                    | Description                                                 |
| -------------------------- | ----------------------------------------------------------- |
| Single configuration point | One `setMemoryManager()` call controls all allocation       |
| Memory budgeting           | World-level tracking of total allocation                    |
| Testability                | Inject mock allocators for deterministic testing            |
| Thread safety              | Per-World allocators enable independent parallel simulation |
| Custom placement           | Users can control memory locality (e.g., NUMA-aware)        |

## 7. Non-Goals

- GPU memory management (out of scope for CPU dynamics)
- Custom allocators for Eigen internal temporaries
- Replacing system malloc for non-DART allocations
- Lock-free allocators (ConstraintSolver is single-threaded per-World)
