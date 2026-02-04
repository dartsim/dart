# dart/common/

Agent guidelines for the common module.

## Overview

Foundation utilities shared across all DART modules: memory management, design patterns, and helpers.

## Memory Allocators

Hierarchical allocation system for cache-friendly simulation:

- **MemoryManager** — Owns all allocators; World creates one per instance
- **FreeListAllocator** — General-purpose allocation with coalescing
- **PoolAllocator** — Fixed-size block allocation (built on FreeListAllocator)
- **FrameAllocator** — Bump/arena allocator for per-step scratch memory (reset each step)
- **CAllocator** — Thin wrapper around `malloc`/`free` (default fallback)

**Key design decision**: Components _borrow_ allocators from World's MemoryManager rather than owning them. This keeps `sizeof(Skeleton)` and `sizeof(ConstraintSolver)` small and avoids per-component allocator overhead.

FrameAllocator maintains a 32-byte alignment invariant: `allocate()` assumes `mCur` is always 32-byte aligned. `allocateAligned()` rounds up after each call to preserve this.

## Design Patterns

- **Aspect System** — Runtime composition (`aspect.hpp`, `composite.hpp`)
- **Factory** — Type-string object creation (`factory.hpp`)
- **Observer/Signal** — Event notification (`subject.hpp`, `signal.hpp`)
- **STL Allocator Adapter** — `stl_allocator.hpp` wraps MemoryAllocator for STL containers

## Testing

Unit tests: `tests/unit/common/`
Benchmarks: `tests/benchmark/dynamics/bm_dynamics_cache.cpp` (allocator-intensive)

## See Also

- @docs/onboarding/architecture.md — Memory management in architecture overview
- @docs/design/hierarchical_allocator.md — Original design document
