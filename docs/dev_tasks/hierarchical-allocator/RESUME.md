# Resume: Hierarchical Allocator Task

**Last session**: Audited all modified files, created comprehensive task tracking.

## Current State: CODE DOES NOT COMPILE

The working tree has partially-applied Phase 4 changes across 11 tracked files + 9 untracked files.
Phase 1 (pools) and Phase 3 (FrameAllocator) code is complete and correct.
Phase 4 (hierarchical wiring) is half-done with compile-breaking bugs.

## To Continue

Read `docs/dev_tasks/hierarchical-allocator/README.md` for the full task plan.

**Immediate next step**: Execute Steps 1-5 from README.md in order:

1. Add FrameAllocator to MemoryManager
2. Fix world.cpp compile errors
3. Fix ConstraintSolver to borrow FrameAllocator
4. Wire Skeleton to borrow base allocator
5. Register missing test + build verification

## Key Files to Read First

- `dart/common/memory_manager.hpp` + `.cpp` — needs FrameAllocator added
- `dart/simulation/world.hpp` + `.cpp` — hpp is correct, cpp has 5 bugs
- `dart/constraint/constraint_solver.hpp` + `.cpp` — needs ptr type change
- `dart/common/frame_allocator.hpp` — the new allocator (inherits MemoryAllocator)

## Build Commands

```bash
pixi run build        # Must pass after fixes
pixi run test-unit    # 248+ tests must pass
pixi run lint         # Format before commit
pixi run test-all     # Full validation
```
