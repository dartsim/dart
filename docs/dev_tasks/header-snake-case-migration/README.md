# Header Snake_Case Migration Task

## Overview

Migrate DART header files from PascalCase to snake_case naming convention.

**Branch**: `feature/header-snake-case-migration`
**PR**: https://github.com/dartsim/dart/pull/2475

## Current Status

**Phase**: IN REVIEW - Awaiting CI

| Phase      | Description                                     | Status    |
| ---------- | ----------------------------------------------- | --------- |
| Phase 1    | DART 7 additions (All.hpp, Export.hpp, Fwd.hpp) | COMMITTED |
| Phase 2a   | dart/common structural renames                  | COMMITTED |
| Phase 2b-h | All remaining modules                           | COMMITTED |
| Phase 3    | Verification, lint, tests                       | PASSED    |
| Phase 4    | Compatibility fixes (Codex review feedback)     | COMMITTED |

### Summary

- **~325 header files renamed** (structural changes only)
- **All tests passing** (143/143)
- **Lint passing**
- **PR updated** with comprehensive description

### Cleanup

Per AGENTS.md, delete this folder (`docs/dev_tasks/header-snake-case-migration/`) when PR is merged.

## Key Files

- `docs/onboarding/header-migration-analysis.md` - Full migration strategy
- `/tmp/rename_headers.py` - Script for renaming headers
- `/tmp/update_cmake.py` - Script for updating CMakeLists.txt

## Resume Instructions

See [RESUME.md](RESUME.md) for detailed continuation instructions.

## Important Distinctions

### Safe to Rename Now (Structural)

Multi-word PascalCase headers where the snake_case version is structurally different:

- `BodyNode.hpp` → `body_node.hpp` (SAFE)
- `CollisionDetector.hpp` → `collision_detector.hpp` (SAFE)

### Defer to DART 8 (Case-Only)

Single-word PascalCase headers that collide on case-insensitive filesystems:

- `World.hpp` → `world.hpp` (DEFER - collision risk)
- `Frame.hpp` → `frame.hpp` (DEFER - collision risk)

## Validation Commands

```bash
pixi run build          # Must pass
pixi run test           # Must pass
pixi run lint           # Must pass
pixi run -e gazebo test-gz  # Gazebo compatibility
```
