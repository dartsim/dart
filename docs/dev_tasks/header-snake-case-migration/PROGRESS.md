# Progress Log: Header Snake_Case Migration

## Final Status: IN REVIEW

All structural renames and compatibility fixes completed. PR awaiting CI and review.

- **PR**: https://github.com/dartsim/dart/pull/2475
- **Branch**: `feature/header-snake-case-migration`
- **Files Renamed**: ~325 header files
- **Tests**: All 143 tests passing locally
- **Lint**: Passing
- **Commits**: 7 (includes compatibility fixes)

### Compatibility Fixes Applied

1. `examples/lcp_solvers/main.cpp` - Updated `lcp/All.hpp` to `lcp/all.hpp`
2. `dart/utils/urdf/UrdfParser.cpp` - Updated `BackwardCompatibility.hpp` to `backward_compatibility.hpp`
3. `dart/utils/urdf/urdf_world_parser.hpp` - Same fix
4. Created `dart/All.hpp` - PascalCase wrapper for `dart/all.hpp`
5. Created `dart/Export.hpp` - PascalCase wrapper for `dart/export.hpp`
6. Updated `dart/gui/CMakeLists.txt` - Install generated `All.hpp` for examples/tutorials
7. Updated documentation references in `docs/background/lcp/02_overview.md`

### Cleanup Note

Per AGENTS.md, remove `docs/dev_tasks/header-snake-case-migration/` folder when PR is merged.

---

## Session 1

### Phase 1: DART 7 Additions (COMPLETED)

- Identified 21 DART 7 header files safe for immediate case-only rename
- Files: All.hpp, Export.hpp, Fwd.hpp across modules
- Method: `git diff --name-status origin/release-6.16..HEAD` to identify new files
- Committed: `22eeb6d3075`
- PR Created: https://github.com/dartsim/dart/pull/2475

### Phase 2a: dart/common (COMPLETED)

- Renamed 40 multi-word PascalCase headers to snake_case
- Updated all internal includes
- Committed: `665a099f082`

### Phase 2b-h: Remaining Modules (IN PROGRESS)

- dart/math: 32 files renamed
- dart/collision: 46 files renamed
- dart/constraint: 23 files renamed
- dart/dynamics: 96 files renamed
- dart/gui: 38 files renamed
- dart/utils: 24 files renamed
- dart/sensor: 1 file renamed
- dart/simulation: 1 file renamed
- dart/optimizer: 1 file renamed
- dart/lcpsolver: 2 files renamed

Total: ~264 files renamed (uncommitted)

### Include Updates Applied

1. Full path includes via sed script
2. Relative includes via sed script
3. CMakeLists.txt via Python script (to avoid partial match issues)

### Build Status

- CMake configuration: PASSED
- Build (383 targets): PASSED
- Tests: PENDING
- Lint: PENDING

### Issues Encountered & Resolved

1. **Relative includes not updated**: Fixed by adding patterns for quoted filenames
2. **CMakeLists.txt partial matches**: Fixed by using Python with proper regex word boundaries
3. **Overlapping sed patterns**: Sorted by length (longest first) to prevent partial matches

### Files Modified

- ~500+ source files with include updates
- ~20 CMakeLists.txt files with header list updates
- Created `docs/onboarding/header-migration-analysis.md`

## Remaining Work

### Immediate (This PR)

- [x] Run tests: `pixi run test` - PASSED
- [x] Run lint: `pixi run lint` - PASSED
- [x] Commit Phase 2b-h changes - DONE
- [x] Push and update PR description - DONE
- [x] Address Codex review feedback - DONE
- [ ] CI passing on all platforms - PENDING

### Future (DART 8)

- Case-only renames deferred to DART 8:
  - World.hpp → world.hpp
  - Frame.hpp → frame.hpp
  - Joint.hpp → joint.hpp
  - etc.
- These require CMake compat wrapper infrastructure or DART 8 breaking changes
