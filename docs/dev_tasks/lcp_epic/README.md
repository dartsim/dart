# LCP Epic: Dynamics-Agnostic LCP API + Solvers

> **AI Agents**: Start with [RESUME.md](RESUME.md) for current session state and next steps.

## Status Overview

| Phase   | Description                           | Status                 |
| ------- | ------------------------------------- | ---------------------- |
| Phase 1 | Solver Selection API + Demo Stability | ✅ Complete (PR #2464) |
| Phase 2 | Test Infrastructure + Problem Factory | ✅ Complete            |
| Phase 3 | Python Bindings                       | ✅ Complete            |
| Phase 4 | Future Solvers, Benchmarks, Docs      | 📋 Not Started         |

**Current Work**: Fixing ImGui blurriness at high DPI (see RESUME.md)

---

## What Was Built

### Examples

| Example       | Status      | Description                                  |
| ------------- | ----------- | -------------------------------------------- |
| `lcp_physics` | ✅ Complete | 5 physics scenarios with ImGui control panel |
| `lcp_solvers` | ❌ Removed  | Moved to `tests/benchmark/lcpsolver/`        |

### lcp_physics Features

- **5 Scenarios**: mass_ratio, box_stack, ball_drop, dominos, inclined_plane
- **ImGui Widget**: Play/pause/step/reset, scenario switching, solver selection
- **Parameter Editing**: Timestep, gravity
- **Performance Monitoring**: FPS, step time graph, contact count
- **Headless Mode**: `--headless --frames N --out DIR`
- **Solvers**: Dantzig (pivoting), PGS (iterative)

### Test Infrastructure

| File                                           | Purpose                            |
| ---------------------------------------------- | ---------------------------------- |
| `tests/common/lcpsolver/LcpProblemFactory.hpp` | Unified LCP problem generation     |
| `tests/unit/math/lcp/test_AllSolversSmoke.cpp` | Smoke tests for all 19 solvers     |
| `tests/benchmark/lcpsolver/`                   | Google Benchmark performance tests |

### Python Bindings

- `LcpSolverType` enum
- `CollisionDetectorType` enum
- `WorldConfig` with solver selection
- `World.create(config)` factory

---

## Active Bug: ImGui Blurriness

**Problem**: Text is blurry at `--gui-scale > 1`

**Cause**: `FontGlobalScale` scales bitmap fonts at runtime

**Fix Status**: `setFontScale()` rebuilds fonts; `applyImGuiScale()` skips
`FontGlobalScale` after `setFontScale()`. Added viewport/traits fallback to
keep framebuffer scale >= 1 and scaled the lcp_physics widget layout to the
current ImGui font size. Added an ImGui debug section in lcp_physics to report
DisplaySize/FramebufferScale/FontGlobalScale for diagnosis (needs visual test)

**Details**: See [RESUME.md](RESUME.md) for full context and next steps

**Scope Unknown**: May affect all ImGui examples, not just lcp_physics

---

## Commands Reference

```bash
# Build
pixi run build

# Run lcp_physics
./build/default/cpp/Release/bin/lcp_physics                    # Default
./build/default/cpp/Release/bin/lcp_physics --gui-scale 2      # High DPI
./build/default/cpp/Release/bin/lcp_physics --list             # List scenarios
./build/default/cpp/Release/bin/lcp_physics --headless --scenario mass --frames 100

# Run tests
pixi run test-all

# Clean corrupted frame files
find . -maxdepth 1 -type f -name $'*[\x80-\xff]*' -delete
```

---

## Phase Details

### Phase 1: Foundation ✅

- `LcpSolverType` enum (Dantzig/Pgs/Lemke)
- `WorldConfig` solver selection
- Demo stability fixes
- Unit test: `tests/unit/simulation/test_WorldConfig.cpp`

### Phase 2: Test Infrastructure ✅

- `LcpProblemFactory` with edge cases, well-conditioned, ill-conditioned problems
- Smoke tests for all 19 LCP solvers
- All tests pass (143+)

### Phase 3: Python Bindings ✅

- Enums and WorldConfig bound
- `World.create(config)` works
- Python tests pass

### Phase 4: Future Work 📋

| Priority | Item                                  |
| -------- | ------------------------------------- |
| P1       | APGD solver (10-100x faster than PGS) |
| P1       | TGS solver (GPU real-time)            |
| P2       | Runtime solver switching              |
| P2       | Benchmark infrastructure              |
| P3       | Documentation expansion               |

---

## Key Files

```
dart/math/lcp/                           # LCP solver implementations
dart/gui/ImGuiHandler.*                  # ImGui integration (has uncommitted changes)
examples/lcp_physics/                    # Main example
tests/common/lcpsolver/LcpProblemFactory.hpp
tests/unit/math/lcp/test_AllSolversSmoke.cpp
tests/benchmark/lcpsolver/               # Performance benchmarks
```

---

## Git Info

- **Branch**: `refactor/lcp_plan`
- **PR**: #2464
- **Uncommitted**: lcp_physics widget layout scaling (needs visual test)

---

**Last Updated**: 2026-01-19
