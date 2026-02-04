# LCP Epic: Dynamics-Agnostic LCP API + Solvers

> **AI Agents**: Start with [RESUME.md](RESUME.md) for current session state and next steps.

## Status Overview

| Phase   | Description                           | Status                 |
| ------- | ------------------------------------- | ---------------------- |
| Phase 1 | Solver Selection API + Demo Stability | ✅ Complete (PR #2464) |
| Phase 2 | Test Infrastructure + Problem Factory | ✅ Complete            |
| Phase 3 | Python Bindings                       | ✅ Complete            |
| Phase 4 | Future Solvers, Benchmarks, Docs      | 📋 Not Started         |

**Current State**: All code committed and pushed. Needs visual verification of high-DPI fixes.

---

## What Was Built

### GUI Fixes (dart/gui/)

| Fix                  | Commit        | Description                                              |
| -------------------- | ------------- | -------------------------------------------------------- |
| Font scaling         | `0472a322d68` | `setFontScale()` loads fonts at target size              |
| World switching      | `27881b54067` | Clear nodes before switching worlds (fixes white screen) |
| Framebuffer scale    | `b0bb5e95b6c` | Normalize scale when viewport/traits flipped             |
| FontGlobalScale skip | `26f7cc3725f` | Avoid double-scaling with rebuilt fonts                  |

### lcp_physics Example

| Feature       | Description                                               |
| ------------- | --------------------------------------------------------- |
| ImGui widget  | Full control panel with scenarios, solvers, parameters    |
| 5 scenarios   | mass_ratio, box_stack, ball_drop, dominos, inclined_plane |
| Headless mode | `--headless --frames N --out DIR`                         |
| Debug section | DPI diagnostics (DisplaySize, FontSize, etc.)             |
| UI scaling    | Layout scales with font size at high DPI                  |

### Test Infrastructure

| File                                           | Purpose                            |
| ---------------------------------------------- | ---------------------------------- |
| `tests/common/lcpsolver/LcpProblemFactory.hpp` | Unified LCP problem generation     |
| `tests/unit/math/lcp/test_AllSolversSmoke.cpp` | Smoke tests for all 19 solvers     |
| `tests/benchmark/lcpsolver/`                   | Google Benchmark performance tests |

### Python Bindings

- `LcpSolverType` and `CollisionDetectorType` enums
- `WorldConfig` with solver selection
- `World.create(config)` factory

---

## Commands Reference

```bash
# Build
pixi run build

# Run lcp_physics
./build/default/cpp/Release/bin/lcp_physics                    # Default
./build/default/cpp/Release/bin/lcp_physics --gui-scale 2      # High DPI test
./build/default/cpp/Release/bin/lcp_physics --list             # List scenarios
./build/default/cpp/Release/bin/lcp_physics --headless --scenario mass --frames 100

# Run tests
pixi run test-all

# Clean corrupted frame files
find . -maxdepth 1 -type f -name $'*[\x80-\xff]*' -delete
```

---

## Phase 4 Future Work

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
dart/gui/ImGuiHandler.*              # Font scaling implementation
dart/gui/WorldNode.*                 # Scenario switching fix
examples/lcp_physics/main.cpp        # Main example with all features
tests/common/lcpsolver/              # Test infrastructure
tests/benchmark/lcpsolver/           # Performance benchmarks
```

---

## Git Info

- **Branch**: `refactor/lcp_plan`
- **PR**: #2464
- **State**: All committed and pushed

---

**Last Updated**: 2026-01-19
