# Resume: LCP Epic

> **For AI Agents**: Read this entire file before taking any action.

## Quick Status

- **Branch**: `refactor/lcp_plan`
- **PR**: #2464
- **State**: All committed locally. 3 unpushed commits. Builds clean, tests pass.
- **Next Action**: Push commits, then visual verification at `--gui-scale 2`

---

## Last Session (2026-02-03)

Cleaned up corrupted headless rendering artifacts and committed outstanding work:

1. **Deleted**: Non-ASCII corrupted file + leftover `headless_frames_run*/` and `rigid_frames/` dirs
2. **Committed**: `.gitignore` patterns for `headless_frames*/`, `rigid_frames/`, `*_frames/`
3. **Committed**: Headless capture fix — replaced `viewer.record()` with manual `captureBuffer()` + `osgDB::writeImageFile()` to eliminate corrupted filenames. Added `ImGuiViewer(ViewerConfig)` constructor and texture readiness check.
4. **Committed**: Dev task doc updates

## Recent Commits (3 Unpushed)

```
b774e965e32 docs: update lcp_epic dev task docs with headless capture changes
8552f94b0b7 fix(gui): use manual captureBuffer for headless frame capture
47bf2f1b938 chore: add gitignore patterns for headless frame capture artifacts
3382e369c7a checkpoint(examples): update ImGui debug readout and docs
2bedcd11bb8 checkpoint(examples): add ImGui debug readout for lcp_physics
```

---

## What Was Built

### 1. ImGui Font Scaling Fix (Framework-Level)

Added `ImGuiHandler::setFontScale()` that rebuilds the font atlas at the target
pixel size instead of using `FontGlobalScale` (which causes bitmap blur).

**Key files**:

- `dart/gui/ImGuiHandler.hpp` - `setFontScale()` declaration
- `dart/gui/ImGuiHandler.cpp` - Implementation with framebuffer scale handling

### 2. WorldNode Scenario Switching Fix

Fixed white screen when switching scenarios by clearing all ShapeFrameNodes
before replacing the world (prevents dangling Frame pointers).

**Key files**:

- `dart/gui/WorldNode.hpp` - `clearAllShapeFrameNodes()` declaration
- `dart/gui/WorldNode.cpp` - Implementation

### 3. lcp_physics Example Enhancements

- Full ImGui control panel (play/pause/step/reset, scenario switching, solver selection)
- 5 physics scenarios: mass_ratio, box_stack, ball_drop, dominos, inclined_plane
- Performance monitoring (FPS, step time graph, contact count)
- Headless mode with `--headless --frames N --out DIR`
- ImGui Debug section for DPI diagnostics
- UI scales with font size at high DPI

### 4. Applied to All 11 ImGui Examples

All examples call `setFontScale()` after `setImGuiScale()`:

- atlas_simbicon, box_stacking, coupler_constraint, fetch, free_joint_cases
- heightmap, imgui, lcp_physics, mimic_pendulums, point_cloud, tinkertoy

---

## Visual Verification Needed

```bash
cd /home/js/dev/dartsim/dart/task_4
./build/default/cpp/Release/bin/lcp_physics --gui-scale 2
```

**Check**:

1. Is the ImGui text crisp (not blurry)?
2. Does switching scenarios work (not white screen)?
3. Does the UI layout scale properly?

**If issues persist**, open the "ImGui Debug" section in lcp_physics to see:

- DisplaySize, DisplayFramebufferScale, FontSize, FontGlobalScale, UiScale

---

## Commands Reference

```bash
# Build
pixi run build

# Run lcp_physics
./build/default/cpp/Release/bin/lcp_physics                    # Default
./build/default/cpp/Release/bin/lcp_physics --gui-scale 2      # High DPI
./build/default/cpp/Release/bin/lcp_physics --list             # List scenarios
./build/default/cpp/Release/bin/lcp_physics --headless --scenario mass --frames 1 --out ./frames --gui-scale 2

# Run tests
pixi run test-all
```

---

## Technical Context

### Font Scaling Architecture

```cpp
// User calls:
viewer->setImGuiScale(guiScale);                    // Widget sizes
viewer->getImGuiHandler()->setFontScale(guiScale);  // Font at target size

// In ImGuiHandler::newFrame():
if (mUseFontScale && (mFontScaleDirty || scaleChanged)) {
    rebuildDefaultFont(mFontScale, framebufferScale);
}

// Font loaded at: 13.0f * fontScale * framebufferScale pixels
// FontGlobalScale set to: 1.0f / framebufferScale
```

### WorldNode Scenario Switching

```cpp
void WorldNode::setWorld(std::shared_ptr<World> newWorld) {
  clearAllShapeFrameNodes();  // Remove old nodes first
  mWorld = newWorld;
}
```

### UI Scaling in lcp_physics

```cpp
const float uiScale = ImGui::GetFontSize() / 13.0f;
ImGui::SetNextWindowSize(ImVec2(340.0f * uiScale, 600.0f * uiScale), ...);
```

---

## How to Fully Resume

```bash
cd /home/js/dev/dartsim/dart/task_4
git checkout refactor/lcp_plan
git status  # Should show "ahead by 3 commits"
git log --oneline -5

# Push if not yet pushed
git push

# Build if needed
pixi run build

# Visual test
./build/default/cpp/Release/bin/lcp_physics --gui-scale 2

# If all works, this phase is complete
# If issues found, debug using ImGui Debug section
```

---

## Phase 4 Future Work (Not Started)

- Additional LCP solvers (APGD, TGS, ADMM)
- Runtime solver switching
- Benchmark infrastructure
- Documentation expansion

---

## Key Files Reference

| File                            | Purpose                        |
| ------------------------------- | ------------------------------ |
| `dart/gui/ImGuiHandler.*`       | Font scaling implementation    |
| `dart/gui/WorldNode.*`          | Scenario switching fix         |
| `examples/lcp_physics/main.cpp` | Main example with all features |
| `docs/dev_tasks/lcp_epic/`      | This documentation             |

---

**Last Updated**: 2026-02-03
