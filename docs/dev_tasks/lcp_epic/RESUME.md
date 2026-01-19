# Resume: LCP Epic

> **For AI Agents**: Read this entire file before taking any action.

## Quick Status

- **Branch**: `refactor/lcp_plan`
- **PR**: #2464
- **State**: Ready to commit - all changes complete, needs visual verification
- **Blocker**: Human visual test needed to confirm fix works

---

## What Was Done (This Session)

### 1. ImGuiHandler Font Scaling (Framework-Level Fix)

Added `setFontScale(float)` to `dart/gui/ImGuiHandler`:

- Rebuilds font atlas at `fontScale * framebufferScale` pixels
- Handles runtime DPI changes (e.g., moving window between monitors)
- Sets `FontGlobalScale = 1/framebufferScale` to avoid double-scaling

**Files changed**:

- `dart/gui/ImGuiHandler.hpp` - Added method declaration and member variables
- `dart/gui/ImGuiHandler.cpp` - Added implementation

### 2. Applied Fix to All 11 ImGui Examples

Every example now calls `setFontScale()` after `setImGuiScale()`:

| Example            | File                                  |
| ------------------ | ------------------------------------- |
| atlas_simbicon     | `examples/atlas_simbicon/main.cpp`    |
| box_stacking       | `examples/box_stacking/main.cpp`      |
| coupler_constraint | `examples/coupler_constraint/main.cpp`|
| fetch              | `examples/fetch/main.cpp`             |
| free_joint_cases   | `examples/free_joint_cases/main.cpp`  |
| heightmap          | `examples/heightmap/main.cpp`         |
| imgui              | `examples/imgui/main.cpp`             |
| lcp_physics        | `examples/lcp_physics/main.cpp`       |
| mimic_pendulums    | `examples/mimic_pendulums/main.cpp`   |
| point_cloud        | `examples/point_cloud/main.cpp`       |
| tinkertoy          | `examples/tinkertoy/main.cpp`         |

### 3. Build Verified

All examples compile successfully with `pixi run build`.

---

## Files Ready to Commit

```
M  .gitignore                           # Corrupted frame file documentation
M  dart/gui/ImGuiHandler.cpp            # setFontScale() implementation
M  dart/gui/ImGuiHandler.hpp            # setFontScale() declaration
M  docs/dev_tasks/lcp_epic/README.md    # Updated status
A  docs/dev_tasks/lcp_epic/RESUME.md    # This file
M  examples/atlas_simbicon/main.cpp     # Added setFontScale()
M  examples/box_stacking/main.cpp       # Added setFontScale()
M  examples/coupler_constraint/main.cpp # Added setFontScale()
M  examples/fetch/main.cpp              # Added setFontScale()
M  examples/free_joint_cases/main.cpp   # Added setFontScale()
M  examples/heightmap/main.cpp          # Added setFontScale()
M  examples/imgui/main.cpp              # Added setFontScale()
M  examples/lcp_physics/main.cpp        # Uses setFontScale() instead of local helper
M  examples/mimic_pendulums/main.cpp    # Added setFontScale()
M  examples/point_cloud/main.cpp        # Added setFontScale()
M  examples/tinkertoy/main.cpp          # Added setFontScale()
```

---

## Next Steps

### Step 1: Visual Test (Requires Human)

```bash
cd /home/js/dev/dartsim/dart/task_4

# Test any example at high DPI
./build/default/cpp/Release/bin/lcp_physics --gui-scale 2
./build/default/cpp/Release/bin/imgui --gui-scale 2
./build/default/cpp/Release/bin/box_stacking --gui-scale 2
```

**Check**: Is the ImGui text crisp or blurry at scale 2?

### Step 2: Based on Visual Test Result

**If text is CRISP** (fix works):

```bash
git add -A
git commit -m "fix(gui): add setFontScale() for crisp ImGui text at high DPI

Add ImGuiHandler::setFontScale() that rebuilds the font atlas at the
target size instead of using FontGlobalScale which causes blur.

- Load font at fontScale * framebufferScale pixels with 2x oversampling
- Handle runtime DPI changes in newFrame()
- Apply fix to all 11 ImGui examples

This fixes blurry ImGui widgets when using --gui-scale > 1."

git push origin refactor/lcp_plan
```

**If text is still BLURRY** (needs debugging):

1. Check that `mFramebufferScale` is being detected correctly
2. Verify the font atlas is actually being rebuilt (add logging)
3. Test with explicit pixel sizes

---

## Technical Details

### How setFontScale() Works

```cpp
// In setFontScale():
mFontScale = scale;
mUseFontScale = true;
mFontScaleDirty = true;

// In newFrame():
if (mUseFontScale && (mFontScaleDirty || scaleChanged)) {
    rebuildDefaultFont(mFontScale, framebufferScale);
    mFontScaleDirty = false;
}

// In rebuildDefaultFont():
config.SizePixels = 13.0f * fontScale * framebufferScale;
io.FontGlobalScale = 1.0f / framebufferScale;
```

### Why FontGlobalScale Alone Causes Blur

ImGui's default font is a 13px bitmap. When you set `FontGlobalScale = 2.0`:

1. Renders the 13px bitmap
2. Scales it up 2x at display time
3. Result: 26px but blurry (upscaled bitmap)

With `setFontScale()`:

1. Loads a 26px font at initialization
2. Renders natively at 26px
3. Result: 26px and crisp (native resolution)

---

## Clean Up Corrupted Frame Files

If you see files with non-ASCII names in git status:

```bash
find . -maxdepth 1 -type f -name $'*[\x80-\xff]*' -delete
```

---

## How to Fully Resume

```bash
cd /home/js/dev/dartsim/dart/task_4
git checkout refactor/lcp_plan
git status  # Should show all files above

# Build (may already be built)
pixi run build

# Visual test
./build/default/cpp/Release/bin/lcp_physics --gui-scale 2

# If crisp: commit and push (see commands above)
```

---

## Previous Session Work (Already Committed)

| Commit        | Description                                         |
| ------------- | --------------------------------------------------- |
| `4639cd41e5b` | Load fonts at scaled size (per-example, superseded) |
| `6ee024c0f6c` | Fix GUI scaling and slip compliance warnings        |
| `bc4e6e7a8e7` | Add ImGui widget to lcp_physics                     |
| `c9c630b0464` | Remove lcp_solvers example (moved to benchmarks)    |

---

## Phase 4 Future Work (Not Started)

- Additional LCP solvers (APGD, TGS, ADMM)
- Runtime solver switching
- Benchmark infrastructure
- Documentation expansion

---

**Last Updated**: 2026-01-19 13:50 PST
